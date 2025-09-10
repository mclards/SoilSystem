#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <ModbusMaster.h>
#include <EEPROM.h>
#include <OneButton.h>

/* ================== HiveMQ Cloud ================== */
const char* mqtt_server = "c33fda18bbcd4c9998277dfe166c7c4a.s1.eu.hivemq.cloud";
const int   mqtt_port = 8883; // TLS
const char* mqtt_user = "gwapooo";
const char* mqtt_pass = "_Mclards23";
const char* clientID = "SoilMon0002";

/* ================== Topics ================== */
const char* topicSprinkler = "Sprinkler";
const char* topicRuntime = "Runtime";
const char* topicSprinklerLong = "SprinklerLong";
const char* topicTemperature = "Temperature";
const char* topicLastwill = "Lastwill";
const char* topicJSON = "SoilSystem";
const char* topicThreshold = "Threshold";

/* ================== MQTT Objects ================== */
WiFiClientSecure espClient;
PubSubClient client(espClient);

/* ================== Global Variables ================== */
bool pumpRunning = false;
bool manualMode = false;
unsigned long pumpStartTime = 0;
int pumpRuntime = 20;  // default runtime (secs)
int threshold = 85;  // default threshold (%)

/* ================== Hardware Pins ================== */
int pumpPin = 2;
int buttonPin = 15;

/* ================== OneButton ================== */
OneButton oneBtn(buttonPin, true, true);

/* ================== RS485 Soil Sensor ================== */
#define RX2_PIN   16
#define TX2_PIN   17
#define RE_DE_PIN 4
ModbusMaster soilNode;

void preTransmission() { digitalWrite(RE_DE_PIN, HIGH); delayMicroseconds(10); }
void postTransmission() { digitalWrite(RE_DE_PIN, LOW);  delayMicroseconds(10); }

void initRS485() {
    Serial2.begin(4800, SERIAL_8N1, RX2_PIN, TX2_PIN);
    pinMode(RE_DE_PIN, OUTPUT);
    digitalWrite(RE_DE_PIN, LOW);
    soilNode.begin(1, Serial2);
    soilNode.preTransmission(preTransmission);
    soilNode.postTransmission(postTransmission);
}

bool readSoilSensor(float& humidity, float& temperature) {
    uint8_t result = soilNode.readHoldingRegisters(0x0000, 2);
    if (result == soilNode.ku8MBSuccess) {
        humidity = soilNode.getResponseBuffer(0) * 0.1;
        temperature = (int16_t)soilNode.getResponseBuffer(1) * 0.1;
        return true;
    }
    Serial.printf("RS485 Modbus error: 0x%02X\n", result);
    return false;
}

/* ================== EEPROM Addresses ================== */
#define EEPROM_SIZE     8
#define ADDR_RUNTIME    0   // 2 bytes
#define ADDR_THRESHOLD  2   // 2 bytes

/* ================== EEPROM Helpers ================== */
void loadSettings() {
    EEPROM.begin(EEPROM_SIZE);

    uint16_t r, t;
    EEPROM.get(ADDR_RUNTIME, r);
    EEPROM.get(ADDR_THRESHOLD, t);

    if (r == 0xFFFF || r == 0) r = 20;  // default
    if (t == 0xFFFF || t == 0) t = 85;  // default

    pumpRuntime = r;
    threshold = t;

    Serial.printf("Loaded settings: runtime=%d threshold=%d\n", pumpRuntime, threshold);
}

void saveRuntime(int v) {
    pumpRuntime = v;  // keep global in sync
    uint16_t r = (uint16_t)v;
    EEPROM.put(ADDR_RUNTIME, r);
    EEPROM.commit();
    Serial.printf("Saved runtime=%d\n", v);
}

void saveThreshold(int v) {
    threshold = v;  // keep global in sync
    uint16_t t = (uint16_t)v;
    EEPROM.put(ADDR_THRESHOLD, t);
    EEPROM.commit();
    Serial.printf("Saved threshold=%d\n", v);
}

/* ================== Queue for MQTT messages ================== */
struct MqttMessage {
    char topic[64];
    char payload[256];
};
QueueHandle_t mqttQueue;

/* ================== WiFi Setup ================== */
void setup_wifi() {
    WiFiManager wm;
    if (!wm.autoConnect("SoilSystem_Setup")) {
        Serial.println("Failed to connect, restarting...");
        ESP.restart();
    }
    Serial.println("WiFi connected: " + WiFi.SSID());
}

/* ================== MQTT Callback ================== */
void callback(char* topic, byte* message, unsigned int length) {
    String msg;
    for (unsigned int i = 0; i < length; i++) msg += (char)message[i];

    if (String(topic) == topicSprinkler) {
        if (msg == "1" && !manualMode) {
            digitalWrite(pumpPin, HIGH); pumpRunning = true; pumpStartTime = millis();
            Serial.println("Pump ON (Runtime Mode)");
        } else if (msg == "0" && pumpRunning && !manualMode) {
            digitalWrite(pumpPin, LOW); pumpRunning = false;
            Serial.println("Pump OFF (Runtime Mode via MQTT)");
        }
    } else if (String(topic) == topicRuntime) {
        int v = msg.toInt();
        if (v > 0) {
            pumpRuntime = v; saveRuntime(pumpRuntime);
            if (client.connected())
                client.publish(topicRuntime, String(pumpRuntime).c_str(), true);
        }
    } else if (String(topic) == topicSprinklerLong) {
        if (msg == "1") { manualMode = true; pumpRunning = true; digitalWrite(pumpPin, HIGH); } else { manualMode = false;pumpRunning = false;digitalWrite(pumpPin, LOW); }
    } else if (String(topic) == topicThreshold) {
        int t = msg.toInt();
        if (t >= 1 && t <= 100) {
            threshold = t; saveThreshold(threshold);
            if (client.connected())
                client.publish(topicThreshold, String(threshold).c_str(), true);
        }
    }
}

/* ================== Pump Runtime/Threshold Check ================== */
void checkPumpRuntime(float moistureValue) {
    if (pumpRunning && !manualMode) {
        unsigned long elapsed = (millis() - pumpStartTime) / 1000;
        if ((pumpRuntime > 0 && elapsed >= (unsigned long)pumpRuntime) || (moistureValue >= threshold)) {
            digitalWrite(pumpPin, LOW); pumpRunning = false;
            MqttMessage msg;
            snprintf(msg.topic, sizeof(msg.topic), "%s", topicSprinkler);
            snprintf(msg.payload, sizeof(msg.payload), "0");
            xQueueSend(mqttQueue, &msg, portMAX_DELAY);
            Serial.println("Pump OFF (Runtime done / Threshold reached)");
        }
    }
}

/* ================== MQTT Reconnect ================== */
bool reconnectOnce() {
    if (client.connected()) return true;
    Serial.println("MQTT: attempting connect...");
    bool ok = client.connect(clientID, mqtt_user, mqtt_pass, topicLastwill, 0, true, "System Disconnected");
    if (ok) {
        client.publish(topicLastwill, "System Online", true);
        client.subscribe(topicSprinkler);
        client.subscribe(topicRuntime);
        client.subscribe(topicSprinklerLong);
        client.subscribe(topicThreshold);
        Serial.println("Connected to MQTT");
    } else {
        Serial.printf("MQTT connect failed, rc=%d\n", client.state());
    }
    return ok;
}

/* ================== Task 1: MQTT ================== */
void mqttTask(void* pvParameters) {
    (void)pvParameters;
    MqttMessage msg;
    unsigned long lastAttempt = 0;

    for (;;) {
        if (!client.connected()) {
            unsigned long now = millis();
            if (now - lastAttempt >= 5000) { lastAttempt = now; reconnectOnce(); }
        } else { client.loop(); }

        if (xQueueReceive(mqttQueue, &msg, 0) == pdTRUE) {
            if (client.connected()) {
                client.publish(msg.topic, msg.payload, true);
                Serial.printf("MQTT Published → %s : %s\n", msg.topic, msg.payload);
            } else {
                xQueueSend(mqttQueue, &msg, portMAX_DELAY);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/* ================== Task 2: Sensor ================== */
void sensorTask(void* pvParameters) {
    (void)pvParameters;
    for (;;) {
        float humidity = 0, temperature = 0;
        if (readSoilSensor(humidity, temperature)) {
            MqttMessage msg;
            snprintf(msg.topic, sizeof(msg.topic), "%s", topicJSON);
            snprintf(msg.payload, sizeof(msg.payload),
                "{\"moisture\":%.1f,\"temperature\":%.2f,\"runtime\":%d,\"pumpRunning\":%d,\"threshold\":%d}",
                humidity, temperature, pumpRuntime, pumpRunning, threshold);
            xQueueSend(mqttQueue, &msg, portMAX_DELAY);

            snprintf(msg.topic, sizeof(msg.topic), "%s", topicTemperature);
            snprintf(msg.payload, sizeof(msg.payload), "%.2f", temperature);
            xQueueSend(mqttQueue, &msg, portMAX_DELAY);

            checkPumpRuntime(humidity);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/* ================== Task 3: Button ================== */
void onClick() {
    if (!pumpRunning) {
        if (pumpRuntime > 0) {
            digitalWrite(pumpPin, HIGH); pumpRunning = true; pumpStartTime = millis();
            MqttMessage msg; snprintf(msg.topic, sizeof(msg.topic), "%s", topicSprinkler);
            snprintf(msg.payload, sizeof(msg.payload), "1");
            xQueueSend(mqttQueue, &msg, portMAX_DELAY);
        }
    } else if (pumpRunning && !manualMode) {
        digitalWrite(pumpPin, LOW); pumpRunning = false;
        MqttMessage msg; snprintf(msg.topic, sizeof(msg.topic), "%s", topicSprinkler);
        snprintf(msg.payload, sizeof(msg.payload), "0");
        xQueueSend(mqttQueue, &msg, portMAX_DELAY);
    }
}

void onLongPressStart() {
    if (!manualMode) {
        manualMode = true; pumpRunning = true; digitalWrite(pumpPin, HIGH);
        MqttMessage msg; snprintf(msg.topic, sizeof(msg.topic), "%s", topicSprinklerLong);
        snprintf(msg.payload, sizeof(msg.payload), "1");
        xQueueSend(mqttQueue, &msg, portMAX_DELAY);
    } else {
        manualMode = false; pumpRunning = false; digitalWrite(pumpPin, LOW);
        MqttMessage msg; snprintf(msg.topic, sizeof(msg.topic), "%s", topicSprinklerLong);
        snprintf(msg.payload, sizeof(msg.payload), "0");
        xQueueSend(mqttQueue, &msg, portMAX_DELAY);
    }
}

void buttonTask(void* pvParameters) {
    (void)pvParameters;
    oneBtn.setPressMs(800);
    oneBtn.attachClick(onClick);
    oneBtn.attachLongPressStart(onLongPressStart);

    for (;;) {
        oneBtn.tick();
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

/* ================== Setup ================== */
void setup() {
    Serial.begin(115200);
    pinMode(pumpPin, OUTPUT); digitalWrite(pumpPin, LOW);
    pinMode(buttonPin, INPUT_PULLUP);

    loadSettings();
    setup_wifi();
    initRS485();

    espClient.setInsecure();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    client.setKeepAlive(10);

    mqttQueue = xQueueCreate(10, sizeof(MqttMessage));
    if (!mqttQueue) ESP.restart();

    // ---- Connect once during setup and publish saved values before subscribing ----
    if (client.connect(clientID, mqtt_user, mqtt_pass, topicLastwill, 0, true, "System Disconnected")) {
        client.publish(topicLastwill, "System Online", true);

        // publish EEPROM-saved values FIRST
        client.publish(topicRuntime, String(pumpRuntime).c_str(), true);
        client.publish(topicThreshold, String(threshold).c_str(), true);

        // only now subscribe to topics
        client.subscribe(topicSprinkler);
        client.subscribe(topicRuntime);
        client.subscribe(topicSprinklerLong);
        client.subscribe(topicThreshold);

        Serial.println("MQTT connected and initial values published");
    }

    xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(buttonTask, "Button Task", 2048, NULL, 1, NULL, 1);
}

void loop() {}
