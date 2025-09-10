# 📌 Soil Monitoring System - Changelog

---

## Version 3.1 (Stable Baseline Update)

**Date:** 2025-09-09

### Changes:

* Default runtime changed from **20s → 5s**
* Manual override (OneButton long press) behavior clarified and fixed
* Runtime & Threshold values always included in JSON payload (no separate publishes)
* EEPROM sync retained for `Runtime` and `Threshold`
* Button task logic simplified for stability

---

## Version 3.0 (Stable Baseline)

**Date:** 2025-09-09

### Features:

* EEPROM storage for `Runtime` and `Threshold`
* Queue-based MQTT publishing for reliable delivery
* Three FreeRTOS tasks:

  * **MQTT Task** → Handles broker connection & publishing
  * **Sensor Task** → Reads soil sensor, publishes JSON payload
  * **Button Task** → Handles short/long press with OneButton
* Auto mode: stops pump after runtime or when threshold reached
* Manual mode: override ON/OFF via button or MQTT
* MQTT retained publishes for LastWill and initial values
