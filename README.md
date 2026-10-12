# 🌱 Flora-Gotchi: Plant Empathy System (AI x IoT)

**Flora-Gotchi** (AgriSense) is a fully local, automated smart irrigation and early-stage pest detection system developed by **Team Gerbang Logika** from **SMA Kristen Gloria 2 Surabaya** for the **AI x IoT Hackathon 2026 (FTI UK Petra)**.

The system pairs an ESP32 microcontroller with an 8-class vision classification model running directly on a mobile browser to automate watering, detect plant diseases, and prevent water waste—**100% offline without any cloud server, third-party tunneling, or subscription fees**.

---

## 🚀 Key Features

* **Soil-Moisture-Driven Irrigation State Machine:** Utilizes an `IDLE` ➔ `WATERING` ➔ `SOAK_CHECK` logic loop. Water is only delivered after two consecutive dry readings and verified after soaking (+8% moisture increase).
* **8-Class On-Device AI Vision Model:** Uses TensorFlow.js to classify leaf conditions (`healthy_leaf`, `yellow_leaf`, `dry_leaf`, `aphid`, `caterpillar`, `snail`, `whitefly`, `spider_mite`) using the mobile device's camera via `/vision`.
* **On-Board ESP32 Web Server:** Hosts the *"Terrarium Journal"* virtual pet web dashboard directly from the ESP32 chip (`http://192.168.100.100` or `http://plant.local`).
* **Hardware Resilience & Safety Logic:**
  * **Isolated Power Supply:** Mini water pump is powered by a separate 4x AA battery pack via a 5V Relay module to eliminate ESP32 brownout/boot-loop issues.
  * **Brownout & Reset Protection:** Firmware inspects `esp_reset_reason()`. Three consecutive unclean resets trigger a pump lockout and activate the reservoir alarm.
  * **Boot Delay Guard:** Ignores relay inputs for the first 4 seconds upon boot to prevent accidental flooding.
  * **Noise Filter:** Averages 8 analog readings per sensing cycle to prevent sensor jitter.

---

## 📂 Repository Structure

```text
.
├── dashboard/        # Web dashboard UI assets and TensorFlow.js scanner logic
├── docs/             # Final report, logbooks, schematics, and presentation slides (.pptx/.pdf in Indonesian)
├── firmware/         # Complete ESP32 Arduino C++ firmware (plant_watering_v3_ai_vision.ino)
├── model/            # TensorFlow.js model files exported from Google Teachable Machine
├── .gitignore        # Git exclusion rules for IDEs and build artifacts
└── LICENSE.txt       # Project license