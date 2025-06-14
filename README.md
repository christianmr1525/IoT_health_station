# IoT Health Monitoring Station

This project consisted on creating an health monitoring embedded system using a ESP32 microcontroller.

This project is a full-stack IoT-based health monitoring station, built entirely from scratch using an ESP32 microcontroller. It integrates multiple biomedical and environmental sensors to collect and transmit real-time data to a web dashboard, enabling remote health and environmental monitoring.

## 🧰 Tech Stack

| Component      | Technology/Tool          |
|----------------|--------------------------|
| Microcontroller| ESP32 DevKit V4          |
| Sensors        | DHT11, MAX30102, MQ-135  |
| Communication  | MQTT (Mosquitto broker)  |
| Backend        | Python                   |
| Database       | MySQL                    |
| Cloud          | Microsoft Azure          |
| Debugging Tools| Putty, Serial Monitor    |
| PCB Design     | KiCad                    |
| 3D Modeling    | SolidWorks               |

---
## 🖥️ System Architecture

```text
[Sensors]
    ↓
[ESP32 Microcontroller]
    ↓ (MQTT)
[Python Backend on Azure]
    ↓
[Database + Web Dashboard]
```

![3D Model](Photos/3D_model.jpg)

The 3D model, pcb, programming and comissioning was built from scratch.

The result is shown below:

![Health Station result](Photos/Health_Station_result.jpg)


