# Reactive Fuzzy Row-Following Robot for Plantain Crops

This repository contains the **source code, ROS 2 nodes, ESP32 firmware, and experimental datasets** for a low-cost tracked robot performing **reactive fuzzy logic–based row following** in plantain crops.

---

## 📸 Field Pictures

<p align="center">
  <img src="docs/img/robot_field_side.jpg" alt="Robot side view in plantain field" width="45%"/>
  <img src="docs/img/robot_field_front.jpg" alt="Robot front view in plantain row" width="45%"/>
</p>

---

## 🎥 Demo Video

[Watch on YouTube](https://www.youtube.com/watch?v=u4LEPf4ucak&list=PLrb4Ia3jTb1fGVidyRJrpKlKpQCu3HL8m)

<p align="center">
  <a href="https://www.youtube.com/watch?v=u4LEPf4ucak&list=PLrb4Ia3jTb1fGVidyRJrpKlKpQCu3HL8m" target="_blank">
    <img src="https://img.youtube.com/vi/u4LEPf4ucak/hqdefault.jpg" alt="YouTube demo thumbnail" width="60%"/>
  </a>
</p>

---

## 📂 Repository Structure (actual)

| Path | Description |
|---|---|
| **`ROS2nodes/`** | ROS 2 Python nodes: LiDAR reading, wall-following (fuzzy), serial bridge to ESP32 (`lidarReadings.py`, `wallFollower.py`, `esp32_serial_bridge.py`). |
| **`esp32/`** | ESP32 firmware: `main.cpp` (PI control + dead-zone compensation + PWM), `platformio.ini`. |
| **`testes/CSV_ROBOT/`** | Experimental datasets (CSV) collected in field/park campaigns. Each subfolder is one experiment (e.g., `PruebaCampo2-cono160-setpoint1metro-distanciacultivo1metro/`, `prueba1_parque/`, etc.). **Contains `plotar.py`** for plotting. |
| **`docs/img/`** | Images used in the README (you should place `robot_field_side.jpg` and `robot_field_front.jpg` here). |

> Note: folders like `firmware/`, `ros2/`, or `data/` mentioned elsewhere are **not** present here. Use the paths above.

---

## 🧭 Quick Start (Plotting)

Follow these steps to generate plots from your experimental CSV data.

### 1️⃣ Go to the plotting script folder
```bash
cd testes/CSV_ROBOT
```
### 2️⃣ Select an experiment folder

Each experiment folder inside `testes/CSV_ROBOT/` contains its own CSV dataset.  
The script `plotar.py` plots **only one experiment at a time**, based on the folder name defined in its configuration.

List available experiment folders:
```bash
ls
```


3️⃣ Edit the configuration
```bash
Open the script:
```
nano plotar.py


Find the line near the top:

```bash
DATA_SUBFOLDER = "PruebaCampo2-cono160-setpoint1metro-distanciacultivo1metro"
```


Replace the text inside the quotes with the name of the folder you want to plot, for example:

```bash

DATA_SUBFOLDER = "prueba1_parque"
```

Save and exit Nano:

Press Ctrl + O, then Enter

Press Ctrl + X to close



4️⃣ Run the plotting script
```bash
python3 plotar.py
```

All generated plots (.png) will be saved inside the same experiment folder you selected.

Example:

testes/CSV_ROBOT/prueba1_parque/
 ├─ Distance_Error.png
 ├─ IMU_Yaw_Error.png
 ├─ Lidar_Left.png
 ├─ Valid_Readings.png
 ├─ comparison_desired_vs_measured_left.png
 └─ comparison_desired_vs_measured_right.png

📊 Default Plot Ranges
Variable	Range	Units
Desired vs. Measured velocity	0 → 6.28	rad/s
Distance error	−1 → +1	m
Yaw error	−2 → +2	rad
Valid readings	0 → 100	count
LiDAR left distance	0 → 3	m
Time window	0 → 300	s


---

## 🤝 <span style="font-size:1.4em; font-weight:bold;">Acknowledgments</span>

This work was developed within the **Postdoctoral Research Program (BitPointer SAS – Ministry of Science, Technology and Innovation, Colombia)**, focusing on **low-cost agricultural robotics** for plantain crops.

<p align="center">
  ---
<!-- BLIND FOR REVIEW: authors/affiliations/acknowledgments removed for double-anonymous submission.
     Restore after peer-review / acceptance. -->
---

</p>

---
