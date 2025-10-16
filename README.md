# Rover-Architecture-Ros2

# URC Rover System - ROS2 Jazzy

A Basic ROS2 implementation simulating (URC) 2026 missions using Turtlesim.
---

## System Overview

This project simulates a Mars rover completing four mission types:
1. **Science Mission**: Navigate to sites, capture images with GNSS data.
2. **Delivery Mission**: Pick and deliver objects using action servers.
3. **Equipment Servicing**: Maintenance tasks at equipment locations.
4. **Autonomous Navigation**: Visit 4 waypoints (2 GNSS, 2 Vision-based).

---

## 🔧 Prerequisites

- **OS**: Ubuntu 24.04
- **ROS2**: Jazzy Jalisco
- **Python**: 3.10+
- **Hardware**: Webcam (optional - uses dummy camera if not available)

---

### 2. Unzip urc_rover_ws.zip
---

### 3. Build Workspace
```bash
cd ~/urc_rover_ws
colcon build
source install/setup.bash
```

---

## Running Missions

### Science Mission

```bash
ros2 launch urc_rover science_mission.launch.py
```

**What it does:**
```
- Turtle navigates to 3 geological sites
- Captures images at each location
- Records GNSS coordinates
- Saves data to `data/images/`
```
**Expected Output:**
```
[science_mission]: Starting Science Mission...
[science_mission]: --- Visiting Site_Alpha ---
[navigation_controller]: Going to (2.0, 2.0)
[science_mission]: Arrived at Site_Alpha!
[camera_handler]: Saved: site01.jpg
```
**Result:**

```
Stuck at navigation (need to fix the bug)
Video: 
```


### Autonomous Navigation
```bash
ros2 launch urc_rover autonomous_nav.launch.py
```

**What it does:**
```
- Visits 4 waypoints autonomously
- 2 GNSS-based waypoints
- 2 Vision-based targets
- LED indicator changes (Red→Green→Red)
```

**Expected Output:**
```
LED STATUS: AUTONOMOUS (Red)
[autonomous_nav]: Waypoint 1/4: GNSS_Point_1
[autonomous_nav]: Reached GNSS_Point_1!
LED STATUS: TARGET_REACHED (Green)
```
**Result:**
```
Stuck at navigation (need to fix the bug)
Video: 
```


### Delivery Mission
```bash
# Terminal 1: Start system
ros2 launch urc_rover rover_system.launch.py mission_mode:=delivery

# Terminal 2: Run delivery
ros2 run urc_rover delivery_mission
```

**What it does:**
```
- Navigates to pickup location
- Picks up package
- Delivers to destination
- Uses ROS2 Action Server/Client
```
**Result:**

```
Stuck at navigation (need to fix the bug)
Video: 
```
---

## Architecture


### Node Graph

|        Node             |     Type      |           Purpose               |
|-------------------------|---------------|---------------------------------|
| `mission_manager`       | Coordinator   |  Manages mission mode switching |
| `navigation_controller` | Action Server | Controls turtle movement        |
| `camera_handler`        | Sensor        | Captures and saves images       |
| `gnss_simulator`        | Sensor        | Simulates GPS coordinates       |
| `led_indicator`         | Indicator     | Displays system status          | 
| `science_mission`       | Mission       | Science task execution          |
| `delivery_mission`      | Mission       | Delivery task with actions      |
| `autonomous_nav`        | Mission       | Autonomous waypoint navigation  |

-----------------------------------------------------------------------------

## 📡 ROS2 Communication

### Topics
|         Topic             |     Type    |    Publisher   |    Subscriber  |     Purpose       |
|---------------------------|-------------|----------------|----------------|-------------------|
| `/turtle1/cmd_vel`        | `Twist`     | nav_controller | turtlesim      | Move turtle       |      
| `/turtle1/pose`           | `Pose`      | turtlesim      | nav_controller | Position feedback |
| `/camera/image_raw`       | `Image`     | camera_handler | -              | Camera feed       |
| `/camera/capture_request` | `String`    | missions       | camera_handler | Request image     |
| `/camera/capture_confirm` | `String`    | camera_handler | missions       | Confirm capture   |
| `/gnss/fix`               | `NavSatFix` | gnss_simulator | missions       | GPS data          |
| `/led_status`             | `String`    | missions       | led_indicator  | LED commands      |


## Project Structure

```
urc_rover/
├── urc_rover/
│   ├── mission_manager.py          
│   ├── missions/
│   │   ├── science_mission.py    
│   │   ├── delivery_mission.py    
│   │   ├── autonomous_nav.py       
│   │   └── servicing_mission.py    
│   ├── navigation/
│   │   └── navigation_controller.py 
│   ├── sensors/
│   │   ├── camera_handler.py      
│   │   └── gnss_simulator.py    
│   └── utils/
│       └── led_indicator.py
├── launch/
│   ├── rover_system.launch.py
│   ├── science_mission.py
│   ├── autonomous_nav.launch.py
