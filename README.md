# Maze-Solving Robot 🤖

Autonomous differential-drive robot that navigates an unknown maze using the **Bug2 algorithm**, records its path, and returns to the starting position.

**University of York | ELE00098H Robotics Design and Construction | January 2026**

![20260126_112141](https://github.com/user-attachments/assets/1270bafc-66b2-4bb2-842c-280b5c00ac31)


<img width="383" height="710" alt="image" src="https://github.com/user-attachments/assets/1a0a6042-b6a1-4ad9-b69d-6a8edeeafd8c" />

---

## 🎯 Project Overview

This project was developed as part of the ELE00098H Robotics module assessment. The robot must:

1. **Navigate** from START to FINISH through an unknown maze
2. **Avoid obstacles** using real-time sensor feedback
3. **Record** its path during exploration
4. **Return** to start using the recorded path

The maze is approximately 200cm long with corridor widths of 29-34cm.

---

## ✅ Features

- **Bug2 Navigation Algorithm** - Modified for sensor-based decision making
- **Path Recording** - Every move stored for efficient return journey
- **5 Distance Sensors** - Ultrasonic (front) + 4x IR (left, right, front-left, front-right)
- **Encoder Odometry** - Tracks distance with straight-line correction
- **Obstacle Avoidance** - Real-time diagonal obstacle detection and avoidance maneuvers
- **Dual Speed Modes** - Slow exploration (0.3) and fast return (0.6)

---

## 🔧 Hardware

| Component | Description |
|-----------|-------------|
| **Microcontroller** | Arduino Nano 33 BLE |
| **Motor Driver** | TB6612FNG Dual H-Bridge |
| **Motors** | 2x Micro metal gear motors with quadrature encoders |
| **Front Sensor** | HC-SR04 Ultrasonic (long range) |
| **Side Sensors** | 4x Sharp IR distance sensors via TCA9548A I2C multiplexer |
| **Power** | Li-Po battery pack |
| **Chassis** | Custom 3D printed / laser cut design |

### Sensor Configuration

```
        [Front US]
           ↑
    [FL]   |   [FR]
      \    |    /
       \   |   /
  [L] ← [ROBOT] → [R]
```

- **Front (US):** Long-range obstacle detection (up to 400cm)
- **Left/Right (IR):** Wall detection for corridor navigation
- **Front-Left/Front-Right (IR):** Diagonal obstacle detection at 45°

---

## 🧠 Algorithm

### Bug2 Navigation (Modified)

The classic Bug2 algorithm tries to move directly toward a goal, and when blocked, follows the obstacle boundary until it can resume direct travel. Our modification uses sensor data for smarter turn decisions.

#### Decision Logic

```
┌─────────────────────────────────────────────────────────────┐
│                      MAIN LOOP                               │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. IF facing NORTH and front clear:                         │
│     → Move forward, track north progress                     │
│     → Check if finish reached (≥195cm north)                 │
│                                                              │
│  2. IF facing EAST and left opens up:                        │
│     → Turn left (toward north)                               │
│     → If north blocked, turn back + move forward             │
│                                                              │
│  3. IF facing WEST and right opens up:                       │
│     → Turn right (toward north)                              │
│     → If north blocked, turn back + move forward             │
│                                                              │
│  4. IF facing SOUTH:                                         │
│     → Turn toward any open direction                         │
│                                                              │
│  5. IF front blocked:                                        │
│     → Compare left vs right sensor readings                  │
│     → Turn toward MORE OPEN direction                        │
│     → Move forward short to clear corner                     │
│                                                              │
│  6. ELSE (front clear, not facing north):                    │
│     → Move forward                                           │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Path Recording & Return

Every move is recorded as one of five types:

| Move Type | Description |
|-----------|-------------|
| `M_FORWARD` | Forward 20cm |
| `M_FORWARD_SHORT` | Forward 10cm |
| `M_RIGHT` | Turn right 90° |
| `M_LEFT` | Turn left 90° |
| `M_AWAY` | Turn around 180° |

**Return Algorithm:**
1. Turn 180° to face start
2. Replay moves in reverse order
3. Invert turns: `LEFT ↔ RIGHT`
4. Execute at higher speed (0.7 vs 0.3)

---

## 🐛 Challenges & Solutions

### 1. Ultrasonic Sensor Noise

**Problem:** Random low readings (0.5-2cm) caused false wall detections, stopping the robot in clear corridors.

**Symptoms:**
```
B:0 F:32.15 L:14.5 R:13.2
Action: Moving Forward
North progress: 60

B:0 F:0.52 L:14.1 R:13.7
Wall stop at: 0.52          ← False detection!
```

**Solution:** Two-part filtering:
```cpp
if(f_val < front_closed && f_val > 7){  // Ignore readings < 7cm
  wall_cnt++;
  if(wall_cnt >= 5){                     // Require 5 consecutive readings
    stop();
  }
} else {
  wall_cnt = 0;                          // Reset on clean reading
}
```

### 2. Infinite Loop at Junctions

**Problem:** Robot got stuck turning back and forth at junctions without moving forward.

**Symptoms:**
```
B:0 F:0.66 → Turn Right
B:1 → Turn Left back to North  
B:0 F:0.97 → Turn Right
[repeats forever]
```

**Solution:** Added forward movement after turn-back sequences:
```cpp
if(check < 10){
  turn_right();           // North blocked, turn back
  go_forward_short();     // CRITICAL: Move away from junction
}
```

### 3. Suboptimal Path Selection

**Problem:** Fixed right-turn preference caused robot to take longer routes.

**Solution:** Sensor-based direction choice:
```cpp
if(l_val > 15 && r_val > 15){
  // Both sides open - choose MORE OPEN direction
  if(r_val > l_val) turn_right();
  else turn_left();
}
```

---

## 📁 Project Structure

```
maze-solving-robot/
├── README.md
├── src/
│   ├── working/                    # Final working implementation
│   │   ├── RoboBug2_working_return.ino
│   │   ├── RoboHW.cpp             # Hardware abstraction
│   │   ├── RoboHW.h
│   │   ├── RoboNav.cpp            # Navigation functions
│   │   ├── RoboNav.h
│   │   ├── RoboMap.cpp            # Bug2 algorithm & path recording
│   │   └── RoboMap.h
│   │
│   └── experimental/               # Alternative approaches (not used)
│       ├── grid-based/            # Grid mapping approach
│       │   ├── GridHW.cpp/h
│       │   ├── GridNav.cpp/h
│       │   └── GridMap.cpp/h
│       └── node-based/            # Topological mapping approach
│           ├── NodesHW.cpp/h
│           ├── NodesNav.cpp/h
│           └── NodesMap.cpp/h
│
├── docs/
│   ├── images/
│   └── presentation.pdf
│
└── hardware/
    └── CAD/
```

---

## 🔬 Experimental Approaches

### Grid-Based Mapping

**Concept:** Divide maze into discrete cells, track visited/blocked/unknown states, use depth-first search with backtracking.

**Why it failed:** 
- Required precise odometry that our hardware couldn't reliably provide
- Small positioning errors accumulated over time
- Robot lost track of its actual grid position

**What we learned:** Map-based approaches need accurate localization.

### Node-Based (Topological) Mapping

**Concept:** Represent maze as a graph where junctions are nodes and corridors are edges. No precise positioning needed.

**Why it failed:**
- Junction detection was unreliable
- Sometimes detected false junctions in corridors
- Sometimes missed real junctions

**What we learned:** Junction detection requires careful tuning and possibly additional sensors.

### Why Bug2 Worked

- **Reactive** - Responds to current sensor readings, not accumulated position
- **No map required** - Doesn't need to remember maze structure
- **Robust to errors** - Odometry drift doesn't compound
- **Simple** - Fewer failure modes

---

## 🚀 Building & Running

### Prerequisites

- Arduino IDE 2.x
- Arduino Mbed OS Nano Boards package

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/OsamaAlsahli/maze-solving-robot.git
   ```

2. Open `src/working/RoboBug2_working_return.ino` in Arduino IDE

3. Select board: **Arduino Nano 33 BLE**

4. Upload to robot

5. Open Serial Monitor (9600 baud) to view debug output

### Serial Output

```
==========BUG2 TRIAL==========
B:0 F:45.23 L:14.2 R:13.8
Action: Moving Forward
North progress: 20

B:0 F:38.67 L:13.9 R:14.1
Action: Moving Forward
North progress: 40

...

=== REACHED FINISH LINE! ===
=== RECORDED PATH ===
Total moves: 23
0: FWD
1: FWD
2: RIGHT
...
=== RETURNING TO START ===
Return step 1/23
...
=== BACK AT START ===
COMPLETE!
```

---

## 📊 Results

| Metric | Value |
|--------|-------|
| Maze Length | ~200cm |
| Completion Rate | ✅ 80% |
| Average Solve Time | ~45 seconds |
| Return Journey | ✅ Working |
| Unknown Maze Test | ⚠️ Partial Completion |
| Overall Assessment | ✅ Passed ("flying colours") |
---

## 📚 Lessons Learned

1. **Start simple** - Reactive algorithms often outperform complex map-based approaches
2. **Sensor filtering is critical** - Real sensors are noisy; always filter
3. **Test incrementally** - Debug each component before integration
4. **Log everything** - Serial output saved hours of debugging
5. **Have fallback plans** - We tried 3 approaches before finding one that worked

---

## 🎥 Demo

[Demo Video](https://drive.google.com/file/d/1f_MGWNPrCgAjsQp2QtPEfJgEfQqXqQkQ/view?usp=sharing)
---

## 👤 Author

**Osama Alsahli**  
BEng (Hons) Engineering  
University of York

- GitHub: [@OsamaAlsahli](https://github.com/OsamaAlsahli)
- Linkedin: [Linkedin](https://www.linkedin.com/in/osama-alsahli-325080317/)
- email: ossamaalsahli@gmail.com

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- University of York ELE00098H teaching team
- Lab technicians for hardware support and guidance
- Anthropic Claude for debugging assistance

---

*This project was completed as part of the ELE00098H Robotics Design and Construction module, contributing 100% of the module grade.*
