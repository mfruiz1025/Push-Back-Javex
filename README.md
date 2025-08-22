# 🚀 VEX GPS Autonomous Template - Complete Guide

A professional-grade VEX V5 template with GPS-based positioning, advanced PID control, and sophisticated autonomous capabilities.

## 📁 Project Structure Overview

```
AutonomoConGPS/
├── include/          # Header files (.h) - Declarations
├── src/             # Source files (.cpp) - Implementations  
├── data/            # Data files (PID tuning, etc.)
├── vex/             # VEX build system files
└── main files       # makefile, LICENSE, etc.
```

---

## 🏗️ INCLUDE Directory - Header Files

### 🔧 Core System Files

| **File** | **Purpose** | **What It Does** |
|----------|-------------|------------------|
| **`vex.h`** | VEX API wrapper | Includes all VEX libraries, defines utility macros (`waitUntil`, `repeat`) |
| **`robot-config.h`** | Hardware definitions | Declares all motors, sensors, pneumatics, and controller |
| **`parameters.h`** | Configuration constants | PID values, power settings, robot-specific parameters |

### 🎮 Control & Input Files

| **File** | **Purpose** | **What It Does** |
|----------|-------------|------------------|
| **`controller.h`** | Controller input handling | Maps all controller buttons/joysticks to variables (`A1`, `A2`, `A3`, `A4`, `X`, `Y`, `A`, `B`, etc.) |
| **`basic-functions.h`** | Low-level movement | Basic motor control functions (`moveLeft`, `moveRight`, `moveForward`, `lockBase`) |

### 🧠 Intelligence & Navigation Files

| **File** | **Purpose** | **What It Does** |
|----------|-------------|------------------|
| **`GPS.h`** | GPS positioning system | Advanced GPS-based movement with precise coordinate targeting |
| **`base.h`** | Advanced movement | Sophisticated base control (PID movement, timed movement, curves) |
| **`sensors.h`** | Sensor data management | Handles encoder, IMU, and sensor data processing |
| **`autonomous.h`** | Autonomous routines | Declarations for all autonomous programs |

### 🔬 Utility & Math Files

| **File** | **Purpose** | **What It Does** |
|----------|-------------|------------------|
| **`PID.h`** | PID controller class | Precise control algorithms for smooth, accurate movement |
| **`math-tools.h`** | Mathematical utilities | Angle calculations, coordinate transformations |
| **`my-timer.h`** | Timer utilities | Custom timer class for autonomous timing |
| **`adjusment.h`** | PID tuning tools | Functions to tune PID parameters and log data |

---

## 📝 SRC Directory - Source Files

### 🔧 Core Implementation Files

| **File** | **Key Functions** | **What It Does** |
|----------|-------------------|------------------|
| **`main.cpp`** | `main()`, `usercontrol()`, `autonomous()` | Program entry point, driver control, autonomous selection |
| **`robot-config.cpp`** | `vexcodeInit()`, motor/sensor setup | Hardware initialization and configuration |

### 🎮 Movement & Control Implementation

| **File** | **Key Functions** | **What It Does** |
|----------|-------------------|------------------|
| **`basic-functions.cpp`** | `moveLeft()`, `moveRight()`, `moveForward()`, `lockBase()` | Basic motor control implementation |
| **`base.cpp`** | `timerForward()`, `posForwardRel()`, `PIDPosForwardAbs()` | Advanced movement with PID, timing, heading control |
| **`sensors.cpp`** | `updateSensors()`, `getBaseHeading()`, `resetBasePos()` | Sensor data processing and position tracking |

### 🧠 Intelligence Implementation

| **File** | **Key Functions** | **What It Does** |
|----------|-------------------|------------------|
| **`GPS.cpp`** | `gpsPIDMove()`, `gpsAim()`, `updateGpsPos()` | GPS-based navigation and coordinate targeting |
| **`autonomous.cpp`** | `auton_gps_precision()`, `auton_gps_test()` | Your autonomous routines and sequences |
| **`PID.cpp`** | PID algorithm implementation | Precise control mathematics |

### 🔬 Utility Implementation

| **File** | **Key Functions** | **What It Does** |
|----------|-------------------|------------------|
| **`math-tools.cpp`** | `deg2rad()`, `calAbsDeltaAng()`, `sign()` | Mathematical utility functions |
| **`my-timer.cpp`** | Timer class implementation | Custom timing functionality |
| **`adjustment.cpp`** | PID tuning utilities | Tools for optimizing PID parameters |

---

## 🎯 How to Use the Template

### 1. 🏁 Getting Started

```cpp
// 1. Configure your robot hardware in robot-config.cpp
motor MotorL1(PORT14, ratio18_1, true);  // Define your motors
inertial IMU = inertial(PORT11);          // Define sensors

// 2. Set parameters in parameters.h
const float BASE_LEFT_POWER = 1.0;       // Motor power scaling
const float BASE_FORWARD_PID[3] = {2.0, 0.1, 0.5}; // PID tuning
```

### 2. 🎮 Driver Control

```cpp
// In usercontrol() - Manual joystick control
if (std::abs(A3 + A1) > MOVEMENT_LOWER_LIMIT)
  moveLeft(A3 + A1);   // Tank/Arcade drive mixing

// Controller buttons for mechanisms
if(B && !last_B) {
  Pinza.close();       // Gripper control
}
```

### 3. 🤖 Autonomous Programming

```cpp
// Create your autonomous in autonomous.cpp
void auton_my_routine(void) {
  auton_init();  // Initialize GPS and timers
  
  // GPS-based precise movement
  my_gps.gpsPIDMove(600, 0, 1, 50, "PID");      // Move to (600,0)
  my_gps.gpsPIDMove(0, 600, 1, 50, "continue"); // Move to (0,600)
  
  // Mechanism control
  Pinza.close();
  recoleccion_precise(100, 1000);
  
  auton_pre_usercontrol();  // Clean up
}
```

### 4. 🎯 Movement Functions Reference

| **Function** | **Use Case** | **Example** |
|--------------|--------------|-------------|
| **`moveLeft(power)`** | Basic motor control | `moveLeft(50)` - 50% power |
| **`my_base.timerForward(power, time)`** | Timed movement | `my_base.timerForward(50, 1000)` - 1 second |
| **`my_base.posForwardRel(power, distance)`** | Distance-based | `my_base.posForwardRel(50, 500)` - 500mm |
| **`my_gps.gpsPIDMove(x, y, dir, power, mode)`** | GPS precision | `my_gps.gpsPIDMove(600, 0, 1, 50, "PID")` |

### 5. 📊 GPS Movement Modes

| **Mode** | **Rotation Before Move** | **Movement Type** | **Final Stopping** | **Best For** |
|----------|-------------------------|-------------------|-------------------|--------------|
| **"PID"** | ✅ Yes - Rotates to target angle first | Moves 70% distance with heading correction | ✅ PID Stop - Precise deceleration | Precise positioning, scoring |
| **"continue"** | ❌ No - Skips initial rotation | Moves 70% distance with heading correction | ✅ PID Stop - Precise deceleration | Smooth path following |
| **"heading"** | ✅ Yes - Rotates to target angle first | Moves full distance with heading correction | ❌ No PID Stop - Continues without brake | Continuous motion |

### 6. 📍 GPS Function Parameters

#### `gpsPIDMove(x_target, y_target, direction, max_power, mode)`
- **`x_target`** - Target X coordinate in mm (positive = right, negative = left)
- **`y_target`** - Target Y coordinate in mm (positive = forward, negative = backward)
- **`direction`** - Movement direction: `1` = forward, `-1` = backward
- **`max_power`** - Maximum power percentage (0-100)
- **`mode`** - `"PID"` for precise stop, `"continue"` for smooth transition, `"heading"` for heading correction

#### `gpsAim(x_target, y_target, heading_offset)`
- **`x_target`** - X coordinate of target to aim at in mm
- **`y_target`** - Y coordinate of target to aim at in mm  
- **`heading_offset`** - Additional angle offset in degrees (0 = point directly at target)

### 7. 📊 Sensor Usage

```cpp
// Get current position/heading
float heading = my_sensors.getBaseHeading();
float forward_pos = my_sensors.getBaseForwardPos();
float gps_x = my_gps.getGpsX();
float gps_y = my_gps.getGpsY();

// Reset sensors
my_sensors.resetBasePos();
my_gps.initGPS(0, 0, 0);  // Reset GPS to origin
```

### 8. 🔧 PID Tuning

```cpp
// Adjust in parameters.h
const float BASE_FORWARD_PID[3] = {Kp, Ki, Kd};   // Forward movement
const float BASE_ROTATE_BIG_PID[3] = {Kp, Ki, Kd}; // Large turns
const float BASE_ROTATE_SMALL_PID[3] = {Kp, Ki, Kd}; // Small adjustments
```

---

## 🎛️ Template Features

### ✅ Built-in Capabilities
- **GPS-based positioning** with real-world coordinates
- **Multi-layer movement system** (basic → advanced → GPS)
- **PID control** for smooth, precise movement
- **Sensor fusion** (GPS + IMU + encoders)
- **Autonomous selection** with controller interface
- **Real-time debugging** via brain/controller screens
- **Background threading** for continuous sensor updates

### 🎮 Ready-to-Use Functions
- **Movement**: `moveLeft`, `moveRight`, `moveForward`, `lockBase`
- **Advanced**: `timerForward`, `posForwardRel`, `PIDPosForwardAbs`
- **GPS**: `gpsPIDMove`, `gpsAim`, `updateGpsPos`
- **Mechanisms**: Direct pneumatic/motor control via `robot-config`

---

## 🎮 Controller Interface

### Button Mappings
- **RIGHT Arrow**: Cycle through autonomous modes
- **DOWN Arrow**: Run autonomous manually (for testing)
- **UP Arrow**: Toggle between GPS and manual mode (if implemented)
- **B Button**: Base lock toggle
- **A1-A4**: Joystick axes for manual control
- **L1, L2, R1, R2, X, Y, A, B**: Available for mechanism control

### Autonomous Selection
1. **GPS Precision**: Your custom autonomous sequence
2. **Auto Two**: Available slot
3. **Auto Three**: Available slot  
4. **Auto Four**: Available slot
5. **Skills/Test**: Skills challenge or GPS test routine

---

## 🔧 Configuration & Setup

### Hardware Configuration
1. **Edit `robot-config.cpp`**: Define all motors, sensors, and pneumatics
2. **Update port assignments**: Match your robot's wiring
3. **Configure gear ratios**: Set appropriate gear ratios for your motors

### Parameter Tuning
1. **Edit `parameters.h`**: Adjust PID values and power settings
2. **Test movement**: Use `auton_gps_test()` to verify GPS accuracy
3. **Tune PID**: Use adjustment functions for optimal performance

### Creating New Autonomous
1. **Add function to `autonomous.cpp`**: Write your routine
2. **Declare in `autonomous.h`**: Add function declaration
3. **Add to switch case in `main.cpp`**: Include in autonomous selection

---

## 🏆 Example Autonomous Sequences

### GPS Test Pattern (Z-Route)
```cpp
void auton_gps_test(void) {
  auton_init();
  my_gps.gpsPIDMove(600, 0, 1, 60);      // Right
  my_gps.gpsPIDMove(0, 600, 1, 60);      // Forward
  my_gps.gpsPIDMove(600, 600, 1, 60);    // Diagonal
  my_gps.gpsPIDMove(0, 0, 1, 60);        // Home
  my_gps.gpsAim(0, -1200, 0);           // Aim behind
  auton_pre_usercontrol();
}
```

### Competition Routine Example
```cpp
void auton_gps_precision(void) {
  auton_init();
  
  // Step 1: Collection
  my_gps.gpsPIDMove(my_gps.getGpsX(), my_gps.getGpsY() + 320, 1, 50, "PID");
  Pinza.close();
  wait(200, msec);
  
  // Step 2: Scoring
  my_gps.gpsPIDMove(my_gps.getGpsX(), my_gps.getGpsY() - 550, -1, 65, "PID");
  Pinza.open();
  recoleccion_precise(100, 3000);
  
  auton_pre_usercontrol();
}
```

---

## 🚨 Troubleshooting

### Common Issues
1. **GPS not initializing**: Check sensor connections and port assignments
2. **Inaccurate movement**: Tune PID parameters in `parameters.h`
3. **Motors not responding**: Verify motor declarations in `robot-config.cpp`
4. **Compilation errors**: Ensure all function declarations match implementations

### Debugging Tools
- **Brain screen**: Shows real-time GPS coordinates and heading
- **Controller screen**: Displays current autonomous mode and status
- **`auton_gps_test()`**: Use for testing GPS accuracy and movement
- **Adjustment functions**: Use for PID tuning and optimization

---

## 📜 License

This template is part of the JAVEX robotics project. See LICENSE file for details.

---

**Ready to compete at the highest levels with GPS precision and advanced control algorithms!** 🏆

*For support or questions, refer to the VEX community forums or your team's documentation.*
