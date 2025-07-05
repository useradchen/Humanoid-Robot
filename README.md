# Humanoid-Robot

(walking_with_joystick)This project provides C++ code to control a humanoid robot's leg joints using Dynamixel actuators. It supports walking, kicking, and balancing, and includes inverse kinematics and trajectory interpolation.

## 🔧 Hardware Compatibility

- **Controllers**: OpenCM 9.04 / OpenCR
- **Actuators**: 12 Dynamixel motors (XM series or compatible)
- **Communication**: 1 Mbps via USB/serial

## 🧩 System Overview

### Joint Mapping

| Joint Name         | ID |
|--------------------|----|
| Left Ankle Roll    |  1 |
| Left Ankle Pitch   |  2 |
| Left Hip Pitch     |  3 |
| Left Hip Roll      |  4 |
| Left Hip Yaw       |  5 |
| Right Ankle Roll   |  6 |
| Right Ankle Pitch  |  7 |
| Right Hip Pitch    |  8 |
| Right Hip Roll     |  9 |
| Right Hip Yaw      | 10 |
| Head Yaw           | 11 |
| Head Pitch         | 12 |

---

## 📌 Key Features

- Inverse Kinematics computation for each leg
- Linear trajectory generation with smooth interpolation
- Separate walking and kicking behavior
- Balance compensation via hip and ankle roll
- Sync write to control multiple actuators simultaneously

---

## 🚀 Functions Overview

### `initializePosition()`
Initializes all actuators:
- Sets joint mode and PID gains
- Moves to a neutral squatting pose using inverse kinematics

---

### `calculateInverseKinematics(float x, float y, float z)`
Calculates the required joint angles for a given foot position.

Returns: `IKAngles` structure with:
- `theta_hp`: hip pitch
- `theta_ap`: ankle pitch
- `theta_ar`: ankle roll
- `theta_hr`: hip roll

---

### `convertRadianToPosition(float radian)`  
### `convertDegreeToPosition(float degree)`
Converts angles to Dynamixel position values (0–4095).

---

### `readAllActuatorPositions(int32_t* positions)`
Reads and stores the current position of all 12 actuators.

---

### `LinearLegTrajectory::generateSmoothTrajectory(...)`
Generates interpolated trajectories between current and target foot positions using sine/cosine curves.

---

### `moveToCoordinate(...)`
Moves one leg to a specified `(x, y, z)` position.
- Used for walking motion
- Includes rotation and side-stepping support

---

### `kickMoveToCoordinate(...)`
Specialized trajectory that simulates a kick:
- Pulls back then swings forward
- Hip and ankle angles are adjusted to exaggerate motion

---

### `walking(...)`
High-level walking function.
- Alternates left and right steps
- Takes `stepLength`, `rotationAngle`, `sideStepLength` as input

---

### `kick(...)`
High-level kicking function.
- Can perform forward or backward kicks
- Controlled by `stepLength`, `rotationAngle`, `sideStepLength`

---

## 🖥️ Sample Use

```cpp
initializePosition();

float step = 50.0; // forward step in mm
float rotation = 15.0; // rotation angle in degrees
float sidestep = 0.0;

walking(step, rotation, sidestep); // Walk forward
kick(step, 0.0, 0.0);              // Kick forward
