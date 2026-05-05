
# Arduino-Based 3-DOF Robotic Arm: Kinematic Analysis and Implementation

## Project Overview
This project involves the design and implementation of a 3-degree-of-freedom (DOF) robotic manipulator for the **Arduino Robotic Training (ART) v9** competition. The system is engineered to solve two primary operational modes: manual joint-space control via serial communication and autonomous Cartesian-space positioning through an analytical Inverse Kinematics (IK) engine.

## The Team
This project was created by:  
- Benberkat Ayoub (Supervisor)
- Ihamouine Abderrahmane (Supervisor)
- BADI Aicha (team member)  
- Zerouali Rayan (team member)
- Mokeddem Chaima (team member)
- Aliouane Abde Elhadi (team member)    

### note: 
- All the team members had designated indevidual roles, **HOWEVER** every member was included and contributed to the other objectives. 
## Kinematic Framework

### 1. Coordinate System Standardization
The system utilizes a standard Cartesian coordinate system (X, Y, Z) where the Z-axis represents vertical altitude, and the XY-plane represents the floor. The origin (0,0,0) is defined at the center of the base rotation plate.

### 2. Angle Based Control:  
We made it possible to control the overall position of the arm by adjusting the rotation of each servo independently using sliders with an application we created.

### 3. Inverse Kinematics (IK) Engine
To calculate the necessary joint angles for a target coordinate, the system employs **Planar Decoupling**:
- **Azimuthal Rotation (base rotation):** Resolved via the floor projection using `atan2(y, x)`.
- **Planar Reach (shortest path & elbow angle):** Calculated as the magnitude of the XY vector.
- **Elevation and Extension (shoulder angle):** Resolved in a 2D vertical plane using the Law of Cosines. The model accounts for the fixed base height (bh) by translating the target to a relative coordinate `Z_relative = Z - base height` before solving the triangle.

## Firmware Architecture

### 1. Data Parsing and State Management
The firmware implements a header-based serial protocol via `SoftwareSerial` to handle incoming packets from the control interface. To manage the limited memory and hardware constraints of the Arduino Uno:
- **Floating-Point Parsing:** Due to standard library limitations regarding `sscanf`, the parser utilizes native `.toFloat()` casting to extract high-precision coordinates.
- **Coordinate Caching:** As coordinates are received as separate packets (Kx, Ky, Kz), the system caches the values in global state variables and only triggers the IK solver upon receipt of the final Z component.
- **Asynchronous Buffering:** A `static` local buffer is utilized within the serial handler to ensure data persistence across non-blocking loops.

### 2. Interpolated Motion Engine
To minimize mechanical stress on 3D-printed components and prevent voltage sags from the MG996R actuators, the firmware avoids instantaneous position writes.
- **Velocity Limiting:** The `updatemotion` function utilizes a `millis()` based timer to increment joint angles toward their targets in steps.
- **Jitter Reduction:** By decoupling the high-level IK math from the low-level servo writes, the system maintains a stable PWM duty cycle even during heavy calculations.

## Hardware Specifications

### Pin Assignment
| Peripheral | Pin | Technical Role |
| :--- | :--- | :--- |
| **HC-05 RX** | 10 | Serial Receive |
| **HC-05 TX** | 11 | Serial Transmit |
| **Base Servo** | 3 | Azimuthal Rotation  |
| **Shoulder Servo** | 5 | Elevation Angle |
| **Elbow Servo** | 6 | Extension Angle |
| **Gripper Servo** | 9 | End Effector State |

### Mechanical Link Parameters
- **Base Height (bh):** 9.0 cm
- **Shoulder Link (L1):** 12.0 cm
- **Elbow Link (L2):** 15.0 cm
- **Total Maximum Reach:** 27.0 cm (excluding base height)

## Repository Structure
- **/code**: Contains all iterations of the firmware, including the final `robotic_arm_app.ino`.
- **/app**: Source files for the MIT App Inventor control dashboard.
- **/schematics**: Circuit diagrams, pinout tables, and hardware assembly notes.
- **/vids&pics**: Visual documentation of simulation tests and physical hardware calibration.
