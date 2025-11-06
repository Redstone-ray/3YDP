# Stewart Platform Control System

Python-based control system for a 3-DOF Stewart Platform with Arduino servo controller.

## Files

- **`ServoController.py`**: Serial communication with Arduino, sends servo angles
- **`SPV4.py`**: Stewart Platform kinematics (inverse/forward) and 3D visualization
- **`servo_controller/servo_controller.ino`**: Arduino firmware (if present)

## Features

### ServoController

- **Serial Communication**: Sends 3 bytes (servo angles) to Arduino over serial
- **Neutral Angle Calibration**: Define the horizontal position for each servo
- **Servo Direction Flip**: Reverse servo direction for backwards-mounted servos
- **Auto-kill**: Automatically kills conflicting Python processes holding the serial port
- **3D Visualization**: Optional real-time visualization using matplotlib (requires `SPV4.py`)

### SPV4 (Stewart Platform Kinematics)

- **Inverse Kinematics**: Calculate servo angles from desired platform orientation and position
- **Forward Kinematics**: Calculate platform position from servo angles (simplified - only needs 3 primary angles)
- **3D Visualization**: Interactive matplotlib visualization of platform state

## Installation

```powershell
# Install dependencies
pip install pyserial numpy matplotlib psutil

# Optional: For process management
pip install psutil
```

## Usage

### Interactive Mode

Run `ServoController.py` directly to enter interactive mode:

```powershell
python ServoController.py
```

You'll be prompted to:
1. Select COM port (default: COM3)
2. Enable visualization (y/n)
3. Enter servo angles (e.g., `30 45 60`)

### Configuration

Edit the constants at the bottom of `ServoController.py`:

```python
# Neutral angles: real servo angle when commanded 0° (horizontal position)
# Example: [15, 20, 15] means servo 1 is horizontal at 15°, servo 2 at 20°, servo 3 at 15°
NEUTRAL_ANGLES = [15.0, 20.0, 15.0]

# Flip servo direction if mounted backwards
# Example: [True, False, True] flips servos 1 and 3
FLIP_SERVOS = [True, False, True]
```

**Neutral Angles**: Define where each servo is horizontal (0° commanded angle)

**Flip behavior**: When `True`, servo angle is inverted around its neutral position
- Without flip: `real_angle = neutral + commanded` (e.g., neutral=0, command 5° → send 5°)
- With flip:    `real_angle = neutral - commanded` (e.g., neutral=15, command 5° → send 10°)
- Example with `neutral_angle=15°` and flip=True:
  - Command 0° → Send 15° (horizontal)
  - Command 5° → Send 10° (tilt down)
  - Command -5° → Send 20° (tilt up)
- Use this when servo rotation is opposite to expected direction

### Programmatic Usage

```python
from ServoController import ServoController

# Create controller
controller = ServoController(
    port="COM3",                         # Serial port
    baudrate=115200,                     # Serial speed
    visualize=True,                      # Enable 3D visualization
    flip_servo=[True, False, True],      # Flip servos 1 and 3
    neutral_angles=[15.0, 20.0, 15.0]    # Horizontal positions
)

# Send angles
controller.send_angles([30, 45, 60])  # Degrees

# Close when done
controller.close()
```

### With SPV4 Kinematics

```python
from SPV4 import StewartPlatform
from ServoController import ServoController

# Create platform and controller
platform = StewartPlatform(l=10, l_base=10, l_link1=8, l_link2=8)
controller = ServoController(port="COM3", visualize=False)

# Define desired orientation and position
normal_vector = [0.1, 0, 1]  # Platform normal (will be normalized)
center_position = [0, 0, 14]  # Platform center in 3D space

# Calculate required servo angles
ik = platform.inverse_kinematics(normal_vector, center_position)
servo_angles = [ik['theta_11'], ik['theta_21'], ik['theta_31']]

# Send to Arduino
controller.send_angles(servo_angles)

# Verify with forward kinematics
fk = platform.forward_kinematics(*servo_angles)
print(f"Platform position: {fk['x1']}, {fk['x2']}, {fk['x3']}")
```

## Communication Protocol

### Arduino → Python
- **Format**: 3 bytes per command
- **Byte values**: 0-90 (clamped angle in degrees)
- **Example**: `0x1E 0x2D 0x3C` → Servos at 30°, 45°, 60°

### Processing Pipeline

```
User Input → Offsets → Flip → Clamp (0-90°) → Pack to bytes → Serial → Arduino
   [30°]   →  [35°]  → [55°] →    [55°]      →    0x37      →  COM3
```

## Troubleshooting

### "Permission denied" on COM port
- Close Arduino IDE Serial Monitor
- Check if another Python script is running (auto-killed on startup)
- Verify COM port number in Device Manager

### Import errors
- Without visualization: Only `pyserial` is required
- With visualization: Requires `numpy`, `matplotlib`, and `SPV4.py`

### Servos moving backwards
- Set `flip_servo=[True, False, False]` (or appropriate combination)
- Test with small angles first (e.g., 10°, 20°, 30°)

### Visualization not working
- Ensure `SPV4.py` is in the same directory
- Check matplotlib backend: `import matplotlib; matplotlib.use('TkAgg')`
- Disable if not needed: Set `visualize=False`

## Platform Geometry

Default parameters (modify in `SPV4.py`):
- **Platform triangle side**: 10 units
- **Base circle radius**: 10 units
- **Link 1 length**: 8 units (servo to elbow)
- **Link 2 length**: 8 units (elbow to platform)

Base attachment points at 90°, 210°, 330° on circle.

## License

MIT License - Free for academic and personal use.
