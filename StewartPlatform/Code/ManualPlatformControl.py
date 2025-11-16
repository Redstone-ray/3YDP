"""Simple manual platform control using arrow keys."""

import time
import numpy as np
import keyboard
import matplotlib.pyplot as plt
from SPV4 import StewartPlatform
from ServoController import ServoController

# Configure matplotlib for better rendering
plt.ion()  # Enable interactive mode
plt.rcParams['figure.raise_window'] = False  # Don't steal focus

# Initialize
platform = StewartPlatform(l=25.9, l_base=10, l_link1=8, l_link2=10)
controller = ServoController(
    port="COM8", visualize=True,
    flip_servo=[True, True, True],
    neutral_angles=[122, 126, 126],
    min_servo_angle=76, max_servo_angle=155
)

controller.print_commanded_angle_limits()

# State
pitch, roll, height = 0.0, 0.0, 12.0
MAX_ANGLE, MAX_HEIGHT, MIN_HEIGHT = 20, 15.0, 4.0


print("\nArrows: Pitch/Roll | I/K: Height | H: Home | ESC: Quit\n")

try:
    last_angles = None
    frame_count = 0
    while True:
        # Read keys
        if keyboard.is_pressed('esc'): break
        if keyboard.is_pressed('up'): pitch = min(pitch + 1, MAX_ANGLE)
        if keyboard.is_pressed('down'): pitch = max(pitch - 1, -MAX_ANGLE)
        if keyboard.is_pressed('left'): roll = max(roll - 1, -MAX_ANGLE)
        if keyboard.is_pressed('right'): roll = min(roll + 1, MAX_ANGLE)
        if keyboard.is_pressed('i'): height = min(height + 0.3, MAX_HEIGHT)
        if keyboard.is_pressed('k'): height = max(height - 0.3, MIN_HEIGHT)
        if keyboard.is_pressed('h'):
            pitch, roll, height = 0.0, 0.0, 12.0
        
        # Calculate and send
        try:
            p_rad, r_rad = pitch * np.pi / 180, roll * np.pi / 180 #deg to rad
            normal = [-np.sin(r_rad), np.sin(p_rad), np.cos(p_rad) * np.cos(r_rad)]
            ik = platform.inverse_kinematics(normal, [0, 0, height])
            angles = [ik['theta_11'], ik['theta_21'], ik['theta_31']]
            
            # Only send if changed (prevents clamping spam)
            if angles != last_angles:
                controller.send_angles(angles)
                print(f"P:{pitch:+.1f}° R:{roll:+.1f}° H:{height:.1f}mm | θ:{[f'{a:.1f}' for a in angles]}")
                last_angles = angles
        except:
            pass  # Ignore unreachable positions
        
        # Process matplotlib events to keep window responsive
        # Reduce update frequency to prevent overload
        frame_count += 1
        if frame_count % 5 == 0:  # Update display every 5 frames
            try:
                plt.gcf().canvas.draw_idle()  # Request draw instead of forcing it
                plt.gcf().canvas.flush_events()  # Process events without blocking
            except:
                pass  # Ignore matplotlib errors
            time.sleep(0.02)
        else:
            time.sleep(0.005)
        
finally:
    controller.close()
