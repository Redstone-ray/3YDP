"""Simple manual platform control using arrow keys."""

import time
import numpy as np
import keyboard
import matplotlib.pyplot as plt
from SPV4 import StewartPlatform
from ServoController import ServoController

# Initialize
platform = StewartPlatform(l=25.9, l_base=10, l_link1=8, l_link2=10)
controller = ServoController(
    port="COM3", visualize=True,
    flip_servo=[True, True, True],
    neutral_angles=[20, 20, 20],
    min_servo_angle=0, max_servo_angle=37
)

controller.print_commanded_angle_limits()

# State
pitch, roll, height = 0.0, 0.0, 12.0
MAX_ANGLE, MAX_HEIGHT, MIN_HEIGHT = 20, 15.0, 4.0


print("\nArrows: Pitch/Roll | I/K: Height | H: Home | ESC: Quit\n")

try:
    last_angles = None
    while True:
        # Read keys
        if keyboard.is_pressed('esc'): break
        if keyboard.is_pressed('up'): pitch = min(pitch + 0.3, MAX_ANGLE)
        if keyboard.is_pressed('down'): pitch = max(pitch - 0.3, -MAX_ANGLE)
        if keyboard.is_pressed('left'): roll = max(roll - 0.3, -MAX_ANGLE)
        if keyboard.is_pressed('right'): roll = min(roll + 0.3, MAX_ANGLE)
        if keyboard.is_pressed('i'): height = min(height + 0.1, MAX_HEIGHT)
        if keyboard.is_pressed('k'): height = max(height - 0.1, MIN_HEIGHT)
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
        plt.pause(0.01)
        
finally:
    controller.close()
