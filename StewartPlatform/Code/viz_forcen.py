"""
Forcen Visualization - Separate Process
Reads ball position data from a binary file and displays it.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import struct
import sys
import os

# Shared data structure:
# 8 floats: x, y, weight_g, pitch, roll, error_x, error_y, timestamp
STRUCT_FORMAT = '8d'  # 8 doubles (64 bytes)
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

def visualize_forcen(data_file='ball_state.dat'):
    """Visualize ball position from shared data file."""
    
    # Set up the plot
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-150, 150)
    ax.set_ylim(-150, 150)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.axvline(x=0, color='k', linewidth=0.5)
    
    # Draw setpoint
    setpoint = Circle((0, 0), 5, color='red', alpha=0.5, label='Setpoint')
    ax.add_patch(setpoint)
    
    # Draw ball
    ball = Circle((0, 0), 25.4, color='grey', alpha=0.3)
    ax.add_patch(ball)
    
    # Add text displays
    coord_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                         verticalalignment='top', fontfamily='monospace',
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    ax.set_xlabel('X Position (mm)')
    ax.set_ylabel('Y Position (mm)')
    ax.set_title('Ball Position - Real-time Visualization')
    ax.legend()
    
    plt.ion()
    plt.rcParams['figure.raise_window'] = False  # Don't steal focus on update
    plt.show()
    
    print(f"Waiting for data file: {data_file}")
    
    last_timestamp = 0
    frame_count = 0
    fps_start = time.time()
    fps = 0.0
    no_data_count = 0
    
    try:
        while True:
            try:
                if os.path.exists(data_file):
                    with open(data_file, 'rb') as f:
                        data = f.read(STRUCT_SIZE)
                        if len(data) == STRUCT_SIZE:
                            x, y, weight_g, pitch, roll, error_x, error_y, timestamp = struct.unpack(STRUCT_FORMAT, data)
                            
                            # Only update if data is new
                            if timestamp != last_timestamp:
                                last_timestamp = timestamp
                                no_data_count = 0
                                
                                # Update ball position
                                ball.center = (x, y)
                                
                                # Update text
                                coord_text.set_text(
                                    f'X: {x:7.1f} mm\n'
                                    f'Y: {y:7.1f} mm\n'
                                    f'Weight: {weight_g:5.1f} g\n'
                                    f'Pitch: {pitch:+5.1f}°\n'
                                    f'Roll: {roll:+5.1f}°\n'
                                    f'FPS: {fps:4.1f}'
                                )
                                
                                # Color based on distance from setpoint
                                distance = np.sqrt(error_x**2 + error_y**2)
                                if weight_g < 300:
                                    ball.set_color('grey')
                                    ball.set_alpha(0.3)
                                elif distance < 10:
                                    ball.set_color('green')
                                    ball.set_alpha(0.8)
                                elif distance < 30:
                                    ball.set_color('yellow')
                                    ball.set_alpha(0.8)
                                else:
                                    ball.set_color('orange')
                                    ball.set_alpha(0.8)
                                
                                # Update plot
                                fig.canvas.draw_idle()
                                fig.canvas.flush_events()
                                
                                # Calculate FPS
                                frame_count += 1
                                if frame_count % 20 == 0:
                                    fps = 20 / (time.time() - fps_start)
                                    fps_start = time.time()
                            else:
                                no_data_count += 1
                else:
                    no_data_count += 1
                
                # Keep matplotlib responsive
                if no_data_count > 0:
                    fig.canvas.flush_events()
                
                time.sleep(0.02)  # ~50 Hz visualization update rate
                
            except Exception as e:
                print(f"Error reading data: {e}")
                time.sleep(0.1)
                continue
            
    except KeyboardInterrupt:
        print("\nVisualization stopped")
    finally:
        plt.close()


if __name__ == "__main__":
    data_file = sys.argv[1] if len(sys.argv) > 1 else 'ball_state.dat'
    visualize_forcen(data_file)
