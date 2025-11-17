"""
SPV4 Visualization - Separate Process
Reads platform state from a binary file and displays 3D visualization.
"""

import time
import numpy as np
import struct
import sys
import os

# Shared data structure:
# 9 doubles: x1, x2, x3 (3 points, 3 coords each), 6 angles
# Total: 9*3 = 27 doubles + 6 doubles = 33 doubles BUT we'll pack simpler:
# x1_x, x1_y, x1_z, x2_x, x2_y, x2_z, x3_x, x3_y, x3_z, theta_11, theta_12, theta_21, theta_22, theta_31, theta_32, timestamp
# = 16 doubles (15 + timestamp)
STRUCT_FORMAT = '16d'  # 16 doubles (128 bytes)
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

def visualize_spv4(data_file='platform_state.dat'):
    """Visualize Stewart Platform from shared data file."""
    
    # Import here to avoid slowing down controller startup
    from SPV4 import StewartPlatform
    import matplotlib.pyplot as plt
    
    # Create platform
    platform = StewartPlatform(l=25.9, l_base=10, l_link1=8, l_link2=10)
    
    # Set up matplotlib
    plt.ion()
    plt.rcParams['figure.raise_window'] = False  # Don't steal focus on update
    
    print(f"SPV4 Visualization - Waiting for data file: {data_file}")
    
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
                            unpacked = struct.unpack(STRUCT_FORMAT, data)
                            
                            # Extract data
                            x1 = np.array(unpacked[0:3])
                            x2 = np.array(unpacked[3:6])
                            x3 = np.array(unpacked[6:9])
                            theta_11, theta_12, theta_21, theta_22, theta_31, theta_32 = unpacked[9:15]
                            timestamp = unpacked[15]
                            
                            # Only update if data is new
                            if timestamp != last_timestamp:
                                last_timestamp = timestamp
                                no_data_count = 0
                                
                                # Visualize platform
                                platform.visualize(x1, x2, x3,
                                                 theta_11, theta_12,
                                                 theta_21, theta_22,
                                                 theta_31, theta_32)
                                
                                # Add FPS to title
                                if platform.ax is not None:
                                    platform.ax.set_title(f'Stewart Platform (FPS: {fps:.1f})')
                                
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
                if no_data_count > 0 and platform.fig is not None:
                    platform.fig.canvas.flush_events()
                
                time.sleep(0.02)  # ~50 Hz visualization update rate
                
            except Exception as e:
                print(f"Error reading data: {e}")
                time.sleep(0.1)
                continue
            
    except KeyboardInterrupt:
        print("\nSPV4 Visualization stopped")
    finally:
        plt.close('all')


if __name__ == "__main__":
    data_file = sys.argv[1] if len(sys.argv) > 1 else 'platform_state.dat'
    visualize_spv4(data_file)
