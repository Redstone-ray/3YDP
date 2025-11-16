"""
Ball Balancing Controller for Stewart Platform
Integrates ForcenInterface, SPV4 kinematics, and ServoController with PID control.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import keyboard
from ForcenInterface import ForcenInterface
from SPV4 import StewartPlatform
from ServoController import ServoController


class PIDController:
    """PID controller with derivative filtering and feedforward for high-inertia systems."""
    
    def __init__(self, kp=0.1, ki=0.0, kd=0.05, pf=0.3, output_limit=20.0, 
                 derivative_filter_coeff=0.1, use_velocity_feedforward=True):
        """
        Initialize PID controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            pf: Feedforward gain for velocity compensation (default: 0.3)
            output_limit: Maximum output magnitude in degrees
            derivative_filter_coeff: Low-pass filter coefficient for derivative (0-1, lower = more filtering)
            use_velocity_feedforward: Enable velocity-based feedforward for high-inertia loads
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pf = pf
        self.output_limit = output_limit
        self.derivative_filter_coeff = derivative_filter_coeff
        self.use_velocity_feedforward = use_velocity_feedforward
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.filtered_derivative = 0.0
        self.last_position = None
    
    def update(self, error, current_time=None, current_position=None, velocity_estimate=None):
        """
        Calculate PID output with enhanced control for high-inertia systems.
        
        Args:
            error: Current error (setpoint - measurement)
            current_time: Current time in seconds (uses time.time() if None)
            current_position: Current ball position for velocity calculation (optional)
            velocity_estimate: Pre-calculated velocity for feedforward (optional)
        
        Returns:
            Control output (clamped to output_limit)
        """
        if current_time is None:
            current_time = time.time()
        
        # Initialize on first call
        if self.last_time is None:
            self.last_time = current_time
            self.last_error = error
            self.last_position = current_position
            return 0.0
        
        # Calculate time delta
        dt = current_time - self.last_time
        if dt <= 0:
            return 0.0  # Avoid division by zero
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (with anti-windup: only accumulate if not saturated)
        if abs(self.integral * self.ki) < self.output_limit * 0.8:  # 80% threshold
            self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term with low-pass filtering (reduces noise amplification)
        raw_derivative = (error - self.last_error) / dt
        self.filtered_derivative = (self.derivative_filter_coeff * raw_derivative + 
                                    (1 - self.derivative_filter_coeff) * self.filtered_derivative)
        d_term = self.kd * self.filtered_derivative
        
        # Velocity feedforward for predictive control
        ff_term = 0.0
        if self.use_velocity_feedforward and velocity_estimate is not None:
            # Anticipate ball motion: counter-steer against velocity
            # For high inertia, we need to act early
            ff_term = -self.pf * velocity_estimate
        
        # Calculate output
        output = p_term + i_term + d_term + ff_term
        
        # Clamp output
        output = np.clip(output, -self.output_limit, self.output_limit)
        
        # Update state
        self.last_error = error
        self.last_time = current_time
        self.last_position = current_position
        
        return output
    
    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.filtered_derivative = 0.0
        self.last_position = None
    
    def set_gains(self, kp=None, ki=None, kd=None, pf=None):
        """Update PID gains on-the-fly."""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if pf is not None:
            self.pf = pf




class BallBalancingController:
    """Main controller integrating sensor, kinematics, and servo control."""
    
    def __init__(self, 
                 forcen_port='COM7',
                 servo_port='COM8',
                 platform_height=12.0,
                 setpoint_x=0.0,
                 setpoint_y=0.0,
                 pid_kp=0.1,
                 pid_ki=0.0,
                 pid_kd=0.05,
                 pid_pf=0.3,
                 max_tilt=20.0,
                 enable_spv4_viz=True,
                 enable_forcen_viz=True,
                 forcen_config=None,
                 servo_config=None):
        """
        Initialize ball balancing controller.
        
        Args:
            forcen_port: COM port for Forcen sensor
            servo_port: COM port for servo controller
            platform_height: Platform height in mm (default: 12.0)
            setpoint_x: Target X position in mm (default: 0.0)
            setpoint_y: Target Y position in mm (default: 0.0)
            pid_kp: PID proportional gain (default: 0.1)
            pid_ki: PID integral gain (default: 0.0)
            pid_kd: PID derivative gain (default: 0.05)
            pid_pf: PID feedforward gain (default: 0.3)
            max_tilt: Maximum tilt angle in degrees (default: 20.0)
            enable_spv4_viz: Enable SPV4 3D platform visualization (default: True)
            enable_forcen_viz: Enable Forcen ball position visualization (default: True)
            forcen_config: Dict of ForcenInterface parameters (optional)
            servo_config: Dict of ServoController parameters (optional)
        """
        self.platform_height = platform_height
        self.setpoint_x = setpoint_x
        self.setpoint_y = setpoint_y
        self.enable_spv4_viz = enable_spv4_viz
        self.enable_forcen_viz = enable_forcen_viz
        
        # Configure matplotlib for non-blocking visualization
        if enable_spv4_viz or enable_forcen_viz:
            plt.ion()
            plt.rcParams['figure.raise_window'] = False
        
        # Initialize Forcen sensor
        print("="*60)
        print("Initializing Forcen Sensor")
        print("="*60)
        
        forcen_defaults = {
            'com_port': forcen_port,
            'baud_rate': 115200,
            'sample_rate': 100,
            'arm_cog_x': 0.0,
            'arm_cog_y': 0.0,
            'arm_cog_z': 0.005,
            'arm_weight': 1.2,
            'arm_length_z': 0.009,
            'arm_weight_confidence': 0.7,
            'rotation_angle': 135,
            'scale_x': 1.1,
            'scale_y': 1.1,
            'flip_x': False,
            'flip_y': True,
        }
        if forcen_config:
            forcen_defaults.update(forcen_config)
        
        self.sensor = ForcenInterface(**forcen_defaults)
        
        # Initialize Stewart Platform kinematics
        print("\nInitializing Stewart Platform Kinematics")
        self.platform = StewartPlatform(l=25.9, l_base=10, l_link1=8, l_link2=10)
        
        # Initialize Servo Controller
        print("Initializing Servo Controller")
        servo_defaults = {
            'port': servo_port,
            'visualize': enable_spv4_viz,
            'flip_servo': [True, True, True],
            'neutral_angles': [122, 126, 126],
            'min_servo_angle': 78,
            'max_servo_angle': 155
        }
        if servo_config:
            servo_defaults.update(servo_config)
        
        self.controller = ServoController(**servo_defaults)
        self.controller.print_commanded_angle_limits()
        
        # Initialize PID controllers (one for each axis)
        self.pid_x = PIDController(kp=pid_kp, ki=pid_ki, kd=pid_kd, pf=pid_pf, output_limit=max_tilt)
        self.pid_y = PIDController(kp=pid_kp, ki=pid_ki, kd=pid_kd, pf=pid_pf, output_limit=max_tilt)
        
        # State tracking
        self.running = False
        self.last_angles = None
        
        # Visualization setup (will be initialized in run())
        self.fig = None
        self.ball = None
        self.coord_text = None
        
        # PID tuning increments
        self.kp_step = 0.005
        self.ki_step = 0.001
        self.kd_step = 0.005
        self.pf_step = 0.01
        self.gui = None
        self.gui_thread = None
        
        # Ball tracking for velocity estimation
        self.ball_position_history = []
        self.ball_time_history = []
        self.max_history = 5  # Keep last 5 samples for velocity estimation
    
    def calibrate_sensor(self, taring_time=5, save_to_eeprom=False):
        """
        Initialize and calibrate the Forcen sensor.
        
        Args:
            taring_time: Calibration time in seconds
            save_to_eeprom: Whether to save calibration to EEPROM
        
        Returns:
            True if successful, False otherwise
        """
        print("\n" + "="*60)
        print("Sensor Calibration")
        print("="*60)
        
        # Connect to sensor
        if not self.sensor.connect():
            print("Failed to connect to sensor!")
            return False
        
        # Initialize sensor
        if not self.sensor.initialize():
            print("Sensor initialization failed!")
            return False
        
        # Run calibration
        if not self.sensor.calibrate(taring_time=taring_time, save_to_eeprom=save_to_eeprom):
            print("Sensor calibration failed!")
            return False
        
        # Start data streaming
        print("\nStarting data stream...")
        if not self.sensor.start_streaming():
            print("Failed to start streaming!")
            return False
        
        print("✓ Sensor ready\n")
        return True
    
    def _handle_keyboard_input(self):
        """
        Handle keyboard input for PID tuning.
        Returns True to continue running, False to stop.
        """
        # ESC to stop
        if keyboard.is_pressed('esc'):
            return False
        
        # Kp adjustment
        if keyboard.is_pressed('q'):
            self.pid_x.kp += self.kp_step
            self.pid_y.kp += self.kp_step
            print(f"\n[Kp↑] X: {self.pid_x.kp:.3f}, Y: {self.pid_y.kp:.3f}")
            time.sleep(0.1)
        elif keyboard.is_pressed('a'):
            self.pid_x.kp = max(0, self.pid_x.kp - self.kp_step)
            self.pid_y.kp = max(0, self.pid_y.kp - self.kp_step)
            print(f"\n[Kp↓] X: {self.pid_x.kp:.3f}, Y: {self.pid_y.kp:.3f}")
            time.sleep(0.1)
        
        # Ki adjustment
        if keyboard.is_pressed('w'):
            self.pid_x.ki += self.ki_step
            self.pid_y.ki += self.ki_step
            print(f"\n[Ki↑] X: {self.pid_x.ki:.3f}, Y: {self.pid_y.ki:.3f}")
            time.sleep(0.1)
        elif keyboard.is_pressed('s'):
            self.pid_x.ki = max(0, self.pid_x.ki - self.ki_step)
            self.pid_y.ki = max(0, self.pid_y.ki - self.ki_step)
            print(f"\n[Ki↓] X: {self.pid_x.ki:.3f}, Y: {self.pid_y.ki:.3f}")
            time.sleep(0.1)
        
        # Kd adjustment
        if keyboard.is_pressed('e'):
            self.pid_x.kd += self.kd_step
            self.pid_y.kd += self.kd_step
            print(f"\n[Kd↑] X: {self.pid_x.kd:.3f}, Y: {self.pid_y.kd:.3f}")
            time.sleep(0.1)
        elif keyboard.is_pressed('d'):
            self.pid_x.kd = max(0, self.pid_x.kd - self.kd_step)
            self.pid_y.kd = max(0, self.pid_y.kd - self.kd_step)
            print(f"\n[Kd↓] X: {self.pid_x.kd:.3f}, Y: {self.pid_y.kd:.3f}")
            time.sleep(0.1)
        
        # Pf (feedforward) adjustment
        if keyboard.is_pressed('t'):
            self.pid_x.pf += self.pf_step
            self.pid_y.pf += self.pf_step
            print(f"\n[Pf↑] X: {self.pid_x.pf:.3f}, Y: {self.pid_y.pf:.3f}")
            time.sleep(0.1)
        elif keyboard.is_pressed('g'):
            self.pid_x.pf = max(0, self.pid_x.pf - self.pf_step)
            self.pid_y.pf = max(0, self.pid_y.pf - self.pf_step)
            print(f"\n[Pf↓] X: {self.pid_x.pf:.3f}, Y: {self.pid_y.pf:.3f}")
            time.sleep(0.1)
        
        # Reset integral and velocity history
        if keyboard.is_pressed('r'):
            self.pid_x.reset()
            self.pid_y.reset()
            self.ball_position_history.clear()
            self.ball_time_history.clear()
            print(f"\n[Reset] Integral terms & velocity history cleared")
            time.sleep(0.3)
        
        # Print PID values
        if keyboard.is_pressed('p'):
            print(f"\n{'='*50}")
            print(f"X-PID: Kp={self.pid_x.kp:.3f} Ki={self.pid_x.ki:.3f} Kd={self.pid_x.kd:.3f} Pf={self.pid_x.pf:.3f}")
            print(f"Y-PID: Kp={self.pid_y.kp:.3f} Ki={self.pid_y.ki:.3f} Kd={self.pid_y.kd:.3f} Pf={self.pid_y.pf:.3f}")
            print(f"{'='*50}\n")
            time.sleep(0.3)
        
        return True
    
    def run(self, update_rate=0.05, verbose=True):
        """
        Main control loop with keyboard-based PID tuning.
        
        Args:
            update_rate: Control loop update period in seconds (default: 0.05 = 20 Hz)
            verbose: Print debug information (default: True)
        
        Keyboard Controls:
            Q/A: Increase/Decrease Kp for both axes
            W/S: Increase/Decrease Ki for both axes
            E/D: Increase/Decrease Kd for both axes
            T/G: Increase/Decrease Pf (feedforward) for both axes
            R: Reset integral terms
            P: Print current PID values
            ESC: Stop controller
        """
        print("="*60)
        print("Starting Ball Balancing Control Loop")
        print("="*60)
        print(f"Setpoint: ({self.setpoint_x:.1f}, {self.setpoint_y:.1f}) mm")
        print(f"Platform Height: {self.platform_height:.1f} mm")
        print(f"Update Rate: {1/update_rate:.1f} Hz")
        print(f"SPV4 Visualization: {'Enabled' if self.enable_spv4_viz else 'Disabled'}")
        print(f"Forcen Visualization: {'Enabled' if self.enable_forcen_viz else 'Disabled'}")
        print("\nKeyboard Controls:")
        print("  Q/A: Kp +/-    W/S: Ki +/-    E/D: Kd +/-    T/G: Pf +/-")
        print("  R: Reset Integral    P: Print PID    ESC: Stop")
        print("\nNote: High-inertia ball - using velocity feedforward & filtered derivative")
        print("Press Ctrl+C or ESC to stop\n")
        
        # Set up Forcen visualization
        if self.enable_forcen_viz:
            self.fig, self.ball, self.coord_text = self.sensor.visualize_ball_position()
            if self.fig is None:
                print("Warning: Forcen visualization setup failed!")
        else:
            self.fig, self.ball, self.coord_text = None, None, None
        
        self.running = True
        frame_count = 0
        last_pid_print = time.time()
        
        # Performance tracking
        loop_times = []
        sensor_read_count = 0
        last_hz_update = time.time()
        actual_loop_hz = 0.0
        actual_sensor_hz = 0.0
        
        try:
            while self.running:
                loop_start = time.time()
                
                # Handle keyboard input for PID tuning
                if not self._handle_keyboard_input():
                    break
                
                # 1. Read sensor data
                data = self.sensor.read_data()
                
                if data:
                    sensor_read_count += 1
                    # 2. Get ball position
                    x, y = self.sensor.estimate_ball_position(data)
                    
                    if x is not None and y is not None:
                        # 3. Calculate errors
                        error_x = -self.setpoint_x - x
                        error_y = self.setpoint_y - y
                        
                        # 3.5. Estimate ball velocity for feedforward control
                        self.ball_position_history.append((x, y))
                        self.ball_time_history.append(loop_start)
                        
                        # Keep only recent history
                        if len(self.ball_position_history) > self.max_history:
                            self.ball_position_history.pop(0)
                            self.ball_time_history.pop(0)
                        
                        # Calculate velocity using linear regression for better noise rejection
                        vel_x, vel_y = 0.0, 0.0
                        if len(self.ball_position_history) >= 3:
                            times = np.array(self.ball_time_history)
                            times = times - times[0]  # Normalize to start at 0
                            x_positions = np.array([pos[0] for pos in self.ball_position_history])
                            y_positions = np.array([pos[1] for pos in self.ball_position_history])
                            
                            # Simple linear fit: velocity = slope
                            if len(times) > 1:
                                vel_x = np.polyfit(times, x_positions, 1)[0]  # mm/s
                                vel_y = np.polyfit(times, y_positions, 1)[0]  # mm/s
                        
                        # 4. Update PID controllers with velocity feedforward
                        # Note: In platform frame, tilting in +Y (roll) moves ball in +X
                        # So: error_x → roll correction, error_y → pitch correction
                        roll_correction = self.pid_x.update(error_x, loop_start, x, vel_x)
                        pitch_correction = self.pid_y.update(error_y, loop_start, y, vel_y)
                        
                        # 5. Calculate normal vector from pitch and roll
                        pitch_rad = pitch_correction * np.pi / 180
                        roll_rad = roll_correction * np.pi / 180
                        normal = [-np.sin(roll_rad), 
                                 np.sin(pitch_rad), 
                                 np.cos(pitch_rad) * np.cos(roll_rad)]
                        
                        try:
                            # 6. Compute inverse kinematics
                            ik = self.platform.inverse_kinematics(normal, [0, 0, self.platform_height])
                            angles = [ik['theta_11'], ik['theta_21'], ik['theta_31']]
                            
                            # 7. Send angles to servos (only if changed)
                            if angles != self.last_angles:
                                self.controller.send_angles(angles)
                                self.last_angles = angles
                            
                            # 8. Update Forcen visualization
                            if self.fig is not None:
                                self.ball.center = (x, y)
                                weight_g = self.sensor.get_weight(data)
                                
                                # Update coordinate text
                                self.coord_text.set_text(
                                    f'X: {x:7.1f} mm\n'
                                    f'Y: {y:7.1f} mm\n'
                                    f'Weight: {weight_g:5.1f} g\n'
                                    f'Pitch: {pitch_correction:+5.1f}°\n'
                                    f'Roll: {roll_correction:+5.1f}°'
                                )
                                
                                # Color based on distance from setpoint
                                distance = np.sqrt(error_x**2 + error_y**2)
                                if weight_g < 300:
                                    self.ball.set_color('grey')
                                    self.ball.set_alpha(0.3)
                                elif distance < 10:  # Within 10mm of setpoint
                                    self.ball.set_color('green')
                                    self.ball.set_alpha(0.8)
                                elif distance < 30:
                                    self.ball.set_color('yellow')
                                    self.ball.set_alpha(0.8)
                                else:
                                    self.ball.set_color('orange')
                                    self.ball.set_alpha(0.8)
                                
                                # Update plot (throttled)
                                if frame_count % 5 == 0:
                                    self.fig.canvas.draw_idle()
                                    self.fig.canvas.flush_events()
                            
                            # Print status
                            if verbose:
                                weight_g = self.sensor.get_weight(data)
                                print(f"\rLoop:{actual_loop_hz:4.1f}Hz Sensor:{actual_sensor_hz:4.1f}Hz  "
                                      f"Ball:({x:6.1f},{y:6.1f})mm  "
                                      f"Err:({error_x:+5.1f},{error_y:+5.1f})  "
                                      f"Tilt:(P:{pitch_correction:+4.1f}° R:{roll_correction:+4.1f}°)  "
                                      f"θ:[{angles[0]:4.1f},{angles[1]:4.1f},{angles[2]:4.1f}]°  "
                                      f"W:{weight_g:5.1f}g", 
                                      end='', flush=True)
                        
                        except Exception as e:
                            if verbose:
                                print(f"\rIK Error: {e}                    ", end='', flush=True)
                    else:
                        # No valid position - hold current position or go to neutral
                        if verbose:
                            weight_g = self.sensor.get_weight(data)
                            print(f"\r[No position detected] Weight: {weight_g:5.1f}g                    ", 
                                  end='', flush=True)
                
                # Frame timing
                frame_count += 1
                loop_end = time.time()
                elapsed = loop_end - loop_start
                
                # Track loop timing
                loop_times.append(elapsed)
                if len(loop_times) > 50:  # Keep last 50 samples
                    loop_times.pop(0)
                
                # Update Hz calculations every second
                if loop_end - last_hz_update >= 1.0:
                    if len(loop_times) > 0:
                        avg_loop_time = sum(loop_times) / len(loop_times)
                        actual_loop_hz = 1.0 / avg_loop_time if avg_loop_time > 0 else 0.0
                    actual_sensor_hz = sensor_read_count / (loop_end - last_hz_update)
                    sensor_read_count = 0
                    last_hz_update = loop_end
                
                sleep_time = max(0, update_rate - elapsed)
                time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print("\n\nStopping controller...")
        
        finally:
            self.stop()
            
    def go_home(self):
        neutral_ik = self.platform.inverse_kinematics([0.01, 0.01, 1], [0, 0, self.platform_height])
        neutral_angles = [neutral_ik['theta_11'], neutral_ik['theta_21'], neutral_ik['theta_31']]
        self.controller.send_angles(neutral_angles)
    
    def stop(self):
        """Stop the controller and clean up resources."""
        self.running = False
        
        # Return to neutral position
        print("\nReturning to neutral position...")
        try:
            self.go_home()
            time.sleep(0.5)
        except:
            pass
        # Close connections
        self.controller.close()
        self.sensor.disconnect()
        
        # Close plots
        plt.ioff()
        plt.close('all')
        
        print("Controller stopped.")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()


# ========== Main Script ==========
if __name__ == "__main__":
    print("="*60)
    print("Ball Balancing Stewart Platform Controller")
    print("="*60)
    print()

    # Create controller with context manager
    with BallBalancingController(
        forcen_port="COM7",
        servo_port="COM8",
        platform_height=10.6,
        setpoint_x=0.0,
        setpoint_y=0.0,
        pid_kp=0.055,      # Increased Kp for heavier ball
        pid_ki=0.01,     # Small integral to handle steady-state error
        pid_kd=0.03,      # Increased Kd for damping (counters inertia)
        pid_pf=0.0,       # Feedforward gain for velocity compensation
        max_tilt=14.0,
        enable_spv4_viz=True,   # Disable 3D platform visualization
        enable_forcen_viz=True   # Keep ball position visualization
    ) as controller:
        
        # Move platform to home position before calibration
        print("Moving platform to home position...")
        time.sleep(2)  # Wait for platform to settle and servos to reach position
        controller.go_home()
        time.sleep(1)  # Wait for platform to settle and servos to reach position        
        
        # Calibrate sensor
        if not controller.calibrate_sensor(taring_time=5, save_to_eeprom=False):
            print("Calibration failed! Exiting...")
            exit(1)
        
        # Run control loop with keyboard tuning
        controller.run(update_rate=0.01, verbose=True)
    
    print("\nExiting...")
