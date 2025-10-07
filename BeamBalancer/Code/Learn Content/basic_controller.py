import cv2
import numpy as np
import json
import serial
import time
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from threading import Thread
import queue
from ball_detection import detect_ball_x

class BasicPIDController:
    def __init__(self, config_file="config.json"):
        """Initialize controller, load config, set defaults and queues."""
        # Load experiment and hardware config from JSON file
        with open(config_file, 'r') as f:
            self.config = json.load(f)
        # PID gains (controlled by sliders in GUI)
        self.Kp = 2.5
        self.Ki = 0.5
        self.Kd = 1.0
        # Scale factor for converting from pixels to meters
        self.scale_factor = self.config['calibration']['pixel_to_meter_ratio'] * self.config['camera']['frame_width'] / 2
        # Servo port name and center angle
        self.servo_port = self.config['servo']['port']
        self.neutral_angle = self.config['servo']['neutral_angle']
        self.servo = None
        # Controller-internal state
        self.setpoint = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        # Data logs for plotting results
        self.time_log = []
        self.position_log = []
        self.setpoint_log = []
        self.control_log = []
        self.start_time = None
        # Thread-safe queue for most recent ball position measurement
        self.position_queue = queue.Queue(maxsize=1)
        self.running = False    # Main run flag for clean shutdown
        # Servo rate limiting
        self.last_servo_send_time = 0
        self.servo_send_interval = 0.05  # Send servo commands every 50ms (20 Hz max)

    def connect_servo(self):
        """Try to open serial connection to servo, return True if success."""
        try:
            # Add timeout to prevent indefinite blocking
            self.servo = serial.Serial(
                self.servo_port, 
                115200,
                timeout=0.1,           # Read timeout
                write_timeout=0.1      # Write timeout to prevent blocking
            )
            time.sleep(2)
            print(f"[SERVO] Connected with timeouts (write: 0.1s, send rate: {1/self.servo_send_interval:.0f} Hz)")
            return True
        except Exception as e:
            print(f"[SERVO] Failed: {e}")
            return False

    def send_servo_angle(self, angle):
        """Send angle command to servo motor (clipped for safety with rate limiting)."""
        if self.servo:
            # Rate limiting: only send if enough time has passed
            current_time = time.time()
            # if current_time - self.last_servo_send_time < self.servo_send_interval:
                # Skip this send - too soon after last command
                # return
            
            servo_angle = self.neutral_angle + angle
            servo_angle = int(np.clip(servo_angle, 0, 30))
            try:
                # Track write timing to identify blocking
                write_start = time.time()
                self.servo.write(bytes([servo_angle]))
                write_time = time.time() - write_start
                
                # Update last send time only after successful send
                self.last_servo_send_time = current_time
                
                # Warn if write takes too long (blocking the control thread)
                if write_time > 0.05:
                    print(f"[SERVO] WARNING: Write took {write_time:.3f}s - THIS IS BLOCKING THE CONTROL THREAD!")
            except serial.SerialTimeoutException:
                print(f"[SERVO] Write timeout - servo not responding")
            except Exception as e:
                print(f"[SERVO] Send failed: {e}")

    def update_pid(self, position, dt=0.033):
        """Perform PID calculation and return control output."""
        error = self.setpoint - position  # Compute error
        print(f"Error: {error} | Position: {position} | Setpoint: {self.setpoint}")
        
        error = error * 100  # Scale error for easier tuning (if needed)
        # Proportional term
        P = self.Kp * error
        # Integral term accumulation
        if error < 10:  # Prevent integral windup for large errors
            self.integral += error * dt
        I = self.Ki * self.integral
        # Derivative term calculation
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative
        self.prev_error = error
        # PID output (limit to safe beam range)
        output = P + I + D
        output = np.clip(output, -15, 15)
        print(error)
        return output

    def camera_thread(self):
        """Dedicated thread for video capture and ball detection."""
        cap = cv2.VideoCapture(self.config['camera']['index'], cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        frame_count = 0
        detection_count = 0
        last_stats_time = time.time()
        
        while self.running:
            ret, frame = cap.read()
            if not ret:
                print("[CAMERA] Failed to read frame")
                time.sleep(0.01)  # Brief pause before retrying
                continue
            
            frame = cv2.resize(frame, (320, 240))
            frame_count += 1
            
            # Detect ball position in frame
            found, x_normalized, vis_frame = detect_ball_x(frame)
            if found:
                detection_count += 1
                # Convert normalized to meters using scale
                position_m = x_normalized * self.scale_factor
                # Always keep latest measurement only
                try:
                    if self.position_queue.full():
                        self.position_queue.get_nowait()
                    self.position_queue.put_nowait(position_m)
                except Exception as e:
                    print(f"[CAMERA] Queue error: {e}")
            
            # Print detection statistics every 5 seconds
            now = time.time()
            if now - last_stats_time > 5.0:
                detection_rate = (detection_count / frame_count * 100) if frame_count > 0 else 0
                print(f"[CAMERA] Stats: {frame_count} frames, {detection_count} detections ({detection_rate:.1f}%)")
                frame_count = 0
                detection_count = 0
                last_stats_time = now
            
            # Show processed video with overlays
            cv2.imshow("Ball Tracking", vis_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC exits
                self.running = False
                break
        
        cap.release()
        cv2.destroyAllWindows()

    def control_thread(self):
        """Runs PID control loop in parallel with GUI and camera."""
        if not self.connect_servo():
            print("[ERROR] No servo - running in simulation mode")
        self.start_time = time.time()
        last_time = time.time()
        last_position = None
        no_data_count = 0
        loop_count = 0
        
        print("[CONTROL] Thread started - monitoring for blocking operations...")
        
        while self.running:
            loop_start = time.time()
            loop_count += 1
            
            try:
                # Wait for latest ball position from camera
                get_start = time.time()
                position = self.position_queue.get(timeout=0.1)
                get_time = time.time() - get_start
                
                if get_time > 0.15:  # Should never take more than timeout + small overhead
                    print(f"[CONTROL] WARNING: queue.get() took {get_time:.3f}s (expected ~0.1s max)")
                
                # Reset no-data counter when we get data
                no_data_count = 0
                
                # Print time since last position measurement
                now = time.time()
                elapsed = now - last_time
                if elapsed > 1.0:
                    print(f"[CONTROL] WARNING: Large gap detected: {elapsed:.3f}s since last position")
                last_time = now
                last_position = position
                
                # Compute control output using PID
                pid_start = time.time()
                control_output = self.update_pid(position)
                pid_time = time.time() - pid_start
                if pid_time > 0.1:
                    print(f"[CONTROL] WARNING: update_pid() took {pid_time:.3f}s - CHECK FOR BLOCKING PRINT STATEMENTS!")
                
                # Send control command to servo (real or simulated)
                servo_start = time.time()
                self.send_servo_angle(control_output)
                servo_time = time.time() - servo_start
                if servo_time > 0.2:
                    print(f"[CONTROL] WARNING: send_servo_angle() took {servo_time:.3f}s - SERIAL WRITE IS BLOCKING!")
                
                # Log results for plotting
                current_time = time.time() - self.start_time
                self.time_log.append(current_time)
                self.position_log.append(position)
                self.setpoint_log.append(self.setpoint)
                self.control_log.append(control_output)
                
                print(f"[CONTROL] Time since last position: {elapsed:.3f}s")
                print(f"[CONTROL] Thread running] Pos: {position:.3f}m, Output: {control_output:.1f}°")
                
            except queue.Empty:
                no_data_count += 1
                if no_data_count % 20 == 1:  # Print every ~2 seconds (20 * 0.1s)
                    print(f"[CONTROL] No ball detected for {no_data_count * 0.1:.1f}s - waiting for camera...")
                continue
                
            except Exception as e:
                print(f"[CONTROL] Error: {e}")
                import traceback
                traceback.print_exc()
                break
            
            # Check total loop time
            loop_time = time.time() - loop_start
            if loop_time > 0.5:
                print(f"[CONTROL] WARNING: Control loop iteration #{loop_count} took {loop_time:.3f}s - THIS IS THE BLOCKING ISSUE!")
        
        if self.servo:
            # Return to neutral on exit
            self.send_servo_angle(0)
            self.servo.close()

    def create_gui(self):
        """Build Tkinter GUI with large sliders and labeled controls."""
        self.root = tk.Tk()
        self.root.title("Basic PID Controller")
        self.root.geometry("520x400")

        # Title label
        ttk.Label(self.root, text="PID Gains", font=("Arial", 18, "bold")).pack(pady=10)

        # Kp slider
        ttk.Label(self.root, text="Kp (Proportional)", font=("Arial", 12)).pack()
        self.kp_var = tk.DoubleVar(value=self.Kp)
        kp_slider = ttk.Scale(self.root, from_=0, to=2, variable=self.kp_var,
                              orient=tk.HORIZONTAL, length=500)
        kp_slider.pack(pady=5)
        self.kp_label = ttk.Label(self.root, text=f"Kp: {self.Kp:.1f}", font=("Arial", 11))
        self.kp_label.pack()

        # Ki slider
        ttk.Label(self.root, text="Ki (Integral)", font=("Arial", 12)).pack()
        self.ki_var = tk.DoubleVar(value=self.Ki)
        ki_slider = ttk.Scale(self.root, from_=0, to=10, variable=self.ki_var,
                              orient=tk.HORIZONTAL, length=500)
        ki_slider.pack(pady=5)
        self.ki_label = ttk.Label(self.root, text=f"Ki: {self.Ki:.1f}", font=("Arial", 11))
        self.ki_label.pack()

        # Kd slider
        ttk.Label(self.root, text="Kd (Derivative)", font=("Arial", 12)).pack()
        self.kd_var = tk.DoubleVar(value=self.Kd)
        kd_slider = ttk.Scale(self.root, from_=0, to=20, variable=self.kd_var,
                              orient=tk.HORIZONTAL, length=500)
        kd_slider.pack(pady=5)
        self.kd_label = ttk.Label(self.root, text=f"Kd: {self.Kd:.1f}", font=("Arial", 11))
        self.kd_label.pack()

        # Setpoint slider
        ttk.Label(self.root, text="Setpoint (meters)", font=("Arial", 12)).pack()
        pos_min = self.config['calibration']['position_min_m']
        pos_max = self.config['calibration']['position_max_m']
        self.setpoint_var = tk.DoubleVar(value=self.setpoint)
        setpoint_slider = ttk.Scale(self.root, from_=pos_min, to=pos_max,
                                   variable=self.setpoint_var,
                                   orient=tk.HORIZONTAL, length=500)
        setpoint_slider.pack(pady=5)
        self.setpoint_label = ttk.Label(self.root, text=f"Setpoint: {self.setpoint:.3f}m", font=("Arial", 11))
        self.setpoint_label.pack()

        # Button group for actions
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=20)
        ttk.Button(button_frame, text="Reset Integral",
                   command=self.reset_integral).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Plot Results",
                   command=self.plot_results).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Stop",
                   command=self.stop).pack(side=tk.LEFT, padx=5)

        # Schedule periodic GUI update
        self.update_gui()

    def update_gui(self):
        """Reflect latest values from sliders into program and update display."""
        if self.running:
            # PID parameters
            self.Kp = self.kp_var.get()
            self.Ki = self.ki_var.get()
            self.Kd = self.kd_var.get()
            self.setpoint = self.setpoint_var.get()
            # Update displayed values
            self.kp_label.config(text=f"Kp: {self.Kp:.1f}")
            self.ki_label.config(text=f"Ki: {self.Ki:.1f}")
            self.kd_label.config(text=f"Kd: {self.Kd:.1f}")
            self.setpoint_label.config(text=f"Setpoint: {self.setpoint:.3f}m")
            # Call again after 50 ms (if not stopped)
            self.root.after(50, self.update_gui)

    def reset_integral(self):
        """Clear integral error in PID (button handler)."""
        self.integral = 0.0
        print("[RESET] Integral term reset")

    def plot_results(self):
        """Show matplotlib plots of position and control logs."""
        if not self.time_log:
            print("[PLOT] No data to plot")
            return
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        # Ball position trace
        ax1.plot(self.time_log, self.position_log, label="Ball Position", linewidth=2)
        ax1.plot(self.time_log, self.setpoint_log, label="Setpoint",
                 linestyle="--", linewidth=2)
        ax1.set_ylabel("Position (m)")
        ax1.set_title(f"Basic PID Control (Kp={self.Kp:.1f}, Ki={self.Ki:.1f}, Kd={self.Kd:.1f})")
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        # Control output trace
        ax2.plot(self.time_log, self.control_log, label="Control Output",
                 color="orange", linewidth=2)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Beam Angle (degrees)")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()

    def stop(self):
        """Stop everything and clean up threads and GUI."""
        self.running = False
        # Try to safely close all windows/resources
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        """Entry point: starts threads, launches GUI mainloop."""
        print("[INFO] Starting Basic PID Controller")
        print("Use sliders to tune PID gains in real-time")
        print("Close camera window or click Stop to exit")
        self.running = True

        # Start camera and control threads, mark as daemon for exit
        cam_thread = Thread(target=self.camera_thread, daemon=True)
        ctrl_thread = Thread(target=self.control_thread, daemon=True)
        cam_thread.start()
        ctrl_thread.start()

        # Build and run GUI in main thread
        self.create_gui()
        self.root.mainloop()

        # After GUI ends, stop everything
        self.running = False
        print("[INFO] Controller stopped")

if __name__ == "__main__":
    try:
        controller = BasicPIDController()
        controller.run()
    except FileNotFoundError:
        print("[ERROR] config.json not found. Run simple_autocal.py first.")
    except Exception as e:
        print(f"[ERROR] {e}")
