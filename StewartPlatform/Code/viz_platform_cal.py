import cv2
import numpy as np
import json
from datetime import datetime
import matplotlib.pyplot as plt
import time
import struct
import os

class SimplePlatformCalibrator:
    """Interactive calibration system for circular ball-balancing platform."""

    def __init__(self):
        # Physical system parameters
        self.PLATFORM_RADIUS_M = 0.15  # known real-world radius in meters

        # Camera configuration
        self.CAM_INDEX = 0
        self.FRAME_W, self.FRAME_H = 640, 480

        # Calibration state tracking
        self.current_frame = None
        self.phase = "geometry"  # start directly in geometry phase

        # Geometry calibration data
        self.left_point = None  
        self.right_point = None 
        self.up_point = None  
        self.down_point = None 
        self.center_point = None

        self.pixel_to_meter_ratio = None
        self.pixel_radius = None  

        # --- Live plot state ---
        self.enable_live_plot = True
        self.plot_initialized = False
        self.plot_history_sec = 10  # keep last ? second of data on graph


        self.plot_times = []
        self.plot_dx = []
        self.plot_dy = []
        self.plot_speed = []
        self.plot_accel = []

        self.last_dx = None
        self.last_dy = None
        self.last_time = None
        self.last_speed = None

        self.plot_update_interval = 1.0   # update per ? sec, change if your openCV is dying
        self.last_plot_update = 0

        #forceN sensor displays
        self.FS_FMT = '8d'                 # x, y, weight, pitch, roll, err_x, err_y, t
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.FS_FILE = os.path.join(script_dir, "ball_state.dat")   # written by controller, assume in same folder as this script
        self.fs_last_read = 0.0
        self.fs_update_interval = 1.0 / 60.0  # read at most 60 Hz
        self.fs_last_position = (None, None)  # (x_mm, y_mm) in sensor frame


        print("[INIT] Platform Calibrator initialized.")

    # ============================
    # Mouse Handling
    # ============================

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.phase == "geometry":

            if self.left_point is None:
                self.left_point = (x, y)
                print("[GEO] Leftmost point selected.")
            elif self.right_point is None:
                self.right_point = (x, y)
                print("[GEO] Rightmost point selected.")
            elif self.up_point is None:
                self.up_point = (x, y)
                print("[GEO] Topmost point selected.")
            elif self.down_point is None:
                self.down_point = (x, y)
                print("[GEO] Bottommost point selected.")
                self.calculate_geometry()

    # ============================
    # Geometry Calibration
    # ============================

    def calculate_geometry(self):
        if None in (self.left_point, self.right_point, self.up_point, self.down_point):
            print("[GEO] Not enough points selected.")
            return

        cx = (self.left_point[0] + self.right_point[0]) / 2
        cy = (self.up_point[1] + self.down_point[1]) / 2
        self.center_point = (int(cx), int(cy))

        horizontal_d = abs(self.right_point[0] - self.left_point[0])
        vertical_d = abs(self.down_point[1] - self.up_point[1])
        self.pixel_radius = (horizontal_d + vertical_d) / 4

        self.pixel_to_meter_ratio = self.PLATFORM_RADIUS_M / self.pixel_radius

        print(f"[GEO] Center point: {self.center_point}")
        print(f"[GEO] Pixel radius: {self.pixel_radius:.2f}")
        print(f"[GEO] Pixel-to-meter ratio: {self.pixel_to_meter_ratio:.6f}")

        self.phase = "preview"

    # ============================
    # Ball Detection (brightness)
    # ============================

    def detect_ball_position(self, frame):

        if self.center_point is None or not self.pixel_to_meter_ratio:
            # Still allow detection BEFORE calibration (return only pixels)
            pass

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (13, 13), 2)
        edges = cv2.Canny(blurred, 50, 150)
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,          # slightly lower → more robust
            minDist=40,

            param1=80,       # SOFTER CANNY → more consistent edges
            param2=16,       # MUCH LOWER → stable detection (big change)

            minRadius=10,
            maxRadius=32
        )

        if circles is None:
            return None
        
        circles = np.round(circles[0, :]).astype("int")
        circles = sorted(circles, key=lambda c: c[2], reverse=True)  # largest
        x, y, r = circles[0]

        if r < 5 or r > 80:
            return None


        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        # mask_platform = np.zeros((self.FRAME_H, self.FRAME_W), dtype=np.uint8)
        # cv2.circle(mask_platform, self.center_point, int(self.pixel_radius), 255, -1)
        # masked_blur = cv2.bitwise_and(blurred, blurred, mask=mask_platform)
        
        # _, thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV)

        # contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
        #                                cv2.CHAIN_APPROX_SIMPLE)
        # if not contours:
        #     return None

        # largest = max(contours, key=cv2.contourArea)
        # ((x, y), radius) = cv2.minEnclosingCircle(largest)

        # if radius < 5 or radius > 120:
        #     return None

        if self.center_point and self.pixel_to_meter_ratio:
            cx, cy = self.center_point
            dx_m = (x - cx) * self.pixel_to_meter_ratio
            dy_m = -(y - cy) * self.pixel_to_meter_ratio
        else:
            dx_m, dy_m = None, None  # Not yet calibrated

        return dx_m, dy_m, (int(x), int(y)), int(r)

    # ============================
    # Drawing Overlay
    # ============================

    def draw_overlay(self, frame):
        overlay = frame.copy()

        phase_text = {
            "geometry": "Click Left Right Up Down of platform in order",
            "preview": "Calibration complete! Check Ball Position and Press 's' to save."
        }

        cv2.putText(overlay, f"Phase: {self.phase}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(overlay, phase_text[self.phase], (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        # Draw geometry points
        if self.left_point:
            cv2.circle(overlay, self.left_point, 6, (255, 0, 0), -1)
        if self.right_point:
            cv2.circle(overlay, self.right_point, 6, (0, 255, 0), -1)
        if self.up_point:
            cv2.circle(overlay, self.up_point, 6, (0, 0, 255), -1)
        if self.down_point:
            cv2.circle(overlay, self.down_point, 6, (255, 255, 0), -1)

        if self.center_point and self.pixel_radius:
            cv2.circle(overlay, self.center_point, 5, (0,255,255), -1)
            cv2.circle(overlay, self.center_point, int(self.pixel_radius),
                       (255,0,255), 2)

        # ===================================================
        # ALWAYS draw detected ball (NEW BEHAVIOR)
        # ===================================================
        if self.phase == "preview":
            pos = self.detect_ball_position(frame)
            if pos is not None:
                dx_m, dy_m, (x, y), radius = pos

                if dx_m is not None and dy_m is not None:
                    self.update_live_plot(dx_m, dy_m)

                cv2.circle(overlay, (x, y), radius, (0, 255, 255), 2)
                cv2.circle(overlay, (x, y), 3, (0, 255, 255), -1)

                # Only show metric coordinates after calibration
                if dx_m is not None:

                    cv2.putText(overlay, f"x={dx_m:.4f}m y={dy_m:.4f}m",
                                (x + 20, y + 20),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0,255,255), 1)
                    
                self.write_cv_dat(dx_m, dy_m, x, y, radius)

                fs_x_mm, fs_y_mm = self.read_force_sensor_ball()

                if (fs_x_mm is not None and fs_y_mm is not None and
                    self.center_point is not None and
                    self.pixel_to_meter_ratio):

                    # Forcen gives mm; convert to meters
                    fs_x_m = fs_x_mm / 1000.0
                    fs_y_m = fs_y_mm / 1000.0

                    cx, cy = self.center_point

                    # Convert m → pixels using same calibration as CV
                    fx_px = int(cx + fs_x_m / self.pixel_to_meter_ratio)
                    fy_px = int(cy - fs_y_m / self.pixel_to_meter_ratio)

                    # Draw force-sensor ball as bright green, same radius as CV
                    cv2.circle(overlay, (fx_px, fy_px), radius, (0, 255, 0), 2)
                    cv2.circle(overlay, (fx_px, fy_px), 3, (0, 255, 0), -1)

                    cv2.putText(overlay, "force",
                                (fx_px + 20, fy_px),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 1)

        return overlay
    # ============================
    # live dx dy v a plotting update
    # ============================
    
    def update_live_plot(self, dx, dy):
        if not self.enable_live_plot:
            return

        now = time.time()
        if now - self.last_plot_update < self.plot_update_interval:
            return

        self.last_plot_update = now

        # -----------------------------
        # Compute velocity (1st derivative)
        # -----------------------------
        if self.last_time is not None:
            dt = now - self.last_time
            if dt > 0:
                vx = (dx - self.last_dx) / dt
                vy = (dy - self.last_dy) / dt
                speed = np.sqrt(vx*vx + vy*vy)
            else:
                speed = 0.0
        else:
            speed = 0.0

        # -----------------------------
        # Compute acceleration (2nd derivative)
        # -----------------------------
        if self.last_speed is not None and self.last_time is not None:
            dt = now - self.last_time
            if dt > 0:
                accel = (speed - self.last_speed) / dt
            else:
                accel = 0.0
        else:
            accel = 0.0

        # Store previous step
        self.last_time = now
        self.last_dx = dx
        self.last_dy = dy
        self.last_speed = speed

        # -----------------------------
        # Append new samples
        # -----------------------------
        self.plot_times.append(now)
        self.plot_dx.append(dx)
        self.plot_dy.append(dy)
        self.plot_speed.append(speed)
        self.plot_accel.append(accel)

        # Trim history > N seconds
        while self.plot_times and (now - self.plot_times[0] > self.plot_history_sec):
            self.plot_times.pop(0)
            self.plot_dx.pop(0)
            self.plot_dy.pop(0)
            self.plot_speed.pop(0)
            self.plot_accel.pop(0)

        # Normalize time axis to t=0
        t0 = self.plot_times[0]
        t_vals = [t - t0 for t in self.plot_times]

        # -----------------------------
        # Create the figure once
        # -----------------------------
        if not self.plot_initialized:
            plt.ion()
            self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(
                4, 1, figsize=(8, 10), sharex=True
            )

            # dx
            self.line_dx, = self.ax1.plot([], [], color="cyan")
            self.ax1.set_ylabel("dx (m)")
            self.ax1.grid(True)

            # dy
            self.line_dy, = self.ax2.plot([], [], color="yellow")
            self.ax2.set_ylabel("dy (m)")
            self.ax2.grid(True)

            # velocity
            self.line_speed, = self.ax3.plot([], [], color="lime")
            self.ax3.set_ylabel("speed (m/s)")
            self.ax3.grid(True)

            # acceleration
            self.line_accel, = self.ax4.plot([], [], color="magenta")
            self.ax4.set_ylabel("accel (m/s²)")
            self.ax4.set_xlabel("time (s)")
            self.ax4.grid(True)

            self.fig.suptitle("Real-Time Ball Motion Analytics")
            self.plot_initialized = True

        # -----------------------------
        # Update lines
        # -----------------------------
        self.line_dx.set_data(t_vals, self.plot_dx)
        self.line_dy.set_data(t_vals, self.plot_dy)
        self.line_speed.set_data(t_vals, self.plot_speed)
        self.line_accel.set_data(t_vals, self.plot_accel)

        # Autoscale
        for ax in (self.ax1, self.ax2, self.ax3, self.ax4):
            ax.relim()
            ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def write_cv_dat(self, dx, dy, px, py, r):
        """Write CV data to cv_ball_state.dat as 6 doubles."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_dir, "cv_ball_state.dat")
        try:
            timestamp = time.time()
            data = struct.pack(
                '6d',
                dx if dx is not None else 0.0,
                dy if dy is not None else 0.0,
                float(px) if px is not None else 0.0,
                float(py) if py is not None else 0.0,
                float(r) if r is not None else 0.0,
                timestamp
            )
            with open(file_path, "wb") as f:
                f.write(data)
        except Exception as e:
            print(f"[CV-DAT ERROR] {e}")

    def read_force_sensor_ball(self):
        """Read latest ball position from ball_state.dat (Forcen), return (x_mm, y_mm)."""
        now = time.time()
        # Rate limit to avoid hammering the filesystem
        if now - self.fs_last_read < self.fs_update_interval:
            return self.fs_last_position

        self.fs_last_read = now

        try:
            size = struct.calcsize(self.FS_FMT)
            with open(self.FS_FILE, "rb") as f:
                raw = f.read(size)
                if len(raw) == size:
                    x_mm, y_mm, *_ = struct.unpack(self.FS_FMT, raw)
                    self.fs_last_position = (x_mm, y_mm)
        except OSError:
            # File missing / locked → keep last good position
            pass

        return self.fs_last_position

    # ============================
    # Save Configuration
    # ============================

    def save_config(self):
        config = {
            "timestamp": datetime.now().isoformat(),
            "platform_radius_m": float(self.PLATFORM_RADIUS_M),
            "camera": {
                "index": int(self.CAM_INDEX),
                "frame_width": int(self.FRAME_W),
                "frame_height": int(self.FRAME_H)
            },
            "ball_detection": {
                "mode": "brightness_threshold"
            },
            "calibration": {
                "pixel_to_meter_ratio":
                    float(self.pixel_to_meter_ratio) if self.pixel_to_meter_ratio else None,
                "center_point_px": self.center_point,
                "radius_px": float(self.pixel_radius) if self.pixel_radius else None,
                "left_point_px": self.left_point,
                "right_point_px": self.right_point,
                "up_point_px": self.up_point,
                "down_point_px": self.down_point
            }
        }

        with open("platform_config.json", "w") as f:
            json.dump(config, f, indent=2)

        print("[SAVE] Configuration saved to platform_config.json")

    # ============================
    # Main Loop
    # ============================

    def run(self):
        self.cap = cv2.VideoCapture(self.CAM_INDEX, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_H)

        cv2.namedWindow("Platform Calibration")
        cv2.setMouseCallback("Platform Calibration", self.mouse_callback)

        print("[INFO] Phase 2: Click Left Right Up Down of platform")
        print("[INFO] Press 's' to save, 'q' to quit.")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame = cv2.rotate(frame, cv2.ROTATE_180)
            self.current_frame = frame

            display = self.draw_overlay(frame)
            cv2.imshow("Platform Calibration", display)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

            if key == ord('s') and self.phase == "preview":
                self.save_config()
                break

        self.cap.release()
        cv2.destroyAllWindows()


# ============================
# Entry Point
# ============================

if __name__ == "__main__":
    calibrator = SimplePlatformCalibrator()
    calibrator.run()
