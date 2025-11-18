import cv2
import numpy as np
import json
from datetime import datetime

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
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        mask_platform = np.zeros((self.FRAME_H, self.FRAME_W), dtype=np.uint8)
        cv2.circle(mask_platform, self.center_point, int(self.pixel_radius), 255, -1)
        masked_blur = cv2.bitwise_and(blurred, blurred, mask=mask_platform)
        
        _, thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest)

        if radius < 5 or radius > 120:
            return None

        if self.center_point and self.pixel_to_meter_ratio:
            cx, cy = self.center_point
            dx_m = (x - cx) * self.pixel_to_meter_ratio
            dy_m = -(y - cy) * self.pixel_to_meter_ratio
        else:
            dx_m, dy_m = None, None  # Not yet calibrated

        return dx_m, dy_m, (int(x), int(y)), int(radius)

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

                cv2.circle(overlay, (x, y), radius, (0, 255, 255), 2)
                cv2.circle(overlay, (x, y), 3, (0, 255, 255), -1)

                # Only show metric coordinates after calibration
                if dx_m is not None:
                    cv2.putText(overlay, f"x={dx_m:.4f}m y={dy_m:.4f}m",
                                (x + 20, y + 20),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0,255,255), 1)

        return overlay

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
