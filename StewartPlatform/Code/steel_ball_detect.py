import cv2
import numpy as np

# ===============================
# Hough Circle Test Script
# ===============================

CAM_INDEX = 0
FRAME_W, FRAME_H = 640, 480

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

print("[INFO] Running Hough Circle Test...")
print("[INFO] Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Rotate (same as your main script)
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    # ----------------------------------------
    # 1. Grayscale + Blur
    # ----------------------------------------
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (13, 13), 2)

    # ----------------------------------------
    # 2. Canny Edges (for visualization only)
    # ----------------------------------------
    edges = cv2.Canny(blurred, 50, 150)

    # ----------------------------------------
    # 3. Hough Circle Detection
    # ----------------------------------------
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
    
    # circles = cv2.HoughCircles(
    #     blurred,
    #     cv2.HOUGH_GRADIENT,
    #     dp=1.4,
    #     minDist=40,
    #     param1=135,   # Canny high threshold
    #     param2=40,    # Lower = more sensitive
    #     minRadius=8,
    #     maxRadius=40
    # )

    display = frame.copy()

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")

        # Choose the largest circle (steel ball is biggest bright object)
        circles = sorted(circles, key=lambda c: c[2], reverse=True)
        x, y, r = circles[0]

        cv2.circle(display, (x, y), r, (0, 255, 255), 2)
        cv2.circle(display, (x, y), 3, (0, 255, 255), -1)

        cv2.putText(display, f"Ball Center: ({x}, {y})  r={r}",
                    (10, 450), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)

    # Show all debug views
    cv2.imshow("Raw", frame)
    cv2.imshow("Blurred", blurred)
    cv2.imshow("Edges", edges)
    cv2.imshow("Hough Detection", display)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()