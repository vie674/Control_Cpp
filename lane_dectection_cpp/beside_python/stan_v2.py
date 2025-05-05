import cv2
import numpy as np
import math
import serial

# Initialize video and serial
vidcap = cv2.VideoCapture(0)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Load calibration
camera_matrix = np.loadtxt('cameraMatrix.txt', delimiter=',')
distortion_coeffs = np.loadtxt('cameraDistortion.txt', delimiter=',')

def undistort_image(image):
    h, w = image.shape[:2]
    new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (w, h), 1, (w, h))
    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, distortion_coeffs, None, new_camera_mtx, (w, h), 5)
    undistorted_img = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)
    x, y, w, h = roi
    return undistorted_img[y:y+h, x:x+w]

def send_motor_servo_control(motor_speed, servo_angle):
    try:
        data = f"M-{motor_speed} S{servo_angle} "
        ser.write(data.encode('utf-8'))
    except Exception as e:
        print(f"[Send] Error: {e}")

def stanley_control(e, psi, v, k=1.0):
    delta = psi + np.arctan(k * e / (v + 1e-6)) * (180 / np.pi)
    return np.clip(delta, -28, 28)

def convert_hsv(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

def HSV_color_selection(image):
    hsv = convert_hsv(image)
    white_mask = cv2.inRange(hsv, np.uint8([0, 0, 200]), np.uint8([180, 50, 255]))
    yellow_mask = cv2.inRange(hsv, np.uint8([10, 100, 100]), np.uint8([25, 255, 255]))
    mask = cv2.bitwise_or(white_mask, yellow_mask)
    gray = cv2.cvtColor(cv2.bitwise_and(image, image, mask=mask), cv2.COLOR_BGR2GRAY)
    return cv2.GaussianBlur(gray, (7, 7), 0)

def perspective_transform(frame):
    height, width = frame.shape[:2]
    pts1 = np.float32([(0, height), (width, height), (int(width * 0.8), int(height * 0.65)), (int(width * 0.25), int(height * 0.65))])
    pts2 = np.float32([[0, height], [width, height], [width, 0], [0, 0]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    warped = cv2.warpPerspective(frame, matrix, (640, 480))
    return warped, (pts1, pts2)

def detect_lane_points(mask, left_base, right_base, height):
    y = 472
    lx, rx = [], []
    while y > int(height * 0.55):
        for base, points in [(left_base, lx), (right_base, rx)]:
            img = mask[y - 35:y, base - 30:base + 30]
            _, contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                M = cv2.moments(contour)
                if M['m00'] > 50:
                    cx = int(M['m10'] / M['m00'])
                    points.append(base - 30 + cx)
                    base = base - 30 + cx
        y -= 35
    return lx, rx

def fit_lane_curve(points, y_start=470, step=35):
    coords = [(points[i], y_start - i * step) for i in range(len(points))]
    if len(coords) >= 2:
        fit = np.polyfit([p[1] for p in coords], [p[0] for p in coords], 2)
        return fit, coords
    return None, []

def draw_lane_polynomial(overlay, fit, color, y_range):
    x_vals = np.linspace(min(y_range), max(y_range), 50)
    y_vals = fit[0] * x_vals ** 2 + fit[1] * x_vals + fit[2]
    for i in range(len(x_vals) - 1):
        pt1 = (int(y_vals[i]), int(x_vals[i]))
        pt2 = (int(y_vals[i + 1]), int(x_vals[i + 1]))
        cv2.line(overlay, pt1, pt2, color, 2)

def calculate_ct_errors(left, right):
    if not left or not right:
        return 0
    lane_center = (left[0] + right[0]) / 2
    car_center = 320
    return lane_center - car_center

def draw_midpoints_and_angle(midpoints, overlay):
    if len(midpoints) < 2:
        return 0
    [vx, vy, x0, y0] = cv2.fitLine(np.array(midpoints), cv2.DIST_L2, 0, 0.01, 0.01)
    slope = vy[0] / vx[0]
    return -np.arctan(slope) * (180 / np.pi)

def visualize_result(frame, warped, overlay, pts1, pts2, denta, offset, angle_y):
    inv_matrix = cv2.getPerspectiveTransform(pts2, pts1)
    lane_overlay = cv2.warpPerspective(overlay, inv_matrix, (640, 480))
    result = cv2.addWeighted(frame, 1, lane_overlay, 0.5, 0)
    end_x = int(320 + 100 * np.sin(np.radians(denta)))
    end_y = int(480 - 100 * np.cos(np.radians(denta)))
    cv2.line(result, (320, 480), (end_x, end_y), (255, 0, 0), 2)
    cv2.putText(result, f'Offset: {offset:.2f}px', (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(result, f'Angle Y: {angle_y:.2f} deg', (30, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(result, f'Steering: {80 + denta:.2f} deg', (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    return result

def main_loop():
    prevLx, prevRx = [], []
    while True:
        success, frame = vidcap.read()
        if not success:
            break
        frame = undistort_image(frame)
        frame = cv2.resize(frame, (640, 480))
        warped, (pts1, pts2) = perspective_transform(frame)
        mask = HSV_color_selection(warped)

        hist = np.sum(mask[mask.shape[0]//2:, :], axis=0)
        midpoint = hist.shape[0] // 2
        left_base = np.argmax(hist[:midpoint])
        right_base = np.argmax(hist[midpoint:]) + midpoint

        lx, rx = detect_lane_points(mask, left_base, right_base, height=480)
        lx = lx if lx else prevLx
        rx = rx if rx else prevRx
        prevLx, prevRx = lx, rx

        left_fit, left_pts = fit_lane_curve(lx)
        right_fit, right_pts = fit_lane_curve(rx)
        overlay = warped.copy()
        if left_fit: draw_lane_polynomial(overlay, left_fit, (0, 255, 0), [p[1] for p in left_pts])
        if right_fit: draw_lane_polynomial(overlay, right_fit, (0, 255, 255), [p[1] for p in right_pts])

        midpoints = [((lx[i] + rx[i]) / 2, 470 - i * 35) for i in range(min(len(lx), len(rx)))]
        angle_y = draw_midpoints_and_angle(midpoints, overlay)
        offset = calculate_ct_errors(lx, rx)
        denta = stanley_control(offset * 0.000914, angle_y, 0.4, k=0.79)

        send_motor_servo_control(4900, 80 + denta)
        result = visualize_result(frame, warped, overlay, pts1, pts2, denta, offset, angle_y)
        cv2.imshow("Lane Detection", result)
        if cv2.waitKey(10) == 27:
            break
    vidcap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main_loop()
