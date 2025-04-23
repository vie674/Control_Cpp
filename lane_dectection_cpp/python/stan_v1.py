from time import sleep

import cv2
import numpy as np
import time
import math
import serial 

vidcap = cv2.VideoCapture(0)
success, image = vidcap.read()

mid = 80
lane_offset = 0
anlge_inclination_wrt_y=0
car_to_lane_center = 0
curvature = 0 
anlge_inclination_wrt_x =0 
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Load camera parameters as global variables
camera_matrix = np.loadtxt('cameraMatrix.txt', delimiter=',')
distortion_coeffs = np.loadtxt('cameraDistortion.txt', delimiter=',')

def undistort_image(image):
    """
    Undistort an input image using global camera matrix and distortion coefficients.
    
    :param image: Input distorted image (numpy array)
    :return: Undistorted image (numpy array)
    """
    h, w = image.shape[:2]
    new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (w, h), 1, (w, h))
    
    # Compute undistortion map
    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, distortion_coeffs, None, new_camera_mtx, (w, h), 5)
    undistorted_img = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)
    
    # Crop image
    x, y, w, h = roi
    undistorted_img = undistorted_img[y:y+h, x:x+w]
    
    return undistorted_img

def send_motor_servo_control(motor_speed, servo_angle):
    try:
        # Gửi dữ liệu xuống STM32
        send_data = f"M-{motor_speed} S{servo_angle} "
        ser.write(send_data.encode('utf-8'))
    except Exception as e:
        print(f"[Send] Error: {e}")


def stanley_control(e,psi,v, k=1.0):
    """
    Hàm điều khiển Stanley để tính toán góc lái.
    - theta_c: Hướng của xe.
    - k: Hệ số điều khiển.
    Returns:
    - delta: Góc lái cần điều khiển (radian).
    """
    cons = 0.000001

    # Tính toán góc lái theo phương pháp Stanley
    delta = psi + np.arctan(k * e / (v+ cons )) * (180 / np.pi)
    if delta > 28:
        delta = 28
    elif delta < -28:
        delta = -28
    return delta
def calculate_ct_errors(left, right):
    """
    Tính toán cross track error và heading error dựa trên các phương trình bậc hai cho làn trái và phải.

    Parameters:
    - left_fit, right_fit: Các hệ số phương trình bậc hai cho lane trái và phải.
    - image_height: Chiều cao của ảnh (tính theo pixel).
    - image_width: Chiều rộng của ảnh (tính theo pixel).

    Returns:
    - cross_track_error: Lỗi cross track theo pixel.
    - heading_error: Lỗi heading theo độ.
    """

    # Tính toán tọa độ x của lane trái và phải tại y_eval
    left_lane_x = left[0]
    right_lane_x = right[0]

    # Trung tâm của làn đường
    lane_center_x = (left_lane_x + right_lane_x) / 2

    # Vị trí trung tâm của xe, giả sử xe ở chính giữa ảnh (x = 320)
    car_position_x = 320  # 320 cho ảnh có kích thước 480x640

    # Tính toán cross track error theo pixel
    cross_track_error =  lane_center_x - car_position_x

    return cross_track_error
def convert_hsv(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

def HSV_color_selection(image):
    # Chuyển ảnh sang HSV
    converted_image = convert_hsv(image)

    # Mask màu trắng
    lower_threshold = np.uint8([0, 0, 200])
    upper_threshold = np.uint8([180, 50, 255])
    white_mask = cv2.inRange(converted_image, lower_threshold, upper_threshold)

    # Mask màu vàng
    lower_threshold = np.uint8([10, 100, 100])
    upper_threshold = np.uint8([25, 255, 255])
    yellow_mask = cv2.inRange(converted_image, lower_threshold, upper_threshold)

    # Kết hợp mask trắng và vàng
    mask = cv2.bitwise_or(white_mask, yellow_mask)
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    img_gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
    blurred_img = cv2.GaussianBlur(img_gray, (7, 7), 0)
    return blurred_img

def nothing(x):
    pass


prevLx = []
prevRx = []

while success:
    success, image = vidcap.read()
    image = undistort_image (image)
    frame = cv2.resize(image, (640, 480),interpolation=cv2.INTER_AREA)

    ## Choosing points for perspective transformation
    height, width = frame.shape[:2]
    tl = (int(width * 0.80), int(height * 0.65))
    tr = (int(width * 0.25), int(height * 0.65))
    bl = (int(0), int(height))
    br = (int(width), int(height))

    cv2.circle(frame, tl, 5, (0, 0, 255), -1)
    cv2.circle(frame, bl, 5, (0, 0, 255), -1)
    cv2.circle(frame, tr, 5, (0, 0, 255), -1)
    cv2.circle(frame, br, 5, (0, 0, 255), -1)

    ## Aplying perspective transformation
    pts1 = np.float32([bl, br, tl, tr])
    pts2 = np.float32([[0, int(height)], [int(width), int(height)], [int(width), 0], [0, 0]])

    # Matrix to warp the image for birdseye window
    matrix = cv2.getPerspectiveTransform(pts1, pts2)

    transformed_frame = cv2.warpPerspective(frame, matrix, (640, 480))

    ### Object Detection
    # Image Thresholding
    mask = HSV_color_selection(transformed_frame)

    # Histogram
    histogram = np.sum(mask[mask.shape[0] // 2:, :], axis=0)
    midpoint = int(histogram.shape[0] / 2)
    left_base = np.argmax(histogram[:midpoint])
    right_base = np.argmax(histogram[midpoint:]) + midpoint

    # Sliding Window
    y = 472
    lx = []
    rx = []
    slope = 0.0

    msk = mask.copy()

    while y > int (height *0.55):
        ## Left threshold
        img = mask[y - 35:y, left_base - 30:left_base + 30]
        _,contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] >50 :
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                lx.append(left_base - 30 + cx)
                cv2.circle(msk, (left_base - 30 + cx,y), 2, (0, 0, 0), -1)
                left_base = left_base - 30 + cx


        ## Right threshold
        img = mask[y - 35:y, right_base - 30:right_base + 30]
        _,contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] >50:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                rx.append(right_base - 30 + cx)
                right_base = right_base -30 + cx

        cv2.rectangle(msk, (left_base - 30, y), (left_base + 30, y - 35), (255, 0, 0), 2)
        cv2.rectangle(msk, (right_base - 30, y), (right_base + 30, y - 35), (255, 0, 0), 2)
        y -= 35
    print("leng trai",len(lx),"leng phai",len(rx))

    overlay = transformed_frame.copy()

    # Ensure lx and rx are not empty
    if len(lx) == 0:
        lx = prevLx
    else:
        prevLx = lx
    if len(rx) == 0:
        rx = prevRx
    else:
        prevRx = rx

    if len(lx) < 2 and len(rx) > 2 :
        y_eval =450
        right_points = [(rx[i], 470 - i * 35) for i in range(len(rx))]
        right_fit = np.polyfit([p[1] for p in right_points], [p[0] for p in right_points], 2)
        x_right_vals = np.linspace(min([p[1] for p in right_points]), max([p[1] for p in right_points]), 50)
        right_y_vals = right_fit[0] * x_right_vals ** 2 + right_fit[1] * x_right_vals + right_fit[2]
        right_points_on_image = [(int(y), int(x)) for y, x in zip(right_y_vals, x_right_vals)]

        for i in range(len(right_points_on_image) - 1):
            cv2.line(overlay, right_points_on_image[i], right_points_on_image[i + 1], (0, 165, 255),
                 2)  # Cam cho lane phải
        right_curvature = ((1 + (2 * right_fit[0] * y_eval + right_fit[1]) ** 2) ** 1.5) / np.abs(2 * right_fit[0])* 0.00062857
        slope  = 2 * right_fit[0] * y_eval + right_fit[1]
        anlge_inclination_wrt_y = - np.arctan(slope) * (180 / np.pi)
        print("slope",slope)
        #print("anlge_inclination_wrt_x", anlge_inclination_wrt_x)
        #if (anlge_inclination_wrt_x >= 0):
        #    anlge_inclination_wrt_y = 90 - anlge_inclination_wrt_x


        #elif (anlge_inclination_wrt_x < 0):
        #    anlge_inclination_wrt_y = - (90 - math.fabs(anlge_inclination_wrt_x))

        car_to_right_lane_distance = math.fabs(rx[0] - 320)
        car_to_right_center = (530 / 2) - car_to_right_lane_distance
        lane_offset = - car_to_lane_center

    elif  len(rx) < 2 and len(lx) > 2 :
        y_eval = 450
        left_points = [(lx[i], 470 - i * 35) for i in range(len(lx))]
        left_fit = np.polyfit([p[1] for p in left_points], [p[0] for p in left_points], 2)

        x_left_vals = np.linspace(min([p[1] for p in left_points]), max([p[1] for p in left_points]), 50)
        left_y_vals = left_fit[0] * x_left_vals ** 2 + left_fit[1] * x_left_vals + left_fit[2]
        left_points_on_image = [(int(y), int(x)) for y, x in zip(left_y_vals, x_left_vals)]

        for i in range(len(left_points_on_image) - 1):
            cv2.line(overlay, left_points_on_image[i], left_points_on_image[i + 1], (165, 33, 255),
                     2)  # Xanh lá cho lane trái

        left_curvature = ((1 + (2 * left_fit[0] * y_eval + left_fit[1]) ** 2) ** 1.5) / np.abs(2 * left_fit[0]) * 0.00062857

        slope = 2 * left_fit[0] * y_eval + left_fit[1]
        print("slope", slope)
        anlge_inclination_wrt_y = -np.arctan(slope) * (180 / np.pi)
        #print("anlge_inclination_wrt_x",anlge_inclination_wrt_x)
        #if (anlge_inclination_wrt_x >= 0):
        #    anlge_inclination_wrt_y = 90 - anlge_inclination_wrt_x


        #elif (anlge_inclination_wrt_x < 0):
        #    anlge_inclination_wrt_y = - (90 - math.fabs(anlge_inclination_wrt_x))
        car_to_left_lane_distance = math.fabs(lx[0] - 320)
        car_to_lane_center = (530 / 2) - car_to_left_lane_distance
        lane_offset =  car_to_lane_center

    elif len(lx) >= 2 and len(rx) >= 2:
        # Ensure both lx and rx have the same length
        min_length = min(len(lx), len(rx))
        print ("min_length   ",min_length)

        # Create a copy of the transformed frame


        # Create the top and bottom points for the quadrilateral
        previus_midpoint =(320,472)

        midpoints = []  # List to store midpoints

        # Vẽ mid point
        for i in range (min_length):
            if min_length > 1 :
                print("i",i)

                left = (lx[i], 470 - i*35 )
                right = (rx[i], 470 - i*35 )

                cv2.line(overlay, left, right, (0, 255, 0), 1)  # Đường màu xanh lá
                distance = np.sqrt((right[0] - left[0]) ** 2 + (right[1] - left[1]) ** 2)

                # Tính trung điểm giữa left và right
                now_midpoint = ((left[0] + right[0]) / 2, (left[1] + right[1]) / 2)

                # Lưu trung điểm vào danh sách
                midpoints.append(now_midpoint)

                # Vẽ điểm trung tâm (màu đỏ) lên overlay
                cv2.circle(overlay, (int(now_midpoint[0]), int(now_midpoint[1])), 5, (0, 0, 255), -1)

                # Vẽ đường thẳng từ trung điểm trước đó đến trung điểm hiện tại
                cv2.line(overlay, (int(previus_midpoint[0]), int(previus_midpoint[1])),
                         (int(now_midpoint[0]), int(now_midpoint[1])), (200, 100, 250), 1)
                #Cập nhật mid point
                previus_midpoint = now_midpoint

                # In ra chiều dài
                print(f"Distance between is {distance:.2f} pixels")

                # Draw the filled polygon on the transformed frame
                alpha = 1  # Opacity factor
                cv2.addWeighted(overlay, alpha, transformed_frame, 1 - alpha, 0, transformed_frame)

        if len(midpoints) >= 3:
            # Chọn 3 điểm đầu tiên
            points = np.array(midpoints[:len(midpoints)], dtype=np.float32)

            # Sử dụng hàm cv2.fitLine để tìm phương trình đường thẳng
            [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
            print("vy",vy[0])
            print("vx", vx[0])
            # Tính toán phương trình đường thẳng từ các tham số tìm được
            slope = vy[0] / vx[0]  # Độ dốc (slope)

            intercept = y0 - slope * x0  # Giao điểm với trục y (intercept)

            anlge_inclination_wrt_x = -np.arctan(slope) * (180 / np.pi)
            mid = 80
            if (anlge_inclination_wrt_x >= 0):
                anlge_inclination_wrt_y = 90 - anlge_inclination_wrt_x
                #angle = mid - anlge_inclination_wrt_y
                #steering_angle = angle

            elif (anlge_inclination_wrt_x < 0):
                anlge_inclination_wrt_y = - (90 - math.fabs(anlge_inclination_wrt_x))
                #angle = mid - anlge_inclination_wrt_y
                #steering_angle = angle

        elif len(midpoints) == 2:

            # Chọn 2 điểm đầu tiên
            x1, y1 = midpoints[0]
            x2, y2 = midpoints[1]

            # Tính độ dốc (slope)
            slope = (y2 - y1) / (x2 - x1)

            # Tính giao điểm với trục y (intercept)
            intercept = y1 - slope * x1

            anlge_inclination_wrt_x = -np.arctan(slope) * (180 / np.pi)
            mid = 80
            if (anlge_inclination_wrt_x >= 0):
                anlge_inclination_wrt_y = 90 - anlge_inclination_wrt_x
                # angle = mid - anlge_inclination_wrt_y
                # steering_angle = angle

            elif (anlge_inclination_wrt_x < 0):
                anlge_inclination_wrt_y = - (90 - math.fabs(anlge_inclination_wrt_x))
                # angle = mid - anlge_inclination_wrt_y
                # steering_angle = angle

            # Vẽ đường thẳng từ phương trình
            #left_x = int(150)  # Điểm bắt đầu (x = 0)
            #left_y = int(slope * left_x + intercept)  # Tính toán y tương ứng

            #right_x = int(440)  # Điểm kết thúc (x = chiều rộng ảnh)
            #right_y = int(slope * right_x + intercept)  # Tính toán y tương ứng

            # Vẽ đường thẳng trên ảnh
            #cv2.line(overlay, (int(left_x), int(left_y)), (int(right_x), int(right_y)), (255, 0, 0), 2)

        left_points = [(lx[i], 470 - i * 35) for i in range(min_length)]
        right_points = [(rx[i], 470 - i * 35) for i in range(min_length)]

        lane_offset = calculate_ct_errors(lx ,rx)

        # Khớp một đa thức bậc 2 với các điểm lane trái và phải
        left_fit = np.polyfit([p[1] for p in left_points], [p[0] for p in left_points], 2)
        right_fit = np.polyfit([p[1] for p in right_points], [p[0] for p in right_points], 2)

        # Tạo các giá trị x cho đường trái và phải
        x_left_vals = np.linspace(min([p[1] for p in left_points]), max([p[1] for p in left_points]), 50)
        x_right_vals = np.linspace(min([p[1] for p in right_points]), max([p[1] for p in right_points]), 50)

        # Tính toán y từ các giá trị x dựa trên đa thức bậc 2 đã khớp
        left_y_vals = left_fit[0] * x_left_vals ** 2 + left_fit[1] * x_left_vals + left_fit[2]
        right_y_vals = right_fit[0] * x_right_vals ** 2 + right_fit[1] * x_right_vals + right_fit[2]

        # Chuyển các điểm (x, y) thành tọa độ pixel trên ảnh
        left_points_on_image = [(int(y), int(x)) for y, x in zip(left_y_vals, x_left_vals)]
        right_points_on_image = [(int(y), int(x)) for y, x in zip(right_y_vals, x_right_vals)]

        for i in range(len(left_points_on_image) - 1):
            cv2.line(overlay, left_points_on_image[i], left_points_on_image[i + 1], (165, 33, 255),
                     2)  # Xanh lá cho lane trái
            cv2.line(overlay, right_points_on_image[i], right_points_on_image[i + 1], (0, 165, 255),
                     2)  # Cam cho lane phải


        # Calculate the curvature
        y_eval = 420
        left_curvature = ((1 + (2 * left_fit[0] * y_eval + left_fit[1]) ** 2) ** 1.5) / np.abs(2 * left_fit[0])*0.000628
        right_curvature = ((1 + (2 * right_fit[0] * y_eval + right_fit[1]) ** 2) ** 1.5) / np.abs(2 * right_fit[0])*0.000628

        print("left_curvature",left_curvature)
        print("right_curvature",right_curvature)
        curvature = (left_curvature + right_curvature) / 2
    else :
        lane_offset = 0
        anlge_inclination_wrt_y = 0
    # Stanley controler
    denta = stanley_control(lane_offset * 0.000914, anlge_inclination_wrt_y, 0.4, k=0.79)
    steering_angle = mid + denta
    send_motor_servo_control (4900,steering_angle)

    alpha = 1  # Opacity factor
    cv2.addWeighted(overlay, alpha, transformed_frame, 1 - alpha, 0, transformed_frame)
    # Display the transformed frame with the highlighted lane
    #cv2.imshow("Transformed Frame with Highlighted Lane", overlay)
    #   # Calculate the end point of the line based on the angle
    line_length = 100  # Length of the line
    end_x = int(320 + line_length * np.sin(np.radians(denta)))
    end_y = int(480 - line_length * np.cos(np.radians(denta)))


    # Inverse perspective transformation to map the lanes back to the original image
    inv_matrix = cv2.getPerspectiveTransform(pts2, pts1)
    original_perpective_lane_image = cv2.warpPerspective(transformed_frame, inv_matrix, (640, 480))

    # Combine the original frame with the lane image
    result = cv2.addWeighted(frame, 1, original_perpective_lane_image, 0.5, 0)

    # Draw a straight line in the center of the frame pointing with the current angle
    cv2.line(result, (320, 480), (end_x, end_y), (255, 0, 0), 2)
    cv2.line(result, (320, 480), (320, 440), (0, 0, 0), 2)
    # Display the curvature, offset, and angle on the frame
    cv2.putText(result, f'Curvature: {curvature:.2f} m', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(result, f'Offset: {(lane_offset):.6f} m', (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(result, f'steering_angle: {steering_angle:.2f} deg', (30, 110), cv2.FONT_HERSHEY_SIMPLEX, 1,
                (255, 255, 255), 2)
    cv2.putText(result, f'anlge_inclination_wrt_y: {anlge_inclination_wrt_y:.2f} deg', (30, 160),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(result, f'anlge_inclination_wrt_x: {anlge_inclination_wrt_x:.2f} deg', (30, 200),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(result, f'denta: {denta:.2f} ', (30, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    #cv2.imshow("Original", frame)
    #cv2.imshow("Bird's Eye View", transformed_frame)
    #cv2.imshow("Lane Detection - Image Thresholding", mask)
    #cv2.imshow("Lane Detection - Sliding Windows", msk)
    #cv2.imshow('Lane Detection', result)

    if cv2.waitKey(10) == 27:
        break

vidcap.release()
cv2.destroyAllWindows()

