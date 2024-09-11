import cv2
import numpy as np
import warnings

def binary(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 210, 255, cv2.THRESH_BINARY)
    return thresh


def denoise_frame(image):
    kernel = np.ones((3, 3), np.float32) / 9
    denoiseframe = cv2.filter2D(image, -1, kernel)
    blur = cv2.GaussianBlur(denoiseframe, (5, 5), 0)
    return blur


def warp_perspective(image):

    # Get image size
    height, width = image.shape[:2]

    # Perspective points to be warped
    pts1 = np.float32(region(image))

    # Window to be shown
    #pts2 = np.float32([[0, 240], [0, 0], [320, 240], [320, 0]])
    pts2 = np.float32([[0, 480], [0, 0], [640, 0], [640, 480]])
    # Matrix to warp the image for bird_eye_view window
    matrix = cv2.getPerspectiveTransform(pts1, pts2)

    # Final warping perspective
    bird_eye_view = cv2.warpPerspective(image, matrix, (width, height))

    return bird_eye_view


def region(image):
    # creating a polygon to focus only on the road in the picture
    height, width = image.shape[:2]
    bottom_left = [width * 0, height * 1]
    top_left = [width * 0, height * 0.5]
    bottom_right = [width * 1, height * 1]
    top_right = [width * 1, height * 0.5]
    return [bottom_left, top_left, top_right, bottom_right]


def region_selection(image):
    mask = np.zeros_like(image)
    if len(image.shape) > 2:
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        # color of the mask polygon (white)
        ignore_mask_color = 255
        # created this polygon based on where placed camera
    vertices = np.array([region(image)], dtype=np.int32)
    # filling the polygon with white color and generating the final mask
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    # performing Bitwise AND on the input image and mask to get only the edges on the road
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


def detect_line(thresh):
    contours, hier = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    if contours is None:
        return []
    return [cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01) for cnt in contours]


def find_from_to_xy(img, line):
    ht, wd, dp = img.shape
    [vx, vy, x, y] = line

    # y = ax + b
    x1, y1 = int(x-vx*int(ht/4)), int(y-vy*int(ht/4))
    x2, y2 = int(x+vx*int(ht/4)), int(y+vy*int(ht/4))

    # kiem tra bien
    x1 = x1 if x1 <= wd else x1 if x1 >= 0 else 0
    x2 = x2 if x2 <= wd else x2 if x2 >= 0 else 0
    y1 = y1 if y1 <= ht else y1 if y1 >= int(ht/4) else int(ht/4)
    y2 = y2 if y2 <= ht else y2 if y2 >= int(ht/4) else int(ht/4)
    return x1, y1, x2, y2


def find_angle(img, line):
    x1, y1, x2, y2 = find_from_to_xy(img, line)
    dx, dy = x2 - x1, y2 - y1
    angle = np.arctan2(dy, dx) * (180 / np.pi)
    return angle, x1, y1, x2, y2


def draw(img, lines):
    ht, wd, dp = img.shape
    center_x = wd / 2
    count_left, count_right = 0, 0
    left_x, left_y, right_x, right_y = [], [], [], []
    for line in lines:
        angle, x1, y1, x2, y2 = find_angle(img, line)
        if (angle <= 20) and (angle >= - 20):
            return img
        if x1 < x2:
            if x2 < center_x:
                left_x.append(x1)
                left_x.append(x2)
                left_y.append(y1)
                left_y.append(y2)
            if x1 > center_x:
                right_x.append(x1)
                right_x.append(x2)
                right_y.append(y1)
                right_y.append(y2)
        if x1 > x2:
            if x1 < center_x:
                left_x.append(x1)
                left_x.append(x2)
                left_y.append(y1)
                left_y.append(y2)
            if x2 > center_x:
                right_x.append(x1)
                right_x.append(x2)
                right_y.append(y1)
                right_y.append(y2)
        if len(left_x) == 0 or len(left_y) == 0:
            count_left = 0
        else:
            count_left += 1
            if count_left < 2:
                cv2.line(img, (left_x[0], left_y[0]), (left_x[1], left_y[1]), [255, 0, 0], 2)
        if len(right_x) == 0 or len(right_y) == 0:
            count_right = 0
        else:
            count_right += 1
            if count_right < 2:
                cv2.line(img, (right_x[0], right_y[0]), (right_x[1], right_y[1]), [255, 0, 0], 2)
    return img


def text_line_detect(image, lines):
    wt, wd, dp = image.shape
    left_point, right_point, center_x = point_x(image, lines)
    if len(left_point) > 0:
        cv2.putText(image, 'Left Lane Detected', (int(wt*3/4), int(wd/3)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        cv2.putText(image, 'No Left Lane Detected', (int(wt*3/4), int(wd/3)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2, cv2.LINE_AA)

    if len(right_point) > 0:
        cv2.putText(image, 'Right Lane Detected', (int(wt*3/4), int(wd/2)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        cv2.putText(image, 'No Right Lane Detected', (int(wt*3/4), int(wd/2)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2, cv2.LINE_AA)

    return image


def point_x(image, lines):
    wt, wd, dp = image.shape
    center_x = wd / 2
    left_point = []
    right_point = []
    for line in lines:
        [vx, vy, x, y] = line
        if x < center_x:
            left_point.append(x)
        if x > center_x:
            right_point.append(x)
    return left_point, right_point, center_x


def offset(image, lines):
    wt, wd, dp = image.shape
    left_point, right_point, center_x = point_x(image, lines)
    left_mean, right_mean = 0, 0
    if len(left_point) > 0 and len(right_point) > 0:
        left_mean = np.mean(left_point)
        if left_mean > 100:
            left_mean = 0
        right_mean = np.mean(right_point)
        if 40 < left_mean:
            cv2.putText(image, 'Turn right', (int(wt * 3 / 4), int(wd / 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2, cv2.LINE_AA)
        elif right_mean < 270:
            cv2.putText(image, 'Turn left', (int(wt * 3 / 4), int(wd / 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2, cv2.LINE_AA)
    if len(left_point) < 1 and len(right_point) > 0:
        right_mean = np.mean(right_point)
        if right_mean < 270:
            cv2.putText(image, 'Turn left', (int(wt * 3 / 4), int(wd / 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2, cv2.LINE_AA)
    if len(left_point) > 0 and len(right_point) < 1:
        left_mean = np.mean(left_point)
        if left_mean > 100:
            left_mean = 0
        if 40 < left_mean:
            cv2.putText(image, 'Turn right', (int(wt * 3 / 4), int(wd / 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2, cv2.LINE_AA)
    return image


def find_offset(image, lines):

    wt, wd, dp = image.shape
    center_x = wd / 2
    cv2.line(image, (wd // 2, wt), (wd // 2, 0), [230, 43, 239], 2)
    x_left, x_right, y_left, y_right = [], [], [], []
    offset_left, offset_right, y_offset_left, y_offset_right = [], [], [], []

    for line in lines:
        angle, x1, y1, x2, y2 = find_angle(image, line)
        if x1 < center_x:
            x_left.append(x1)
            y_left.append(y1)
        elif x1 > center_x:
            x_right.append(x1)
            y_right.append(y1)
        if x2 < center_x:
            x_left.append(x2)
            y_left.append(y2)
        elif x2 > center_x:
            x_right.append(x2)
            y_right.append(y2)

    left_x_empty = find_0(x_left)
    right_x_empty = find_0(x_right)

    if left_x_empty == 0 and right_x_empty == 0:
        line_left = np.polyfit(x_left, y_left, 1)
        for i in range(len(y_right)):
            line_left[1] -= y_right[i]
            x_line_left = np.roots(line_left) // 1
            line_left[1] += y_right[i]
            a = np.int_(x_line_left[0])
            dist_left = abs(np.int_(a - x_right[i]))
            center_lane_left = abs(np.int_(dist_left // 2))
            if a > 0:
                center_lane_left += a
            offset_left.append(np.int_(center_x - center_lane_left))
            y_offset_left.append(y_right[i])

        line_right = np.polyfit(x_right, y_right, 1)
        for i in range(len(y_left)):
            line_right[1] -= y_left[i]
            x_line_right = np.roots(line_right) // 1
            line_right[1] += y_left[i]
            b = np.int_(x_line_right[0])
            dist_right = abs(np.int_(b - x_left[i]))
            center_lane_right = abs(np.int_(dist_right // 2))
            if np.int_(x_left[i]) > 0:
                center_lane_right += np.int_(x_left[i])

            offset_right.append(np.int_(center_x - center_lane_right))
            y_offset_right.append(y_left[i])

        max_center_line_left, i = find_max(offset_left)
        max_center_line_right, j = find_max(offset_right)

        if abs(max_center_line_left) > 50 or abs(max_center_line_right) > 50:
            if abs(y_offset_left[i]) > abs(y_offset_right[j]):
                cv2.putText(image, 'Turn left', (int(wt * 3 / 4), int(wd / 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2, cv2.LINE_AA)
            elif abs(y_offset_left[i]) < abs(y_offset_right[j]):
                cv2.putText(image, 'Turn right', (int(wt * 3 / 4), int(wd / 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2, cv2.LINE_AA)
        elif abs(max_center_line_left) <= 50 or abs(max_center_line_right) <= 50:
            cv2.putText(image, 'Straight', (int(wt * 3 / 4), int(wd / 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2, cv2.LINE_AA)

    if left_x_empty == 1 and right_x_empty == 0:
        for i in range(len(y_right)):
            x_left.append(0)
            dist = abs(np.int_((x_left[i]) - (x_right[i])))
            center_lane = abs(np.int_(dist // 2))
            offset_left.append(np.int_(center_x - center_lane))
        cv2.putText(image, 'Turn left', (int(wt * 3 / 4), int(wd / 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 2, cv2.LINE_AA)

    if right_x_empty == 1 and left_x_empty == 0:
        for i in range(len(y_left)):
            x_right.append(wd)
            dist = abs(np.int_((x_left[i]) - (x_right[i])))
            center_lane = abs(np.int_(dist // 2))
            offset_right.append(np.int_(center_x - center_lane))
        cv2.putText(image, 'Turn right', (int(wt * 3 / 4), int(wd / 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 2, cv2.LINE_AA)

    if right_x_empty == 1 and left_x_empty == 1:
        cv2.putText(image, 'No Line Detected', (int(wt * 3 / 4), int(wd / 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 2, cv2.LINE_AA)
    warnings.filterwarnings("ignore")

    return image


def find_max(arr):
    x_max = arr[0]
    a = 0
    for i in range(len(arr)):
        if abs(x_max) < abs(arr[i]):
            x_max = arr[i]
            a = i
    return x_max, a


def find_0(arr):
    a = 0
    for i in range(len(arr)):
        if arr[i] == 0:
            a += 1
    if a == len(arr):
        return 1
    else:
        return 0
def steering_angle(image, lines):
    wt, wd, dp = image.shape
    center_frame = wd / 2
    cv2.line(image, (wd // 2, wt), (wd // 2, 0), [230, 43, 239], 2)
    x_left, x_right, y_left, y_right = [], [], [], []
    offset_left, offset_right, y_offset_left, y_offset_right = [], [], [], []
    max_left_offset, max_right_offset, center_lane_x, center_lane_y, i, j = 0, 0, 0, 0, 0, 0
    ppm_y = 196/8.5
    ratio_x = 60/640
    angle_cam = math.radians(30)
    dis_from_cam_to_max = 57.5
    dis_from_cam_to_min = 22
    for line in lines:
        angle, x1, y1, x2, y2 = find_angle(image, line)
        if x1 < center_frame:
            x_left.append(x1)
            y_left.append(y1)
        elif x1 > center_frame:
            x_right.append(x1)
            y_right.append(y1)
        if x2 < center_frame:
            x_left.append(x2)
            y_left.append(y2)
        elif x2 > center_frame:
            x_right.append(x2)
            y_right.append(y2)

    left_x_empty = find_0(x_left)
    right_x_empty = find_0(x_right)

    if left_x_empty == 0 and right_x_empty == 0:
        line_left = np.polyfit(x_left, y_left, 1)
        for i in range(len(y_right)):
            line_left[1] -= y_right[i]
            x_line_left = np.roots(line_left) // 1
            line_left[1] += y_right[i]
            a = np.int_(x_line_left[0])
            dist_left = abs(np.int_(a - x_right[i]))
            center_lane_left = abs(np.int_(dist_left // 2))
            if a > 0:
                center_lane_left += a
            if np.int_(center_frame - center_lane_left) > 0:
                offset_left.append(np.int_(center_frame - center_lane_left))
                y_offset_left.append(y_right[i])
            elif np.int_(center_frame - center_lane_left) < 0:
                offset_right.append(np.int_(center_frame - center_lane_left))
                y_offset_right.append(y_right[i])

        line_right = np.polyfit(x_right, y_right, 1)
        for i in range(len(y_left)):
            line_right[1] -= y_left[i]
            x_line_right = np.roots(line_right) // 1
            line_right[1] += y_left[i]
            b = np.int_(x_line_right[0])
            dist_right = abs(np.int_(b - x_left[i]))
            center_lane_right = abs(np.int_(dist_right // 2))
            if np.int_(x_left[i]) > 0:
                center_lane_right += np.int_(x_left[i])
            if np.int_(center_frame - center_lane_right) > 0:
                offset_left.append(np.int_(center_frame - center_lane_right))
                y_offset_left.append(y_left[i])
            elif np.int_(center_frame - center_lane_right) < 0:
                offset_right.append(np.int_(center_frame - center_lane_right))
                y_offset_right.append(y_left[i])

        if len(offset_left) > 0:
            max_left_offset, i = find_max(offset_left)
        if len(offset_right) > 0:
            max_right_offset, j = find_max(offset_right)

        if abs(max_left_offset) > abs(max_right_offset):
            center_lane_x = max_left_offset
            center_lane_y = y_offset_left[i]
        elif abs(max_left_offset) < abs(max_right_offset):
            center_lane_x = max_right_offset
            center_lane_y = y_offset_right[j]

    if left_x_empty == 1 and right_x_empty == 0:
        for i in range(len(y_right)):
            x_left.append(0)
            dist = abs(np.int_((x_left[i]) - (x_right[i])))
            center_lane = abs(np.int_(dist // 2))
            offset_left.append(np.int_(center_frame - center_lane))
            y_offset_left.append(y_right[i])
            max_left_offset, j = find_max(offset_left)
            center_lane_x = max_left_offset
            center_lane_y = y_offset_left[j]

    if right_x_empty == 1 and left_x_empty == 0:
        for i in range(len(y_left)):
            x_right.append(wd)
            dist = abs(np.int_((x_left[i]) - (x_right[i])))
            center_lane = abs(np.int_(dist // 2))
            offset_right.append(np.int_(center_frame - center_lane))
            y_offset_right.append(y_left[i])
            max_right_offset, j = find_max(offset_right)
            center_lane_x = max_right_offset
            center_lane_y = y_offset_right[j]

    # find d
    dis_from_cam_to_center_lane = dis_from_cam_to_max - ((wt-center_lane_y) / (ppm_y * math.cos(angle_cam)))

    # find delta_x
    ppm_center_lane = (480 / (wt - center_lane_y)) * 10.46
    delta_x = center_lane_x/ppm_center_lane
    angle = math.degrees(math.atan(delta_x/dis_from_cam_to_center_lane))
    warnings.filterwarnings("ignore")

    return angle
