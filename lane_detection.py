# -*- coding: utf-8 -*-
"""
lane_detection.py
車道線辨識邏輯（模組化）：灰階、高斯濾波、HSV 顏色過濾、Canny、ROI、Hough、左右車道平均、CTE
含單邊車道 fallback 與時間平滑，避免畫面跳動。
"""
import cv2
import numpy as np

# ---- 可調參數 ----
CANNY_LOW, CANNY_HIGH = 50, 150
HOUGH_THRESHOLD = 30
MIN_LINE_LENGTH = 30
MAX_LINE_GAP = 150
ROI_TOP_Y = 0.58          # ROI 上緣（佔畫面高度比例）；0.58 = 看得比較遠
ROI_TOP_W = 0.08          # ROI 上緣左右內縮（越小上緣越寬，容納彎道）
MIN_SLOPE = 0.25          # 過濾過於水平的雜訊線

# 記憶前一次結果做平滑（避免抖動）
_prev_lane_center = None
_SMOOTH_ALPHA = 0.6       # 越大越跟隨即時、越小越平滑


def hsv_color_mask(frame):
    """HSV 顏色過濾：抓白色與黃色車道線。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, (0, 0, 190), (180, 50, 255))    # 放寬白色（柏油反光也算）
    yellow = cv2.inRange(hsv, (15, 70, 100), (40, 255, 255))
    return cv2.bitwise_or(white, yellow)


def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)
    roi = np.array([[
        (int(width * 0.05), height),
        (int(width * ROI_TOP_W), int(height * ROI_TOP_Y)),
        (int(width * (1 - ROI_TOP_W)), int(height * ROI_TOP_Y)),
        (int(width * 0.95), height),
    ]], dtype=np.int32)
    cv2.fillPoly(mask, roi, 255)
    return cv2.bitwise_and(edges, mask)


def separate_lines(lines):
    left, right = [], []
    if lines is None:
        return left, right
    for line in lines:
        x1, y1, x2, y2 = np.ravel(line)
        if x1 == x2:
            continue
        slope = (y2 - y1) / (x2 - x1)
        if abs(slope) < MIN_SLOPE:
            continue
        (left if slope < 0 else right).append((int(x1), int(y1), int(x2), int(y2)))
    return left, right


def average_line(lines, img_shape):
    if not lines:
        return None
    height = img_shape[0]
    xs, ys = [], []
    for x1, y1, x2, y2 in lines:
        xs += [x1, x2]
        ys += [y1, y2]
    try:
        slope, intercept = np.polyfit(xs, ys, 1)
    except Exception:
        return None
    if slope == 0:
        return None
    y1 = height
    y2 = int(height * ROI_TOP_Y)
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return (x1, y1, x2, y2)


def detect_lanes(frame):
    """
    回傳 (標註後的影像, cte, lane_center)
    cte = 車道中心 - 畫面中心；+ 表示車道偏右(需左轉)，- 表示偏左(需右轉)
    單邊車道時，用「另一邊線距畫面邊界合理比例」推估車道中心。
    """
    global _prev_lane_center
    height, width = frame.shape[:2]
    result = frame.copy()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)

    color = hsv_color_mask(frame)
    combined = cv2.bitwise_or(edges, color)
    masked = region_of_interest(combined)

    lines = cv2.HoughLinesP(masked, 1, np.pi / 180, HOUGH_THRESHOLD,
                            minLineLength=MIN_LINE_LENGTH, maxLineGap=MAX_LINE_GAP)

    left_lines, right_lines = separate_lines(lines)
    left_avg = average_line(left_lines, frame.shape)
    right_avg = average_line(right_lines, frame.shape)

    lane_center = None
    if left_avg and right_avg:
        # 兩邊都有：最準
        for ln in (left_avg, right_avg):
            cv2.line(result, (ln[0], ln[1]), (ln[2], ln[3]), (0, 255, 0), 4)
        lane_center = (left_avg[0] + right_avg[0]) // 2
        cv2.line(result, (lane_center, height),
                 (lane_center, int(height * ROI_TOP_Y)), (0, 255, 255), 3)
    elif left_avg:
        # 只有左線：車道中心約在左線右方「畫面一半車道寬」處
        cv2.line(result, (left_avg[0], left_avg[1]), (left_avg[2], left_avg[3]), (0, 165, 255), 4)
        lane_center = left_avg[0] + int(width * 0.18)
        cv2.line(result, (lane_center, height),
                 (lane_center, int(height * ROI_TOP_Y)), (0, 255, 255), 3)
    elif right_avg:
        # 只有右線
        cv2.line(result, (right_avg[0], right_avg[1]), (right_avg[2], right_avg[3]), (0, 165, 255), 4)
        lane_center = right_avg[0] - int(width * 0.18)
        cv2.line(result, (lane_center, height),
                 (lane_center, int(height * ROI_TOP_Y)), (0, 255, 255), 3)
    elif lines is not None:
        for line in lines:
            x1, y1, x2, y2 = np.ravel(line)
            cv2.line(result, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # 時間平滑：避免 CTE 大幅跳動導致馬達抖動
    if lane_center is not None:
        if _prev_lane_center is not None:
            lane_center = int(_SMOOTH_ALPHA * lane_center +
                              (1 - _SMOOTH_ALPHA) * _prev_lane_center)
        _prev_lane_center = lane_center
    else:
        _prev_lane_center = None

    image_center = width // 2
    cte = None if lane_center is None else lane_center - image_center

    # ROI 邊界示意
    cv2.line(result, (int(width * ROI_TOP_W), int(height * ROI_TOP_Y)),
             (int(width * 0.05), height), (255, 120, 0), 1)
    cv2.line(result, (int(width * (1 - ROI_TOP_W)), int(height * ROI_TOP_Y)),
             (int(width * 0.95), height), (255, 120, 0), 1)

    return result, cte, lane_center
