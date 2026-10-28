# -*- coding: utf-8 -*-
"""
app.py  —  Flask 遠端遙控自走車 + 車道線辨識 + YOLO 物件偵測
執行：python app.py   瀏覽器開 http://<Pi-IP>:5000/
"""
import cv2
import threading
import time
from flask import Flask, Response, render_template, jsonify, request

from motor_control import motor
from lane_detection import detect_lanes
from pid_controller import PIDController, decide_action

app = Flask(__name__)

# ---------------- 攝影機 ----------------
USE_PICAM = True
try:
    from picamera2 import Picamera2
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(
        main={'size': (640, 480), 'format': 'BGR888'}))
    picam2.start()
    try:
        picam2.set_controls({"AeEnable": True, "AwbEnable": True})
    except Exception:
        pass
    time.sleep(3)
except Exception as e:
    print(f"[app] PiCamera2 不可用（{e}），改用 VideoCapture(0)")
    USE_PICAM = False
    cap = cv2.VideoCapture(0)

# ---------------- YOLO（可選） ----------------
yolo_model = None
try:
    from ultralytics import YOLO
    yolo_model = YOLO('yolo11n.pt')
    print("[app] YOLO 模型載入完成")
except Exception as e:
    print(f"[app] 未載入 YOLO（{e}），跳過物件偵測")

# ---------------- 全域名稱空間 ----------------
lock = threading.Lock()
pid = PIDController(kp=0.5, ki=0.0, kd=0.1)
ai_status = {
    'objects': {},
    'cte': None,
    'auto_drive': False,
    'auto_detail': {},
    'last_update': 0,
}


def read_frame():
    if USE_PICAM:
        f = picam2.capture_array()
        f = cv2.cvtColor(f, cv2.COLOR_BGR2RGB)   # 此 Pi 通道修正
        f = cv2.rotate(f, cv2.ROTATE_180)         # 鏡頭倒裝
        return True, f
    ok, f = cap.read()
    return ok, f


def ai_worker():
    """背景執行緒：跑 YOLO，避免卡住影像串流。"""
    global ai_status
    frame_count = 0
    while True:
        ok, frame = read_frame()
        if not ok:
            continue
        frame_count += 1
        if frame_count % 3 != 0:      # 跳幀處理，節省 CPU
            time.sleep(0.01)
            continue
        if yolo_model is not None:
            results = yolo_model.predict(frame, imgsz=320, verbose=False)
            counts = {}
            for r in results:
                for c in r.boxes.cls.tolist():
                    name = yolo_model.names[int(c)]
                    counts[name] = counts.get(name, 0) + 1
            with lock:
                ai_status['objects'] = counts
                ai_status['last_update'] = time.time()
        time.sleep(0.05)


def auto_drive_worker():
    """
    自動駕駛執行緒：依 CTE 用 PID 計算左右輪速差，行進中修正。
    修正方式是「持續前進 + 左右輪不同速度」，避免原地轉圈。
    """
    last_status = {}
    while True:
        with lock:
            auto = ai_status['auto_drive']
            cte = ai_status['cte']
        if auto:
            if cte is None:
                # 沒抓到車道：停下來，避免亂衝
                motor.stop()
                last_status = {'action': 'stop(no lane)'}
            else:
                left_ofs, right_ofs, u = pid.offsets(cte, base_speed=motor.speed)
                motor.drive_offset(left_ofs, right_ofs, motor.speed)
                last_status = {
                    'action': decide_action(left_ofs, right_ofs),
                    'u': round(u, 1),
                    'left': round(left_ofs, 1),
                    'right': round(right_ofs, 1),
                }
        with lock:
            ai_status['auto_detail'] = last_status
        time.sleep(0.1)


def gen_frames():
    global ai_status
    while True:
        ok, frame = read_frame()
        if not ok:
            continue

        result, cte, lane_center = detect_lanes(frame)
        with lock:
            ai_status['cte'] = cte

        h, w = frame.shape[:2]
        cv2.line(result, (w // 2, h), (w // 2, int(h * 0.6)), (255, 0, 0), 2)
        if cte is not None:
            d = "R->turnL" if cte > 0 else ("L->turnR" if cte < 0 else "CENTER")
            cv2.putText(result, f"CTE={cte:+d}px {d}", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        with lock:
            objs = ai_status['objects']
        y = 80
        for name, c in objs.items():
            cv2.putText(result, f"{name}: {c}", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            y += 30

        ok2, buf = cv2.imencode('.jpg', result)
        if not ok2:
            continue
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/move', methods=['POST'])
def api_move():
    body = request.get_json(silent=True) or {}
    action = body.get('action', 'stop')
    speed = body.get('speed')
    with lock:
        if action == 'stop':
            st = motor.stop()
        else:
            st = motor.move(action, speed=speed)
    return jsonify(st)


@app.route('/api/servo', methods=['POST'])
def api_servo():
    body = request.get_json(silent=True) or {}
    d = body.get('direction', '')
    with lock:
        st = motor.servo(d, step=body.get('step'))
    return jsonify(st)


@app.route('/api/speed', methods=['POST'])
def api_speed():
    body = request.get_json(silent=True) or {}
    try:
        motor.speed = max(0, min(100, int(body.get('speed', 60))))
    except (TypeError, ValueError):
        pass
    return jsonify(motor.status())


@app.route('/api/auto', methods=['POST'])
def api_auto():
    on = bool((request.get_json(silent=True) or {}).get('on', False))
    with lock:
        ai_status['auto_drive'] = on
        if not on:
            motor.stop()
    return jsonify({'auto_drive': on})


@app.route('/api/status')
def api_status():
    with lock:
        data = dict(motor.status())
        data['cte'] = ai_status['cte']
        data['objects'] = ai_status['objects']
        data['auto_drive'] = ai_status['auto_drive']
        data['auto_detail'] = ai_status.get('auto_detail', {})
    data['yolo'] = yolo_model is not None
    return jsonify(data)


if __name__ == '__main__':
    threading.Thread(target=ai_worker, daemon=True).start()
    threading.Thread(target=auto_drive_worker, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
