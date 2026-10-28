# -*- coding: utf-8 -*-
"""
motor_control.py
封裝 LOBOROBOT2.py（老師提供的小車運動庫）。

LOBOROBOT2 實際 API（對照）：
    moveforward(speed, t_time)     前進
    movebackward(speed, t_time)    後退
    moveLeft(speed, t_time)        左移（平移）
    moveRight(speed, t_time)       右移（平移）
    turnLeft(speed, t_time)        左轉（原地）
    turnRight(speed, t_time)       右轉（原地）
    backward_Left / backward_Right 後左退 / 後右退
    t_stop(t_time)                 停止（含舵機斷電）
    set_servo_angle(channel, angle, t_time)   舵機角度 0~180
    stop_servo_angle(channel)                 舵機停止

舵機通道：channel 9 = 水平（底座左右），channel 10 = 垂直（頂部上下）
連續控制時 t_time 傳 0（不停留），由 Flask 端持續送指令或自行計時。
"""
import time

# ---- 硬體參數 ----
DEFAULT_SPEED = 60          # 一般行進速度 %
# 實測：這台車的通道與預期相反（按右→鏡頭下、按左→鏡頭上），故對調
SERVO_H_CHANNEL = 10        # 水平舵機（實際接在 channel 10）
SERVO_V_CHANNEL = 9         # 垂直舵機（實際接在 channel 9）
SERVO_H_CENTER = 90         # 水平置中角度
SERVO_V_CENTER = 70         # 垂直置中角度（依實車可調）
SERVO_H_STEP = 15
SERVO_V_STEP = 10
SERVO_MIN, SERVO_MAX = 20, 160

# 若實車方向與畫面相反，把對應的值改成 -1 即可反轉
SERVO_H_INVERT = 1          # 水平方向
SERVO_V_INVERT = 1          # 垂直方向

try:
    from LOBOROBOT2 import LOBOROBOT
    _ROBOT = LOBOROBOT()
    MOCK = False
    print("[motor_control] 已載入 LOBOROBOT2 硬體控制")
except Exception as e:
    _ROBOT = None
    MOCK = True
    print(f"[motor_control] 未載入硬體（{e}），進入 MOCK 模式")


class MotorControl:
    """對外統一介面，供 app.py 呼叫。"""

    def __init__(self):
        self.current_action = 'stop'
        self.speed = DEFAULT_SPEED
        self.servo_h = SERVO_H_CENTER
        self.servo_v = SERVO_V_CENTER
        self._last_servo_time = 0

    # ---------- 基礎動作 ----------
    def move(self, action, speed=None, t_time=0):
        """
        action: forward / backward / left / right / stop
        t_time: 0 表示不停留（給網頁連續控制用）
        """
        action = (action or 'stop').lower()
        speed = speed or self.speed

        if MOCK or _ROBOT is None:
            self.current_action = action
            return self.status()

        # 後退緩啟動，降低瞬時電流衝擊
        if action == 'backward':
            time.sleep(0.01)

        if action == 'forward':
            _ROBOT.moveforward(speed, t_time)
        elif action == 'backward':
            _ROBOT.movebackward(speed, t_time)
        elif action == 'left':          # 遙控頁的左轉 = 原地左轉
            _ROBOT.turnLeft(speed, t_time)
        elif action == 'right':
            _ROBOT.turnRight(speed, t_time)
        elif action == 'strafe_left':   # 平移左（備用）
            _ROBOT.moveLeft(speed, t_time)
        elif action == 'strafe_right':  # 平移右（備用）
            _ROBOT.moveRight(speed, t_time)
        elif action == 'back_left':
            _ROBOT.backward_Left(speed, t_time)
        elif action == 'back_right':
            _ROBOT.backward_Right(speed, t_time)
        else:
            _ROBOT.t_stop(0)
            action = 'stop'

        self.current_action = action
        return self.status()

    def stop(self):
        if not MOCK and _ROBOT is not None:
            _ROBOT.t_stop(0)
        self.current_action = 'stop'
        return self.status()

    def drive_offset(self, left_ofs, right_ofs, base_speed=None):
        """
        行進中修正：持續前進，左右輪不同速度。
        對應 LOBOROBOT2.move_with_offset(speed, l_ofs, r_ofs, t_time)。
        left_ofs / right_ofs 為相對於 base_speed 的增減量（可為負）。
        """
        speed = base_speed or self.speed
        # 與 LOBOROBOT2 的左右輪慣例一致（0/2 左、1/3 右）
        l_speed = max(0, min(100, int(speed + left_ofs)))
        r_speed = max(0, min(100, int(speed + right_ofs)))

        if not MOCK and _ROBOT is not None:
            _ROBOT.MotorRun(0, 'forward', l_speed)
            _ROBOT.MotorRun(2, 'forward', l_speed)
            _ROBOT.MotorRun(1, 'forward', r_speed)
            _ROBOT.MotorRun(3, 'forward', r_speed)

        self.current_action = 'auto'
        return self.status()

    # ---------- 舵機（攝影機角度） ----------
    def servo(self, direction, step=None):
        """direction: left/right/up/down/center"""
        direction = (direction or '').lower()

        if direction in ('left', 'right'):
            delta = (step or SERVO_H_STEP) * SERVO_H_INVERT
            self.servo_h += -delta if direction == 'left' else delta
            self.servo_h = max(SERVO_MIN, min(SERVO_MAX, self.servo_h))
            self._apply_servo(SERVO_H_CHANNEL, self.servo_h)
        elif direction in ('up', 'down'):
            delta = (step or SERVO_V_STEP) * SERVO_V_INVERT
            self.servo_v += -delta if direction == 'up' else delta
            self.servo_v = max(SERVO_MIN, min(SERVO_MAX, self.servo_v))
            self._apply_servo(SERVO_V_CHANNEL, self.servo_v)
        elif direction == 'center':
            self.servo_h, self.servo_v = SERVO_H_CENTER, SERVO_V_CENTER
            self._apply_servo(SERVO_H_CHANNEL, self.servo_h)
            self._apply_servo(SERVO_V_CHANNEL, self.servo_v)

        return self.status()

    def _apply_servo(self, channel, angle):
        if MOCK or _ROBOT is None:
            return
        _ROBOT.set_servo_angle(channel, angle, 0)
        # 若連續轉動角度需讓舵機有時間到位
        now = time.time()
        if now - self._last_servo_time > 0.5:
            time.sleep(0.2)
        self._last_servo_time = now

    # ---------- 狀態 ----------
    def status(self):
        return {
            'action': self.current_action,
            'speed': self.speed,
            'servo_h': self.servo_h,
            'servo_v': self.servo_v,
            'mock': MOCK,
        }


motor = MotorControl()
