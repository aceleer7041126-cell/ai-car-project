# -*- coding: utf-8 -*-
"""
pid_controller.py
PID 控制器：輸入 CTE（車道偏差像素），輸出「行進中的左右輪速差」修正量。
u(t) = Kp*e(t) + Ki*∫e dt + Kd*de/dt

重點：修正方式是移動中左右輪速差（move_with_offset），
      不是原地左右轉（turnLeft/turnRight），否則車只會原地打轉。
"""


class PIDController:
    def __init__(self, kp=0.25, ki=0.0, kd=0.08, limit=35.0, deadzone=25.0):
        """
        kp   : 比例增益（每 1px 偏差給多少速度差）
        ki   : 積分增益（消除長期偏差，通常保持 0）
        kd   : 微分增益（抑制抖動）
        limit: 速差上限 %（避免一輪太快一輪太慢）
        deadzone: 死區（|CTE| 小於此值視為置中，直走不修正）
        """
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit = limit
        self.deadzone = deadzone
        self.prev_error = 0.0
        self.integral = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt=1.0):
        """error = CTE（車道中心 - 畫面中心）。回傳修正量 u（左右輪速差 %）。"""
        if error is None:
            return 0.0

        # 死區：小幅偏差不修正，避免左右抖動
        if abs(error) < self.deadzone:
            self.prev_error = error
            return 0.0

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        u = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return max(-self.limit, min(self.limit, u))

    def offsets(self, error, base_speed=60):
        """
        回傳 (left_offset, right_offset, u)：
        u > 0 → 車道偏右，車需往左修 → 左輪減速、右輪加速
        u < 0 → 車道偏左，車需往右修 → 左輪加速、右輪減速
        """
        u = self.compute(error)
        # u 正：left = -u, right = +u
        left = -u
        right = +u
        # 保證速度落在 0~100
        left = max(-base_speed, min(100 - base_speed, left))
        right = max(-base_speed, min(100 - base_speed, right))
        return left, right, u


def decide_action(left_offset, right_offset):
    """給 UI 顯示用：判斷目前是偏左/偏右修正還是直走。"""
    if abs(left_offset) < 1 and abs(right_offset) < 1:
        return 'forward'
    return 'left' if left_offset < 0 else 'right'
