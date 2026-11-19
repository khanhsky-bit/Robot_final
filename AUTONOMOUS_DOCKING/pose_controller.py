"""Bộ điều khiển tư thế (Pose Control) tọa độ cực (Công việc 1 - Khánh).

Tham khảo: P. Corke, "Robotics, Vision & Control", Springer 2017;
mã gốc PythonRobotics (move_to_pose.py).

Biến cực (theo mã gốc, góc chuẩn hóa về [-pi, pi]):
    rho   = ||(x*, y*) - (x, y)||
    alpha = atan2(y* - y, x* - x) - theta
    beta  = theta* - theta - alpha
Luật điều khiển (quy ước Corke, k_beta < 0):
    v     = k_rho * rho
    omega = k_alpha * alpha + k_beta * beta
Điều kiện ổn định: k_rho > 0, k_beta < 0, k_alpha - k_rho > 0.
(Mã gốc PythonRobotics viết  w = Kp_alpha*alpha - Kp_beta*beta  với Kp_beta = 3 > 0,
 tương đương k_beta = -3.)

Giao tiếp chuẩn với Chinh:
    compute_control(current_pose, target_pose, v_max_scale=1.0)
        -> (v, omega, omega_R, omega_L, dist_error)
"""
import math
import numpy as np

import config as C
from config import wrap_angle


class PoseController:
    def __init__(self, k_rho=C.K_RHO, k_alpha=C.K_ALPHA, k_beta=C.K_BETA,
                 r=C.WHEEL_RADIUS, L=C.WHEEL_BASE, omega_max=C.OMEGA_MAX,
                 allow_reverse=False):
        if not (k_rho > 0 and k_beta < 0 and k_alpha - k_rho > 0):
            raise ValueError("Gains vi phạm điều kiện ổn định: k_rho>0, k_beta<0, k_alpha>k_rho")
        self.k_rho, self.k_alpha, self.k_beta = k_rho, k_alpha, k_beta
        self.r, self.L, self.omega_max = r, L, omega_max
        self.allow_reverse = allow_reverse

    # ---- tọa độ cực ----
    @staticmethod
    def polar_errors(current_pose, target_pose):
        x, y, th = current_pose
        xg, yg, thg = target_pose
        dx, dy = xg - x, yg - y
        rho = math.hypot(dx, dy)
        alpha = wrap_angle(math.atan2(dy, dx) - th)
        beta = wrap_angle(thg - th - alpha)
        return rho, alpha, beta

    # ---- luật điều khiển (chưa bão hòa) ----
    def raw_velocity(self, current_pose, target_pose):
        x, y, th = current_pose
        xg, yg, thg = target_pose
        dx, dy = xg - x, yg - y
        rho, alpha, beta = self.polar_errors(current_pose, target_pose)
        v = self.k_rho * rho
        if abs(alpha) > math.pi / 2:
            if self.allow_reverse:
                # như mã gốc: lùi, tính lại alpha theo hướng đuôi robot
                alpha = wrap_angle(math.atan2(-dy, -dx) - th)
                beta = wrap_angle(thg - th - alpha)
                v = -v
            else:
                v = 0.0     # đích ở phía sau: quay tại chỗ hướng về đích, không lùi
        omega = self.k_alpha * alpha + self.k_beta * beta
        return v, omega, rho, alpha, beta

    # ---- quy đổi & bão hòa động cơ ----
    def wheel_speeds(self, v, omega):
        w_R = (v + omega * self.L / 2.0) / self.r
        w_L = (v - omega * self.L / 2.0) / self.r
        return w_R, w_L

    def saturate(self, v, omega, v_max_scale=1.0):
        """Co cả hai bánh cùng tỉ lệ nếu max|w| > omega_max*v_max_scale (giữ nguyên độ cong)."""
        lim = self.omega_max * min(max(v_max_scale, 0.0), 1.0)
        w_R, w_L = self.wheel_speeds(v, omega)
        m = max(abs(w_R), abs(w_L))
        if m > lim > 0:
            s = lim / m
            w_R, w_L, v, omega = w_R * s, w_L * s, v * s, omega * s
        elif lim == 0:
            w_R = w_L = v = omega = 0.0
        return v, omega, w_R, w_L

    def compute_control(self, current_pose, target_pose, v_max_scale=1.0):
        """Trả về (v, omega, omega_R, omega_L, dist_error). Đơn vị m/s, rad/s, m."""
        v, omega, rho, _, _ = self.raw_velocity(current_pose, target_pose)
        v, omega, w_R, w_L = self.saturate(v, omega, v_max_scale)
        return v, omega, w_R, w_L, rho


# Hàm tiện ích theo đúng chữ ký hợp đồng (dùng controller mặc định)
_default = PoseController()


def compute_control(current_pose, target_pose, v_max_scale=1.0):
    return _default.compute_control(current_pose, target_pose, v_max_scale)


def run_to_pose(start, goal, controller=None, T_max=60.0, tol_pos=0.005, v_max_scale=1.0,
                dt=C.DT):
    """Mô phỏng vòng kín thuần Pose Control (không FSM) để kiểm chứng. Trả về dict log."""
    from robot_model import DifferentialDriveRobot
    ctrl = controller or PoseController()
    rb = DifferentialDriveRobot(*start, dt=dt)
    log = {k: [] for k in ("t", "x", "y", "theta", "v", "omega", "wR", "wL", "rho", "alpha", "beta")}
    t = 0.0
    while t < T_max:
        v, om, wR, wL, rho = ctrl.compute_control(rb.pose, goal, v_max_scale)
        _, al, be = ctrl.polar_errors(rb.pose, goal)
        for k, val in zip(log, (t, rb.x, rb.y, rb.theta, v, om, wR, wL, rho, al, be)):
            log[k].append(val)
        if rho < tol_pos:
            break
        rb.step_wheels(wR, wL)
        t += dt
    return {k: np.array(v_) for k, v_ in log.items()}


if __name__ == "__main__":
    # Tự kiểm tra nhanh giao tiếp
    out = compute_control((0.0, 0.0, 0.0), C.Q_GOAL)
    assert len(out) == 5
    print("compute_control((0,0,0), q_goal) =", tuple(round(float(a), 4) for a in out))
