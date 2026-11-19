"""Baseline: tái lập ví dụ gốc PythonRobotics `move_to_pose.py` (Công việc 1 - Khánh).

Nguồn: https://github.com/AtsushiSakai/PythonRobotics (PathTracking/move_to_pose)
Tác giả gốc: D. Ingram, A. Sakai, S. M. Yazdian, W. Zheng. Giấy phép MIT.
Tham khảo: P. Corke, "Robotics, Vision & Control", Springer, 2017.

Giữ nguyên luật điều khiển và hằng số của mã gốc (Kp_rho=9, Kp_alpha=15, Kp_beta=3,
dt=0.01, v<=15, w<=7) nhưng bỏ hoạt ảnh để chạy không giao diện và lưu ảnh.
Dùng 5 cặp (xuất phát, đích) cố định thay vì ngẫu nhiên để kết quả lặp lại được.
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def angle_mod(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class PathFinderController:
    def __init__(self, Kp_rho, Kp_alpha, Kp_beta):
        self.Kp_rho, self.Kp_alpha, self.Kp_beta = Kp_rho, Kp_alpha, Kp_beta

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
        rho = np.hypot(x_diff, y_diff)
        v = self.Kp_rho * rho
        alpha = angle_mod(np.arctan2(y_diff, x_diff) - theta)
        beta = angle_mod(theta_goal - theta - alpha)
        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            alpha = angle_mod(np.arctan2(-y_diff, -x_diff) - theta)
            beta = angle_mod(theta_goal - theta - alpha)
            w = self.Kp_alpha * alpha - self.Kp_beta * beta
            v = -v
        else:
            w = self.Kp_alpha * alpha - self.Kp_beta * beta
        return rho, v, w


controller = PathFinderController(9, 15, 3)
dt = 0.01
MAX_SIM_TIME = 5
MAX_LINEAR_SPEED = 15
MAX_ANGULAR_SPEED = 7


def move_to_pose(xs, ys, ths, xg, yg, thg):
    x, y, theta = xs, ys, ths
    xt, yt, vt, wt = [x], [y], [0], [0]
    rho, t = np.hypot(xg - x, yg - y), 0
    while rho > 0.001 and t < MAX_SIM_TIME:
        t += dt
        xt.append(x); yt.append(y)
        rho, v, w = controller.calc_control_command(xg - x, yg - y, theta, thg)
        v = np.sign(v) * min(abs(v), MAX_LINEAR_SPEED)
        w = np.sign(w) * min(abs(w), MAX_ANGULAR_SPEED)
        vt.append(v); wt.append(w)
        theta = theta + w * dt
        x = x + v * np.cos(theta) * dt
        y = y + v * np.sin(theta) * dt
    return xt, yt, vt, wt, theta


CASES = [  # (x0, y0, th0, xg, yg, thg)
    (2.0, 3.0, 0.0, 15.0, 12.0, np.pi / 2),
    (18.0, 4.0, np.pi, 5.0, 14.0, -np.pi / 2),
    (10.0, 18.0, -np.pi / 2, 10.0, 3.0, 0.0),
    (3.0, 16.0, np.pi / 4, 17.0, 5.0, np.pi),
    (12.0, 8.0, 3.0, 4.0, 4.0, -2.0),
]

if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "results", "figures"); os.makedirs(out, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 7))
    print(f"{'#':<3}{'rho_cuoi(m)':>12}{'e_theta(deg)':>14}")
    for i, c in enumerate(CASES, 1):
        xt, yt, vt, wt, th_end = move_to_pose(*c)
        ln, = ax.plot(xt, yt, lw=1.6, label=f"Ca {i}")
        ax.arrow(c[0], c[1], np.cos(c[2]), np.sin(c[2]), width=.1, color=ln.get_color(), alpha=.5)
        ax.arrow(c[3], c[4], np.cos(c[5]), np.sin(c[5]), width=.1, color="g")
        rho_f = np.hypot(c[3] - xt[-1], c[4] - yt[-1])
        e_th = np.degrees(angle_mod(c[5] - th_end))
        print(f"{i:<3}{rho_f:>12.4f}{e_th:>14.3f}")
    ax.set_xlim(0, 20); ax.set_ylim(0, 20); ax.set_aspect("equal"); ax.grid(alpha=.3)
    ax.set_title("Baseline PythonRobotics – move_to_pose (đích: mũi tên xanh)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(out, "fig_baseline_pythonrobotics.png"), dpi=200)
    print("Đã lưu results/figures/fig_baseline_pythonrobotics.png")
