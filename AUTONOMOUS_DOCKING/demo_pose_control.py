"""Kiểm chứng PoseController + robot_model (Khánh): Pose Control thuần, chưa có FSM.

Xuất: quỹ đạo 5 tư thế xuất phát, đồ thị (rho, alpha, beta), v/omega, tốc độ bánh (có bão hòa),
và bảng kết quả results/data/pose_control_check.csv.
"""
import os, csv, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import config as C
from pose_controller import PoseController, run_to_pose
from robot_model import DifferentialDriveRobot

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "results", "figures"); DATA = os.path.join(HERE, "results", "data")
os.makedirs(FIG, exist_ok=True); os.makedirs(DATA, exist_ok=True)

GOAL = C.Q_GOAL
D = math.radians
CASES = {
    "1. Trước trạm": (0.5, 4.5, 0.0),
    "2. Lệch trái":  (0.5, 6.0, 0.0),
    "3. Lệch phải":  (0.5, 3.0, 0.0),
    "4. Góc lệch lớn": (1.5, 4.5, D(150)),
    "5. Vị trí xa bất kỳ": (1.0, 1.0, D(-120)),
}


def draw_station(ax):
    xg, yg, _ = GOAL
    ax.add_patch(plt.Rectangle((xg, yg - .2), .12, .4, color="tab:green", label="Trạm sạc"))
    xs, ys, _ = C.staging_point()
    ax.plot(xs, ys, "m^", ms=8, label="Điểm chờ")


def main():
    ctrl = PoseController()
    logs, rows = {}, []
    for name, s in CASES.items():
        lg = run_to_pose(s, GOAL, ctrl)
        logs[name] = lg
        ep = lg["rho"][-1] * 1000
        eth = math.degrees(C.wrap_angle(lg["theta"][-1] - GOAL[2]))
        path = float(np.sum(np.hypot(np.diff(lg["x"]), np.diff(lg["y"]))))
        rows.append((name, *s[:2], math.degrees(s[2]), round(lg["t"][-1], 2), round(ep, 2),
                     round(eth, 3), round(path, 3), round(float(np.max(np.abs([lg["wR"], lg["wL"]]))), 2)))

    # --- Hình 1: quỹ đạo ---
    fig, ax = plt.subplots(figsize=(9, 6.5))
    for name, lg in logs.items():
        ln, = ax.plot(lg["x"], lg["y"], lw=1.8, label=name)
        s = CASES[name]
        ax.plot(s[0], s[1], "o", color=ln.get_color())
        ax.arrow(s[0], s[1], .3 * math.cos(s[2]), .3 * math.sin(s[2]), head_width=.08, color=ln.get_color())
    rb = DifferentialDriveRobot(*GOAL); p = rb.body_corners()
    ax.fill(p[:, 0], p[:, 1], color="tab:orange", alpha=.7, label="Robot tại đích")
    draw_station(ax)
    ax.set_aspect("equal"); ax.grid(alpha=.3); ax.legend(fontsize=8, loc="center left", bbox_to_anchor=(1.01, .5))
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Pose Control: quỹ đạo từ 5 tư thế xuất phát tới trạm sạc")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig_pose_control_trajectories.png"), dpi=200); plt.close(fig)

    # --- Hình 2: chi tiết 1 ca (vị trí xa bất kỳ) ---
    for tag, key in (("far", "5. Vị trí xa bất kỳ"), ("left", "2. Lệch trái")):
        lg = logs[key]; t = lg["t"]
        fig, axs = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
        axs[0, 0].plot(t, lg["rho"]); axs[0, 0].set_ylabel(r"$\rho$ [m]")
        axs[0, 1].plot(t, np.degrees(lg["alpha"]), label=r"$\alpha$")
        axs[0, 1].plot(t, np.degrees(lg["beta"]), label=r"$\beta$")
        axs[0, 1].set_ylabel("Góc [độ]"); axs[0, 1].legend()
        axs[1, 0].plot(t, lg["v"], label="v [m/s]")
        axs[1, 0].plot(t, lg["omega"], label=r"$\omega$ [rad/s]")
        axs[1, 0].set_ylabel("Vận tốc robot"); axs[1, 0].legend()
        axs[1, 1].plot(t, lg["wR"], label=r"$\omega_R$"); axs[1, 1].plot(t, lg["wL"], label=r"$\omega_L$")
        axs[1, 1].axhline(C.OMEGA_MAX, color="r", ls="--", lw=.8); axs[1, 1].axhline(-C.OMEGA_MAX, color="r", ls="--", lw=.8)
        axs[1, 1].set_ylabel("Tốc độ bánh [rad/s]"); axs[1, 1].legend()
        for a in axs.ravel(): a.grid(alpha=.3)
        for a in axs[1]: a.set_xlabel("t [s]")
        fig.suptitle(f"Pose Control – {key}: biến cực, vận tốc và tốc độ bánh")
        fig.tight_layout(); fig.savefig(os.path.join(FIG, f"fig_pose_control_signals_{tag}.png"), dpi=200); plt.close(fig)

    # --- Hình 3: ảnh hưởng hệ số khuếch đại (ca 'xa bất kỳ') ---
    gains = {"k=(1, 3, -1) [chuẩn]": (1.0, 3.0, -1.0), "k=(0.5, 3, -1)": (0.5, 3.0, -1.0),
             "k=(1, 3, -0.3)": (1.0, 3.0, -0.3), "k=(1, 3, -2)": (1.0, 3.0, -2.0)}
    fig, ax = plt.subplots(figsize=(8, 6))
    grows = []
    for gname, g in gains.items():
        lg = run_to_pose(CASES["5. Vị trí xa bất kỳ"], GOAL, PoseController(*g))
        ax.plot(lg["x"], lg["y"], label=gname)
        grows.append((gname, round(lg["t"][-1], 2),
                      round(math.degrees(C.wrap_angle(lg["theta"][-1] - GOAL[2])), 3)))
    draw_station(ax); ax.set_aspect("equal"); ax.grid(alpha=.3); ax.legend(fontsize=8)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(r"Ảnh hưởng $(k_\rho, k_\alpha, k_\beta)$ lên quỹ đạo")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig_pose_control_gains.png"), dpi=200); plt.close(fig)

    with open(os.path.join(DATA, "pose_control_check.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["case", "x0", "y0", "theta0_deg", "time_s", "e_p_mm", "e_theta_deg", "path_m", "max_wheel_rad_s"])
        w.writerows(rows)
    print(f"{'Ca':<22}{'t(s)':>7}{'e_p(mm)':>9}{'e_th(deg)':>10}{'path(m)':>9}{'max|w|':>8}")
    for r_ in rows:
        print(f"{r_[0]:<22}{r_[4]:>7}{r_[5]:>9}{r_[6]:>10}{r_[7]:>9}{r_[8]:>8}")
    print("Gains:", grows)


if __name__ == "__main__":
    main()
