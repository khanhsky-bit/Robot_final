"""Môi trường mô phỏng 2D bằng Matplotlib (Công việc 2 - Chinh).

* simulate(...)    : chạy vòng kín robot + (FSM | Pose Control thuần), trả về log.
* draw_scene(...)  : vẽ thân robot, 2 bánh, mũi tên hướng, trạm sạc, điểm chờ, quỹ đạo, bảng sai số.
* animate(...)     : hoạt ảnh (hiển thị hoặc lưu GIF).
* plot_summary(...): ảnh tổng hợp quỹ đạo + sai số + vận tốc cho báo cáo.
"""
import math
import numpy as np
import matplotlib.patches as mp
import matplotlib.pyplot as plt

import config as C
from fsm import DockingFSM, State
from pose_controller import PoseController
from robot_model import DifferentialDriveRobot

STATE_COLORS = {"START": "#9e9e9e", "APPROACH": "#1f77b4", "ALIGN": "#ff7f0e",
                "DOCKING": "#2ca02c", "DOCKED": "#006400", "ERROR": "#d62728", "DIRECT": "#7f7f7f"}
KEYS = ("t", "x", "y", "theta", "v", "omega", "wR", "wL", "e_p", "e_theta", "e_lat", "rho_goal")


# ---------------------------------------------------------------------------
def simulate(start, mode="fsm", disturbance=None, t_max=C.T_MAX, dt=C.DT, **fsm_kwargs):
    """mode='fsm' : quy trình đầy đủ; mode='direct': Pose Control thuần tới q_goal (đường cơ sở).
    disturbance=(t, dx, dy, dtheta): "đẩy" robot tại thời điểm t (kiểm tra khả năng phục hồi)."""
    rb = DifferentialDriveRobot(*start, dt=dt)
    fsm = DockingFSM(**fsm_kwargs) if mode == "fsm" else None
    ctrl = PoseController()
    log = {k: [] for k in KEYS}
    log["state"] = []
    pushed = False
    n = int(t_max / dt)
    for k in range(n):
        t = k * dt
        if disturbance and not pushed and t >= disturbance[0]:
            rb.set_pose(rb.x + disturbance[1], rb.y + disturbance[2], rb.theta + disturbance[3])
            pushed = True
        pose = rb.pose
        if mode == "fsm":
            v, om, wR, wL = fsm.step(pose, dt)
            st = fsm.state.value
            e_p, e_th, e_lat = fsm.errors(pose)
        else:
            v, om, wR, wL, rho = ctrl.compute_control(pose, C.Q_GOAL)
            st = "DIRECT"
            e_p, e_th, e_lat = DockingFSM().errors(pose)
        for key, val in zip(KEYS, (t, pose[0], pose[1], pose[2], v, om, wR, wL, e_p, e_th, e_lat, e_p)):
            log[key].append(val)
        log["state"].append(st)
        if mode == "fsm" and fsm.finished:
            break
        if mode == "direct" and e_p < C.DOCK_POS_TOL:
            break
        rb.step_wheels(wR, wL)
    out = {k: np.array(log[k]) for k in KEYS}
    out["state"] = np.array(log["state"])
    out["final_state"] = fsm.state.value if fsm else ("DOCKED" if e_p < C.DOCK_POS_TOL else "TIMEOUT")
    out["events"] = fsm.history if fsm else []
    out["start"] = tuple(start)
    return out


# ---------------------------------------------------------------------------
def _wheel_patches(pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    out = []
    for side in (+1, -1):
        cx = x - side * (C.WHEEL_BASE / 2) * s
        cy = y + side * (C.WHEEL_BASE / 2) * c
        hl, hw = C.WHEEL_RADIUS, 0.012
        pts = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
        R = np.array([[c, -s], [s, c]])
        out.append(pts @ R.T + np.array([cx, cy]))
    return out


def draw_station(ax):
    xg, yg, thg = C.Q_GOAL
    plate_x = xg + C.BODY_LENGTH / 2           # mặt trạm chạm mũi robot khi đã ghép
    ax.add_patch(mp.Rectangle((plate_x, yg - 0.25), 0.10, 0.50, color="#2e7d32", zorder=2))
    ax.plot([plate_x, plate_x], [yg - 0.04, yg + 0.04], color="gold", lw=3, zorder=3)  # tiếp điểm sạc
    ax.plot([C.staging_point()[0], plate_x], [yg, yg], ls=":", color="gray", lw=1, zorder=1)  # trục trạm
    sx, sy, _ = C.staging_point()
    ax.plot(sx, sy, marker="^", color="m", ms=9, ls="", zorder=4)


def draw_scene(ax, log, i, trail=True, panel=True):
    pose = (log["x"][i], log["y"][i], log["theta"][i])
    rb = DifferentialDriveRobot(*pose)
    ax.clear()
    draw_station(ax)
    if trail:
        ax.plot(log["x"][:i + 1], log["y"][:i + 1], "b-", lw=1.4, alpha=.8, zorder=2)
    ax.plot(*log["start"][:2], "ko", ms=5, zorder=3)
    ax.add_patch(mp.Polygon(rb.body_corners(), closed=True, fc="#ffcc80", ec="k", zorder=5))
    for w in _wheel_patches(pose):
        ax.add_patch(mp.Polygon(w, closed=True, fc="#37474f", zorder=6))
    ax.arrow(pose[0], pose[1], 0.22 * math.cos(pose[2]), 0.22 * math.sin(pose[2]),
             head_width=0.07, color="r", length_includes_head=True, zorder=7)
    ax.set_aspect("equal"); ax.grid(alpha=.3)
    ax.set_xlim(0, 5.8); ax.set_ylim(0.3, 6.6)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    if panel:
        st = log["state"][i]
        txt = (f"t = {log['t'][i]:5.1f} s    Trạng thái: {st}\n"
               f"e_p = {log['e_p'][i]*1000:7.1f} mm    e_θ = {math.degrees(C.wrap_angle(log['theta'][i]-C.Q_GOAL[2])):6.1f}°\n"
               f"v = {log['v'][i]:.2f} m/s   ω = {log['omega'][i]:+.2f} rad/s")
        ax.text(0.02, 0.98, txt, transform=ax.transAxes, va="top", family="monospace", fontsize=8.5,
                bbox=dict(fc="white", ec=STATE_COLORS.get(st, "k"), lw=2, alpha=.9), zorder=10)


def animate(log, title="", save_path=None, step=4, show=False):
    import matplotlib
    from matplotlib.animation import FuncAnimation, PillowWriter
    if not show:
        matplotlib.use("Agg")
    fig, ax = plt.subplots(figsize=(6.2, 7.0))
    frames = list(range(0, len(log["t"]), step)) + [len(log["t"]) - 1] * 15

    def upd(i):
        draw_scene(ax, log, i)
        ax.set_title(title)
    ani = FuncAnimation(fig, upd, frames=frames, interval=40)
    if save_path:
        ani.save(save_path, writer=PillowWriter(fps=15), dpi=70)
        plt.close(fig)
    elif show:
        plt.show()
    return ani


def _shade_states(ax, log):
    st, t = log["state"], log["t"]
    i0 = 0
    for i in range(1, len(st) + 1):
        if i == len(st) or st[i] != st[i0]:
            ax.axvspan(t[i0], t[min(i, len(t) - 1)], color=STATE_COLORS.get(st[i0], "w"), alpha=.12, lw=0)
            i0 = i


def plot_summary(log, title, path):
    fig = plt.figure(figsize=(14, 7.5))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.0, 1.25])
    ax = fig.add_subplot(gs[:, 0])
    n = len(log["t"])
    draw_scene(ax, log, n - 1, panel=False)
    for frac in (0.0, 0.25, 0.5, 0.75):                      # vài "ảnh chụp" robot dọc quỹ đạo
        i = int(frac * (n - 1))
        rb = DifferentialDriveRobot(log["x"][i], log["y"][i], log["theta"][i])
        ax.add_patch(mp.Polygon(rb.body_corners(), closed=True, fc="none", ec="gray", ls="--", zorder=4))
    # tô màu quỹ đạo theo trạng thái
    for name in dict.fromkeys(log["state"]):
        m = log["state"] == name
        ax.plot(log["x"][m], log["y"][m], ".", ms=2.5, color=STATE_COLORS.get(name, "k"), label=name, zorder=3)
    ax.legend(loc="lower left", fontsize=8, markerscale=4)
    ax.set_title(title)

    t = log["t"]
    a1 = fig.add_subplot(gs[0, 1]); _shade_states(a1, log)
    a1.plot(t, log["e_p"] * 1000, "b"); a1.set_ylabel("e_p [mm]"); a1.set_yscale("symlog", linthresh=10)
    a1.axhline(C.DOCK_POS_TOL * 1000, color="g", ls="--", lw=.8)
    a2 = fig.add_subplot(gs[1, 1], sharex=a1); _shade_states(a2, log)
    a2.plot(t, np.degrees(np.vectorize(C.wrap_angle)(log["e_theta"])), "r", label="e_θ")
    a2.plot(t, log["e_lat"] * 1000 / 10, "k", lw=1, label="e_lat [cm]")
    a2.set_ylabel("e_θ [độ]"); a2.legend(fontsize=7)
    a3 = fig.add_subplot(gs[2, 1], sharex=a1); _shade_states(a3, log)
    a3.plot(t, log["v"], label="v [m/s]"); a3.plot(t, log["omega"], label="ω [rad/s]")
    a3.plot(t, log["wR"] / 10, ":", label="ω_R/10"); a3.plot(t, log["wL"] / 10, ":", label="ω_L/10")
    a3.set_ylabel("Vận tốc"); a3.set_xlabel("t [s]"); a3.legend(fontsize=7, ncol=4)
    for a in (a1, a2, a3): a.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(path, dpi=170); plt.close(fig)
