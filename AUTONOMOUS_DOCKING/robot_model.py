"""Mô hình động học robot hai bánh vi sai (Công việc 1 - Khánh).

Phương trình liên tục:
    x'     = v cos(theta)
    y'     = v sin(theta)
    theta' = omega
với v = r (w_R + w_L) / 2 ,  omega = r (w_R - w_L) / L.

Rời rạc hóa (Euler) theo bước dt:
    theta_{k+1} = theta_k + omega dt
    x_{k+1}     = x_k + v cos(theta_k) dt
    y_{k+1}     = y_k + v sin(theta_k) dt
Tùy chọn method='midpoint' dùng góc trung điểm theta_k + omega dt/2 (chính xác hơn).
"""
import math
import numpy as np

import config as C


class DifferentialDriveRobot:
    def __init__(self, x=0.0, y=0.0, theta=0.0,
                 r=C.WHEEL_RADIUS, L=C.WHEEL_BASE,
                 length=C.BODY_LENGTH, width=C.BODY_WIDTH,
                 omega_max=C.OMEGA_MAX, dt=C.DT, method="euler"):
        if method not in ("euler", "midpoint"):
            raise ValueError("method phải là 'euler' hoặc 'midpoint'")
        self.r, self.L = r, L
        self.length, self.width = length, width
        self.omega_max, self.dt, self.method = omega_max, dt, method
        self.set_pose(x, y, theta)

    # ---- trạng thái ----
    def set_pose(self, x, y, theta):
        self.x, self.y = float(x), float(y)
        self.theta = C.wrap_angle(float(theta))

    @property
    def pose(self):
        return np.array([self.x, self.y, self.theta])

    # ---- quy đổi vận tốc ----
    def wheels_to_body(self, w_R, w_L):
        v = self.r * (w_R + w_L) / 2.0
        omega = self.r * (w_R - w_L) / self.L
        return v, omega

    def body_to_wheels(self, v, omega):
        w_R = (v + omega * self.L / 2.0) / self.r
        w_L = (v - omega * self.L / 2.0) / self.r
        return w_R, w_L

    def saturate_wheels(self, w_R, w_L):
        """Giới hạn |w| <= omega_max, co cả hai bánh cùng tỉ lệ để giữ bán kính cong."""
        m = max(abs(w_R), abs(w_L))
        if m > self.omega_max:
            s = self.omega_max / m
            return w_R * s, w_L * s
        return w_R, w_L

    # ---- cập nhật trạng thái ----
    def step_vw(self, v, omega, dt=None):
        dt = self.dt if dt is None else dt
        th_mid = self.theta + (0.5 * omega * dt if self.method == "midpoint" else 0.0)
        self.x += v * math.cos(th_mid) * dt
        self.y += v * math.sin(th_mid) * dt
        self.theta = C.wrap_angle(self.theta + omega * dt)
        return self.pose

    def step_wheels(self, w_R, w_L, dt=None):
        w_R, w_L = self.saturate_wheels(w_R, w_L)
        v, omega = self.wheels_to_body(w_R, w_L)
        return self.step_vw(v, omega, dt)

    # ---- hình học thân (phục vụ vẽ mô phỏng) ----
    def body_corners(self):
        """4 góc thân chữ nhật trong hệ toàn cục (tâm trục bánh = tâm thân)."""
        hl, hw = self.length / 2, self.width / 2
        pts = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
        c, s = math.cos(self.theta), math.sin(self.theta)
        R = np.array([[c, -s], [s, c]])
        return pts @ R.T + np.array([self.x, self.y])


# ---------------------------------------------------------------------------
# Kịch bản kiểm chứng 3 chuyển động cơ bản
# ---------------------------------------------------------------------------
def simulate_wheels(w_R, w_L, T, method="euler", dt=C.DT, pose0=(0, 0, 0)):
    rb = DifferentialDriveRobot(*pose0, dt=dt, method=method)
    n = int(round(T / dt))
    traj = np.zeros((n + 1, 3))
    traj[0] = rb.pose
    for k in range(n):
        traj[k + 1] = rb.step_wheels(w_R, w_L)
    return traj


def analytic_pose(w_R, w_L, t, r=C.WHEEL_RADIUS, L=C.WHEEL_BASE, pose0=(0, 0, 0)):
    """Nghiệm giải tích với w_R, w_L không đổi (cung tròn / đường thẳng / quay tại chỗ)."""
    x0, y0, th0 = pose0
    v = r * (w_R + w_L) / 2
    om = r * (w_R - w_L) / L
    if abs(om) < 1e-12:
        return np.array([x0 + v * math.cos(th0) * t, y0 + v * math.sin(th0) * t, th0])
    th = th0 + om * t
    return np.array([x0 + v / om * (math.sin(th) - math.sin(th0)),
                     y0 - v / om * (math.cos(th) - math.cos(th0)),
                     C.wrap_angle(th)])


MOTIONS = {
    "Đi thẳng":      dict(w_R=8.0, w_L=8.0, T=5.0),     # v = 0.4 m/s
    "Quay tại chỗ":  dict(w_R=6.0, w_L=-6.0, T=5.0),    # omega = 2.0 rad/s
    "Chạy đường cong": dict(w_R=8.0, w_L=5.0, T=8.0),   # cung tròn
}


def verify_basic_motions(save_dir=None, verbose=True):
    """Chạy 3 chuyển động, so sánh Euler/midpoint với nghiệm giải tích, vẽ đồ thị."""
    import matplotlib
    if save_dir:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = []
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.6))
    for ax, (name, p) in zip(axes, MOTIONS.items()):
        te = simulate_wheels(p["w_R"], p["w_L"], p["T"], "euler")
        tm = simulate_wheels(p["w_R"], p["w_L"], p["T"], "midpoint")
        ts = np.arange(len(te)) * C.DT
        ta = np.array([analytic_pose(p["w_R"], p["w_L"], t) for t in ts])
        err_e = np.hypot(*(te[-1, :2] - ta[-1, :2]))
        err_m = np.hypot(*(tm[-1, :2] - ta[-1, :2]))
        err_th = abs(C.wrap_angle(te[-1, 2] - ta[-1, 2]))
        v = C.WHEEL_RADIUS * (p["w_R"] + p["w_L"]) / 2
        om = C.WHEEL_RADIUS * (p["w_R"] - p["w_L"]) / C.WHEEL_BASE
        rows.append((name, p["w_R"], p["w_L"], v, om, err_e * 1000, err_m * 1000, math.degrees(err_th)))

        ax.plot(ta[:, 0], ta[:, 1], "k-", lw=3, alpha=.25, label="Giải tích")
        ax.plot(te[:, 0], te[:, 1], "b--", lw=1.4, label="Euler (dt=0.02)")
        for pose, col, lab in ((te[0], "tab:green", "Robot lúc đầu"), (te[-1], "tab:orange", "Robot lúc cuối")):
            poly = DifferentialDriveRobot(*pose).body_corners()
            ax.fill(poly[:, 0], poly[:, 1], color=col, alpha=.55, label=lab)
            ax.arrow(pose[0], pose[1], .15 * math.cos(pose[2]), .15 * math.sin(pose[2]),
                     head_width=.04, color="r", length_includes_head=True)
        allx = np.concatenate([te[:, 0], [-.3, .3]]); ally = np.concatenate([te[:, 1], [-.3, .3]])
        ax.set_xlim(allx.min() - .25, allx.max() + .25); ax.set_ylim(ally.min() - .25, ally.max() + .25)
        ax.set_aspect("equal"); ax.grid(alpha=.3)
        ax.set_title(f"{name}\n$\\omega_R$={p['w_R']}, $\\omega_L$={p['w_L']} rad/s")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.legend(fontsize=7, loc="upper center", bbox_to_anchor=(.5, -.32), ncol=2)
    fig.suptitle("Kiểm chứng mô hình động học: 3 chuyển động cơ bản", fontsize=13)
    fig.tight_layout()
    if verbose:
        print(f"{'Chuyển động':<18}{'wR':>5}{'wL':>5}{'v(m/s)':>8}{'om(rad/s)':>10}"
              f"{'e_Euler(mm)':>13}{'e_Mid(mm)':>11}{'e_th(deg)':>10}")
        for r_ in rows:
            print(f"{r_[0]:<18}{r_[1]:>5.1f}{r_[2]:>5.1f}{r_[3]:>8.3f}{r_[4]:>10.3f}"
                  f"{r_[5]:>13.3f}{r_[6]:>11.4f}{r_[7]:>10.4f}")
    if save_dir:
        import os, csv
        os.makedirs(save_dir, exist_ok=True)
        fig.savefig(os.path.join(save_dir, "fig_kinematics_3motions.png"), dpi=200)
    else:
        plt.show()
    plt.close(fig)
    return rows


if __name__ == "__main__":
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    rows = verify_basic_motions(save_dir=os.path.join(here, "results", "figures"))
    import csv
    os.makedirs(os.path.join(here, "results", "data"), exist_ok=True)
    with open(os.path.join(here, "results", "data", "kinematics_verification.csv"), "w",
              newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["motion", "w_R", "w_L", "v", "omega", "err_pos_euler_mm", "err_pos_midpoint_mm", "err_theta_deg"])
        w.writerows(rows)
