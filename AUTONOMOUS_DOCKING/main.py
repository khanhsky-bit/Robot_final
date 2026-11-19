"""Tích hợp toàn bộ module và chạy 5 kịch bản xuất phát (Công việc 2 - Chinh).

    python main.py              # chạy 5 kịch bản, lưu ảnh + bảng số liệu
    python main.py --gif        # lưu thêm hoạt ảnh GIF (results/animations/)
    python main.py --show 4     # xem hoạt ảnh trực tiếp kịch bản số 4 (cần giao diện đồ họa)
"""
import argparse, csv, math, os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mp

import config as C
from metrics import compute_metrics
from simulation import simulate, plot_summary, animate, draw_station, STATE_COLORS
from robot_model import DifferentialDriveRobot

HERE = os.path.dirname(os.path.abspath(__file__))
FIG, DATA, ANI = (os.path.join(HERE, "results", d) for d in ("figures", "data", "animations"))
D = math.radians

SCENARIOS = [  # (tên, (x0, y0, theta0))
    ("1. Trước trạm",        (0.5, 4.5, 0.0)),
    ("2. Lệch trái",         (0.5, 6.0, 0.0)),
    ("3. Lệch phải",         (0.5, 3.0, 0.0)),
    ("4. Góc lệch lớn",      (1.5, 4.5, D(150))),
    ("5. Vị trí xa bất kỳ",  (1.0, 1.0, D(-120))),
]
COLS = ["final_state", "e_p_mm", "e_theta_deg", "time_s", "path_m", "smoothness",
        "lat_at_0p8m_mm", "ang_at_0p8m_deg", "max_wheel_rad_s"]


def slug(name):
    return "s" + name.split(".")[0]


def write_csv(path, rows, header):
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f); w.writerow(header); w.writerows(rows)


def fmt(x):
    return f"{x:.2f}" if isinstance(x, float) else str(x)


def run_all(make_gif=False):
    for d in (FIG, DATA, ANI):
        os.makedirs(d, exist_ok=True)
    results = []
    for name, start in SCENARIOS:
        lg = simulate(start, "fsm")
        base = simulate(start, "direct")
        mf, mb = compute_metrics(lg), compute_metrics(base)
        results.append((name, start, lg, base, mf, mb))
        plot_summary(lg, f"Kịch bản {name} – FSM: {lg['final_state']}", os.path.join(FIG, f"fig_{slug(name)}_summary.png"))
        write_csv(os.path.join(DATA, f"log_{slug(name)}.csv"),
                  zip(lg["t"], lg["x"], lg["y"], lg["theta"], lg["v"], lg["omega"], lg["wR"], lg["wL"],
                      lg["e_p"], lg["e_theta"], lg["e_lat"], lg["state"]),
                  ["t", "x", "y", "theta", "v", "omega", "wR", "wL", "e_p", "e_theta", "e_lat", "state"])
        if make_gif:
            animate(lg, title=f"Kịch bản {name}", save_path=os.path.join(ANI, f"{slug(name)}.gif"))
        print(f"{name:<22} {lg['final_state']:<8} t={mf['time_s']:6.2f}s  e_p={mf['e_p_mm']:.2f}mm  e_th={mf['e_theta_deg']:.3f}°")

    # --- bảng tổng hợp ---
    hdr = ["scenario", "x0", "y0", "theta0_deg"] + COLS
    write_csv(os.path.join(DATA, "summary_fsm.csv"),
              [[n, s[0], s[1], round(math.degrees(s[2]), 1)] + [round(mf[c], 3) if not isinstance(mf[c], str) else mf[c] for c in COLS]
               for n, s, _, _, mf, _ in results], hdr)
    write_csv(os.path.join(DATA, "summary_direct_baseline.csv"),
              [[n, s[0], s[1], round(math.degrees(s[2]), 1)] + [round(mb[c], 3) if not isinstance(mb[c], str) else mb[c] for c in COLS]
               for n, s, _, _, _, mb in results], hdr)
    with open(os.path.join(DATA, "summary_table.md"), "w", encoding="utf-8") as f:
        f.write("| Kịch bản | Phương pháp | Kết quả | e_p (mm) | e_θ (°) | Thời gian (s) | Quỹ đạo (m) | Độ mượt (rad/s) | Lệch ngang @0,8 m (mm) | Lệch góc @0,8 m (°) |\n")
        f.write("|---|---|---|---|---|---|---|---|---|---|\n")
        for n, _, _, _, mf, mb in results:
            for lab, m in (("FSM + Pose Control", mf), ("Pose Control thuần", mb)):
                f.write(f"| {n} | {lab} | {m['final_state']} | {m['e_p_mm']:.2f} | {m['e_theta_deg']:.2f} | {m['time_s']:.2f} | "
                        f"{m['path_m']:.2f} | {m['smoothness']:.2f} | {m['lat_at_0p8m_mm']:.1f} | {m['ang_at_0p8m_deg']:.2f} |\n")

    # --- hình: quỹ đạo cả 5 kịch bản ---
    fig, axs = plt.subplots(1, 2, figsize=(14, 6.5), sharey=True)
    cmap = plt.get_cmap("tab10")
    for ax, key, ttl in ((axs[0], 3, "FSM + Pose Control (điểm chờ → ghép)"), (axs[1], 4, "Pose Control thuần (không điểm chờ)")):
        draw_station(ax)
        for i, r in enumerate(results):
            lg = r[key - 0] if False else (r[2] if key == 3 else r[3])
            ax.plot(lg["x"], lg["y"], color=cmap(i), lw=1.8, label=r[0])
            ax.plot(*r[1][:2], "o", color=cmap(i))
            ax.arrow(r[1][0], r[1][1], .25 * math.cos(r[1][2]), .25 * math.sin(r[1][2]), head_width=.07, color=cmap(i))
        ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_title(ttl); ax.set_xlabel("x [m]")
        ax.set_xlim(0, 5.8); ax.set_ylim(0.3, 6.6)
    axs[0].set_ylabel("y [m]"); axs[0].legend(fontsize=8, loc="lower left")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig_all_trajectories.png"), dpi=170); plt.close(fig)

    # --- hình: khoảng 0,8 m cuối, FSM vs thuần ---
    fig, ax = plt.subplots(figsize=(9, 5))
    xs0 = C.staging_point()[0]
    for i, r in enumerate(results):
        for lg, ls in ((r[2], "-"), (r[3], "--")):
            m = lg["x"] >= xs0 - 0.6
            ax.plot(lg["x"][m], lg["y"][m] - C.Q_GOAL[1], ls, color=cmap(i), lw=1.5)
    draw_station(ax)
    ax.plot([], [], "k-", label="FSM"); ax.plot([], [], "k--", label="Pose Control thuần")
    ax.set_xlim(xs0 - 0.6, 5.35); ax.set_ylim(C.Q_GOAL[1] - .6, C.Q_GOAL[1] + .6)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend(); ax.grid(alpha=.3)
    ax.set_title("Đoạn cuối trước trạm: FSM (nét liền) vs Pose Control thuần (nét đứt)")
    ax.set_ylim(-0.6 + 0, 0.6 + 0); 
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig_final_approach_compare.png"), dpi=170); plt.close(fig)

    # --- biểu đồ cột chỉ tiêu ---
    fig, axs = plt.subplots(1, 4, figsize=(16, 4))
    names = [r[0].split(". ")[1] for r in results]; xs = np.arange(len(names)); w = .38
    for ax, key, lab in zip(axs, ("time_s", "path_m", "smoothness", "ang_at_0p8m_deg"),
                            ("Thời gian [s]", "Độ dài quỹ đạo [m]", "Độ mượt Σ|Δω| [rad/s]", "Lệch góc khi cách trạm 0,8 m [°]")):
        ax.bar(xs - w / 2, [r[4][key] for r in results], w, label="FSM")
        ax.bar(xs + w / 2, [r[5][key] for r in results], w, label="Pose Control thuần")
        ax.set_xticks(xs); ax.set_xticklabels(names, rotation=25, ha="right", fontsize=8); ax.set_title(lab, fontsize=10); ax.grid(axis="y", alpha=.3)
    axs[0].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig_metrics_compare.png"), dpi=170); plt.close(fig)
    return results


def fault_recovery_demo():
    """Mở rộng: robot bị đẩy lệch ngang khi đang DOCKING -> FSM phải lùi về APPROACH và thử lại."""
    start = SCENARIOS[0][1]
    lg = simulate(start, "fsm", disturbance=(12.0, 0.0, 0.10, 0.0))
    plot_summary(lg, f"Phục hồi lỗi: bị đẩy lệch 100 mm lúc t = 12 s – kết quả {lg['final_state']}",
                 os.path.join(FIG, "fig_fault_recovery.png"))
    print("\nPhục hồi lỗi:", lg["final_state"], f"t={lg['t'][-1]:.1f}s")
    for e in lg["events"]:
        print("  ", e)
    return lg


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--gif", action="store_true", help="lưu hoạt ảnh GIF")
    ap.add_argument("--show", type=int, default=0, help="xem hoạt ảnh trực tiếp kịch bản 1-5")
    a = ap.parse_args()
    if a.show:
        name, start = SCENARIOS[a.show - 1]
        animate(simulate(start, "fsm"), title=f"Kịch bản {name}", show=True)
    else:
        matplotlib.use("Agg")
        run_all(make_gif=a.gif)
        fault_recovery_demo()
        print("\nĐã lưu ảnh vào results/figures, số liệu vào results/data")
