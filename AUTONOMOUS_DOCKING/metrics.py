"""Chỉ tiêu đánh giá (Công việc 2 - Chinh)."""
import math
import numpy as np

import config as C


def final_position_error_mm(log):
    return float(np.hypot(log["x"][-1] - C.Q_GOAL[0], log["y"][-1] - C.Q_GOAL[1]) * 1000.0)


def final_angle_error_deg(log):
    return abs(math.degrees(C.wrap_angle(log["theta"][-1] - C.Q_GOAL[2])))


def completion_time_s(log):
    return float(log["t"][-1])


def path_length_m(log):
    return float(np.sum(np.hypot(np.diff(log["x"]), np.diff(log["y"]))))


def smoothness_index(log):
    """Tổng biến thiên vận tốc góc  sum |omega[k+1] - omega[k]|  [rad/s]. Càng nhỏ càng mượt."""
    return float(np.sum(np.abs(np.diff(log["omega"]))))


def state_at_distance(log, d=C.D_STAGE):
    """Lệch ngang [mm] và lệch góc [độ] tại thời điểm robot cách trạm lần đầu <= d (chất lượng tư thế vào trạm)."""
    idx = np.where(log["rho_goal"] <= d)[0]
    i = int(idx[0]) if len(idx) else len(log["t"]) - 1
    return float(abs(log["e_lat"][i]) * 1000.0), abs(math.degrees(log["e_theta"][i]))


def compute_metrics(log):
    lat, ang = state_at_distance(log)
    return {
        "final_state": log["final_state"],
        "e_p_mm": final_position_error_mm(log),
        "e_theta_deg": final_angle_error_deg(log),
        "time_s": completion_time_s(log),
        "path_m": path_length_m(log),
        "smoothness": smoothness_index(log),
        "lat_at_0p8m_mm": lat,
        "ang_at_0p8m_deg": ang,
        "max_wheel_rad_s": float(max(np.max(np.abs(log["wR"])), np.max(np.abs(log["wL"])))),
    }
