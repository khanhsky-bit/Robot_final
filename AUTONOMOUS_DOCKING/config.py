"""Hằng số chuẩn của Interface Contract (dùng chung Khánh & Chinh).

Đơn vị: mét, giây, radian.
"""
import math

# --- Thông số vật lý robot ---
WHEEL_RADIUS = 0.05      # r  [m]
WHEEL_BASE = 0.30        # L  khoảng cách hai bánh [m]
BODY_LENGTH = 0.30       # chiều dài thân [m]
BODY_WIDTH = 0.24        # chiều rộng thân [m]
OMEGA_MAX = 10.0         # tốc độ góc bánh tối đa [rad/s]
DT = 0.02                # bước thời gian mô phỏng [s]

# --- Trạm sạc ---
Q_GOAL = (5.0, 4.5, 0.0)   # (x, y, theta_goal)
D_STAGE = 0.8              # khoảng lùi điểm chờ [m]

# --- Hệ số khuếch đại Pose Control (quy ước Corke: k_rho>0, k_beta<0, k_alpha>k_rho) ---
K_RHO = 1.0
K_ALPHA = 3.0
K_BETA = -1.0


def wrap_angle(a):
    """Chuẩn hóa góc về [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def staging_point(q_goal=Q_GOAL, d_stage=D_STAGE):
    """Điểm chờ: lùi d_stage dọc trục hướng theta_goal."""
    x, y, th = q_goal
    return (x - d_stage * math.cos(th), y - d_stage * math.sin(th), th)

# --- Ngưỡng của FSM (Chinh) ---
APPROACH_TOL = 0.05                  # [m] vào ALIGN khi cách điểm chờ < ngưỡng này
ALIGN_POS_TOL = 0.02                 # [m] hết chỉnh vị trí tại điểm chờ
ALIGN_ANG_TOL = 1.0                  # [độ] sai số hướng cho phép để vào DOCKING
DOCK_POS_TOL = 0.005                 # [m] = 5 mm, điều kiện DOCKED
DOCK_ANG_TOL = 2.0                   # [độ]
DOCK_LAT_LIMIT = 0.05                # [m] lệch ngang tối đa so với trục trạm khi DOCKING
DOCK_ANG_LIMIT = 10.0                # [độ] lệch hướng tối đa khi DOCKING
SCALE_APPROACH = 1.0                 # v_max_scale từng giai đoạn
SCALE_ALIGN = 0.4
SCALE_DOCKING = 0.3
MAX_RETRIES = 2                      # số lần thử lại APPROACH sau lỗi DOCKING
T_MAX = 90.0                         # [s] quá thời gian => ERROR
