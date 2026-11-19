"""Máy trạng thái hữu hạn cho quy trình ghép trạm (Công việc 2 - Chinh).

START -> APPROACH -> ALIGN -> DOCKING -> DOCKED
                \\-> ERROR (quá thời gian / lệch quá lớn / hết lượt thử lại)

* APPROACH : Pose Control tới điểm chờ q_staging, tốc độ tối đa.
* ALIGN    : chỉnh mịn tại điểm chờ (tốc độ giảm) rồi quay tại chỗ về hướng trạm.
* DOCKING  : tiến chậm theo trục hướng vào trạm; giám sát lệch ngang và lệch góc.
* Lỗi DOCKING (lệch vượt ngưỡng) => quay lại APPROACH, tối đa MAX_RETRIES lần;
  khi thử lại robot được phép lùi ra điểm chờ (không quay đầu 180° sát trạm).
"""
import math
from enum import Enum

import config as C
from config import wrap_angle
from pose_controller import PoseController


class State(Enum):
    START = "START"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    DOCKING = "DOCKING"
    DOCKED = "DOCKED"
    ERROR = "ERROR"


class DockingFSM:
    def __init__(self, q_goal=C.Q_GOAL, d_stage=C.D_STAGE, controller=None):
        self.q_goal = tuple(q_goal)
        self.q_staging = C.staging_point(q_goal, d_stage)
        self.ctrl = controller or PoseController()
        self.ctrl_rev = PoseController(allow_reverse=True)   # dùng khi thử lại: cho phép lùi ra theo trục trạm
        self.state = State.START
        self.retries = 0
        self.error_reason = ""
        self.t = 0.0
        self.history = []          # (t, from_state, to_state, lý do)

    # ---- sai số so với trạm ----
    def errors(self, pose):
        xg, yg, thg = self.q_goal
        x, y, th = pose
        e_p = math.hypot(xg - x, yg - y)
        e_th = wrap_angle(th - thg)
        e_lat = -(x - xg) * math.sin(thg) + (y - yg) * math.cos(thg)   # lệch ngang so với trục trạm
        return e_p, e_th, e_lat

    def _goto(self, new, why=""):
        self.history.append((round(self.t, 3), self.state.value, new.value, why))
        self.state = new

    def _fail(self, why):
        self.error_reason = why
        self._goto(State.ERROR, why)

    @staticmethod
    def _stop():
        return 0.0, 0.0, 0.0, 0.0

    # ---- bước điều khiển ----
    def step(self, pose, dt=C.DT):
        """Nhận tư thế đo được, trả (v, omega, wR, wL). Cập nhật trạng thái."""
        self.t += dt
        e_p, e_th, e_lat = self.errors(pose)

        if self.state not in (State.DOCKED, State.ERROR) and self.t > C.T_MAX:
            self._fail("Quá thời gian cho phép")

        if self.state == State.START:
            self._goto(State.APPROACH, "khởi tạo xong")

        if self.state == State.APPROACH:
            ctrl = self.ctrl_rev if self.retries > 0 else self.ctrl
            v, om, wR, wL, rho = ctrl.compute_control(pose, self.q_staging, C.SCALE_APPROACH)
            if rho < C.APPROACH_TOL:
                self._goto(State.ALIGN, f"cách điểm chờ {rho*1000:.0f} mm")
            return v, om, wR, wL

        if self.state == State.ALIGN:
            rho_s = math.hypot(self.q_staging[0] - pose[0], self.q_staging[1] - pose[1])
            if rho_s > C.ALIGN_POS_TOL:
                # trong ALIGN điểm chờ chỉ cách vài cm: cho phép lùi thay vì quay đầu
                v, om, wR, wL, _ = self.ctrl_rev.compute_control(pose, self.q_staging, C.SCALE_ALIGN)
                return v, om, wR, wL
            if abs(e_th) > math.radians(C.ALIGN_ANG_TOL):
                om = max(-2.0, min(2.0, -3.0 * e_th))          # quay tại chỗ
                v, om, wR, wL = self.ctrl.saturate(0.0, om, C.SCALE_ALIGN)
                return v, om, wR, wL
            self._goto(State.DOCKING, f"lệch ngang {e_lat*1000:.1f} mm, lệch góc {math.degrees(e_th):.2f}°")

        if self.state == State.DOCKING:
            if abs(e_lat) > C.DOCK_LAT_LIMIT or abs(e_th) > math.radians(C.DOCK_ANG_LIMIT):
                if self.retries < C.MAX_RETRIES:
                    self.retries += 1
                    self._goto(State.APPROACH, f"lệch quá ngưỡng, thử lại lần {self.retries}")
                    return self._stop()
                self._fail("Lệch quá ngưỡng, hết lượt thử lại")
                return self._stop()
            if e_p < C.DOCK_POS_TOL:
                if abs(e_th) <= math.radians(C.DOCK_ANG_TOL):
                    self._goto(State.DOCKED, f"e_p={e_p*1000:.1f} mm, e_θ={math.degrees(e_th):.2f}°")
                else:
                    self._fail("Tới trạm nhưng sai hướng")
                return self._stop()
            v, om, wR, wL, _ = self.ctrl.compute_control(pose, self.q_goal, C.SCALE_DOCKING)
            return v, om, wR, wL

        return self._stop()           # DOCKED / ERROR

    @property
    def finished(self):
        return self.state in (State.DOCKED, State.ERROR)
