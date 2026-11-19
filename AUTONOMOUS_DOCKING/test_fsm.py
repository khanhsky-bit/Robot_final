"""Kiểm thử nhanh FSM:  python test_fsm.py"""
import math
import config as C
from simulation import simulate
from metrics import compute_metrics
from main import SCENARIOS


def test_five_scenarios_docked():
    for name, s in SCENARIOS:
        lg = simulate(s, "fsm"); m = compute_metrics(lg)
        assert lg["final_state"] == "DOCKED", name
        assert m["e_p_mm"] <= 5.0 + 1e-6 and m["e_theta_deg"] <= C.DOCK_ANG_TOL, name
        order = [e[2] for e in lg["events"]]
        assert order == ["APPROACH", "ALIGN", "DOCKING", "DOCKED"], (name, order)


def test_timeout_gives_error():
    old = C.T_MAX; C.T_MAX = 3.0
    try:
        lg = simulate(SCENARIOS[4][1], "fsm", t_max=10)
        assert lg["final_state"] == "ERROR"
    finally:
        C.T_MAX = old


def test_disturbance_retry_then_docked():
    lg = simulate(SCENARIOS[0][1], "fsm", disturbance=(12.0, 0, 0.10, 0))
    assert lg["final_state"] == "DOCKED"
    assert any(e[1] == "DOCKING" and e[2] == "APPROACH" for e in lg["events"])


def test_no_retries_left_gives_error():
    old = C.MAX_RETRIES; C.MAX_RETRIES = 0
    try:
        lg = simulate(SCENARIOS[0][1], "fsm", disturbance=(12.0, 0, 0.10, 0))
        assert lg["final_state"] == "ERROR"
    finally:
        C.MAX_RETRIES = old


if __name__ == "__main__":
    for f in (test_five_scenarios_docked, test_timeout_gives_error,
              test_disturbance_retry_then_docked, test_no_retries_left_gives_error):
        f(); print("PASS", f.__name__)
