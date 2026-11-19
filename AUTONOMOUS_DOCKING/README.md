# AUTONOMOUS_DOCKING – Mô phỏng hoàn chỉnh (Khánh + Chinh)

Điều khiển robot vi sai tự động tiếp cận trạm sạc: mô hình động học + Pose Control (tọa độ cực).

## Cấu trúc
| File | Nội dung |
|---|---|
| `config.py` | Hằng số Interface Contract (r, L, thân, ω_max, dt, q_goal, d_stage, hệ số k) |
| `robot_model.py` | Lớp `DifferentialDriveRobot`, cập nhật rời rạc, kiểm chứng 3 chuyển động |
| `pose_controller.py` | Lớp `PoseController`, hàm `compute_control(current_pose, target_pose, v_max_scale=1.0)` → `(v, omega, omega_R, omega_L, dist_error)` |
| `baseline_pythonrobotics.py` | Tái lập ví dụ gốc PythonRobotics `move_to_pose` |
| `demo_pose_control.py` | Kiểm chứng Pose Control vòng kín 5 tư thế xuất phát, xuất đồ thị + CSV |

| `fsm.py` | `DockingFSM`: START → APPROACH → ALIGN → DOCKING → DOCKED / ERROR, điểm chờ, thử lại khi lệch |
| `simulation.py` | Vòng kín robot + FSM, vẽ 2D (thân, 2 bánh, hướng, trạm, điểm chờ, quỹ đạo, bảng sai số), hoạt ảnh GIF |
| `metrics.py` | e_p (mm), e_θ (độ), thời gian, độ dài quỹ đạo, độ mượt Σ\|Δω\| |
| `main.py` | Chạy 5 kịch bản (FSM và Pose Control thuần), xuất ảnh + bảng số liệu |
| `test_fsm.py` | Kiểm thử FSM (5 kịch bản, quá thời gian, phục hồi lỗi, hết lượt thử lại) |

## Cài đặt trên Windows (PowerShell / VS Code)
```powershell
cd AUTONOMOUS_DOCKING
python -m venv .venv
.\.venv\Scripts\Activate.ps1      # nếu bị chặn: Set-ExecutionPolicy -Scope Process Bypass
pip install -r requirements.txt
```
Trong VS Code: `Ctrl+Shift+P` → *Python: Select Interpreter* → chọn `.venv`.

## Chạy
```powershell
python robot_model.py               # kiểm chứng 3 chuyển động
python baseline_pythonrobotics.py   # baseline PythonRobotics
python demo_pose_control.py         # kiểm chứng Pose Control
python pose_controller.py           # tự kiểm tra giao tiếp
python main.py                      # CHẠY CHÍNH: 5 kịch bản + bảng + đồ thị
python main.py --gif                # thêm hoạt ảnh GIF (results/animations/)
python main.py --show 4             # xem hoạt ảnh trực tiếp kịch bản 4 (cửa sổ Matplotlib)
python test_fsm.py                  # kiểm thử
```
Ảnh lưu ở `results/figures/`, số liệu ở `results/data/`.

## Quy ước
- Pose = `[x, y, theta]` (m, m, rad ∈ [−π, π]).
- Luật: `v = k_rho·rho`, `omega = k_alpha·alpha + k_beta·beta`, điều kiện `k_rho>0, k_beta<0, k_alpha>k_rho`
  (mã gốc PythonRobotics dùng `Kp_beta=3` với dấu trừ ⇒ tương đương `k_beta=-3`).
- Bão hòa: nếu `max(|ω_R|,|ω_L|) > ω_max·v_max_scale` thì co **cả hai** bánh cùng tỉ lệ (giữ độ cong). `v_max_scale ∈ (0,1]` để FSM giảm tốc ở giai đoạn DOCKING.
- Mặc định không lùi (`allow_reverse=False`): đích phía sau thì quay tại chỗ trước; đặt `True` để giống mã gốc.
- Dừng khi `dist_error` nhỏ (FSM tự quyết ngưỡng; demo dùng 5 mm).

## FSM
- Điểm chờ = q_goal lùi 0,8 m dọc trục: (4,2; 4,5; 0).
- APPROACH (tốc độ tối đa) → ALIGN khi cách điểm chờ < 50 mm; ALIGN (v_max_scale 0,4) chỉnh vị trí < 20 mm rồi quay tại chỗ tới |e_θ| ≤ 1°.
- DOCKING (v_max_scale 0,3): DOCKED khi e_p < 5 mm và |e_θ| ≤ 2°.
- Lệch ngang > 50 mm hoặc lệch góc > 10° khi DOCKING ⇒ lùi ra và thử lại (tối đa 2 lần), hết lượt hoặc quá 90 s ⇒ ERROR.
- Mọi ngưỡng nằm cuối `config.py`.
