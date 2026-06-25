from pathlib import Path
from datetime import datetime
import COMM.uart_comm as uart


def save_trajectory_debug_log(
    t_vals,
    x_traj, y_traj, z_traj,
    Q1_traj, d2_traj, Q3_traj, Q4_traj,
    gripper_cmd=0.0,
    out_dir="logs"
):
    """
    Her trajectory noktası için:
    idx | t | x y z | Q1 d2 Q3 Q4 | grip | frame_len | frame_hex
    text dosyasına yazar.
    """
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_path = Path(out_dir) / f"trajectory_debug_{ts}.txt"

    with open(file_path, "w", encoding="utf-8") as f:
        f.write("RPRR TRAJECTORY DEBUG LOG\n")
        f.write(f"created_at = {ts}\n\n")
        f.write(
            "idx\t"
            "t[s]\t"
            "x[mm]\t y[mm]\t z[mm]\t"
            "Q1[rad]\t d2[mm]\t Q3[rad]\t Q4[rad]\t"
            "grip\t"
            "frame_len\t"
            "frame_hex\n"
        )

        for i, (tt, x, y, z, q1, d2, q3, q4) in enumerate(
            zip(t_vals, x_traj, y_traj, z_traj, Q1_traj, d2_traj, Q3_traj, Q4_traj)
        ):
            frame = uart.build_frame(
                float(q1),
                float(d2),
                float(q3),
                float(q4),
                float(gripper_cmd)
            )

            f.write(
                f"{i}\t"
                f"{float(tt):.6f}\t"
                f"{float(x):.3f}\t {float(y):.3f}\t {float(z):.3f}\t"
                f"{float(q1):.6f}\t {float(d2):.3f}\t {float(q3):.6f}\t {float(q4):.6f}\t"
                f"{float(gripper_cmd):.1f}\t"
                f"{len(frame)}\t"
                f"{frame.hex()}\n"
            )

    print(f"[LOG] Trajectory debug kaydedildi: {file_path}")
    return str(file_path)


def save_trajectory_frames_bin(
    Q1_traj, d2_traj, Q3_traj, Q4_traj,
    gripper_cmd=0.0,
    out_dir="logs"
):
    """
    Tüm frame'leri ham binary olarak yazar.
    Her kayıt 24 byte.
    """
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_path = Path(out_dir) / f"trajectory_frames_{ts}.bin"

    with open(file_path, "wb") as f:
        for q1, d2, q3, q4 in zip(Q1_traj, d2_traj, Q3_traj, Q4_traj):
            frame = uart.build_frame(
                float(q1),
                float(d2),
                float(q3),
                float(q4),
                float(gripper_cmd)
            )
            f.write(frame)

    print(f"[LOG] Binary frame dump kaydedildi: {file_path}")
    return str(file_path)