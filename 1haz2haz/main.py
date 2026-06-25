# MAIN.PY
import sys
import time
import numpy as np
from PyQt6.QtWidgets import QApplication

from GUI.gui_window import MainWindow

import GENERAL_CONTROL.Trajectory as trajectory
from GENERAL_CONTROL.Agrion_PRRR_IK import Agrion_PRRR_IK 
from GENERAL_CONTROL.Agrion_PRRR_FK import Agrion_PRRR_FK 
from GENERAL_CONTROL.debug_logger import save_trajectory_debug_log, save_trajectory_frames_bin

import COMM.uart_comm as uart
from COMM.uart_comm import RobotMode
from config_init import OBSTACLES, HOME_Position, WAIT_Position, UART_PORT, UART_BAUD

# =============================================================================
# CONFIG
# =============================================================================
Home_x = HOME_Position['x']
Home_y = HOME_Position['y']
Home_z = HOME_Position['z']
Home_phi = (HOME_Position['phi'])

HOME_MAGIC = 12345.0
N_PER_SEG  = 5
DEFAULT_T  = 2.0

# TeachPad waypoint listesi
teach_waypoints: list[tuple] = []


# =============================================================================
# HELPERS
# =============================================================================
def fk_traj_from_joints(d1_traj, Q2_traj, Q3_traj, Q4_traj):
    x_traj, y_traj, z_traj = [], [], []
    for d1i, Q2i, Q3i, Q4i in zip(d1_traj, Q2_traj, Q3_traj, Q4_traj):
        xi, yi, zi = Agrion_PRRR_FK.solve(float(d1i), float(Q2i), float(Q3i), float(Q4i))
        x_traj.append(xi); y_traj.append(yi); z_traj.append(zi)
    return x_traj, y_traj, z_traj


def plot_all(window, t_vals, x_traj, y_traj, z_traj,
             d1_traj, Q2_traj, Q3_traj, Q4_traj):
    t_arr = np.array(t_vals, dtype=float)
    x_arr = np.array(x_traj, dtype=float)
    y_arr = np.array(y_traj, dtype=float)
    z_arr = np.array(z_traj, dtype=float)
    x_vel = np.gradient(x_arr, t_arr); y_vel = np.gradient(y_arr, t_arr); z_vel = np.gradient(z_arr, t_arr)
    x_acc = np.gradient(x_vel, t_arr); y_acc = np.gradient(y_vel, t_arr); z_acc = np.gradient(z_vel, t_arr)
    window.update_joint_display(d1_traj, Q2_traj, Q3_traj, Q4_traj)
    window.plot_path_and_robot(x_traj, y_traj, z_traj,
                                d1_traj[-1], Q2_traj[-1], Q3_traj[-1], Q4_traj[-1])
    window.plot_time_graphs_xyz(t_arr, x_arr, y_arr, z_arr,
                                 x_vel, y_vel, z_vel, x_acc, y_acc, z_acc)


def joints_from_waypoints_xyz(waypoints_xyz, phi_rad):
    q_list = []
    for p in waypoints_xyz:
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        q_list.append(Agrion_PRRR_IK.solve(x, y, z, phi_rad, elbow="up"))
    return q_list


def quintic_through_joint_waypoints(q_list, T_total, n_per_seg):
    seg_count = len(q_list) - 1
    T_seg     = T_total / seg_count
    t_all = []; d1_all, Q2_all, Q3_all, Q4_all = [], [], [], []
    t_offset = 0.0
    for i in range(seg_count):
        t_vals, d1, Q2, Q3, Q4 = trajectory.quintic_joint_trajectory(
            q_list[i], q_list[i+1], T=T_seg, n=n_per_seg
        )
        if i > 0:
            t_vals = t_vals[1:]; d1, Q2, Q3, Q4 = d1[1:], Q2[1:], Q3[1:], Q4[1:]
        t_all  += [t_offset + float(tv) for tv in t_vals]
        d1_all += list(d1); Q2_all += list(Q2)
        Q3_all += list(Q3); Q4_all += list(Q4)
        t_offset = t_all[-1]
    return t_all, d1_all, Q2_all, Q3_all, Q4_all


def _compute_dt(t_vals):
    if len(t_vals) < 2:
        return 0.02
    T  = t_vals[-1] - t_vals[0]
    dt = T / (len(t_vals) - 1)
    return max(0.02, min(dt, 0.1))


# =============================================================================
# MAIN HANDLER
# =============================================================================
def handle_action(mode, x, y, z, phi, t, window, ser):
    """
    Tanınan mod stringleri (büyük/küçük harf fark etmez):
        HOME          – STM32'ye homing frame'i gönder
        CALCULATE     – IK + quintic hesapla, grafikleri güncelle
        SEND          – Son hesaplanan joint pozisyonunu UART ile gönder
        TEACH_SAVE    – Mevcut joint pozisyonunu waypoint olarak kaydet
        TEACH_CLEAR   – Python waypoint listesini temizle
        TEACH_RUN     – Kayıtlı waypoint'leri MANUAL frame olarak gönder
        AUTO          – STM32'ye otonom çalıştırma komutu gönder
        VISION_PICK   – Tek bir hedef için CALCULATE + SEND zinciri (vision sekmesinden)
    """
    global teach_waypoints

    try:
        mode = str(mode).upper()

        # ── HOME ─────────────────────────────────────────────────────────────
        if mode == "HOME":
            if ser is None:
                window.show_error("UART port açık değil. HOME gönderilemedi.")
                return
            phi_rad = np.deg2rad(Home_phi)
            d1, Q2, Q3, Q4 = Agrion_PRRR_IK.solve(Home_x, Home_y, Home_z, phi_rad, elbow="up")
            uart.send_frame(ser, d1, Q2, Q3, Q4, 0.0, mode=RobotMode.MANUAL, debug=True)

            window.current_x   = Home_x
            window.current_y   = Home_y
            window.current_z   = Home_z
            window.current_phi = phi_rad
            window.x_input.setText(str(Home_x))
            window.y_input.setText(str(Home_y))
            window.z_input.setText(str(Home_z))
            window.phi_input.setText(str(Home_phi))
            return

        # ── SEND ─────────────────────────────────────────────────────────────
        if mode == "SEND":
            if ser is None:
                window.show_error("UART port açık değil. Gönderilemedi.")
                return
            if window.last_joint_traj is None:
                window.show_error("Önce 'Hesapla' yapmalısın. Gönderilecek veri yok.")
                return

            d1_list, Q2_list, Q3_list, Q4_list, grip_cmd = window.last_joint_traj

            # Gripper komutu: pick_place'den geliyorsa window.gripper_cmd güncel
            grip_cmd = float(getattr(window, "gripper_cmd", grip_cmd))

            d1 = float(d1_list[-1]); Q2 = float(Q2_list[-1])
            Q3 = float(Q3_list[-1]); Q4 = float(Q4_list[-1])

            uart.send_frame(
                ser,
                d1, Q2, Q3, Q4,
                gripper_cmd=grip_cmd,
                mode=RobotMode.MANUAL,
                debug=True,
            )

            window.gripper_cmd = 0.0
            if hasattr(window, "grip_label"):
                window.grip_label.setText("Gripper : Idle")
            return

        # ── CALCULATE / WAIT ─────────────────────────────────────────────────
        if mode in ("CALCULATE", "WAIT"):
            x       = float(x)
            y       = float(y)
            z       = float(z)
            t       = float(t)   if str(t).strip()   != "" else DEFAULT_T
            phi_deg = float(phi) if str(phi).strip()  != "" else 0.0
            phi_rad = np.deg2rad(phi_deg)

            x0, y0, z0 = window.current_x, window.current_y, window.current_z
            phi0        = getattr(window, "current_phi", phi_rad)

            d1_0, Q2_0, Q3_0, Q4_0 = Agrion_PRRR_IK.solve(x0, y0, z0, phi0,    elbow="up")
            d1_1, Q2_1, Q3_1, Q4_1 = Agrion_PRRR_IK.solve(x,  y,  z,  phi_rad, elbow="up")

            t_vals, d1_traj, Q2_traj, Q3_traj, Q4_traj = \
                trajectory.quintic_joint_trajectory(
                    [d1_0, Q2_0, Q3_0, Q4_0],
                    [d1_1, Q2_1, Q3_1, Q4_1],
                    T=t, n=200,
                )

            x_traj, y_traj, z_traj = [], [], []
            for d1i, Q2i, Q3i, Q4i in zip(d1_traj, Q2_traj, Q3_traj, Q4_traj):
                xi, yi, zi = Agrion_PRRR_FK.solve(float(d1i), float(Q2i),
                                                   float(Q3i), float(Q4i))
                x_traj.append(xi); y_traj.append(yi); z_traj.append(zi)

            t_arr = np.array(t_vals, dtype=float)
            x_arr = np.array(x_traj, dtype=float)
            y_arr = np.array(y_traj, dtype=float)
            z_arr = np.array(z_traj, dtype=float)

            x_vel = np.gradient(x_arr, t_arr)
            y_vel = np.gradient(y_arr, t_arr)
            z_vel = np.gradient(z_arr, t_arr)
            x_acc = np.gradient(x_vel, t_arr)
            y_acc = np.gradient(y_vel, t_arr)
            z_acc = np.gradient(z_vel, t_arr)

            window.update_joint_display(
                list(d1_traj), list(Q2_traj), list(Q3_traj), list(Q4_traj)
            )
            window.plot_path_and_robot(
                x_traj, y_traj, z_traj,
                d1_traj[-1], Q2_traj[-1], Q3_traj[-1], Q4_traj[-1],
            )
            window.plot_time_graphs_xyz(
                t_arr, x_arr, y_arr, z_arr,
                x_vel, y_vel, z_vel, x_acc, y_acc, z_acc,
            )

            grip_cmd = float(getattr(window, "gripper_cmd", 0.0))
            window.last_joint_traj = (
                list(d1_traj), list(Q2_traj),
                list(Q3_traj), list(Q4_traj),
                grip_cmd,
            )
            window.last_t_vals = t_vals
            window.current_x, window.current_y, window.current_z = x, y, z
            window.current_phi = phi_rad
            return

        # ── TEACH_SAVE ───────────────────────────────────────────────────────
        if mode == "TEACH_SAVE":
            if ser is None:
                window.show_error("UART port açık değil. Waypoint gönderilemedi.")
                return
            if window.last_joint_traj is None:
                window.show_error("Kaydedilecek pozisyon yok. Önce 'Hesapla' yapın.")
                return

            d1_list, Q2_list, Q3_list, Q4_list, grip_cmd = window.last_joint_traj
            d1  = float(d1_list[-1]); Q2  = float(Q2_list[-1])
            Q3  = float(Q3_list[-1]); Q4  = float(Q4_list[-1])
            grp = float(grip_cmd)

            uart.send_frame(ser, d1, Q2, Q3, Q4, grp,
                            mode=RobotMode.TEACHPAD, debug=True)
            teach_waypoints.append((d1, Q2, Q3, Q4, grp))
            print(f"[TEACH] Waypoint #{len(teach_waypoints)} kaydedildi: "
                  f"d1={d1:.2f} Q2={Q2:.2f} Q3={Q3:.2f} Q4={Q4:.2f} grip={grp:.2f}")
            if hasattr(window, "teach_label"):
                window.teach_label.setText(f"Teach waypoints: {len(teach_waypoints)}")
            return

        # ── TEACH_CLEAR ──────────────────────────────────────────────────────
        if mode == "TEACH_CLEAR":
            teach_waypoints.clear()
            print("[TEACH] Waypoint listesi temizlendi.")
            if hasattr(window, "teach_label"):
                window.teach_label.setText("Teach waypoints: 0")
            if ser is not None:
                uart.send_frame(ser, -1.0, 0.0, 0.0, 0.0, 0.0,
                                mode=RobotMode.TEACHPAD, debug=True)
            return

        # ── TEACH_RUN ────────────────────────────────────────────────────────
        if mode == "TEACH_RUN":
            if ser is None:
                window.show_error("UART port açık değil.")
                return
            if not teach_waypoints:
                window.show_error("Kayıtlı waypoint yok. Önce TEACH_SAVE kullanın.")
                return
            print(f"[TEACH_RUN] {len(teach_waypoints)} waypoint gönderiliyor...")
            for i, (d1, Q2, Q3, Q4, grp) in enumerate(teach_waypoints):
                # 1) Konum paketi  (gripper=0)
                # 2) Aynı konum + gripper komutu
                uart.send_with_gripper(
                    ser,
                    d1, Q2, Q3, Q4,
                    gripper_cmd=grp,
                    mode=RobotMode.MANUAL,
                    delay=0.05,
                    debug=False,
                )
                print(f"  [{i+1}/{len(teach_waypoints)}] d1={d1:.2f} Q2={Q2:.2f} "
                      f"Q3={Q3:.2f} Q4={Q4:.2f} grip={grp:.2f}")
                time.sleep(0.05)
            print("[TEACH_RUN] Tamamlandı.")
            return

        # ── AUTO ─────────────────────────────────────────────────────────────
        if mode == "AUTO":
            if ser is None:
                window.show_error("UART port açık değil. AUTO gönderilemedi.")
                return
            uart.send_frame(ser, 0.0, 0.0, 0.0, 0.0, 0.0,
                            mode=RobotMode.AUTO, debug=True)
            print("[AUTO] STM32'ye otonom çalıştırma komutu gönderildi.")
            return

        # ── VISION_PICK ──────────────────────────────────────────────────────
        # pick_place.py'deki step_request sinyali bu moda düşer.
        # "CALCULATE" veya "SEND" olarak gelir, zaten yukarıda işlenir.
        # Bu blok bilinmeyen VISION_* modları için bir güvenlik ağı.
        if mode.startswith("VISION_"):
            print(f"[VISION] Mod alındı: {mode} — işlem yok (alt mod gerekli)")
            return

        # ── Bilinmeyen mod ───────────────────────────────────────────────────
        window.show_error(f"Bilinmeyen mod: {mode}")

    except Exception as e:
        window.show_error(f"Hata: {e}")
        import traceback
        traceback.print_exc()


# =============================================================================
# APP
# =============================================================================
def main():
    app = QApplication(sys.argv)

    ser = None
    try:
        ser = uart.open_port(UART_PORT, UART_BAUD)
    except Exception as e:
        print(f"UART açılamadı: {e}")

    def callback(mode, x, y, z, phi, t, window):
        return handle_action(mode, x, y, z, phi, t, window, ser)

    window = MainWindow(callback)

    # DONE ack callback — runs in worker thread, no GUI involvement
    def ack_callback():
        return uart.wait_for_done(ser, timeout=1.0, debug=True)
    window.ack_callback = ack_callback

    # Direct UART send callable — worker thread calls this after IK (bypasses GUI timing)
    def uart_send(d1, Q2, Q3, Q4, gripper_cmd):
        if ser is None:
            return
        uart.send_frame(ser, d1, Q2, Q3, Q4,
                        gripper_cmd=float(gripper_cmd),
                        mode=RobotMode.MANUAL, debug=True)
    window.uart_send = uart_send

    window.show()

    app.exec()

    if ser:
        uart.close_port(ser)


if __name__ == "__main__":
    main()
