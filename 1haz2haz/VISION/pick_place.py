# VISION/pick_place.py

import time
import numpy as np
from enum import Enum, auto
from PyQt6.QtCore import QThread, pyqtSignal

from config_init import cam_to_robot
from GENERAL_CONTROL.Agrion_PRRR_IK import Agrion_PRRR_IK

GRIPPER_WAIT = 1.5


class State(Enum):
    IDLE          = auto()
    MOVING_TO_TGT = auto()
    GRIPPING      = auto()
    MOVING_TO_BIN = auto()
    RELEASING     = auto()
    RETURNING     = auto()
    DONE          = auto()
    PAUSED        = auto()
    ESTOP         = auto()


class PickPlaceWorker(QThread):
    state_changed   = pyqtSignal(str)
    log_entry       = pyqtSignal(int, int, str, str)
    step_request    = pyqtSignal(str, float, float, float, float, float)
    gripper_request = pyqtSignal(float)
    cycle_done      = pyqtSignal()
    error_occurred  = pyqtSignal(str)
    step_confirmed  = pyqtSignal()
    ack_received    = pyqtSignal(str)   # "STM32: DONE ✓" — Process Steps'te gösterilir

    def __init__(self, targets: list, pos_a: dict, pos_b: dict,
                 run_mode: str = "full_auto",
                 gripper_wait: float = GRIPPER_WAIT,
                 ack_callback=None,
                 uart_send=None):
        super().__init__()
        self.targets      = targets
        self.pos_a        = pos_a
        self.pos_b        = pos_b
        self.run_mode     = run_mode
        self.gripper_wait = gripper_wait
        self.ack_callback = ack_callback
        self.uart_send    = uart_send
        self._running     = True
        self._paused      = False
        self._estop       = False
        self._step_ok     = False
        self._current_target = 0   # hangi hedef için DONE gösterilecek

    # ── External control ────────────────────────────────────────
    def pause(self):
        self._paused = True
        self._set_state(State.PAUSED)

    def resume(self):
        self._paused = False

    def emergency_stop(self):
        self._estop   = True
        self._running = False
        self._set_state(State.ESTOP)

    def confirm_step(self):
        self._step_ok = True

    # ── Main loop ────────────────────────────────────────────────
    def run(self):
        try:
            self._run_cycle()
        except Exception as e:
            self.error_occurred.emit(f"Pick-place error: {e}")

    def _run_cycle(self):
        olgun = [t for t in self.targets if t['class'] == 'fully']

        if not olgun:
            self.error_occurred.emit("No ripe targets in locked list. Aborting.")
            return

        self._set_state(State.IDLE)

        for target_idx, target in enumerate(olgun):
            if not self._running:
                break

            self._wait_if_paused()
            if self._estop:
                break

            coords = target['coords']
            t_no   = target_idx + 1
            self._current_target = t_no

            # Camera frame (cm) → mm → Robot frame (mm)
            x_cam_mm = float(coords[0]) * 10
            y_cam_mm = float(coords[1]) * 10
            z_cam_mm = float(coords[2]) * 10
            x_mm, y_mm, z_mm = cam_to_robot(x_cam_mm, y_cam_mm, z_cam_mm)

            # ── Step 1: Go to tomato AND grab (single packet, gripper=2) ──
            self._emit_log(t_no, 1, "Go to target", 'active')
            self._set_state(State.MOVING_TO_TGT)
            self._send_move(x_mm, y_mm, z_mm, phi=0.0, t=2.0, gripper_cmd=2.0)
            self._emit_log(t_no, 1, "Go to target", 'done')
            self._emit_log(t_no, 2, "Grab", 'done')

            if not self._running:
                break

            # ── Step 2: Go to basket AND release (single packet, gripper=1) ──
            self._emit_log(t_no, 3, "Carry to pool", 'active')
            self._set_state(State.MOVING_TO_BIN)
            self._send_move(self.pos_b['x'], self.pos_b['y'],
                            self.pos_b['z'], self.pos_b['phi'],
                            t=2.0, gripper_cmd=1.0)
            self._emit_log(t_no, 3, "Carry to pool", 'done')
            self._emit_log(t_no, 4, "Place", 'done')

        # ── All targets done → return to start ───────────────────
        if self._running and not self._estop:
            self._current_target = 0
            self._set_state(State.RETURNING)
            self.log_entry.emit(0, 0, "Return to start position", 'active')
            self._send_move(
                self.pos_a['x'], self.pos_a['y'],
                self.pos_a['z'], self.pos_a['phi'],
                t=2.0
            )
            self.log_entry.emit(0, 0, "Return to start position", 'done')
            self._set_state(State.DONE)
            self.cycle_done.emit()

    # ── Helpers ──────────────────────────────────────────────────
    def _set_state(self, state: State):
        self.state_changed.emit(state.name)

    def _emit_log(self, t_no: int, step_no: int, label: str, status: str):
        self.log_entry.emit(t_no, step_no, label, status)

    def _wait_for_ack(self, overall_timeout: float = 30.0):
        """
        Blocks until STM32 sends DONE. Runs in worker thread — GUI stays
        responsive and E-stop is honoured. Emits ack_received on success.
        """
        if self.ack_callback is None:
            self.msleep(100)
            return
        deadline = time.time() + overall_timeout
        while self._running and not self._estop:
            if self.ack_callback():
                self.ack_received.emit("STM32: DONE ✓")
                return
            if time.time() > deadline:
                self.error_occurred.emit("Timeout: no DONE from STM32.")
                return

    def _send_move(self, x: float, y: float, z: float,
                   phi: float, t: float, gripper_cmd: float = 0.0):
        """
        Performs IK in the worker thread, sends UART directly via uart_send
        callable (no GUI-thread timing dependency), then waits for DONE.
        Also emits CALCULATE signal so the trajectory plots update
        asynchronously in the GUI thread.
        """
        # IK in worker thread — no msleep dependency on GUI processing
        phi_rad = np.deg2rad(float(phi))
        try:
            d1, Q2, Q3, Q4 = Agrion_PRRR_IK.solve(
                float(x), float(y), float(z), phi_rad, elbow="up"
            )
        except Exception as e:
            self.error_occurred.emit(f"IK failed: {e}")
            return

        # Update display label (fire-and-forget)
        self.gripper_request.emit(float(gripper_cmd))

        # Send UART directly — thread-safe callable passed from main.py
        if self.uart_send is not None:
            self.uart_send(d1, Q2, Q3, Q4, float(gripper_cmd))
        else:
            # Fallback: go through GUI-thread SEND signal (old path)
            self.step_request.emit("SEND", x, y, z, phi, t)

        # Fire-and-forget plot update (does not block or affect timing)
        self.step_request.emit("CALCULATE", x, y, z, phi, t)

        self._wait_for_ack()
        self._wait_step_confirm()

    def _wait_if_paused(self):
        while self._paused and self._running and not self._estop:
            self.msleep(100)

    def _wait_step_confirm(self):
        if self.run_mode != "step_by_step":
            return
        self._step_ok = False
        while not self._step_ok and self._running and not self._estop:
            self.msleep(100)
