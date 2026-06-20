#!/usr/bin/env python3
"""
Aero Hand Open — Simple GUI Controller
Double-click or run: python aero_hand_gui.py
Requires: pip install aero-open-sdk
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time

try:
    import serial.tools.list_ports
except ImportError:
    raise SystemExit("ERROR: pyserial not installed. Run: pip install pyserial")

try:
    from aero_open_sdk.aero_hand import AeroHand
except ImportError:
    raise SystemExit("ERROR: aero-open-sdk not installed. Run: pip install aero-open-sdk")


# ── Poses (7-joint: thumb_abd, thumb_flex, thumb_ip, index, middle, ring, pinky) ──
OPEN_POSE = [100.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0]
GRIP_POSE = [100.0, 80.0, 60.0, 90.0, 90.0, 90.0, 90.0]

DEFAULT_TORQUE = 400  # 0-1000

# ── Demo sequence — verbatim from sdk/examples/run_sequence.py ──
DEMO_SEQUENCE = [
    ## Open Palm
    ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0),

    ## Pinch fingers one by one
    ([100.0, 35.0, 23.0, 0.0, 0.0, 0.0, 50.0], 0.5),
    ([100.0, 35.0, 23.0, 0.0, 0.0, 0.0, 50.0], 0.25),
    ([100.0, 42.0, 23.0, 0.0, 0.0, 52.0, 0.0], 0.5),
    ([100.0, 42.0, 23.0, 0.0, 0.0, 52.0, 0.0], 0.25),
    ([83.0, 42.0, 23.0, 0.0, 50.0, 0.0, 0.0], 0.5),
    ([83.0, 42.0, 23.0, 0.0, 50.0, 0.0, 0.0], 0.25),
    ([75.0, 25.0, 30.0, 50.0, 0.0, 0.0, 0.0], 0.5),
    ([75.0, 25.0, 30.0, 50.0, 0.0, 0.0, 0.0], 0.25),

    ## Open Palm
    ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),
    ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),

    ## Peace Sign
    ([90.0, 0.0, 0.0, 0.0, 0.0, 90.0, 90.0], 0.5),
    ([90.0, 45.0, 60.0, 0.0, 0.0, 90.0, 90.0], 0.5),
    ([90.0, 45.0, 60.0, 0.0, 0.0, 90.0, 90.0], 1.0),

    ## Open Palm
    ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),
    ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),

    ## Rockstar Sign
    ([0.0, 0.0, 0.0, 0.0, 90.0, 90.0, 0.0], 0.5),
    ([0.0, 0.0, 0.0, 0.0, 90.0, 90.0, 0.0], 1.0),

    ## Open Palm
    ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),
]


def find_aero_ports():
    all_ports = list(serial.tools.list_ports.comports())
    esp_ports = [p for p in all_ports if "Espressif" in (p.manufacturer or "") or "JTAG" in (p.description or "")]
    other_ports = [p for p in all_ports if p not in esp_ports]
    ordered = esp_ports + other_ports
    return [(p.device, f"{p.device}  {p.description}") for p in ordered]


class AeroHandGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Aero Hand Controller")
        self.root.resizable(False, False)

        self.hand = None
        self.grasped = False
        self._busy = False
        self._torque_after_id = None  # debounce handle

        self._build_ui()
        self._refresh_ports()

    # ── UI Construction ────────────────────────────────────────────────────────

    def _build_ui(self):
        PAD = {"padx": 10, "pady": 6}

        # ── Connection ──
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=8)
        conn_frame.grid(row=0, column=0, sticky="ew", **PAD)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=30, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(4, 4))

        self.refresh_btn = ttk.Button(conn_frame, text="Refresh", width=7, command=self._refresh_ports)
        self.refresh_btn.grid(row=0, column=2)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connect, width=12)
        self.connect_btn.grid(row=0, column=3, padx=(8, 0))

        self.conn_indicator = tk.Label(conn_frame, text="●", font=("Arial", 14), fg="gray")
        self.conn_indicator.grid(row=0, column=4, padx=(6, 0))

        # ── Hand Control ──
        ctrl_frame = ttk.LabelFrame(self.root, text="Hand Control", padding=8)
        ctrl_frame.grid(row=1, column=0, sticky="ew", **PAD)

        self.grasp_btn = ttk.Button(ctrl_frame, text="Power Grasp", command=self._power_grasp,
                                    width=18, state="disabled")
        self.grasp_btn.grid(row=0, column=0, padx=6, pady=4)

        self.home_btn = ttk.Button(ctrl_frame, text="Run Homing", command=self._run_homing,
                                   width=18, state="disabled")
        self.home_btn.grid(row=0, column=1, padx=6, pady=4)

        # ── Demo Sequence ──
        seq_frame = ttk.LabelFrame(self.root, text="Demo Sequence", padding=8)
        seq_frame.grid(row=2, column=0, sticky="ew", **PAD)

        self.seq_btn = ttk.Button(seq_frame, text="Run Gesture Sequence", command=self._run_sequence,
                                  width=22, state="disabled")
        self.seq_btn.grid(row=0, column=0, padx=6, pady=4)

        # ── Settings ──
        set_frame = ttk.LabelFrame(self.root, text="Settings", padding=8)
        set_frame.grid(row=3, column=0, sticky="ew", **PAD)

        ttk.Label(set_frame, text="Torque:").grid(row=0, column=0, sticky="w")
        self.torque_var = tk.IntVar(value=DEFAULT_TORQUE)
        torque_spin = ttk.Spinbox(set_frame, from_=0, to=1000, textvariable=self.torque_var, width=6,
                                  command=self._on_torque_changed)
        torque_spin.grid(row=0, column=1, padx=(4, 0))
        self.torque_var.trace_add("write", lambda *_: self._on_torque_changed())

        # ── Status bar ──
        self.status_var = tk.StringVar(value="Not connected.")
        ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w").grid(
            row=4, column=0, sticky="ew", padx=10, pady=(0, 8))

    # ── Port management ────────────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = find_aero_ports()
        labels = [label for _, label in ports]
        self._port_map = {label: device for device, label in ports}
        self.port_combo["values"] = labels
        if labels:
            self.port_combo.current(0)
            self._set_status(f"{len(labels)} port(s) found.")
        else:
            self.port_var.set("")
            self._set_status("No serial ports found. Check USB connection.")

    # ── Connection ─────────────────────────────────────────────────────────────

    def _toggle_connect(self):
        if self.hand is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        label = self.port_var.get()
        if not label:
            messagebox.showwarning("No Port", "Select a serial port first.")
            return

        port = self._port_map.get(label, label.split()[0])
        self._set_status(f"Connecting to {port}...")
        self.connect_btn.config(state="disabled")

        def do_connect():
            try:
                hand = AeroHand(port=port)
                torque = self.torque_var.get()
                for i in range(7):
                    hand.set_torque(i, torque)
                    time.sleep(0.01)
                self.hand = hand
                self.root.after(0, self._on_connected, port)
            except Exception as e:
                self.root.after(0, self._on_connect_failed, str(e))

        threading.Thread(target=do_connect, daemon=True).start()

    def _on_connected(self, port):
        self.connect_btn.config(text="Disconnect", state="normal")
        self.conn_indicator.config(fg="#22c55e")
        self._set_controls(True)
        self.grasped = False
        self.grasp_btn.config(text="Power Grasp")
        self._set_status(f"Connected to {port}  |  Torque: {self.torque_var.get()}")

    def _on_connect_failed(self, err):
        self.connect_btn.config(state="normal")
        self.conn_indicator.config(fg="gray")
        self._set_status(f"Connection failed: {err}")
        messagebox.showerror("Connection Error", err)

    def _disconnect(self):
        if self.hand:
            try:
                self.hand.set_joint_positions([0.0] * 7)
                time.sleep(0.05)
                self.hand.close()
            except Exception:
                pass
            self.hand = None
        self.connect_btn.config(text="Connect")
        self.conn_indicator.config(fg="gray")
        self._set_controls(False)
        self.grasped = False
        self.grasp_btn.config(text="Power Grasp")
        self._set_status("Disconnected.")

    # ── Hand control ───────────────────────────────────────────────────────────

    def _power_grasp(self):
        if not self.hand or self._busy:
            return
        try:
            if self.grasped:
                self.hand.set_joint_positions(OPEN_POSE)
                self.grasped = False
                self.grasp_btn.config(text="Power Grasp")
                self._set_status("Released.")
            else:
                self.hand.set_joint_positions(GRIP_POSE)
                self.grasped = True
                self.grasp_btn.config(text="Release Grasp")
                self._set_status("Power grasp active.")
        except Exception as e:
            self._set_status(f"Error: {e}")

    # ── Homing ─────────────────────────────────────────────────────────────────

    def _run_homing(self):
        if not self.hand or self._busy:
            return
        self._busy = True
        self._set_controls(False)
        self._set_status("Homing in progress — please wait (up to 3 min)...")

        def do_homing():
            try:
                self.hand.send_homing()
                self.root.after(0, self._on_homing_done)
            except Exception as e:
                self.root.after(0, self._on_homing_error, str(e))

        threading.Thread(target=do_homing, daemon=True).start()

    def _on_homing_done(self):
        self._set_status("Homing complete.")
        self._busy = False
        self._set_controls(True)

    def _on_homing_error(self, err):
        self._set_status(f"Homing error: {err}")
        self._busy = False
        self._set_controls(True)

    # ── Sequence ───────────────────────────────────────────────────────────────

    def _run_sequence(self):
        if not self.hand or self._busy:
            return
        self._busy = True
        self._set_controls(False)
        self._set_status("Running gesture sequence...")

        def do_sequence():
            try:
                # for waypoints in self.hand.create_trajectory(DEMO_SEQUENCE):
                #     self.hand.set_joint_positions(waypoints)
                #     time.sleep(0.01)
                self.hand.run_trajectory(DEMO_SEQUENCE)
                self.root.after(0, self._on_sequence_done)
            except Exception as e:
                print(f"Sequence error: {e}")
                self.root.after(0, self._on_sequence_error, str(e))

        threading.Thread(target=do_sequence, daemon=True).start()

    def _on_sequence_done(self):
        self._set_status("Sequence complete.")
        self._busy = False
        self._set_controls(True)

    def _on_sequence_error(self, err):
        self._set_status(f"Sequence error: {err}")
        self._busy = False
        self._set_controls(True)

    # ── Torque live update ─────────────────────────────────────────────────────

    def _on_torque_changed(self):
        if not self.hand or self._busy:
            return
        if self._torque_after_id:
            self.root.after_cancel(self._torque_after_id)
        self._torque_after_id = self.root.after(150, self._apply_torque)

    def _apply_torque(self):
        self._torque_after_id = None
        if not self.hand:
            return
        try:
            torque = self.torque_var.get()
        except tk.TclError:
            return

        def do_torque():
            try:
                for i in range(7):
                    self.hand.set_torque(i, torque)
                    time.sleep(0.01)
                self.root.after(0, self._set_status, f"Torque set to {torque}")
            except Exception as e:
                self.root.after(0, self._set_status, f"Torque error: {e}")

        threading.Thread(target=do_torque, daemon=True).start()

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _set_controls(self, enabled):
        state = "normal" if enabled else "disabled"
        self.grasp_btn.config(state=state)
        self.home_btn.config(state=state)
        self.seq_btn.config(state=state)

    def _set_status(self, msg):
        self.status_var.set(msg)

    def on_close(self):
        if self.hand:
            try:
                self.hand.set_joint_positions([0.0] * 7)
                time.sleep(0.05)
                self.hand.close()
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    app = AeroHandGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
