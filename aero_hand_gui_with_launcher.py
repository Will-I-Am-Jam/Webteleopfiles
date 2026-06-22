      
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
import os
import subprocess

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

# Windows batch file used to launch the ROS 2 webcam teleop setup through WSL.
# Change this path if your .bat file is somewhere else.
LAUNCH_BAT_PATH = r"C:\aero_launcher\launch_aero_hand.bat"

# WSL distribution to open/restart when using the WSL terminal buttons.
WSL_DISTRO = "Ubuntu-22.04"

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

        # ── ROS / WSL Launcher ──
        launch_frame = ttk.LabelFrame(self.root, text="ROS / WSL Launch", padding=8)
        launch_frame.grid(row=4, column=0, sticky="ew", **PAD)

        self.launch_bat_btn = ttk.Button(
            launch_frame,
            text="Launch Webcam Teleop",
            command=self._run_launch_bat,
            width=24
        )
        self.launch_bat_btn.grid(row=0, column=0, padx=6, pady=4)

        ttk.Label(
            launch_frame,
            text=f"Runs: {LAUNCH_BAT_PATH}"
        ).grid(row=0, column=1, columnspan=2, sticky="w", padx=(8, 0))

        self.open_wsl_btn = ttk.Button(
            launch_frame,
            text="Open WSL Terminal",
            command=self._open_wsl_terminal,
            width=24
        )
        self.open_wsl_btn.grid(row=1, column=0, padx=6, pady=4)

        self.restart_wsl_btn = ttk.Button(
            launch_frame,
            text="Restart WSL + Open",
            command=self._restart_wsl_terminal,
            width=24
        )
        self.restart_wsl_btn.grid(row=1, column=1, sticky="w", padx=6, pady=4)

        ttk.Label(
            launch_frame,
            text=f"Distro: {WSL_DISTRO}"
        ).grid(row=1, column=2, sticky="w", padx=(8, 0))

        # ── Status bar ──
        self.status_var = tk.StringVar(value="Not connected.")
        ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w").grid(
            row=5, column=0, sticky="ew", padx=10, pady=(0, 8))

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
                for waypoints in self.hand.create_trajectory(DEMO_SEQUENCE):
                    self.hand.set_joint_positions(waypoints)
                    time.sleep(0.01)
                self.root.after(0, self._on_sequence_done)
            except Exception as e:
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

    # ── ROS / WSL Launch ──────────────────────────────────────────────────────

    def _run_launch_bat(self):
        """Run the Windows .bat launcher that attaches USB devices and starts ROS 2."""
        if not os.path.exists(LAUNCH_BAT_PATH):
            messagebox.showerror(
                "Launcher Not Found",
                f"Could not find the launch file:\n{LAUNCH_BAT_PATH}\n\n"
                "Update LAUNCH_BAT_PATH near the top of this Python file."
            )
            self._set_status("Launch failed: .bat file not found.")
            return

        try:
            # Use Windows 'start' so the ROS launch opens in its own Command Prompt window.
            if self.hand:
                self._disconnect()  # Disconnect the hand before launching ROS, if connected.
            subprocess.Popen(
                ["cmd.exe", "/c", "start", "", LAUNCH_BAT_PATH],
                shell=False
            )
            self._set_status("Started ROS launch batch file.")
        except Exception as e:
            messagebox.showerror("Launch Error", str(e))
            self._set_status(f"Launch failed: {e}")

    def _open_wsl_terminal(self):
        """Open a visible WSL 2 terminal for the configured Ubuntu distribution."""
        try:
            # Prefer Windows Terminal because it handles interactive WSL sessions well.
            subprocess.Popen(
                [
                    "wt.exe",
                    "new-tab",
                    "--title",
                    f"{WSL_DISTRO} Terminal",
                    "wsl.exe",
                    "-d",
                    WSL_DISTRO,
                ],
                shell=False,
            )
            self._set_status(f"Opened WSL terminal: {WSL_DISTRO}")
        except FileNotFoundError:
            # Fallback for systems without Windows Terminal on PATH.
            try:
                subprocess.Popen(
                    ["cmd.exe", "/c", "start", "", "wsl.exe", "-d", WSL_DISTRO],
                    shell=False,
                )
                self._set_status(f"Opened WSL terminal: {WSL_DISTRO}")
            except Exception as e:
                messagebox.showerror("WSL Launch Error", str(e))
                self._set_status(f"Could not open WSL terminal: {e}")
        except Exception as e:
            messagebox.showerror("WSL Launch Error", str(e))
            self._set_status(f"Could not open WSL terminal: {e}")

    def _restart_wsl_terminal(self):
        """Shutdown WSL 2, then reopen the configured Ubuntu terminal."""
        confirm = messagebox.askyesno(
            "Restart WSL?",
            "This will shut down all running WSL sessions, including any active ROS launch.\n\n"
            "Continue?",
        )
        if not confirm:
            return

        self._set_status("Restarting WSL 2...")

        def do_restart():
            try:
                subprocess.run(
                    ["wsl.exe", "--shutdown"],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                time.sleep(1.0)
                self.root.after(0, self._open_wsl_terminal)
            except Exception as e:
                self.root.after(0, self._on_wsl_restart_error, str(e))

        threading.Thread(target=do_restart, daemon=True).start()

    def _on_wsl_restart_error(self, err):
        messagebox.showerror("WSL Restart Error", err)
        self._set_status(f"WSL restart failed: {err}")

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

    