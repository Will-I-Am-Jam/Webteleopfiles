      
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

# Default USBIPD bus IDs. These are editable in the GUI before launch.
DEFAULT_HAND_BUS_ID = "1-11"
DEFAULT_CAMERA_BUS_ID = "1-3"

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

        # USBIPD bus IDs used by the ROS / WSL launcher.
        self.hand_busid_var = tk.StringVar(value=DEFAULT_HAND_BUS_ID)
        self.camera_busid_var = tk.StringVar(value=DEFAULT_CAMERA_BUS_ID)

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

        ttk.Label(launch_frame, text="Hand BUSID:").grid(row=0, column=0, sticky="w", padx=(6, 4), pady=4)
        self.hand_busid_entry = ttk.Entry(launch_frame, textvariable=self.hand_busid_var, width=10)
        self.hand_busid_entry.grid(row=0, column=1, sticky="w", padx=(0, 10), pady=4)

        ttk.Label(launch_frame, text="Camera BUSID:").grid(row=0, column=2, sticky="w", padx=(6, 4), pady=4)
        self.camera_busid_entry = ttk.Entry(launch_frame, textvariable=self.camera_busid_var, width=10)
        self.camera_busid_entry.grid(row=0, column=3, sticky="w", padx=(0, 10), pady=4)

        self.launch_bat_btn = ttk.Button(
            launch_frame,
            text="Launch Webcam Teleop",
            command=self._run_launch_bat,
            width=24
        )
        self.launch_bat_btn.grid(row=1, column=0, columnspan=2, padx=6, pady=4)

        self.stop_teleop_btn = ttk.Button(
            launch_frame,
            text="Stop Webcam Teleop",
            command=self._stop_webcam_teleop,
            width=24
        )
        self.stop_teleop_btn.grid(row=1, column=2, columnspan=2, padx=6, pady=4)

        self.open_wsl_btn = ttk.Button(
            launch_frame,
            text="Open WSL Terminal",
            command=self._open_wsl_terminal,
            width=24
        )
        self.open_wsl_btn.grid(row=2, column=0, columnspan=2, padx=6, pady=4)

        self.restart_wsl_btn = ttk.Button(
            launch_frame,
            text="Restart WSL + Open",
            command=self._restart_wsl_terminal,
            width=24
        )
        self.restart_wsl_btn.grid(row=2, column=2, columnspan=2, padx=6, pady=4)

        self.bind_hand_btn = ttk.Button(
            launch_frame,
            text="Bind Hand BUSID",
            command=self._bind_hand_busid,
            width=24
        )
        self.bind_hand_btn.grid(row=3, column=0, columnspan=2, padx=6, pady=4)

        self.bind_camera_btn = ttk.Button(
            launch_frame,
            text="Bind Camera BUSID",
            command=self._bind_camera_busid,
            width=24
        )
        self.bind_camera_btn.grid(row=3, column=2, columnspan=2, padx=6, pady=4)

        self.list_usbipd_btn = ttk.Button(
            launch_frame,
            text="List USBIPD BUSIDs",
            command=self._list_usbipd_busids,
            width=24
        )
        self.list_usbipd_btn.grid(row=4, column=0, columnspan=2, padx=6, pady=4)

        ttk.Label(
            launch_frame,
            text="Use List to find BUSIDs\nif Hand/Camera not shared already then Bind."
        ).grid(row=4, column=2, columnspan=2, sticky="w", padx=6, pady=4)

        ttk.Label(
            launch_frame,
            text=f"Launcher: {LAUNCH_BAT_PATH}  |  Distro: {WSL_DISTRO}"
        ).grid(row=5, column=0, columnspan=4, sticky="w", padx=6, pady=(4, 0))

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
        """Run the Windows .bat launcher with the hand/camera USBIPD bus IDs from the GUI."""
        if not os.path.exists(LAUNCH_BAT_PATH):
            messagebox.showerror(
                "Launcher Not Found",
                f"Could not find the launch file:\n{LAUNCH_BAT_PATH}\n\n"
                "Update LAUNCH_BAT_PATH near the top of this Python file."
            )
            self._set_status("Launch failed: .bat file not found.")
            return

        hand_busid = self.hand_busid_var.get().strip()
        camera_busid = self.camera_busid_var.get().strip()

        if not hand_busid:
            messagebox.showwarning("Missing Hand BUSID", "Enter the USBIPD bus ID for the hand, for example 1-11.")
            return

        if not camera_busid:
            messagebox.showwarning("Missing Camera BUSID", "Enter the USBIPD bus ID for the webcam, for example 1-3.")
            return

        try:
            # Use Windows 'start' so the ROS launch opens in its own Command Prompt window.
            # Pass hand_busid and camera_busid as arguments to launch_aero_hand.bat.
            if self.hand:
                self._disconnect()  # Disconnect the hand before launching ROS, if connected.

            subprocess.Popen(
                ["cmd.exe", "/c", "start", "", LAUNCH_BAT_PATH, hand_busid, camera_busid],
                shell=False
            )
            self._set_status(f"Started webcam teleop with hand={hand_busid}, camera={camera_busid}.")
        except Exception as e:
            messagebox.showerror("Launch Error", str(e))
            self._set_status(f"Launch failed: {e}")

    def _stop_webcam_teleop(self):
        """Stop the ROS 2 webcam teleop launch and its child nodes inside WSL."""
        confirm = messagebox.askyesno(
            "Stop Webcam Teleop?",
            "This will stop the webcam teleop ROS launch inside WSL.\n\n"
            "The WSL terminal window may stay open, but the ROS nodes should stop.\n\n"
            "Continue?",
        )
        if not confirm:
            return

        self.stop_teleop_btn.config(state="disabled")
        self._set_status("Stopping webcam teleop in WSL...")

        stop_command = """
pkill -2 -f "webcam_teleop.launch.py" 2>/dev/null || true
sleep 1
pkill -15 -f "webcam_mocap|dex_retargeting_node|aero_hand_node" 2>/dev/null || true
sleep 0.5
pkill -9 -f "webcam_mocap|dex_retargeting_node|aero_hand_node" 2>/dev/null || true
""".strip()

        def do_stop():
            try:
                result = subprocess.run(
                    ["wsl.exe", "-d", WSL_DISTRO, "--", "bash", "-lc", stop_command],
                    capture_output=True,
                    text=True,
                    timeout=10,
                )
                if result.returncode not in (0, 1):
                    msg = (result.stderr or result.stdout or "Unknown stop error").strip()
                    self.root.after(0, self._on_stop_teleop_error, msg)
                else:
                    self.root.after(0, self._on_stop_teleop_done)
            except Exception as e:
                self.root.after(0, self._on_stop_teleop_error, str(e))

        threading.Thread(target=do_stop, daemon=True).start()

    def _on_stop_teleop_done(self):
        self.stop_teleop_btn.config(state="normal")
        self._set_status("Stopped webcam teleop ROS processes in WSL.")

    def _on_stop_teleop_error(self, err):
        self.stop_teleop_btn.config(state="normal")
        messagebox.showerror("Stop Teleop Error", err)
        self._set_status(f"Failed to stop webcam teleop: {err}")

    def _bind_hand_busid(self):
        """Bind the hand BUSID from the GUI field using usbipd."""
        busid = self.hand_busid_var.get().strip()
        if not busid:
            messagebox.showwarning("Missing Hand BUSID", "Enter the hand BUSID first, for example 1-11.")
            return
        self._bind_usbipd_busid(busid, "hand")

    def _bind_camera_busid(self):
        """Bind the camera BUSID from the GUI field using usbipd."""
        busid = self.camera_busid_var.get().strip()
        if not busid:
            messagebox.showwarning("Missing Camera BUSID", "Enter the camera BUSID first, for example 1-3.")
            return
        self._bind_usbipd_busid(busid, "camera")

    def _bind_usbipd_busid(self, busid, label):
        """
        Run 'usbipd bind --busid <busid>'.

        Binding usually requires Administrator privileges. This method first tries
        a normal bind. If Windows denies access, it launches an elevated PowerShell
        command, which should show a UAC prompt.
        """
        confirm = messagebox.askyesno(
            f"Bind {label.title()} BUSID?",
            f"This will run usbipd bind for the {label} device:\n\n"
            f"BUSID: {busid}\n\n"
            "Windows may ask for Administrator permission. Continue?",
        )
        if not confirm:
            return

        self.bind_hand_btn.config(state="disabled")
        self.bind_camera_btn.config(state="disabled")
        self._set_status(f"Binding {label} BUSID {busid}...")

        def do_bind():
            try:
                result = self._run_usbipd_command(["bind", "--busid", busid], timeout=20)
                output = ((result.stdout or "") + "\n" + (result.stderr or "")).strip()

                if result.returncode == 0:
                    self.root.after(0, self._on_bind_usbipd_done, busid, label, output)
                    return

                needs_admin = (
                    "Access denied" in output
                    or "administrator" in output.lower()
                    or "requires administrator" in output.lower()
                    or "privileges" in output.lower()
                )

                if needs_admin:
                    elevated_result = self._run_usbipd_bind_elevated(busid)
                    elevated_output = (
                        (elevated_result.stdout or "") + "\n" + (elevated_result.stderr or "")
                    ).strip()

                    if elevated_result.returncode == 0:
                        self.root.after(0, self._on_bind_usbipd_done, busid, label, elevated_output)
                    else:
                        msg = elevated_output or f"Elevated bind exited with code {elevated_result.returncode}"
                        self.root.after(0, self._on_bind_usbipd_error, busid, label, msg)
                    return

                msg = output or f"usbipd bind exited with code {result.returncode}"
                self.root.after(0, self._on_bind_usbipd_error, busid, label, msg)

            except Exception as e:
                self.root.after(0, self._on_bind_usbipd_error, busid, label, str(e))

        threading.Thread(target=do_bind, daemon=True).start()

    def _run_usbipd_command(self, args, timeout=15):
        """Run usbipd with args, trying PATH first and then the default install location."""
        commands_to_try = [
            ["usbipd"] + args,
            [r"C:\Program Files\usbipd-win\usbipd.exe"] + args,
        ]

        last_error = None
        for cmd in commands_to_try:
            try:
                return subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=timeout,
                )
            except FileNotFoundError as e:
                last_error = e

        raise FileNotFoundError(
            "Could not find usbipd. Install usbipd-win or add it to PATH. "
            "Tried 'usbipd' and 'C:\\Program Files\\usbipd-win\\usbipd.exe'."
        ) from last_error

    def _run_usbipd_bind_elevated(self, busid):
        """
        Run usbipd bind through elevated PowerShell.

        This should show a Windows UAC prompt. The usbipd output may appear in
        a separate elevated process, so this function mainly tells us whether
        PowerShell successfully started the elevated bind command.
        """
        escaped_busid = busid.replace("'", "''")

        ps_command = (
            "$exe = (Get-Command usbipd.exe -ErrorAction SilentlyContinue).Source; "
            "if (-not $exe) { $exe = 'C:\\Program Files\\usbipd-win\\usbipd.exe' }; "
            "if (-not (Test-Path $exe)) { throw 'usbipd.exe was not found.' }; "
            f"Start-Process -FilePath $exe -ArgumentList @('bind','--busid','{escaped_busid}') -Verb RunAs -Wait"
        )

        return subprocess.run(
            [
                "powershell.exe",
                "-NoProfile",
                "-ExecutionPolicy",
                "Bypass",
                "-Command",
                ps_command,
            ],
            capture_output=True,
            text=True,
            timeout=90,
        )

    def _on_bind_usbipd_done(self, busid, label, output):
        self.bind_hand_btn.config(state="normal")
        self.bind_camera_btn.config(state="normal")
        self._set_status(f"Bound {label} BUSID {busid}.")
        messagebox.showinfo(
            "USBIPD Bind Complete",
            f"Finished binding the {label} device.\n\nBUSID: {busid}\n\n"
            "Now you can attach it or launch webcam teleop.\n\n"
            "If this was run through a UAC prompt, use 'List USBIPD BUSIDs' to verify it shows as Shared."
        )

    def _on_bind_usbipd_error(self, busid, label, err):
        self.bind_hand_btn.config(state="normal")
        self.bind_camera_btn.config(state="normal")
        messagebox.showerror(
            "USBIPD Bind Error",
            f"Could not bind the {label} device.\n\nBUSID: {busid}\n\n{err}\n\n"
            "Try running this GUI as Administrator, or run this manually in Administrator PowerShell:\n\n"
            f"usbipd bind --busid {busid}"
        )
        self._set_status(f"Failed to bind {label} BUSID {busid}: {err}")

    def _list_usbipd_busids(self):
        """Run 'usbipd list' on Windows and show the available USB BUSIDs."""
        self.list_usbipd_btn.config(state="disabled")
        self._set_status("Listing USBIPD devices...")

        def do_list():
            try:
                result = self._run_usbipd_list()
                output = result.stdout.strip()
                error_output = result.stderr.strip()

                if result.returncode != 0:
                    msg = error_output or output or f"usbipd list exited with code {result.returncode}"
                    self.root.after(0, self._on_usbipd_list_error, msg)
                    return

                if not output:
                    output = "usbipd list returned no devices."

                self.root.after(0, self._show_usbipd_list_window, output)
            except Exception as e:
                self.root.after(0, self._on_usbipd_list_error, str(e))

        threading.Thread(target=do_list, daemon=True).start()

    def _run_usbipd_list(self):
        """Run 'usbipd list' using the shared usbipd command helper."""
        return self._run_usbipd_command(["list"], timeout=10)

    def _show_usbipd_list_window(self, output):
        self.list_usbipd_btn.config(state="normal")
        self._set_status("USBIPD devices listed.")

        win = tk.Toplevel(self.root)
        win.title("USBIPD BUSIDs")
        win.geometry("900x420")

        info = ttk.Label(
            win,
            text="Copy the BUSID values for your hand and camera into the GUI fields.",
            padding=(8, 8),
        )
        info.pack(fill="x")

        frame = ttk.Frame(win, padding=(8, 0, 8, 8))
        frame.pack(fill="both", expand=True)

        scrollbar = ttk.Scrollbar(frame)
        scrollbar.pack(side="right", fill="y")

        text_box = tk.Text(
            frame,
            wrap="none",
            font=("Consolas", 10),
            yscrollcommand=scrollbar.set,
        )
        text_box.pack(side="left", fill="both", expand=True)
        scrollbar.config(command=text_box.yview)

        text_box.insert("1.0", output)
        text_box.config(state="disabled")

        button_frame = ttk.Frame(win, padding=(8, 0, 8, 8))
        button_frame.pack(fill="x")

        def copy_output():
            self.root.clipboard_clear()
            self.root.clipboard_append(output)
            self._set_status("Copied usbipd list output to clipboard.")

        ttk.Button(button_frame, text="Copy Output", command=copy_output).pack(side="left")
        ttk.Button(button_frame, text="Refresh", command=lambda: [win.destroy(), self._list_usbipd_busids()]).pack(side="left", padx=8)
        ttk.Button(button_frame, text="Close", command=win.destroy).pack(side="right")

    def _on_usbipd_list_error(self, err):
        self.list_usbipd_btn.config(state="normal")
        messagebox.showerror(
            "USBIPD List Error",
            f"Could not run 'usbipd list'.\n\n{err}\n\n"
            "Make sure usbipd-win is installed and try running the GUI as a normal Windows app."
        )
        self._set_status(f"USBIPD list failed: {err}")

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

    