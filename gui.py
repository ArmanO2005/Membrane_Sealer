import tkinter as tk
from tkinter import scrolledtext
import threading
import queue
import sys
import yaml
from pathlib import Path
from c_motor_control.abort import AbortedError

def _resolve_config_path():
    rel = Path('a_config') / 'motor_config.yaml'
    if getattr(sys, 'frozen', False):
        # Prefer an editable copy next to the exe; fall back to the bundled one.
        beside_exe = Path(sys.executable).parent / rel
        if beside_exe.exists():
            return beside_exe
        return Path(sys._MEIPASS) / rel
    return Path(__file__).parent / rel


_CONFIG_PATH = _resolve_config_path()


BG      = '#1e1e2e'
BG2     = '#181825'
SURFACE = '#313244'
FG      = '#cdd6f4'
BLUE    = '#89b4fa'
GREEN   = '#a6e3a1'
RED     = '#f38ba8'
YELLOW  = '#f9e2af'
TEAL    = '#89dceb'
GRAY    = '#6c7086'


def _load_config():
    with open(_CONFIG_PATH, 'r') as f:
        return yaml.safe_load(f)


class _LogRedirector:
    def __init__(self, q):
        self._q = q
        self._orig = sys.__stdout__

    def write(self, text):
        self._q.put(text)
        if self._orig:
            self._orig.write(text)

    def flush(self):
        pass


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Membrane Sealer Control System")
        self.root.configure(bg=BG)
        self.root.resizable(True, True)
        self.root.minsize(900, 500)

        self._log_q: queue.Queue = queue.Queue()
        self.sealer = None
        self._busy = False
        self._stopping = False
        self._worker = None
        self._all_btns: list = []
        self._sequence_btns: list = []
        self._subsystem_btns: dict = {}  # subsystem name -> list of widgets

        try:
            wheel_cfg = _load_config().get('wheel', {})
        except Exception:
            wheel_cfg = {}
        self._default_seal_point = str(wheel_cfg.get('seal_point', 0))
        self._default_lift_point = str(wheel_cfg.get('lift_point', 0))
        self._default_dwell      = str(wheel_cfg.get('dwell_time', 1))

        sys.stdout = _LogRedirector(self._log_q)

        self._build_ui()
        self._poll_log()

    # ------------------------------------------------------------------ layout

    def _build_ui(self):
        left = tk.Frame(self.root, bg=BG, padx=12, pady=12)
        left.pack(side=tk.LEFT, fill=tk.Y)

        divider = tk.Frame(self.root, bg=SURFACE, width=1)
        divider.pack(side=tk.LEFT, fill=tk.Y)

        right = tk.Frame(self.root, bg=BG, padx=12, pady=12)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        tk.Label(left, text="Membrane Sealer Control",
                 bg=BG, fg=FG, font=('Segoe UI', 13, 'bold')).pack(anchor='w', pady=(0, 8))

        # ---- connection ----
        self._section(left, "Connection")
        row = self._row(left)
        self.connect_btn    = self._btn(row, "Connect",    self._do_connect,    BLUE)
        self.disconnect_btn = self._btn(row, "Disconnect", self._do_disconnect, GRAY)
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 4))
        self.disconnect_btn.pack(side=tk.LEFT)
        self.status_lbl = tk.Label(left, text="● Disconnected",
                                   bg=BG, fg=RED, font=('Segoe UI', 9))
        self.status_lbl.pack(anchor='w', pady=(3, 2))

        # ---- full sequence ----
        self._section(left, "Full Sequence")
        row = self._row(left)
        self._btn(row, "Initialize", self._do_initialize, GREEN, sequence=True).pack(side=tk.LEFT, padx=(0, 4))
        self._btn(row, "Seal",       self._do_seal,       BLUE,  sequence=True).pack(side=tk.LEFT, padx=(0, 4))
        self._btn(row, "Shutdown",   self._do_off,        RED,   sequence=True).pack(side=tk.LEFT)
        # Not registered via _btn so _lock() leaves it clickable while an operation runs.
        self.stop_btn = tk.Button(row, text="STOP", command=self._do_stop,
                                  bg=RED, fg=BG2, activebackground=FG,
                                  font=('Segoe UI', 9, 'bold'),
                                  relief=tk.FLAT, padx=12, pady=5, cursor='hand2')
        self.stop_btn.pack(side=tk.LEFT, padx=(12, 0))

        row = self._row(left)
        self.seal_point_var = self._entry(row, "Seal point:", self._default_seal_point, 8)
        self.seal_dwell_var = self._entry(row, "Dwell (s):",  self._default_dwell,      5)
        row = self._row(left)
        self.seal_rotations_var = self._entry(row, "Rotations:",        "1.0",   5)
        self.seal_velocity_var  = self._entry(row, "Spindle velocity:", "10000", 7)

        # ---- heater ----
        self._section(left, "Heater")
        row = self._row(left)
        self._btn(row, "Enable",         self._do_heater_on,    YELLOW, subsystem='wheel').pack(side=tk.LEFT, padx=(0, 4))
        self._btn(row, "Disable",        self._do_heater_off,   GRAY,   subsystem='wheel').pack(side=tk.LEFT, padx=(0, 4))
        self._btn(row, "Check Setpoint", self._do_heater_check, GRAY,   subsystem='wheel').pack(side=tk.LEFT)

        # ---- wheel ----
        self._section(left, "Wheel")
        row = self._row(left)
        self._btn(row, "Home",  self._do_wheel_home,  subsystem='wheel').pack(side=tk.LEFT, padx=(0, 4))
        self._btn(row, "Lower", self._do_wheel_lower, subsystem='wheel').pack(side=tk.LEFT, padx=(0, 4))
        self._btn(row, "Lift",  self._do_wheel_lift,  subsystem='wheel').pack(side=tk.LEFT)
        row = self._row(left)
        self.wheel_lower_var = self._entry(row, "Lower to:", self._default_seal_point, 8, subsystem='wheel')
        self.wheel_lift_var  = self._entry(row, "Lift to:",  self._default_lift_point, 8, subsystem='wheel')

        # ---- spindle ----
        self._section(left, "Spindle")
        row = self._row(left)
        self._btn(row, "Home",          self._do_spindle_home,   subsystem='spindle').pack(side=tk.LEFT, padx=(0, 4))
        self._btn(row, "Full Rotation", self._do_spindle_rotate, subsystem='spindle').pack(side=tk.LEFT, padx=(0, 4))
        self._btn(row, "Go to Notch",   self._do_spindle_notch,  subsystem='spindle').pack(side=tk.LEFT)

        # ---- log panel ----
        hdr = tk.Frame(right, bg=BG)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text="Output Log", bg=BG, fg=FG,
                 font=('Segoe UI', 11, 'bold')).pack(side=tk.LEFT)
        tk.Button(hdr, text="Clear", command=self._clear_log,
                  bg=SURFACE, fg=FG, font=('Segoe UI', 8),
                  relief=tk.FLAT, padx=6, pady=2).pack(side=tk.RIGHT)
        self.log_box = scrolledtext.ScrolledText(
            right, bg=BG2, fg=GREEN, font=('Consolas', 9),
            state='disabled', wrap=tk.WORD)
        self.log_box.pack(fill=tk.BOTH, expand=True, pady=(4, 0))

        self._update_conn_state()

    def _section(self, parent, title):
        tk.Frame(parent, bg=SURFACE, height=1).pack(fill=tk.X, pady=(8, 1))
        tk.Label(parent, text=title, bg=BG, fg=TEAL,
                 font=('Segoe UI', 9, 'bold')).pack(anchor='w', pady=(0, 3))

    def _row(self, parent):
        f = tk.Frame(parent, bg=BG)
        f.pack(fill=tk.X, pady=(0, 2))
        return f

    def _entry(self, parent, label, default, width, subsystem=None):
        tk.Label(parent, text=label, bg=BG, fg=FG,
                 font=('Segoe UI', 9)).pack(side=tk.LEFT, padx=(0, 4))
        var = tk.StringVar(value=default)
        e = tk.Entry(parent, textvariable=var, width=width,
                     bg=SURFACE, fg=FG, insertbackground=FG,
                     relief=tk.FLAT, font=('Segoe UI', 9))
        e.pack(side=tk.LEFT, padx=(0, 10))
        if subsystem:
            self._subsystem_btns.setdefault(subsystem, []).append(e)
        return var

    def _btn(self, parent, text, cmd, color=GRAY, subsystem=None, sequence=False):
        b = tk.Button(parent, text=text, command=cmd,
                      bg=color, fg=BG2, activebackground=FG,
                      font=('Segoe UI', 9, 'bold'),
                      relief=tk.FLAT, padx=8, pady=5, cursor='hand2')
        self._all_btns.append(b)
        if subsystem:
            self._subsystem_btns.setdefault(subsystem, []).append(b)
        if sequence:
            self._sequence_btns.append(b)
        return b

    # ------------------------------------------------------------------ state

    def _update_conn_state(self):
        live = self.sealer is not None
        self.connect_btn.configure(state=tk.DISABLED if live else tk.NORMAL)
        self.disconnect_btn.configure(state=tk.NORMAL if live else tk.DISABLED)
        self.stop_btn.configure(state=tk.NORMAL if live and not self._stopping else tk.DISABLED)

        if not live:
            for btns in self._subsystem_btns.values():
                for w in btns:
                    w.configure(state=tk.DISABLED)
            for b in self._sequence_btns:
                b.configure(state=tk.DISABLED)
            self.status_lbl.configure(text="● Disconnected", fg=RED)
            return

        motor_map = {
            'spindle': self.sealer.spindle_motor,
            'wheel':   self.sealer.wheel_motor,
        }
        for subsystem, widgets in self._subsystem_btns.items():
            state = tk.NORMAL if motor_map.get(subsystem) is not None else tk.DISABLED
            for w in widgets:
                w.configure(state=state)

        for b in self._sequence_btns:
            b.configure(state=tk.NORMAL)

        n = sum(1 for m in motor_map.values() if m is not None)
        total = len(motor_map)
        color = GREEN if n == total else YELLOW
        self.status_lbl.configure(text=f"● Connected ({n}/{total})", fg=color)

    def _lock(self):
        for b in self._all_btns:
            b.configure(state=tk.DISABLED)

    def _unlock(self):
        for b in self._all_btns:
            b.configure(state=tk.NORMAL)
        self._update_conn_state()

    # ---------------------------------------------------------------- threading

    def _run(self, fn, *args):
        if self._busy:
            self._append("⚠  Another operation is in progress.\n")
            return
        self._busy = True
        self.root.after(0, self._lock)

        def worker():
            try:
                fn(*args)
            except AbortedError:
                sys.stdout.write("Operation aborted.\n")
            except Exception as exc:
                sys.stdout.write(f"ERROR: {exc}\n")
            finally:
                if not self._stopping:
                    self._busy = False
                    self.root.after(0, self._unlock)

        self._worker = threading.Thread(target=worker, daemon=True)
        self._worker.start()

    # ----------------------------------------------------------------- log

    def _poll_log(self):
        while not self._log_q.empty():
            self._append(self._log_q.get_nowait())
        self.root.after(100, self._poll_log)

    def _append(self, text: str):
        self.log_box.configure(state=tk.NORMAL)
        self.log_box.insert(tk.END, text)
        self.log_box.see(tk.END)
        self.log_box.configure(state=tk.DISABLED)

    def _clear_log(self):
        self.log_box.configure(state=tk.NORMAL)
        self.log_box.delete('1.0', tk.END)
        self.log_box.configure(state=tk.DISABLED)

    # ---------------------------------------------------------------- handlers

    def _do_connect(self):
        def connect():
            from d_robot_control.membrane_sealer import MembraneSealer
            config = _load_config()
            sys.stdout.write("Connecting to motors...\n")
            self.sealer = MembraneSealer(config)
            self.root.after(0, self._update_conn_state)
        self._run(connect)

    def _do_disconnect(self):
        if self.sealer is not None:
            self.sealer.close()
        self.sealer = None
        self._update_conn_state()
        self._append("Disconnected.\n")

    def _do_stop(self):
        if self.sealer is None or self._stopping:
            return
        self._stopping = True
        self._busy = True
        self.sealer.request_stop()
        self._lock()
        self.stop_btn.configure(state=tk.DISABLED)
        self._append("■ STOP requested — aborting, lifting wheel, homing...\n")
        running = self._worker

        def stopper():
            try:
                if running is not None:
                    running.join()
                self.sealer.stop_and_home()
            except Exception as exc:
                sys.stdout.write(f"ERROR during stop: {exc}\n")
            finally:
                self._stopping = False
                self._busy = False
                self.root.after(0, self._unlock)

        threading.Thread(target=stopper, daemon=True).start()

    def _do_initialize(self):
        self._run(self.sealer.initialize)

    def _do_seal(self):
        try:
            seal_point = int(self.seal_point_var.get())
            dwell      = float(self.seal_dwell_var.get())
            rotations  = float(self.seal_rotations_var.get())
            velocity   = int(self.seal_velocity_var.get())
        except ValueError:
            self._append("ERROR: Invalid seal parameter value.\n")
            return
        self._run(self.sealer.seal, seal_point, dwell, rotations, velocity)

    def _do_off(self):
        self._run(self.sealer.off)

    def _do_heater_on(self):
        self._run(self.sealer.wheel_motor.enable_heater)

    def _do_heater_off(self):
        self._run(self.sealer.wheel_motor.disable_heater)

    def _do_heater_check(self):
        def check():
            ok = self.sealer.wheel_motor.heater_ready()
            print(f"[Wheel] Heater setpoint OK: {'YES' if ok else 'NO'}")
        self._run(check)

    def _do_wheel_home(self):
        self._run(self.sealer.wheel_motor.home)

    def _do_wheel_lower(self):
        try:
            point = int(self.wheel_lower_var.get())
        except ValueError:
            self._append("ERROR: Invalid lower position.\n")
            return
        self._run(self.sealer.wheel_motor.lower, point)

    def _do_wheel_lift(self):
        try:
            point = int(self.wheel_lift_var.get())
        except ValueError:
            self._append("ERROR: Invalid lift position.\n")
            return
        self._run(self.sealer.wheel_motor.lift, point)

    def _do_spindle_home(self):
        self._run(self.sealer.spindle_motor.home)

    def _do_spindle_rotate(self):
        self._run(self.sealer.spindle_motor.full_rotation)

    def _do_spindle_notch(self):
        self._run(self.sealer.spindle_motor.move_to_notch)


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
