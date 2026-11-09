from c_motor_control.Spindle_utils import SpindleController
from c_motor_control.Wheel_utils import WheelController
from concurrent.futures import ThreadPoolExecutor
from c_motor_control import abort


class MembraneSealer:
    def __init__(self, config):
        self.spindle_motor = self._try("Spindle", SpindleController, config['spindle'])
        self.wheel_motor   = self._try("Wheel",   WheelController,   config['wheel'])

    def _try(self, name, cls, config):
        try:
            return cls(config)
        except Exception as e:
            print(f"[{name}] Connection failed: {e}")
            return None

    def _motors(self):
        return [m for m in (self.wheel_motor, self.spindle_motor) if m is not None]

    def close(self):
        for motor in self._motors():
            try:
                motor.close()
            except Exception:
                pass

    # ---------------------------------------------------------------- sequences

    def initialize(self):
        if self.wheel_motor:   self.wheel_motor.enable_heater()
        # Wheel must be clear of the insert before the spindle turns.
        if self.wheel_motor:   self.wheel_motor.home()
        if self.spindle_motor: self.spindle_motor.home()
        if self.spindle_motor: abort.sleep(2)
        if self.spindle_motor: self.spindle_motor.move_to_notch()
        print("Initialized.")

    def seal(self, seal_point=None, dwell_time=None, rotations=1.0, spindle_velocity=None):
        """Press + rotate: lower the hot wheel onto the seam, dwell, roll the seam by
        turning the spindle, then lift and return the spindle to the notch."""
        if self.wheel_motor is None or self.spindle_motor is None:
            raise RuntimeError("Seal requires both Wheel and Spindle connected")
        if dwell_time is None:
            dwell_time = self.wheel_motor.config.get("dwell_time", 0)

        if not self.wheel_motor.heater_ready():
            print("[Wheel] WARNING: heater setpoint-OK not asserted; sealing anyway.")

        self.wheel_motor.lower(seal_point)
        abort.sleep(dwell_time)
        print(f"[Spindle] Rolling seam: {rotations} rotation(s)")
        self.spindle_motor.rotate_by(rotations, velocity=spindle_velocity)
        self.wheel_motor.lift()
        self.spindle_motor.move_to_notch()
        print("Seal complete.")

    def _run_concurrently(self, *tasks):
        tasks = [t for t in tasks if t is not None]
        with ThreadPoolExecutor(max_workers=len(tasks) or 1) as pool:
            futures = [pool.submit(t) for t in tasks]
        for f in futures:
            f.result()  # re-raise any exception from the worker threads

    # ---------------------------------------------------------------- stop / off

    def request_stop(self):
        """Signal any running operation to abort. Safe to call from another thread."""
        abort.request()

    def stop_and_home(self):
        """Halt every motor, lift the wheel off the membrane, then home the spindle.
        Call only once the aborted operation's thread has exited."""
        abort.clear()
        for motor in self._motors():
            try:
                motor.stop()
            except Exception as e:
                print(f"[{motor.name}] Stop failed: {e}")
        if self.wheel_motor:   self.wheel_motor.home()
        if self.spindle_motor: self.spindle_motor.home()
        print("Stopped. All components homed.")

    def off(self):
        if self.wheel_motor:   self.wheel_motor.disable_heater()
        if self.wheel_motor:   self.wheel_motor.home()
        if self.spindle_motor: self.spindle_motor.home()
        print("Shutdown complete. Heater off.")
