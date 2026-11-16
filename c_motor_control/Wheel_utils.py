from c_motor_control.TMCM1111_utils import TMCM1111Controller
import time
from c_motor_control import abort


class WheelController(TMCM1111Controller):
    """Heated sealing wheel. Mechanically/electrically the same as the Roller's Staker.
    GPIO header (TMCM-1111 FW manual V1.13, SIO/GIO tables pp. 37-40):
      heater MOSFET  -> header pins 2 (GND) + 4 (PWMU0) -> SIO 2, bank 2
      setpoint line  -> header pins 1 (GND) + 3 (PWMD0) -> GIO 1, bank 0
    """
    SETPOINT_OK_PIN = 1   # GIO port 1 = PWMD0 (input)
    HEATER_PIN = 2        # SIO port 2 = PWMU0 (output)
    IO_MASK = 7           # GP 78: PWMU0-2 outputs, PWMD0-2 inputs (factory default)

    def __init__(self, config):
        super().__init__(config)
        # GP 78 lives in EEPROM; enforce it so PWMU0 drives and PWMD0 reads. Only write on mismatch.
        if self.interface.get_global_parameter(78, 0) != self.IO_MASK:
            self.interface.set_global_parameter(78, 0, self.IO_MASK)

    # ------------------------------------------------------------------ heater
    # Staker heater topped out at ~240 C; assumed identical here.
    def enable_heater(self):
        self.set_digital_output(self.HEATER_PIN)

    def disable_heater(self):
        self.clear_digital_output(self.HEATER_PIN)

    def heater_ready(self) -> bool:
        return bool(self.get_digital_input(self.SETPOINT_OK_PIN))

    # ------------------------------------------------------------------ motion
    def home(self):
        """Double-approach homing (ported unchanged from StakerController)."""
        print(f"[{self.name}] Homing started...")

        # If already sitting on the switch, back off until it releases.
        while True:
            not_triggered = self.motor.get_axis_parameter(self.AP.HomeSwitch)

            if not not_triggered:
                self.motor.rotate(self.config.get("home_search_velocity"))
                abort.sleep(2)
                self.motor.stop()
                self.wait_for_stop()

            if not_triggered:
                break

        # Fast seek, back off, slow seek.
        self.motor.rotate(-self.config.get("home_search_velocity"))
        start_time = time.time()
        while True:
            not_triggered = self.motor.get_axis_parameter(self.AP.HomeSwitch)  # 0: triggered, 1: not_triggered
            if not not_triggered:
                self.motor.stop()
                self.wait_for_stop()

                self.motor.rotate(self.config.get("home_search_velocity"))
                abort.sleep(1)
                self.motor.stop()
                self.wait_for_stop()

                self.motor.rotate(-self.config.get("home_search_velocity") // 3)
                while True:
                    not_triggered_copy = self.motor.get_axis_parameter(self.AP.HomeSwitch)
                    if not not_triggered_copy:
                        self.motor.stop()
                        self.wait_for_stop()
                        break
                    abort.sleep(0.001)

                centering_time = self.config.get("home_centering_time")
                if centering_time:
                    self.motor.rotate(-self.config.get("home_search_velocity") // 3)
                    abort.sleep(centering_time)
                    self.motor.stop()
                    self.wait_for_stop()
                break
            if time.time() - start_time > self.config.get("home_timeout"):
                self.motor.stop()
                raise TimeoutError(f"[{self.name}] Homing timed out")
            abort.sleep(0.01)

        # Zero stays at the switch so seal_point/lift_point don't shift when home_position is tuned.
        self.motor.set_axis_parameter(self.AP.ActualPosition, 0)
        print(f"[{self.name}] Switch found. Position zeroed.")

        home_position = self.config.get("home_position", 0)
        if home_position:
            print(f"[{self.name}] Moving to home position {home_position}")
            self.move_to(home_position, velocity=self.config.get("home_search_velocity") // 3)
        print(f"[{self.name}] Homing complete.")

    def lower(self, point=None):
        """Move onto the seam (blocking)."""
        if point is None:
            point = self.config.get("seal_point")
        print(f"[{self.name}] Lowering to {point}")
        self.move_to(point, velocity=self.config.get("velocity"))

    def lift(self, point=None):
        """Move clear of the seam (blocking)."""
        if point is None:
            point = self.config.get("lift_point")
        print(f"[{self.name}] Lifting to {point}")
        self.move_to(point, velocity=self.config.get("velocity") * 3)
