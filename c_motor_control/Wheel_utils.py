from c_motor_control.TMCM1111_utils import TMCM1111Controller
import time
from c_motor_control import abort


class WheelController(TMCM1111Controller):
    """Heated sealing wheel. Mechanically/electrically the same as the Roller's Staker:
    the heater SSR is driven from a digital output on this TMCM1111."""
    SETPOINT_OK_PIN = 1
    HEATER_PIN = 2

    def __init__(self, config):
        super().__init__(config)

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

        self.motor.set_axis_parameter(self.AP.ActualPosition, 0)
        print(f"[{self.name}] Homing complete. Position zeroed.")

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
