from c_motor_control.TMCM1111_utils import TMCM1111Controller
from c_motor_control import abort


class SpindleController(TMCM1111Controller):
    def __init__(self, config):
        super().__init__(config)

    def home(self):
        if self.motor.get_axis_parameter(self.AP.HomeSwitch) == 0:
            self.motor.rotate(self.config.get("velocity"))
            abort.sleep(2)
            self.motor.stop()

        super().home()

        self.motor.set_axis_parameter(self.AP.ActualPosition, 0)

    def move_to_notch(self):
        self.move_to(self.config.get("notch_position"), velocity=self.config.get("velocity"))

    def full_rotation(self, wait=True):
        self.rotate_by(1.0, wait=wait)

    def rotate_by(self, rotations, velocity=None, wait=True):
        """Relative move of `rotations` spindle turns. Blocks until done unless wait=False."""
        steps = int(round(rotations * self.config.get("full_rotation_steps")))
        self.move_by(steps, velocity=velocity or self.config.get("velocity"))
        if wait:
            self.wait_for_position()
