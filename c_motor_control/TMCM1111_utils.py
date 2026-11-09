from pytrinamic.connections import ConnectionManager
from pytrinamic.modules.TMCM1111 import TMCM1111
import time
from c_motor_control import abort

# import RPi.GPIO as GPIO


class TMCM1111Controller:
    '''Wrapper for TMCM1111 motor controller'''
    def __init__(self, config):
        self.config = config
        self.interface = ConnectionManager(f"--interface usb_tmcl --port {self.config.get('port')}").connect()
        self.module = TMCM1111(self.interface)
        self.motor = self.module.motors[0]
        self.name = self.config.get("name")
        self.AP = self.motor.AP


    def home(self):
        print(f"[{self.name}] Homing started...")

        self.motor.rotate(-self.config.get("home_search_velocity"))
        start_time = time.time()
        while True:
            not_triggered = self.motor.get_axis_parameter(self.AP.HomeSwitch) # 0: triggered, 1: not_triggered
            if not not_triggered:
                abort.sleep(self.config.get("home_centering_time"))
                self.motor.stop()
                self.wait_for_stop()
                print(f"[{self.name}] Switch triggered.")
                break
            if time.time() - start_time > self.config.get("home_timeout"):
                self.motor.stop()
                raise TimeoutError(f"[{self.name}] Homing timed out")
            abort.sleep(0.01)


        self.motor.set_axis_parameter(self.AP.ActualPosition, 0)
        print(f"[{self.name}] Homing complete. Position zeroed.")


    def rotate(self, velocity):
        self.motor.rotate(velocity)


    def stop(self):
        self.motor.stop()

    def wait_for_stop(self, timeout=15):
        """Block until the motor's actual velocity settles to zero."""
        start = time.time()
        while True:
            if self.motor.get_axis_parameter(self.AP.ActualVelocity, True) == 0:
                return
            if time.time() - start > timeout:
                raise TimeoutError(f"[{self.name}] Motor did not come to a stop in time")
            abort.sleep(0.01)

    def close(self):
        self.interface.close()
    
    def set_digital_output(self, pin: int) -> None:
        self.interface.set_digital_output(pin, self.module.module_id)

    def clear_digital_output(self, pin: int) -> None:
        self.interface.clear_digital_output(pin, self.module.module_id)

    def get_digital_output(self, pin: int) -> int:
        return self.interface.get_digital_output(pin, self.module.module_id)

    def get_digital_input(self, pin: int) -> int:
        return self.interface.get_digital_input(pin, self.module.module_id)
    
    def move_by(self, distance, velocity=None):
        """Move by a relative distance in microsteps."""
        self.motor.set_axis_parameter(self.AP.RampType, 0)
        if velocity is not None:
            self.motor.set_axis_parameter(self.AP.MaxPositioningSpeed, velocity)
        self.motor.move_by(distance)


    def move_to(self, position, velocity=None):
        """Move to an absolute position in microsteps."""
        if velocity is not None:
            self.motor.set_axis_parameter(self.AP.MaxPositioningSpeed, velocity)
        self.motor.move_to(position)
        self.wait_for_position()

    def wait_for_position(self, timeout=None):
        """Block until the motor reaches its target position.
        timeout=None waits indefinitely (abort still interrupts via abort.sleep)."""
        abort.sleep(0.05)  # let the controller clear PositionReachedFlag from the previous move
        start = time.time()
        while True:
            if self.motor.get_axis_parameter(self.AP.PositionReachedFlag):
                return
            if timeout is not None and time.time() - start > timeout:
                self.motor.stop()
                raise TimeoutError(f"[{self.name}] Did not reach target position in time")
            abort.sleep(0.01)