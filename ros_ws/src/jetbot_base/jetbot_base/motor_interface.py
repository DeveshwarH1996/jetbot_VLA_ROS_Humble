from abc import ABC, abstractmethod

class MotorInterface(ABC):
    """
    Hardware Abstraction Layer (HAL) for JetBot motors.
    Ensures that the ROS2 node is decoupled from the specific hardware driver.
    """

    @abstractmethod
    def set_speeds(self, left: float, right: float):
        """
        Set motor speeds.
        Args:
            left: Speed for left motor in range [-1.0, 1.0]
            right: Speed for right motor in range [-1.0, 1.0]
        """
        pass

    @abstractmethod
    def stop(self):
        """Stop all motors immediately."""
        pass

class MockMotorInterface(MotorInterface):
    """
    Mock implementation for testing without physical hardware.
    """
    def __init__(self):
        self.left = 0.0
        self.right = 0.0

    def set_speeds(self, left: float, right: float):
        self.left = left
        self.right = right

    def stop(self):
        self.left = 0.0
        self.right = 0.0

class WaveshareMotorInterface(MotorInterface):
    """
    Interface for Waveshare JetBot using the Adafruit MotorHAT.
    Requires: Adafruit_MotorHAT library.
    """
    def __init__(self, i2c_bus=1):
        try:
            from Adafruit_MotorHAT import Adafruit_MotorHAT
            self.driver = Adafruit_MotorHAT(i2c_bus=i2c_bus)
            self.motor_l = self.driver.getMotor(1)
            self.motor_r = self.driver.getMotor(2)
            self._adafruit = Adafruit_MotorHAT 
        except ImportError:
            print("Error: Adafruit_MotorHAT library not found. Falling back to Mock interface.")
            raise ImportError("Missing Adafruit_MotorHAT library")

    def set_speeds(self, left: float, right: float):
        self._set_motor(self.motor_l, left)
        self._set_motor(self.motor_r, right)

    def _set_motor(self, motor, value):
        speed = int(abs(value) * 255) 
        motor.setSpeed(speed)
        if value > 0:
            motor.run(self._adafruit.FORWARD)
        elif value < 0:
            motor.run(self._adafruit.BACKWARD)
        else:
            motor.run(self._adafruit.RELEASE)

    def stop(self):
        self.motor_l.run(self._adafruit.RELEASE)
        self.motor_r.run(self._adafruit.RELEASE)
