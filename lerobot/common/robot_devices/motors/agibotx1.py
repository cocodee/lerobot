import logging


class AgibotX1MotorsBus():
    def __init__(
        self,
        config: AgibotX1MotorsBusConfig,
    ):
        self.motors = config.motors
        self.mock = config.mock
        self.calibration = None
        self.is_connected = False
        self.logs = {}
    def motor_names(self) -> list[str]:
        pass
    def set_calibration(self, calibration: dict[str, list]):
        pass
    def apply_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):
        pass
    def revert_calibration(self, values: np.ndarray | list, motor_names: list[str] | None):
        pass
    def read(self, data_name, motor_names: str | list[str] | None = None):
        pass
    def write(self, data_name, values: int | float | np.ndarray, motor_names: str | list[str] | None = None):
        pass