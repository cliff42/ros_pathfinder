from abc import ABC

class Motor(ABC):
    def set_speed(self, speed: float) -> float:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

class MotorPair():
    def __init__(
            self,
            left: Motor,
            right: Motor
    ) -> None:
        self._left = left
        self._right = right

    def set_speeds(self, left_speed: float, right_speed: float) -> tuple[float, float]:
        try:
            set_left_speed = self._left.set_speed(left_speed)
            set_right_speed = self._right.set_speed(right_speed)
        except Exception:
            self.close()
            raise

        return set_left_speed, set_right_speed

    def stop(self) -> None:
        self._left.set_speed(0.0)
        self._right.set_speed(0.0)
    
    def close(self):
        self._left.close()
        self._right.close()
