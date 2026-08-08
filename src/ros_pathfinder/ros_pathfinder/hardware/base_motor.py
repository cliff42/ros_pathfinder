from abc import ABC

class Motor(ABC):
    def set_effort(self, effort: float) -> float:
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

    def set_efforts(self, left_effort: float, right_effort: float) -> tuple[float, float]:
        try:
            set_left_speed = self._left.set_effort(left_effort)
            set_right_speed = self._right.set_effort(right_effort)
        except Exception:
            self.close()
            raise

        return set_left_speed, set_right_speed

    def stop(self) -> None:
        self._left.set_effort(0.0)
        self._right.set_effort(0.0)
    
    def close(self):
        self._left.close()
        self._right.close()
