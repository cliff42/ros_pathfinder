from gpiozero import PhaseEnableMotor
from threading import Lock

# class to server gpiozero PhaseEnableMotors: https://gpiozero.readthedocs.io/en/stable/api_output.html#gpiozero.PhaseEnableMotor
class GpioPhaseEnableMotor:
    def __init__(self, 
                 phase_pin: int, 
                 enable_pin: int, 
                 deadband = 0.005,
                 max_abs_effort = 1.0,
                 inverted = False,
    ) -> None:
        if deadband < 0.0:
            raise ValueError("deadband must be > 0")
        if not 0.0 < max_abs_effort <= 1.0:
            raise ValueError("max_abs_effort must be between 0 and 1")

        self._deadband = deadband
        self._max_abs_effort = max_abs_effort
        self._inverted = inverted

        self._closed = False
        self._motor = PhaseEnableMotor(phase_pin, enable_pin)     
        self._motor.stop() # make sure motor is initialized in a stopped position

        self._lock = Lock()

    def set_effort(self, effort: float) -> float:
        effort = max(-self._max_abs_effort, min(self._max_abs_effort, effort))

        if abs(effort) < self._deadband:
            effort = 0.0

        if self._inverted:
            effort = -effort

        with self._lock:
            self._require_open()

            if effort > 0.0:
                self._motor.forward(effort)
            elif effort < 0.0:
                self._motor.backward(abs(effort))
            else:
                self._motor.stop()
        
        return effort
    
    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            
            self._motor.stop()
            self._motor.close()
            self._closed = True
    
    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("motor has been closed")


        

