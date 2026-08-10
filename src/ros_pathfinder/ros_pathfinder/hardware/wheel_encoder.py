import math
from dataclasses import dataclass
from typing import Optional, Protocol


class EncoderSensor(Protocol):
    def read_angle_rad(self) -> float:
        ...

    def close(self) -> None:
        ...

@dataclass
class WheelEncoderReading:
    timestamp_ns: int
    position_rad: float
    velocity_rad_s: Optional[float]

class WheelEncoder:
    def __init__(self, sensor: EncoderSensor, gear_ratio: float, direction: int = 1) -> None:
        self._closed = False
        self._sensor = sensor
        self._gear_ratio = gear_ratio
        self._direction = direction

        self._prev_sensor_angle_rad: Optional[float]= None
        self._prev_timestamp_ns: Optional[int] = None
        self._wheel_position_rad = 0.0

    def sample(self, timestamp_ns: int) -> WheelEncoderReading:
        sensor_angle_rad = self._sensor.read_angle_rad()

        if self._prev_sensor_angle_rad is None:
            self._prev_sensor_angle_rad = sensor_angle_rad
            self._prev_timestamp_ns = timestamp_ns

            return WheelEncoderReading(
                timestamp_ns=timestamp_ns,
                position_rad=self._wheel_position_rad,
                velocity_rad_s=None
            )

        assert self._prev_timestamp_ns is not None

        elapsed_ns = timestamp_ns - self._prev_timestamp_ns

        if elapsed_ns <= 0:
            raise ValueError("timestamps must increase")

        sensor_delta_rad = sensor_angle_rad - self._prev_sensor_angle_rad
        if sensor_delta_rad > math.pi:
            sensor_delta_rad -= (math.pi * 2.0)
        elif sensor_delta_rad < -math.pi:
            sensor_delta_rad += (math.pi * 2.0)

        wheel_delta_rad = (self._direction * sensor_delta_rad) / self._gear_ratio

        wheel_velocity_rad_s = wheel_delta_rad / (elapsed_ns * 1e-9)

        self._wheel_position_rad += wheel_delta_rad
        self._prev_sensor_angle_rad = sensor_angle_rad
        self._prev_timestamp_ns = timestamp_ns

        return WheelEncoderReading(
            timestamp_ns=timestamp_ns,
            position_rad=self._wheel_position_rad,
            velocity_rad_s=wheel_velocity_rad_s
        )

    def close(self) -> None:
        if self._closed:
            return
        
        self._sensor.close()
        self._closed = True