import math
from dataclasses import dataclass
from typing import Optional

import board
import busio
from adafruit_bno08x import BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C


@dataclass
class AngularVelocityReading:
    x_rad_s: float
    y_rad_s: float
    z_rad_s: float


class BNO08XIMU:
    DEFAULT_ADDRESS = 0x4A

    def __init__(
        self,
        address: int = DEFAULT_ADDRESS,
        i2c_frequency_hz: int = 400000,
    ) -> None:
        self._closed = False
        self._i2c_bus = busio.I2C(
            board.SCL,
            board.SDA,
            frequency=i2c_frequency_hz
        )

        try:
            self._device = BNO08X_I2C(self._i2c_bus, address=address)
            self._device.enable_feature(BNO_REPORT_GYROSCOPE)
        except Exception:
            self._i2c_bus.deinit()
            self._closed = True
            raise

    def read_angular_velocity(
        self,
    ) -> Optional[AngularVelocityReading]:
        self._require_open()

        reading = self._device.gyro
        if reading is None:
            return None

        return AngularVelocityReading(
            x_rad_s=reading[0],
            y_rad_s=reading[1],
            z_rad_s=reading[2],
        )

    def close(self) -> None:
        if self._closed:
            return

        self._i2c_bus.deinit()
        self._closed = True

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("IMU has been closed")
