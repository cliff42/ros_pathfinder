from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class CollisionConfirmationConfig:
    required_consecutive_scans: int = 2

    def __post_init__(self) -> None:
        if self.required_consecutive_scans <= 0:
            raise ValueError("required_consecutive_scans must be positive")


class CollisionConfirmation:
    def __init__(self, config: CollisionConfirmationConfig) -> None:
        self._config = config
        self._last_scan_sequence: Optional[int] = None
        self._consecutive_collision_scans = 0

    @property
    def consecutive_collision_scans(self) -> int:
        return self._consecutive_collision_scans

    @property
    def last_scan_sequence(self) -> Optional[int]:
        return self._last_scan_sequence

    def update(
        self,
        collision_detected: bool,
        scan_sequence: int,
    ) -> bool:
        if scan_sequence < 0:
            raise ValueError("scan_sequence cannot be negative")

        if scan_sequence == self._last_scan_sequence:
            return (
                self._consecutive_collision_scans
                >= self._config.required_consecutive_scans
            )

        self._last_scan_sequence = scan_sequence
        if collision_detected:
            self._consecutive_collision_scans += 1
        else:
            self._consecutive_collision_scans = 0

        return (
            self._consecutive_collision_scans
            >= self._config.required_consecutive_scans
        )
