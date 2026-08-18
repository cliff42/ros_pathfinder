"""Run both drive motors forward briefly without starting ROS."""

import argparse
import time

from ros_pathfinder.hardware.base_motor import MotorPair
from ros_pathfinder.hardware.gpio_motor import GpioPhaseEnableMotor


LEFT_PHASE_PIN = 20
LEFT_ENABLE_PIN = 21
RIGHT_PHASE_PIN = 23
RIGHT_ENABLE_PIN = 24

TEST_DURATION_S = 3.0
DEFAULT_EFFORT = 0.25
MAX_TEST_EFFORT = 0.50
MOTOR_DEADBAND = 0.002


def actuate_both_motors_forward(effort: float = DEFAULT_EFFORT) -> None:
    """Drive both motors forward for three seconds and always stop them."""
    if not 0.0 < effort <= MAX_TEST_EFFORT:
        raise ValueError(
            f'effort must be greater than 0 and at most {MAX_TEST_EFFORT}'
        )

    created_motors = []
    try:
        left_motor = GpioPhaseEnableMotor(
            phase_pin=LEFT_PHASE_PIN,
            enable_pin=LEFT_ENABLE_PIN,
            deadband=MOTOR_DEADBAND,
            max_abs_effort=MAX_TEST_EFFORT,
            inverted=False,
        )
        created_motors.append(left_motor)

        right_motor = GpioPhaseEnableMotor(
            phase_pin=RIGHT_PHASE_PIN,
            enable_pin=RIGHT_ENABLE_PIN,
            deadband=MOTOR_DEADBAND,
            max_abs_effort=MAX_TEST_EFFORT,
            inverted=False,
        )
        created_motors.append(right_motor)

        motor_pair = MotorPair(left_motor, right_motor)
        print(
            f'Driving both motors forward at {effort:.0%} effort for '
            f'{TEST_DURATION_S:.1f} seconds...'
        )
        motor_pair.set_efforts(effort, effort)
        time.sleep(TEST_DURATION_S)
    finally:
        for motor in created_motors:
            motor.close()

    print('Motor test complete; both motors are stopped.')


def main() -> None:
    """Parse the optional test effort and run the motor smoke test."""
    parser = argparse.ArgumentParser(
        description='Drive both motors forward for three seconds.',
    )
    parser.add_argument(
        '--effort',
        type=float,
        default=DEFAULT_EFFORT,
        help=(
            'PWM effort from 0 to 0.5; defaults to '
            f'{DEFAULT_EFFORT}'
        ),
    )
    args = parser.parse_args()
    actuate_both_motors_forward(args.effort)


if __name__ == '__main__':
    main()
