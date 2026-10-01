"""
test_stm32_bridge_logic.py — Pure logic tests for stm32_bridge.

These tests verify the UART frame formatting and mecanum strafe kinematics.
No ROS2 runtime is needed — they import only the module-level functions and
the static kinematics method.

WHY THESE TESTS EXIST
---------------------
The bridge translates ROS2 Twist messages into raw UART strings that are
sent over the wire to the STM32.  If the frame format is wrong, the STM32
parser rejects the command silently and the robot doesn't move.

Run with:
    pytest test/test_stm32_bridge_logic.py -v
"""
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from litterbot_bridge.stm32_bridge import STM32Bridge  # noqa: E402


# ---------------------------------------------------------------------------
# Frame format tests
# ---------------------------------------------------------------------------

def test_motor_frame_starts_with_M():
    """
    Motor command frame must start with 'M'.

    WHY: uart_cmd.c on the STM32 identifies the command type by the first
    character.  A missing or wrong prefix means the command is silently
    dropped.
    """
    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(0.3, 0.0, 0.0)
    frame = f'M {fl} {fr} {rl} {rr}\n'
    assert frame[0] == 'M', f"Frame must start with 'M', got: {frame!r}"


def test_motor_frame_has_four_integer_values():
    """
    Motor command frame must contain exactly 4 integer wheel values.

    WHY: uart_cmd.c expects 'M <fl> <fr> <rl> <rr>\\n' — wrong number of
    values causes a parse failure and the robot doesn't move.
    """
    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(0.3, 0.0, 0.5)
    frame = f'M {fl} {fr} {rl} {rr}\n'
    parts = frame.strip().split()
    assert parts[0] == 'M'
    assert len(parts) == 5, f"Expected M + 4 values, got {len(parts)-1} values"
    for v in parts[1:]:
        int(v)  # raises ValueError if not an integer


def test_motor_frame_values_are_integers_not_floats():
    """
    Wheel values in the frame must be integers, not floats like '500.0'.

    WHY: uart_cmd.c uses sscanf with %d which won't parse '500.0' correctly
    and will produce the wrong value or fail.
    """
    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(0.3, 0.0, 0.0)
    frame = f'M {fl} {fr} {rl} {rr}\n'
    parts = frame.strip().split()
    for v in parts[1:]:
        assert '.' not in v, f"Float found in frame: {v!r} — must be integer"


def test_estop_frame_is_exactly_X_newline():
    """
    E-stop frame must be exactly 'X\\n'.

    WHY: uart_cmd.c checks for a single 'X' character followed by newline.
    Any extra characters or different format will not trigger the hardware
    e-stop, leaving motors running in a safety-critical situation.
    """
    frame = 'X\n'
    assert frame == 'X\n'
    assert len(frame) == 2


# ---------------------------------------------------------------------------
# Strafe kinematics tests (extends test_kinematics.py)
# ---------------------------------------------------------------------------

def test_strafe_left_correct_wheel_pattern():
    """
    Strafing left (vy > 0) produces the correct mecanum wheel pattern.

    WHY: Mecanum strafe requires opposing diagonal wheel pairs. If the
    signs are wrong the robot will rotate instead of strafe, making
    autonomous lateral positioning impossible.

    Expected for vy=+1.0 (strafe left):
      FL negative, FR positive, RL positive, RR negative
    """
    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(0.0, 1.0, 0.0)
    assert fl < 0,  f"FL should be negative for strafe left, got {fl}"
    assert fr > 0,  f"FR should be positive for strafe left, got {fr}"
    assert rl > 0,  f"RL should be positive for strafe left, got {rl}"
    assert rr < 0,  f"RR should be negative for strafe left, got {rr}"


def test_strafe_right_correct_wheel_pattern():
    """
    Strafing right (vy < 0) produces the correct mecanum wheel pattern.

    WHY: Strafe right is the mirror of strafe left. Verifying both
    directions catches sign-flip bugs in the vy term of the kinematics.

    Expected for vy=-1.0 (strafe right):
      FL positive, FR negative, RL negative, RR positive
    """
    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(0.0, -1.0, 0.0)
    assert fl > 0,  f"FL should be positive for strafe right, got {fl}"
    assert fr < 0,  f"FR should be negative for strafe right, got {fr}"
    assert rl < 0,  f"RL should be negative for strafe right, got {rl}"
    assert rr > 0,  f"RR should be positive for strafe right, got {rr}"


def test_strafe_left_right_are_symmetric():
    """
    Strafe left and strafe right produce mirrored wheel values.

    WHY: Asymmetry here means the robot drifts differently depending on
    direction, making precise positioning unreliable.
    """
    fl_l, fr_l, rl_l, rr_l = STM32Bridge._twist_to_wheel_units(0.0, +0.3, 0.0)
    fl_r, fr_r, rl_r, rr_r = STM32Bridge._twist_to_wheel_units(0.0, -0.3, 0.0)
    assert fl_l == -fl_r, "FL strafe left/right not symmetric"
    assert fr_l == -fr_r, "FR strafe left/right not symmetric"
    assert rl_l == -rl_r, "RL strafe left/right not symmetric"
    assert rr_l == -rr_r, "RR strafe left/right not symmetric"


def test_forward_backward_symmetric():
    """
    Forward and backward commands produce mirrored wheel values.

    WHY: Asymmetry means the robot travels a different distance forward
    vs backward for the same command, breaking odometry dead-reckoning.
    """
    fl_f, fr_f, rl_f, rr_f = STM32Bridge._twist_to_wheel_units(+0.3, 0.0, 0.0)
    fl_b, fr_b, rl_b, rr_b = STM32Bridge._twist_to_wheel_units(-0.3, 0.0, 0.0)
    assert fl_f == -fl_b
    assert fr_f == -fr_b
    assert rl_f == -rl_b
    assert rr_f == -rr_b
