"""
test_kinematics.py — Unit tests for stm32_bridge mecanum kinematics.

These tests verify:
  1. Full-forward Twist produces firmware-range output [-1000, 1000]
  2. Zero Twist produces all zeros
  3. Output is clamped to [-1000, 1000]
  4. All-forward command produces equal positive values for all four wheels
"""
import sys
import os

# Allow import without ROS2 environment
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from litterbot_bridge.stm32_bridge import STM32Bridge  # noqa: E402 — path hack above


def test_zero_twist_gives_zero_output():
    """Stopped robot → all wheel commands are 0."""
    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(0.0, 0.0, 0.0)
    assert fl == 0 and fr == 0 and rl == 0 and rr == 0


def test_full_forward_output_in_firmware_range():
    """Large forward velocity → values in [-1000, 1000], not [-100, 100]."""
    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(1.0, 0.0, 0.0)
    # All four wheels should be equal and positive for pure forward motion
    assert fl == fr == rl == rr
    assert fl > 0
    # Must be in firmware range — if this is ≤ 100 the bug is still present
    assert fl > 100, f"Expected >100 (firmware range), got {fl} — speed scaling bug!"
    assert fl <= 1000


def test_output_clamped_to_firmware_range():
    """Very large velocity → clamped to ±1000."""
    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(100.0, 0.0, 0.0)
    assert fl == 1000
    assert fr == 1000

    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(-100.0, 0.0, 0.0)
    assert fl == -1000


def test_pure_rotation_left_right_asymmetry():
    """Counter-clockwise rotation → left wheels negative, right wheels positive."""
    fl, fr, rl, rr = STM32Bridge._twist_to_wheel_units(0.0, 0.0, 1.0)
    assert fl < 0 and rl < 0, "Left wheels should be negative for CCW rotation"
    assert fr > 0 and rr > 0, "Right wheels should be positive for CCW rotation"
