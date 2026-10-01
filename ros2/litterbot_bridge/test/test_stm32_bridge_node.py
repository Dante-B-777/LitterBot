"""
test_stm32_bridge_node.py — Node-level behaviour tests for STM32Bridge.

These tests instantiate the STM32Bridge ROS2 node directly (mock serial mode)
and exercise its callback logic without needing to spin an executor.

WHY THESE TESTS EXIST
---------------------
The kinematics math and frame format are covered in other test files.
These tests cover the *state machine* behaviour of the node:
  - Does e-stop actually block motor commands?
  - Does clearing e-stop actually re-enable motor commands?
  - Does the node publish to /stm32_tx when expected?

Run with:
    pytest test/test_stm32_bridge_node.py -v

Note: rclpy.init() is called once per test session (module scope).
"""
import sys
import os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import rclpy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from litterbot_bridge.stm32_bridge import STM32Bridge


# ---------------------------------------------------------------------------
# Session-scoped rclpy init/shutdown
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialize and shut down rclpy once for the whole test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def bridge():
    """Create a fresh STM32Bridge node in mock mode for each test."""
    node = STM32Bridge()
    yield node
    node.destroy_node()


# ---------------------------------------------------------------------------
# E-stop state machine tests
# ---------------------------------------------------------------------------

def test_estop_inactive_by_default(bridge):
    """
    E-stop flag starts as False when the node initialises.

    WHY: If the robot boots with e-stop active it won't respond to any
    drive commands, requiring a manual reset before it can operate.
    """
    assert bridge._estop_active is False


def test_estop_activates_on_true(bridge):
    """
    Receiving Bool(True) on /emergency_stop sets _estop_active = True.

    WHY: This is the primary safety interlock. If the flag isn't set,
    the STOP button in the UI has no effect and the robot keeps moving.
    """
    msg = Bool()
    msg.data = True
    bridge._emergency_stop_callback(msg)
    assert bridge._estop_active is True


def test_estop_deactivates_on_false(bridge):
    """
    Receiving Bool(False) on /emergency_stop clears _estop_active.

    WHY: If clear doesn't work, the robot can never resume after an
    e-stop without restarting the node.
    """
    # Activate first
    msg = Bool()
    msg.data = True
    bridge._emergency_stop_callback(msg)
    assert bridge._estop_active is True

    # Now clear
    msg.data = False
    bridge._emergency_stop_callback(msg)
    assert bridge._estop_active is False


def test_cmd_vel_blocked_during_estop(bridge):
    """
    Motor commands are silently dropped while e-stop is active.

    WHY: The STM32 has a 250 ms motor watchdog. If cmd_vel callbacks
    keep sending M frames during e-stop, the watchdog would restart
    the motors even after the operator hit STOP.

    Verified by: activating e-stop, calling the callback directly,
    and confirming no /stm32_tx message was published.
    """
    published = []

    # Monkey-patch the publisher to capture calls
    original_publish = bridge._stm32_tx_pub.publish
    bridge._stm32_tx_pub.publish = lambda msg: published.append(msg.data)

    # Activate e-stop (this publishes "X" — clear the list after)
    estop_msg = Bool()
    estop_msg.data = True
    bridge._emergency_stop_callback(estop_msg)
    published.clear()  # ignore the "X" publish from estop activation

    # Now send a drive command — should be dropped
    twist = Twist()
    twist.linear.x = 0.3
    bridge._cmd_vel_callback(twist)

    assert len(published) == 0, (
        f"Motor command published during e-stop: {published}"
    )

    # Restore
    bridge._stm32_tx_pub.publish = original_publish


def test_cmd_vel_resumes_after_estop_clear(bridge):
    """
    Motor commands are processed normally after e-stop is cleared.

    WHY: Confirms the resume path works. If clearing e-stop doesn't
    re-enable the callback, the robot requires a full restart to drive
    again after every safety stop.
    """
    published = []
    original_publish = bridge._stm32_tx_pub.publish
    bridge._stm32_tx_pub.publish = lambda msg: published.append(msg.data)

    # Activate then clear e-stop
    msg = Bool()
    msg.data = True
    bridge._emergency_stop_callback(msg)
    published.clear()

    msg.data = False
    bridge._emergency_stop_callback(msg)
    published.clear()  # ignore "ESTOP CLEAR" message

    # Now send a drive command — should go through
    twist = Twist()
    twist.linear.x = 0.3
    bridge._cmd_vel_callback(twist)

    assert len(published) == 1, (
        "Expected one /stm32_tx publish after e-stop clear, "
        f"got {len(published)}"
    )
    assert published[0].startswith('M'), (
        f"Expected M frame, got: {published[0]!r}"
    )

    bridge._stm32_tx_pub.publish = original_publish


def test_estop_publishes_X_to_stm32_tx(bridge):
    """
    Activating e-stop publishes 'X' to /stm32_tx.

    WHY: The UI logs /stm32_tx in the Jetson→STM32 panel. If 'X' is
    not published, the operator has no visible confirmation that the
    e-stop command was actually sent to the STM32.
    """
    published = []
    original_publish = bridge._stm32_tx_pub.publish
    bridge._stm32_tx_pub.publish = lambda msg: published.append(msg.data)

    msg = Bool()
    msg.data = True
    bridge._emergency_stop_callback(msg)

    assert 'X' in published, (
        f"Expected 'X' in /stm32_tx, published: {published}"
    )

    bridge._stm32_tx_pub.publish = original_publish


def test_motor_command_publishes_M_frame_to_stm32_tx(bridge):
    """
    Sending a drive command publishes the M frame to /stm32_tx.

    WHY: The UI reads /stm32_tx to populate the blue 'Jetson→STM32'
    log panel. If this isn't published, the operator can't see what
    commands are being sent to the robot.
    """
    published = []
    original_publish = bridge._stm32_tx_pub.publish
    bridge._stm32_tx_pub.publish = lambda msg: published.append(msg.data)

    twist = Twist()
    twist.linear.x = 0.3
    bridge._cmd_vel_callback(twist)

    assert len(published) == 1
    assert published[0].startswith('M'), f"Expected M frame, got: {published[0]!r}"

    bridge._stm32_tx_pub.publish = original_publish
