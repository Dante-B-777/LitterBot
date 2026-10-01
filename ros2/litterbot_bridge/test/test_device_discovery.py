import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from litterbot_bridge.device_discovery import DeviceResolution, detect_serial_device
import litterbot_bridge.device_discovery as dd


def test_preferred_override_wins(monkeypatch):
    monkeypatch.setattr(dd.os.path, 'exists', lambda path: path == '/dev/ttySTM32')

    result = detect_serial_device('stm32', preferred='/dev/ttySTM32')

    assert result == DeviceResolution('/dev/ttySTM32', 'launch override')


def test_env_var_used_when_preferred_is_auto(monkeypatch):
    monkeypatch.setenv('LITTERBOT_LIDAR_PORT', '/dev/ttyLIDAR')
    monkeypatch.setattr(dd.os.path, 'exists', lambda path: path == '/dev/ttyLIDAR')

    result = detect_serial_device('lidar', preferred='auto')

    assert result == DeviceResolution('/dev/ttyLIDAR', 'env:LITTERBOT_LIDAR_PORT')


def test_returns_not_found_when_no_match(monkeypatch):
    monkeypatch.delenv('LITTERBOT_STM32_PORT', raising=False)
    monkeypatch.setattr(dd.os.path, 'exists', lambda path: False)
    monkeypatch.setattr(dd, 'glob', lambda pattern: [])

    result = detect_serial_device('stm32')

    assert result == DeviceResolution('', 'not found')
