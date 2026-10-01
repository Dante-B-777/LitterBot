from __future__ import annotations

from dataclasses import dataclass
from glob import glob
import os
from pathlib import Path


@dataclass(frozen=True)
class DeviceResolution:
    path: str
    source: str


DEVICE_PROFILES = {
    'stm32': {
        'env_var': 'LITTERBOT_STM32_PORT',
        'by_id_patterns': (
            '/dev/serial/by-id/*STMicroelectronics*',
            '/dev/serial/by-id/*STM32*',
            '/dev/serial/by-id/*STLink*',
        ),
        'fallback_patterns': (
            '/dev/ttySTM32',
            '/dev/ttyACM*',
            '/dev/ttyUSB*',
        ),
    },
    'lidar': {
        'env_var': 'LITTERBOT_LIDAR_PORT',
        'by_id_patterns': (
            '/dev/serial/by-id/*RPLIDAR*',
            '/dev/serial/by-id/*rplidar*',
            '/dev/serial/by-id/*SLAMTEC*',
            '/dev/serial/by-id/*CP210*',
            '/dev/serial/by-id/*Silicon_Labs*',
        ),
        'fallback_patterns': (
            '/dev/ttyLIDAR',
            '/dev/ttyUSB*',
            '/dev/ttyACM*',
        ),
    },
}

AUTO_VALUES = {'', 'auto', 'AUTO', 'Auto'}


def _normalized_excludes(exclude_paths: list[str] | tuple[str, ...] | None) -> set[str]:
    excludes: set[str] = set()
    for path in exclude_paths or []:
        if not path:
            continue
        excludes.add(path)
        try:
            excludes.add(str(Path(path).resolve()))
        except OSError:
            pass
    return excludes


def _path_available(path: str, excludes: set[str]) -> bool:
    if not path:
        return False
    try:
        resolved = str(Path(path).resolve())
    except OSError:
        resolved = path
    return os.path.exists(path) and path not in excludes and resolved not in excludes


def _iter_candidates(patterns: tuple[str, ...], excludes: set[str]) -> list[str]:
    seen: set[str] = set()
    matches: list[str] = []
    for pattern in patterns:
        for path in sorted(glob(pattern)):
            if path in seen:
                continue
            seen.add(path)
            if _path_available(path, excludes):
                matches.append(path)
    return matches


def detect_serial_device(
    kind: str,
    preferred: str = '',
    exclude_paths: list[str] | tuple[str, ...] | None = None,
) -> DeviceResolution:
    profile = DEVICE_PROFILES.get(kind)
    if profile is None:
        raise ValueError(f'Unknown device kind: {kind}')

    excludes = _normalized_excludes(exclude_paths)

    if preferred not in AUTO_VALUES and _path_available(preferred, excludes):
        return DeviceResolution(preferred, 'launch override')

    env_value = os.getenv(profile['env_var'], '').strip()
    if env_value and _path_available(env_value, excludes):
        return DeviceResolution(env_value, f'env:{profile["env_var"]}')

    for candidate in _iter_candidates(profile['by_id_patterns'], excludes):
        return DeviceResolution(candidate, 'udev by-id')

    for candidate in _iter_candidates(profile['fallback_patterns'], excludes):
        return DeviceResolution(candidate, 'fallback scan')

    return DeviceResolution('', 'not found')
