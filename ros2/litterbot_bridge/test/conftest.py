# conftest.py — shared pytest configuration for all test files

def pytest_addoption(parser):
    parser.addoption(
        '--serial-port',
        default=None,
        help='Serial port for STM32 hardware tests (e.g. /dev/ttyUSB0)',
    )
