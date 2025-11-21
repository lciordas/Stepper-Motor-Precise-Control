"""
pytest configuration file for stepper motor tests.

This file sets up the test environment by:
1. Ensuring the machine module is mocked before any imports
2. Providing common fixtures for tests
3. Mocking time.sleep for fast test execution
"""

import sys
import os
from unittest.mock import MagicMock, patch
import pytest

# Add parent and src directory to path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
src_dir = os.path.join(parent_dir, 'src')
sys.path.insert(0, parent_dir)
sys.path.insert(0, src_dir)

# Import our mock module
import machine_mock


def pytest_configure(config):
    """
    Called before test collection starts.

    Sets up the mock machine module in sys.modules to ensure
    it's available for all test imports.
    """
    # Replace the machine module with our mock
    sys.modules['machine'] = machine_mock

    # Optionally add markers for test categorization
    config.addinivalue_line(
        "markers", "hardware: marks tests that would require real hardware"
    )
    config.addinivalue_line(
        "markers", "slow: marks tests as slow running"
    )


@pytest.fixture(autouse=True)
def reset_mock_tracking():
    """
    Automatically reset mock tracking before and after each test.

    This ensures tests don't interfere with each other.
    """
    # Reset before test
    machine_mock.reset_all_tracking()

    # Run the test
    yield

    # Reset after test
    machine_mock.reset_all_tracking()


@pytest.fixture
def mock_time_sleep():
    """
    Mock time.sleep to make tests run instantly.

    Usage:
        def test_something(mock_time_sleep):
            # time.sleep is now mocked
            motor.spin(1, 60, "cw")
            assert mock_time_sleep.call_count > 0
    """
    with patch('time.sleep') as mock_sleep:
        yield mock_sleep


@pytest.fixture
def motor_instance():
    """
    Provide a pre-configured motor instance for tests.

    Usage:
        def test_something(motor_instance):
            motor_instance.spin_rotor(1, 60, "cw")
    """
    from microstepper import MicrostepMotor

    motor = MicrostepMotor(
        ain1=0, ain2=1, pwma=2,
        bin1=3, bin2=4, pwmb=5,
        verbose=False
    )

    # Reset tracking after initialization so tests start clean
    machine_mock.reset_all_tracking()

    return motor


@pytest.fixture
def verbose_motor_instance():
    """
    Provide a motor instance with verbose logging enabled.

    Useful for debugging tests.
    """
    from microstepper import MicrostepMotor

    motor = MicrostepMotor(
        ain1=0, ain2=1, pwma=2,
        bin1=3, bin2=4, pwmb=5,
        verbose=True
    )

    # Reset tracking after initialization
    machine_mock.reset_all_tracking()

    return motor


@pytest.fixture
def capture_operations():
    """
    Fixture to easily capture and analyze hardware operations.

    Returns a function that when called, returns categorized operations.

    Usage:
        def test_something(motor_instance, capture_operations):
            motor_instance._energize_phase('A', 1.0)
            ops = capture_operations()
            assert len(ops['phase_a_ops']) > 0
    """
    def get_categorized_ops():
        ops = machine_mock.get_all_operations()

        # Categorize operations by component
        categorized = {
            'phase_a_ops': [],
            'phase_b_ops': [],
            'pwm_freq_ops': [],
            'pwm_duty_ops': [],
            'all_pin_ops': ops['pin_operations'],
            'all_pwm_ops': ops['pwm_operations']
        }

        # Categorize pin operations
        for op in ops['pin_operations']:
            if op['type'] == 'value_set':
                if op['pin'] in [0, 1, 2]:  # Phase A pins
                    categorized['phase_a_ops'].append(op)
                elif op['pin'] in [3, 4, 5]:  # Phase B pins
                    categorized['phase_b_ops'].append(op)

        # Categorize PWM operations
        for op in ops['pwm_operations']:
            if op['type'] == 'freq_set':
                categorized['pwm_freq_ops'].append(op)
            elif op['type'] == 'duty_set':
                categorized['pwm_duty_ops'].append(op)

        return categorized

    return get_categorized_ops


# Optional: Add custom pytest options
def pytest_addoption(parser):
    """Add custom command line options."""
    parser.addoption(
        "--run-hardware",
        action="store_true",
        default=False,
        help="run tests that require real hardware"
    )
    parser.addoption(
        "--verbose-motor",
        action="store_true",
        default=False,
        help="enable verbose motor logging during tests"
    )


def pytest_collection_modifyitems(config, items):
    """Modify test collection based on markers and options."""

    # Skip hardware tests unless --run-hardware is passed
    if not config.getoption("--run-hardware"):
        skip_hardware = pytest.mark.skip(reason="need --run-hardware option to run")
        for item in items:
            if "hardware" in item.keywords:
                item.add_marker(skip_hardware)