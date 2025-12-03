# Testing stepper_motor.py Without MicroPython Hardware

This project contains a complete testing framework that allows you to test the stepper motor control logic without requiring MicroPython or actual hardware.

## Project Structure

```
stepper/
├── src/                       # Source code (for Pico 2)
│   ├── stepper_motor.py       # Main motor control class
│   ├── rotor_angle.py         # Rotor angle calculations
│   └── electrical_cycle.py    # Electrical cycle calculations for microstepping
├── test/                      # Test code (local only)
│   ├── machine_mock.py        # Mock MicroPython machine module
│   ├── test_stepper_motor.py  # Unit tests
│   └── conftest.py            # pytest configuration
└── README_TESTING.md          # This file

## Installation

Install pytest if you haven't already:
```bash
pip install pytest
```

## Running Tests

### Run All Tests
```bash
# From the project root directory
python3 -m pytest test/ -v

# Or specifically the microstepper tests
python3 -m pytest test/test_stepper_motor.py -v
```

### Run Specific Test Class
```bash
python3 -m pytest test/test_stepper_motor.py::TestSectorRotation -v
```

## How It Works

The testing framework works by replacing the `machine` module with our mock before importing the source modules:

```python
import sys
from test import machine_mock
sys.modules['machine'] = machine_mock

# Now we can import from src without hardware
from src.stepper_motor import StepperMotor
from src.rotor_angle import RotorAngle
```

## Mock Features

The mock implementation provides:

1. **Pin Class** - Simulates GPIO pins
   - Tracks all pin operations
   - Stores pin values and modes
   - Records initialization and value changes

2. **PWM Class** - Simulates PWM control
   - Tracks frequency and duty cycle changes
   - Records all PWM operations
   - Converts duty_u16 values to percentages

3. **Operation Tracking** - Monitor hardware interactions
   - `machine_mock.get_all_operations()` - Get all tracked operations
   - `machine_mock.reset_all_tracking()` - Clear operation history
   - `machine_mock.print_operations()` - Display operations for debugging

## Test Categories

The test suite includes:

- **Initialization** - Motor setup and initial state
- **Alignment** - Rotor alignment algorithms
- **Sector Rotation** - Full-step rotation logic
- **Microstepping** - Fine positioning within sectors
- **Continuous Spin** - RPM-based rotation
- **Position Setting** - Arbitrary angle positioning
- **Phase Control** - PWM and pin control verification

## What Can Be Tested

✅ **Testable Without Hardware:**
- All mathematical calculations (angles, sectors, positions)
- Control flow logic (alignment, rotation sequences)
- State tracking (rotor position updates)
- Command generation (correct Pin/PWM values)
- Direction selection algorithms
- Parameter validation

❌ **Not Testable Without Hardware:**
- Actual motor movement
- Real PWM signal generation
- Physical torque and speed
- Electrical characteristics
- Hardware timing precision

## Writing New Tests

To add new tests:

1. Create a test class in `test_stepper_motor.py`
2. Use `setup_method` to initialize the motor
3. Use pytest fixtures from `conftest.py`
4. Mock `time.sleep` for fast execution

Example:
```python
class TestNewFeature:
    def setup_method(self):
        machine_mock.reset_all_tracking()
        self.motor = StepperMotor(
            ain1=0, ain2=1, pwma=2,
            bin1=3, bin2=4, pwmb=5
        )

    @patch('time.sleep')
    def test_my_feature(self, mock_sleep):
        # Your test code here
        self.motor.some_method()

        # Verify operations
        ops = machine_mock.get_all_operations()
        assert len(ops['pwm_operations']) > 0
```

## Debugging Tests

To debug failing tests:

1. **Print tracked operations:**
   ```python
   machine_mock.print_operations()
   ```

2. **Use pytest's -s flag to see print statements:**
   ```bash
   python3 -m pytest test_stepper_motor.py -s
   ```

3. **Use pytest's --pdb flag to drop into debugger on failure:**
   ```bash
   python3 -m pytest test_stepper_motor.py --pdb
   ```

## CI/CD Integration

This testing framework enables continuous integration since no hardware is required. You can:

- Run tests in GitHub Actions
- Add pre-commit hooks
- Test on multiple Python versions

## Limitations

While the mock provides good testing capabilities, remember:

1. Timing is simulated (time.sleep is mocked)
2. Electrical characteristics aren't modeled
3. Physical constraints aren't enforced
4. Real-world noise and interference aren't simulated

Always validate critical functionality on actual hardware before deployment.

## Troubleshooting

**ImportError: No module named 'machine'**
- Make sure you're importing machine_mock before stepper_motor
- Check that conftest.py is in the same directory

**Tests pass but hardware doesn't work**
- Verify pin numbers match your hardware
- Check power supply and connections
- Ensure PWM frequency is compatible with your driver

**Mock tracking shows no operations**
- Call `machine_mock.reset_all_tracking()` before your test
- Ensure you're not accidentally resetting after operations

## License

This testing framework is provided as-is for testing purposes.