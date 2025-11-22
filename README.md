# Stepper Motor Control Library

A MicroPython library for controlling bipolar stepper motors with microstepping capability.

## Features

- **Full and microstepping control** - Support for both full-step and microstep positioning
- **Precise angle positioning** - Position the rotor to any angle with high precision
- **Speed control** - Spin the motor at specified RPM
- **Direction control** - Clockwise, counter-clockwise, or shortest path
- **Comprehensive testing** - Full test suite that runs without hardware

## Project Structure

```
stepper/
├── src/                       # Source code (deploy to Pico 2)
│   ├── stepper_motor.py       # Main motor control class
│   └── rotor_angle.py         # Rotor angle calculations
├── test/                      # Test code (local development only)
│   ├── machine_mock.py        # Mock MicroPython machine module
│   ├── test_stepper_motor.py  # Unit tests
│   ├── test_example.py        # Example/demo script
│   └── conftest.py            # pytest configuration
├── README.md                  # This file
└── README_TESTING.md          # Testing documentation
```

## Deployment to Pico 2

### Copying files to your MicroPython device

1. Copy **only** the files from `src/` to your Pico 2:
   - `stepper_motor.py`
   - `rotor_angle.py`

2. On your Pico 2, import and use:

```python
from stepper_motor import StepperMotor

# Initialize motor with pin connections
motor = StepperMotor(
    ain1=0, ain2=1, pwma=2,  # Phase A pins
    bin1=3, bin2=4, pwmb=5,  # Phase B pins
    verbose=True             # Enable debug logging
)
```

### Local Development and Testing

For development and testing on your computer (no hardware required):

1. Clone this repository
2. Install pytest: `pip install pytest`
3. Run tests: `python3 -m pytest test/ -v`

**Note:** The test files use mocks and should never be copied to your Pico 2.

## Usage Examples

### Basic Positioning

```python
# Position rotor to 90 degrees
motor.set_rotor(90.0, "cw")

# Position using shortest path
motor.set_rotor(270.0, "closest")
```

### Continuous Rotation

```python
# Spin 5 revolutions at 60 RPM clockwise
motor.spin_rotor(5, 60, "cw")

# Spin forever at 120 RPM
from math import inf
motor.spin_rotor(inf, 120, "ccw")
```

### Microstepping

The motor automatically uses microstepping for precise positioning within sectors:

```python
# Position to 45.5 degrees (between full steps)
motor.set_rotor(45.5, "cw")
```

## Motor Specifications

The library assumes:
- **Two-phase bipolar stepper motor**
- **200 steps per revolution** (1.8° per step)
- **50 rotor teeth** spaced 7.2° apart
- **4 poles per phase** spaced 90° apart
- **H-bridge driver** (like TB6612FNG) for current control

## API Reference

### StepperMotor Class

#### `__init__(ain1, ain2, pwma, bin1, bin2, pwmb, verbose=False)`
Initialize the motor controller.

**Parameters:**
- `ain1, ain2, pwma`: Phase A control pins
- `bin1, bin2, pwmb`: Phase B control pins
- `verbose`: Enable debug logging

#### `set_rotor(target_angle, direction, delay=0.01)`
Position the rotor to a specific angle.

**Parameters:**
- `target_angle`: Target position in degrees (0-360)
- `direction`: "cw", "ccw", or "closest"
- `delay`: Time between steps in seconds

#### `spin_rotor(num_revolutions, rpm, direction)`
Spin the motor continuously.

**Parameters:**
- `num_revolutions`: Number of revolutions (can be `inf`)
- `rpm`: Speed in revolutions per minute
- `direction`: "cw" or "ccw"

#### `align_rotor(direction)`
Align the rotor to the nearest sector boundary.

**Parameters:**
- `direction`: "cw", "ccw", or "closest"

#### Properties

- `is_aligned`: Boolean indicating if rotor is at a sector boundary
- `aligned_position`: Current aligned position (1-4) or None

## Testing

The project includes comprehensive tests that run without hardware:

```bash
# Run all tests
python3 -m pytest test/ -v

# Run example demonstration
python3 test/test_example.py
```

See [README_TESTING.md](README_TESTING.md) for detailed testing documentation.

## Hardware Setup

### Wiring Example (TB6612FNG H-Bridge)

```
Microcontroller     TB6612FNG       Motor
--------------     -----------     -------
GPIO 0     ------>   AIN1
GPIO 1     ------>   AIN2
GPIO 2     ------>   PWMA    ----> Phase A
GPIO 3     ------>   BIN1
GPIO 4     ------>   BIN2
GPIO 5     ------>   PWMB    ----> Phase B
GND        ------>   GND
3.3V/5V    ------>   VCC
                     VM       <---- Motor Power
```

## License

This project is provided as-is for educational and development purposes.

## Contributing

Contributions are welcome! Please ensure all tests pass before submitting pull requests.

## Troubleshooting

### Motor not moving
- Check power supply voltage and current
- Verify pin connections
- Enable verbose mode to see debug output

### Inaccurate positioning
- Ensure motor is properly initialized (starts aligned)
- Check for mechanical issues (binding, loose couplings)
- Verify step delay is appropriate for your motor

### Import errors in tests
- Make sure you're running tests from the project root
- Ensure pytest is installed: `pip install pytest`