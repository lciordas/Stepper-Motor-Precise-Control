# Stepper Motor Control Library

A MicroPython library for controlling bipolar stepper motors with arbitrary precission.<br>
For more details see the following blog posts:

**Rethinking Microstepping, Part1: A More Accurate Field-Control Method**<br>
Why the Standard Microstepping Method Is Only an Approximation — and How to Correct It<br>
https://medium.com/@22lciordas/rethinking-microstepping-a-more-accurate-field-control-method-845758315e32

**Rethinking Microstepping, Part 2: Implementing Arbitrary-Resolution Rotor Positioning**<br>
Turning theory into code: An open-loop MicroPython stepper motor driver for the Raspberry Pi Pico 2<br>
https://medium.com/@22lciordas/rethinking-microstepping-part-2-implementing-arbitrary-resolution-rotor-positioning-b5f646c5666f


## Motor Specifications

The library assumes:
- **Two-phase bipolar stepper motor with 4 poles per phase** (poles of the same phase are 90° apart; poles of the two phases alternate and are 45° apart)
- **50 rotor teeth** spaced 7.2° apart
- **200 steps per revolution** (1.8° per step)
- **H-bridge driver** (like TB6612FNG) for current control

## Project Structure

```
stepper/
├── src/                       # Source code (deploy to Pico 2)
│   ├── stepper_motor.py       # Main motor control class
│   ├── rotor_angle.py         # Rotor angle calculations
│   └── electrical_cycle.py    # Electrical cycle calculations for microstepping
├── test/                      # Test code (local development only)
│   ├── machine_mock.py        # Mock MicroPython machine module
│   ├── test_stepper_motor.py  # Unit tests
│   └── conftest.py            # pytest configuration
├── MicroStepping.ipynb        # Jupyter notebook for microstepping analysis
├── README.md                  # This file
└── README_TESTING.md          # Testing documentation
```

## Usage

Copy the files from `src/` to your Pico 2, then:

```python
from stepper_motor import StepperMotor

motor = StepperMotor(
    ain1=0, ain2=1, pwma=2,  # Phase A pins
    bin1=3, bin2=4, pwmb=5,  # Phase B pins
    electric_cycle_calculator="geometric"  # or "sinusoidal"
)

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

Two microstepping calculation methods are available:

- **Sinusoidal** (default): Standard approach using sinusoidal current waveforms. Simple and widely used, but introduces slight field magnitude and angle errors at intermediate microstep positions due to the 45° pole spacing in hybrid steppers.

- **Geometric**: Precise method that decomposes the desired magnetic field along the actual stator pole directions. Produces a constant-magnitude rotating field at all microstep positions. See `MicroStepping.ipynb` for a detailed analysis.

## API Reference

### StepperMotor Class

#### `__init__(ain1, ain2, pwma, bin1, bin2, pwmb, electric_cycle_calculator="sinusoidal")`
Initialize the motor controller.

**Parameters:**
- `ain1, ain2, pwma`: Phase A control pins
- `bin1, bin2, pwmb`: Phase B control pins
- `electric_cycle_calculator`: Method for calculating phase currents during microstepping
  - `"sinusoidal"`: Standard sinusoidal approximation (default)
  - `"geometric"`: Precise geometric decomposition accounting for 45° pole spacing

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

## Development

For local development and testing (no hardware required):

1. Clone this repository
2. Install pytest: `pip install pytest`
3. Run tests: `python3 -m pytest test/ -v`

The test files use mocks and should never be copied to your Pico 2. See [README_TESTING.md](README_TESTING.md) for detailed testing documentation.

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

### Inaccurate positioning
- Ensure motor is properly initialized (starts aligned)
- Check for mechanical issues (binding, loose couplings)
- Verify step delay is appropriate for your motor

### Import errors in tests
- Make sure you're running tests from the project root
- Ensure pytest is installed: `pip install pytest`