"""
Mock implementation of MicroPython's machine module for testing.

This module provides mock Pin and PWM classes that simulate the behavior
of MicroPython's hardware control classes without requiring actual hardware.
"""

class Pin:
    """
    Mock Pin class that simulates a GPIO pin.
    Tracks all operations for verification in tests.
    """

    # Pin modes (matching MicroPython constants)
    IN  = 0
    OUT = 1

    # Class-level tracking of all pin operations
    _operations = []

    def __init__(self, pin_num, mode=None):
        """
        Initialize a mock pin.
        Parameters:
            pin_num: GPIO pin number
            mode:    Pin mode (Pin.IN or Pin.OUT)
        """
        self.pin_num = pin_num
        self.mode    = mode
        self._value  = 0
        
        # Track initialization
        Pin._operations.append({'type': 'init', 'pin': pin_num, 'mode': mode})

    def value(self, val=None):
        """
        Get or set pin value.
        Parameters:
            val: If provided, set the pin to this value (0 or 1)
                 If None, return current value
        Returns:
            Current pin value if val is None
        """
        if val is not None:
            self._value = val
            Pin._operations.append({'type': 'value_set', 'pin': self.pin_num, 'value': val})
        return self._value

    @classmethod
    def reset_tracking(cls):
        """Reset operation tracking."""
        cls._operations = []

    @classmethod
    def get_operations(cls):
        """Get list of all pin operations."""
        return cls._operations.copy()

class PWM:
    """
    Mock PWM class that simulates Pulse Width Modulation control.
    Tracks all PWM operations for verification in tests.
    """

    # Class-level tracking of all PWM operations
    _operations = []

    def __init__(self, pin_obj):
        """
        Initialize a mock PWM controller.
        Parameters:
            pin_obj: Pin object to use for PWM
        """
        self.pin       = pin_obj
        self._freq     = 0
        self._duty     = 0
        self._duty_u16 = 0

        # Track initialization
        PWM._operations.append({'type': 'init','pin': pin_obj.pin_num})

    def freq(self, frequency=None):
        """
        Get or set PWM frequency.

        Parameters:
            frequency: If provided, set frequency in Hz
                      If None, return current frequency

        Returns:
            Current frequency if frequency is None
        """
        if frequency is not None:
            self._freq = frequency
            PWM._operations.append({'type':'freq_set','pin':self.pin.pin_num,'frequency':frequency})
        return self._freq

    def duty_u16(self, duty=None):
        """
        Get or set PWM duty cycle (16-bit resolution).

        Parameters:
            duty: If provided, set duty cycle (0-65535)
                  If None, return current duty cycle

        Returns:
            Current duty cycle if duty is None
        """
        if duty is not None:
            self._duty_u16 = duty
            self._duty = duty / 65535.0  # Convert to 0.0-1.0 range
            # Track duty cycle changes
            PWM._operations.append({'type': 'duty_set','pin': self.pin.pin_num,
                                    'duty_u16': duty,'duty_percent': self._duty * 100})
        return self._duty_u16

    @classmethod
    def reset_tracking(cls):
        """Reset operation tracking (useful between tests)."""
        cls._operations = []

    @classmethod
    def get_operations(cls):
        """Get list of all PWM operations."""
        return cls._operations.copy()

# ==============================
#  Helper functions for testing
# ==============================

def reset_all_tracking():
    """Reset tracking for all mock classes."""
    Pin.reset_tracking()
    PWM.reset_tracking()


def get_all_operations():
    """Get all operations from all mock classes."""
    return {'pin_operations': Pin.get_operations(),
            'pwm_operations': PWM.get_operations()}

def print_operations():
    """Print all tracked operations (useful for debugging tests)."""
    ops = get_all_operations()

    print("\n=== Pin Operations ===")
    for op in ops['pin_operations']:
        print(f"  {op}")

    print("\n=== PWM Operations ===")
    for op in ops['pwm_operations']:
        print(f"  {op}")