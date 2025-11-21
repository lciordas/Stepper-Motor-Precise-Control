"""
Module for representing stepper motor rotor angles with fixed-precision arithmetic.

This module provides the RotorAngle class for precise angle representation and
manipulation without floating-point rounding errors.
"""

class RotorAngle:
    """
    Represents the angular position of a stepper-motor rotor using fixed-precision
    integer arithmetic.

    Rotation angles are measured relative to the positive vertical axis; a positive
    angle corresponds to a clockwise rotation.

    One full revolution is divided into 200 equal sectors, each corresponding to the
    rotor turning one full step (1.8° / step or sector). A full step is the largest
    rotation the rotor can perform in a single step. The sectors are numbered from 1
    to 200, starting at the vertical reference position and increasing clockwise.

    We choose to represent an angle as a pair (sector, position_within_sector)
    because this structure aligns naturally with how the control software interacts
    with the motor (see class 'MicrostepperMotor' for how the current class is used).

    To avoid floating-point error accumulation, the position within a sector is
    stored using fixed precision. Each sector is subdivided into `SECTOR_TICKS`
    discrete integer ticks. 'SECTOR_TICKS' must be a power of 2 - this help with
    implementing micro-stepping, because a full step gets split into a number of
    micro-steps which is itself a power of 2 (# micro-steps <= SECTOR_TICKS).

    Therefore, an angle is represented as a tuple: (sector, ticks)
    where:
        • `sector` identifies which of the 200 sectors the rotor is in.
        • `ticks`  identifies the position within the sector as an
                   integer in the range [0, SECTOR_TICKS),

    This representation allows precise arithmetic on angles — addition, subtraction,
    and comparison — without any loss of accuracy from floating-point rounding.

    Public Interface:
    -----------------
    Class Constants:
        FULL_STEPS_PER_REV: int
            The number of full steps required for one complete revolution.

        SECTOR_COUNT: int
            The number of sectors into which the circle is divided.

        SECTOR_SIZE: float
            The size of each sector in degrees.

        SECTOR_TICKS: int, positive, a power of 2
            The number of discrete ticks within each sector for precision.

        TOTAL_TICKS: int
            The total number of discrete ticks around the complete circle.

    Properties:
        to_degrees: float
            Returns the rotor angle measured in degrees [0, 360).
            WARNING: For display only - do not use for calculations due to float precision.

        sector: int
            Returns the current sector number [1, 200].

        sector_position_in_ticks: int
            Returns the position within the sector in ticks [0, SECTOR_TICKS).

        sector_position_in_degrees: float
            Returns the position within the sector in degrees [0, SECTOR_SIZE).
            WARNING: For display only - do not use for calculations due to float precision.

        sector_half: int
            Returns which half of the sector the rotor is in (1 or 2).
            1 = first half, 2 = second half (clockwise from first).

    Class Methods:
        from_degrees(angle):
            Creates a RotorAngle from an angle in degrees.
            Parameters:
                angle: float - angle in degrees
            Returns:
                RotorAngle instance with the angle quantized to the nearest tick

    Methods:
        __init__(sector=1, ticks=0):
            Initialize a RotorAngle with specific sector and tick position.
            Parameters:
                sector: int - sector number [1, SECTOR_COUNT]
                ticks:  int - position within sector [0, SECTOR_TICKS)

        move_one_sector(clockwise):
            Moves the rotor angle by one sector in the specified direction.
            Modifies this instance in-place.
            Parameters:
                clockwise: bool - True for clockwise, False for counter-clockwise

        rotate_to_sector_boundary(clockwise):
            Moves the angle to the nearest sector boundary in the specified direction.
            Does nothing if already at a boundary.
            Parameters:
                clockwise: bool - True for clockwise, False for counter-clockwise

        __eq__(other):
            Checks equality with another RotorAngle.
            Returns True if both sector and position match exactly.

        __repr__():
            Returns a string representation for debugging.

        __str__():
            Returns a formatted string with angle details for display.
    """

    # Constants 
    FULL_STEPS_PER_REV = 200                          # the number of full steps the rotor needs to complete a revolution
    SECTOR_COUNT       = FULL_STEPS_PER_REV           # the number of sectors into which we partition the circle
    SECTOR_SIZE        = 360 / SECTOR_COUNT           # the size of a sector, in degrees
    SECTOR_TICKS       = 2^16                         # number of ticks into which we divide a sector (must be a power of 2)
    TOTAL_TICKS        = SECTOR_COUNT * SECTOR_TICKS  # the total number of discrete ticks around the complete circle

    @staticmethod
    def is_power_of_2(n):
        """
        Check wheter a positive integer is a positive power of two.
        """
        if not isinstance(n, int) or n < 0:
            return False
        
        # Common trick for testing if power of 2:
        # iff a power of two, binary representation of 'n' contains
        # only one '1'; the binary representation of 'n-1' flips all 
        # bits and shifts them => bitwise AND gives all 0s.
        return n & (n-1) == 0 
    
    def __init__(self, sector=1, ticks=0):
        """
        Initialize a RotorAngle.

        Parameters:
            sector: int - sector number (1-based) [1, FULL_STEPS_PER_REV]
            ticks : int - position within sector  [0, SECTOR_RESOLUTION)
        """
        assert RotorAngle.is_power_of_2(RotorAngle.SECTOR_COUNT), \
            f"RotorAngle.SECTOR_COUNT must be a power of 2, got {RotorAngle.SECTOR_COUNT}"        
        assert 1 <= sector <= RotorAngle.SECTOR_COUNT, \
            f"sector must be in [1, {RotorAngle.SECTOR_COUNT}], got {sector}"
        assert 0 <= ticks < RotorAngle.SECTOR_TICKS, \
            f"fixed_position must be in [0, {RotorAngle.SECTOR_TICKS}), got {ticks}"

        self._sector = sector   # the sector the rotor is in     (integer in [1, SECTOR_COUNT])
        self._ticks  = ticks    # the position within the sector (integer in [0, SECTOR_TICKS))

    @classmethod
    def from_degrees(cls, angle):
        """
        Create a RotorAngle from an angle in degrees.
        
        Angles are measured relative to the positive vertical axis.
        A positive angle corresponds to a clockwise rotation.
        
        Parameters:
            angle: float - angle in degrees

        Returns:
            RotorAngle: new instance (with input angle quantized)
        """
        # Restrict angle to [0, 360)
        angle_normalized = angle % 360

        # Convert angle to ticks (using integer arithmetic after rounding)
        angle_in_ticks = round(angle_normalized * cls.TOTAL_TICKS / 360)
        if angle_in_ticks == cls.TOTAL_TICKS:
            angle_in_ticks = 0

        # Calculate sector and position within sector
        sector             = (angle_in_ticks // cls.SECTOR_TICKS) + 1
        position_in_sector =  angle_in_ticks % cls.SECTOR_TICKS

        return cls(sector=sector, ticks=position_in_sector)

    @property
    def to_degrees(self):
        """
        The rotor angle measured in degrees [0, 360).

        WARNING:
        This representation is useful for logging and display, but do not use it 
        for calculations, as it is subject to floating-point error accumulation, 
        which this class is meant to prevent in the first place.

        Returns:
            float: angle in degrees
        """
        return (self._sector - 1) * self.SECTOR_SIZE + self.sector_position_in_degrees

    @property
    def sector(self):
        """
        The sector the rotor is in.

        Returns:
            sector: int - sector number [1, FULL_STEPS_PER_REV]
        """
        return self._sector

    @property
    def sector_position_in_ticks(self):
        """
        The rotor position within the sector it lies in, measured in ticks.

        Returns:
            int - position within sector in ticks [0, SECTOR_TICKS)
        """
        return self._ticks

    @property
    def sector_position_in_degrees(self):
        """
        The rotor position within the sector it lies in, measured in degrees.

        WARNING:
        This representation is useful for logging and display, but do not use it 
        for calculations, as it is subject to floating-point error accumulation, 
        which this class is meant to prevent in the first place.

        Returns:
            float: position within sector in degrees [0, SECTOR_SIZE)
        """
        return self._ticks * self.SECTOR_SIZE / self.SECTOR_TICKS

    @property
    def sector_half(self):
        """
        Which half of the sector the rotor lies in (1 or 2).
        The second half of a sector is positioned clockwise
        relative to its first half.

        Returns:
            int: 1 for first half, 2 for second half
        """
        return 1 if self._ticks < self.SECTOR_TICKS // 2 else 2

    def move_one_sector(self, clockwise):
        """
        Move the rotor angle by one sector in the specified direction.
        It modifies this instance in-place.

        Parameters:
            clockwise: bool - True for clockwise movement, False otherwise
        """
        if clockwise:
            self._sector = (self._sector % self.SECTOR_COUNT) + 1
        else:
            self._sector = ((self._sector - 2) % self.SECTOR_COUNT) + 1

    def rotate_to_sector_boundary(self, clockwise):
        """
        Move the angle to the nearest sector boundary in a specified direction.
        Does not do anything if already at a sector boundary.

        Parameters:
            clockwise: bool - True for clockwise rotation, False otherwise
        """
        # If already aligned, nothing to do
        if self._ticks == 0:
            return

        # If rotating clockwise, move to the next sector
        if clockwise:
            self._sector = (self._sector % self.SECTOR_COUNT) + 1

        # Align at sector boundary
        self._ticks = 0

    def __eq__(self, other):
        """
        Check equality with another RotorAngle.

        Parameters:
            other: RotorAngle - angle to compare with

        Returns:
            bool: True iff both sector and position in sector are equal
        """
        if not isinstance(other, RotorAngle):
            return False
        return self._sector == other._sector and self._ticks == other._ticks

    def __repr__(self):
        return f"RotorAngle(sector={self._sector}, sector_position={self._ticks})"

    def __str__(self):
        s  = f" angle              = {self.to_degrees:.6f}°\n"
        s += f" sector             = {self._sector}\n"
        s += f" in-sector position = {self._ticks}\n"
        s += f" in-sector angle    = {self.sector_position_in_degrees:.6f}°\n"
        return s