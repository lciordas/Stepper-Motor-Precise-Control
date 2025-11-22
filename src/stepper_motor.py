from machine import Pin, PWM
from math    import cos, sin, pi, isinf
import time

from electric_cycle import calculate_electric_cycle, calculate_currents_sinusoidal
from rotor_angle    import RotorAngle

class StepperMotor:
    """
    This interface controls a two-phase bipolar stepper motor.

    We assume the following motor characteristics:
    + Two phases: Phase A and Phase B
    + Bipolar drive: each phase has two terminals, with current direction controlled by an H-bridge
    + Pole layout: each phase has 4 poles spaced 90° apart; poles from different phases are offset by 45°
    + Pole polarity: when a phase is energized, its four poles alternate in polarity
    + Rotor: 50 teeth, spaced 7.2° apart

    When describing mechanical configuration, we picture the motor such that the eight stator
    poles are at angles of 0°, 45°, 90°, 135°, 180°, 225°, 270°, and 315° relative to vertical.
    By convention, the top pole (0°) belongs to Phase A, and positive rotation is clockwise.

    The motor's basic operating mode is full-stepping.
    In this mode, we cycle through four electrical states:
        1. Phase A: max positive current; Phase B: not energized
        2. Phase A: not energized;        Phase B: max positive current
        3. Phase A: max negative current; Phase B: not energized
        4. Phase A: not energized;        Phase B: max negative current

    Each change of electrical state (a 90° shift in electrical angle) moves the rotor 1.8°,
    known as a full step. This is the largest increment the rotor can move in a single step.
    A full revolution therefore requires 360° / 1.8° = 200 full steps.
    Cycling through all four states — one full electrical cycle — produces 7.2° of mechanical
    rotation, equal to one rotor-tooth pitch.

    We say the rotor is 'aligned' when one of its teeth lines up exactly with a stator pole.
    Due to rotor symmetry, the tooth on the opposite side of the rotor simultaneously aligns
    with the opposite stator pole. However, aligning a rotor tooth with one stator pole means
    the other (non-opposing) poles are not aligned with rotor teeth. This is because these teeth
    are spaced every 7.2°, while the stator poles are spaced every 45°, so only one pole pair can
    line up with opposing rotor teeth at any given moment.

    There are four unique rotor alignments possible within one electrical cycle.
    We label them positions 1 through 4:
        + position 1: tooth aligned with top pole               (tooth makes   0° angle with the vertical)
        + position 2: tooth aligned with pole at  45° clockwise (tooth makes 1.8° angle with the vertical)
        + position 3: tooth aligned with pole at  90° clockwise (tooth makes 3.6° angle with the vertical)
        + position 4: tooth aligned with pole at 135° clockwise (tooth makes 5.4° angle with the vertical)

    Public Interface:
    -----------------
    Properties:
        is_aligned: bool
            Returns True if the rotor is aligned with a sector boundary, False otherwise.

        aligned_position: int or None
            Returns the aligned position (1-4) if the rotor is aligned, None otherwise.
            The position indicates which of the four electrical states the motor is in.

        rotor_angle: RotorAngle
            The current angular position of the rotor, represented with fixed-precision arithmetic.

    Methods:
        __init__(ain1, ain2, pwma, bin1, bin2, pwmb, verbose=False):
            Initialize the motor controller with GPIO pin assignments for the H-bridge.
            The motor is energized to align a rotor tooth with the top stator pole.

        align_rotor(direction):
            Aligns a rotor tooth with the nearest stator pole in the specified direction.
            Parameters:
                direction: "cw" (clockwise), "ccw" (counter-clockwise), or "closest"

        spin_rotor(num_revolutions, rpm, direction):
            Spins the motor continuously for a specified number of revolutions at a given speed.
            Parameters:
                num_revolutions: Number of complete revolutions (can be fractional or inf)
                rpm:             Speed in revolutions per minute
                direction:       "cw" (clockwise) or "ccw" (counter-clockwise)

        turn_rotor(target_angle, direction, delay=0.01):
            Positions the rotor to a specific angle with microstepping precision.
            Parameters:
                target_angle: Target position in degrees (0-360)
                direction:    how to get there - "cw" (clockwise), "ccw" (counter-clockwise), or "closest"
                delay:        Time delay between rotation steps in seconds
    """

    # Constants
    PWM_FREQUENCY  = 25000   # PWM frequency (higher = quieter, smoother)
    MAX_MICROSTEPS = 2**4    # Maximum number of micro-steps into which we can break a full step (must be a power of 2).
    MIN_DELAY      = 0.01    # Minimum delay between successive steps; needed for the motor to settle down (typically 10ms is safe)

    def __init__(self, ain1, ain2, pwma, bin1, bin2, pwmb):
        """
        Initialize the motor and its interface.
        The motor is energized such that a rotor tooth is aligned with the top stator pole.

        Parameters:
            ain1: number of Pico 2 GPIO pin connected to Phase A input 1 pin on H-bridge
            ain2: number of Pico 2 GPIO pin connected to Phase A input 2 pin on H-bridge 
            pwma: number of Pico 2 GPIO pin connected to Phase A PWM pin on H-bridge
            bin1: number of Pico 2 GPIO pin connected to Phase B input 1 pin on H-bridge
            bin2: number of Pico 2 GPIO pin connected to Phase A input 2 pin on H-bridge
            pwmb: number of Pico 2 GPIO pin connected to Phase B PWM pin on H-bridge
        """
        # The max number of micro-steps into which we can break a full step must
        # be a power of 2 and cannot larger than the number of ticks in a sector.
        assert RotorAngle.is_power_of_2(StepperMotor.MAX_MICROSTEPS)
        assert StepperMotor.MAX_MICROSTEPS <= RotorAngle.SECTOR_TICKS

        # Configure GPIO pins
        # -- Phase A
        self.ain1 = Pin(ain1, Pin.OUT)
        self.ain2 = Pin(ain2, Pin.OUT)
        self.pwma = PWM(Pin(pwma))        
        self.pwma.freq(StepperMotor.PWM_FREQUENCY)
        # -- Phase B    
        self.bin1 = Pin(bin1, Pin.OUT)
        self.bin2 = Pin(bin2, Pin.OUT)
        self.pwmb = PWM(Pin(pwmb))
        self.pwmb.freq(StepperMotor.PWM_FREQUENCY)

        # Pre-calculate the current intensities for phases A and B throughout a complete 
        # electrical cycle, divided into micro-steps. The cycle consists of 4 full steps, 
        # each subdivided into a number of micro-steps (here use maximum number allowed).
        self.electric_cycle = \
            calculate_electric_cycle(StepperMotor.MAX_MICROSTEPS, calculate_currents_sinusoidal)

        # Energize the motor using the first configuration in the electric cycle.
        # This translates into a mechanical configuration in which one rotor tooth 
        # aligns with one of the poles (actually 2 opposite teeth are both aligned
        # with two opposite poles).
        self._energize_phase(phase='A', I=1.0)
        self._energize_phase(phase='B', I=0.0)

        # Imagine rotating the motor so that the two stator poles aligned with the 
        # rotor teeth lie at the top and bottom of the circular stator. We define 
        # this mechanical arrangement as the 0° reference position for the rotor.
        # See 'RotorAngle' docstring for an explanations of 'sector' and 'ticks'.
        self.rotor_angle = RotorAngle(sector=1, ticks=0)
                
    @property
    def is_aligned(self):
        """
        Whether any rotor tooth is aligned with a stator pole.

        This equivalent to the rotor being aligned with a sector boundary.
        See class docstring for more details.

        Returns:
            bool: True if aligned
        """
        return self.rotor_angle.sector_position_in_ticks == 0

    @property
    def aligned_position(self):
        """
        The aligned position (1-4) if the rotor is aligned, None otherwise.

        There are 4 possible aligned positions, depending on which pair 
        of stator poles is aligned with a pair of opposite rotor teeth.

        Returns:
            int or None: aligned position number (1-4) or None
        """
        if not self.is_aligned:
            return None
        return (self.rotor_angle.sector - 1) % 4 + 1

    def align_rotor(self, direction):
        """
        Aligns a pair of opposite rotor teeth with a pair of stator poles.

        The rotor will be turned by the smallest angle that brings it to 
        an aligned position, in the direction specified. We can request
        that the rotor is turned (counter)-clockwise, or just request it
        to be turned to the closest aligned configuration.
        
        Parameters
        ----------
        direction: in which direction to turn the rotor in order to align it ("cw", "ccw", "closest")
        """
        assert direction in ("cw", "ccw", "closest")

        # Nothing to do if the rotor is aligned already
        if self.is_aligned:
            return

        # Decide which of the four aligned configurations to use. This depends on the current rotor angle.
        # We conceptually divide the full circle into 200 sectors of 1.8°. The rotor is aligned when its 
        # angle lies exactly on one of these sector boundaries.
        # Since we know the rotor is currently not aligned, its angle must lie inside one of these sectors.
        # Aligning the rotor therefore means rotating it to one of the two sector boundaries. Which one we
        # choose is controlled via the 'direction' argument: the closest boundary or the boundary positioned 
        # (counter)-clockwise.

        sector_norm  = (self.rotor_angle.sector - 1) % 4 + 1   # normalize sector to 1-4
        position_ccw = sector_norm                             # the counter-clockwise aligned position (1-4)
        position_cw  = (sector_norm % 4) + 1                   # the clocwise aligned position (1-4)

        if   direction == "cw" : position = position_cw
        elif direction == "ccw": position = position_ccw
        else:                    position = position_ccw if self.rotor_angle.sector_half == 1 else position_cw

        # Update rotor position
        self.rotor_angle.rotate_to_sector_boundary(clockwise = (position == position_cw))

        # Physically move the rotor to the aligned position
        align_actions  = [('A', +1), ('B', +1), ('A', -1), ('B', -1)]
        phase, current = align_actions[position-1]
        self._energize_phase(phase=phase, I=current)

        # Wait for the motor to settle down in the aligned position.
        time.sleep(StepperMotor.MIN_DELAY)

    def spin_rotor(self, num_revolutions, rpm, direction, num_microsteps=1):
        """
        Spin the rotor for a specified number of revolutions at a given speed.

        The rotor might not travel exactly 'num_revolutions * 360°' because:
        1. Before starting to spin the rotor, the rotor is aligned (in the 
           chosen direction of rotation). Only after aligning it we do start 
           counting revolutions.
        2. This method spins the rotor for a whole number of full steps (even 
           when micro-stepping, that only makes the movement smoother), so a 
           non-integer 'num_revolutions' might get rounded to a multiple of 
           full steps not exactly equal to the intended angle.
        
        The rotor might spin slower than requested via the 'rpm' argument, especially 
        when increasing the number of micro-steps. This is due to the fact that we 
        impose a minimum delay between successive micro-steps, to allow the current to 
        settle. With a StepperMotor.MIN_DELAY of 10ms the max rpm is 30.
        
        Parameters
        ----------
        num_revolutions : float
            Number of complete revolutions to spin the motor (must be positive or zero)
            Can be fractional (e.g., 0.5 for half revolution) or 'inf' to spin forever.
        rpm : float
            Speed of rotation in revolutions per minute (actual rpm could be lower)
        direction : str
            Direction of rotation - either "cw" (clockwise) or "ccw" (counter-clockwise).
        num_microsteps: int
            Into how many smaller steps to break down a full step rotation (a power of 2).
            Higher count results in smoother movement.
        """
        assert num_revolutions >= 0
        assert rpm > 0
        assert direction in ("cw", "ccw")

        # Convert "revolutions" into "full steps" 
        num_fullsteps = round(num_revolutions * RotorAngle.FULL_STEPS_PER_REV)

        # Convert 'rpm' into how long it takes to turn one full step (in seconds)
        time_per_step = 60 / (RotorAngle.FULL_STEPS_PER_REV * rpm)

        # start by aligning the rotor...
        self.align_rotor(direction)

        # ... and continue by turning it one full step at a time
        self._rotate_full_steps(num_fullsteps, time_per_step, num_microsteps)

    def turn_rotor(self, target_angle, direction, num_microsteps=1):
        """
        Positions the rotor to an arbitrary angle.

        Angles are measured relative to the positive vertical axis; a positive 
        angle corresponds to a clockwise rotation. We can specify in which 
        direction to turn the rotor towards the target position: clockwise / 
        counter-clockwise, or whichever results in the shortest path.

        Note that the rotor cannot be positioned *exactly* at the requested 
        position for two reasons:
         + the hardware and electronics have limited precission
         + angles are represented internally with fixed limied precission
        In practice, the second reason should have no impact, as we should
        choose the resolution used when encoding angles as fine as necessary.
        
        Parameters
        ----------
        angle:              - target rotor position as angle relative to vertical
                              measured in degrees (clockwise is positive direction)
        direction:          - in which direction to turn the rotor ("closest", "cw", "ccw")
        num_microsteps: int - into how many smaller steps to break down a full step rotation (a power of 2).        
        """
        assert direction in ("cw", "ccw", "closest")

        # We cannot position the rotor to an arbitrary angle; we
        # quantify the input using fixed precission arithmetic.
        target_position = RotorAngle.from_degrees(target_angle)

        # Special case: target and current angle are the same
        if target_position == self.rotor_angle:
            return

        # Calculate current and target sectors
        current_sector = self.rotor_angle.sector
        target_sector  = target_position.sector
        
        # Special case: rotor is already in the target sector
        if current_sector == target_sector:
            self._rotate_in_sector(target_position.sector_position_in_ticks)
            time.sleep(StepperMotor.MIN_DELAY)
            return

        # Determine direction of rotation if "closest" is specified
        if direction == "closest":
            trgt_degrees = target_position.to_degrees
            curr_degrees = self.rotor_angle.to_degrees 
            angle_cw     = (trgt_degrees - curr_degrees) % 360
            angle_ccw    = (curr_degrees - trgt_degrees) % 360
            direction    = "cw" if angle_cw <= angle_ccw else "ccw"

        # Start by aligning the rotor.
        self.align_rotor(direction=direction)

        # Next rotate a whole number of full steps
        if direction == "cw": num_full_steps =   (target_sector  - current_sector) % RotorAngle.SECTOR_COUNT
        else:                 num_full_steps = -((current_sector - target_sector ) % RotorAngle.SECTOR_COUNT)
        time_per_step = StepperMotor.MIN_DELAY * num_microsteps
        self._rotate_full_steps(num_full_steps, time_per_step, num_microsteps)

        # Finaly, position rotor within target sector
        self._rotate_in_sector(target_position.sector_position_in_ticks)

        # Wait for the motor to settle down.
        time.sleep(StepperMotor.MIN_DELAY)

    def _rotate_full_steps(self, num_fullsteps, time_per_step, num_microsteps):
        """
        Rotate an aligned rotor a given number of full steps ('inf' to rotate forever).

        This method can only be used when the rotor is aligned.
        Note that it leaves the rotor also in an aligned state.

        Completing each full step might take longer than the duration requested 
        via the argument, especially when increasing the number of micro-steps. 
        This is due to the fact that we impose a minimum delay between successive 
        micro-steps, to allow the current to settle (see StepperMotor.MIN_DELAY).

        Parameters
        ----------
        num_fullsteps : int or inf - by how many full steps to rotate the rotor (positive for clockwise)
        time_per_step : float      - how long it takes to rotate by one sector, in seconds
        num_microsteps: int        - how many micro-steps to take to rotate one sector (power of 2)        
        """
        assert self.is_aligned                                         # this method may only be used with an aligned rotor
        assert isinstance(num_fullsteps, int) or isinf(num_fullsteps)  # cannot rotate a fractional number of full steps

        # Trivial case
        if num_fullsteps == 0:
            return

        # Direction of rotation
        direction     = "cw" if num_fullsteps > 0 else "ccw"
        num_fullsteps = abs(num_fullsteps)

        # Rotate one full step at a time using '_rotate_sector',
        # which is responsible for micro-stepping, if any.
        i = 0
        while i < num_fullsteps or isinf(num_fullsteps):
            i += 1
            self._rotate_full_step(direction, time_per_step, num_microsteps)            

    def _rotate_full_step(self, direction, duration, num_microsteps):
        """
        Rotate an aligned rotor by one full step.

        This method can only be used when the rotor is aligned.
        The rotation may be performed in one step (full-stepping) or broken down
        into smaller steps (micro-stepping). The number of micro-steps must be a
        power of 2, no larger than MAX_MICROSTEPS.

        Completing the full step might take longer than the duration requested 
        via the argument, especially when increasing the number of micro-steps. 
        This is due to the fact that we impose a minimum delay between successive 
        micro-steps, to allow the current to settle (see StepperMotor.MIN_DELAY)
        
        Parameters
        ----------
        direction     : str   - direction of rotation ("cw" or "ccw")
        duration      : float - how long it takes to complete the full step (in seconds)
        num_microsteps: int   - how many micro-steps to take to rotate one full step (power of 2)
        """
        # The number of micro-steps must be a power of two and cannot exceed 'MAX_MICROSTEPS'.
        assert RotorAngle.is_power_of_2(num_microsteps)
        assert num_microsteps <= StepperMotor.MAX_MICROSTEPS
        assert direction in ('cw', 'ccw')
        assert self.is_aligned   # this method may only be used with an aligned rotor

        # Get the current sector and calculate which quarter of the electric cycle we're in.
        # The electric cycle has a period of 4 sectors:
        #   Sectors 1, 5,  9, 13... start at quarter 1 (Phase A=+1, Phase B= 0)
        #   Sectors 2, 6, 10, 14... start at quarter 2 (Phase A= 0, Phase B=+1)
        #   Sectors 3, 7, 11, 15... start at quarter 3 (Phase A=-1, Phase B= 0)
        #   Sectors 4, 8, 12, 16... start at quarter 4 (Phase A= 0, Phase B=-1)
        current_sector   = self.rotor_angle.sector
        starting_quarter = ((current_sector - 1) % 4) + 1  # 1-4

        # Calculate the offset in the 'electric_cycle' array due to
        # which quarter of the electric cycle we are currently in.
        # The array has 4 * MAX_MICROSTEPS entries for one complete
        # electrical cycle => each quarter has MAX_MICROSTEPS entries.
        offset = (starting_quarter - 1) * StepperMotor.MAX_MICROSTEPS

        # Calculate how many entries to skip in the 'electric_cycle'
        # array when using fewer microsteps than MAX_MICROSTEPS.
        stride = StepperMotor.MAX_MICROSTEPS // num_microsteps

        # how long to wait in between steps for current to settle down.
        downtime = max(duration/num_microsteps, StepperMotor.MAX_MICROSTEPS)

        # Rotate one microstep at a time.
        for microstep in range(num_microsteps):

            # Calculate the index into the 'electric_cycle' array
            if direction == 'cw':
                cycle_index = (offset + (microstep + 1) * stride) % len(self.electric_cycle)
            else:
                cycle_index = (offset - (microstep + 1) * stride) % len(self.electric_cycle)

            # Energize stator phases
            IA, IB = self.electric_cycle[cycle_index]
            self._energize_phase(phase='A', I=IA)
            self._energize_phase(phase='B', I=IB)

            # Wait before next micro-step
            time.sleep(downtime)

        # Update the rotor angle to reflect the new position
        self.rotor_angle.move_one_sector(clockwise=(direction == 'cw'))

    def _rotate_in_sector(self, target_ticks):
        """
        Position the rotor inside a sector.

        This method positions the rotor to a specific location within the current 
        sector, given in 'ticks'. The rotor must already be in the target sector 
        before calling this method. If not, use '_rotate_full_steps(...)' first, 
        to rotate the rotor to the correct sector.

        Parameters
        ----------
        target_ticks: int - target position within the sector [0, RotorAngle.SECTOR_TICKS)
        """
        assert isinstance(target_ticks, int)  
        assert 0 <= target_ticks < RotorAngle.SECTOR_TICKS

        # Trivial case, the rotor is already in the target position
        if target_ticks == self.rotor_angle.sector_position_in_ticks:
            return

        # Find the aligned rotor position that corresponds to the
        # ccw border of the sector in which the rotor is right now.
        position_ccw = (self.rotor_angle.sector - 1) % 4 + 1                         
        
        # Find the electric angle that corresponds to the ccw aligned position
        electric_angle_aligned = [0, pi/2, pi, 3*pi/2][position_ccw-1]

        # Calculate by how much to change the electric angle that produces the
        # ccw aligned position, in order to bring the rotor to the target angle.
        # Remember that turning the rotor by one sector (1.8°) is accomplished 
        # by rotating the electric angle by pi/2.
        incremental_electric_angle = 0
        if target_ticks > 0:    
            incremental_electric_angle = (target_ticks * pi) / (2 * RotorAngle.SECTOR_TICKS)
        electric_angle = electric_angle_aligned + incremental_electric_angle

        # Physically move the rotor to the new position
        self._energize_phase(phase='A', I=cos(electric_angle))
        self._energize_phase(phase='B', I=sin(electric_angle))

        # Update rotor state
        self.rotor_angle = RotorAngle(self.rotor_angle.sector, target_ticks)

    def _energize_phase(self, phase, I):
        """
        Set the current through a given motor phase.
        
        Parameters:
            phase: 'A' or 'B'
            I:     Current intensity as ratio of maximum intensity (-1.0 to +1.0)
        """
        assert phase in ('A', 'B')
        assert abs(I) <= 1.0
 
        # Select the pins that control the current through the given phase. 
        in1 = self.ain1 if phase == 'A' else self.bin1
        in2 = self.ain2 if phase == 'A' else self.bin2
        pwm = self.pwma if phase == 'A' else self.pwmb
                
        # Set current direction based on sign
        in1.value(1 if I >= 0 else 0)
        in2.value(0 if I >= 0 else 1)
         
        # Set PWM duty cycle (0-65535 for 16-bit)
        # Use absolute value since direction is handled by IN pins
        duty = int(abs(I) * 65535)
        pwm.duty_u16(duty)
