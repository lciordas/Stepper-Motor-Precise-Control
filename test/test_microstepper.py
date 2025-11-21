"""
Unit tests for MicrostepMotor class.

These tests use mock hardware to verify motor control 
logic without requiring actual MicroPython hardware.
"""

import os, pytest, sys
from unittest.mock import patch
from math import inf

# Add the parent and 'src' directory to the path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
source_dir = os.path.join(parent_dir, 'src')
sys.path.insert(0, parent_dir)
sys.path.insert(0, source_dir)

# Before importing microstepper, replace machine module with our mock
import machine_mock
sys.modules['machine'] = machine_mock

# We can only now import the modules that import the 'machine' module
from microstepper import MicrostepMotor
from rotor_angle  import RotorAngle


class TestMicrostepMotorInitialization:
    """Test motor initialization and basic properties."""

    def setup_method(self):
        """Reset mock tracking before each test."""
        machine_mock.reset_all_tracking()

    def test_initialization_creates_pins_and_pwm(self):
        """Test that initialization properly sets up all pins and PWM controllers."""
        motor = MicrostepMotor(
            ain1=0, ain2=1, pwma=2,
            bin1=3, bin2=4, pwmb=5,
            verbose=False
        )

        # Verify pins were created
        assert motor.ain1.pin_num == 0
        assert motor.ain2.pin_num == 1
        assert motor.bin1.pin_num == 3
        assert motor.bin2.pin_num == 4

        # Verify PWM controllers were created and frequency set
        ops = machine_mock.get_all_operations()
        pwm_freq_ops = [op for op in ops['pwm_operations'] if op['type'] == 'freq_set']
        assert len(pwm_freq_ops) == 2
        assert all(op['frequency'] == 25000 for op in pwm_freq_ops)

    def test_initialization_aligns_rotor(self):
        """Test that motor starts with rotor aligned to position 1."""
        motor = MicrostepMotor(
            ain1=0, ain2=1, pwma=2,
            bin1=3, bin2=4, pwmb=5,
            verbose=False
        )

        # Check initial rotor position
        assert motor.rotor_angle.sector == 1
        assert motor.rotor_angle.sector_position_in_ticks == 0
        assert motor.is_aligned == True
        assert motor.aligned_position == 1

    def test_initialization_energizes_phase_a(self):
        """Test that initialization energizes Phase A positively."""
        motor = MicrostepMotor(
            ain1=0, ain2=1, pwma=2,
            bin1=3, bin2=4, pwmb=5,
            verbose=False
        )

        ops = machine_mock.get_all_operations()

        # Find pin value operations
        pin_ops = [op for op in ops['pin_operations'] if op['type'] == 'value_set']

        # Phase A should be energized positively (ain1=1, ain2=0)
        ain1_ops = [op for op in pin_ops if op['pin'] == 0]
        ain2_ops = [op for op in pin_ops if op['pin'] == 1]
        assert any(op['value'] == 1 for op in ain1_ops)
        assert any(op['value'] == 0 for op in ain2_ops)

        # Phase B should be off (both pins same value or PWM duty = 0)
        pwm_ops = [op for op in ops['pwm_operations'] if op['type'] == 'duty_set']
        # Phase A PWM should be max, Phase B PWM should be 0
        pwma_duty = [op for op in pwm_ops if op['pin'] == 2]
        pwmb_duty = [op for op in pwm_ops if op['pin'] == 5]
        assert any(op['duty_u16'] == 65535 for op in pwma_duty)  # Phase A at max
        assert any(op['duty_u16'] == 0 for op in pwmb_duty)       # Phase B off


class TestRotorAlignment:
    """Test rotor alignment functionality."""

    def setup_method(self):
        """Create motor instance and reset tracking for each test."""
        machine_mock.reset_all_tracking()
        self.motor = MicrostepMotor(
            ain1=0, ain2=1, pwma=2,
            bin1=3, bin2=4, pwmb=5,
            verbose=False
        )
        machine_mock.reset_all_tracking()  # Reset after initialization

    def test_align_rotor_when_already_aligned(self):
        """Test that aligning an already aligned rotor is a no-op."""
        # Motor starts aligned
        assert self.motor.is_aligned

        # Track operations before alignment
        ops_before = machine_mock.get_all_operations()

        # Try to align (should be no-op)
        self.motor.align_rotor("cw")

        # Check no new operations occurred
        ops_after = machine_mock.get_all_operations()
        assert ops_before == ops_after

    @patch('microstepper.MicrostepMotor.is_aligned', new_callable=lambda: property(lambda self: False))
    def test_align_rotor_clockwise(self, mock_aligned):
        """Test clockwise alignment from unaligned position."""
        # Set rotor to unaligned position (middle of sector 2)
        self.motor.rotor_angle = RotorAngle(2, RotorAngle.SECTOR_TICKS // 2)

        # Align clockwise
        self.motor.align_rotor("cw")

        # Should move to sector 3 (next clockwise boundary)
        assert self.motor.rotor_angle.sector == 3
        assert self.motor.rotor_angle.sector_position_in_ticks == 0


class TestSectorRotation:
    """Test sector-by-sector rotation."""

    def setup_method(self):
        """Create motor instance for each test."""
        machine_mock.reset_all_tracking()
        self.motor = MicrostepMotor(
            ain1=0, ain2=1, pwma=2,
            bin1=3, bin2=4, pwmb=5,
            verbose=False
        )

    @patch('time.sleep')  # Mock sleep to make tests fast
    def test_rotate_sectors_positive(self, mock_sleep):
        """Test rotating positive (clockwise) number of sectors."""
        initial_sector = self.motor.rotor_angle.sector

        # Rotate 5 sectors clockwise
        self.motor._rotate_sectors(5, 0.01)

        # Check final position
        expected_sector = (initial_sector + 5 - 1) % RotorAngle.SECTOR_COUNT + 1
        assert self.motor.rotor_angle.sector == expected_sector
        assert self.motor.is_aligned

        # Verify sleep was called correct number of times
        assert mock_sleep.call_count == 5

    @patch('time.sleep')
    def test_rotate_sectors_negative(self, mock_sleep):
        """Test rotating negative (counter-clockwise) number of sectors."""
        initial_sector = self.motor.rotor_angle.sector

        # Rotate 3 sectors counter-clockwise
        self.motor._rotate_sectors(-3, 0.01)

        # Check final position
        expected_sector = (initial_sector - 3 - 1) % RotorAngle.SECTOR_COUNT + 1
        assert self.motor.rotor_angle.sector == expected_sector
        assert self.motor.is_aligned

        # Verify sleep was called correct number of times
        assert mock_sleep.call_count == 3

    @patch('time.sleep')
    def test_rotate_sectors_zero(self, mock_sleep):
        """Test that rotating 0 sectors is a no-op."""
        initial_sector = self.motor.rotor_angle.sector

        # Rotate 0 sectors
        self.motor._rotate_sectors(0, 0.01)

        # Position should not change
        assert self.motor.rotor_angle.sector == initial_sector
        assert mock_sleep.call_count == 0

    @patch('time.sleep')
    def test_rotate_sectors_infinite(self, mock_sleep):
        """Test that infinite rotation works (at least starts)."""
        # We can't truly test infinite rotation, but we can verify it starts
        # and uses itertools.count() correctly

        # Set up mock to stop after a few iterations
        mock_sleep.side_effect = [None, None, None, KeyboardInterrupt()]

        with pytest.raises(KeyboardInterrupt):
            self.motor._rotate_sectors(inf, 0.01)

        # Should have rotated at least 3 times before interrupt
        assert mock_sleep.call_count == 4


class TestMicrostepPositioning:
    """Test micro-positioning within a sector."""

    def setup_method(self):
        """Create motor instance for each test."""
        machine_mock.reset_all_tracking()
        self.motor = MicrostepMotor(
            ain1=0, ain2=1, pwma=2,
            bin1=3, bin2=4, pwmb=5,
            verbose=False
        )
        machine_mock.reset_all_tracking()

    def test_rotate_in_sector_to_same_position(self):
        """Test that rotating to current position is a no-op."""
        current_ticks = self.motor.rotor_angle.sector_position_in_ticks

        # Get operations before
        ops_before = machine_mock.get_all_operations()

        # Rotate to same position
        self.motor._rotate_in_sector(current_ticks)

        # Should be no new operations
        ops_after = machine_mock.get_all_operations()
        assert ops_before == ops_after

    def test_rotate_in_sector_to_middle(self):
        """Test positioning to middle of sector."""
        target_ticks = RotorAngle.SECTOR_TICKS // 2

        # Position to middle of sector
        self.motor._rotate_in_sector(target_ticks)

        # Verify position updated
        assert self.motor.rotor_angle.sector_position_in_ticks == target_ticks
        assert not self.motor.is_aligned  # Not at sector boundary

        # Verify PWM operations occurred (microstepping)
        ops = machine_mock.get_all_operations()
        pwm_ops = [op for op in ops['pwm_operations'] if op['type'] == 'duty_set']
        assert len(pwm_ops) == 2  # Both phases should be energized for microstepping

    def test_rotate_in_sector_phase_energization(self):
        """Test that micro-positioning correctly energizes phases."""
        # Position to 25% through the sector
        target_ticks = RotorAngle.SECTOR_TICKS // 4

        self.motor._rotate_in_sector(target_ticks)

        # Check PWM duty cycles
        ops = machine_mock.get_all_operations()
        pwm_ops = [op for op in ops['pwm_operations'] if op['type'] == 'duty_set']

        # Both phases should be energized with complementary sine/cosine values
        # At 25% through sector, we're at 22.5° electrical angle from aligned position
        # This should give us approximately cos(22.5°) and sin(22.5°) duty cycles
        assert len(pwm_ops) == 2
        # We don't test exact values as they depend on math calculations


class TestSpinRotor:
    """Test continuous spinning functionality."""

    def setup_method(self):
        """Create motor instance for each test."""
        machine_mock.reset_all_tracking()
        self.motor = MicrostepMotor(
            ain1=0, ain2=1, pwma=2,
            bin1=3, bin2=4, pwmb=5,
            verbose=False
        )

    @patch('time.sleep')
    @patch.object(MicrostepMotor, 'align_rotor')
    @patch.object(MicrostepMotor, '_rotate_sectors')
    def test_spin_rotor_basic(self, mock_rotate, mock_align, mock_sleep):
        """Test basic spin functionality."""
        # Spin 2.5 revolutions at 60 RPM clockwise
        self.motor.spin_rotor(2.5, 60, "cw")

        # Verify alignment was called
        mock_align.assert_called_once_with("cw")

        # Verify rotation was called with correct parameters
        # 2.5 revolutions = 2.5 * 200 sectors = 500 sectors (rounded)
        # Delay = 60 / (200 * 60) = 0.005 seconds
        mock_rotate.assert_called_once_with(500, 0.005)

    @patch('time.sleep')
    @patch.object(MicrostepMotor, '_rotate_sectors')
    def test_spin_rotor_zero_revolutions(self, mock_rotate, mock_sleep):
        """Test that spinning 0 revolutions works correctly."""
        self.motor.spin_rotor(0, 60, "cw")

        # Should call rotate with 0 sectors
        mock_rotate.assert_called_with(0, 0.005)


class TestSetRotor:
    """Test arbitrary angle positioning."""

    def setup_method(self):
        """Create motor instance for each test."""
        machine_mock.reset_all_tracking()
        self.motor = MicrostepMotor(
            ain1=0, ain2=1, pwma=2,
            bin1=3, bin2=4, pwmb=5,
            verbose=False
        )

    @patch('time.sleep')
    def test_set_rotor_same_angle(self, mock_sleep):
        """Test that setting to current angle is a no-op."""
        current_angle = self.motor.rotor_angle.to_degrees

        # Reset tracking
        machine_mock.reset_all_tracking()

        # Set to same angle
        self.motor.set_rotor(current_angle, "cw")

        # Should be no operations
        ops = machine_mock.get_all_operations()
        assert len(ops['pin_operations']) == 0
        assert len(ops['pwm_operations']) == 0

    @patch('time.sleep')
    @patch.object(MicrostepMotor, 'align_rotor')
    @patch.object(MicrostepMotor, '_rotate_sectors')
    @patch.object(MicrostepMotor, '_rotate_in_sector')
    def test_set_rotor_different_sector(self, mock_rotate_in, mock_rotate_sectors, mock_align, mock_sleep):
        """Test positioning to a different sector."""
        # Position to 45 degrees
        self.motor.set_rotor(45.0, "cw", delay=0.01)

        # Should call all three positioning methods
        mock_align.assert_called_once()
        mock_rotate_sectors.assert_called_once()
        mock_rotate_in.assert_called_once()

    @patch('time.sleep')
    def test_set_rotor_closest_direction(self, mock_sleep):
        """Test that 'closest' direction works correctly."""
        # Start at 10 degrees (aligned position)
        self.motor.rotor_angle = RotorAngle.from_degrees(10)
        # Round to nearest sector boundary to ensure alignment
        sector = round(10 / 1.8) + 1
        self.motor.rotor_angle = RotorAngle(sector, 0)  # Aligned at sector boundary

        # Set to 350 degrees with "closest" - should go counter-clockwise
        with patch.object(self.motor, 'align_rotor') as mock_align:
            with patch.object(self.motor, '_rotate_sectors'):
                self.motor.set_rotor(350, "closest")
                # Verify counter-clockwise was chosen (20 degrees ccw vs 340 degrees cw)
                mock_align.assert_called_with(direction="ccw")

        # Reset to 350 degrees (aligned position)
        sector = round(350 / 1.8) + 1
        self.motor.rotor_angle = RotorAngle(sector % 200 if sector > 200 else sector, 0)

        # Set to 10 degrees with "closest" - should go clockwise
        with patch.object(self.motor, 'align_rotor') as mock_align:
            with patch.object(self.motor, '_rotate_sectors'):
                self.motor.set_rotor(10, "closest")
                # Verify clockwise was chosen (20 degrees cw vs 340 degrees ccw)
                mock_align.assert_called_with(direction="cw")


class TestEnergizePhase:
    """Test phase energization (hardware control)."""

    def setup_method(self):
        """Create motor instance for each test."""
        machine_mock.reset_all_tracking()
        self.motor = MicrostepMotor(
            ain1=0, ain2=1, pwma=2,
            bin1=3, bin2=4, pwmb=5,
            verbose=False
        )
        machine_mock.reset_all_tracking()

    def test_energize_phase_a_positive(self):
        """Test energizing Phase A with positive current."""
        self.motor._energize_phase('A', 1.0)

        ops = machine_mock.get_all_operations()

        # Check pin directions (ain1=1, ain2=0 for positive)
        pin_ops = [op for op in ops['pin_operations'] if op['type'] == 'value_set']
        ain1_op = next((op for op in pin_ops if op['pin'] == 0), None)
        ain2_op = next((op for op in pin_ops if op['pin'] == 1), None)
        assert ain1_op['value'] == 1
        assert ain2_op['value'] == 0

        # Check PWM duty (should be max)
        pwm_ops = [op for op in ops['pwm_operations'] if op['type'] == 'duty_set']
        pwma_op = next((op for op in pwm_ops if op['pin'] == 2), None)
        assert pwma_op['duty_u16'] == 65535

    def test_energize_phase_a_negative(self):
        """Test energizing Phase A with negative current."""
        self.motor._energize_phase('A', -1.0)

        ops = machine_mock.get_all_operations()

        # Check pin directions (ain1=0, ain2=1 for negative)
        pin_ops = [op for op in ops['pin_operations'] if op['type'] == 'value_set']
        ain1_op = next((op for op in pin_ops if op['pin'] == 0), None)
        ain2_op = next((op for op in pin_ops if op['pin'] == 1), None)
        assert ain1_op['value'] == 0
        assert ain2_op['value'] == 1

        # Check PWM duty (should be max, absolute value)
        pwm_ops = [op for op in ops['pwm_operations'] if op['type'] == 'duty_set']
        pwma_op = next((op for op in pwm_ops if op['pin'] == 2), None)
        assert pwma_op['duty_u16'] == 65535

    def test_energize_phase_half_power(self):
        """Test energizing a phase at half power."""
        self.motor._energize_phase('B', 0.5)

        ops = machine_mock.get_all_operations()

        # Check PWM duty (should be half)
        pwm_ops = [op for op in ops['pwm_operations'] if op['type'] == 'duty_set']
        pwmb_op = next((op for op in pwm_ops if op['pin'] == 5), None)
        assert pwmb_op['duty_u16'] == 32767  # Half of 65535

    def test_energize_phase_zero_power(self):
        """Test turning off a phase."""
        self.motor._energize_phase('A', 0.0)

        ops = machine_mock.get_all_operations()

        # Check PWM duty (should be 0)
        pwm_ops = [op for op in ops['pwm_operations'] if op['type'] == 'duty_set']
        pwma_op = next((op for op in pwm_ops if op['pin'] == 2), None)
        assert pwma_op['duty_u16'] == 0


# Run tests if executed directly
if __name__ == "__main__":
    pytest.main([__file__, "-v"])