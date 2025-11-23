"""
In this module we assume that we are working with a bipolar stepper motor with two phases.
Each phase has four windings around four poles, positioned around the stator as follows:
Phase A poles: A1 @ 0   , A2 @ 1/2π, A3 @ π   , A4 @ 3/2π 
Phase B poles: B1 @ 1/4π, B2 @ 3/4π, B3 @ 5/4π, B4 @ 7/4π

The windings around the poles are such that passing an electric current through the phase 
generates alternating magnetic polarities:
Positive current through Phase A:  A1 - Nord , A2 - South, A3 - Nord , A4 - South
Negative current through Phase A:  A1 - South, A2 - Nord , A3 - South, A4 - Nord
Positive current through Phase B:  B1 - Nord , B2 - South, B3 - Nord , B4 - South
Negative current through Phase B:  B1 - South, B2 - Nord , B3 - South, B4 - Nord

Electrical Angle:
-----------------
To drive the rotor in full-step increments, we cycle through four distinct electrical states:

  + Phase A at maximum positive current, Phase B at zero
  + Phase B at maximum positive current, Phase A at zero
  + Phase A at maximum negative current, Phase B at zero
  + Phase B at maximum negative current, Phase A at zero

We can express these four states compactly as the pairs: (1, 0), (0, 1), (-1, 0), (0, -1).

If we treat the first value as the y-coordinate and the second as the x-coordinate, these 
pairs describe a unit vector that starts vertical and rotates clockwise in 90° increments.

The "electrical angle" is defined as the clockwise angle between this rotating unit vector 
and the vertical. It is not a physical angle, but a convenient way to track which pattern 
of electrical currents is being applied to the two motor phases.

Magnetic Angle:
---------------
The currents flowing through the stator coils generate a magnetic field. As we advance the 
electrical angle, this magnetic field rotates as well. By convention, when the electrical 
angle is zero, we define the magnetic angle to be zero too.

Unlike the electrical angle, the magnetic angle is a physical angle: it describes the actual
rotation of the magnetic field configuration relative to a reference direction in the stator.

A 90° change in electrical angle results in only a 45° rotation of the magnetic field. This is 
because advancing the electrical angle by 90° energizes a different pair of stator poles that 
are physically positioned 45° apart. Consequently, the magnetic angle progresses at half the 
rate of the electrical angle.

Mechanical Angle:
-----------------
The mechanical angle is the physical angle through which the rotor itself turns. 

Micro-stepping:
---------------
Micro-stepping means turning the rotor in increments smaller than one full step (1.8°).
The idea is to move through the same electric cycle, but in smaller steps. For example,
rotating the electrical angle by only 45° would equally energize both phases and create
a magnetic field positioned in the middle between two poles, therefore rotated by 22.5°.

Micro-stepping means rotating the rotor in increments smaller than one full step (1.8°).
This can be achieved by advancing through the electrical cycle in smaller increments.
For example, if we rotate the electrical angle by only 45°, both phases become equally 
energized. This produces a magnetic field oriented halfway between the two adjacent 
stator poles—i.e., rotated by 22.5°. 

The problem
-----------
It turns out that implementing micro-stepping by tracing a sinusoidal electrical cycle only 
provides an approximate solution. This hasn’t been obvious so far, because the approximation 
happens to be exact in the special cases of full-step and half-step operation.

The real issue is this: we want to generate a magnetic field of constant magnitude, rotated 
by an arbitrary angle. A sinusoidal pattern of phase currents would achieve this if the stator 
poles were arranged 90° apart. But in a typical hybrid stepper, the poles are positioned 45° 
apart, not 90°. 

Because of the 45° spacing of the stator poles, a sinusoidal electrical cycle does not generate 
a perfectly uniform rotating magnetic field. Two problems arise:
+ The field strength varies as the electrical angle advances, instead of remaining constant.
+ The direction of the resulting magnetic field deviates slightly from the target angle.

Both issues disappear at full-step and half-step positions, where the geometry aligns cleanly 
with the sinusoidal pattern. But at intermediate micro-step positions, the mismatch caused by 
the 45° pole spacing leads to these unavoidable distortions.

The solution
------------
The solution is to decompose the desired magnetic field vector into components along the two 
stator-pole directions that bracket it—those two directions are always separated by 45°. 
"""

from math import cos, sin, sqrt, pi

def calculate_currents_geometric(cycle_position):
    """
    Calculate phase currents by decomposing the magnetic field along the directions of the stator poles that bracket it.

    See the module docstring for details.

    Parameters:
        cycle_position (float): Position within the electrical cycle, ranging from
                                0.0 (start) to 1.0 (full cycle).
                                
    Returns:
        tuple: A pair (IA, IB) where:
               - IA: Current intensity for Phase A (cos wave), range [-1.0, +1.0]
               - IB: Current intensity for Phase B (sin wave), range [-1.0, +1.0]
    """

    # Which quarter of the electric cycle are we in.
    # This determines in between which stator poles is the direction of the magnetic field. 
    quarter = int(cycle_position * 4) + 1

    # Consider two axes that form a 45° angle, and a unit vector whose direction lies between them.
    # One axis is positioned counter-clockwise, the other one clockwise relative to the unit vector.
    
    # If 'theta' is the angle between the unit vector and the ccw axis, then its decomposition along 
    # the two axes is:
    #   ccw component = cos(theta) - sin(theta)
    #   cw  component = sqrt(2) * sin(theta)
    #
    # This is the situation we face when decomposing the rotated magnetic field along the direction of 
    # the two stator poles that bracket it. This decomposition is useful because we control the intensity 
    # of the field along the direction that connects opposing poles. Setting the field magnitude to 1, 
    # its components are:

    position_in_quarter = (cycle_position * 4) % 1.0
    theta = position_in_quarter * pi / 4
    
    B_ccw = cos(theta) - sin(theta)
    B_cw  = sqrt(2) * sin(theta)

    # Below we translate magnetic field intensity to current intensity.
    # We are working on units where a full current (intensity 1) generates a full magnetic field.
    # Note the sign of the current: generating the same magnetic field along the A1 - A3 vs A2 - A4 
    # direction requires flipping the direction of the current through Phase A (as polarities of 
    # poles A1, A2, A3, A4 alternates).
    
    # ccw direction: line connecting poles A1 - A3
    # cw  direction: line connecting poles B1 - B3        
    if quarter == 1:
        Ia = +B_ccw
        Ib = +B_cw

    # ccw direction: line connecting poles B1 - B3            
    # cw  direction: line connecting poles A2 - A4
    elif quarter == 2:
        Ia = -B_cw 
        Ib = +B_ccw

    # ccw  direction: line connecting poles A2 - A4
    # c2 direction: line connecting poles B2 - B4            
    elif quarter == 3:
        Ia = -B_ccw
        Ib = -B_cw

    # left  direction: line connecting poles B2 - B4            
    # right direction: line connecting poles A1 - A3
    elif quarter == 4:
        Ia = +B_cw
        Ib = -B_ccw

    return Ia, Ib

def calculate_currents_sinusoidal(cycle_position):
    """
    Calculate phase currents using the sinusoidal approximation.

    This function provides a basic sinusoidal waveform for the two motor phases,
    with a 90-degree phase shift between them. This is the standard approach for
    smooth micro-stepping in stepper motors.

    See the module docstring for details.

    Parameters:
        cycle_position (float): Position within the electrical cycle, ranging from
                                0.0 (start) to 1.0 (full cycle).
                                
    Returns:
        tuple: A pair (IA, IB) where:
               - IA: Current intensity for Phase A (cos wave), range [-1.0, +1.0]
               - IB: Current intensity for Phase B (sin wave), range [-1.0, +1.0]
    """
    theta = 2*pi*cycle_position
    return cos(theta), sin(theta)


def calculate_electric_cycle(num_microsteps, current_calculator):
    """
    Generate the sequence of electrical configurations for micro-stepping control.

    This function calculates the current intensities for phases A and B throughout
    a complete electrical cycle, divided into micro-steps. The cycle consists of
    4 full steps, each subdivided into the specified number of micro-steps.

    At key points in the cycle (0%, 25%, 50%, 75%), exact values are enforced to
    avoid floating-point rounding errors:
      - n = 0:                Phase A at +1, Phase B at  0 (  0° position)
      - n = num_microsteps:   Phase A at  0, Phase B at +1 ( 90° position)
      - n = 2*num_microsteps: Phase A at -1, Phase B at  0 (180° position)
      - n = 3*num_microsteps: Phase A at  0, Phase B at -1 (270° position)

    Between these cardinal points, the provided current calculator function is used
    to determine the current intensities for smooth transitions.

    Parameters:
        num_microsteps (int): Number of micro-steps per full step (a power of 2)

        current_calculator (function): A function that calculates current intensities
                                       for a given position in the electrical cycle.
                                       Signature: current_calculator(cycle_position) -> (IA, IB)
                                       where:
                                        - cycle_position: float between 0.0 and 1.0 representing
                                          the position within one complete electrical cycle
                                        - Returns: tuple (IA, IB) with current intensities for
                                          Phase A and Phase B (values between -1.0 and 1.0)

    Returns:
        list: A list of tuples (IA, IB) where:
              - IA: Current intensity for Phase A (-1.0 to +1.0)
              - IB: Current intensity for Phase B (-1.0 to +1.0)
              The list contains 4*num_microsteps entries representing one
              complete electrical cycle.
    """

    # To move the rotor in full steps, we cycle the electric configuration through 4 stages.
    # When we break a full step into micro-steps, the number of stages increases proportionally
    # (we complete the same electric cycle, but in finer steps).
    num_stages = 4 * num_microsteps

    # For each stage store a tuple (IA, IB): the signed 
    # intensity of electric curent through each phase.
    electric_cycle = []
    
    for n in range(num_stages):
               
        # Regardless of how we compute the intermediate electrical states used to move the rotor
        # between full steps, the electrical configuration at certain key points in the cycle
        # is always known: at 0%, 25%, 50%, and 75% of the electrical period, one phase is fully
        # energized and the other phase is completely off. To avoid floating-point rounding errors, 
        # we explicitly hard-code the electrical configuration for these special points. 
        if n == 0:
            IA, IB = (+1, 0)
        elif n == num_microsteps:
            IA, IB = (0, +1)
        elif n == 2 * num_microsteps:
            IA, IB = (-1, 0)
        elif n == 3 * num_microsteps:
            IA, IB = (0, -1)

        # In between full steps.
        else:
            IA, IB = current_calculator(n/num_stages)
            
        electric_cycle.append((IA, IB))
    return electric_cycle
