from math import cos, sin, pi

def calculate_currents_sinusoidal(cycle_position):
    """
    Calculate phase currents using simple sinusoidal interpolation.

    This function provides a basic sinusoidal waveform for the two motor phases,
    with a 90-degree phase shift between them. This is the standard approach for
    smooth micro-stepping in stepper motors.

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
