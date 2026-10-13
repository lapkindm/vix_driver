"""Find every drive on the bus and make a coordinated move.

Usage:  python examples/multi_axis.py /dev/ttyUSB0

ALL discovered axes WILL move 1 rev forward and back (assuming 4000 steps/rev).
"""

import logging
import sys

from vix_driver import CommsMode, ViXBus, ViXError

logging.basicConfig(level=logging.INFO)

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

with ViXBus(port) as bus:
    found = bus.discover(range(1, 17))
    print("Drives found:", found)
    if not found:
        sys.exit("No drives answered - check wiring, baud rate and addresses")

    # Drives only speak when addressed; echo stays on for the RS232 daisy chain.
    bus.set_comms_mode_all(CommsMode.QUIET_ECHO)

    axes = bus.group()                       # every discovered drive
    print("Faults:", axes.drive_faults() or "none")

    try:
        axes.energize()
        axes.move_relative(4000, velocity=2, acceleration=20, timeout=10)
        print("Positions:", axes.positions())
        axes.move_relative(-4000, timeout=10)
        print("Positions:", axes.positions())
    except ViXError as exc:
        bus.stop_all()
        sys.exit(f"Stopped: {exc}")
