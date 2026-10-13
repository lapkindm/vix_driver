"""Connect to a ViX IM drive, print its state, and make a short move.

Usage:  python examples/basic.py /dev/ttyUSB0

The motor WILL move one revolution forward and back (assuming 4000 steps/rev).
"""

import logging
import sys

from vix_driver import DriveFaultError, ViXBus

logging.basicConfig(level=logging.INFO)

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

with ViXBus(port, baudrate=9600) as bus:
    drive = bus.drive(1)

    print("Firmware:", drive.revision())
    print("Mode:", drive.get_mode().name)
    print("Target position (PT):", drive.position_target())
    print("Inputs 1-5:", drive.inputs())
    print("Status:", drive.status())
    print("Drive faults:", drive.drive_faults())

    try:
        drive.energize()
    except DriveFaultError as exc:
        sys.exit(f"Could not energise: {exc}")

    drive.move_relative(4000, velocity=2, acceleration=20, timeout=10)
    print("After move, PT =", drive.position_target())
    drive.move_relative(-4000, timeout=10)
    print("Back, PT =", drive.position_target())
