"""Read-only check of the ViX drives before the first Bliss session.

Usage (with spec stopped, since only one program can hold the port):

    python examples/verify_addresses.py /dev/ttyS5

Sends only reads (R(RV), R(ST), R(DF), R(PT), STATUS). Nothing is written and
nothing moves. For every drive found it prints the motor resolution and the
commanded position, also converted to degrees/mm with the steps_per_unit of
the axis the Bliss config assigns to that address. Compare those numbers
with the DIAL positions spec's `wm th tth chi phi y z` (spec's phi = Bliss x) showed before you
quit spec: if they match, the address mapping in the config is right.

For the encoder axes (th, tth) it also reads the encoder (PA) and the
POSMAIN report. PA should be within a few counts of PT; if it is a
different multiple of PT, the encoder steps_per_unit in the config must
be scaled accordingly. POSMAIN must not be armed (AR1).
"""

import re
import sys

ENCODER_AXES = {1, 2}

from vix_driver import ViXBus, ViXError

# address -> (axis, steps_per_unit), as in bliss_config/xrd3003tt_vix.yml
CONFIG = {
    1: ("th", 2000),  # encoder
    2: ("tth", 2000),  # encoder
    3: ("y", 12500),
    4: ("x", 12500),  # spec "phi" (x translation, mm); sign -1: user = -dial
    5: ("z", 1250000),  # spec sign -1: user = -dial
    6: ("chi", 250000),
}

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyS5"

with ViXBus(port, baudrate=9600) as bus:
    found = bus.discover(range(1, 17))
    print(f"Drives answering on {port}: {sorted(found) or 'none'}\n")
    if not found:
        sys.exit("No reply: check the port, baud rate and that spec is not running.")

    for address in sorted(found):
        drive = bus.drive(address)
        axis, spu = CONFIG.get(address, ("?", None))
        try:
            report = "\n".join(drive.status_report())
            match = re.search(r"RESOLUTION\s*\.*\s*(\d+)", report)
            resolution = match.group(1) if match else "?"
            pt = drive.position_target()
            status = drive.status()
            faults = drive.drive_faults()
        except ViXError as exc:
            print(f"address {address}: error {exc}")
            continue
        dial = f"{pt / spu + 0.0:.4f}" if spu else "-"  # + 0.0 avoids "-0.0000"
        print(f"address {address}  -> {axis:4}  firmware {found[address]:6}  "
              f"resolution {resolution:>6} steps/rev  PT {pt:>10}  dial {dial}")
        print(f"            energised={status[9]}  scale={status[30]}  loadenc={status[29]}  "
              f"faults: {faults}")
        if address in ENCODER_AXES:
            try:
                pa = drive.position_actual()
                posmain = drive.query("POSMAIN")
            except ViXError as exc:
                print(f"            encoder: error {exc}")
            else:
                print(f"            encoder PA {pa}  (PA - PT = {pa - pt})  POSMAIN report: {posmain}")

    missing = sorted(set(CONFIG) - set(found))
    extra = sorted(set(found) - set(CONFIG))
    if missing:
        print(f"\nIn config but not answering: {missing}")
    if extra:
        print(f"\nAnswering but not in config: {extra}")
