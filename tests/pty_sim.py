"""Serve the simulated ViX daisy chain on a pseudo-terminal.

Usage:  python tests/pty_sim.py LINK [ADDRESS ...]

Creates a pty, symlinks its device to LINK (e.g. /tmp/vix_sim) and answers
EASI-V commands on it until killed. Lets real serial clients (pyserial,
Bliss) talk to the simulator.

Test hook: a line ``SIM:<address>.<attribute>=<python literal>`` (e.g.
``SIM:2.limit_on_next_move=14``) sets simulator state instead of being
treated as a command.
"""

import ast
import os
import sys
import termios
import tty

sys.path.insert(0, os.path.dirname(__file__))
from fake_vix import FakeViX  # noqa: E402


def main():
    link = sys.argv[1]
    addresses = [int(a) for a in sys.argv[2:]] or [1]
    master, slave = os.openpty()
    tty.setraw(slave)
    attrs = termios.tcgetattr(slave)
    attrs[3] &= ~termios.ECHO
    termios.tcsetattr(slave, termios.TCSANOW, attrs)
    if os.path.lexists(link):
        os.remove(link)
    os.symlink(os.ttyname(slave), link)
    print(f"serving ViX addresses {addresses} on {link} -> {os.ttyname(slave)}", flush=True)

    fake = FakeViX(addresses=addresses)
    buf = b""
    while True:
        buf += os.read(master, 1024)
        while b"\r" in buf:
            line, buf = buf.split(b"\r", 1)
            if line.startswith(b"SIM:"):
                target, value = line[4:].decode().split("=", 1)
                address, attr = target.split(".", 1)
                unit = fake.unit(int(address))
                if attr.startswith("vars."):  # e.g. SIM:1.vars.EM=2000
                    unit.vars[attr[5:]] = ast.literal_eval(value)
                else:
                    setattr(unit, attr, ast.literal_eval(value))
                continue
            fake.write(line + b"\r")
            if fake.out:
                os.write(master, fake.out)
                fake.out = b""


if __name__ == "__main__":
    main()
