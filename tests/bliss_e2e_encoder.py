"""End-to-end test of encoder axes (th, tth) with tests/encoder_mode_config.yml.

Needs a Beacon whose config is that file with ``serial.url`` pointing at the
simulator link, and the simulator serving six drives::

    python tests/pty_sim.py <link> 1 2 3 4 5 6
    BEACON_HOST=localhost:25000 PYTHONPATH=. python tests/bliss_e2e_encoder.py <link>

Use a fresh Beacon/Redis: the POSMAIN check runs during the first hardware
initialisation of tth.
"""
import os
import sys
import time

from bliss.config.static import get_config

cfg = get_config()
LINK = sys.argv[1]
res = []


def sim(cmd):
    """Poke simulator state (see tests/pty_sim.py)."""
    fd = os.open(LINK, os.O_WRONLY | os.O_NOCTTY)
    os.write(fd, f"SIM:{cmd}\r".encode())
    os.close(fd)
    time.sleep(0.05)


def check(name, cond, detail=""):
    res.append(cond)
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {detail}")


# POSMAIN armed on drive 2 must stop tth from initialising
sim("2.posmain=True"); sim("1.encoder_error=0")
tth = cfg.get("tth")
try:
    tth.position
    check("POSMAIN armed + encoder mode refused", False)
except Exception as e:
    msg = str(e) + str(e.__cause__ or "")
    check("POSMAIN armed + encoder mode refused", "POSMAIN" in msg or "initialize" in msg, type(e).__name__)
sim("2.posmain=False")
tth.enable()  # Bliss disables an axis whose initialisation failed

th = cfg.get("th")
th.on(); th.sync_hard(); th.dial = 0; th.position = 0  # clean start: no offset
d = th.controller._drive(th)
th.move(1.5)
check("position read from encoder", th.position == 1.5 and d.position_actual() == 3000,
      f"pos={th.position} PA={d.position_actual()} PT={d.position_target()}")

sim("1.encoder_error=-4")          # motor lost 4 steps (< slop 10)
th.sync_hard()
check("lost steps visible in position", abs(th.position - (1.5 - 4/2000)) < 1e-9, f"{th.position}")
th.move(2.0)
check("next move corrects them", th.position == 2.0 and d.position_actual() == 4000,
      f"pos={th.position} PA={d.position_actual()} PT={d.position_target()}")

sim("1.encoder_error=-30")         # 30 lost steps > slop
th.sync_hard()
try:
    th.move(2.5)
    check("discrepancy > slop blocks the move", False)
except RuntimeError as e:
    check("discrepancy > slop blocks the move", "max_encoder_error" in str(e), str(e)[:110])

th.dial = 0                         # recovery: zero encoder and commanded position together
check("dial=0 zeroes PA and PT", (d.position_actual(), d.position_target()) == (0, 0) and th.dial == 0,
      f"PA={d.position_actual()} PT={d.position_target()}")
th.move(th.position + 0.5)
check("moves again after re-zero", d.position_actual() == 1000, f"PA={d.position_actual()}")

try:
    th.dial = 3
    check("non-zero encoder dial refused", False)
except ValueError:
    check("non-zero encoder dial refused", True)

sim("1.encoder_error=7")
th.home()
check("homing zeroes encoder", d.position_actual() == 0 and th.dial == 0, f"PA={d.position_actual()}")

tth.on(); tth.move(10)
check("tth (encoder) moves", tth.position == 10, f"{tth.position}")
chi = cfg.get("chi"); chi.on(); chi.move(0.1)
check("chi (no encoder) unaffected", chi.position == 0.1, f"{chi.position}")
print(f"{sum(res)}/{len(res)} passed")
raise SystemExit(0 if all(res) else 1)
