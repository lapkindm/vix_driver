"""End-to-end test of the XRD-3003TT instrument config (bliss_config/xrd3003tt_vix.yml)
with drives 1/2 set up like the real th/tth drives: LOADENC on, EM = 2000,
POSMAIN armed (as read from the drives on 2026-10-06).

    python tests/pty_sim.py <link> 1 2 3 4 5 6
    BEACON_HOST=localhost:25000 PYTHONPATH=. python tests/bliss_e2e_xrd.py <link>

Use a fresh Beacon/Redis (hardware initialisation runs once).
"""
import os
import sys
import time

import gevent

from bliss.config.static import get_config

LINK = sys.argv[1]
res = []


def sim(cmd):
    fd = os.open(LINK, os.O_WRONLY | os.O_NOCTTY)
    os.write(fd, f"SIM:{cmd}\r".encode())
    os.close(fd)
    time.sleep(0.05)


def check(name, cond, detail=""):
    res.append(bool(cond))
    print(f"{'PASS' if cond else 'FAIL'}  {name}  {detail}")


for a in (1, 2):
    sim(f"{a}.loadenc=True")
    sim(f"{a}.posmain=True")
    sim(f"{a}.vars.EM=2000")
    sim(f"{a}.resolution=25000")
for a in (3, 4, 5, 6):
    sim(f"{a}.resolution=25000")

cfg = get_config()
th, tth, chi, x, z = (cfg.get(n) for n in ("th", "tth", "chi", "x", "z"))
for ax in (th, tth, chi, x, z):
    ax.on()
d1 = th.controller._axes_data["th"].drive
check("LOADENC drive accepted, POSMAIN armed allowed", th.state.READY, th.state)
check("th converts with EM (2000), not the motor resolution",
      th.controller._steps_per_rev(th) == 2000 and chi.controller._steps_per_rev(chi) == 25000)
check("th velocity 1 deg/s -> V 1.0 (EM counts/s), as spec", abs(d1.velocity() - 1.0) < 1e-9 and th.velocity == 1.0,
      (d1.velocity(), th.velocity))
check("th acceleration 8 -> AA 8.0, as spec", abs(d1.acceleration() - 8.0) < 1e-9, d1.acceleration())
d6 = chi.controller._axes_data["chi"].drive
check("chi 0.32 deg/s -> V 3.2 rps (motor steps), as spec", abs(d6.velocity() - 3.2) < 1e-9, d6.velocity())

th.dial = 0; th.position = 0
th.move(1.5)
check("th move in encoder counts", th.position == 1.5 and d1.position_target() == 3000 and th.encoder.read() == 1.5,
      (th.position, d1.position_target()))

st3 = th.controller.raw_write_read("1R(ST3)").strip()
check("POSMAIN bit 17 latched after a move, axis still READY (not MOVING)",
      st3.startswith("*1") and th.state.READY, (st3, th.state))
th.move(1.6)
check("next move works with bit 17 latched", th.position == 1.6, th.position)
th.move(1.5)

# position read caching: fewer PT queries per move, same result
drv = th.controller._axes_data["th"].drive
real_pt = drv.position_target
calls = []
drv.position_target = lambda: calls.append(1) or real_pt()
th.move(1.7)
n_move = len(calls)
drv.position_target = real_pt
check("move reads PT at most 3 times (was 5-6)", n_move <= 3 and th.position == 1.7, (n_move, th.position))
th.move(1.5)

# a long move (umv) shows the position on the way, refreshed every MOVING_REFRESH s
sim("1.MOVE_TIME=2.0")
seen = []
mv = gevent.spawn(th.move, 2.5)
while not mv.ready():
    seen.append(round(th.position, 3))
    gevent.sleep(0.1)
mv.get()
sim("1.MOVE_TIME=0.15")
between = sorted({p for p in seen if 1.5 < p < 2.5})
check("long move: intermediate positions shown", len(between) >= 2 and th.position == 2.5, (between, th.position))
th.move(1.5)

# an out-of-band move (e.g. spec) is still detected once the cache has expired
y = cfg.get("y"); y.on(); y.position
sim("3.vars.PT=25000")  # drive 3 now 2.0 mm, Bliss still believes its old value
time.sleep(th.controller.POSITION_TTL + 0.1)
try:
    y.move(y.position + 0.1)
    err = None
except Exception as e:
    err = e
check("external position change detected after the cache expiry", err is not None and "discrepancy" in str(err),
      str(err)[:90])
y.sync_hard()

sim("1.encoder_error=50")  # 50 counts = 0.025 deg > encoder tolerance 0.005
try:
    th.move(2)
    err = None
except Exception as e:
    err = e
check("Bliss flags an encoder/drive mismatch at the end of the move", err is not None and "final position" in str(err), repr(err)[:120])
sim("1.encoder_error=0")
th.sync_hard()
th.move(2)
check("moves again once they agree", th.position == 2)

sim("4.vars.PT=24150")  # as on the real drive 4 (spec phi): dial 1.932
x.sync_hard(); x.position = x.dial * x.sign  # offset 0
check("x (spec phi): sign -1 -> user = -dial, unit mm", x.position == -1.932 and x.dial == 1.932 and x.unit == "mm",
      (x.dial, x.position, x.unit))
x.move(-1.0)
check("x user move", abs(x.dial - 1.0) < 1e-6, x.dial)
sim("5.vars.PT=9677625"); z.sync_hard()
check("z dial from the drive, limits are dial limits", z.dial == 7.7421 and z.dial_limits == (-43.96007, 28.03993),
      (z.dial, z.dial_limits))

print(f"\n{sum(res)}/{len(res)} passed")
sys.exit(0 if all(res) else 1)
