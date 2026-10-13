"""End-to-end test of the Bliss controller against the pty simulator.

Needs a Bliss environment, a running Beacon whose config contains the YAML
from examples/bliss_vix_sim.yml (axes ``tth`` at address 1, ``th`` at
address 2, ``serial.url`` pointing at the simulator link), and::

    python tests/pty_sim.py <link> 1 2

Run with BEACON_HOST set and this package on PYTHONPATH:

    BEACON_HOST=localhost:25000 PYTHONPATH=. python tests/bliss_e2e.py
"""

import logging
import os
import time

import gevent

from bliss.common.axis.error import AxisFaultError, AxisOnLimitError
from bliss.common.standard import move
from bliss.config.static import get_config

if os.environ.get("VIX_TRACE"):
    logging.basicConfig(filename=os.environ["VIX_TRACE"], level=logging.WARNING,
                        format="%(relativeCreated)8.0f %(message)s")
    logging.getLogger("vix_driver").setLevel(logging.DEBUG)

cfg = get_config()
tth = cfg.get("tth")
th = cfg.get("th")
ctrl = tth.controller
link = ctrl.config.config_dict["serial"]["url"]
results = []


def sim(cmd):
    """Poke simulator state (see tests/pty_sim.py)."""
    fd = os.open(link, os.O_WRONLY | os.O_NOCTTY)
    os.write(fd, f"SIM:{cmd}\r".encode())
    os.close(fd)
    time.sleep(0.05)


def check(name, condition, detail=""):
    results.append(condition)
    print(f"{'PASS' if condition else 'FAIL'}  {name}  {detail}")


for ax in (tth, th):
    ax.apply_config()
    ax.sync_hard()  # the simulator may have restarted (like a drive power cycle)
    ax.position = ax.dial  # no user offset
    ax.on()
d1, d2 = ctrl._drive(tth), ctrl._drive(th)

check("velocity converted to rev/s", d1.velocity() == 0.25, f"V={d1.velocity()}")
check("no spurious acceleration change", tth.acceleration == 50.0, f"{tth.acceleration}")
check("command queuing disabled", d1.read("CQ") == 0)

tth.move(12.5)
check("single move", tth.position == 12.5, f"{tth.position}")

move(tth, 30, th, -5)
check("group move", (tth.position, th.position) == (30, -5), f"{tth.position}, {th.position}")

tth.rmove(-10)
check("relative move", tth.position == 20, f"{tth.position}")

tth.dial = 0  # Bliss keeps the user position and moves the offset
check("dial reset to 0", tth.dial == 0 and d1.position_target() == 0 and tth.position == 20,
      f"dial={tth.dial} position={tth.position}")

try:
    tth.dial = 5
    check("non-zero dial refused", False)
except ValueError:
    check("non-zero dial refused", True)
tth.position = tth.dial  # drop the offset again

# Concurrent greenlets polling two drives on one bus must not mix replies.
move(tth, 11, th, 22)
seen = []


def poll(ax, expected):
    for _ in range(25):
        value = ctrl.read_position(ax) / ax.steps_per_unit  # dial position
        if value != expected:
            seen.append((ax.name, value))
        gevent.sleep(0)


gevent.joinall([gevent.spawn(poll, tth, 11), gevent.spawn(poll, th, 22)], raise_error=True)
check("concurrent greenlets keep replies apart", not seen, f"{seen[:3]}")

# Jog changes V; the next move must restore the configured velocity.
th.jog(5)
gevent.sleep(0.2)
th.stop()
check("jog stopped", "READY" in th.state, str(th.state))
th.move(0)
check("velocity restored after jog", d2.velocity() == 0.25, f"V={d2.velocity()}")

# Hardware limit hit during a move.
sim("2.limit_on_next_move=14")
try:
    th.move(3)
    check("limit hit raises AxisOnLimitError", False)
except AxisOnLimitError as exc:
    check("limit hit raises AxisOnLimitError", True, str(exc))
th.move(1)
check("can move again after limit", "READY" in th.state and "LIMPOS" not in th.state, str(th.state))

# Stop a running move.
sim("1.MOVE_TIME=2.0")
mv = gevent.spawn(tth.move, 40)
gevent.sleep(0.3)
check("state MOVING during move", "MOVING" in tth.state, str(tth.state))
tth.stop()
mv.join()
check("stop ends the move", "READY" in tth.state, str(tth.state))

# Drive fault during a move.
mv = gevent.spawn(tth.move, 45)
gevent.sleep(0.3)
sim("1.df=[1, 17]")
sim("1.energised=False")
mv.join()
check("fault during move raises AxisFaultError", isinstance(mv.exception, AxisFaultError),
      repr(mv.exception))
sim("1.df=[]")
sim("1.MOVE_TIME=0.15")
tth.on()
check("on() recovers after fault", "READY" in tth.state, str(tth.state))

tth.move(7)
tth.home()
check("homing zeroes position", tth.position == 0, f"{tth.position}")

check("custom command vix_status", "Motor energised" in tth.vix_status())
check("raw_write_read", ctrl.raw_write_read("1R(PT)") == "*0")

print(f"\n{sum(results)}/{len(results)} checks passed")
raise SystemExit(0 if all(results) else 1)
