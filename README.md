# vix-driver

Python driver for **Parker ViX250IM / ViX500IM** microstepper indexer drives
over RS232. It uses the drives' EASI-V ASCII command set.

Written from the *ViX250IM & ViX500IM Microstepper RS232 Indexer Drive User
Guide* (Parker 1600.324.01b, Feb 2004, firmware 2.0 onwards). The guide is not
included here; Parker publishes it at
<https://www.parkermotion.com/manuals/Digiplan/ViX-IM_UG_2-04.pdf>.

## Install

```bash
pip install -e .
```

The only runtime dependency is `pyserial`.

## Quick start

```python
from vix_driver import ViXBus, Mode

with ViXBus("/dev/ttyUSB0") as bus:            # 9600 8N1 by default
    drive = bus.drive(1)                        # axis address 1..255

    drive.energize()                            # ON, then checks status and faults
    drive.set_motion(velocity=5, acceleration=50)

    drive.move_relative(4000)                   # steps; blocks until done
    drive.move_absolute(0, velocity=10)
    drive.position_target()                     # PT (commanded position)

    drive.jog(2, positive=False)                # continuous mode
    drive.stop()                                # controlled stop (S)
    drive.wait_for_move()

    drive.configure_homing(velocity=-5, acceleration=50, edge_positive=False)
    drive.go_home()

    drive.inputs()                              # (stop, reg, home, lim-, lim+)
    drive.set_output(1, True)
```

## Several drives on one bus

```python
from vix_driver import ViXBus, CommsMode

with ViXBus("/dev/ttyUSB0") as bus:
    bus.discover()                          # {1: '2.1b', 2: '2.1b', 3: '2.1b'}
    bus.set_comms_mode_all(CommsMode.QUIET_ECHO)   # recommended, see below

    x, y = bus.drive(1), bus.drive(2)       # individual drives
    axes = bus.group(1, 2, 3)               # or bus.group() for all discovered

    axes.energize()
    axes.move_relative({1: 4000, 2: -2000, 3: 8000}, velocity=5, acceleration=50)
    axes.move_absolute({2: 0})              # axes not listed stay put
    axes.positions()                        # {1: 4000, 2: 0, 3: 8000}
    axes.drive_faults()                     # {} when all healthy

    bus.stop_all()                          # 0S: controlled stop, every drive
    bus.kill_all()                          # 0K: kill every drive
```

**How replies are matched.** Replies carry no address. The bus therefore runs
one transaction at a time (safe across threads) and ties each reply to the
command that caused it:

- **RS232 daisy chain (default, `echo=True`).** Every command travels round the
  ring and comes back to the PC. The bus waits for that echo before accepting a
  reply. Anything that arrives earlier is discarded, such as a late reply to an
  earlier read or another drive's message. A missing echo is reported as a
  broken chain. Keep echo on for every drive in the chain (`EX` = 2 or 3).
- **RS485 multidrop (`ViXBus(port, echo=False)`).** The drives must run with
  echo off (`EX` = 0 or 1), so this matching isn't available. After a timeout
  the bus drains the line briefly before the next command.

**Recommended setup:** `bus.set_comms_mode_all(CommsMode.QUIET_ECHO)` (`EX=2`).
Drives then only talk when addressed, so a program running on one drive can't
inject text into another drive's reply. Add `bus.group().save()` to make it
permanent.

**Groups.** `DriveGroup` commands take one value for all axes or a
`{address: value}` dict.
- Parameters are set on every axis first, then the `G` commands go out back to
  back. The axes start a few milliseconds apart.
- `group.go(simultaneous=True)` sends one global `0G`. It is only allowed when
  the group contains every drive found by `discover()`, because `0G` starts
  every drive on the bus.
- `wait_for_move()` watches all axes. If one faults, hits a limit or loses
  power, the others get a controlled stop and the error is raised, with
  `.address` naming the drive. If a `G` is rejected, axes already started are
  stopped.

Drives ignore reads on address 0, so the driver refuses them.

## API overview (`ViXDrive`)

| Area | Methods |
|---|---|
| Motion | `move_absolute()`, `move_relative()`, `jog()`, `go()`, `stop()`, `kill()`, `pause()`, `resume()`, `wait_for_move()` |
| Parameters | `set_motion(velocity, acceleration, deceleration, distance)`, `velocity()`, `acceleration()`, `deceleration()`, `distance()`, `set_mode()`, `get_mode()`, `set_direction()` |
| Position | `position_target()`, `position_actual()`, `position_error()`, `position_incremental()`, `zero_position()` |
| Status | `status()`, `drive_faults()`, `user_faults()`, `is_moving()`, `in_position()`, `is_busy()`, `is_energized()`, `check_faults()`, `status_report()`, `revision()` |
| Power | `energize()`, `de_energize()` |
| Homing / limits | `configure_homing()`, `homing_config()`, `go_home()`, `configure_limits()`, `limits_config()` |
| I/O | `inputs()`, `set_outputs()`, `set_output()`, `analogue_input()` |
| Profiles | `define_profile()`, `use_profile()`, `current_profile()` |
| Config | `set_motor()`, `configure_stall_detect()`, `configure_scale()`, `set_command_queuing()`, `set_comms_mode()`, `read()`/`write()` for any system variable |
| Programs | `run_label()`, `arm()`, `list_program()`, `clear_program()` |
| Persistence | `save()`, `reset()`, `factory_reset()` |
| Raw | `query(body)` and `command(body)` for anything not wrapped |

`ViXBus`: `drive()`, `drives()`, `group()`, `broadcast()`, `discover()`, `stop_all()`,
`kill_all()`, `set_comms_mode_all()`.
`DriveGroup`: `energize()`, `de_energize()`, `set_motion()`, `move_relative()`,
`move_absolute()`, `go()`, `stop()`, `kill()`, `wait_for_move()`, `positions()`,
`status()`, `drive_faults()`, `check_faults()`, `is_moving()`, `write()`, `save()`.

Units are motor steps, rev/s and rev/s² unless SCALE or LOADENC is enabled.

### Errors

- `ViXCommandError`: the drive replied `*E`. The driver then reads the UF word,
  so `exc.user_faults` says why (e.g. "Drive de-energised"). Reading UF clears it.
- `DriveFaultError`: a hardware fault (DF). `.terminal` is true if the fault
  needs a power cycle.
- `LimitError`: `wait_for_move()` saw a limit hit (ST bits 13/14).
- `ViXTimeoutError`: no reply, or the move didn't finish in time.

## Things to know about the drive

- **Command queuing (CQ).** With `CQ=1`, the factory default, commands sent
  during a move are buffered until the move ends. That includes position reads.
  Only `K`, `S`, `C`, `R(ST)`, `R(DF)`, `R(UF)` and `R(RB)` answer immediately.
  `wait_for_move()` therefore polls `R(ST)`. To read position mid-move, call
  `drive.set_command_queuing(False)`.
- **PT vs PA.** Without an encoder on X2, use `position_target()` (PT). PA and PE
  need an encoder.
- **Echo.** Create the bus with `echo=True` (the default) when the drives echo
  (`EX` = 2 or 3, the RS232 default), and `echo=False` when they don't (`EX` = 0
  or 1, the RS485 default). See "Several drives on one bus".
- **Enable input.** X4 pin 11, with polarity set by `ES`, must be active or `ON`
  fails with DF bit 21.
- **Mode P** makes the drive act as a step/direction base drive. `ON`/`OFF` are
  then prohibited.
- **Saving.** After `MOTOR`, `BR` (baud) or address changes, call `save()` then `reset()`.

## Bliss

`vix_driver.bliss_vix.ViX` is a Bliss motor controller: one controller per
serial line, one axis per drive. See [examples/bliss_vix.yml](examples/bliss_vix.yml).

```yaml
- plugin: generic
  class: ViX
  package: vix_driver.bliss_vix
  serial: {url: /dev/ttyS0, baudrate: 9600}
  axes:
    - {name: tth, address: 1, steps_per_rev: 4000, steps_per_unit: 4000,
       velocity: 1, acceleration: 10}
```

Install this package into the Bliss environment (`pip install -e .`).

**Supported:** moves, group moves (`move(tth, 30, th, 15)`), stop/Ctrl-C, jog,
`axis.on()`/`off()`, `home()`, `hw_limit()`, read-only X2 encoders, and
`raw_write_read("1R(PT)")`. Custom commands: `tth.vix_status()`,
`tth.vix_faults()` and `tth.vix_inputs()`.

**Behaviour to be aware of:**

- **Position.** The position is the drive's commanded position PT in motor
  steps. The drive can only reset it to 0, so `axis.dial = 0` works and other
  dial values are refused. User offsets (`axis.position = x`) work as usual.
- **Units.** Velocity and acceleration are converted from Bliss steps/s to the
  drive's rev/s using `steps_per_rev`. At start-up that value is checked against
  the motor resolution reported by the drive, and a mismatch aborts.
- **Command queuing.** The controller sets `CQ=0` on each drive so position and
  state can be polled during moves. It does this again after every
  `axis.on()`, which also re-sends velocity and acceleration.
- **Power cycles.** A power-cycled or faulted drive comes back de-energised
  (state OFF or FAULT) with its position reset. Run `axis.on()` and then
  `axis.sync_hard()`; Bliss refuses to move while dial and drive disagree.
- **Status.** `state()` reads the status word byte by byte: ST3 (moving,
  stall) and ST2 (energised, limits), ~90 ms each on a real drive instead of
  ~250 ms for the whole word, and ST1 only while homing. ST bit 17 (POSMAIN
  correction) stays set until the next move command, so it is not taken as
  "moving".
- **Caching.** Every query costs 70–90 ms on a real daisy chain, and Bliss
  reads position and state several times around each move. The controller
  reuses a stopped axis's position for up to 2 s (`POSITION_TTL`) and its
  state for 0.5 s (`STATE_TTL`), as long as nothing is sent to that drive; any
  command clears both, and the first read after a move goes to the drive.
  During a move the position is read again every 0.5 s (`MOVING_REFRESH`), so
  `umv` shows it on the way while short scan steps cost no extra query. A
  position changed by another program (e.g. spec) is noticed at most 2 s later.
- **Limits.** ST bits 13/14 ("limit seen during last move") are reported as
  LIMNEG/LIMPOS, so a move stopped by a limit raises `AxisOnLimitError`.
  `hw_limit(±1)` and `home(±1)` take the direction in motor (hardware) sense,
  as IcePAP does.
- **Encoders.** An encoder on a drive's X2 input is a Bliss encoder with an
  `address`, read as PA. Axes using it with `encoder: $enc` and
  `read_position: encoder` take their position from it, and Bliss corrects each
  move's target by the commanded-minus-encoder difference. In that mode Bliss's
  own discrepancy check compares the encoder with itself, so it never trips.
  The per-axis `max_encoder_error` (user units) restores spec's `slop`: a move
  is refused when encoder and commanded position differ by more. Lost steps
  accumulate in that difference until the axis is re-homed or re-zeroed
  (`axis.dial = 0` zeroes encoder and commanded position together). The
  drive's own position maintenance (POSMAIN) would correct the same error a
  second time, so start-up aborts if it is armed on an encoder-mode axis.
- **Load-mounted encoders (LOADENC).** On a drive with LOADENC on (ST bit 29),
  positions, velocity and acceleration are in load-encoder counts. The
  controller then converts velocity and acceleration with the system variable
  EM (counts per "rev") instead of the motor resolution. Such drives usually
  run position maintenance (POSMAIN) as well. Use the default
  `read_position: controller` and let the drive correct itself. An encoder on
  the axis still lets Bliss flag an encoder/drive mismatch at the end of a
  move.
- **Unsupported.** SCALE enabled on a drive (start-up aborts), and
  trajectories.
- **gevent.** The controller passes a `gevent.lock.RLock` to the bus. Bliss
  patches gevent without threads, so a `threading.RLock` would let greenlets
  interleave transactions (verified: timeouts with it, none with the gevent lock).

## XRD-3003TT configuration (from spec)

[bliss_config/xrd3003tt_vix.yml](bliss_config/xrd3003tt_vix.yml) holds the six
ViX axes, translated from the spec config. These were checked on the
instrument (2026-10-06, `examples/verify_addresses.py`):

- **Addresses.** Drive address = spec channel + 1. The drive positions matched
  spec's dial values.
- **Names and units.** spec's `phi` (MOT003) is an x translation in mm and is
  called `x` in Bliss. th, tth and chi are in degrees; x, y and z in mm.
- **Signs and limits.** `x` and `z` use `sign: -1`, spec's user/dial sign.
  The limits are spec's dial limits, except `x`: ±10 mm since 2026-10-09,
  after a wrong alignment correction drove the stage into the setup.
- **th/tth** run LOADENC with EM = 2000 and POSMAIN armed (`AR1 DB1`).
  Positions are in encoder counts, 2000 per degree, and the drive corrects
  itself.
- **Speeds.** Velocities and accelerations are spec's slew and ramp, except
  that `x` and `y` accelerate at 4 mm/s² (0.5 s to 2 mm/s; spec: 16 mm/s²).
- **Serial port.** For a local device (`url: /dev/...`) the controller opens
  the port with pyserial. Through bliss.comm's Serial, the echo of the longer
  commands such as `1W(CQ,0)` lost its first bytes on the real drives, which
  never happened with pyserial, also inside Bliss. Remote URLs (ser2net,
  rfc2217, tango) still use bliss.comm.
- **STATUS timing.** A real drive sends STATUS over about 3.3 s, with gaps of
  up to 0.3 s between lines. `status_report()` waits 1 s for each line.

User offsets are not in the config. After the first start, set each axis to
spec's user value (`axis.position = ...`).

## Tests

The tests run against a simulated drive and need no hardware or pyserial:

```bash
python -m unittest discover -s tests
```

`tests/bliss_e2e.py` runs Bliss axes against the simulator over a
pseudo-terminal (`tests/pty_sim.py`) through a Beacon server. Setup steps are
in its docstring; the config is [examples/bliss_vix_sim.yml](examples/bliss_vix_sim.yml).
`tests/bliss_e2e_xrd.py` runs the instrument config with drives 1/2 set up
like the real ones (LOADENC, EM, POSMAIN). `tests/bliss_e2e_encoder.py`
covers Bliss-side encoder correction (`tests/encoder_mode_config.yml`): reading from the encoder, correcting lost steps, the
`max_encoder_error` check, re-zeroing, homing, and the POSMAIN conflict.

## Use on hardware

Since October 2026 the driver runs the XRD-3003TT diffractometer: six ViX500IM
drives on one RS232 daisy chain at 9600 baud, two of them with load encoders
(LOADENC) and position maintenance (POSMAIN). Moves, status, the daisy-chain
echo, LOADENC velocities and the position caching are confirmed there; the
timing notes above come from those drives.

Not exercised on hardware yet, and taken from the manual's examples:

- homing and limit searches, and jogging;
- the `inputs()`, `LIMITS`, `HOME` and `PROFILE0` reply formats;
- whether `EX=2` still sends `*E` for rejected commands.
