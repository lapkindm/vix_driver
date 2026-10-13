"""Bliss motor controller for Parker ViX250IM / ViX500IM drives.

One controller per serial line; each axis is one drive, selected by its
address. Example YAML::

    - class: ViX
      package: vix_driver.bliss_vix
      serial:
        url: /dev/ttyS0          # or ser2net://..., rfc2217://..., tango://...
        baudrate: 9600
      echo: true                 # RS232 daisy chain (default); false on RS485
      axes:
        - name: tth
          address: 1
          steps_per_rev: 4000    # optional: motor resolution, read from the drive if omitted
          steps_per_unit: 4000   # motor steps per user unit (e.g. per degree)
          velocity: 1            # user units/s
          acceleration: 10       # user units/s^2
          power_on_init: false   # energise the drive when the axis initialises
          home_velocity: 0.5     # optional, user units/s; sign comes from axis.home(±1)
          home_acceleration: 5   # optional, user units/s^2
          home_edge: "+"         # optional, "+" or "-" (default: search direction)
          home_switch_nc: false  # optional, home switch normally closed
          home_mode: 0           # optional, see HOME command (0, 1, 3, 4)

Axes with an encoder on the drive's X2 input can use it the Bliss way::

      encoders:
        - name: th_enc
          address: 1
          steps_per_unit: 2000   # PA counts per user unit
          tolerance: 0.005       # end-of-move check
      axes:
        - name: th
          address: 1
          encoder: $th_enc
          read_position: encoder # position from the encoder; moves corrected
          max_encoder_error: 0.005  # refuse to move if encoder and commanded
                                    # position differ more (spec's slop)

Positions are motor steps (PT, the commanded position), or encoder counts
(PA) with ``read_position: encoder``. Velocity and
acceleration are converted between Bliss's steps/s and the drive's rev/s
with the motor resolution (steps/rev), read from each drive at start-up
(``steps_per_rev`` in the config is optional and then cross-checked).

Drives with LOADENC on (load-mounted encoder, ST bit 29) work in load-encoder
steps: PT, PA, D, V and A are all in encoder counts, and the conversion uses
the system variable EM (counts per "rev") instead of the motor resolution.
Such drives usually also run position maintenance (POSMAIN): use the default
``read_position: controller`` and let the drive correct its own position.
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass

import gevent.lock

from bliss import global_map
from bliss.comm.exceptions import CommunicationTimeout
from bliss.comm.util import SERIAL, get_comm
from bliss.common.axis.state import AxisState
from bliss.common.logtools import log_debug, log_warning
from bliss.common.utils import object_method
from bliss.controllers.motor import Controller

from .drive import ViXBus, ViXDrive, ViXError
from .protocol import Mode, ProtocolError, parse_fields

_RESOLUTION = re.compile(r"RESOLUTION\s*\.*\s*(\d+)")


class BlissSerialTransport:
    """Adapts a ``bliss.comm`` serial object to the driver's transport interface."""

    def __init__(self, comm, poll: float = 0.02):
        self._comm = comm
        self._poll = poll

    def write(self, data: bytes) -> None:
        self._comm.write(data)

    def read(self, size: int = 64) -> bytes:
        try:
            return self._comm.raw_read(size, timeout=self._poll)
        except CommunicationTimeout:
            return b""

    def reset_input_buffer(self) -> None:
        self._comm.flush()

    def close(self) -> None:
        self._comm.close()


@dataclass
class _AxisData:
    drive: ViXDrive
    configured_steps_per_rev: int | None  # from YAML, optional
    power_on_init: bool
    home: dict
    velocity_rps: float | None = None
    acceleration_rpss: float | None = None
    mode: Mode | None = None  # last mode we set; None = unknown
    velocity_dirty: bool = False  # V was changed by a jog
    resolution: int | None = None  # motor steps/rev, resolved on first use
    max_encoder_error: float | None = None  # user units, spec's "slop"
    homing: bool = False  # a home search was started (state() then also reads ST1)
    # Caches (see ViX.read_position / ViX.state): every query costs 70-90 ms on
    # the real daisy chain, and Bliss reads position and state several times
    # around each move.
    moving: bool = False  # a motion command was sent and no stopped state seen since
    pos_cache: float | None = None  # last PT read from the drive (steps)
    pos_time: float = 0.0
    state_cache: tuple | None = None  # (state names, time) of the last stopped state


class ViX(Controller):
    """Bliss controller for Parker ViX IM drives sharing one serial line."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._bus: ViXBus | None = None
        self._comm = None
        self._axes_data: dict[str, _AxisData] = {}

    # -- initialisation ----------------------------------------------------

    def initialize(self):
        serial_cfg = self.config.config_dict.get("serial", {}) or {}
        url = str(serial_cfg.get("url", ""))
        if url.startswith("/"):
            # Local device: pyserial directly. With bliss.comm's Serial the
            # echo of longer commands (e.g. 1W(CQ,0)) lost its first bytes on
            # the real drives (2026-10-06), never with pyserial, also inside
            # Bliss (gevent patches select, so reads stay cooperative).
            import serial

            self._comm = serial.Serial(url, int(serial_cfg.get("baudrate", 9600)), timeout=0.02)
            transport = self._comm
        else:  # ser2net://, rfc2217://, tango://
            self._comm = get_comm(
                self.config.config_dict, ctype=SERIAL, baudrate=9600, eol="\r", timeout=1.0
            )
            transport = BlissSerialTransport(self._comm)
            global_map.register(self, children_list=[self._comm])
        self._bus = ViXBus(
            transport=transport,
            echo=self.config.get("echo", bool, True),
            timeout=self.config.get("timeout", float, 1.0),
            settle=self.config.get("settle", float, 0.03),
            lock=gevent.lock.RLock(),
        )

    def close(self):
        if self._comm is not None:
            self._comm.close()

    def initialize_axis(self, axis):
        address = axis.config.get("address", int)
        if address is None:
            raise ValueError(f"{axis.name}: 'address' (ViX drive address 1..255) is required")
        home = {
            key: axis.config.get(f"home_{key}", conv)
            for key, conv in (("velocity", float), ("acceleration", float),
                              ("edge", str), ("switch_nc", bool), ("mode", int))
        }
        self._axes_data[axis.name] = _AxisData(
            drive=self._bus.drive(address),
            configured_steps_per_rev=axis.config.get("steps_per_rev", int),
            power_on_init=axis.config.get("power_on_init", bool, False),
            home={k: v for k, v in home.items() if v is not None},
            max_encoder_error=axis.config.get("max_encoder_error", float),
        )
        if self._axes_data[axis.name].max_encoder_error is not None and not axis.config.get("encoder"):
            raise ValueError(f"{axis.name}: max_encoder_error needs an 'encoder' on the axis")

    def initialize_hardware_axis(self, axis):
        data = self._data(axis)
        drive = data.drive
        try:
            revision = drive.revision()
        except ViXError as exc:
            raise RuntimeError(
                f"{axis.name}: no ViX drive answering at address {drive.address}"
            ) from exc
        log_debug(self, "%s: ViX firmware %s at address %d", axis.name, revision, drive.address)

        st = drive.status()
        if st[30]:
            raise RuntimeError(
                f"{axis.name}: SCALE is enabled on drive {drive.address}; "
                "this controller works in motor or load-encoder steps (send SCALE0 and SV)"
            )
        self._steps_per_rev(axis)
        if axis.config.get("read_position", str, "controller") == "encoder":
            self._check_posmain_off(axis, drive)
        # Reads (position) must answer during moves: disable command queuing.
        drive.set_command_queuing(False)
        if data.power_on_init and not st[9]:
            self.set_on(axis)

    def _check_posmain_off(self, axis, drive: ViXDrive) -> None:
        """Refuse drive-side position maintenance together with Bliss encoder mode.

        With ``read_position: encoder`` Bliss corrects each target by the
        commanded-minus-encoder difference; POSMAIN would correct the same
        error again inside the drive.
        """
        try:
            fields = parse_fields(drive.query("POSMAIN"))
        except (ViXError, ProtocolError) as exc:
            log_warning(self, "%s: cannot read POSMAIN state (%s); make sure it is off "
                        "(POSMAIN0) when using read_position: encoder", axis.name, exc)
            return
        if fields.get("AR") == "1":
            raise RuntimeError(
                f"{axis.name}: position maintenance (POSMAIN) is armed on drive "
                f"{drive.address}, but read_position: encoder makes Bliss correct the "
                "position itself. Disable one of them (e.g. send POSMAIN0 and SV)."
            )

    def _steps_per_rev(self, axis) -> int:
        """Steps per "rev" used to convert velocity and acceleration.

        With LOADENC off: the motor resolution, read from the drive's STATUS
        report. With LOADENC on (ST bit 29): the system variable EM, since
        distance, velocity and acceleration are then in load-encoder steps and
        V1 = EM counts/s (manual, LOADENC command). If ``steps_per_rev`` is in
        the config it must match. Resolved lazily because Bliss runs
        ``initialize_hardware_axis`` only in the first process using the axis.
        """
        data = self._data(axis)
        if data.resolution is not None:
            return data.resolution
        configured = data.configured_steps_per_rev
        if data.drive.status()[29]:
            em = abs(int(float(data.drive.read_raw("EM"))))
            if configured is not None and configured != em:
                raise RuntimeError(
                    f"{axis.name}: drive {data.drive.address} has LOADENC on with EM={em} "
                    f"load steps/rev but steps_per_rev is {configured} in config"
                )
            log_debug(self, "%s: LOADENC on, EM=%d load steps/rev", axis.name, em)
            data.resolution = em
            return em
        try:
            match = _RESOLUTION.search("\n".join(data.drive.status_report()))
        except ViXError as exc:
            match = None
            log_warning(self, "%s: cannot read STATUS: %s", axis.name, exc)
        if match is not None:
            resolution = int(match.group(1))
            if configured is not None and configured != resolution:
                raise RuntimeError(
                    f"{axis.name}: drive {data.drive.address} motor resolution is "
                    f"{resolution} steps/rev but steps_per_rev is {configured} in config"
                )
        elif configured is not None:
            log_warning(self, "%s: using steps_per_rev=%d from config (not verified)",
                        axis.name, configured)
            resolution = configured
        else:
            raise RuntimeError(
                f"{axis.name}: cannot read the motor resolution from drive "
                f"{data.drive.address}; set steps_per_rev in the config"
            )
        data.resolution = resolution
        return resolution

    def _data(self, axis) -> _AxisData:
        return self._axes_data[axis.name]

    # Position read from the drive is reused for POSITION_TTL s while the axis
    # is stopped and no command that changes it has been sent; the state for
    # STATE_TTL s. The expiry keeps Bliss's discrepancy check meaningful when
    # another program (spec) moves a drive.
    POSITION_TTL = 2.0
    STATE_TTL = 0.5
    # While a move runs, the position is read again at most every
    # MOVING_REFRESH s (umv shows it); a short scan step ends before that, so
    # it costs scans no extra query.
    MOVING_REFRESH = 0.5

    @staticmethod
    def _invalidate(data: _AxisData) -> None:
        data.pos_cache = None
        data.state_cache = None

    def _invalidate_address(self, address: int) -> None:
        for data in self._axes_data.values():
            if address in (0, data.drive.address):
                self._invalidate(data)

    def _drive(self, axis) -> ViXDrive:
        return self._axes_data[axis.name].drive

    # -- power -------------------------------------------------------------

    def set_on(self, axis):
        """Energise, then re-apply settings a drive reset would have lost."""
        data = self._data(axis)
        self._invalidate(data)
        data.drive.energize()
        data.drive.set_command_queuing(False)
        if data.velocity_rps is not None:
            data.drive.set_motion(velocity=data.velocity_rps)
            data.velocity_dirty = False
        if data.acceleration_rpss is not None:
            data.drive.set_motion(acceleration=data.acceleration_rpss)
        data.mode = None

    def set_off(self, axis):
        self._invalidate(self._data(axis))
        self._drive(axis).de_energize()

    # -- state & position --------------------------------------------------

    def state(self, axis):
        """Read only the status bytes needed: R(ST3) answers in ~90 ms on the
        real drives, the full R(ST) takes ~250 ms (measured 2026-10-06).

        ST3: 18 possible stall, 19 moving; ST2: 9 energised, 13/14 limit seen
        during last move; ST1: 5 going home.

        Not bit 17 (position-maintenance move): it is LATCHED after a POSMAIN
        correction until the next G/GH/ON/Z (manual, POSMAIN; seen on the tth
        drive 2026-10-06, where it kept the axis "MOVING" forever).
        """
        data = self._data(axis)
        now = time.monotonic()
        if not data.moving and data.state_cache and now - data.state_cache[1] < self.STATE_TTL:
            state = AxisState()
            for name in data.state_cache[0]:
                state.set(name)
            return state
        state = self._read_state(data)
        if state.MOVING:
            data.moving = True
            data.state_cache = None
        else:
            if data.moving:  # motion just ended: the next position read goes to the drive
                data.moving = False
                data.pos_cache = None
            data.state_cache = (state.current_states_names, now)
        return state

    def _read_state(self, data: _AxisData) -> AxisState:
        drive = data.drive
        st3 = drive.status(3)
        if st3[19]:
            return AxisState("MOVING")
        st2 = drive.status(2)
        state = AxisState()
        if not st2[9]:
            state.set("FAULT" if drive.drive_faults() else "OFF")
            return state
        if st3[18]:  # possible stall
            state.set("FAULT")
            return state
        if data.homing and drive.status(1)[5]:
            state.set("MOVING")
            return state
        state.set("READY")
        if st2[14]:
            state.set("LIMPOS")
        if st2[13]:
            state.set("LIMNEG")
        return state

    def read_position(self, axis):
        """PT (the trajectory position) in steps. While a move runs, it is read
        from the drive at most every MOVING_REFRESH s, otherwise the last value
        is returned (Bliss only displays it); the first read after the move has
        ended goes to the drive, and that value is reused for POSITION_TTL s
        while the axis stays stopped and nothing is sent to it."""
        data = self._data(axis)
        if data.pos_cache is not None:
            ttl = self.MOVING_REFRESH if data.moving else self.POSITION_TTL
            if time.monotonic() - data.pos_time < ttl:
                return data.pos_cache
        data.pos_cache = float(data.drive.position_target())
        data.pos_time = time.monotonic()
        return data.pos_cache

    def set_position(self, axis, new_position):
        """The drive can only zero its position (W(PA,0) zeroes PA, PT, PE, PF)."""
        if abs(new_position) >= 0.5:
            raise ValueError(
                f"{axis.name}: a ViX drive can only reset its position to 0 steps; "
                "set the user position (axis.position = ...) to apply an offset instead"
            )
        self._invalidate(self._data(axis))
        self._drive(axis).zero_position()
        return self.read_position(axis)

    # -- velocity & acceleration -------------------------------------------

    # The drive reports V and AA with fewer decimals than it accepts. When the
    # read-back matches our setpoint to that precision, report the setpoint so
    # Bliss does not see a spurious change.
    _VELOCITY_QUANTUM = 0.01  # rps
    _ACCEL_QUANTUM = 0.1  # rps^2

    def read_velocity(self, axis):
        data = self._data(axis)
        spr = self._steps_per_rev(axis)
        rps = data.drive.velocity()
        if data.velocity_rps is not None and abs(rps - data.velocity_rps) <= self._VELOCITY_QUANTUM:
            rps = data.velocity_rps
        return rps * spr

    def set_velocity(self, axis, new_velocity):
        data = self._data(axis)
        rps = new_velocity / self._steps_per_rev(axis)
        data.drive.set_motion(velocity=rps)
        data.velocity_rps = rps
        data.velocity_dirty = False

    def read_acceleration(self, axis):
        data = self._data(axis)
        spr = self._steps_per_rev(axis)
        rpss = data.drive.acceleration()
        if data.acceleration_rpss is not None and abs(rpss - data.acceleration_rpss) <= self._ACCEL_QUANTUM:
            rpss = data.acceleration_rpss
        return rpss * spr

    def set_acceleration(self, axis, new_acc):
        data = self._data(axis)
        rpss = new_acc / self._steps_per_rev(axis)
        data.drive.set_motion(acceleration=rpss)
        data.acceleration_rpss = rpss

    # -- motion ------------------------------------------------------------

    def _set_mode(self, data: _AxisData, mode: Mode) -> None:
        if data.mode != mode:
            data.drive.set_mode(mode)
            data.mode = mode

    def _check_encoder_error(self, axis, data: _AxisData) -> None:
        """spec-style slop check: refuse to move when the encoder and the
        commanded position disagree by more than ``max_encoder_error``.

        Bliss's own discrepancy check cannot see this with
        ``read_position: encoder`` (it compares the encoder with itself) and
        would silently correct any number of lost steps on the next move.
        """
        encoder_dial = axis.encoder.read()
        commanded_dial = data.drive.position_target() / axis.steps_per_unit
        error = encoder_dial - commanded_dial
        if abs(error) > data.max_encoder_error:
            raise RuntimeError(
                f"{axis.name}: encoder ({encoder_dial:.5f}) and commanded position "
                f"({commanded_dial:.5f}) differ by {error:+.5f} > max_encoder_error "
                f"{data.max_encoder_error:g}; the motor may have lost steps. Check the "
                "axis, then re-home (axis.home()) or re-zero at a known position (axis.dial = 0)."
            )

    def prepare_move(self, motion):
        data = self._data(motion.axis)
        if data.max_encoder_error is not None:
            self._check_encoder_error(motion.axis, data)
        self._set_mode(data, Mode.ABSOLUTE)
        if data.velocity_dirty and data.velocity_rps is not None:
            data.drive.set_motion(velocity=data.velocity_rps)
            data.velocity_dirty = False
        data.drive.set_motion(distance=int(round(motion.target_pos)))
        data.state_cache = None

    def _started(self, data: _AxisData) -> None:
        data.moving = True
        data.state_cache = None
        data.pos_time = time.monotonic()  # first refresh MOVING_REFRESH s into the move

    def start_one(self, motion):
        data = self._data(motion.axis)
        self._started(data)
        data.drive.go()

    def start_all(self, *motions):
        started = []
        try:
            for motion in motions:
                self.start_one(motion)
                started.append(motion)
        except BaseException:
            self.stop_all(*started)
            raise

    def stop(self, axis):
        self._data(axis).state_cache = None
        self._drive(axis).stop()

    def stop_all(self, *motions):
        for motion in motions:
            try:
                self.stop(motion.axis)
            except ViXError as exc:
                log_warning(self, "stop failed on %s: %s", motion.axis.name, exc)

    def start_jog(self, axis, velocity, direction):
        data = self._data(axis)
        self._set_mode(data, Mode.CONTINUOUS)
        data.drive.set_direction(direction > 0)
        data.drive.set_motion(velocity=abs(velocity) / self._steps_per_rev(axis))
        data.velocity_dirty = True
        self._started(data)
        data.drive.go()

    def stop_jog(self, axis):
        self.stop(axis)

    def limit_search(self, axis, limit):
        data = self._data(axis)
        self._set_mode(data, Mode.CONTINUOUS)
        data.drive.set_direction(limit > 0)
        self._started(data)
        data.drive.go()

    # -- homing ------------------------------------------------------------

    def home_search(self, axis, switch):
        data = self._data(axis)
        drive = data.drive
        home = data.home
        spr = self._steps_per_rev(axis)
        drive.user_faults()  # a full UF read clears stale homing faults
        if "velocity" in home:
            rps = abs(home["velocity"] * axis.steps_per_unit) / spr
            accel = home.get("acceleration")
            rpss = (
                abs(accel * axis.steps_per_unit) / spr
                if accel is not None
                else data.acceleration_rpss or 10.0
            )
            edge = home.get("edge", "+" if switch > 0 else "-")
            drive.configure_homing(
                velocity=rps if switch > 0 else -rps,
                acceleration=rpss,
                edge_positive=edge == "+",
                normally_closed=home.get("switch_nc", False),
                mode=home.get("mode", 0),
            )
        else:
            log_warning(
                self,
                "%s: no home_velocity in config, using the HOME settings stored in "
                "the drive (search direction %+d ignored)", axis.name, switch,
            )
        data.mode = None
        data.homing = True
        self._invalidate(data)
        self._started(data)
        drive.command("GH")

    def home_state(self, axis):
        drive = self._drive(axis)
        st = drive.status()
        state = AxisState()
        if st[5] or st[19]:
            state.set("MOVING")
        elif not st[9]:
            state.set("FAULT" if drive.drive_faults() else "OFF")
        elif any(drive.user_faults(1)[b] for b in (5, 6, 7)):
            state.set("FAULT")  # homing failed: no signal / missing Z / too narrow
        else:
            state.set("READY")
        if not state.MOVING:
            data = self._data(axis)
            data.homing = data.moving = False
            self._invalidate(data)
        return state

    # -- encoders (X2 primary encoder, read-only) --------------------------

    def initialize_encoder(self, encoder):
        address = encoder.config.get("address", int)
        if address is None:
            raise ValueError(f"{encoder.name}: 'address' of the ViX drive is required")
        encoder.vix_drive = self._bus.drive(address)

    def read_encoder(self, encoder):
        return float(encoder.vix_drive.position_actual())

    def set_encoder(self, encoder, new_value):
        """Only 0 is possible: W(PA,0) zeroes the encoder and the commanded
        position together. Called when setting the dial of an axis with
        ``read_position: encoder``."""
        if abs(new_value) >= 0.5:
            raise ValueError(
                f"{encoder.name}: a ViX drive can only reset its encoder to 0; "
                "set the user position (axis.position = ...) to apply an offset instead"
            )
        self._invalidate_address(encoder.vix_drive.address)
        encoder.vix_drive.zero_position()

    # -- information & raw access ------------------------------------------

    def get_id(self, axis):
        return f"ViX at address {self._drive(axis).address}, firmware {self._drive(axis).revision()}"

    def get_axis_info(self, axis):
        data = self._data(axis)
        lines = [
            f"VIX DRIVE: address {data.drive.address}",
            f"    steps_per_rev: {self._steps_per_rev(axis)}",
        ]
        lines += ["    " + line for line in data.drive.status_report()]
        return "\n".join(lines)

    @staticmethod
    def _split_address(com: str) -> tuple[int, str]:
        match = re.fullmatch(r"\s*(\d+)(\S+)\s*", com)
        if match is None:
            raise ValueError("command must be '<address><command>', e.g. '1R(PT)'")
        return int(match.group(1)), match.group(2)

    def raw_write(self, com):
        """Send e.g. ``"1O(1XX)"`` (no reply expected)."""
        address, body = self._split_address(com)
        self._invalidate_address(address)
        self._bus.command(address, body)

    def raw_write_read(self, com):
        """Send e.g. ``"1R(PT)"`` and return the reply lines (``*`` included)."""
        address, body = self._split_address(com)
        self._invalidate_address(address)
        return "\n".join(self._bus.query_lines(address, body, idle=0.2))

    @object_method(types_info=("None", "str"))
    def vix_faults(self, axis):
        """Drive (DF) and user (UF) fault descriptions. Reading UF clears it."""
        drive = self._drive(axis)
        return f"drive faults: {drive.drive_faults()}\nuser faults: {drive.user_faults()}"

    @object_method(types_info=("None", "str"))
    def vix_status(self, axis):
        """Decoded ST status word."""
        return str(self._drive(axis).status())

    @object_method(types_info=("None", "str"))
    def vix_inputs(self, axis):
        """User inputs 1..5 (stop, reg, home, lim-, lim+)."""
        names = ("stop", "reg", "home", "lim-", "lim+")
        return ", ".join(
            f"{n}={'1' if v else '0'}" for n, v in zip(names, self._drive(axis).inputs())
        )
