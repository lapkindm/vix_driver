"""Serial driver for Parker ViX250IM / ViX500IM microstepper indexer drives.

Typical use::

    from vix_driver import ViXBus, Mode

    with ViXBus("/dev/ttyUSB0") as bus:
        drive = bus.drive(1)
        drive.energize()
        drive.set_motion(velocity=5, acceleration=50)   # rps, rps^2
        drive.move_relative(4000)                       # steps; waits for completion
        print(drive.position_target())

Units are motor steps, revs/s and revs/s^2 unless SCALE or LOADENC is
enabled on the drive, in which case they are the configured user units.
"""

from __future__ import annotations

import logging
import re
import threading
import time
from typing import TYPE_CHECKING, Protocol

from . import protocol as p
from .protocol import BitReport, CommsMode, Mode

if TYPE_CHECKING:
    from .group import DriveGroup

log = logging.getLogger(__name__)

_LINE_END = re.compile(r"[\r\n]")


class ViXError(Exception):
    """Base class for driver errors."""


class ViXTimeoutError(ViXError):
    """The drive did not reply in time."""


class ViXCommandError(ViXError):
    """The drive rejected a command with ``*E``.

    ``user_faults`` holds the decoded UF word read right after the rejection
    (when available), which explains why.
    """

    def __init__(self, command: str, reply: str, user_faults: BitReport | None = None):
        reason = f": {user_faults}" if user_faults else ""
        super().__init__(f"{command!r} rejected by drive ({reply}){reason}")
        self.command = command
        self.reply = reply
        self.user_faults = user_faults


class DriveFaultError(ViXError):
    """The drive reports a hardware fault (DF)."""

    def __init__(self, faults: BitReport, address: int | None = None):
        where = f"drive {address}: " if address is not None else ""
        super().__init__(f"{where}drive fault: {faults}")
        self.faults = faults
        self.address = address

    @property
    def terminal(self) -> bool:
        """True if a fault needs a power cycle or repair to clear."""
        return any(b in p.TERMINAL_DRIVE_FAULTS for b in self.faults.set_bits)


class LimitError(ViXError):
    """A move was stopped by an end-of-travel limit."""

    def __init__(self, status: BitReport, address: int | None = None):
        sides = [s for bit, s in ((13, "-ve"), (14, "+ve")) if status[bit]]
        where = f"drive {address}: " if address is not None else ""
        super().__init__(f"{where}{' and '.join(sides)} limit hit during move")
        self.status = status
        self.address = address


class Transport(Protocol):
    """The subset of ``serial.Serial`` the driver uses."""

    def write(self, data: bytes) -> int | None: ...
    def read(self, size: int = 1) -> bytes: ...
    def reset_input_buffer(self) -> None: ...
    def close(self) -> None: ...


class ViXBus:
    """One serial port shared by one or more drives with different addresses.

    Replies from the drives carry no address, so the bus runs one
    transaction at a time (thread-safe) and ties each reply to its command.

    With ``echo=True`` (RS232 daisy chain, EX = 2 or 3) every command travels
    round the ring and comes back to the PC. The bus waits for that echo
    before accepting a reply, so stray ``*`` lines from other drives that
    arrive earlier are discarded and a missing echo is reported as a broken
    chain. Use ``echo=False`` on an RS485 multidrop bus, where drives must
    run with echo off (EX = 0 or 1).

    :param port: serial device, e.g. ``"/dev/ttyUSB0"`` or ``"COM3"``.
        Ignored when ``transport`` is given.
    :param baudrate: 9600 (factory default) or 19200.
    :param echo: whether the drives echo commands (see above).
    :param timeout: default seconds to wait for a reply.
    :param settle: seconds to listen for an ``*E`` after commands that
        produce no reply.
    :param xonxoff: enable software flow control (the drive supports it).
    :param transport: an already-open serial-like object (for tests or
        custom links); the bus does not configure it.
    :param lock: the re-entrant lock serialising transactions. Defaults to
        ``threading.RLock()``; under gevent without thread patching (e.g.
        Bliss) pass ``gevent.lock.RLock()``, otherwise greenlets sharing an
        OS thread would all "own" the lock and interleave transactions.
    """

    def __init__(
        self,
        port: str | None = None,
        baudrate: int = 9600,
        *,
        echo: bool = True,
        timeout: float = 1.0,
        settle: float = 0.05,
        xonxoff: bool = False,
        transport: Transport | None = None,
        lock=None,
    ):
        if transport is None:
            if port is None:
                raise ValueError("either port or transport is required")
            if baudrate not in p.BAUD_RATES:
                raise ValueError(f"baudrate must be one of {p.BAUD_RATES}")
            import serial  # pyserial; imported lazily so tests run without it

            transport = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=xonxoff,
                timeout=0.02,
            )
        self._io = transport
        self.echo = echo
        self.timeout = timeout
        self.settle = settle
        self.resync_time = 0.1
        self.known_addresses: frozenset[int] | None = None
        self._lock = lock if lock is not None else threading.RLock()
        self._pending = ""
        self._dirty = False

    # -- lifecycle ---------------------------------------------------------

    def close(self) -> None:
        self._io.close()

    def __enter__(self) -> ViXBus:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def drive(self, address: int) -> ViXDrive:
        return ViXDrive(self, address)

    def drives(self, *addresses: int) -> list[ViXDrive]:
        return [ViXDrive(self, a) for a in addresses]

    def group(self, *addresses: int) -> DriveGroup:
        """A :class:`DriveGroup` for coordinated multi-axis commands.

        With no addresses, uses every drive found by :meth:`discover`.
        """
        from .group import DriveGroup

        if not addresses:
            if self.known_addresses is None:
                raise ValueError("give addresses or call discover() first")
            addresses = tuple(sorted(self.known_addresses))
        return DriveGroup(self.drives(*addresses))

    def broadcast(self) -> ViXDrive:
        """A handle on address 0: commands go to every drive, reads are refused."""
        return ViXDrive(self, p.GLOBAL_ADDRESS)

    # -- bus-wide operations -----------------------------------------------

    def discover(self, addresses=range(1, 17), timeout: float = 0.15) -> dict[int, str]:
        """Find the drives on the bus by asking each address for its firmware revision.

        Returns ``{address: revision}`` and remembers the addresses in
        :attr:`known_addresses`. Each absent address costs ``timeout`` seconds.
        """
        found: dict[int, str] = {}
        for address in addresses:
            try:
                found[address] = p.strip_reply(self.query(address, "R(RV)", timeout))
            except ViXTimeoutError:
                continue
        self.known_addresses = frozenset(found)
        return found

    def stop_all(self) -> None:
        """Controlled stop (``0S``) on every drive."""
        self.command(p.GLOBAL_ADDRESS, "S", settle=0)

    def kill_all(self) -> None:
        """Kill (``0K``) every drive. Not a substitute for a hardware E-stop."""
        self.command(p.GLOBAL_ADDRESS, "K", settle=0)

    def set_comms_mode_all(self, mode: CommsMode | int) -> None:
        """Set EX on every drive (``0W(EX,n)``) and update :attr:`echo` to match.

        On an RS232 daisy chain keep echo on (QUIET_ECHO or VERBOSE_ECHO): the
        chain relays commands as echoes. QUIET_ECHO (EX=2) is recommended for
        multi-drive use, so drives only speak when addressed.
        """
        mode = CommsMode(mode)
        self.command(p.GLOBAL_ADDRESS, f"W(EX,{mode.value})")
        self.echo = mode in (CommsMode.QUIET_ECHO, CommsMode.VERBOSE_ECHO)

    # -- raw I/O -----------------------------------------------------------

    def _send(self, address: int, body: str, timeout: float | None) -> float:
        """Send a command; with echo on, also consume its echo.

        Returns the deadline for the reply (``timeout`` from now, after any resync).
        """
        wire = p.format_command(address, body)
        if self._dirty:
            # A previous transaction timed out; its reply may still be on the way.
            self._drain(time.monotonic() + self.resync_time)
            self._dirty = False
        deadline = time.monotonic() + (timeout or self.timeout)
        self._io.reset_input_buffer()  # drop unsolicited output (EX = 1 or 3)
        self._pending = ""
        log.debug("TX %r", wire)
        self._io.write(wire.encode("ascii"))
        if not self.echo:
            return deadline
        echo = wire.strip().upper()
        while (line := self._read_line(deadline)) is not None:
            if line.upper() == echo:
                return deadline
            log.debug("discarding %r received before echo of %r", line, echo)
        self._dirty = True
        raise ViXTimeoutError(
            f"no echo of {echo!r}: daisy chain broken, or the drives have echo "
            "off (then create the bus with echo=False)"
        )

    def _drain(self, until: float) -> None:
        while self._read_line(until) is not None:
            pass

    def _read_line(self, deadline: float) -> str | None:
        """Return the next non-empty line, or None once ``deadline`` passes."""
        while True:
            parts = _LINE_END.split(self._pending, maxsplit=1)
            if len(parts) == 2:
                line, self._pending = parts[0].strip(), parts[1]
                if line:
                    log.debug("RX %r", line)
                    return line
                continue
            if time.monotonic() >= deadline:
                return None
            # Return as soon as a byte arrives (read(64) would wait out the
            # port timeout for every reply shorter than 64 bytes), then take
            # whatever else is already buffered.
            chunk = self._io.read(1)
            if not chunk:
                continue
            waiting = getattr(self._io, "in_waiting", 0)
            if waiting:
                chunk += self._io.read(waiting)
            text = chunk.decode("ascii", errors="replace")
            if p.BELL in text:
                log.warning("drive reported a buffer overflow (BEL)")
            for c in (p.XON, p.XOFF, p.BELL):
                text = text.replace(c, "")
            self._pending += text

    def _replies(self, deadline: float):
        """Yield ``*`` reply lines until ``deadline``; other lines are skipped."""
        while (line := self._read_line(deadline)) is not None:
            if line.startswith("*"):
                yield line

    # -- command primitives ------------------------------------------------

    def query(self, address: int, body: str, timeout: float | None = None) -> str:
        """Send a command and return its single ``*`` reply line."""
        if address == p.GLOBAL_ADDRESS:
            raise ValueError("drives ignore read commands sent to address 0")
        with self._lock:
            deadline = self._send(address, body, timeout)
            for line in self._replies(deadline):
                if p.is_error(line):
                    raise ViXCommandError(f"{address}{body}", line)
                return line
            self._dirty = True
            raise ViXTimeoutError(
                f"no reply to {address}{body!r} (no drive at this address? if a move "
                "is running with CQ=1, reads other than R(ST)/R(DF)/R(UF)/R(RB) "
                "wait for it to finish)"
            )

    def query_lines(
        self,
        address: int,
        body: str,
        timeout: float | None = None,
        idle: float = 0.3,
        replies_only: bool = True,
    ) -> list[str]:
        """Send a command with a multi-line reply (e.g. STATUS, ARM, LIST).

        Collects lines until ``idle`` seconds pass with no new line. With
        ``replies_only`` only ``*`` lines are kept (LIST output has no ``*``).
        """
        if address == p.GLOBAL_ADDRESS:
            raise ValueError("drives ignore read commands sent to address 0")
        with self._lock:
            deadline = self._send(address, body, timeout)
            lines: list[str] = []
            while (line := self._read_line(deadline)) is not None:
                if line.startswith("*") or not replies_only:
                    lines.append(line)
                    deadline = time.monotonic() + idle
            if not lines:
                self._dirty = True
                raise ViXTimeoutError(f"no reply to {address}{body!r}")
            if p.is_error(lines[0]):
                raise ViXCommandError(f"{address}{body}", lines[0])
            return lines

    def command(self, address: int, body: str, settle: float | None = None) -> None:
        """Send a command that has no reply, raising if the drive answers ``*E``."""
        with self._lock:
            self._send(address, body, None)
            deadline = time.monotonic() + (self.settle if settle is None else settle)
            for line in self._replies(deadline):
                if p.is_error(line):
                    raise ViXCommandError(f"{address}{body}", line)
                log.debug("ignoring unexpected reply %r", line)


class ViXDrive:
    """A single ViX250IM / ViX500IM drive at a given address on a :class:`ViXBus`."""

    def __init__(self, bus: ViXBus, address: int):
        if not p.GLOBAL_ADDRESS <= address <= p.MAX_ADDRESS:
            raise ValueError(f"address must be 0..{p.MAX_ADDRESS}")
        self.bus = bus
        self.address = address

    def __repr__(self) -> str:
        return f"ViXDrive(address={self.address})"

    # -- raw access --------------------------------------------------------

    def query(self, body: str, timeout: float | None = None) -> str:
        """Send ``body`` and return the raw ``*`` reply line."""
        return self.bus.query(self.address, body, timeout)

    def command(self, body: str, settle: float | None = None) -> None:
        """Send ``body`` (no reply expected).

        On ``*E`` the UF word is read so the raised :class:`ViXCommandError`
        says why. Note this clears UF on the drive.
        """
        try:
            self.bus.command(self.address, body, settle)
        except ViXCommandError as exc:
            if self.address != p.GLOBAL_ADDRESS:
                try:
                    exc = ViXCommandError(exc.command, exc.reply, self.user_faults())
                except ViXError:
                    pass
            raise exc from None

    # -- system variables --------------------------------------------------

    def read(self, name: str) -> int | float:
        """Read a numeric system variable, e.g. ``read("PT")``."""
        name = name.upper()
        if name not in p.VARIABLES:
            raise ValueError(f"unknown system variable {name!r}")
        if name in p.BITFIELD_VARIABLES:
            raise ValueError(f"{name} is a bit field; use status()/drive_faults()/user_faults()")
        return p.parse_number(self.query(f"R({name})"))

    def read_raw(self, name: str) -> str:
        """Read any system variable and return the reply text without the ``*``."""
        return p.strip_reply(self.query(f"R({name.upper()})"))

    def write(self, name: str, value: float | int) -> None:
        """Write a system variable, e.g. ``write("IT", 50)``."""
        name = name.upper()
        var = p.VARIABLES.get(name)
        if var is None:
            raise ValueError(f"unknown system variable {name!r}")
        if not var.writable:
            raise ValueError(f"{name} ({var.description}) is read-only")
        if var.minimum is not None and value < var.minimum:
            raise ValueError(f"{name} must be >= {var.minimum}, got {value}")
        if var.maximum is not None and value > var.maximum:
            raise ValueError(f"{name} must be <= {var.maximum}, got {value}")
        if name == "BR" and value not in p.BAUD_RATES:
            raise ValueError(f"BR must be one of {p.BAUD_RATES}")
        self.command(f"W({name},{p.format_value(value)})")

    # -- positions & flags -------------------------------------------------

    def position_target(self) -> int:
        """PT: commanded (open loop) position. The useful one without an encoder."""
        return int(self.read("PT"))

    def position_actual(self) -> int:
        """PA: position from the primary encoder on X2 (needs an encoder)."""
        return int(self.read("PA"))

    def position_error(self) -> int:
        """PE: PT - PA."""
        return int(self.read("PE"))

    def position_incremental(self) -> int:
        """PI: distance moved by the last move."""
        return int(self.read("PI"))

    def zero_position(self) -> None:
        """Set PA, PT, PF and PE to zero."""
        self.write("PA", 0)

    def is_moving(self) -> bool:
        """ST bit 19. Uses R(ST), which answers immediately even mid-move."""
        return self.status()[19]

    def in_position(self) -> bool:
        """ST bit 20: stationary and the in-position time (IT) has elapsed."""
        return self.status()[20]

    def is_busy(self) -> bool:
        """RB: 1 while a program or subroutine is executing. Always immediate."""
        return bool(self.read("RB"))

    def analogue_input(self) -> int:
        """AI: differential analogue input, -2047..+2047."""
        return int(self.read("AI"))

    # -- status & faults ---------------------------------------------------

    def _bits(self, var: str, names: dict[int, str], byte: int | None) -> BitReport:
        if byte is None:
            return p.parse_bits(self.query(f"R({var})"), names)
        if not 1 <= byte <= 4:
            raise ValueError("byte must be 1..4")
        return p.parse_bits(self.query(f"R({var}{byte})"), names, first_bit=8 * (byte - 1) + 1)

    def status(self, byte: int | None = None) -> BitReport:
        """ST status word (see :data:`protocol.STATUS_BITS`)."""
        return self._bits("ST", p.STATUS_BITS, byte)

    def drive_faults(self, byte: int | None = None) -> BitReport:
        """DF hardware fault word (see :data:`protocol.DRIVE_FAULT_BITS`)."""
        return self._bits("DF", p.DRIVE_FAULT_BITS, byte)

    def user_faults(self, byte: int | None = None) -> BitReport:
        """UF user fault word. Reading the whole word clears it on the drive."""
        return self._bits("UF", p.USER_FAULT_BITS, byte)

    def is_energized(self) -> bool:
        return self.status()[9]

    def check_faults(self) -> None:
        """Raise :class:`DriveFaultError` if any drive fault is active."""
        faults = self.drive_faults()
        if faults:
            raise DriveFaultError(faults, self.address)

    def status_report(self, timeout: float = 3.0, idle: float = 1.0) -> list[str]:
        """The verbose STATUS listing, one string per line (``*`` stripped).

        A real ViX500IM (firmware 3.1) sends it over ~3.3 s with gaps of up
        to 0.3 s between lines, hence the long ``idle``.
        """
        lines = self.bus.query_lines(self.address, "STATUS", timeout=timeout, idle=idle)
        return [line[1:] for line in lines]

    def revision(self) -> str:
        return self.read_raw("RV")

    # -- power stage -------------------------------------------------------

    def energize(self, check: bool = True, delay: float = 0.2) -> None:
        """Energise the motor (``ON``). Also clears the DF and UF words.

        The hardware enable input (X4 pin 11, sense ES) must be active.
        With ``check`` set, confirm via ST after ``delay`` seconds and raise
        :class:`DriveFaultError` (or :class:`ViXError`) if it did not energise.
        Not allowed in mode P (the enable input energises the drive there).
        """
        self.command("ON")
        if not check or self.address == p.GLOBAL_ADDRESS:
            return
        time.sleep(delay)
        if not self.is_energized():
            self.check_faults()
            raise ViXError(f"drive {self.address}: did not energise (enable input active? motor defined?)")

    def de_energize(self) -> None:
        """De-energise the motor (``OFF``)."""
        self.command("OFF")

    # -- motion parameters -------------------------------------------------

    def set_mode(self, mode: Mode | str) -> None:
        """Select absolute, incremental, continuous, bidirectional or position mode."""
        self.command(f"M{Mode(mode).value}")

    def get_mode(self) -> Mode:
        return p.parse_mode(self.query("M"))

    def set_motion(
        self,
        velocity: float | None = None,
        acceleration: float | None = None,
        deceleration: float | None = None,
        distance: int | None = None,
    ) -> None:
        """Set any of V, AA, AD, D. If only ``acceleration`` is given, it is used
        for deceleration too (``A`` command)."""
        # Round to the drive's resolution (V: 0.001, A/AA/AD: 0.01).
        if acceleration is not None:
            acceleration = round(acceleration, 2)
            _check_accel(acceleration)
        if deceleration is not None:
            deceleration = round(deceleration, 2)
            _check_accel(deceleration)
        if velocity is not None:
            velocity = round(velocity, 3)
            if not 0.001 <= velocity <= 5000:
                raise ValueError("velocity must be 0.001..50 rps (..5000 user units with SCALE)")
        if distance is not None and not -p.INT32_MAX <= distance <= p.INT32_MAX:
            raise ValueError("distance out of 32-bit range")

        if acceleration is not None and deceleration is None:
            self.command(f"A{p.format_value(acceleration)}")
        else:
            if acceleration is not None:
                self.command(f"AA{p.format_value(acceleration)}")
            if deceleration is not None:
                self.command(f"AD{p.format_value(deceleration)}")
        if velocity is not None:
            self.command(f"V{p.format_value(velocity)}")
        if distance is not None:
            self.command(f"D{int(distance)}")

    def velocity(self) -> float:
        return p.parse_number(self.query("V"))

    def acceleration(self) -> float:
        return p.parse_number(self.query("AA"))

    def deceleration(self) -> float:
        return p.parse_number(self.query("AD"))

    def distance(self) -> int:
        return int(p.parse_number(self.query("D")))

    def set_direction(self, positive: bool | None = None) -> None:
        """``H+`` (CW), ``H-`` (CCW), or with ``None`` reverse the current direction.
        Ignored in mode A."""
        self.command("H" if positive is None else ("H+" if positive else "H-"))

    # -- motion ------------------------------------------------------------

    def go(self) -> None:
        """Start a move with the current mode and parameters (``G``)."""
        self.command("G")

    def stop(self) -> None:
        """Controlled stop at the current deceleration (``S``). Also aborts programs."""
        self.command("S", settle=0)

    def kill(self) -> None:
        """Stop at the LIMITS deceleration (default 200 rps^2) and abort programs (``K``).

        Not a substitute for a hardware emergency stop.
        """
        self.command("K", settle=0)

    def pause(self) -> None:
        """Hold command execution until :meth:`resume` (``PS``)."""
        self.command("PS")

    def resume(self) -> None:
        """Continue after :meth:`pause` (``C``)."""
        self.command("C")

    def move_absolute(
        self, position: int, velocity: float | None = None, acceleration: float | None = None,
        *, wait: bool = True, timeout: float | None = None,
    ) -> None:
        """Move to ``position`` in mode A. Waits for completion unless ``wait=False``."""
        self.set_mode(Mode.ABSOLUTE)
        self.set_motion(velocity=velocity, acceleration=acceleration, distance=position)
        self.go()
        if wait:
            self.wait_for_move(timeout)

    def move_relative(
        self, distance: int, velocity: float | None = None, acceleration: float | None = None,
        *, wait: bool = True, timeout: float | None = None,
    ) -> None:
        """Move by ``distance`` (signed) in mode I. Waits unless ``wait=False``."""
        self.set_mode(Mode.INCREMENTAL)
        self.set_motion(velocity=velocity, acceleration=acceleration, distance=distance)
        self.go()
        if wait:
            self.wait_for_move(timeout)

    def jog(self, velocity: float, positive: bool = True, acceleration: float | None = None) -> None:
        """Run continuously (mode C) until :meth:`stop` or :meth:`kill`."""
        self.set_mode(Mode.CONTINUOUS)
        self.set_direction(positive)
        self.set_motion(velocity=velocity, acceleration=acceleration)
        self.go()

    def wait_for_move(
        self,
        timeout: float | None = None,
        poll: float = 0.05,
        require_in_position: bool = True,
    ) -> BitReport:
        """Block until the motor stops; returns the final status word.

        Polls R(ST), which the drive answers immediately even while moving.
        Raises :class:`DriveFaultError` if the drive faulted, :class:`LimitError`
        if a limit stopped the move, :class:`ViXTimeoutError` on timeout.

        :param require_in_position: also wait for ST bit 20, which is set
            once the motor has been still for the in-position time IT.
        """
        deadline = None if timeout is None else time.monotonic() + timeout
        time.sleep(poll)  # let the drive start the move before the first poll
        while not move_finished(st := self.status(), require_in_position):
            if deadline is not None and time.monotonic() > deadline:
                raise ViXTimeoutError(f"drive {self.address}: move did not finish within {timeout} s")
            time.sleep(poll)
        self.check_move_result(st)
        return st

    def check_move_result(self, st: BitReport) -> None:
        """Raise if the final status of a move shows a fault, a de-energised
        drive or a limit hit."""
        if not st[9] or st[18]:
            self.check_faults()
        if not st[9]:
            raise ViXError(f"drive {self.address}: de-energised during move")
        if st[13] or st[14]:
            raise LimitError(st, self.address)

    # -- homing & limits ---------------------------------------------------

    def configure_homing(
        self,
        velocity: float,
        acceleration: float,
        *,
        edge_positive: bool = True,
        normally_closed: bool = False,
        mode: int = 0,
        enabled: bool = True,
    ) -> None:
        """Configure homing (``HOME``) for :meth:`go_home`.

        :param velocity: search velocity in rps; its sign sets the search direction.
        :param acceleration: accel/decel in rps^2.
        :param edge_positive: home on the switch edge nearest the +ve (CW) limit.
        :param normally_closed: home switch type.
        :param mode: 0 = stop in switch window, 1 = stop on edge,
            3 = seek encoder Z after switch, 4 = seek Z without a switch.
        """
        if mode not in (0, 1, 3, 4):
            raise ValueError("mode must be 0, 1, 3 or 4")
        _check_accel(acceleration)
        args = ",".join((
            "+" if edge_positive else "-",
            "1" if normally_closed else "0",
            p.format_value(velocity),
            p.format_value(acceleration),
            str(mode),
        ))
        self.command(f"HOME{int(enabled)}({args})")

    def homing_config(self) -> dict[str, str]:
        """Current HOME settings, e.g. ``{'AR': '1', 'E': '-', 'TP': '1', 'V': '+10.00', ...}``."""
        return p.parse_fields(self.query("HOME"))

    def go_home(self, *, wait: bool = True, timeout: float | None = 60.0) -> None:
        """Run the configured homing sequence (``GH``); PA is zeroed at home."""
        self.command("GH")
        if not wait:
            return
        time.sleep(0.05)
        deadline = None if timeout is None else time.monotonic() + timeout
        while self.status()[5]:  # ST bit 5: going home
            if deadline is not None and time.monotonic() > deadline:
                raise ViXTimeoutError(f"drive {self.address}: homing did not finish within {timeout} s")
            time.sleep(0.05)
        self.wait_for_move(timeout)
        uf = self.user_faults(1)
        failed = [b for b in (5, 6, 7) if uf[b]]
        if failed:
            raise ViXError(f"drive {self.address}: homing failed: {', '.join(p.USER_FAULT_BITS[b] for b in failed)}")

    def configure_limits(
        self,
        *,
        positive_enabled: bool = True,
        negative_enabled: bool = True,
        normally_closed: bool = True,
        continue_program: bool = False,
        deceleration: float | None = None,
    ) -> None:
        """Configure the LIM+/LIM- inputs (``LIMITS``).

        ``deceleration`` (rps^2, default 200 on the drive) also sets the K deceleration.
        """
        mask = (0 if positive_enabled else 1) + (0 if negative_enabled else 2)
        args = [str(mask), "1" if normally_closed else "0", "1" if continue_program else "0"]
        if deceleration is not None:
            _check_accel(deceleration)
            args.append(p.format_value(deceleration))
        self.command(f"LIMITS({','.join(args)})")

    def limits_config(self) -> dict[str, str]:
        """Current LIMITS settings, e.g. ``{'LM': '0', 'TP': '1', 'M': '0', 'LD': '200.0'}``."""
        return p.parse_fields(self.query("LIMITS"))

    # -- I/O ---------------------------------------------------------------

    def inputs(self) -> tuple[bool, ...]:
        """User inputs 1..5 (IS): stop, reg, home, LIM-, LIM+. True = closed contact."""
        return p.parse_bits(self.query("IS")).bits

    def set_outputs(self, *states: bool | None) -> None:
        """Set outputs 1..3 in order; ``None`` leaves an output unchanged.

        ``set_outputs(True, None, False)`` sends ``O(1X0)``.
        """
        if not 1 <= len(states) <= p.NUM_OUTPUTS:
            raise ValueError(f"give 1..{p.NUM_OUTPUTS} output states")
        pattern = "".join("X" if s is None else ("1" if s else "0") for s in states)
        self.command(f"O({pattern})")

    def set_output(self, output: int, state: bool) -> None:
        """Set a single output (1..3), leaving the others unchanged."""
        if not 1 <= output <= p.NUM_OUTPUTS:
            raise ValueError(f"output must be 1..{p.NUM_OUTPUTS}")
        states: list[bool | None] = [None] * output
        states[-1] = state
        self.set_outputs(*states)

    # -- profiles ----------------------------------------------------------

    def define_profile(
        self, number: int, acceleration: float, deceleration: float, distance: int, velocity: float
    ) -> None:
        """Store move profile 1..8 (``PROFILEn(AA,AD,D,V)``)."""
        if not 1 <= number <= 8:
            raise ValueError("profile number must be 1..8")
        _check_accel(acceleration)
        _check_accel(deceleration)
        args = ",".join(p.format_value(v) for v in (acceleration, deceleration, int(distance), velocity))
        self.command(f"PROFILE{number}({args})")

    def use_profile(self, number: int) -> None:
        """Copy profile 1..8 into the current move parameters (``USE``).

        Warning: the drive silently uses profile 0 if the profile is undefined.
        """
        if not 1 <= number <= 8:
            raise ValueError("profile number must be 1..8")
        self.command(f"USE({number})")

    def current_profile(self) -> dict[str, str]:
        """Current move parameters (``PROFILE0``), e.g. ``{'AA': '10.0', 'D': '4000', ...}``."""
        return p.parse_fields(self.query("PROFILE0"))

    # -- drive configuration -----------------------------------------------

    def set_motor(
        self,
        motor_type: int,
        current: float,
        resolution: int,
        max_velocity: int,
        third_harmonic: float,
        resistance: float,
        inductance: float,
        timeout: float = 14.0,
    ) -> None:
        """Define the motor (``MOTOR``). Can take up to 12 s to execute.

        Issue it with HV present (otherwise 80 V is assumed), then :meth:`save`
        and :meth:`reset` for a new motor type to take effect.

        :param motor_type: 0..1023 (bits 0-7 id, bit 8 temp sensor, bit 9 series).
        :param current: RMS motor current, 0.1..5.6 A (ViX500).
        :param resolution: 200..51200 steps/rev.
        :param max_velocity: 1..3000 rpm.
        :param third_harmonic: -15..15 %.
        :param resistance: line-to-line winding resistance, ohms.
        :param inductance: line-to-line winding inductance, mH.
        """
        checks = (
            (0 <= motor_type <= 1023, "motor_type must be 0..1023"),
            (0.1 <= current <= 5.6, "current must be 0.1..5.6 A"),
            (200 <= resolution <= 51200, "resolution must be 200..51200 steps/rev"),
            (1 <= max_velocity <= 3000, "max_velocity must be 1..3000 rpm"),
            (-15 <= third_harmonic <= 15, "third_harmonic must be -15..15 %"),
        )
        for ok, message in checks:
            if not ok:
                raise ValueError(message)
        args = ",".join(
            p.format_value(v)
            for v in (motor_type, round(current, 1), resolution, max_velocity,
                      third_harmonic, resistance, inductance)
        )
        self.command(f"MOTOR({args})", settle=timeout)

    def configure_stall_detect(self, error_window: int = 4000, run_fault_label: bool = False,
                               output: int = 0, enabled: bool = True) -> None:
        """Encoder-based stall detection (``STALL``); needs an encoder on X2 and EM set."""
        if not 0 <= error_window <= 65535:
            raise ValueError("error_window must be 0..65535")
        if not 0 <= output <= p.NUM_OUTPUTS:
            raise ValueError("output must be 0..3 (0 = none)")
        self.command(f"STALL{int(enabled)}({error_window},{int(run_fault_label)},{output})")

    def configure_scale(self, scla: int, scld: int, sclv: int, peu: int, enabled: bool = True) -> None:
        """User units for A, D and V (``SCALE``). Issue once; not for on-the-fly changes."""
        for name, v in (("scla", scla), ("scld", scld), ("sclv", sclv), ("peu", peu)):
            if not 1 <= v <= 100_000_000:
                raise ValueError(f"{name} must be 1..100000000")
        self.command(f"SCALE{int(enabled)}({scla},{scld},{sclv},{peu})")

    def set_command_queuing(self, enabled: bool) -> None:
        """CQ. With queuing off, commands (and position reads) act immediately mid-move."""
        self.write("CQ", int(enabled))

    def set_comms_mode(self, mode: CommsMode | int) -> None:
        """Set EX (echo / unsolicited output). Takes effect immediately."""
        self.write("EX", CommsMode(mode).value)

    # -- programs ----------------------------------------------------------

    def run_label(self, label: str) -> None:
        """Run a stored program label (``GOTO``)."""
        self.command(f"GOTO({label.upper()})")

    def arm(self, start: bool, fault: bool = True) -> None:
        """Enable/disable the START label at power-up and the FAULT label (``ARM``)."""
        self.command(f"ARM{int(start)}{int(fault)}")

    def list_program(self, label: str = "ALL", timeout: float = 3.0) -> list[str]:
        """Return stored program lines (``LIST``)."""
        return self.bus.query_lines(
            self.address, f"LIST({label.upper()})", timeout=timeout, replies_only=False
        )

    def clear_program(self, label: str = "ALL") -> None:
        """Delete a program label, or all programs and declarations (``CLEAR``)."""
        self.command(f"CLEAR({label.upper()})")

    # -- persistence -------------------------------------------------------

    def save(self, wait: float = 2.0) -> None:
        """Store variables and programs in non-volatile memory (``SV``)."""
        self.command("SV", settle=0)
        time.sleep(wait)

    def reset(self, wait: float = 2.0) -> None:
        """Reset the controller (``Z``); unsaved settings are lost."""
        self.command("Z", settle=0)
        time.sleep(wait)

    def factory_reset(self) -> None:
        """Restore factory settings (``RFS``). Drive must be OFF; :meth:`save` to keep."""
        self.command("RFS")


def move_finished(st: BitReport, require_in_position: bool = True) -> bool:
    """True when a status word shows the motor has stopped (or was de-energised)."""
    if not st[9]:
        return True
    return not st[19] and (st[20] or not require_in_position)


def _check_accel(value: float) -> None:
    if not 0.01 <= value <= 99999.99:
        raise ValueError(f"acceleration must be 0.01..99999.99, got {value}")
