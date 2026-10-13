"""Coordinated commands for several drives on one bus.

Example::

    with ViXBus("/dev/ttyUSB0") as bus:
        print(bus.discover())                 # {1: '2.1b', 2: '2.1b', 3: '2.1b'}
        axes = bus.group(1, 2, 3)
        axes.energize()
        axes.move_relative({1: 4000, 2: -2000, 3: 8000}, velocity=5, acceleration=50)
        print(axes.positions())               # {1: 4000, 2: -2000, 3: 8000}
"""

from __future__ import annotations

import logging
import time
from collections.abc import Iterable, Iterator, Mapping
from typing import TypeVar

from . import protocol as p
from .drive import ViXDrive, ViXError, ViXTimeoutError, move_finished
from .protocol import BitReport, Mode

log = logging.getLogger(__name__)

T = TypeVar("T")
PerAxis = T | Mapping[int, T]


class DriveGroup:
    """Several drives on the same :class:`~vix_driver.ViXBus`, addressed together.

    Commands go to each drive in turn (the bus carries one transaction at a
    time), so axes start a few milliseconds apart. :meth:`go` with
    ``simultaneous=True`` starts them together with a global ``0G``.

    Arguments that accept ``PerAxis`` values take either one value for every
    axis or a ``{address: value}`` mapping.
    """

    def __init__(self, drives: Iterable[ViXDrive]):
        self._drives = {d.address: d for d in drives}
        if not self._drives:
            raise ValueError("a group needs at least one drive")
        if p.GLOBAL_ADDRESS in self._drives:
            raise ValueError("address 0 is the broadcast address, not a drive")
        buses = {id(d.bus) for d in self._drives.values()}
        if len(buses) != 1:
            raise ValueError("all drives in a group must share one bus")
        self.bus = next(iter(self._drives.values())).bus

    def __repr__(self) -> str:
        return f"DriveGroup({sorted(self._drives)})"

    def __iter__(self) -> Iterator[ViXDrive]:
        return iter(self._drives.values())

    def __len__(self) -> int:
        return len(self._drives)

    def __getitem__(self, address: int) -> ViXDrive:
        return self._drives[address]

    @property
    def addresses(self) -> list[int]:
        return list(self._drives)

    def _value(self, value: PerAxis[T] | None, address: int) -> T | None:
        if isinstance(value, Mapping):
            return value.get(address)
        return value

    def _targets(self, values: PerAxis[int]) -> dict[int, int]:
        if isinstance(values, Mapping):
            unknown = set(values) - set(self._drives)
            if unknown:
                raise ValueError(f"addresses not in group: {sorted(unknown)}")
            return dict(values)
        return {a: values for a in self._drives}

    # -- power -------------------------------------------------------------

    def energize(self, check: bool = True) -> None:
        """Energise every drive, then check them all."""
        for drive in self:
            drive.energize(check=False)
        if check:
            time.sleep(0.2)
            for drive in self:
                if not drive.is_energized():
                    drive.check_faults()
                    raise ViXError(f"drive {drive.address}: did not energise")

    def de_energize(self) -> None:
        for drive in self:
            drive.de_energize()

    # -- motion ------------------------------------------------------------

    def set_motion(
        self,
        velocity: PerAxis[float] | None = None,
        acceleration: PerAxis[float] | None = None,
        deceleration: PerAxis[float] | None = None,
    ) -> None:
        for a, drive in self._drives.items():
            drive.set_motion(
                velocity=self._value(velocity, a),
                acceleration=self._value(acceleration, a),
                deceleration=self._value(deceleration, a),
            )

    def move_relative(
        self,
        distances: PerAxis[int],
        velocity: PerAxis[float] | None = None,
        acceleration: PerAxis[float] | None = None,
        *,
        wait: bool = True,
        timeout: float | None = None,
    ) -> None:
        """Move each axis by its distance (mode I). Axes not in ``distances`` stay put."""
        self._move(Mode.INCREMENTAL, distances, velocity, acceleration, wait, timeout)

    def move_absolute(
        self,
        positions: PerAxis[int],
        velocity: PerAxis[float] | None = None,
        acceleration: PerAxis[float] | None = None,
        *,
        wait: bool = True,
        timeout: float | None = None,
    ) -> None:
        """Move each axis to its position (mode A). Axes not in ``positions`` stay put."""
        self._move(Mode.ABSOLUTE, positions, velocity, acceleration, wait, timeout)

    def _move(self, mode, targets, velocity, acceleration, wait, timeout) -> None:
        targets = self._targets(targets)
        moving = [self._drives[a] for a in targets]
        for drive in moving:
            drive.set_mode(mode)
            drive.set_motion(
                velocity=self._value(velocity, drive.address),
                acceleration=self._value(acceleration, drive.address),
                distance=targets[drive.address],
            )
        self._start(moving)
        if wait:
            self.wait_for_move(timeout, drives=moving)

    def _start(self, drives: list[ViXDrive]) -> None:
        started: list[ViXDrive] = []
        for drive in drives:
            try:
                drive.go()
            except ViXError:
                self._stop(started)
                raise
            started.append(drive)

    def go(self, simultaneous: bool = False) -> None:
        """Start every drive with its current parameters.

        ``simultaneous=True`` sends one global ``0G``, which starts *every*
        drive on the bus. It is only allowed when this group contains all
        the drives found by ``bus.discover()``.
        """
        if not simultaneous:
            self._start(list(self))
            return
        known = self.bus.known_addresses
        if known is None:
            raise ViXError("call bus.discover() before a simultaneous go")
        if set(self._drives) != set(known):
            raise ViXError(
                f"0G would also start drives {sorted(set(known) - set(self._drives))}; "
                "a simultaneous go needs a group of every drive on the bus"
            )
        self.bus.command(p.GLOBAL_ADDRESS, "G")

    def stop(self) -> None:
        """Controlled stop (``S``) on every drive in the group."""
        self._stop(list(self))

    def kill(self) -> None:
        """Kill (``K``) every drive in the group."""
        for drive in self:
            try:
                drive.kill()
            except ViXError as exc:
                log.error("kill failed on drive %d: %s", drive.address, exc)

    def _stop(self, drives: Iterable[ViXDrive]) -> None:
        for drive in drives:
            try:
                drive.stop()
            except ViXError as exc:
                log.error("stop failed on drive %d: %s", drive.address, exc)

    def wait_for_move(
        self,
        timeout: float | None = None,
        poll: float = 0.05,
        require_in_position: bool = True,
        stop_on_error: bool = True,
        drives: Iterable[ViXDrive] | None = None,
    ) -> dict[int, BitReport]:
        """Wait until every drive has stopped; returns the final status words.

        If one drive faults, hits a limit or is de-energised, the others
        are stopped (``stop_on_error``) and the error is raised. On timeout
        the drives are left running.
        """
        pending = {d.address: d for d in (drives if drives is not None else self)}
        finished: dict[int, BitReport] = {}
        deadline = None if timeout is None else time.monotonic() + timeout
        time.sleep(poll)
        while pending:
            for address, drive in list(pending.items()):
                st = drive.status()
                if not move_finished(st, require_in_position):
                    continue
                try:
                    drive.check_move_result(st)
                except ViXError:
                    if stop_on_error:
                        self._stop(d for a, d in pending.items() if a != address)
                    raise
                finished[address] = st
                del pending[address]
            if not pending:
                break
            if deadline is not None and time.monotonic() > deadline:
                raise ViXTimeoutError(
                    f"drives {sorted(pending)} did not finish within {timeout} s"
                )
            time.sleep(poll)
        return finished

    # -- monitoring --------------------------------------------------------

    def positions(self) -> dict[int, int]:
        """Commanded position (PT) of each drive."""
        return {a: d.position_target() for a, d in self._drives.items()}

    def status(self) -> dict[int, BitReport]:
        return {a: d.status() for a, d in self._drives.items()}

    def drive_faults(self) -> dict[int, BitReport]:
        """DF word of each drive with an active fault (empty dict = all healthy)."""
        faults = {a: d.drive_faults() for a, d in self._drives.items()}
        return {a: f for a, f in faults.items() if f}

    def check_faults(self) -> None:
        """Raise :class:`DriveFaultError` for the first drive with a fault."""
        for drive in self:
            drive.check_faults()

    def is_moving(self) -> bool:
        return any(d.is_moving() for d in self)

    # -- configuration -----------------------------------------------------

    def write(self, name: str, value: PerAxis[float]) -> None:
        """Write a system variable on every drive (or per-axis values)."""
        for a, drive in self._drives.items():
            v = self._value(value, a)
            if v is not None:
                drive.write(name, v)

    def save(self) -> None:
        """Save every drive's configuration (one ``SV`` each, then one wait)."""
        for drive in self:
            drive.save(wait=0)
        time.sleep(2.0)
