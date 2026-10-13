"""Low-level EASI-V protocol helpers for Parker ViX250IM / ViX500IM drives.

Reference: "ViX250IM & ViX500IM Microstepper RS232 Indexer Drive User Guide",
Parker part no. 1600.324.01b (Feb 2004, software revision 2.0 onwards),
chapters 4 and 6.

Wire format summary
-------------------
* RS232, 8 data bits, no parity, 1 stop bit, 9600 (default) or 19200 baud,
  XON/XOFF supported.
* Every command is ``<address><command>`` with no spaces, terminated by CR.
  Address 1..255; address 0 is a global broadcast (reads to address 0 are
  ignored by the drive).
* Replies are prefixed with ``*``. A rejected command replies with a bare
  ``*E``; the reason is then available in the UF (user fault) word.
* System variable EX selects echo / unsolicited-message behaviour
  (default 3 = speak whenever, echo on, for RS232).
* With command queuing on (CQ=1, default) commands sent while a move is in
  progress are buffered until it completes, including most reads. Only
  ``K``, ``S``, ``C``, ``R(RB)``, ``R(ST)``, ``R(DF)`` and ``R(UF)`` are
  always immediate.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from enum import Enum

CR = "\r"
XON = "\x11"
XOFF = "\x13"
BELL = "\x07"  # sent by the drive on buffer overflow

GLOBAL_ADDRESS = 0
MAX_ADDRESS = 255
BAUD_RATES = (9600, 19200)

NUM_INPUTS = 5
NUM_OUTPUTS = 3


class Mode(str, Enum):
    """Controller mode (``M`` command)."""

    ABSOLUTE = "A"  # indexed moves, absolute positioning
    BIDIRECTIONAL = "B"  # continuous, velocity from the analogue input
    CONTINUOUS = "C"  # continuous move at V until stopped
    INCREMENTAL = "I"  # indexed moves, relative positioning
    POSITION = "P"  # step/direction base-drive mode (ON/OFF prohibited)


class CommsMode(int, Enum):
    """Values for system variable EX."""

    QUIET_NO_ECHO = 0  # speak when spoken to, echo off (RS485 default)
    VERBOSE_NO_ECHO = 1  # speak whenever, echo off
    QUIET_ECHO = 2  # speak when spoken to, echo on
    VERBOSE_ECHO = 3  # speak whenever, echo on (RS232 default)


@dataclass(frozen=True)
class Variable:
    name: str
    description: str
    writable: bool
    minimum: float | None = None
    maximum: float | None = None


INT32_MIN, INT32_MAX = -(2**31), 2**31 - 1

# Table 4-2, "List of System Variables". Byte forms (ST1, DF2, ...) and the
# fieldbus variables (FB, FC, FN, FP) are handled separately.
VARIABLES: dict[str, Variable] = {
    v.name: v
    for v in (
        Variable("AB", "Analogue deadband", True, 0, 255),
        Variable("AI", "Analogue input", False, -2047, 2047),
        Variable("AO", "Analogue offset", True, -2047, 2047),
        Variable("BR", "Baud rate", True),
        Variable("BU", "Program buffer usage (%)", False),
        Variable("CQ", "Command queuing (1 = wait for move, 0 = continuous)", True, 0, 1),
        Variable("DC", "Settling time damping", True, 0, 1),
        Variable("DF", "Drive fault status", False),
        Variable("EI", "Encoder input mode", True, 0, 2),
        Variable("EM", "Encoder counts per rev", True, -4200000, 4200000),
        Variable("EO", "Encoder output mode", True, 0, 2),
        Variable("EQ", "Echo queuing", True, 0, 2),
        Variable("ES", "Enable input sense", True, 0, 1),
        Variable("EX", "Comms response style and echo", True, 0, 3),
        Variable("HF", "Home final velocity (rps)", True, 0.001, 5.0),
        Variable("IC", "I/O configuration", True, 0, 8191),
        Variable("IP", "In position flag", False),
        Variable("IT", "In position time (ms)", True, 1, 500),
        Variable("MS", "Motor standby current (%)", True, 10, 100),
        Variable("MV", "Moving flag", False),
        Variable("PA", "Position actual (write 0 only)", True, 0, 0),
        Variable("PE", "Position error (write 0 only)", True, 0, 0),
        Variable("PF", "Position following", True, INT32_MIN, INT32_MAX),
        Variable("PI", "Position incremental", True, INT32_MIN, INT32_MAX),
        Variable("PM", "Position master (write sets modulus)", True, INT32_MIN, INT32_MAX),
        Variable("PR", "Position registration", False),
        Variable("PS", "Position secondary", False),
        Variable("PT", "Position target", True, INT32_MIN, INT32_MAX),
        Variable("RB", "Ready/busy flag (1 = busy)", False),
        Variable("RM", "Registration move flag", False),
        Variable("RV", "Software revision", False),
        Variable("SC", "S-curve accel/decel", True, 0, 1),
        Variable("SN", "Serial number", False),
        Variable("ST", "Status", False),
        Variable("TT", "Trigger timeout (s)", True, 0, 65),
        Variable("UF", "User fault status", False),
    )
}

# Bit-field variables may be read whole (ST) or a byte at a time (ST1..ST4).
BITFIELD_VARIABLES = ("ST", "DF", "UF")

# Table 4-5, "Status Bits Description".
STATUS_BITS: dict[int, str] = {
    1: "Command processing paused",
    2: "Looping (command executing)",
    3: "Wait for trigger",
    4: "Running program",
    5: "Going home",
    6: "Waiting for delay timeout",
    7: "Registration in progress",
    8: "Last trigger command timed out",
    9: "Motor energised",
    11: "Event triggered",
    12: "Input in LSEL not matching label",
    13: "-ve limit seen during last move",
    14: "+ve limit seen during last move",
    17: "Executing a position maintenance move",
    18: "Possible stall",
    19: "Moving",
    20: "Stationary (in position)",
    21: "No registration signal seen in registration window",
    22: "Cannot stop within the defined registration distance",
    25: "Direction of motion negative",
    27: "Following enabled",
    28: "STOP input active",
    29: "Load mounted encoder enabled",
    30: "Scaling enabled",
    31: "Command input inverted",
}

# Table 4-6, "Drive Fault Bit Description".
DRIVE_FAULT_BITS: dict[int, str] = {
    1: "Composite fault",
    2: "+/-15V supply rail",
    3: "Motor HV under-voltage",
    4: "Motor HV over-voltage",
    6: "V I/O over-voltage",
    7: "Encoder/auxiliary 5V under-voltage",
    8: "Impending power loss (24V logic supply)",
    11: "Motor over temperature",
    12: "Ambient over temperature",
    13: "Drive over temperature",
    14: "Incompatible firmware version",
    15: "Unrecognised power stage",
    16: "Controller diagnostic failure",
    17: "Output stage over current",
    18: "Output driver over current",
    19: "Tracking limit exceeded: stall condition",
    21: "Drive disabled (check enable input and ES)",
    25: "Watchdog",
    32: "CAN I/O error",
}

# Drive faults needing a power cycle or repair (types "T" and "SLEEP").
TERMINAL_DRIVE_FAULTS = frozenset({2, 7, 8, 14, 15, 16, 25})

# Table 4-7, "User Fault Bit Description".
USER_FAULT_BITS: dict[int, str] = {
    1: "Value is out of range",
    2: "Incorrect command syntax",
    3: "Last label already in use",
    4: "Label of this name not defined",
    5: "Missing Z pulse when homing",
    6: "Homing failed - no signal detected",
    7: "Home signal too narrow",
    8: "Drive de-energised",
    9: "Cannot relate END statement to a label",
    10: "Program memory buffer full",
    11: "No more motion profiles available",
    12: "No more sequence labels available",
    13: "End of travel limit hit",
    14: "Still moving",
    15: "Deceleration error",
    16: "Transmit buffer overflow",
    17: "User program nesting overflow",
    18: "Cannot use an undefined profile",
    19: "Drive not ready",
    22: "Save error",
    23: "Command not supported by this product",
    24: "Fieldbus error",
    25: "Input buffer overflow",
    27: "Command not actioned",
    28: "Scale distance is non-integer",
}


class ProtocolError(Exception):
    """A reply from the drive could not be parsed."""


@dataclass(frozen=True)
class BitReport:
    """A decoded bit pattern (ST / DF / UF words, IS inputs).

    Bits are numbered from 1, matching the manual: bit 1 is the leftmost
    character of the reply. A byte read (e.g. ``ST2``) is decoded with
    ``first_bit`` set so bit numbers stay absolute.
    """

    raw: str
    bits: tuple[bool, ...]
    first_bit: int = 1
    names: dict[int, str] | None = None

    def __getitem__(self, bit: int) -> bool:
        index = bit - self.first_bit
        if not 0 <= index < len(self.bits):
            raise IndexError(f"bit {bit} not present in this report")
        return self.bits[index]

    @property
    def set_bits(self) -> list[int]:
        return [i + self.first_bit for i, b in enumerate(self.bits) if b]

    @property
    def active(self) -> list[str]:
        """Descriptions of every set bit (unknown bits shown by number)."""
        names = self.names or {}
        return [names.get(b, f"bit {b}") for b in self.set_bits]

    def __bool__(self) -> bool:
        return any(self.bits)

    def __str__(self) -> str:
        return ", ".join(self.active) if self else "none"


def format_command(address: int, body: str) -> str:
    """Build a wire-ready command string: ``<address><body>\\r``."""
    if not GLOBAL_ADDRESS <= address <= MAX_ADDRESS:
        raise ValueError(f"address must be 0..{MAX_ADDRESS}, got {address}")
    if not body or any(c.isspace() for c in body):
        raise ValueError(f"command must be non-empty and contain no whitespace: {body!r}")
    return f"{address}{body}{CR}"


def format_value(value: float | int | str) -> str:
    """Format a parameter value without exponent notation or a redundant ``.0``."""
    if isinstance(value, bool):
        return str(int(value))
    if isinstance(value, float):
        if value.is_integer():
            return str(int(value))
        return f"{value:.6f}".rstrip("0")
    return str(value)


def is_error(line: str) -> bool:
    return line.startswith("*E")


def strip_reply(line: str) -> str:
    """Remove the leading ``*`` from a reply line."""
    if not line.startswith("*"):
        raise ProtocolError(f"reply does not start with '*': {line!r}")
    return line[1:].strip()


_NUMBER = re.compile(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)")


def _to_number(text: str) -> int | float:
    return float(text) if "." in text else int(text)


def parse_number(line: str) -> int | float:
    """Parse a numeric reply such as ``*4000``, ``*-12.5`` or ``*120.0 120.0``.

    When the reply holds several numbers (``A`` reports accel and decel),
    the first is returned; use :func:`parse_numbers` for all of them.
    """
    values = parse_numbers(line)
    return values[0]


def parse_numbers(line: str) -> list[int | float]:
    text = strip_reply(line)
    tokens = text.split()
    if not tokens or not all(_NUMBER.fullmatch(t) for t in tokens):
        raise ProtocolError(f"expected a numeric reply, got {line!r}")
    return [_to_number(t) for t in tokens]


def parse_bits(
    line: str, names: dict[int, str] | None = None, first_bit: int = 1
) -> BitReport:
    """Parse a bit-pattern reply such as ``*0000_0000_1000_...`` or ``*01100``."""
    text = strip_reply(line).replace("_", "")
    if not text or set(text) - {"0", "1"}:
        raise ProtocolError(f"expected a bit pattern reply, got {line!r}")
    return BitReport(
        raw=line, bits=tuple(c == "1" for c in text), first_bit=first_bit, names=names
    )


def parse_mode(line: str) -> Mode:
    """Parse a mode reply such as ``*MA``."""
    text = strip_reply(line)
    if len(text) != 2 or text[0] != "M":
        raise ProtocolError(f"expected a mode reply, got {line!r}")
    try:
        return Mode(text[1])
    except ValueError:
        raise ProtocolError(f"unknown mode in reply {line!r}") from None


_FIELD = re.compile(r"([A-Z/]+)([+-]?[\d.]+|[+-])")


def parse_fields(line: str) -> dict[str, str]:
    """Parse a settings report such as ``*LM1 TP1 M0 LD100.0`` into a dict.

    Values are kept as strings because some are signs (``E-``) rather than
    numbers. Used for LIMITS, HOME, PROFILE0 and LSEL reports.
    """
    return dict(_FIELD.findall(strip_reply(line)))
