"""A simulated RS232 daisy chain of ViX IM drives, for tests."""

import time


def pattern(*bits: int, width: int = 32, first: int = 1) -> str:
    chars = ["1" if first + i in bits else "0" for i in range(width)]
    return "_".join("".join(chars[i : i + 4]) for i in range(0, width, 4))


IMMEDIATE = {"K", "S", "C", "R(ST)", "R(DF)", "R(UF)", "R(RB)"}


class Unit:
    """State of one simulated drive."""

    MOVE_TIME = 0.15

    def __init__(self, address):
        self.address = address
        self.vars = {"PT": 0, "PA": 0, "PE": 0, "PI": 0, "AI": 512, "RV": "2.1b",
                     "CQ": 1, "RB": 0, "IT": 10, "EX": 3, "EM": 4000}
        self.mode = "I"
        self.v, self.aa, self.ad, self.d = 1.0, 10.0, 10.0, 4000
        self.energised = False
        self.df: set[int] = set()
        self.uf: set[int] = set()
        self.st_extra: set[int] = set()
        self.move_end = 0.0
        self.move_from = 0  # PT at the start of the current move
        self.move_target = 0
        self.move_started = None
        self.continuous = False
        self.limit_on_next_move = None
        self.inputs = "01100"
        self.resolution = 4000
        self.encoder_error = 0  # PA - PT: steps lost by the motor (encoder model)
        self.posmain = False
        self.loadenc = False  # ST bit 29; positions/velocities then in EM counts
        self.home_end = 0.0

    @property
    def moving(self):
        return self.continuous or time.monotonic() < max(self.move_end, self.home_end)

    def trajectory(self) -> int:
        """PT as the drive reports it: on the way to the target during a move."""
        now = time.monotonic()
        if self.move_started is None or now >= self.move_end or self.vars["PT"] != self.move_target:
            return self.vars["PT"]
        done = (now - self.move_started) / max(self.move_end - self.move_started, 1e-9)
        return int(round(self.move_from + done * (self.move_target - self.move_from)))

    @property
    def homing(self):
        return time.monotonic() < self.home_end

    def status_bits(self):
        bits = set(self.st_extra)
        if self.homing:
            bits.update((5, 19))
        if self.energised:
            bits.add(9)
        if self.loadenc:
            bits.add(29)
        bits.add(19 if self.moving else 20)
        return bits

    def handle(self, body, emit, broadcast=False):
        """Process one command. ``emit`` sends a reply line. Returns False if
        the command was buffered (CQ=1 during a move)."""
        if self.moving and self.vars["CQ"] == 1 and body not in IMMEDIATE:
            return False

        def error(uf_bit):
            self.uf.add(uf_bit)
            emit("*E")

        if body.startswith("R("):
            if broadcast:
                return True
            name = body[2:-1]
            if name == "ST":
                emit("*" + pattern(*self.status_bits()))
            elif name in ("ST1", "ST2", "ST3", "ST4"):  # one status byte, bit 8n+1 leftmost
                first = 8 * (int(name[2]) - 1) + 1
                emit("*" + "".join("1" if b in self.status_bits() else "0" for b in range(first, first + 8)))
            elif name == "DF":
                emit("*" + pattern(*self.df))
            elif name == "UF":
                emit("*" + pattern(*self.uf))
                self.uf.clear()
            elif name == "PT":  # trajectory position: moves linearly during MOVE_TIME
                emit(f"*{self.trajectory()}")
            elif name == "PA":
                emit(f"*{self.trajectory() + self.encoder_error}")
            elif name == "UF1":
                emit("*" + "".join("1" if b in self.uf else "0" for b in range(1, 9)))
            else:
                emit(f"*{self.vars[name]}")
        elif body.startswith("W("):
            name, value = body[2:-1].split(",")
            self.vars[name] = float(value) if "." in value else int(value)
            if name == "PA" and self.vars["PA"] == 0:
                for v in ("PT", "PE", "PF"):
                    self.vars[v] = 0
                self.encoder_error = 0
        elif body == "M":
            emit(f"*M{self.mode}")
        elif body[0] == "M" and len(body) == 2:
            self.mode = body[1]
        elif body == "V":
            emit(f"*{self.v:.2f}")
        elif body.startswith("V"):
            val = float(body[1:])
            if val > 50:
                return error(1) or True
            self.v = val
        elif body == "A":
            emit(f"*{self.aa:.1f} {self.ad:.1f}")
        elif body == "AA":
            emit(f"*{self.aa:.1f}")
        elif body == "AD":
            emit(f"*{self.ad:.1f}")
        elif body.startswith("AA"):
            self.aa = float(body[2:])
        elif body.startswith("AD"):
            self.ad = float(body[2:])
        elif body.startswith("A"):
            self.aa = self.ad = float(body[1:])
        elif body == "D":
            emit(f"*{self.d}")
        elif body.startswith("D"):
            self.d = int(body[1:])
        elif body == "G":
            if not self.energised:
                return error(8) or True
            self.move_started = time.monotonic()
            if self.mode == "C":
                self.continuous = True
                return True
            target = self.d if self.mode == "A" else self.vars["PT"] + self.d
            self.move_from, self.move_target = self.vars["PT"], target
            self.vars["PT"] = target
            self.move_end = time.monotonic() + self.MOVE_TIME
            self.st_extra.discard(17)
            if self.posmain:  # a POSMAIN correction latches bit 17 until the next G/GH/ON
                self.st_extra.add(17)
            self.st_extra -= {13, 14}  # "limit seen during last move" resets per move
            if self.limit_on_next_move:
                self.st_extra.add(self.limit_on_next_move)
                self.limit_on_next_move = None
        elif body == "GH":
            if not self.energised:
                return error(8) or True
            self.home_end = time.monotonic() + self.MOVE_TIME
            self.vars["PT"] = 0
            self.encoder_error = 0
        elif body in ("S", "K"):
            self.continuous = False
            self.move_end = 0
        elif body == "ON":
            self.st_extra.discard(17)
            self.uf.clear()
            if not self.df:
                self.energised = True
        elif body == "OFF":
            self.energised = False
        elif body == "IS":
            emit("*" + self.inputs)
        elif body == "POSMAIN":
            emit(f"*AR{int(self.posmain)} DB1 O0 S2")  # format of a real ViX500IM, fw 3.1
        elif body == "LIMITS":
            emit("*LM0 TP1 M0 LD200.0")
        elif body == "HOME":
            emit("* AR1 E- TP1 V+10.00 A10.0 M0")
        elif body == "STATUS":
            emit("*ViX500IM-Stepper Copyright 2003 Parker-Hannifin")
            emit(f"*MOTOR TYPE ............ 255            RESOLUTION .......... {self.resolution}")
            emit("*AXIS: READY")
        elif body == "LIST(ALL)":
            for line in (f"{self.address}START:", f"{self.address}G", f"{self.address}END"):
                emit(line)
        return True


class FakeViX:
    """Serial transport simulating several drives on one RS232 daisy chain.

    With ``echo`` set, each command comes back once (round the ring) before
    any reply, as on a real chain.
    """

    def __init__(self, addresses=(1,), echo=True, eol="\r\n"):
        self.units = {a: Unit(a) for a in addresses}
        self.echo = echo
        self.eol = eol
        self.sent: list[str] = []
        self.out = b""
        self.silent = False
        self.noise: list[str] = []  # lines that arrive just before the next echo
        self._late: list[str] = []  # replies to buffered queries, delivered later

    def unit(self, address) -> Unit:
        return self.units[address]

    # Transport interface
    def write(self, data: bytes):
        cmd = data.decode().rstrip("\r")
        self.sent.append(cmd)
        if self.silent:
            return len(data)
        for line in self.noise + self._late:
            self._emit(line)
        self.noise, self._late = [], []
        if self.echo:
            self._emit(cmd)
        self._handle(cmd)
        return len(data)

    def read(self, size=1):
        chunk, self.out = self.out[:size], self.out[size:]
        return chunk

    def reset_input_buffer(self):
        self.out = b""

    def close(self):
        pass

    def _emit(self, line):
        self.out += (line + self.eol).encode()

    def _handle(self, cmd):
        i = 0
        while i < len(cmd) and cmd[i].isdigit():
            i += 1
        addr, body = int(cmd[:i]), cmd[i:]
        if addr == 0:
            for unit in self.units.values():
                unit.handle(body, lambda line: None, broadcast=True)
            return
        unit = self.units.get(addr)
        if unit is None:
            return
        buffered: list[str] = []
        if not unit.handle(body, self._emit):
            # Buffered until the move ends; deliver its reply late.
            unit.vars["CQ"] = 0
            unit.handle(body, buffered.append)
            unit.vars["CQ"] = 1
            self._late.extend(buffered)
