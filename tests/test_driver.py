"""Tests against a simulated ViX IM drive; no hardware or pyserial needed.

Run with:  python -m unittest discover -s tests
"""

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from vix_driver import (
    DriveFaultError,
    LimitError,
    Mode,
    ProtocolError,
    ViXBus,
    ViXCommandError,
    ViXError,
    ViXTimeoutError,
)
from vix_driver import protocol as p

from fake_vix import FakeViX, Unit, pattern


def make_bus(**kwargs):
    fake = FakeViX(**kwargs)
    bus = ViXBus(transport=fake, echo=fake.echo, timeout=0.2, settle=0.02)
    return fake, bus


class ProtocolTests(unittest.TestCase):
    def test_format_command(self):
        self.assertEqual(p.format_command(1, "R(PA)"), "1R(PA)\r")
        self.assertEqual(p.format_command(12, "G"), "12G\r")
        with self.assertRaises(ValueError):
            p.format_command(256, "G")
        with self.assertRaises(ValueError):
            p.format_command(1, "W(IT, 4)")

    def test_parse_numbers(self):
        self.assertEqual(p.parse_number("*4000"), 4000)
        self.assertEqual(p.parse_number("*-12"), -12)
        self.assertEqual(p.parse_number("*25.0"), 25.0)
        self.assertEqual(p.parse_numbers("*120.0 120.0"), [120.0, 120.0])
        with self.assertRaises(ProtocolError):
            p.parse_number("*MA")

    def test_parse_bits_numbering(self):
        report = p.parse_bits("*" + pattern(1, 19, 21), p.DRIVE_FAULT_BITS)
        self.assertEqual(report.set_bits, [1, 19, 21])
        self.assertIn("Tracking limit exceeded: stall condition", report.active)

    def test_parse_byte_keeps_absolute_bit_numbers(self):
        report = p.parse_bits("*0010_0000", p.STATUS_BITS, first_bit=17)
        self.assertEqual(report.set_bits, [19])
        self.assertEqual(report.active, ["Moving"])

    def test_parse_fields(self):
        self.assertEqual(p.parse_fields("*LM1 TP1 M0 LD100.0"),
                         {"LM": "1", "TP": "1", "M": "0", "LD": "100.0"})
        self.assertEqual(p.parse_fields("* AR1 E- TP1 V+10.00 A10.0 M0")["E"], "-")

    def test_format_value(self):
        self.assertEqual(p.format_value(5.0), "5")
        self.assertEqual(p.format_value(0.001), "0.001")
        self.assertEqual(p.format_value(1e-5), "0.00001")


class DriveTests(unittest.TestCase):
    def setUp(self):
        self.fake, self.bus = make_bus()
        self.drive = self.bus.drive(1)

    def test_reads_with_echo(self):
        self.assertEqual(self.drive.position_target(), 0)
        self.assertEqual(self.drive.revision(), "2.1b")
        self.assertEqual(self.fake.sent[0], "1R(PT)")

    def test_reads_without_echo_cr_only(self):
        _, bus = make_bus(echo=False, eol="\r")
        self.assertEqual(bus.drive(1).analogue_input(), 512)

    def test_write_validation(self):
        self.drive.write("IT", 50)
        self.assertEqual(self.fake.sent[-1], "1W(IT,50)")
        with self.assertRaises(ValueError):
            self.drive.write("MV", 0)  # read-only
        with self.assertRaises(ValueError):
            self.drive.write("PA", 5)  # only 0 allowed
        with self.assertRaises(ValueError):
            self.drive.write("BR", 115200)

    def test_rejection_reports_user_fault_reason(self):
        with self.assertRaises(ViXCommandError) as ctx:
            self.drive.go()  # not energised
        self.assertIn("Drive de-energised", ctx.exception.user_faults.active)
        self.assertIn("Drive de-energised", str(ctx.exception))

    def test_motion_parameters(self):
        self.drive.set_motion(velocity=5, acceleration=50)
        self.assertEqual(self.fake.sent[-2:], ["1A50", "1V5"])
        self.drive.set_motion(acceleration=20, deceleration=30, distance=-800)
        self.assertEqual(self.fake.sent[-3:], ["1AA20", "1AD30", "1D-800"])
        self.assertEqual(self.drive.velocity(), 5.0)
        self.assertEqual(self.drive.acceleration(), 20.0)
        self.assertEqual(self.drive.distance(), -800)
        with self.assertRaises(ValueError):
            self.drive.set_motion(acceleration=0)

    def test_velocity_over_motor_max_rejected_by_drive(self):
        with self.assertRaises(ViXCommandError) as ctx:
            self.drive.set_motion(velocity=60)
        self.assertEqual(ctx.exception.user_faults.active, ["Value is out of range"])

    def test_mode(self):
        self.drive.set_mode(Mode.ABSOLUTE)
        self.assertEqual(self.drive.get_mode(), Mode.ABSOLUTE)
        self.drive.set_mode("C")
        self.assertEqual(self.fake.sent[-1], "1MC")

    def test_energize(self):
        self.drive.energize(delay=0)
        self.assertTrue(self.drive.is_energized())
        self.drive.de_energize()
        self.assertFalse(self.drive.is_energized())

    def test_energize_reports_drive_fault(self):
        self.fake.unit(1).df = {1, 21}
        with self.assertRaises(DriveFaultError) as ctx:
            self.drive.energize(delay=0)
        self.assertEqual(ctx.exception.faults.set_bits, [1, 21])
        self.assertFalse(ctx.exception.terminal)

    def test_move_relative_waits_for_completion(self):
        self.drive.energize(delay=0)
        start = time.monotonic()
        self.drive.move_relative(4000, velocity=5, acceleration=50, timeout=2)
        self.assertGreaterEqual(time.monotonic() - start, Unit.MOVE_TIME)
        self.assertEqual(self.drive.position_target(), 4000)
        self.drive.move_relative(-1000)
        self.assertEqual(self.drive.position_target(), 3000)
        self.assertIn("1MI", self.fake.sent)

    def test_move_absolute(self):
        self.drive.energize(delay=0)
        self.drive.move_absolute(1234)
        self.assertEqual(self.drive.position_target(), 1234)
        self.assertEqual(self.fake.sent[self.fake.sent.index("1MA") + 1], "1D1234")

    def test_reads_are_buffered_mid_move_with_queuing(self):
        self.drive.energize(delay=0)
        self.drive.move_relative(100, wait=False)
        self.assertTrue(self.drive.is_moving())  # R(ST) is always immediate
        with self.assertRaises(ViXTimeoutError) as ctx:
            self.drive.position_target()
        self.assertIn("CQ=1", str(ctx.exception))
        self.drive.wait_for_move(timeout=2)

    def test_reads_mid_move_without_queuing(self):
        self.drive.energize(delay=0)
        self.drive.set_command_queuing(False)
        self.drive.move_relative(100, wait=False)
        mid = self.drive.position_target()  # answered during the move: on the way
        self.assertTrue(0 <= mid < 100, mid)
        self.drive.wait_for_move(timeout=2)
        self.assertEqual(self.drive.position_target(), 100)

    def test_wait_for_move_timeout(self):
        self.drive.energize(delay=0)
        self.drive.jog(2, positive=False)
        self.assertIn("1H-", self.fake.sent)
        with self.assertRaises(ViXTimeoutError):
            self.drive.wait_for_move(timeout=0.1)
        self.drive.stop()
        self.drive.wait_for_move(timeout=1)

    def test_limit_hit_raises(self):
        self.drive.energize(delay=0)
        self.fake.unit(1).limit_on_next_move = 14
        with self.assertRaises(LimitError) as ctx:
            self.drive.move_relative(500)
        self.assertIn("+ve", str(ctx.exception))

    def test_fault_during_move(self):
        self.drive.energize(delay=0)
        self.drive.move_relative(500, wait=False)
        self.fake.unit(1).energised = False
        self.fake.unit(1).df = {1, 17}
        with self.assertRaises(DriveFaultError):
            self.drive.wait_for_move(timeout=1)

    def test_homing_and_limits_config(self):
        self.drive.configure_homing(-15, 100, edge_positive=False, normally_closed=True, mode=1)
        self.assertEqual(self.fake.sent[-1], "1HOME1(-,1,-15,100,1)")
        self.assertEqual(self.drive.homing_config()["V"], "+10.00")
        self.drive.configure_limits(positive_enabled=False, deceleration=100)
        self.assertEqual(self.fake.sent[-1], "1LIMITS(1,1,0,100)")
        self.drive.configure_limits(positive_enabled=False, negative_enabled=False,
                                    normally_closed=False)
        self.assertEqual(self.fake.sent[-1], "1LIMITS(3,0,0)")
        self.assertEqual(self.drive.limits_config()["LD"], "200.0")

    def test_io(self):
        self.assertEqual(self.drive.inputs(), (False, True, True, False, False))
        self.drive.set_outputs(True, None, False)
        self.assertEqual(self.fake.sent[-1], "1O(1X0)")
        self.drive.set_output(3, True)
        self.assertEqual(self.fake.sent[-1], "1O(XX1)")
        with self.assertRaises(ValueError):
            self.drive.set_output(4, True)

    def test_profiles(self):
        self.drive.define_profile(2, 150, 200, 4800, 45)
        self.assertEqual(self.fake.sent[-1], "1PROFILE2(150,200,4800,45)")
        self.drive.use_profile(2)
        self.assertEqual(self.fake.sent[-1], "1USE(2)")
        with self.assertRaises(ValueError):
            self.drive.define_profile(0, 1, 1, 1, 1)

    def test_motor_formatting(self):
        self.drive.set_motor(255, 4.54, 4000, 1500, 0, 0.9, 2.5, timeout=0.02)
        self.assertEqual(self.fake.sent[-1], "1MOTOR(255,4.5,4000,1500,0,0.9,2.5)")
        with self.assertRaises(ValueError):
            self.drive.set_motor(255, 6.0, 4000, 1500, 0, 0.9, 2.5)

    def test_status_report_and_list(self):
        self.assertEqual(self.drive.status_report(timeout=0.2)[-1], "AXIS: READY")
        self.assertEqual(self.drive.list_program(timeout=0.2), ["1START:", "1G", "1END"])

    def test_broadcast(self):
        with self.assertRaises(ValueError):
            self.bus.broadcast().read("PT")
        self.bus.broadcast().kill()
        self.assertEqual(self.fake.sent[-1], "0K")

    def test_multiple_drives(self):
        fake, bus = make_bus(addresses=(1, 2))
        fake.unit(2).vars["PT"] = 7
        self.assertEqual(bus.drive(2).position_target(), 7)
        self.assertEqual(fake.sent[-1], "2R(PT)")

    def test_timeout(self):
        self.fake.silent = True
        with self.assertRaises(ViXTimeoutError):
            self.drive.read("PT")

    def test_bitfield_read_rejected(self):
        with self.assertRaises(ValueError):
            self.drive.read("ST")

    def test_energize_silent_drive(self):
        self.fake.silent = True
        with self.assertRaises(ViXError):
            self.drive.energize(delay=0)


if __name__ == "__main__":
    unittest.main()
