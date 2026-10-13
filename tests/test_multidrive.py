"""Multi-drive bus behaviour: echo sync, discovery, groups, broadcasts."""

import os
import sys
import threading
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from fake_vix import FakeViX

from vix_driver import (
    CommsMode,
    DriveFaultError,
    LimitError,
    ViXBus,
    ViXCommandError,
    ViXError,
    ViXTimeoutError,
)


def make_bus(addresses=(1, 2, 3), echo=True):
    fake = FakeViX(addresses=addresses, echo=echo)
    bus = ViXBus(transport=fake, echo=echo, timeout=0.2, settle=0.02)
    return fake, bus


class EchoSyncTests(unittest.TestCase):
    def test_stray_line_before_echo_is_discarded(self):
        fake, bus = make_bus()
        fake.unit(2).vars["PT"] = 42
        fake.noise.append("*99999")  # unsolicited output from another drive
        self.assertEqual(bus.drive(2).position_target(), 42)

    def test_late_reply_from_buffered_read_does_not_leak(self):
        fake, bus = make_bus()
        d1, d2 = bus.drive(1), bus.drive(2)
        fake.unit(2).vars["PT"] = 7
        d1.energize(delay=0)
        d1.move_relative(4000, wait=False)
        with self.assertRaises(ViXTimeoutError):
            d1.position_target()  # buffered by CQ=1; its reply comes later
        # The late "*4000" arrives before the next echo and must be ignored.
        self.assertEqual(d2.position_target(), 7)
        d1.wait_for_move(timeout=2)

    def test_missing_echo_reports_broken_chain(self):
        fake, bus = make_bus(echo=False)
        bus.echo = True
        with self.assertRaises(ViXTimeoutError) as ctx:
            bus.drive(1).read("PT")
        self.assertIn("echo", str(ctx.exception))

    def test_rs485_style_no_echo(self):
        fake, bus = make_bus(echo=False)
        fake.unit(3).vars["PT"] = -5
        self.assertEqual(bus.drive(3).position_target(), -5)

    def test_concurrent_threads_do_not_mix_replies(self):
        fake, bus = make_bus()
        for a in (1, 2, 3):
            fake.unit(a).vars["PT"] = a * 100
        errors = []

        def worker(address):
            try:
                for _ in range(20):
                    value = bus.drive(address).position_target()
                    if value != address * 100:
                        errors.append((address, value))
            except Exception as exc:  # noqa: BLE001
                errors.append(exc)

        threads = [threading.Thread(target=worker, args=(a,)) for a in (1, 2, 3)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(errors, [])


class BusTests(unittest.TestCase):
    def test_discover(self):
        fake, bus = make_bus(addresses=(1, 3, 7))
        found = bus.discover(range(1, 9), timeout=0.05)
        self.assertEqual(sorted(found), [1, 3, 7])
        self.assertEqual(bus.known_addresses, {1, 3, 7})
        self.assertEqual(bus.group().addresses, [1, 3, 7])

    def test_group_requires_addresses_or_discovery(self):
        _, bus = make_bus()
        with self.assertRaises(ValueError):
            bus.group()

    def test_stop_and_kill_all(self):
        fake, bus = make_bus()
        bus.group(1, 2, 3).energize()
        bus.drive(2).jog(1)
        bus.stop_all()
        self.assertEqual(fake.sent[-1], "0S")
        self.assertFalse(fake.unit(2).moving)
        bus.kill_all()
        self.assertEqual(fake.sent[-1], "0K")

    def test_set_comms_mode_all_updates_echo(self):
        fake, bus = make_bus()
        bus.set_comms_mode_all(CommsMode.QUIET_ECHO)
        self.assertEqual(fake.sent[-1], "0W(EX,2)")
        self.assertTrue(bus.echo)
        bus.set_comms_mode_all(CommsMode.QUIET_NO_ECHO)
        self.assertFalse(bus.echo)

    def test_errors_name_the_drive(self):
        fake, bus = make_bus()
        fake.unit(2).df = {1, 17}
        with self.assertRaises(DriveFaultError) as ctx:
            bus.drive(2).check_faults()
        self.assertEqual(ctx.exception.address, 2)
        self.assertIn("drive 2", str(ctx.exception))


class GroupTests(unittest.TestCase):
    def setUp(self):
        self.fake, self.bus = make_bus()
        self.axes = self.bus.group(1, 2, 3)
        self.axes.energize()

    def test_move_relative_per_axis(self):
        self.axes.move_relative({1: 4000, 2: -2000, 3: 8000}, velocity=5, acceleration=50,
                                timeout=2)
        self.assertEqual(self.axes.positions(), {1: 4000, 2: -2000, 3: 8000})
        # Axes start back to back, after all parameters are set.
        gos = [i for i, c in enumerate(self.fake.sent) if c.endswith("G")]
        self.assertEqual([self.fake.sent[i] for i in gos], ["1G", "2G", "3G"])
        self.assertEqual(gos, list(range(gos[0], gos[0] + 3)))

    def test_move_subset_leaves_others(self):
        self.axes.move_absolute({2: 500}, timeout=2)
        self.assertEqual(self.axes.positions(), {1: 0, 2: 500, 3: 0})
        self.assertNotIn("1G", self.fake.sent)

    def test_per_axis_velocity(self):
        self.axes.move_relative(100, velocity={1: 2, 2: 3, 3: 4}, timeout=2)
        self.assertEqual([self.fake.unit(a).v for a in (1, 2, 3)], [2, 3, 4])

    def test_unknown_address_rejected(self):
        with self.assertRaises(ValueError):
            self.axes.move_relative({9: 100})

    def test_limit_on_one_axis_stops_the_others(self):
        self.fake.unit(2).limit_on_next_move = 14
        for a in (1, 3):
            self.fake.unit(a).MOVE_TIME = 5.0
        with self.assertRaises(LimitError) as ctx:
            self.axes.move_relative(1000, timeout=2)
        self.assertEqual(ctx.exception.address, 2)
        self.assertFalse(self.fake.unit(1).moving)
        self.assertFalse(self.fake.unit(3).moving)

    def test_failed_start_stops_started_axes(self):
        self.fake.unit(3).energised = False
        for a in (1, 2):
            self.fake.unit(a).MOVE_TIME = 5.0
        with self.assertRaises(ViXCommandError) as ctx:
            self.axes.move_relative(1000)
        self.assertIn("Drive de-energised", str(ctx.exception))
        self.assertFalse(self.fake.unit(1).moving)
        self.assertFalse(self.fake.unit(2).moving)

    def test_simultaneous_go_needs_whole_bus(self):
        with self.assertRaises(ViXError):
            self.axes.go(simultaneous=True)  # no discovery yet
        self.bus.discover(range(1, 5), timeout=0.05)
        with self.assertRaises(ViXError):
            self.bus.group(1, 2).go(simultaneous=True)  # would also start drive 3
        self.axes.go(simultaneous=True)
        self.assertEqual(self.fake.sent[-1], "0G")
        self.axes.wait_for_move(timeout=2)

    def test_drive_faults_summary(self):
        self.assertEqual(self.axes.drive_faults(), {})
        self.fake.unit(3).df = {1, 11}
        self.assertEqual(list(self.axes.drive_faults()), [3])

    def test_group_write(self):
        self.axes.write("CQ", 0)
        self.assertTrue(all(self.fake.unit(a).vars["CQ"] == 0 for a in (1, 2, 3)))


if __name__ == "__main__":
    unittest.main()
