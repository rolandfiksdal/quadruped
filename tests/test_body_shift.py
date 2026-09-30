"""Check the sampled body-shift/forward-step experiment without hardware."""

import contextlib
import io
import unittest
from unittest.mock import patch

import body_shift_demo as demo
import leg_kinematics as kin


class BodyShiftTests(unittest.TestCase):
    def test_shift_then_forward_step(self):
        samples = []
        original_check = demo.check_and_print_pose

        def capture(t, phase, body, feet, triangle):
            # Also run the demo's limit and FK reconstruction checks on every leg.
            original_check(t, phase, body, feet, triangle)
            samples.append((t, phase, body, feet.copy(), triangle))

        original_feet = demo.feet_world.copy()
        with patch.object(demo, "check_and_print_pose", capture):
            with contextlib.redirect_stdout(io.StringIO()):
                final_feet = demo.main()

        shift = [sample for sample in samples if sample[1] == "shift"]
        swing = [sample for sample in samples if sample[1] == "swing"]
        self.assertEqual(len(shift), 101)
        self.assertEqual(len(swing), 101)
        self.assertEqual(shift[-1][2:4], swing[0][2:4])
        self.assertEqual(swing[0][0], 2.0)
        self.assertEqual(swing[-1][0], 4.0)
        for _, phase, body, feet, triangle in samples:
            for leg in ("FR", "RL", "RR"):
                self.assertEqual(feet[leg], original_feet[leg])
            if phase == "shift":
                self.assertEqual(feet, original_feet)
            else:
                self.assertEqual(body, demo.end_pos)
                self.assertAlmostEqual(
                    kin.support_margin(*triangle, body[:2]), 0.014852792733044089
                )

        start = swing[0][3]["FL"]
        midpoint = swing[50][3]["FL"]
        end = swing[-1][3]["FL"]
        self.assertEqual(start, original_feet["FL"])
        self.assertAlmostEqual(midpoint[0] - start[0], 0.02)
        self.assertAlmostEqual(midpoint[2] - start[2], 0.03)
        self.assertAlmostEqual(end[0] - start[0], 0.04)
        self.assertEqual(end[2], start[2])
        self.assertEqual(final_feet["FL"], end)
        for sample in swing:
            self.assertEqual(sample[3]["FL"][1], start[1])
            self.assertGreaterEqual(sample[3]["FL"][2], start[2])
        self.assertEqual(demo.feet_world, original_feet)


if __name__ == "__main__":
    unittest.main()
