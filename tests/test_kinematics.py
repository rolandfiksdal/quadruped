"""Offline geometry checks. Run from the repository root with unittest discover."""

import itertools
import math
import unittest

import leg_kinematics as model


class KinematicsTests(unittest.TestCase):
    def assert_vector_close(self, actual, expected, tolerance=1e-10):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, delta=tolerance)

    def test_3d_round_trips_on_selected_branch(self):
        for leg in model.haa_positions:
            for q0, q1, q2 in itertools.product(
                (-0.45, -0.2, 0, 0.2, 0.45),
                (-1.3, -0.7, -0.1, 0.5),
                (0.9, 1.2, 1.8, 2.3),
            ):
                # The inverse deliberately selects a foot below the planar hip.
                if model.forward_kinematics(q1, q2)[1] >= 0:
                    continue
                with self.subTest(leg=leg, angles=(q0, q1, q2)):
                    point = model.forward_kinematics_3d(leg, q0, q1, q2)
                    self.assert_vector_close(
                        model.inverse_kinematics_3d(leg, *point), (q0, q1, q2)
                    )

    def test_rotation_and_left_right_symmetry(self):
        self.assert_vector_close(model.rotate_about_x((0, 1, 0), math.pi / 2), (0, 0, 1))
        for q0 in (-0.3, 0, 0.3):
            left = model.forward_kinematics_3d("FL", q0, -0.7, 1.2)
            right = model.forward_kinematics_3d("FR", q0, -0.7, 1.2)
            self.assert_vector_close(left, (right[0], -right[1], right[2]))

    def test_coordinate_transforms_and_planted_foot(self):
        for leg in model.haa_positions:
            point = model.forward_kinematics_3d(leg, 0.2, -0.7, 1.2)
            body = model.haa_to_body(leg, point)
            self.assert_vector_close(model.body_to_haa(leg, body), point)
            x, z = model.forward_kinematics(-0.7, 1.2)
            zero_haa = model.haa_to_body(leg, model.forward_kinematics_3d(leg, 0, -0.7, 1.2))
            self.assert_vector_close(zero_haa, model.foot_in_body_frame(leg, x, z))

        foot_world = (0.21338, 0.19242, 0)
        for body_world, expected in [
            ((0, 0, 0.25), (0, 0.091, -0.25)),
            ((0, 0.02, 0.25), (0, 0.071, -0.25)),
        ]:
            target = model.body_to_haa("FL", model.world_to_body(foot_world, body_world))
            self.assert_vector_close(target, expected)
            angles = model.inverse_kinematics_3d("FL", *target)
            self.assertTrue(model.within_joint_limits(*angles))
            rebuilt_body = model.haa_to_body("FL", model.forward_kinematics_3d("FL", *angles))
            rebuilt_world = tuple(a + b for a, b in zip(rebuilt_body, body_world))
            self.assert_vector_close(rebuilt_world, foot_world)

    def test_reach_boundaries_and_angle_wrap(self):
        for target in ((0, 0.05, 0), (1, 0.091, -0.25)):
            with self.assertRaises(ValueError):
                model.inverse_kinematics_3d("FL", *target)
        # Radius exactly L0: roundoff may make the squared planar height negative.
        target = (0.2, 0.091 * math.cos(0.01), 0.091 * math.sin(0.01))
        angles = model.inverse_kinematics_3d("FL", *target)
        self.assert_vector_close(model.forward_kinematics_3d("FL", *angles), target)
        # Mathematical wrap checks, deliberately outside mechanical HAA limits.
        for leg, q0 in itertools.product(("FL", "FR"), (-3.1, 3.1)):
            point = model.forward_kinematics_3d(leg, q0, -0.7, 1.2)
            self.assert_vector_close(model.inverse_kinematics_3d(leg, *point), (q0, -0.7, 1.2))

    def test_joint_limits(self):
        limits = (model.HAA_LIMITS_RAD, model.HFE_LIMITS_RAD, model.KFE_LIMITS_RAD)
        for index, (low, high) in enumerate(limits):
            for value, allowed in ((low, True), (high, True), (low - 1e-6, False),
                                   (high + 1e-6, False), (math.nan, False)):
                angles = [0, -0.7, 1.2]
                angles[index] = value
                self.assertEqual(model.within_joint_limits(*angles), allowed)

    def test_support_margin_independent_of_vertex_order(self):
        for triangle in itertools.permutations(((0, 0), (2, 0), (0, 2))):
            for point, expected in (((0.5, 0.5), 0.5), ((1, 1), 0),
                                    ((1.5, 1.5), -1 / math.sqrt(2))):
                self.assertAlmostEqual(model.support_margin(*triangle, point), expected)
        for triangle in (((0, 0), (0, 0), (1, 1)), ((0, 0), (1, 1), (2, 2))):
            with self.assertRaises(ValueError):
                model.support_margin(*triangle, (0, 0))

    def test_current_gait(self):
        swing_order = ("FL", "RR", "FR", "RL")
        counts = {"inside": 0, "edge": 0, "outside": 0}
        for i in range(model.intervals + 1):
            t = i * model.dt
            stance, swing = [], []
            for leg, offset in model.phase_offsets.items():
                x, z, in_stance = model.foot_target(
                    t + offset * model.cycle_duration,
                    model.cycle_duration, model.stance_fraction,
                )
                q1, q2 = model.inverse_kinematics(x, z)
                self.assertTrue(model.within_joint_limits(0, q1, q2))
                self.assert_vector_close(model.forward_kinematics(q1, q2), (x, z))
                if in_stance:
                    stance.append(model.foot_in_body_frame(leg, x, z)[:2])
                else:
                    swing.append(leg)
            self.assertEqual(swing, [swing_order[int(t % 4)]])
            margin = model.support_margin(*stance, model.center_mass)
            state = "edge" if abs(margin) <= 1e-12 else "inside" if margin > 0 else "outside"
            counts[state] += 1
        self.assertEqual(counts, {"inside": 397, "edge": 4, "outside": 0})


if __name__ == "__main__":
    unittest.main()
