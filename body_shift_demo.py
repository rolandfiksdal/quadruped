"""Offline body shift followed by one forward step; no motor commands.

World z = 0 is the initial foot-center height. The body stays level, and its
origin is used as the assumed COM. Touchdown here is a target, not sensed contact.
"""

import math

import leg_kinematics as kin

feet_world = {
    "FL": (0.21338, 0.19242, 0.0),
    "FR": (0.21338, -0.19242, 0.0),
    "RL": (-0.21338, 0.19242, 0.0),
    "RR": (-0.21338, -0.19242, 0.0),
}

dt = 0.02
move_duration = 2.0
start_pos = (0.0, 0.0, 0.25)
end_pos = (0.0, -0.02, 0.25)

step_foot = "FL"
step_distance = 0.04
lift_height = 0.03
swing_duration = 2.0


def check_and_print_pose(t, phase, body_world, foot_targets_world, triangle):
    """Solve all four legs and check limits and FK reconstruction at this time."""
    for leg, foot_target_world in foot_targets_world.items():
        foot_body = kin.world_to_body(foot_target_world, body_world)
        foot_haa = kin.body_to_haa(leg, foot_body)
        q0, q1, q2 = kin.inverse_kinematics_3d(leg, *foot_haa)
        allowed = kin.within_joint_limits(q0, q1, q2)
        if not allowed:
            raise ValueError(f"{phase} t={t:.2f}: {leg} exceeds joint limits")

        rebuilt_haa = kin.forward_kinematics_3d(leg, q0, q1, q2)
        rebuilt_body = kin.haa_to_body(leg, rebuilt_haa)
        rebuilt_world = tuple(rebuilt_body[j] + body_world[j] for j in range(3))
        error_m = math.dist(rebuilt_world, foot_target_world)
        if error_m > 1e-9:
            raise ValueError(f"{phase} t={t:.2f}: {leg} FK error {error_m} m")

        print(f"t={t:.2f} | {phase} | {leg}: "
              f"q0={q0:+.4f}, q1={q1:+.4f}, q2={q2:+.4f}, "
              f"world=({foot_target_world[0]:.5f}, {foot_target_world[1]:.5f}, "
              f"{foot_target_world[2]:.5f}), allowed={allowed}")

    # During the shift, this is the prospective triangle after FL lifts.
    # During swing, it is the triangle of the three scheduled stance feet.
    margin_m = kin.support_margin(*triangle, body_world[:2])
    print(f"t={t:.2f} | {phase} | three-foot support margin={margin_m * 1000:+.2f} mm")


def main():
    # Copy reference positions so rerunning the demo does not accumulate steps.
    foot_positions = feet_world.copy()
    triangle = tuple(position[:2] for leg, position in foot_positions.items()
                     if leg != step_foot)

    # Phase 1: all feet remain planted as the body shifts right.
    move_intervals = round(move_duration / dt)
    for i in range(move_intervals + 1):
        u = i / move_intervals
        t = u * move_duration
        s = 3 * u**2 - 2 * u**3
        body_world = tuple(start_pos[j] + (end_pos[j] - start_pos[j]) * s
                           for j in range(3))
        check_and_print_pose(t, "shift", body_world, foot_positions, triangle)

    # Phase 2: hold the body fixed and swing FL forward with ground clearance.
    body_world = end_pos
    if kin.support_margin(*triangle, body_world[:2]) <= 0:
        raise ValueError("Body shift must give positive support margin before swing")
    swing_start = foot_positions[step_foot]
    swing_intervals = round(swing_duration / dt)
    for i in range(swing_intervals + 1):
        u = i / swing_intervals
        t = move_duration + u * swing_duration
        s = 3 * u**2 - 2 * u**3
        lift = 16 * u**2 * (1 - u)**2

        # Build from the fixed start each time, so displacement does not accumulate.
        foot_targets_world = foot_positions.copy()
        foot_targets_world[step_foot] = (
            swing_start[0] + step_distance * s,
            swing_start[1],
            swing_start[2] + lift_height * lift,
        )
        check_and_print_pose(t, "swing", body_world, foot_targets_world, triangle)

    # Planned touchdown becomes the reference position for a future step.
    foot_positions[step_foot] = foot_targets_world[step_foot]
    print(f"Planned touchdown: {step_foot} world={foot_positions[step_foot]}")
    return foot_positions


if __name__ == "__main__":
    main()




