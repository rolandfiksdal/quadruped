"""Offline quadruped kinematics and gait experiments; no motor connection.

Positions are foot-center coordinates in metres; angles are radians.
Axes: +x forward, +y left, +z up. The 3D model uses outward-positive HAA,
whose mapping to the physical joint convention still needs validation.
"""

import math

HAA_OFFSET_M = 0.091
UPPER_LINK_M = 0.170
LOWER_LINK_M = 0.180

# Shared planning ranges: intersection of all four joints in each group.
# Keep aligned with calibration/limits in Core/Src/robot_joints.c.
HFE_LIMITS_RAD = (-1.360, 0.620)
KFE_LIMITS_RAD = (0.8562, 2.353)
HAA_LIMITS_RAD = (-math.radians(27.5), math.radians(27.5))

haa_positions = {
    "FL": (0.21338, 0.10142, 0.0),
    "FR": (0.21338, -0.10142, 0.0),
    "RL": (-0.21338, 0.10142, 0.0),
    "RR": (-0.21338, -0.10142, 0.0)
}

# HFE locations only at HAA = 0; 3D FK rotates this offset about the HAA pivot.
hfe_positions = {
    "FL": (0.21338, 0.10142 + HAA_OFFSET_M, 0.0),
    "FR": (0.21338, -(0.10142 + HAA_OFFSET_M), 0.0),
    "RL": (-0.21338, 0.10142 + HAA_OFFSET_M, 0.0),
    "RR": (-0.21338, -(0.10142 + HAA_OFFSET_M), 0.0)
}

# Assumed COM projection for the support experiment, not a measured mass model.
center_mass = (0.0, 0.0, 0.0)

start_pos = (-0.05, -0.25)
end_pos = (0.05, -0.25)
z_ground = -0.25  # Foot-center height relative to HFE, not the contact surface.
h = 0.03

dt = 0.020

simulation_duration = 8.0
intervals = round(simulation_duration / dt)
phase_offsets = {"FL": 0.0, "FR": 0.5, "RL": 0.25, "RR": 0.75}
cycle_duration = 4.0
stance_fraction = 0.75


def forward_kinematics(q1_rad, q2_rad, L1_m=UPPER_LINK_M, L2_m=LOWER_LINK_M):
    """HFE/KFE angles to planar (x, z) relative to the HFE pivot."""
    p_foot = [L1_m * math.sin(q1_rad) + L2_m * math.sin(q1_rad + q2_rad),
              (-L1_m) * math.cos(q1_rad) + (-L2_m) * math.cos(q1_rad + q2_rad)]
    x, z = p_foot

    return x, z


def forward_kinematics_3d(leg_name, q0_rad, q1_rad, q2_rad):
    """Three joint angles to a foot position relative to the HAA pivot."""
    x, z = forward_kinematics(q1_rad, q2_rad)
    if (leg_name == "FL" or leg_name == "RL" ):
        s = 1
    elif (leg_name == "FR" or leg_name == "RR" ):
        s = -1
    else:
        raise ValueError(f"Unknown leg: {leg_name}")

    p = (x, s * HAA_OFFSET_M, z)
    rotated_p = rotate_about_x(p, s * q0_rad)

    return rotated_p


def inverse_kinematics(x_m, z_m, L1_m=UPPER_LINK_M, L2_m=LOWER_LINK_M):
    """HFE-relative planar target to the positive-knee branch; limits separate."""
    cos_q2 = (x_m**2 + z_m**2 - L1_m**2 - L2_m**2) / (2 * L1_m * L2_m)

    if (-1-1e-12 < cos_q2 < 1+1e-12):
        cos_q2 = max(min(cos_q2, 1.0), -1.0)
    else:
        raise ValueError("Target outside geometric workspace")

    q2 = math.acos(cos_q2)

    q1 = math.atan2(x_m, -z_m) - math.atan2(L2_m * math.sin(q2), L1_m + L2_m * math.cos(q2))

    return q1, q2


def inverse_kinematics_3d(leg_name, x, y, z):
    """HAA-relative target to (q0, q1, q2), choosing negative planar z.

    Uses the positive-knee branch. Call within_joint_limits() on the result.
    """
    if (leg_name == "FL" or leg_name == "RL" ):
        s = 1
    elif (leg_name == "FR" or leg_name == "RR" ):
        s = -1
    else:
        raise ValueError(f"Unknown leg: {leg_name}")

    d = y**2 + z**2 - HAA_OFFSET_M**2
    if d < -1e-12:
        raise ValueError(f"Unreachable target: {x}, {y}, {z}")

    z_planar = -math.sqrt(max(d, 0.0))

    b_0 = math.atan2(s * HAA_OFFSET_M, -z_planar)
    b_target = math.atan2(y, -z)
    alpha = b_target - b_0
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))

    q0 = s * alpha
    q1, q2 = inverse_kinematics(x, z_planar)

    return q0, q1, q2


def rotate_about_x(point, angle_rad):
    """Rotate a vector by a right-hand angle about +x."""
    rotated_y = point[1] * math.cos(angle_rad) - point[2] * math.sin(angle_rad)
    rotated_z = point[1] * math.sin(angle_rad) + point[2] * math.cos(angle_rad)
    x = point[0]
    return x, rotated_y, rotated_z


def within_joint_limits(q0_rad, q1_rad, q2_rad, haa_limits=HAA_LIMITS_RAD, hfe_limits=HFE_LIMITS_RAD, kfe_limits=KFE_LIMITS_RAD):
    """Check all three angles against inclusive shared planning limits."""
    haa_min, haa_max = haa_limits
    hfe_min, hfe_max = hfe_limits
    kfe_min, kfe_max = kfe_limits

    if (haa_min <= q0_rad <= haa_max and
        hfe_min <= q1_rad <= hfe_max and
        kfe_min <= q2_rad <= kfe_max):
        return True
    else:
        return False


def foot_target(t_seconds, cycle_duration=4.0, stance_fraction=0.75):
    """Time to HFE-relative (x, z, scheduled stance) for the planar gait."""
    cycle_time = t_seconds % cycle_duration
    stance_duration = cycle_duration * stance_fraction
    swing_duration = cycle_duration - stance_duration

    if cycle_time < swing_duration:
        u = cycle_time / swing_duration
        m = - (swing_duration / stance_duration)
        s = (2 * m - 2) * u**3 + (3 - 3 * m) * u**2 + m * u

        x = start_pos[0] + (end_pos[0] - start_pos[0]) * s
        z = z_ground + h * (16 * u**2 * (1 - u)**2)

        return x, z, False
    else:
        u = (cycle_time - swing_duration) / stance_duration

        x = end_pos[0] + (start_pos[0] - end_pos[0]) * u
        z = z_ground

        return x, z, True


def foot_in_body_frame(leg_name, x, z, y=0):
    """Translate an HFE-relative point to body coordinates, assuming HAA = 0."""
    x = hfe_positions[leg_name][0] + x
    y = hfe_positions[leg_name][1] + y
    z = hfe_positions[leg_name][2] + z

    return x, y, z


def world_to_body(foot_world, body_world):
    """World foot position minus body origin; assumes aligned frame axes."""
    x = foot_world[0] - body_world[0]
    y = foot_world[1] - body_world[1]
    z = foot_world[2] - body_world[2]

    return x, y, z


def body_to_hfe(leg_name, foot_body):
    """Translate a body-frame point to the HFE origin, assuming HAA = 0."""
    x = foot_body[0] - hfe_positions[leg_name][0]
    y = foot_body[1] - hfe_positions[leg_name][1]
    z = foot_body[2] - hfe_positions[leg_name][2]

    return x, y, z


def haa_to_body(leg_name, foot_haa):
    """Translate an HAA-relative point (body-aligned axes) to the body origin."""
    x = haa_positions[leg_name][0] + foot_haa[0]
    y = haa_positions[leg_name][1] + foot_haa[1]
    z = haa_positions[leg_name][2] + foot_haa[2]

    return x, y, z


def body_to_haa(leg_name, foot_body):
    """Translate a body-frame point to the HAA origin, keeping body-aligned axes."""
    x = foot_body[0] - haa_positions[leg_name][0]
    y = foot_body[1] - haa_positions[leg_name][1]
    z = foot_body[2] - haa_positions[leg_name][2]

    return x, y, z


def edge_side(a, b, p):
    """Signed 2D cross product: positive on the left of directed edge a -> b."""
    c = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
    return c


def distance_to_edge_line(a, b, p):
    """Unsigned perpendicular distance to the line through distinct a and b."""
    edge_length = math.hypot(b[0] - a[0], b[1] - a[1])
    d = abs(edge_side(a, b, p)) / edge_length

    return d


def support_margin(a, b, c, p):
    """Minimum inward signed distance to the triangle's edge lines, in metres.

    Positive means inside, zero means on the boundary, negative means outside.
    Outside the triangle, this is not generally the Euclidean distance to it.
    Vertices must form a nondegenerate triangle; only x/y coordinates are used.
    """
    signed_area = edge_side(a, b, c)
    if signed_area == 0:
        raise ValueError("Support triangle needs three noncollinear vertices")

    # Make the interior side positive for either vertex ordering.
    orientation = 1 if signed_area > 0 else -1
    distances = []
    for start, end in ((a, b), (b, c), (c, a)):
        edge_length = math.hypot(end[0] - start[0], end[1] - start[1])
        distance = orientation * edge_side(start, end, p) / edge_length
        distances.append(distance)

    return min(distances)

if __name__ == "__main__":
    # Run the text simulation only when this file is executed directly.
    # The animation can import the model without running this loop.
    for i in range(intervals + 1):
        t = i * dt
        stance_feet = []

        for leg_name, phase_offset in phase_offsets.items():
            t_offset = t + phase_offset * cycle_duration
            x, z, in_stance = foot_target(t_offset, cycle_duration, stance_fraction)

            q1, q2 = inverse_kinematics(x, z)
            allowed = within_joint_limits(0.0, q1, q2)
            x_body, y_body, z_body = foot_in_body_frame(leg_name, x, z)
            if (in_stance):
                stance_feet.append((x_body, y_body))
            print(f"t={t:.2f}, leg={leg_name} x={x:.4f}, z={z:.4f}, allowed={allowed}")

        a, b, c = stance_feet
        margin_m = support_margin(a, b, c, center_mass)
        inside_support_triangle = margin_m > 0

        print(f"inside={inside_support_triangle}, support margin={margin_m * 1000:+.2f} mm")
