# Offline model conventions

`leg_kinematics.py` is an evolving geometric model. No Python function sends motor commands.
Positions are in metres, angles in radians, and all foot positions refer to the
center of the 20 mm-radius ball foot rather than its ground-contact surface.

## Frames and joint angles

Axes are +x forward, +y left, +z up. The body origin is centered in the symmetric
hip rectangle at the HAA pivot height, not at a measured center of mass.
HAA origins are at x = +/-0.21338 m and y = +/-0.10142 m in the body frame.
The HAA-to-HFE offset is 0.091 m outward at zero HAA. Link lengths are 0.170 m
and 0.180 m. These values model the CAD geometry and still need physical validation.

| Angle | Model convention |
| --- | --- |
| q0 (HAA) | Positive outward on either side near the normal hanging pose. |
| q1 (HFE) | Zero points the upper link down; positive moves the knee forward. |
| q2 (KFE) | Zero is straight; positive bends the knee, giving lower-link orientation q1 + q2. |

The knee linkage is approximated as a parallelogram with 1:1 angular transmission.
The relationship between model angles and actual calibrated motor directions must
be verified before commanding hardware, particularly HAA and knee bending direction.
Firmware calibration remains authoritative for the motors.

## Function map

For a fixed world foot and a translating, level body:

```text
world foot position
  -> world_to_body(foot_world, body_world)
  -> body_to_haa(leg_name, foot_body)
  -> inverse_kinematics_3d(leg_name, x, y, z)
  -> within_joint_limits(q0, q1, q2)
```

The reverse geometry is `forward_kinematics_3d()` followed by `haa_to_body()`.
HAA-relative positions use axes aligned with the body; they are not coordinates
in a frame that rotates with the HAA motor. The 3D FK already includes the 91 mm
offset, so do not add an HFE position to its result.

`world_to_body()` currently handles translation only. The earlier
`foot_in_body_frame()` and `body_to_hfe()` helpers assume HAA = 0; the planar gait
viewer still uses that model. `forward_kinematics()` and `inverse_kinematics()`
remain the two-link planar building blocks used by their 3D counterparts.

## Scope and checks

- 3D IK selects the negative planar-z, positive-knee solution. FK poses on a
  different branch need not recover the same joint angles.
- Geometric reachability and joint limits are separate checks. A successful IK
  result can still violate joint limits. HFE/KFE planning limits are the
  intersection of all four individually calibrated firmware ranges.
- The current gait has a four-second cycle, 75% stance, and swing order
  FL -> RR -> FR -> RL. Contacts are scheduled, not sensed.
- Swing and stance velocities match at joins; acceleration is not continuous.
- `support_margin()` returns the minimum inward signed distance to triangle
  edge lines. Positive means inside, zero on the boundary, negative outside.
  Outside magnitude is not generally the Euclidean distance to the triangle.
- With the assumed COM at the body origin, the current 20 ms samples over eight
  seconds give 397 inside and four on an edge. Zero clearance occurs at
  1, 3, 5, and 7 seconds. This is not a validated stable walking controller.

## Body shift and one forward step

Run `python body_shift_demo.py` for a separate four-second experiment:

1. With all four feet planted, shift the body 20 mm right over two seconds.
2. Hold the body fixed and swing FL 40 mm forward over two seconds, reaching
   30 mm clearance at the midpoint and returning to its original world height.

The body moves from `(0, 0, 0.25)` to `(0, -0.02, 0.25)`. FL's planned touchdown
is `(0.25338, 0.19242, 0)`. World z = 0 is initial foot-center height. The other
three feet remain fixed, and the assumed COM margin to their triangle stays
at +14.85 mm during swing. During the preceding shift, this is a prospective
three-foot margin; all four feet are still planted.

Each sample checks joint limits and reconstructs the requested world foot
positions through FK. The final target becomes FL's reference for a future
step; touchdown is planned, not detected. Body rotation, support transfer to
the next leg, and integration with the repeating gait are future work.
