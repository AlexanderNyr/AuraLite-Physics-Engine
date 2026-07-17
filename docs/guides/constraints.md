# Constraints Guide (Joints)

## 2D

`JointType2`: `Weld`, `Distance`, `Spring { stiffness, damping }`, `Revolute`, `Prismatic { axis_local }`

Config: `JointConfig2::new(type, body_a, body_b, anchor_a, anchor_b)` + `limits` + `motor` + `break_impulse`

Motor: `JointMotor { target_speed, max_force, enabled }` — drives angular velocity for Revolute, linear for Prismatic.

Limits: `JointLimits { min, max, enabled }`

Break: `break_impulse` threshold, `broken` flag, `JointBreakEvent`.

Solve: Sequential impulse with warm-starting via `prev_manifolds`.

## 3D

`JointType3`: `BallSocket`, `Weld`, `Distance`, `Spring`, `Hinge { axis_local }`, `Slider { axis_local }`, `ConeTwist { axis_local, swing_limit, twist_limit }` (H5)

- BallSocket/Weld: point-to-point
- Distance: holds distance with optional limits
- Spring: stiffness/damping
- Hinge: axis + motor (angular)
- Slider: axis + motor (linear)
- ConeTwist: swing limited to cone half-angle, twist limited around axis, via swing/twist decomposition (quaternion twist projection), corrective angular impulses, tests `joint3_cone_twist_limits_never_exceeded` (angle <= swing+0.15) and `stability_long_run` 1000 steps finite.

Example:

```rust
use auralite_dynamics::{JointConfig3, JointType3, JointLimits, JointMotor};
use auralite_math::Vec3;
let cfg = JointConfig3 {
    joint_type: JointType3::ConeTwist { axis_local: Vec3::Z, swing_limit: std::f64::consts::FRAC_PI_4, twist_limit: std::f64::consts::FRAC_PI_6 },
    body_a, body_b,
    anchor_a: Vec3::ZERO, anchor_b: Vec3::ZERO,
    limits: JointLimits::default(),
    motor: JointMotor::default(),
    break_impulse: 0.0,
    user_data: 0,
};
world.add_joint(cfg).unwrap();
```

## Breakable & Motors Tests

`joint2_break_impulse_breaks_under_excess_force`, `joint3_break_impulse_breaks_under_excess_force`, `joint3_hinge_motor_converges_to_target_speed`, `joint3_slider_motor_converges_to_target_speed`, `joint3_cone_twist_*`.

