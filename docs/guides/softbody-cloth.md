# Soft Body / Cloth Guide (XPBD)

## SoftBody

`SoftBody` has `particles: Vec<Particle>` (position, old_position, velocity, inv_mass, pinned), `constraints: Vec<Constraint>`, `edge_indices`, `triangle_indices`, `damping`, `wind`, `aerodynamic`.

`Particle::new(position, inv_mass)`

`Constraint` enum: `Stretch { p1,p2,rest_length,compliance }`, `Bend`, `Volume`, `Attachment { particle, target, compliance }`, `RigidAttachment2/3 { particle, body, local_offset, compliance }`

Methods: `new(damping)`, `pre_step(dt, gravity)`, `solve_constraints(iterations, dt)`, `post_step(dt)`, `kinetic_energy()`

## Builders

- `build_cloth_grid(rows, cols, spacing, origin, normal, tangent, pin_top, total_mass, stretch_compliance, bend_compliance, damping)` — 8x8 etc.
- `build_cloth_strip(segments, spacing, origin, pin_top, total_mass, stretch_compliance, bend_compliance, damping)`
- `build_soft_cube(...)`

All builders have `#[allow(clippy::too_many_arguments)]` justified: many physical params needed, builder would add complexity, explicit args for determinism.

## Self-Collision

`apply_self_collision(&mut softbody, particle_radius)` — spatial hash queries, pushes particles apart if distance < radius.

## Rigid Coupling

`apply_rigid_coupling_2d/3d` — attaches softbody particles to rigid bodies.

## Example

```rust
use auralite_softbody::{build_cloth_grid, apply_self_collision};
use auralite_math::Vec3;
let mut cloth = build_cloth_grid(8,8,0.15, Vec3 { x: -0.5, y: 0.7, z: 0.0 }, Vec3 { x: 0.0, y: 0.0, z: 1.0 }, Vec3::X, true, 3.0, 0.1, 1.0, 0.01);
for _ in 0..200 {
    cloth.pre_step(1.0/60.0, Vec3 { x: 0.0, y: -9.81, z: 0.0 });
    cloth.solve_constraints(10, 1.0/60.0);
    cloth.post_step(1.0/60.0);
}
```

See `crates/auralite-sandbox/src/main.rs` scene_cloth.

