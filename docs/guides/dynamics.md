# Dynamics Guide

## World2/World3

- `World2::default()` gravity (0, -9.81), `World3` (0, -9.81, 0)
- `set_gravity`, `solver_iterations` (default 10), `sleep_threshold`
- `add_body(BodyBuilder)`, `body(handle)`, `body_mut`, `remove_body`, `body_handles`, `body_count`, `step_count`, `gravity`, `bodies_iter`
- `apply_force`, `apply_impulse`, `apply_impulse_at_point`
- `ray_cast`, `ray_cast_ignoring(self_filter)`
- `state_hash()` FNV-1a over step+id+pos+rot+vel+ang+sleep+kind
- `snapshot()`/`restore()` for rollback
- `serialize_bodies`/`serialize_joints` (low-level, versioned envelope via `auralite-serialize` preferred)

## BodyBuilder

`BodyBuilder2::dynamic().position(Vec2).velocity(Vec2).mass(1.0).add_collider(...).linear_damping(0.01)`

`BodyBuilder3::dynamic().position(Vec3).mass(1.0).inertia_diagonal(Vec3)`

Static: `static_body()`, Kinematic: not yet separate builder but via `kind` field.

## Sleeping

Support-gated: `has_contact_support` (max_p >0 or contact normal_impulse) + velocity squared < `sleep_threshold` + angular threshold → sleeping. Apex bug fixed: airborne bodies never sleep at apex.

## Damping

Linear/angular damping applied as `v *= (1 - damping*dt).max(0)` each step.

## Allocation Budget

Scratch buffers (`scratch_handles`, `scratch_pairs`, `scratch_constraints`, `scratch_raw_contacts`, `scratch_id_to_h`, `prev_manifolds`, `prev_sensor_pairs`) pre-allocated, zero realloc in steady state verified via `steady_state_step_allocation_budget_2d` (capacity before/after 100 frames).

