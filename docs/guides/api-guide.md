# Rust API Guide

## Core Crates

- `auralite-math`: `Vec2`, `Vec3`, `Rot2`, `Quat`, `Ray2`, `Ray3`, `Aabb2`, `Aabb3`, `Plane3`, `Line2` — dimension-safe, `f32` default, `f64` feature, SSE2/NEON.
- `auralite-core`: `Pool<T>`, `Handle<T>`, `StableId`, `Rng`, `hash_bytes`, `Scheduler` trait, `ThreadPoolScheduler`, `SingleThreadScheduler`, `StepConfig`.
- `auralite-geometry`: `Circle2`, `Box2`, `Capsule2`, `ConvexPolygon`, `Edge2`, `Sphere3`, `Box3`, `Capsule3`, `ConvexHull3`, `TriangleMesh`, `TriangleHeightfield3`, `volume()`, `ray_intersection()`, `support()`.
- `auralite-collision`: `DynamicTree2/3`, `CollisionFilter`, `Manifold2/3`, `GJK`, `EPA`, `SAT`, `clip_contacts2`, `FeatureId`.
- `auralite-dynamics`: `World2`, `World3`, `Body2`, `Body3`, `BodyBuilder2/3`, `BodyType`, `Material`, `Collider2/3`, `ColliderShape2/3`, `Joint2/3`, `JointConfig2/3`, `JointType2/3` (including `ConeTwist`), `JointLimits`, `JointMotor`, `SensorEvent` (Begin/Stay/End), `Snapshot2/3`, `InputRecorder` (lockstep).
- `auralite-softbody`: `SoftBody`, `Particle`, `Constraint`, `build_cloth_grid`, `build_cloth_strip`, `apply_self_collision`, `apply_rigid_coupling_2d/3d`.
- `auralite-particles`: `ParticleStorage`, `ParticleType`, `Emitter`, `PbfFluid`, `ForceField`, `FieldType`, `compute_buoyancy`, `volume()`, `apply_force_fields_to_particles`.
- `auralite-vehicles`: `Vehicle2/3`, `VehicleConfig2/3`, `WheelConfig2/3`, `Character2/3`, `CharacterConfig2/3`.
- `auralite-serialize`: `encode`, `decode`, `serialize_world2/3`, `deserialize_world2/3`, `serialize_body2/3`, `TypeTag`, `Error`.
- `auralite-ffi`: C ABI, `auralite_world2_create`, `auralite_set_log_callback`, `auralite_set_scheduler_callback`, etc., safety docs.
- `auralite-gpu`: `GpuBackend`, `CpuBackend`, `GpuEngine` (CPU reference per ADR-13).
- `auralite-sandbox`: headless `cargo run -p auralite-sandbox --release` (16 scenes + watermarked replay viewer `docs/generated/scenes.html`), interactive `cargo run -p auralite-sandbox --features interactive -- --interactive` (eframe).

## Typical Usage

```rust
use auralite_dynamics::{World2, BodyBuilder2, Collider2, ColliderShape2, Material};
use auralite_math::Vec2;
use auralite_geometry::Box2;

let mut world = World2::default();
let ground = BodyBuilder2::static_body().add_collider(Collider2 {
    shape: ColliderShape2::Box(Box2::new(Vec2 { x: 10.0, y: 0.5 }).unwrap()),
    offset: Vec2::ZERO,
    material: Material::default(),
    filter: Default::default(),
});
world.add_body(ground).unwrap();
for _ in 0..60 {
    world.step(1.0/60.0).unwrap();
}
println!("hash {:016x}", world.state_hash());
```

## Determinism

Use `world.state_hash()` for Tier A bitwise checks, `world.snapshot()`/`restore()` for rollback, `InputRecorder` for lockstep input replay.

## Feature Flags

- `multithread` (default) vs `single-thread` in `auralite-dynamics`
- `f32` (default) vs `f64` in `auralite-math`
- `interactive` in `auralite-sandbox` (eframe, x11+wayland+glow)
