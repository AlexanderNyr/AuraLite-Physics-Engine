# Particles / Fluids Guide

## ParticleStorage

SoA storage with free list: `positions: Vec<Vec3>`, `velocities`, `lifetimes`, `alive: Vec<bool>`, free list.

`new(capacity)`, `spawn(pos, vel, lifetime, ParticleType)`, `kill(idx)`, `alive_count()`, `iterate_alive()`

`ParticleType`: `Fluid`, `BuoyancySample`, `Generic`

## Emitter

`Emitter::new(position, direction, cone_angle, speed, rate, lifetime, seed)` — seeded deterministic, `emit(dt, &mut storage)` returns count.

## PBF Fluid

`PbfFluid::new(rest_density, particle_radius, stiffness, viscosity)`

`step(&mut storage, &indices, dt, gravity)` — spatial hash O(n) neighbor search via `SpatialHash`, density relaxation.

## Buoyancy

`compute_buoyancy(&body, &colliders, &fluid_positions, fluid_density, particle_volume, gravity)` — exact `volume()` per shape (`Sphere3::volume() = 4/3πr³`, `Box3 = 8*hx*hy*hz`, `Capsule`, `ConvexHull`), Archimedes `F_buoy = -gravity * fluid_density * displaced_volume`. Test `buoyancy_floating_box_equilibrium` proves neutral equilibrium.

Two-way coupling: `apply_buoyancy_to_world`.

## Force Fields

`FieldType`: `Uniform { acceleration }`, `Radial { center, strength, max_radius }`, `Wind { direction, speed, turbulence }`, `Drag { linear, quadratic }`, `Damping { factor }`

`ForceField { field_type, position, radius, falloff, affects_particles, affects_rigid }`

`apply_force_fields_to_particles(&fields, &mut storage, dt)`

Example:

```rust
use auralite_particles::{ParticleStorage, Emitter, PbfFluid, ParticleType};
use auralite_math::Vec3;
let mut storage = ParticleStorage::new(200);
for i in 0..5 { for j in 0..5 {
    storage.spawn(Vec3 { x: i as f64*0.12, y: j as f64*0.12+1.0, z: 0.0 }, Vec3::ZERO, 10.0, ParticleType::Fluid);
}}
let indices: Vec<usize> = storage.iterate_alive().map(|(i,_,_,_)| i).collect();
let mut fluid = PbfFluid::new(1000.0, 0.06, 0.1, 0.01);
fluid.step(&mut storage, &indices, 1.0/60.0, Vec3 { x: 0.0, y: -9.81, z: 0.0 });
```

