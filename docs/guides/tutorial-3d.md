# 3D Tutorial

## Basic World

```rust
use auralite_dynamics::{World3, BodyBuilder3, Collider3, ColliderShape3, Material};
use auralite_math::Vec3;
use auralite_geometry::Box3;

let mut world = World3::default();
world.add_body(BodyBuilder3::static_body().position(Vec3 { x: 0.0, y: -0.5, z: 0.0 }).add_collider(Collider3 {
    shape: ColliderShape3::Box(Box3::new(Vec3 { x: 10.0, y: 0.5, z: 10.0 }).unwrap()),
    offset: Vec3::ZERO,
    material: Material::default(),
    filter: Default::default(),
})).unwrap();
let b = world.add_body(BodyBuilder3::dynamic().position(Vec3 { x: 0.0, y: 5.0, z: 0.0 }).add_collider(Collider3 {
    shape: ColliderShape3::Box(Box3::new(Vec3 { x: 0.5, y: 0.5, z: 0.5 }).unwrap()),
    offset: Vec3::ZERO,
    material: Material::default(),
    filter: Default::default(),
})).unwrap();
for _ in 0..60 { world.step(1.0/60.0).unwrap(); }
println!("pos {:?}", world.body(b).unwrap().position);
```

## Joints 3D

`JointType3` includes `BallSocket`, `Weld`, `Distance`, `Spring`, `Hinge { axis_local }`, `Slider { axis_local }`, `ConeTwist { axis_local, swing_limit, twist_limit }` (H5). ConeTwist enforces swing (cone half-angle) and twist (around axis) limits via corrective angular impulses. Tests `joint3_cone_twist_limits_never_exceeded`.

## Vehicle 3D

Ray-cast wheels with true normals via `ray_cast_ignoring`.

```rust
use auralite_vehicles::{Vehicle3, VehicleConfig3, WheelConfig3};
use auralite_math::{Vec3, Quat};
let wheels = vec![WheelConfig3::default(); 4];
let mut vehicle = Vehicle3::new(VehicleConfig3::default(), Vec3 { x: 0.0, y: 1.0, z: 0.0 }, Quat::identity(), wheels, &mut world);
vehicle.set_controls(0.5, 0.0, 0.3);
```

## Character 3D

Similar to 2D but with `Vec3` move.

