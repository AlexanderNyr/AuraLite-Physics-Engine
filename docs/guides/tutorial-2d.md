# 2D Tutorial

## Stacking

```rust
use auralite_dynamics::{World2, BodyBuilder2, Collider2, ColliderShape2, Material};
use auralite_math::Vec2;
use auralite_geometry::Box2;

let mut world = World2::default();
world.add_body(BodyBuilder2::static_body().add_collider(Collider2 {
    shape: ColliderShape2::Box(Box2::new(Vec2 { x: 10.0, y: 0.5 }).unwrap()),
    offset: Vec2::ZERO,
    material: Material { restitution: 0.0, friction: 0.8, density: 1.0 },
    filter: Default::default(),
})).unwrap();
for i in 0..5 {
    world.add_body(BodyBuilder2::dynamic().position(Vec2 { x: (i as f64 -2.0)*1.1, y: 1.0 + i as f64*1.1 }).add_collider(Collider2 {
        shape: ColliderShape2::Box(Box2::new(Vec2 { x: 0.5, y: 0.5 }).unwrap()),
        offset: Vec2::ZERO,
        material: Material::default(),
        filter: Default::default(),
    })).unwrap();
}
for _ in 0..3600 { world.step(1.0/60.0).unwrap(); }
assert!(world.body_handles().len() == 6);
println!("hash {:016x}", world.state_hash());
```

## Joints Ragdoll

Use `JointConfig2::new(JointType2::Revolute, body_a, body_b, anchor_a, anchor_b)` and `world.add_joint`.

## Sensors

`CollisionFilter { sensor: true, .. }` makes collider a trigger. `world.sensor_events` yields `SensorEvent { sensor, other, began, is_stay }` where `began=true` Begin, `is_stay=true` Stay (H6), `began=false,is_stay=false` End. Stay emitted each step for ongoing pairs in deterministic sorted order.

## Character 2D

```rust
use auralite_vehicles::{Character2, CharacterConfig2};
use auralite_math::Vec2;
let mut cc = Character2::new(CharacterConfig2::default(), Vec2 { x: 0.0, y: 2.0 });
cc.attach(&mut world);
cc.set_move(Vec2 { x: 1.0, y: 0.0 });
cc.step(1.0/60.0, &mut world);
```

See `crates/auralite-sandbox/src/main.rs` scene functions for more.

