# Quick Start Guide

AuraLite is a high-performance physics engine for Rust.

## Add to your project
```toml
[dependencies]
auralite-dynamics = { path = "path/to/auralite-dynamics" }
```

## Basic 2D Simulation
```rust
use auralite_dynamics::{World2, BodyBuilder2, Collider2, ColliderShape2, Material};
use auralite_math::Vec2;

fn main() {
    let mut world = World2::default();
    
    // Add a falling box
    let body = BodyBuilder2::dynamic()
        .position(Vec2 { x: 0.0, y: 10.0 })
        .add_collider(Collider2 {
            shape: ColliderShape2::Box(auralite_geometry::Box2::new(Vec2 { x: 0.5, y: 0.5 }).unwrap()),
            offset: Vec2::ZERO,
            material: Material::default(),
            filter: Default::default(),
        });
    world.add_body(body).unwrap();
    
    // Step simulation
    for _ in 0..60 {
        world.step(1.0 / 60.0).unwrap();
    }
}
```

## Key Commands
- `cargo test`: Run entire test battery.
- `cargo run -p auralite-sandbox`: Run the demo suite.
- `cargo bench`: Run performance benchmarks.
