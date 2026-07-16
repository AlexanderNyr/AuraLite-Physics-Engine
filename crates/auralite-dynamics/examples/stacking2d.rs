//! 2D stacking demo: static ground with dynamic boxes piled on top.
use auralite_collision::CollisionFilter;
use auralite_dynamics::*;
use auralite_math::Vec2;

fn main() {
    let mut world = World2::default();
    // Ground
    world
        .add_body(BodyBuilder2::static_body().add_collider(Collider2 {
            shape: ColliderShape2::Box(
                auralite_geometry::Box2::new(Vec2 { x: 10.0, y: 0.5 }).unwrap(),
            ),
            offset: Vec2::ZERO,
            material: Material::default(),
            filter: CollisionFilter::default(),
        }))
        .unwrap();
    // Stack boxes
    for i in 0..5 {
        let x = (i as f32 - 2.0) * 1.1;
        let y = 1.0 + i as f32 * 1.1;
        world
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x, y })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Box(
                            auralite_geometry::Box2::new(Vec2 { x: 0.5, y: 0.5 }).unwrap(),
                        ),
                        offset: Vec2::ZERO,
                        material: Material {
                            restitution: 0.0,
                            friction: 0.8,
                            density: 1.0,
                        },
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
    }
    for step in 0..600 {
        world.step(1.0 / 60.0).unwrap();
        if step % 120 == 0 {
            println!(
                "Step {}: {} bodies, hash {}",
                step,
                world.body_count(),
                world.state_hash()
            );
        }
    }
    println!("Final hash: {}", world.state_hash());
    println!("Stacking simulation complete.");
}
