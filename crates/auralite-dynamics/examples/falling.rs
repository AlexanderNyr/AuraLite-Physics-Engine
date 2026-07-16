//! Falling-body vertical-slice example.
use auralite_dynamics::{BodyBuilder2, World2};
use auralite_math::Vec2;
fn main() {
    let mut world = World2::default();
    let body = world
        .add_body(
            BodyBuilder2::dynamic()
                .position(Vec2 { x: 0.0, y: 5.0 })
                .restitution(0.25),
        )
        .expect("valid");
    for _ in 0..300 {
        world.step(1.0 / 60.0).expect("step");
    }
    println!(
        "position={:?}, hash={:016x}",
        world.body(body).expect("body").position,
        world.state_hash()
    );
}
