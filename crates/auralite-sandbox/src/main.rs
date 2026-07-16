//! Headless developer sandbox vertical slice.
use auralite_dynamics::{BodyBuilder2, BodyBuilder3, World2, World3};
use auralite_math::{Vec2, Vec3};
fn main() {
    let mut w2 = World2::default();
    let h2 = w2
        .add_body(BodyBuilder2::dynamic().position(Vec2 { x: 0.0, y: 3.0 }))
        .expect("valid scene");
    let mut w3 = World3::default();
    let h3 = w3
        .add_body(BodyBuilder3::dynamic().position(Vec3 {
            x: 0.0,
            y: 3.0,
            z: 0.0,
        }))
        .expect("valid scene");
    for _ in 0..180 {
        w2.step(1.0 / 60.0).expect("finite step");
        w3.step(1.0 / 60.0).expect("finite step");
    }
    println!(
        "AuraLite sandbox (headless vertical slice)\n2D: {:?} hash {:016x}\n3D: {:?} hash {:016x}",
        w2.body(h2).expect("body").position,
        w2.state_hash(),
        w3.body(h3).expect("body").position,
        w3.state_hash()
    );
}
