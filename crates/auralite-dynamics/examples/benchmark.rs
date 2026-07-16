//! Reproducible self-owned rigid-step timing harness.
use auralite_dynamics::{BodyBuilder2, World2};
use auralite_math::Vec2;
use std::hint::black_box;
use std::time::Instant;
fn main() {
    const BODIES: usize = 1_000;
    const STEPS: usize = 1_000;
    let mut w = World2::default();
    for i in 0..BODIES {
        w.add_body(BodyBuilder2::dynamic().position(Vec2 {
            x: i as f32,
            y: 10.0 + (i % 10) as f32,
        }))
        .expect("body");
    }
    let start = Instant::now();
    for _ in 0..STEPS {
        w.step(1.0 / 60.0).expect("step");
        black_box(w.state_hash());
    }
    let elapsed = start.elapsed();
    println!(
        "bodies={BODIES} steps={STEPS} elapsed_ns={} ns_per_body_step={:.3} hash={:016x}",
        elapsed.as_nanos(),
        elapsed.as_nanos() as f64 / (BODIES * STEPS) as f64,
        w.state_hash()
    );
}
