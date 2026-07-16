//! Dependency-free reproducible math microbenchmark.
use auralite_math::{Real, Rot2, Transform2, Vec2};
use std::{hint::black_box, time::Instant};
fn main() {
    const N: usize = 5_000_000;
    let t = Transform2::new(
        Vec2 { x: 3.0, y: -2.0 },
        Rot2::from_radians(0.713).expect("angle"),
    )
    .expect("transform");
    let mut p = Vec2 { x: 1.0, y: 2.0 };
    let start = Instant::now();
    for i in 0..N {
        p = t.transform_point(
            p * 0.999_999
                + Vec2 {
                    x: (i & 1) as Real * 1.0e-6,
                    y: 0.0,
                },
        );
        p = black_box(p);
    }
    let elapsed = start.elapsed();
    println!(
        "iterations={N} elapsed_ns={} ns_per_transform={:.3} checksum={:?}",
        elapsed.as_nanos(),
        elapsed.as_nanos() as f64 / N as f64,
        p
    );
}
