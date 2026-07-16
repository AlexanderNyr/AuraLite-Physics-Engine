//! Reproducible geometry microbenchmark.
use auralite_geometry::{Box3, TriangleMesh};
use auralite_math::{Real, Vec3};
use std::{hint::black_box, time::Instant};
fn main() {
    let shape = Box3::new(Vec3 {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    })
    .expect("box");
    let start = Instant::now();
    let mut sum = Vec3::ZERO;
    const N: usize = 2_000_000;
    for i in 0..N {
        let a = i as Real * 0.000_17;
        sum += black_box(shape.support(Vec3 {
            x: a.sin(),
            y: a.cos(),
            z: (a * 0.37).sin(),
        }));
    }
    let support = start.elapsed();
    let side = 64usize;
    let vertices = (0..side * side)
        .map(|i| Vec3 {
            x: (i % side) as Real,
            y: ((i * 17) % 11) as Real * 0.01,
            z: (i / side) as Real,
        })
        .collect();
    let mut triangles = Vec::new();
    for z in 0..side - 1 {
        for x in 0..side - 1 {
            let a = (z * side + x) as u32;
            triangles.push([a, a + side as u32, a + 1]);
            triangles.push([a + 1, a + side as u32, a + side as u32 + 1]);
        }
    }
    let start = Instant::now();
    let mesh = TriangleMesh::new(vertices, triangles).expect("mesh");
    let build = start.elapsed();
    println!(
        "support_n={N} support_ns={} ns_per_support={:.3} triangles={} bvh_nodes={} bvh_build_ns={} checksum={:?}",
        support.as_nanos(),
        support.as_nanos() as f64 / N as f64,
        (side - 1) * (side - 1) * 2,
        mesh.bvh().len(),
        build.as_nanos(),
        sum
    );
}
