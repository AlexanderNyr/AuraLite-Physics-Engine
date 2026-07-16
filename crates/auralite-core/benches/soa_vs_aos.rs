//! Benchmark: SoA (Struct of Arrays) vs AoS (Array of Structs) particle layout.
//!
//! Measures throughput for: position + velocity integration, distance computation,
//! and neighbor density accumulation.
//!
//! To run: cargo bench --bench soa_vs_aos

use auralite_math::{Vec3, Real};

const N: usize = 10_000;

// AoS layout
#[derive(Clone, Copy)]
struct ParticleAoS {
    position: Vec3,
    velocity: Vec3,
}

// SoA layout
struct ParticleSoA {
    positions: Vec<Vec3>,
    velocities: Vec<Vec3>,
}

/// AoS integration step.
fn aos_step(particles: &mut [ParticleAoS], gravity: Vec3, dt: Real) {
    for p in particles.iter_mut() {
        p.velocity += gravity * dt;
        p.position += p.velocity * dt;
    }
}

/// SoA integration step.
fn soa_step(positions: &mut [Vec3], velocities: &mut [Vec3], gravity: Vec3, dt: Real) {
    for i in 0..positions.len() {
        velocities[i] += gravity * dt;
        positions[i] += velocities[i] * dt;
    }
}

/// AoS density computation (simplified).
fn aos_density(particles: &[ParticleAoS], h2: Real) -> Vec<Real> {
    let mut densities = vec![0.0; particles.len()];
    for i in 0..particles.len() {
        let mut d = 1.0;
        for j in 0..particles.len() {
            if i == j { continue; }
            let diff = particles[i].position - particles[j].position;
            if diff.length_squared() < h2 {
                d += 1.0;
            }
        }
        densities[i] = d;
    }
    densities
}

/// SoA density computation.
fn soa_density(positions: &[Vec3], h2: Real) -> Vec<Real> {
    let mut densities = vec![0.0; positions.len()];
    for i in 0..positions.len() {
        let mut d = 1.0;
        for j in 0..positions.len() {
            if i == j { continue; }
            let diff = positions[i] - positions[j];
            if diff.length_squared() < h2 {
                d += 1.0;
            }
        }
        densities[i] = d;
    }
    densities
}

fn main() {
    let gravity = Vec3 { x: 0.0, y: -9.81, z: 0.0 };
    let dt = 1.0 / 60.0;
    let h2 = 1.0;

    // AoS
    let mut aos: Vec<ParticleAoS> = (0..N).map(|i| ParticleAoS {
        position: Vec3 { x: (i % 100) as Real * 0.1, y: (i / 100) as Real * 0.1, z: 0.0 },
        velocity: Vec3::ZERO,
    }).collect();

    // SoA
    let mut soa = ParticleSoA {
        positions: (0..N).map(|i| Vec3 { x: (i % 100) as Real * 0.1, y: (i / 100) as Real * 0.1, z: 0.0 }).collect(),
        velocities: vec![Vec3::ZERO; N],
    };

    // Warmup
    for _ in 0..10 {
        aos_step(&mut aos, gravity, dt);
        soa_step(&mut soa.positions, &mut soa.velocities, gravity, dt);
    }

    // Benchmark integration
    let start = std::time::Instant::now();
    for _ in 0..1000 { aos_step(&mut aos, gravity, dt); }
    let aos_time = start.elapsed();

    let start = std::time::Instant::now();
    for _ in 0..1000 { soa_step(&mut soa.positions, &mut soa.velocities, gravity, dt); }
    let soa_time = start.elapsed();

    println!("=== AoS vs SoA Particle Layout Benchmark ===");
    println!("Particles: {}", N);
    println!("Integration (1000 iterations):");
    println!("  AoS: {:?} ({:.1} ns/particle/iter)", aos_time, aos_time.as_nanos() as f64 / (N as f64 * 1000.0));
    println!("  SoA: {:?} ({:.1} ns/particle/iter)", soa_time, soa_time.as_nanos() as f64 / (N as f64 * 1000.0));
    println!("  Ratio: {:.2}x", aos_time.as_nanos() as f64 / soa_time.as_nanos().max(1) as f64);

    // Small density benchmark (N=100)
    let n_small = 100;
    let aos_small: Vec<ParticleAoS> = (0..n_small).map(|i| ParticleAoS {
        position: Vec3 { x: (i % 10) as Real * 0.1, y: (i / 10) as Real * 0.1, z: 0.0 },
        velocity: Vec3::ZERO,
    }).collect();
    let pos_small: Vec<Vec3> = (0..n_small).map(|i| Vec3 { x: (i % 10) as Real * 0.1, y: (i / 10) as Real * 0.1, z: 0.0 }).collect();

    let start = std::time::Instant::now();
    let _d_aos = aos_density(&aos_small, h2);
    let aos_d_time = start.elapsed();

    let start = std::time::Instant::now();
    let _d_soa = soa_density(&pos_small, h2);
    let soa_d_time = start.elapsed();

    println!("\nDensity O(n²) ({} particles, 1 run):", n_small);
    println!("  AoS: {:?}", aos_d_time);
    println!("  SoA: {:?}", soa_d_time);
    println!("  Ratio: {:.2}x", aos_d_time.as_nanos() as f64 / soa_d_time.as_nanos().max(1) as f64);
}
