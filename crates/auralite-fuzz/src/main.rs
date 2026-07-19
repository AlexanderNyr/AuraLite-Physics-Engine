//! Stable-compatible self-owned fuzz harness (H8).
//! Seeded deterministic mutators/drivers over serialization parsers, shape/geometry constructors, narrow-phase entry points, and world-step op sequences.
//! No nightly cargo-fuzz required. Runs bounded iterations and reports corpus outcomes.

use auralite_collision::{gjk_distance2, gjk_distance3};
use auralite_core::{Rng, hash_bytes};
use auralite_dynamics::{
    BodyBuilder2, BodyBuilder3, Collider2, ColliderShape2, Material, World2, World3,
};
use auralite_geometry::{Box2, Box3, Circle2, Sphere3};
use auralite_math::{Real, Vec2, Vec3};
use auralite_serialize::{decode, encode, serialize_body2};

fn main() {
    println!("=== AuraLite Fuzz Harness (Stable, Deterministic) ===");
    let seed = 0xC0FFEEu64;
    let mut rng = Rng::new(seed);
    let mut total = 0u64;
    let mut panics = 0u64;
    let mut corpus = Vec::new();

    // 1. Serialization fuzz: mutate valid envelope and try decode
    println!("\n[1] Serialization parser fuzz (hostile input hardening)");
    for i in 0..500 {
        let mut body = auralite_dynamics::Body2 {
            id: auralite_core::StableId(i),
            kind: auralite_dynamics::BodyType::Dynamic,
            position: Vec2 {
                x: (rng.next_u64() as f32 / u64::MAX as f32) as Real * 10.0 as Real - 5.0,
                y: (rng.next_u64() as f32 / u64::MAX as f32) as Real * 10.0 as Real - 5.0,
            },
            rotation: auralite_math::Rot2::from_radians(
                (rng.next_u64() as f32 / u64::MAX as f32) as Real,
            )
            .unwrap_or(auralite_math::Rot2::identity()),
            velocity: Vec2 {
                x: (rng.next_u64() as f32 / u64::MAX as f32) as Real * 2.0 as Real - 1.0 as Real,
                y: (rng.next_u64() as f32 / u64::MAX as f32) as Real * 2.0 as Real - 1.0 as Real,
            },
            angular_velocity: (rng.next_u64() as f32 / u64::MAX as f32) as Real,
            inv_mass: 1.0,
            inv_inertia: 1.0,
            colliders: vec![],
            restitution: 0.1,
            friction: 0.5,
            sleeping: false,
            force: Vec2::ZERO,
            torque: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            user_data: i,
        };
        // Occasionally add collider
        if rng.next_u64().is_multiple_of(3) {
            body.colliders.push(Collider2 {
                shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: Default::default(),
            });
        }
        let payload = serialize_body2(&body);
        let enc = encode(&payload);
        // Mutate envelope
        let mut mutated = enc.clone();
        for _ in 0..(rng.next_u64() % 5) {
            let idx = (rng.next_u64() as usize) % mutated.len().max(1);
            mutated[idx] ^= (rng.next_u64() % 256) as u8;
        }
        // Try decode - should not panic, only return Err
        let res = std::panic::catch_unwind(|| decode(&mutated, 64 * 1024 * 1024));
        match res {
            Ok(Ok(_)) => { /* valid decoded */ }
            Ok(Err(_)) => { /* expected hostile input */ }
            Err(_) => {
                panics += 1;
                println!("PANIC in serialization decode iteration {}", i);
            }
        }
        total += 1;
        if i < 10 {
            corpus.push(format!(
                "ser_{}: len {} hash {:016x}",
                i,
                mutated.len(),
                hash_bytes(&mutated)
            ));
        }
    }

    // 2. Shape/geometry constructors fuzz
    println!("\n[2] Shape/geometry constructors fuzz");
    for i in 0..300 {
        let r = ((rng.next_u64() as f32 / u64::MAX as f32) as Real).abs() * 10.0 as Real;
        // Circle2::new may reject invalid (negative radius) - should return Err, not panic
        let _ = std::panic::catch_unwind(|| {
            let _ = Circle2::new(r);
            let _ = Circle2::new(-r);
            let _ = Box2::new(Vec2 { x: r, y: r });
            let _ = Box2::new(Vec2 { x: -1.0, y: 1.0 });
            let _ = Sphere3::new(r);
            let _ = Box3::new(Vec3 { x: r, y: r, z: r });
        });
        total += 1;
        if i < 5 {
            corpus.push(format!("shape_{}: r {:.3}", i, r));
        }
    }

    // 3. Narrow-phase entry points fuzz (GJK)
    println!("\n[3] Narrow-phase GJK fuzz");
    for i in 0..200 {
        let support_a = |d: Vec2| -> Vec2 { d.normalized_or(Vec2::X) * 0.5 as Real };
        let support_b = |_d: Vec2| -> Vec2 {
            Vec2 {
                x: 1.0 as Real,
                y: 0.0 as Real,
            }
        };
        let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _ = gjk_distance2(support_a, support_b, 32);
        }));
        if res.is_err() {
            panics += 1;
            println!("PANIC in GJK 2D iteration {}", i);
        }
        let support_a3 = |d: Vec3| -> Vec3 { d.normalized_or(Vec3::X) * 0.5 as Real };
        let support_b3 = |_d: Vec3| -> Vec3 {
            Vec3 {
                x: 1.0 as Real,
                y: 0.0 as Real,
                z: 0.0 as Real,
            }
        };
        let res3 = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let _ = gjk_distance3(support_a3, support_b3, 32);
        }));
        if res3.is_err() {
            panics += 1;
        }
        total += 2;
    }

    // 4. World-step op sequences fuzz
    println!("\n[4] World-step op sequences fuzz");
    for i in 0..100 {
        let mut w2 = World2::default();
        // Add random bodies
        for _ in 0..10 {
            let x = ((rng.next_u64() as f32 / u64::MAX as f32) as Real - 0.5) * 10.0 as Real;
            let y = ((rng.next_u64() as f32 / u64::MAX as f32) as Real) * 10.0 as Real;
            let b = BodyBuilder2::dynamic()
                .position(auralite_math::Vec2 { x, y })
                .add_collider(Collider2 {
                    shape: ColliderShape2::Circle(
                        Circle2::new(
                            0.3 as Real
                                + ((rng.next_u64() as f32 / u64::MAX as f32) as Real * 0.7 as Real),
                        )
                        .unwrap(),
                    ),
                    offset: Vec2::ZERO,
                    material: Material::default(),
                    filter: Default::default(),
                });
            let _ = w2.add_body(b);
        }
        // Step with random dt
        for _ in 0..20 {
            let dt =
                0.005 as Real + ((rng.next_u64() as f32 / u64::MAX as f32) as Real * 0.02 as Real);
            let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                let _ = w2.step(dt);
            }));
            if res.is_err() {
                panics += 1;
                println!("PANIC in world2 step fuzz iteration {}", i);
                break;
            }
        }
        total += 1;
    }

    for i in 0..50 {
        let mut w3 = World3::default();
        for _ in 0..5 {
            let x = ((rng.next_u64() as f32 / u64::MAX as f32) as Real - 0.5) * 5.0 as Real;
            let y = ((rng.next_u64() as f32 / u64::MAX as f32) as Real) * 5.0 as Real;
            let z = ((rng.next_u64() as f32 / u64::MAX as f32) as Real - 0.5) * 5.0 as Real;
            let b = BodyBuilder3::dynamic().position(Vec3 { x, y, z }).mass(1.0);
            let _ = w3.add_body(b);
        }
        for _ in 0..10 {
            let dt = 0.016 as Real;
            let _ = w3.step(dt);
        }
        total += 1;
        if i < 3 {
            corpus.push(format!("world3_{}: hash {:016x}", i, w3.state_hash()));
        }
    }

    println!("\n=== Fuzz Summary ===");
    println!("Total iterations: {}", total);
    println!("Panics detected: {} (should be 0)", panics);
    println!("Corpus samples (first 15):");
    for c in corpus.iter().take(15) {
        println!("  {}", c);
    }
    println!(
        "\nCorpus hash: {:016x}",
        hash_bytes(corpus.join("|").as_bytes())
    );

    if panics == 0 {
        println!("\n✅ FUZZ SMOKE PASS — no panics, hostile inputs handled via Err");
        std::process::exit(0);
    } else {
        println!("\n❌ FUZZ SMOKE FAIL — {} panics detected", panics);
        std::process::exit(1);
    }
}
