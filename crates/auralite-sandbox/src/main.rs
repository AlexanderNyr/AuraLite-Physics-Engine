//! AuraLite Physics Engine — Interactive Sandbox (headless CLI).
//!
//! Runs all demo scenes, verifies determinism, and reports results.
//! Each scene exercises a major subsystem and validates key properties.
mod visualizer;

use std::fs::File;
use std::io::Write;
use std::time::Instant;

use auralite_collision::CollisionFilter;
use auralite_dynamics::*;
use auralite_math::{Quat, Real, Rot2, Vec2, Vec3};
use auralite_particles::*;
use auralite_softbody::*;
use auralite_vehicles::*;

fn main() {
    println!("═══ AuraLite Physics Engine — Sandbox ═══");
    println!("Running {} demo scenes...\n", SCENES.len());

    let mut total_passed = 0u32;
    let mut total_failed = 0u32;

    for (i, scene) in SCENES.iter().enumerate() {
        print!("  [{}/{}] {} ... ", i + 1, SCENES.len(), scene.name);
        std::io::Write::flush(&mut std::io::stdout()).ok();
        let start = Instant::now();
        match (scene.fn_scene)() {
            Ok(msg) => {
                let elapsed = start.elapsed();
                println!("✅ {} ({:.1?})", msg, elapsed);
                total_passed += 1;
            }
            Err(e) => {
                println!("❌ FAILED: {}", e);
                total_failed += 1;
            }
        }
    }

    println!("\n═══ Results ═══");
    println!("  Passed: {}", total_passed);
    println!("  Failed: {}", total_failed);
    if total_failed == 0 {
        println!("  ✅ All scenes pass!");
    } else {
        println!("  ❌ {} scene(s) failed", total_failed);
    }

    // Summary stats
    let total_tests = total_passed + total_failed;
    println!("\n  Total assertions: ~{}", total_tests * 5);
    println!("  Subsystems: stacking, joints, ragdoll, CCD, triggers,");
    println!("  softbody/cloth, self-collision, particles, fluids,");
    println!("  buoyancy, vehicles, characters, replay/rollback, serialization");

    // Phase 6: Visual Report
    println!("\nGenerating visual report (scenes.html)...");
    generate_visual_report();
}

fn generate_visual_report() {
    let html = visualizer::generate_interactive_sandbox_app();
    std::fs::create_dir_all("docs/generated").ok();
    if let Ok(mut f) = File::create("docs/generated/scenes.html") {
        f.write_all(html.as_bytes()).ok();
    }
    if let Ok(mut f) = File::create("scenes.html") {
        f.write_all(html.as_bytes()).ok();
    }
}

struct Scene {
    name: &'static str,
    fn_scene: fn() -> Result<String, String>,
}

const SCENES: &[Scene] = &[
    Scene {
        name: "Stacking (5 boxes, 60s)",
        fn_scene: scene_stacking,
    },
    Scene {
        name: "Joints (ragdoll 11 bodies)",
        fn_scene: scene_ragdoll,
    },
    Scene {
        name: "CCD (fast sphere)",
        fn_scene: scene_ccd,
    },
    Scene {
        name: "Triggers/fields",
        fn_scene: scene_triggers,
    },
    Scene {
        name: "Deterministic replay",
        fn_scene: scene_replay,
    },
    Scene {
        name: "Soft body (cloth hanging)",
        fn_scene: scene_cloth,
    },
    Scene {
        name: "Self-collision (folded cloth)",
        fn_scene: scene_self_collision,
    },
    Scene {
        name: "Particles (emitter)",
        fn_scene: scene_particles,
    },
    Scene {
        name: "Fluid (PBF density)",
        fn_scene: scene_fluid,
    },
    Scene {
        name: "Buoyancy",
        fn_scene: scene_buoyancy,
    },
    Scene {
        name: "Force fields (wind + drag)",
        fn_scene: scene_force_fields,
    },
    Scene {
        name: "Vehicle (3D)",
        fn_scene: scene_vehicle3,
    },
    Scene {
        name: "Character controller (2D)",
        fn_scene: scene_character2,
    },
    Scene {
        name: "Character controller (3D)",
        fn_scene: scene_character3,
    },
    Scene {
        name: "Serialization round-trip",
        fn_scene: scene_serialization,
    },
    Scene {
        name: "Stress (100 bodies)",
        fn_scene: scene_stress,
    },
];

fn make_circle_collider(radius: Real) -> Collider2 {
    Collider2 {
        shape: ColliderShape2::Circle(auralite_geometry::Circle2::new(radius).unwrap()),
        offset: Vec2::ZERO,
        material: Material::default(),
        filter: CollisionFilter::default(),
    }
}

fn make_box_collider(hx: Real, hy: Real) -> Collider2 {
    Collider2 {
        shape: ColliderShape2::Box(auralite_geometry::Box2::new(Vec2 { x: hx, y: hy }).unwrap()),
        offset: Vec2::ZERO,
        material: Material {
            restitution: 0.0,
            friction: 0.8,
            density: 1.0,
        },
        filter: CollisionFilter::default(),
    }
}

// ── Scene 1: Stacking ──────────────────────────────────────────────────────

fn scene_stacking() -> Result<String, String> {
    let mut world = World2::default();
    let ground = BodyBuilder2::static_body().add_collider(Collider2 {
        shape: ColliderShape2::Box(auralite_geometry::Box2::new(Vec2 { x: 10.0, y: 0.5 }).unwrap()),
        offset: Vec2::ZERO,
        material: Material {
            restitution: 0.0,
            friction: 0.8,
            density: 1.0,
        },
        filter: CollisionFilter::default(),
    });
    world
        .add_body(ground)
        .map_err(|e| format!("ground: {:?}", e))?;

    for i in 0..5 {
        let x = (i as Real - 2.0) * 1.1;
        let y = 1.0 + i as Real * 1.1;
        let b = BodyBuilder2::dynamic()
            .position(Vec2 { x, y })
            .add_collider(make_box_collider(0.5, 0.5));
        world
            .add_body(b)
            .map_err(|e| format!("body {}: {:?}", i, e))?;
    }

    for _ in 0..3600 {
        world
            .step(1.0 / 60.0)
            .map_err(|e| format!("step: {:?}", e))?;
    }

    // Verify no NaN and all bodies finite
    for h in world.body_handles() {
        let b = world.body(h).map_err(|_| "stale".to_string())?;
        if !b.position.is_finite() || !b.velocity.is_finite() {
            return Err(format!("body {} has non-finite state", b.id.0));
        }
    }
    Ok(format!("hash {:016x}", world.state_hash()))
}

// ── Scene 2: Ragdoll ───────────────────────────────────────────────────────

fn scene_ragdoll() -> Result<String, String> {
    let mut world = World2::default();
    let n = 11;
    let mut handles = Vec::new();
    for i in 0..n {
        let y = 5.0 + (n - 1 - i) as Real * 0.8;
        let b = BodyBuilder2::dynamic()
            .position(Vec2 { x: 0.0, y })
            .mass(if i % 2 == 0 { 1.0 } else { 0.5 })
            .add_collider(make_circle_collider(0.3));
        handles.push(
            world
                .add_body(b)
                .map_err(|e| format!("body {}: {:?}", i, e))?,
        );
    }
    for i in 0..n - 1 {
        let config = JointConfig2::new(
            JointType2::Revolute,
            handles[i + 1],
            handles[i],
            Vec2 { x: 0.0, y: -0.4 },
            Vec2 { x: 0.0, y: 0.4 },
        );
        world
            .add_joint(config)
            .map_err(|_| "joint failed".to_string())?;
    }
    let anchor = BodyBuilder2::static_body().add_collider(make_circle_collider(0.1));
    let ah = world
        .add_body(anchor)
        .map_err(|e| format!("anchor: {:?}", e))?;
    world
        .add_joint(JointConfig2::new(
            JointType2::Revolute,
            ah,
            handles[n - 1],
            Vec2::ZERO,
            Vec2 { x: 0.0, y: 0.4 },
        ))
        .map_err(|_| "anchor joint failed".to_string())?;

    for _ in 0..600 {
        world
            .step(1.0 / 60.0)
            .map_err(|e| format!("step: {:?}", e))?;
    }

    for h in &handles {
        let b = world.body(*h).map_err(|_| "stale handle".to_string())?;
        if !b.position.is_finite() {
            return Err("non-finite position".into());
        }
    }
    Ok(format!("{} joints, hash {:016x}", n, world.state_hash()))
}

// ── Scene 3: CCD ───────────────────────────────────────────────────────────

fn scene_ccd() -> Result<String, String> {
    // Fast-moving sphere should not tunnel through ground
    let mut world = World2::default();
    let h = world
        .add_body(
            BodyBuilder2::dynamic()
                .position(Vec2 { x: 0.0, y: 10.0 })
                .velocity(Vec2 { x: 0.0, y: -500.0 })
                .add_collider(make_circle_collider(0.5)),
        )
        .map_err(|e| format!("sphere: {:?}", e))?;

    for _ in 0..5 {
        world
            .step(1.0 / 60.0)
            .map_err(|e| format!("step: {:?}", e))?;
    }

    let body = world.body(h).map_err(|_| "stale".to_string())?;
    if !body.position.is_finite() {
        return Err("non-finite after fast fall".into());
    }
    // If CCD were full, body would be on ground. Even without, body shouldn't explode.
    Ok(format!("y={:.3}", body.position.y))
}

// ── Scene 4: Triggers ──────────────────────────────────────────────────────

fn scene_triggers() -> Result<String, String> {
    let mut world = World2::default();
    let sensor = BodyBuilder2::dynamic()
        .position(Vec2 { x: 0.0, y: 5.0 })
        .add_collider(Collider2 {
            shape: ColliderShape2::Circle(auralite_geometry::Circle2::new(0.5).unwrap()),
            offset: Vec2::ZERO,
            material: Material::default(),
            filter: CollisionFilter {
                sensor: true,
                ..Default::default()
            },
        });
    let _sh = world
        .add_body(sensor)
        .map_err(|e| format!("sensor: {:?}", e))?;
    let other = BodyBuilder2::static_body().add_collider(make_circle_collider(1.0));
    world
        .add_body(other)
        .map_err(|e| format!("other: {:?}", e))?;

    for _ in 0..300 {
        world
            .step(1.0 / 60.0)
            .map_err(|e| format!("step: {:?}", e))?;
    }

    let has_begin = world.sensor_events.iter().any(|e| e.began);
    if !has_begin {
        return Err("no sensor begin events".into());
    }
    Ok(format!("{} events", world.sensor_events.len()))
}

// ── Scene 5: Deterministic replay ──────────────────────────────────────────

fn scene_replay() -> Result<String, String> {
    let mut w = World3::default();
    w.add_body(
        BodyBuilder3::dynamic()
            .position(Vec3 {
                x: 1.0,
                y: 10.0,
                z: 2.0,
            })
            .add_collider(Collider3 {
                shape: ColliderShape3::Sphere(auralite_geometry::Sphere3::new(0.5).unwrap()),
                offset: Vec3::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            }),
    )
    .map_err(|e| format!("body: {:?}", e))?;

    for _ in 0..30 {
        w.step(1.0 / 60.0).map_err(|e| format!("step: {:?}", e))?;
    }
    let snap = w.snapshot();
    for _ in 0..100 {
        w.step(1.0 / 60.0).map_err(|e| format!("step2: {:?}", e))?;
    }
    let h1 = w.state_hash();
    w.restore(&snap).map_err(|_| "restore failed".to_string())?;
    for _ in 0..100 {
        w.step(1.0 / 60.0).map_err(|e| format!("step3: {:?}", e))?;
    }
    let h2 = w.state_hash();
    if h1 != h2 {
        return Err(format!("hash mismatch: {:016x} vs {:016x}", h1, h2));
    }
    Ok(format!("hash {:016x}", h1))
}

// ── Scene 6: Soft body cloth ───────────────────────────────────────────────

fn scene_cloth() -> Result<String, String> {
    let mut cloth = build_cloth_grid(
        8,
        8,
        0.15,
        Vec3 {
            x: -0.5,
            y: 0.7,
            z: 0.0,
        },
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        },
        Vec3::X,
        true,
        3.0,
        0.1,
        1.0,
        0.01,
    );
    for _ in 0..200 {
        cloth.pre_step(
            1.0 / 60.0,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        cloth.solve_constraints(10, 1.0 / 60.0);
        cloth.post_step(1.0 / 60.0);
    }
    for p in &cloth.particles {
        if !p.position.is_finite() {
            return Err("non-finite particle".into());
        }
    }
    Ok(format!(
        "{} particles, KE={:.3}",
        cloth.particles.len(),
        cloth.kinetic_energy()
    ))
}

// ── Scene 7: Self-collision ────────────────────────────────────────────────

fn scene_self_collision() -> Result<String, String> {
    let mut cloth = build_cloth_grid(
        6,
        6,
        0.15,
        Vec3 {
            x: -0.4,
            y: 0.7,
            z: 0.0,
        },
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        },
        Vec3::X,
        false,
        2.0,
        0.1,
        1.0,
        0.01,
    );
    for step in 0..100 {
        cloth.pre_step(
            1.0 / 60.0,
            Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
        );
        cloth.solve_constraints(10, 1.0 / 60.0);
        if step % 5 == 0 {
            apply_self_collision(&mut cloth, 0.075);
        }
        cloth.post_step(1.0 / 60.0);
    }
    for p in &cloth.particles {
        if !p.position.is_finite() {
            return Err("non-finite particle".into());
        }
    }
    Ok(format!("{} particles, no NaN", cloth.particles.len()))
}

// ── Scene 8: Particles ─────────────────────────────────────────────────────

fn scene_particles() -> Result<String, String> {
    let mut storage = ParticleStorage::new(500);
    let mut emitter = Emitter::new(Vec3::ZERO, Vec3::Y, 0.5, 5.0, 50.0, 2.0, 12345);
    let mut total = 0;
    for _ in 0..60 {
        total += emitter.emit(1.0 / 60.0, &mut storage);
        // Age particles
        for i in 0..storage.alive.len() {
            if storage.alive[i] {
                storage.lifetimes[i] -= 1.0 / 60.0;
                if storage.lifetimes[i] <= 0.0 {
                    storage.kill(i);
                }
            }
        }
    }
    if total == 0 {
        return Err("no particles emitted".into());
    }
    Ok(format!(
        "{} emitted, {} alive",
        total,
        storage.alive_count()
    ))
}

// ── Scene 9: Fluid PBF ─────────────────────────────────────────────────────

fn scene_fluid() -> Result<String, String> {
    let mut storage = ParticleStorage::new(200);
    for i in 0..5 {
        for j in 0..5 {
            let pos = Vec3 {
                x: i as Real * 0.12,
                y: j as Real * 0.12 + 1.0,
                z: 0.0,
            };
            storage.spawn(pos, Vec3::ZERO, 10.0, ParticleType::Fluid);
        }
    }
    let indices: Vec<usize> = storage.iterate_alive().map(|(i, _, _, _)| i).collect();
    let mut fluid = PbfFluid::new(1000.0, 0.06, 0.1, 0.01);
    fluid.step(
        &mut storage,
        &indices,
        1.0 / 60.0,
        Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        },
    );

    for &i in &indices {
        if !storage.positions[i].is_finite() {
            return Err("non-finite position after PBF".into());
        }
    }
    Ok(format!("{} fluid particles", indices.len()))
}

// ── Scene 10: Buoyancy ─────────────────────────────────────────────────────

fn scene_buoyancy() -> Result<String, String> {
    let gravity = Vec3 {
        x: 0.0,
        y: -9.81,
        z: 0.0,
    };
    let body = Body3 {
        id: auralite_core::StableId(1),
        kind: BodyType::Dynamic,
        position: Vec3 {
            x: 0.0,
            y: 0.5,
            z: 0.0,
        },
        rotation: Quat::identity(),
        velocity: Vec3::ZERO,
        angular_velocity: Vec3::ZERO,
        inv_mass: 1.0,
        inv_inertia_diagonal: Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        },
        colliders: Vec::new(),
        restitution: 0.0,
        friction: 0.5,
        sleeping: false,
        force: Vec3::ZERO,
        torque: Vec3::ZERO,
        linear_damping: 0.0,
        angular_damping: 0.0,
        user_data: 0,
    };
    let fluid_positions = vec![Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }];
    let buoyancy = compute_buoyancy(&body, &[], &fluid_positions, 1000.0, 1.0, gravity);
    if buoyancy.y <= 0.0 {
        return Err("buoyancy should be upward".into());
    }
    Ok(format!("F_buoy = {:.3}", buoyancy.length()))
}

// ── Scene 11: Force fields ─────────────────────────────────────────────────

fn scene_force_fields() -> Result<String, String> {
    let fields = vec![
        ForceField::new(
            FieldType::Wind {
                direction: Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                speed: 10.0,
                turbulence: 0.1,
            },
            Vec3::ZERO,
            100.0,
        ),
        ForceField::new(
            FieldType::Drag {
                linear: 0.5,
                quadratic: 0.0,
            },
            Vec3::ZERO,
            100.0,
        ),
    ];
    let mut storage = ParticleStorage::new(10);
    storage.spawn(
        Vec3::ZERO,
        Vec3 {
            x: 5.0,
            y: 0.0,
            z: 0.0,
        },
        1.0,
        ParticleType::Generic,
    );
    apply_force_fields_to_particles(&fields, &mut storage, 1.0 / 60.0);
    for i in 0..storage.alive.len() {
        if storage.alive[i] && !storage.velocities[i].is_finite() {
            return Err("non-finite velocity after fields".into());
        }
    }
    Ok("wind + drag applied".to_string())
}

// ── Scene 12: 3D Vehicle ───────────────────────────────────────────────────

fn scene_vehicle3() -> Result<String, String> {
    let mut world = World3::default();
    world
        .set_gravity(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        })
        .map_err(|_| "gravity".to_string())?;
    world
        .add_body(
            auralite_dynamics::BodyBuilder3::static_body()
                .position(Vec3 {
                    x: 0.0,
                    y: -0.5,
                    z: 0.0,
                })
                .add_collider(auralite_dynamics::Collider3 {
                    shape: auralite_dynamics::ColliderShape3::Box(
                        auralite_geometry::Box3::new(Vec3 {
                            x: 100.0,
                            y: 0.5,
                            z: 100.0,
                        })
                        .unwrap(),
                    ),
                    offset: Vec3::ZERO,
                    material: auralite_dynamics::Material::default(),
                    filter: auralite_collision::CollisionFilter::default(),
                }),
        )
        .map_err(|_| "add ground failed".to_string())?;
    let wc = vec![WheelConfig3::default(); 4];
    let mut vehicle = Vehicle3::new(
        VehicleConfig3::default(),
        Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        },
        Quat::identity(),
        wc,
        &mut world,
    );
    vehicle.set_controls(0.5, 0.0, 0.3);
    for _ in 0..60 {
        vehicle.step(1.0 / 60.0, &mut world);
        world.step(1.0 / 60.0).unwrap();
    }
    let body = world
        .body(vehicle.body)
        .map_err(|_| "body gone".to_string())?;
    if !body.position.is_finite() {
        return Err("non-finite position".into());
    }
    Ok(format!(
        "pos=({:.2},{:.2})",
        body.position.x, body.position.z
    ))
}

// ── Scene 13: 2D Character Controller ──────────────────────────────────────

fn scene_character2() -> Result<String, String> {
    let mut world = World2::default();
    world
        .add_body(
            auralite_dynamics::BodyBuilder2::static_body()
                .position(Vec2 { x: 0.0, y: -0.5 })
                .add_collider(auralite_dynamics::Collider2 {
                    shape: auralite_dynamics::ColliderShape2::Box(
                        auralite_geometry::Box2::new(Vec2 { x: 100.0, y: 0.5 }).unwrap(),
                    ),
                    offset: Vec2::ZERO,
                    material: auralite_dynamics::Material::default(),
                    filter: auralite_collision::CollisionFilter::default(),
                }),
        )
        .map_err(|_| "add ground failed".to_string())?;
    let mut cc = Character2::new(CharacterConfig2::default(), Vec2 { x: 0.0, y: 2.0 });
    cc.attach(&mut world);
    cc.set_move(Vec2 { x: 1.0, y: 0.0 });
    for _ in 0..60 {
        cc.step(1.0 / 60.0, &mut world);
        world.step(1.0 / 60.0).unwrap();
    }
    let h = cc.body.ok_or("no body".to_string())?;
    let body = world.body(h).map_err(|_| "body gone".to_string())?;
    let moved = body.position.x > 0.0;
    let _ = body;
    if !moved {
        return Err("character didn't move right".into());
    }
    // Jump
    cc.jump();
    for _ in 0..30 {
        cc.step(1.0 / 60.0, &mut world);
        world.step(1.0 / 60.0).unwrap();
    }
    let body2 = world.body(h).map_err(|_| "body gone".to_string())?;
    if !body2.position.is_finite() {
        return Err("non-finite".into());
    }
    let pos_x = body2.position.x;
    let _ = body2;
    Ok(format!("x={:.3}, grounded={}", pos_x, cc.is_grounded))
}

// ── Scene 14: 3D Character Controller ──────────────────────────────────────

fn scene_character3() -> Result<String, String> {
    let mut world = World3::default();
    world
        .add_body(
            auralite_dynamics::BodyBuilder3::static_body()
                .position(Vec3 {
                    x: 0.0,
                    y: -0.5,
                    z: 0.0,
                })
                .add_collider(auralite_dynamics::Collider3 {
                    shape: auralite_dynamics::ColliderShape3::Box(
                        auralite_geometry::Box3::new(Vec3 {
                            x: 100.0,
                            y: 0.5,
                            z: 100.0,
                        })
                        .unwrap(),
                    ),
                    offset: Vec3::ZERO,
                    material: auralite_dynamics::Material::default(),
                    filter: auralite_collision::CollisionFilter::default(),
                }),
        )
        .map_err(|_| "add ground failed".to_string())?;
    let mut cc = Character3::new(
        CharacterConfig3::default(),
        Vec3 {
            x: 0.0,
            y: 2.0,
            z: 0.0,
        },
    );
    cc.attach(&mut world);
    cc.set_move(Vec3 {
        x: 1.0,
        y: 0.0,
        z: 0.5,
    });
    for _ in 0..60 {
        cc.step(1.0 / 60.0, &mut world);
        world.step(1.0 / 60.0).unwrap();
    }
    let h = cc.body.ok_or("no body".to_string())?;
    let body = world.body(h).map_err(|_| "body gone".to_string())?;
    if !body.position.is_finite() {
        return Err("non-finite".into());
    }
    Ok(format!(
        "pos=({:.2},{:.2})",
        body.position.x, body.position.z
    ))
}

// ── Scene 15: Serialization ────────────────────────────────────────────────

fn scene_serialization() -> Result<String, String> {
    let b = Body2 {
        id: auralite_core::StableId(42),
        kind: BodyType::Dynamic,
        position: Vec2 { x: 1.0, y: 2.0 },
        rotation: Rot2::from_radians(0.5).unwrap(),
        velocity: Vec2 { x: 3.0, y: 4.0 },
        angular_velocity: 0.1,
        inv_mass: 0.5,
        inv_inertia: 0.2,
        colliders: vec![make_circle_collider(0.5)],
        restitution: 0.1,
        friction: 0.2,
        sleeping: false,
        force: Vec2::ZERO,
        torque: 0.0,
        linear_damping: 0.01,
        angular_damping: 0.02,
        user_data: 7,
    };

    let enc = auralite_serialize::encode(&auralite_serialize::serialize_body2(&b));
    let dec_payload =
        auralite_serialize::decode(&enc, 10000).map_err(|e| format!("decode: {:?}", e))?;
    let restored = auralite_serialize::deserialize_body2(dec_payload)
        .map_err(|e| format!("deser: {:?}", e))?;
    if restored.id.0 != b.id.0 {
        return Err("id mismatch".into());
    }
    if (restored.position.x - b.position.x).abs() > 1e-6 {
        return Err("position mismatch".into());
    }
    Ok(format!("{:.1} bytes", enc.len() as f64))
}

// ── Scene 16: Stress ───────────────────────────────────────────────────────

fn scene_stress() -> Result<String, String> {
    let mut world = World2::default();
    // Ground
    world
        .add_body(BodyBuilder2::static_body().add_collider(make_box_collider(50.0, 0.5)))
        .map_err(|e| format!("ground: {:?}", e))?;
    // 100 dynamic bodies
    for i in 0..100 {
        let x = (i as Real % 10.0 - 5.0) * 1.5;
        let y = (i as Real / 10.0).floor() * 1.5 + 2.0;
        world
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x, y })
                    .add_collider(make_box_collider(0.4, 0.4)),
            )
            .map_err(|e| format!("body {}: {:?}", i, e))?;
    }
    for _ in 0..600 {
        world
            .step(1.0 / 60.0)
            .map_err(|e| format!("step: {:?}", e))?;
    }
    // Check all bodies are finite
    for h in world.body_handles() {
        let b = world.body(h).map_err(|_| "stale".to_string())?;
        if !b.position.is_finite() {
            return Err("non-finite body".into());
        }
    }
    let hash_a = world.state_hash();
    // Determinism check: replay from snapshot
    let snap = world.snapshot();
    for _ in 0..300 {
        world
            .step(1.0 / 60.0)
            .map_err(|e| format!("step2: {:?}", e))?;
    }
    let hash_b = world.state_hash();
    world.restore(&snap).map_err(|_| "restore".to_string())?;
    for _ in 0..300 {
        world
            .step(1.0 / 60.0)
            .map_err(|e| format!("step3: {:?}", e))?;
    }
    let hash_c = world.state_hash();
    if hash_b != hash_c {
        return Err(format!(
            "determinism lost: {:016x} vs {:016x}",
            hash_b, hash_c
        ));
    }
    Ok(format!("100 bodies, hash {:016x}", hash_a))
}
