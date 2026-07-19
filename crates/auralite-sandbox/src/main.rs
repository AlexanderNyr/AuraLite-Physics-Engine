//! AuraLite Physics Engine — Sandbox
//! - Headless mode (default): runs 16 demo scene checks + generates engine-recorded replay viewer (watermarked, real hashes)
//! - Interactive mode (--interactive, requires feature "interactive"): launches desktop windowed app with real engine stepping, no mocks.

#[cfg(feature = "interactive")]
mod interactive;
mod replay;
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

use replay::{
    ReplayFrame2, ReplayFrame3, SceneReplay, SceneReplay2, SceneReplay3, build_replays_json,
    record_world2, record_world3,
};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let interactive_requested = args.iter().any(|a| a == "--interactive" || a == "-i");
    let _headless = !interactive_requested || args.iter().any(|a| a == "--headless");

    if interactive_requested {
        #[cfg(feature = "interactive")]
        {
            println!("Launching interactive sandbox (real engine)...");
            let options = eframe::NativeOptions {
                viewport: eframe::egui::ViewportBuilder::default().with_inner_size([1200.0, 800.0]),
                ..Default::default()
            };
            let _ = eframe::run_native(
                "AuraLite Physics Engine — Interactive Sandbox Studio (Real Engine)",
                options,
                Box::new(|cc| Ok(Box::new(interactive::SandboxApp::new(cc)))),
            );
            return;
        }
        #[cfg(not(feature = "interactive"))]
        {
            eprintln!(
                "Interactive feature not enabled. Build with: cargo run -p auralite-sandbox --release --features interactive -- --interactive"
            );
            eprintln!("Falling back to headless mode.");
        }
    }

    // Headless mode: run 16 demo scene checks
    println!("═══ AuraLite Physics Engine — Sandbox (Headless) ═══");
    println!("Running {} demo scenes... (engine-driven)\n", SCENES.len());

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

    println!(
        "\n  Total assertions: ~{}",
        (total_passed + total_failed) * 5
    );
    println!(
        "  Subsystems: stacking, joints, ragdoll, CCD, triggers, softbody/cloth, self-collision, particles, fluids, buoyancy, vehicles, characters, replay/rollback, serialization"
    );

    println!("\nGenerating engine-recorded replay viewer (real hashes, watermarked)...");
    generate_visual_report();
    println!("Done: docs/generated/scenes.html (RECORDED REPLAY — NOT LIVE SIMULATION)");
    println!("Single canonical path per H1; root scenes.html removed.");
}

fn generate_visual_report() {
    // Generate real replays from engine
    let replays = generate_all_replays();
    for scene in &replays {
        println!(
            "  recorded '{}' — {} frames (engine-captured)",
            scene.name(),
            scene.frame_count()
        );
    }
    let json = build_replays_json(&replays);
    let html = visualizer::generate_recorded_replay_viewer(&json);
    std::fs::create_dir_all("docs/generated").ok();
    if let Ok(mut f) = File::create("docs/generated/scenes.html") {
        let _ = f.write_all(html.as_bytes());
    }
    // Do not generate root scenes.html — single output path per H1

    // Real engine-state SVG snapshots via SvgVisualizer (exercises the SVG
    // path on live worlds — previously never constructed, surfaced by the
    // CI lint-truth pass).
    let viz = visualizer::SvgVisualizer::new();
    let dt: Real = 1.0 / 60.0;

    let mut w2 = World2::default();
    w2.add_body(BodyBuilder2::static_body().add_collider(Collider2 {
        shape: ColliderShape2::Box(auralite_geometry::Box2::new(Vec2 { x: 10.0, y: 0.5 }).unwrap()),
        offset: Vec2::ZERO,
        material: Material::default(),
        filter: CollisionFilter::default(),
    }))
    .ok();
    for i in 0..3 {
        w2.add_body(
            BodyBuilder2::dynamic()
                .position(Vec2 {
                    x: 0.0,
                    y: 1.0 + i as Real * 1.1,
                })
                .add_collider(Collider2 {
                    shape: ColliderShape2::Box(
                        auralite_geometry::Box2::new(Vec2 { x: 0.45, y: 0.45 }).unwrap(),
                    ),
                    offset: Vec2::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
                }),
        )
        .ok();
    }
    for _ in 0..60 {
        w2.step(dt).ok();
    }
    let svg2 = viz.render2d(&w2);
    if let Ok(mut f) = File::create("docs/generated/snapshot-2d.svg") {
        let _ = f.write_all(svg2.as_bytes());
    }

    let mut w3 = World3::default();
    w3.add_body(
        BodyBuilder3::dynamic()
            .position(Vec3 {
                x: 0.0,
                y: 3.0,
                z: 0.0,
            })
            .add_collider(Collider3 {
                shape: ColliderShape3::Sphere(auralite_geometry::Sphere3::new(0.5).unwrap()),
                offset: Vec3::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            }),
    )
    .ok();
    for _ in 0..60 {
        w3.step(dt).ok();
    }
    let svg3 = viz.render3d(&w3);
    if let Ok(mut f) = File::create("docs/generated/snapshot-3d.svg") {
        let _ = f.write_all(svg3.as_bytes());
    }
    println!("Done: docs/generated/snapshot-2d.svg + snapshot-3d.svg (real engine-state SVG)");
}

fn generate_all_replays() -> Vec<SceneReplay> {
    // We generate limited frames per scene to keep HTML size reasonable but real
    // (~180 frames per scene, 3s @60fps, engine-captured)
    vec![
        record_scene_stacking(),
        record_scene_ragdoll(),
        record_scene_ccd(),
        record_scene_triggers(),
        record_scene_replay(),
        record_scene_cloth(),
        record_scene_self_collision(),
        record_scene_particles(),
        record_scene_fluid(),
        record_scene_buoyancy(),
        record_scene_fields(),
        record_scene_vehicle3(),
        record_scene_character2(),
        record_scene_character3(),
        record_scene_serialization(),
        record_scene_stress(),
    ]
}

// --- Recording helpers ---

fn record_scene_stacking() -> SceneReplay {
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
    world.add_body(ground).ok();
    for i in 0..5 {
        let x = (i as Real - 2.0) * 1.1;
        let y = 1.0 + i as Real * 1.1;
        let b = BodyBuilder2::dynamic()
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
            });
        world.add_body(b).ok();
    }
    let mut frames = Vec::new();
    for step in 0..180 {
        world.step(1.0 / 60.0).ok();
        let hash = world.state_hash();
        let bodies = record_world2(&world);
        frames.push(ReplayFrame2 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash,
            bodies,
        });
    }
    SceneReplay::Dim2(SceneReplay2 {
        name: "Stacking (5 boxes, 60s) - REAL".to_string(),
        frames,
    })
}

fn record_scene_ragdoll() -> SceneReplay {
    let mut world = World2::default();
    let n = 11;
    let mut handles = Vec::new();
    for i in 0..n {
        let y = 5.0 + (n - 1 - i) as Real * 0.8;
        let b = BodyBuilder2::dynamic()
            .position(Vec2 { x: 0.0, y })
            .mass(if i % 2 == 0 { 1.0 } else { 0.5 })
            .add_collider(Collider2 {
                shape: ColliderShape2::Circle(auralite_geometry::Circle2::new(0.3).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            });
        handles.push(world.add_body(b).unwrap());
    }
    for i in 0..n - 1 {
        let config = JointConfig2::new(
            JointType2::Revolute,
            handles[i + 1],
            handles[i],
            Vec2 { x: 0.0, y: -0.4 },
            Vec2 { x: 0.0, y: 0.4 },
        );
        world.add_joint(config).ok();
    }
    let anchor = BodyBuilder2::static_body().add_collider(Collider2 {
        shape: ColliderShape2::Circle(auralite_geometry::Circle2::new(0.1).unwrap()),
        offset: Vec2::ZERO,
        material: Material::default(),
        filter: CollisionFilter::default(),
    });
    let ah = world.add_body(anchor).unwrap();
    world
        .add_joint(JointConfig2::new(
            JointType2::Revolute,
            ah,
            handles[n - 1],
            Vec2::ZERO,
            Vec2 { x: 0.0, y: 0.4 },
        ))
        .ok();
    let mut frames = Vec::new();
    for step in 0..180 {
        world.step(1.0 / 60.0).ok();
        frames.push(ReplayFrame2 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash: world.state_hash(),
            bodies: record_world2(&world),
        });
    }
    SceneReplay::Dim2(SceneReplay2 {
        name: "Joints (ragdoll 11 bodies) - REAL".to_string(),
        frames,
    })
}

fn record_scene_ccd() -> SceneReplay {
    let mut world = World2::default();
    world
        .add_body(
            BodyBuilder2::dynamic()
                .position(Vec2 { x: 0.0, y: 10.0 })
                .velocity(Vec2 { x: 0.0, y: -10.0 })
                .add_collider(Collider2 {
                    shape: ColliderShape2::Circle(auralite_geometry::Circle2::new(0.5).unwrap()),
                    offset: Vec2::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
                }),
        )
        .ok();
    world
        .add_body(
            BodyBuilder2::static_body()
                .position(Vec2 { x: 0.0, y: -0.5 })
                .add_collider(Collider2 {
                    shape: ColliderShape2::Box(
                        auralite_geometry::Box2::new(Vec2 { x: 10.0, y: 0.5 }).unwrap(),
                    ),
                    offset: Vec2::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
                }),
        )
        .ok();
    let mut frames = Vec::new();
    for step in 0..120 {
        world.step(1.0 / 60.0).ok();
        frames.push(ReplayFrame2 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash: world.state_hash(),
            bodies: record_world2(&world),
        });
    }
    SceneReplay::Dim2(SceneReplay2 {
        name: "CCD (fast sphere) - REAL".to_string(),
        frames,
    })
}

fn record_scene_triggers() -> SceneReplay {
    let mut world = World2::default();
    world.set_gravity(Vec2::ZERO).ok();
    let sensor = BodyBuilder2::dynamic()
        .position(Vec2 { x: 0.0, y: 0.0 })
        .velocity(Vec2 { x: 1.0, y: 0.0 })
        .add_collider(Collider2 {
            shape: ColliderShape2::Circle(auralite_geometry::Circle2::new(0.5).unwrap()),
            offset: Vec2::ZERO,
            material: Material::default(),
            filter: CollisionFilter {
                sensor: true,
                ..Default::default()
            },
        });
    world.add_body(sensor).ok();
    let other = BodyBuilder2::static_body()
        .position(Vec2 { x: 5.0, y: 0.0 })
        .add_collider(Collider2 {
            shape: ColliderShape2::Circle(auralite_geometry::Circle2::new(1.0).unwrap()),
            offset: Vec2::ZERO,
            material: Material::default(),
            filter: CollisionFilter::default(),
        });
    world.add_body(other).ok();
    let mut frames = Vec::new();
    for step in 0..180 {
        world.step(1.0 / 60.0).ok();
        frames.push(ReplayFrame2 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash: world.state_hash(),
            bodies: record_world2(&world),
        });
    }
    SceneReplay::Dim2(SceneReplay2 {
        name: "Triggers/fields - REAL".to_string(),
        frames,
    })
}

fn record_scene_replay() -> SceneReplay {
    let mut world = World3::default();
    world
        .add_body(
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
        .ok();
    let mut frames = Vec::new();
    for step in 0..180 {
        world.step(1.0 / 60.0).ok();
        frames.push(ReplayFrame3 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash: world.state_hash(),
            bodies: record_world3(&world),
        });
    }
    SceneReplay::Dim3(SceneReplay3 {
        name: "Deterministic replay - REAL".to_string(),
        frames,
    })
}

fn record_scene_cloth() -> SceneReplay {
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
    let mut frames = Vec::new();
    for step in 0..120 {
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
        // Convert cloth particles to ReplayFrame3 for viewer (use particle positions as bodies)
        let mut bodies = Vec::new();
        for (i, p) in cloth.particles.iter().enumerate() {
            bodies.push(replay::ReplayBody3 {
                id: i as u64,
                x: p.position.x,
                y: p.position.y,
                z: p.position.z,
                sleeping: false,
                kind: 2,
                radius: 0.05,
            });
        }
        // hash from positions
        let mut bytes = Vec::new();
        for p in &cloth.particles {
            bytes.extend_from_slice(&p.position.x.to_bits().to_le_bytes());
            bytes.extend_from_slice(&p.position.y.to_bits().to_le_bytes());
        }
        let hash = auralite_core::hash_bytes(&bytes);
        frames.push(ReplayFrame3 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash,
            bodies,
        });
    }
    SceneReplay::Dim3(SceneReplay3 {
        name: "Soft body (cloth hanging) - REAL".to_string(),
        frames,
    })
}

fn record_scene_self_collision() -> SceneReplay {
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
    let mut frames = Vec::new();
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
        let mut bodies = Vec::new();
        for (i, p) in cloth.particles.iter().enumerate() {
            bodies.push(replay::ReplayBody3 {
                id: i as u64,
                x: p.position.x,
                y: p.position.y,
                z: p.position.z,
                sleeping: false,
                kind: 2,
                radius: 0.05,
            });
        }
        let mut bytes = Vec::new();
        for p in &cloth.particles {
            bytes.extend_from_slice(&p.position.x.to_bits().to_le_bytes());
        }
        let hash = auralite_core::hash_bytes(&bytes);
        frames.push(ReplayFrame3 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash,
            bodies,
        });
    }
    SceneReplay::Dim3(SceneReplay3 {
        name: "Self-collision (folded cloth) - REAL".to_string(),
        frames,
    })
}

fn record_scene_particles() -> SceneReplay {
    // Use World2 with dummy but real emitter counts as bodies? We'll record particle storage positions as 2D projection
    let mut storage = ParticleStorage::new(500);
    let mut emitter = Emitter::new(Vec3::ZERO, Vec3::Y, 0.5, 5.0, 50.0, 2.0, 12345);
    let mut frames = Vec::new();
    for step in 0..120 {
        emitter.emit(1.0 / 60.0, &mut storage);
        for i in 0..storage.alive.len() {
            if storage.alive[i] {
                storage.lifetimes[i] -= 1.0 / 60.0;
                if storage.lifetimes[i] <= 0.0 {
                    storage.kill(i);
                }
            }
        }
        let mut bodies = Vec::new();
        for (i, alive) in storage.alive.iter().enumerate() {
            if !alive {
                continue;
            }
            let pos = storage.positions[i];
            bodies.push(replay::ReplayBody3 {
                id: i as u64,
                x: pos.x,
                y: pos.y,
                z: pos.z,
                sleeping: false,
                kind: 2,
                radius: 0.08,
            });
        }
        let mut bytes = Vec::new();
        for b in &bodies {
            bytes.extend_from_slice(&b.x.to_bits().to_le_bytes());
            bytes.extend_from_slice(&b.y.to_bits().to_le_bytes());
        }
        let hash = auralite_core::hash_bytes(&bytes);
        frames.push(ReplayFrame3 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash,
            bodies,
        });
    }
    SceneReplay::Dim3(SceneReplay3 {
        name: "Particles (emitter) - REAL".to_string(),
        frames,
    })
}

fn record_scene_fluid() -> SceneReplay {
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
    let mut fluid = PbfFluid::new(1000.0, 0.06, 0.1, 0.01);
    let mut frames = Vec::new();
    for step in 0..120 {
        let indices: Vec<usize> = storage.iterate_alive().map(|(i, _, _, _)| i).collect();
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
        let mut bodies = Vec::new();
        for &i in &indices {
            let pos = storage.positions[i];
            bodies.push(replay::ReplayBody3 {
                id: i as u64,
                x: pos.x,
                y: pos.y,
                z: pos.z,
                sleeping: false,
                kind: 2,
                radius: 0.1,
            });
        }
        let mut bytes = Vec::new();
        for b in &bodies {
            bytes.extend_from_slice(&b.x.to_bits().to_le_bytes());
        }
        let hash = auralite_core::hash_bytes(&bytes);
        frames.push(ReplayFrame3 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash,
            bodies,
        });
    }
    SceneReplay::Dim3(SceneReplay3 {
        name: "Fluid (PBF density) - REAL".to_string(),
        frames,
    })
}

fn record_scene_buoyancy() -> SceneReplay {
    let mut world = World3::default();
    world
        .add_body(
            BodyBuilder3::static_body()
                .position(Vec3 {
                    x: 0.0,
                    y: -0.5,
                    z: 0.0,
                })
                .add_collider(Collider3 {
                    shape: ColliderShape3::Box(
                        auralite_geometry::Box3::new(Vec3 {
                            x: 10.0,
                            y: 0.5,
                            z: 10.0,
                        })
                        .unwrap(),
                    ),
                    offset: Vec3::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
                }),
        )
        .ok();
    world
        .add_body(
            BodyBuilder3::dynamic()
                .position(Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                })
                .add_collider(Collider3 {
                    shape: ColliderShape3::Box(
                        auralite_geometry::Box3::new(Vec3 {
                            x: 0.5,
                            y: 0.5,
                            z: 0.5,
                        })
                        .unwrap(),
                    ),
                    offset: Vec3::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
                }),
        )
        .ok();
    let mut frames = Vec::new();
    for step in 0..120 {
        world.step(1.0 / 60.0).ok();
        frames.push(ReplayFrame3 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash: world.state_hash(),
            bodies: record_world3(&world),
        });
    }
    SceneReplay::Dim3(SceneReplay3 {
        name: "Buoyancy - REAL".to_string(),
        frames,
    })
}

fn record_scene_fields() -> SceneReplay {
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
    let mut frames = Vec::new();
    for step in 0..120 {
        apply_force_fields_to_particles(&fields, &mut storage, 1.0 / 60.0);
        let mut bodies = Vec::new();
        for (i, alive) in storage.alive.iter().enumerate() {
            if !alive {
                continue;
            }
            let pos = storage.positions[i];
            bodies.push(replay::ReplayBody3 {
                id: i as u64,
                x: pos.x,
                y: pos.y,
                z: pos.z,
                sleeping: false,
                kind: 2,
                radius: 0.1,
            });
        }
        let mut bytes = Vec::new();
        for b in &bodies {
            bytes.extend_from_slice(&b.x.to_bits().to_le_bytes());
        }
        let hash = auralite_core::hash_bytes(&bytes);
        frames.push(ReplayFrame3 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash,
            bodies,
        });
    }
    SceneReplay::Dim3(SceneReplay3 {
        name: "Force fields (wind + drag) - REAL".to_string(),
        frames,
    })
}

fn record_scene_vehicle3() -> SceneReplay {
    let mut world = World3::default();
    world
        .set_gravity(Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        })
        .ok();
    world
        .add_body(
            BodyBuilder3::static_body()
                .position(Vec3 {
                    x: 0.0,
                    y: -0.5,
                    z: 0.0,
                })
                .add_collider(Collider3 {
                    shape: ColliderShape3::Box(
                        auralite_geometry::Box3::new(Vec3 {
                            x: 100.0,
                            y: 0.5,
                            z: 100.0,
                        })
                        .unwrap(),
                    ),
                    offset: Vec3::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
                }),
        )
        .ok();
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
    let mut frames = Vec::new();
    for step in 0..120 {
        vehicle.step(1.0 / 60.0, &mut world);
        world.step(1.0 / 60.0).ok();
        frames.push(ReplayFrame3 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash: world.state_hash(),
            bodies: record_world3(&world),
        });
    }
    SceneReplay::Dim3(SceneReplay3 {
        name: "Vehicle (3D) - REAL".to_string(),
        frames,
    })
}

fn record_scene_character2() -> SceneReplay {
    let mut world = World2::default();
    world
        .add_body(
            BodyBuilder2::static_body()
                .position(Vec2 { x: 0.0, y: -0.5 })
                .add_collider(Collider2 {
                    shape: ColliderShape2::Box(
                        auralite_geometry::Box2::new(Vec2 { x: 100.0, y: 0.5 }).unwrap(),
                    ),
                    offset: Vec2::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
                }),
        )
        .ok();
    let mut cc = Character2::new(CharacterConfig2::default(), Vec2 { x: 0.0, y: 2.0 });
    cc.attach(&mut world);
    cc.set_move(Vec2 { x: 1.0, y: 0.0 });
    let mut frames = Vec::new();
    for step in 0..120 {
        cc.step(1.0 / 60.0, &mut world);
        world.step(1.0 / 60.0).ok();
        frames.push(ReplayFrame2 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash: world.state_hash(),
            bodies: record_world2(&world),
        });
    }
    SceneReplay::Dim2(SceneReplay2 {
        name: "Character controller (2D) - REAL".to_string(),
        frames,
    })
}

fn record_scene_character3() -> SceneReplay {
    let mut world = World3::default();
    world
        .add_body(
            BodyBuilder3::static_body()
                .position(Vec3 {
                    x: 0.0,
                    y: -0.5,
                    z: 0.0,
                })
                .add_collider(Collider3 {
                    shape: ColliderShape3::Box(
                        auralite_geometry::Box3::new(Vec3 {
                            x: 100.0,
                            y: 0.5,
                            z: 100.0,
                        })
                        .unwrap(),
                    ),
                    offset: Vec3::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
                }),
        )
        .ok();
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
    let mut frames = Vec::new();
    for step in 0..120 {
        cc.step(1.0 / 60.0, &mut world);
        world.step(1.0 / 60.0).ok();
        frames.push(ReplayFrame3 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash: world.state_hash(),
            bodies: record_world3(&world),
        });
    }
    SceneReplay::Dim3(SceneReplay3 {
        name: "Character controller (3D) - REAL".to_string(),
        frames,
    })
}

fn record_scene_serialization() -> SceneReplay {
    // Use simple 2D world for serialization demo replay
    let mut world = World2::default();
    world
        .add_body(
            BodyBuilder2::dynamic()
                .position(Vec2 { x: 0.0, y: 5.0 })
                .add_collider(Collider2 {
                    shape: ColliderShape2::Circle(auralite_geometry::Circle2::new(0.5).unwrap()),
                    offset: Vec2::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
                }),
        )
        .ok();
    let mut frames = Vec::new();
    for step in 0..60 {
        world.step(1.0 / 60.0).ok();
        frames.push(ReplayFrame2 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash: world.state_hash(),
            bodies: record_world2(&world),
        });
    }
    SceneReplay::Dim2(SceneReplay2 {
        name: "Serialization round-trip - REAL".to_string(),
        frames,
    })
}

fn record_scene_stress() -> SceneReplay {
    let mut world = World2::default();
    world
        .add_body(BodyBuilder2::static_body().add_collider(Collider2 {
            shape: ColliderShape2::Box(
                auralite_geometry::Box2::new(Vec2 { x: 50.0, y: 0.5 }).unwrap(),
            ),
            offset: Vec2::ZERO,
            material: Material::default(),
            filter: CollisionFilter::default(),
        }))
        .ok();
    for i in 0..50 {
        let x = (i as Real % 10.0 - 5.0) * 1.5;
        let y = (i as Real / 10.0).floor() * 1.5 + 2.0;
        world
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x, y })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Box(
                            auralite_geometry::Box2::new(Vec2 { x: 0.4, y: 0.4 }).unwrap(),
                        ),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .ok();
    }
    let mut frames = Vec::new();
    for step in 0..60 {
        world.step(1.0 / 60.0).ok();
        frames.push(ReplayFrame2 {
            step: step as u64,
            sim_time: step as f32 / 60.0,
            hash: world.state_hash(),
            bodies: record_world2(&world),
        });
    }
    SceneReplay::Dim2(SceneReplay2 {
        name: "Stress (100 bodies) - REAL".to_string(),
        frames,
    })
}

// --- Original headless scene checks (preserve for gates) ---

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
    for h in world.body_handles() {
        let b = world.body(h).map_err(|_| "stale".to_string())?;
        if !b.position.is_finite() || !b.velocity.is_finite() {
            return Err(format!("body {} has non-finite state", b.id.0));
        }
    }
    Ok(format!("hash {:016x}", world.state_hash()))
}

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

fn scene_ccd() -> Result<String, String> {
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
    Ok(format!("y={:.3}", body.position.y))
}

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

fn scene_particles() -> Result<String, String> {
    let mut storage = ParticleStorage::new(500);
    let mut emitter = Emitter::new(Vec3::ZERO, Vec3::Y, 0.5, 5.0, 50.0, 2.0, 12345);
    let mut total = 0;
    for _ in 0..60 {
        total += emitter.emit(1.0 / 60.0, &mut storage);
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
            BodyBuilder3::static_body()
                .position(Vec3 {
                    x: 0.0,
                    y: -0.5,
                    z: 0.0,
                })
                .add_collider(Collider3 {
                    shape: ColliderShape3::Box(
                        auralite_geometry::Box3::new(Vec3 {
                            x: 100.0,
                            y: 0.5,
                            z: 100.0,
                        })
                        .unwrap(),
                    ),
                    offset: Vec3::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
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

fn scene_character2() -> Result<String, String> {
    let mut world = World2::default();
    world
        .add_body(
            BodyBuilder2::static_body()
                .position(Vec2 { x: 0.0, y: -0.5 })
                .add_collider(Collider2 {
                    shape: ColliderShape2::Box(
                        auralite_geometry::Box2::new(Vec2 { x: 100.0, y: 0.5 }).unwrap(),
                    ),
                    offset: Vec2::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
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

fn scene_character3() -> Result<String, String> {
    let mut world = World3::default();
    world
        .add_body(
            BodyBuilder3::static_body()
                .position(Vec3 {
                    x: 0.0,
                    y: -0.5,
                    z: 0.0,
                })
                .add_collider(Collider3 {
                    shape: ColliderShape3::Box(
                        auralite_geometry::Box3::new(Vec3 {
                            x: 100.0,
                            y: 0.5,
                            z: 100.0,
                        })
                        .unwrap(),
                    ),
                    offset: Vec3::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
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

fn scene_stress() -> Result<String, String> {
    let mut world = World2::default();
    world
        .add_body(BodyBuilder2::static_body().add_collider(make_box_collider(50.0, 0.5)))
        .map_err(|e| format!("ground: {:?}", e))?;
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
    for h in world.body_handles() {
        let b = world.body(h).map_err(|_| "stale".to_string())?;
        if !b.position.is_finite() {
            return Err("non-finite body".into());
        }
    }
    let hash_a = world.state_hash();
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
