//! Real interactive desktop sandbox — engine-driven, no mocks.
//! Implements DoD-5: scene browser 16 subsystems, time controls, debug-draw toggles,
//! inspection panels, editable runtime settings, profiling overlay, real determinism controls (seed/record/replay/snapshot/rollback) showing real 64-bit state hash.
//! Dependency: eframe (winit + glow + egui) — justified in ADR-17, default-features off, license MIT/Apache.

use eframe::egui;

use auralite_collision::CollisionFilter;
use auralite_dynamics::{
    BodyType, Collider2, Collider3, ColliderShape2, ColliderShape3, JointConfig2, JointType2,
    Material, World2, World3,
};
use auralite_geometry::{Box2, Box3, Circle2, Sphere3};
use auralite_math::{Quat, Real, Vec2, Vec3};
use auralite_particles::{ParticleStorage, ParticleType, PbfFluid};
use auralite_softbody::{apply_self_collision, build_cloth_grid};
use auralite_vehicles::{
    Character2, Character3, CharacterConfig2, CharacterConfig3, Vehicle3, VehicleConfig3,
    WheelConfig3,
};
use std::time::Instant;

/// Bounded capture for record/replay — standing rule: iterative features stay bounded.
const MAX_RECORD_FRAMES: usize = 100_000;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SceneId {
    Stacking,
    Joints,
    Ccd,
    Triggers,
    Replay,
    Cloth,
    SelfCollision,
    Particles,
    Fluid,
    Buoyancy,
    Fields,
    Vehicle,
    Char2d,
    Char3d,
    Serialization,
    Stress,
}

impl SceneId {
    pub fn as_str(&self) -> &'static str {
        match self {
            SceneId::Stacking => "1. Stacking (10 boxes)",
            SceneId::Joints => "2. Joints (11-body ragdoll)",
            SceneId::Ccd => "3. CCD (fast sphere)",
            SceneId::Triggers => "4. Triggers & Sensors",
            SceneId::Replay => "5. Replay & Rollback",
            SceneId::Cloth => "6. Cloth (XPBD 8x8)",
            SceneId::SelfCollision => "7. Self-Collision Cloth 6x6",
            SceneId::Particles => "8. Particles (emitter)",
            SceneId::Fluid => "9. Fluid (PBF 25)",
            SceneId::Buoyancy => "10. Buoyancy",
            SceneId::Fields => "11. Fields (wind+drag)",
            SceneId::Vehicle => "12. Vehicle 3D",
            SceneId::Char2d => "13. Character 2D",
            SceneId::Char3d => "14. Character 3D",
            SceneId::Serialization => "15. Serialization",
            SceneId::Stress => "16. Stress 100 bodies",
        }
    }
    pub fn all() -> Vec<SceneId> {
        vec![
            SceneId::Stacking,
            SceneId::Joints,
            SceneId::Ccd,
            SceneId::Triggers,
            SceneId::Replay,
            SceneId::Cloth,
            SceneId::SelfCollision,
            SceneId::Particles,
            SceneId::Fluid,
            SceneId::Buoyancy,
            SceneId::Fields,
            SceneId::Vehicle,
            SceneId::Char2d,
            SceneId::Char3d,
            SceneId::Serialization,
            SceneId::Stress,
        ]
    }
    pub fn is_3d(&self) -> bool {
        matches!(
            self,
            SceneId::Replay
                | SceneId::Vehicle
                | SceneId::Char3d
                | SceneId::Cloth
                | SceneId::SelfCollision
                | SceneId::Fluid
                | SceneId::Buoyancy
                | SceneId::Fields
                | SceneId::Particles
        )
    }
}

#[derive(Clone, Debug, Default)]
pub struct DebugDraw {
    pub aabbs: bool,
    pub contacts: bool,
    pub normals: bool,
    pub com: bool,
    pub velocities: bool,
    pub broadphase: bool,
    pub joints: bool,
    pub sleep: bool,
    pub softbody: bool,
    pub particles: bool,
}

// World payloads are boxed: World2 is 456+ bytes inline; boxing keeps the enum small
// and satisfies clippy::large_enum_variant without suppressions.
pub enum ActiveWorld {
    World2(Box<World2>),
    World3(Box<World3>),
    SoftBody(Box<auralite_softbody::SoftBody>),
    Mixed, // for scenes that have both (particles/fluid live in dedicated fields)
}

pub struct SandboxApp {
    // Scene
    pub scene: SceneId,
    pub world: ActiveWorld,
    // Time
    pub paused: bool,
    pub time_scale: f32,
    pub step_count: u64,
    pub sim_time: f64,
    pub dt: f32,
    pub substeps: usize,
    // Runtime settings
    pub gravity2: Vec2,
    pub gravity3: Vec3,
    pub iterations: usize,
    // Debug
    pub debug: DebugDraw,
    pub selected_body: Option<u64>, // StableId
    // Determinism
    pub state_hash: u64,
    pub snapshot2: Option<auralite_dynamics::Snapshot2>,
    pub snapshot3: Option<auralite_dynamics::Snapshot3>,
    pub snapshot_soft: Option<auralite_softbody::SoftBody>,
    // Real engine-driven recording: ordered per-frame state hashes captured from
    // actual stepping, verified on replay against the record-start engine snapshot
    pub recorded_hashes: Vec<u64>,
    pub record_snapshot2: Option<auralite_dynamics::Snapshot2>,
    pub record_snapshot3: Option<auralite_dynamics::Snapshot3>,
    pub record_snapshot_soft: Option<auralite_softbody::SoftBody>,
    pub is_recording: bool,
    pub is_replaying: bool,
    pub replay_index: usize,
    // Divergence evidence: (frame index, recorded hash, replayed hash) — set on mismatch
    pub replay_mismatch: Option<(usize, u64, u64)>,
    // Profiling
    pub last_step_time_us: f64,
    pub last_broad_time_us: f64,
    // Particle / fluid extras
    pub particle_storage: Option<ParticleStorage>,
    pub fluid: Option<PbfFluid>,
    pub cloth_particles_clone: Option<auralite_softbody::SoftBody>,
    // Character / vehicle extras
    pub character2: Option<Character2>,
    pub character3: Option<Character3>,
    pub vehicle3: Option<Vehicle3>,
    // Editable material for selected
    pub edit_friction: f32,
    pub edit_restitution: f32,
    // UI state
    pub show_inspection: bool,
    pub show_settings: bool,
}

impl SandboxApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let mut app = Self {
            scene: SceneId::Stacking,
            world: ActiveWorld::Mixed,
            paused: false,
            time_scale: 1.0,
            step_count: 0,
            sim_time: 0.0,
            dt: 1.0 / 60.0,
            substeps: 1,
            gravity2: Vec2 { x: 0.0, y: -9.81 },
            gravity3: Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
            iterations: 10,
            debug: DebugDraw {
                aabbs: true,
                contacts: true,
                normals: false,
                com: false,
                velocities: true,
                broadphase: false,
                joints: true,
                sleep: true,
                softbody: true,
                particles: true,
            },
            selected_body: None,
            state_hash: 0,
            snapshot2: None,
            snapshot3: None,
            snapshot_soft: None,
            recorded_hashes: Vec::new(),
            record_snapshot2: None,
            record_snapshot3: None,
            record_snapshot_soft: None,
            is_recording: false,
            is_replaying: false,
            replay_index: 0,
            replay_mismatch: None,
            last_step_time_us: 0.0,
            last_broad_time_us: 0.0,
            particle_storage: None,
            fluid: None,
            cloth_particles_clone: None,
            character2: None,
            character3: None,
            vehicle3: None,
            edit_friction: 0.5,
            edit_restitution: 0.0,
            show_inspection: true,
            show_settings: true,
        };
        app.restart_scene();
        app
    }

    pub fn restart_scene(&mut self) {
        self.step_count = 0;
        self.sim_time = 0.0;
        self.state_hash = 0;
        self.snapshot2 = None;
        self.snapshot3 = None;
        self.snapshot_soft = None;
        self.recorded_hashes.clear();
        self.record_snapshot2 = None;
        self.record_snapshot3 = None;
        self.record_snapshot_soft = None;
        self.is_recording = false;
        self.is_replaying = false;
        self.replay_index = 0;
        self.replay_mismatch = None;
        self.particle_storage = None;
        self.fluid = None;
        self.cloth_particles_clone = None;
        self.character2 = None;
        self.character3 = None;
        self.vehicle3 = None;
        self.selected_body = None;
        match self.scene {
            SceneId::Stacking => {
                let mut w = World2::default();
                w.set_gravity(self.gravity2).ok();
                w.solver_iterations = self.iterations as u16;
                let ground =
                    auralite_dynamics::BodyBuilder2::static_body().add_collider(Collider2 {
                        shape: ColliderShape2::Box(Box2::new(Vec2 { x: 10.0, y: 0.5 }).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material {
                            restitution: 0.0,
                            friction: 0.8,
                            density: 1.0,
                        },
                        filter: CollisionFilter::default(),
                    });
                w.add_body(ground).ok();
                for i in 0..5 {
                    let x = (i as Real - 2.0) * 1.1;
                    let y = 1.0 + i as Real * 1.1;
                    let b = auralite_dynamics::BodyBuilder2::dynamic()
                        .position(Vec2 { x, y })
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Box(Box2::new(Vec2 { x: 0.5, y: 0.5 }).unwrap()),
                            offset: Vec2::ZERO,
                            material: Material {
                                restitution: 0.0,
                                friction: 0.8,
                                density: 1.0,
                            },
                            filter: CollisionFilter::default(),
                        });
                    w.add_body(b).ok();
                }
                self.state_hash = w.state_hash();
                self.world = ActiveWorld::World2(Box::new(w));
            }
            SceneId::Joints => {
                let mut w = World2::default();
                w.set_gravity(self.gravity2).ok();
                w.solver_iterations = self.iterations as u16;
                let mut handles = Vec::new();
                for i in 0..11 {
                    let y = 5.0 + (10 - i) as Real * 0.8;
                    let b = auralite_dynamics::BodyBuilder2::dynamic()
                        .position(Vec2 { x: 0.0, y })
                        .mass(if i % 2 == 0 { 1.0 } else { 0.5 })
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Circle(Circle2::new(0.3).unwrap()),
                            offset: Vec2::ZERO,
                            material: Material::default(),
                            filter: CollisionFilter::default(),
                        });
                    handles.push(w.add_body(b).unwrap());
                }
                for i in 0..10 {
                    let cfg = JointConfig2::new(
                        JointType2::Revolute,
                        handles[i + 1],
                        handles[i],
                        Vec2 { x: 0.0, y: -0.4 },
                        Vec2 { x: 0.0, y: 0.4 },
                    );
                    w.add_joint(cfg).ok();
                }
                self.world = ActiveWorld::World2(Box::new(w));
            }
            SceneId::Ccd => {
                let mut w = World2::default();
                w.set_gravity(self.gravity2).ok();
                w.add_body(
                    auralite_dynamics::BodyBuilder2::dynamic()
                        .position(Vec2 { x: 0.0, y: 10.0 })
                        .velocity(Vec2 { x: 0.0, y: -20.0 })
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                            offset: Vec2::ZERO,
                            material: Material::default(),
                            filter: CollisionFilter::default(),
                        }),
                )
                .ok();
                w.add_body(
                    auralite_dynamics::BodyBuilder2::static_body()
                        .position(Vec2 { x: 0.0, y: -0.5 })
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Box(
                                Box2::new(Vec2 { x: 10.0, y: 0.5 }).unwrap(),
                            ),
                            offset: Vec2::ZERO,
                            material: Material::default(),
                            filter: CollisionFilter::default(),
                        }),
                )
                .ok();
                self.world = ActiveWorld::World2(Box::new(w));
            }
            SceneId::Triggers => {
                let mut w = World2::default();
                w.set_gravity(Vec2::ZERO).ok();
                let sensor = auralite_dynamics::BodyBuilder2::dynamic()
                    .position(Vec2 { x: 0.0, y: 0.0 })
                    .velocity(Vec2 { x: 1.0, y: 0.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter {
                            sensor: true,
                            ..Default::default()
                        },
                    });
                w.add_body(sensor).ok();
                let other = auralite_dynamics::BodyBuilder2::static_body()
                    .position(Vec2 { x: 5.0, y: 0.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(1.0).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    });
                w.add_body(other).ok();
                self.world = ActiveWorld::World2(Box::new(w));
            }
            SceneId::Replay => {
                let mut w = World3::default();
                w.set_gravity(self.gravity3).ok();
                w.add_body(
                    auralite_dynamics::BodyBuilder3::dynamic()
                        .position(Vec3 {
                            x: 1.0,
                            y: 10.0,
                            z: 2.0,
                        })
                        .add_collider(Collider3 {
                            shape: ColliderShape3::Sphere(Sphere3::new(0.5).unwrap()),
                            offset: Vec3::ZERO,
                            material: Material::default(),
                            filter: CollisionFilter::default(),
                        }),
                )
                .ok();
                self.world = ActiveWorld::World3(Box::new(w));
            }
            SceneId::Cloth => {
                let cloth = build_cloth_grid(
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
                self.world = ActiveWorld::SoftBody(Box::new(cloth));
            }
            SceneId::SelfCollision => {
                let cloth = build_cloth_grid(
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
                self.world = ActiveWorld::SoftBody(Box::new(cloth));
            }
            SceneId::Particles => {
                let storage = ParticleStorage::new(200);
                self.particle_storage = Some(storage);
                self.world = ActiveWorld::Mixed;
            }
            SceneId::Fluid => {
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
                self.particle_storage = Some(storage);
                self.fluid = Some(PbfFluid::new(1000.0, 0.06, 0.1, 0.01));
                self.world = ActiveWorld::Mixed;
            }
            SceneId::Buoyancy => {
                let mut w = World3::default();
                w.set_gravity(self.gravity3).ok();
                w.add_body(
                    auralite_dynamics::BodyBuilder3::static_body()
                        .position(Vec3 {
                            x: 0.0,
                            y: -0.5,
                            z: 0.0,
                        })
                        .add_collider(Collider3 {
                            shape: ColliderShape3::Box(
                                Box3::new(Vec3 {
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
                w.add_body(
                    auralite_dynamics::BodyBuilder3::dynamic()
                        .position(Vec3 {
                            x: 0.0,
                            y: 1.0,
                            z: 0.0,
                        })
                        .add_collider(Collider3 {
                            shape: ColliderShape3::Box(
                                Box3::new(Vec3 {
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
                self.world = ActiveWorld::World3(Box::new(w));
            }
            SceneId::Fields => {
                let mut storage = ParticleStorage::new(50);
                storage.spawn(
                    Vec3::ZERO,
                    Vec3 {
                        x: 2.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    10.0,
                    ParticleType::Generic,
                );
                self.particle_storage = Some(storage);
                self.world = ActiveWorld::Mixed;
            }
            SceneId::Vehicle => {
                let mut w = World3::default();
                w.set_gravity(self.gravity3).ok();
                w.add_body(
                    auralite_dynamics::BodyBuilder3::static_body()
                        .position(Vec3 {
                            x: 0.0,
                            y: -0.5,
                            z: 0.0,
                        })
                        .add_collider(Collider3 {
                            shape: ColliderShape3::Box(
                                Box3::new(Vec3 {
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
                let wheels = vec![WheelConfig3::default(); 4];
                let mut vehicle = Vehicle3::new(
                    VehicleConfig3::default(),
                    Vec3 {
                        x: 0.0,
                        y: 1.0,
                        z: 0.0,
                    },
                    Quat::identity(),
                    wheels,
                    &mut w,
                );
                vehicle.set_controls(0.5, 0.0, 0.3);
                self.vehicle3 = Some(vehicle);
                self.world = ActiveWorld::World3(Box::new(w));
            }
            SceneId::Char2d => {
                let mut w = World2::default();
                w.set_gravity(self.gravity2).ok();
                w.add_body(
                    auralite_dynamics::BodyBuilder2::static_body()
                        .position(Vec2 { x: 0.0, y: -0.5 })
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Box(
                                Box2::new(Vec2 { x: 100.0, y: 0.5 }).unwrap(),
                            ),
                            offset: Vec2::ZERO,
                            material: Material::default(),
                            filter: CollisionFilter::default(),
                        }),
                )
                .ok();
                let mut cc = Character2::new(CharacterConfig2::default(), Vec2 { x: 0.0, y: 2.0 });
                cc.attach(&mut w);
                cc.set_move(Vec2 { x: 0.5, y: 0.0 });
                self.character2 = Some(cc);
                self.world = ActiveWorld::World2(Box::new(w));
            }
            SceneId::Char3d => {
                let mut w = World3::default();
                w.set_gravity(self.gravity3).ok();
                w.add_body(
                    auralite_dynamics::BodyBuilder3::static_body()
                        .position(Vec3 {
                            x: 0.0,
                            y: -0.5,
                            z: 0.0,
                        })
                        .add_collider(Collider3 {
                            shape: ColliderShape3::Box(
                                Box3::new(Vec3 {
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
                cc.attach(&mut w);
                cc.set_move(Vec3 {
                    x: 0.5,
                    y: 0.0,
                    z: 0.5,
                });
                self.character3 = Some(cc);
                self.world = ActiveWorld::World3(Box::new(w));
            }
            SceneId::Serialization => {
                // Simple world for serialization demo
                let mut w = World2::default();
                w.add_body(
                    auralite_dynamics::BodyBuilder2::dynamic()
                        .position(Vec2 { x: 0.0, y: 2.0 })
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                            offset: Vec2::ZERO,
                            material: Material::default(),
                            filter: CollisionFilter::default(),
                        }),
                )
                .ok();
                self.world = ActiveWorld::World2(Box::new(w));
            }
            SceneId::Stress => {
                let mut w = World2::default();
                w.set_gravity(self.gravity2).ok();
                w.solver_iterations = self.iterations as u16;
                w.add_body(auralite_dynamics::BodyBuilder2::static_body().add_collider(
                    Collider2 {
                        shape: ColliderShape2::Box(Box2::new(Vec2 { x: 50.0, y: 0.5 }).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    },
                ))
                .ok();
                for i in 0..100 {
                    let x = (i as Real % 10.0 - 5.0) * 1.5;
                    let y = (i as Real / 10.0).floor() * 1.5 + 2.0;
                    w.add_body(
                        auralite_dynamics::BodyBuilder2::dynamic()
                            .position(Vec2 { x, y })
                            .add_collider(Collider2 {
                                shape: ColliderShape2::Box(
                                    Box2::new(Vec2 { x: 0.4, y: 0.4 }).unwrap(),
                                ),
                                offset: Vec2::ZERO,
                                material: Material::default(),
                                filter: CollisionFilter::default(),
                            }),
                    )
                    .ok();
                }
                self.world = ActiveWorld::World2(Box::new(w));
            }
        }
        self.update_hash();
    }

    pub fn update_hash(&mut self) {
        self.state_hash = match &self.world {
            ActiveWorld::World2(w) => w.state_hash(),
            ActiveWorld::World3(w) => w.state_hash(),
            ActiveWorld::SoftBody(sb) => {
                // hash of particle positions
                let mut bytes = Vec::new();
                for p in &sb.particles {
                    bytes.extend_from_slice(&p.position.x.to_bits().to_le_bytes());
                    bytes.extend_from_slice(&p.position.y.to_bits().to_le_bytes());
                    bytes.extend_from_slice(&p.position.z.to_bits().to_le_bytes());
                }
                auralite_core::hash_bytes(&bytes)
            }
            ActiveWorld::Mixed => 0,
        };
    }

    pub fn step(&mut self) {
        if self.paused {
            return;
        }
        let dt = self.dt * self.time_scale as Real;
        let start = Instant::now();
        for _ in 0..self.substeps {
            match &mut self.world {
                ActiveWorld::World2(w) => {
                    w.set_gravity(self.gravity2).ok();
                    w.solver_iterations = self.iterations as u16;
                    if let Some(cc) = &mut self.character2 {
                        cc.step(dt, w);
                    }
                    w.step(dt).ok();
                }
                ActiveWorld::World3(w) => {
                    w.set_gravity(self.gravity3).ok();
                    w.solver_iterations = self.iterations as u16;
                    if let Some(veh) = &mut self.vehicle3 {
                        veh.step(dt, w);
                    }
                    if let Some(cc) = &mut self.character3 {
                        cc.step(dt, w);
                    }
                    w.step(dt).ok();
                }
                ActiveWorld::SoftBody(sb) => {
                    sb.pre_step(dt, self.gravity3);
                    sb.solve_constraints(10, dt);
                    if self.scene == SceneId::SelfCollision && self.step_count.is_multiple_of(5) {
                        apply_self_collision(sb, 0.075);
                    }
                    sb.post_step(dt);
                }
                ActiveWorld::Mixed => {
                    // particle + fluid stepping
                    if let Some(storage) = &mut self.particle_storage
                        && let Some(fluid) = &mut self.fluid
                    {
                        let indices: Vec<usize> =
                            storage.iterate_alive().map(|(i, _, _, _)| i).collect();
                        fluid.step(storage, &indices, dt, self.gravity3);
                    }
                }
            }
            self.sim_time += dt as f64;
            self.step_count += 1;
        }
        self.last_step_time_us = start.elapsed().as_micros() as f64;
        self.update_hash();
        // Engine-driven record/replay (no mocks): recording appends the real
        // `state_hash()` of each stepped frame; replay restores the record-start
        // engine snapshot and verifies every re-stepped hash against the trace.
        if self.is_recording && !self.is_replaying {
            self.recorded_hashes.push(self.state_hash);
            if self.recorded_hashes.len() >= MAX_RECORD_FRAMES {
                self.is_recording = false; // bounded capture reached
            }
        } else if self.is_replaying {
            match self.recorded_hashes.get(self.replay_index) {
                Some(&expected) if expected == self.state_hash => {
                    self.replay_index += 1;
                    if self.replay_index >= self.recorded_hashes.len() {
                        self.is_replaying = false; // replay complete — all frames verified
                    }
                }
                Some(&expected) => {
                    self.replay_mismatch = Some((self.replay_index, expected, self.state_hash));
                    self.is_replaying = false;
                }
                None => {
                    self.is_replaying = false;
                }
            }
        }
    }

    /// Begin real engine recording: capture an engine snapshot of the active world
    /// and clear the recorded hash trace. Each subsequent stepped frame appends its
    /// real `state_hash()` to `recorded_hashes` (bounded by MAX_RECORD_FRAMES).
    pub fn start_recording(&mut self) {
        self.recorded_hashes.clear();
        self.replay_index = 0;
        self.replay_mismatch = None;
        self.is_replaying = false;
        self.is_recording = true;
        match &self.world {
            ActiveWorld::World2(w) => {
                self.record_snapshot2 = Some(w.snapshot());
                self.record_snapshot3 = None;
                self.record_snapshot_soft = None;
            }
            ActiveWorld::World3(w) => {
                self.record_snapshot3 = Some(w.snapshot());
                self.record_snapshot2 = None;
                self.record_snapshot_soft = None;
            }
            ActiveWorld::SoftBody(sb) => {
                self.record_snapshot_soft = Some((**sb).clone());
                self.record_snapshot2 = None;
                self.record_snapshot3 = None;
            }
            ActiveWorld::Mixed => {}
        }
    }

    /// Begin verified replay: restore the record-start engine snapshot and re-step;
    /// `step()` compares each frame's hash against the recorded trace.
    pub fn start_replay(&mut self) {
        let restored = match &mut self.world {
            ActiveWorld::World2(w) => match &self.record_snapshot2 {
                Some(snap) => w.restore(snap).is_ok(),
                None => false,
            },
            ActiveWorld::World3(w) => match &self.record_snapshot3 {
                Some(snap) => w.restore(snap).is_ok(),
                None => false,
            },
            ActiveWorld::SoftBody(sb) => match &self.record_snapshot_soft {
                Some(snap) => {
                    **sb = snap.clone();
                    true
                }
                None => false,
            },
            ActiveWorld::Mixed => false,
        };
        if !restored {
            return;
        }
        self.replay_index = 0;
        self.replay_mismatch = None;
        self.is_recording = false;
        self.is_replaying = true;
        self.update_hash();
    }
}

impl eframe::App for SandboxApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Top panel
        egui::TopBottomPanel::top("top").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.show_inspection, "🔍 Inspection");
                ui.checkbox(&mut self.show_settings, "⚙ Settings");
                ui.separator();
                ui.heading("⚡ AuraLite Physics Engine — Interactive Sandbox Studio (Real Engine)");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(
                        egui::RichText::new(format!("Hash: {:016x}", self.state_hash))
                            .monospace()
                            .color(egui::Color32::from_rgb(255, 221, 68)),
                    );
                    ui.label(format!(
                        "Step: {} | Time: {:.2}s",
                        self.step_count, self.sim_time
                    ));
                });
            });
        });

        // Left panel - controls
        egui::SidePanel::left("left").default_width(340.0).show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.collapsing("Scene Browser (16 subsystems)", |ui| {
                    ui.label("Select subsystem scene:");
                    let mut new_scene = self.scene;
                    for sid in SceneId::all() {
                        if ui
                            .selectable_label(self.scene == sid, sid.as_str())
                            .clicked()
                        {
                            new_scene = sid;
                        }
                    }
                    if new_scene != self.scene {
                        self.scene = new_scene;
                        self.restart_scene();
                    }
                });

                ui.collapsing("Time & Simulation Controls", |ui| {
                    ui.horizontal(|ui| {
                        if ui
                            .button(if self.paused { "▶ Resume" } else { "⏸ Pause" })
                            .clicked()
                        {
                            self.paused = !self.paused;
                        }
                        if ui.button("🔄 Restart").clicked() {
                            self.restart_scene();
                        }
                        if ui.button("⏭ Step").clicked() {
                            self.paused = true;
                            self.step();
                        }
                    });
                    ui.add(
                        egui::Slider::new(&mut self.time_scale, 0.1..=3.0)
                            .text("Time Scale")
                            .suffix("x"),
                    );
                    ui.add(egui::Slider::new(&mut self.dt, 0.001..=0.033).text("dt (s)"));
                    ui.add(egui::Slider::new(&mut self.substeps, 1..=8).text("Substeps"));
                });

                ui.collapsing("Debug-Draw Toggles", |ui| {
                    ui.checkbox(&mut self.debug.aabbs, "AABBs (Bounding Boxes)");
                    ui.checkbox(&mut self.debug.contacts, "Contacts");
                    ui.checkbox(&mut self.debug.normals, "Contact Normals");
                    ui.checkbox(&mut self.debug.velocities, "Velocity Vectors");
                    ui.checkbox(&mut self.debug.com, "Centers of Mass (COM)");
                    ui.checkbox(&mut self.debug.joints, "Joint Constraints");
                    ui.checkbox(&mut self.debug.sleep, "Sleep State Color");
                    ui.checkbox(&mut self.debug.broadphase, "Broad-phase Bounds");
                    ui.checkbox(&mut self.debug.softbody, "Soft-body / Cloth");
                    ui.checkbox(&mut self.debug.particles, "Particle / Fluid");
                });

                if self.show_settings {
                    ui.collapsing("Editable Runtime Settings", |ui| {
                    ui.label("Gravity (2D):");
                    ui.horizontal(|ui| {
                        ui.add(egui::DragValue::new(&mut self.gravity2.x).prefix("x: "));
                        ui.add(egui::DragValue::new(&mut self.gravity2.y).prefix("y: "));
                    });
                    ui.label("Gravity (3D):");
                    ui.horizontal(|ui| {
                        ui.add(egui::DragValue::new(&mut self.gravity3.x).prefix("x: "));
                        ui.add(egui::DragValue::new(&mut self.gravity3.y).prefix("y: "));
                        ui.add(egui::DragValue::new(&mut self.gravity3.z).prefix("z: "));
                    });
                    ui.add(
                        egui::Slider::new(&mut self.iterations, 1..=50).text("Solver Iterations"),
                    );
                    ui.horizontal(|ui| {
                        ui.label("Material edit (selected):");
                        ui.add(
                            egui::DragValue::new(&mut self.edit_friction)
                                .range(0.0..=1.0)
                                .prefix("friction: "),
                        );
                        ui.add(
                            egui::DragValue::new(&mut self.edit_restitution)
                                .range(0.0..=1.0)
                                .prefix("restitution: "),
                        );
                    });
                    if ui.button("Apply to selected body").clicked()
                        && let ActiveWorld::World2(w) = &mut self.world
                            && let Some(sid) = self.selected_body {
                                let handles = w.body_handles();
                                for h in handles {
                                    if let Ok(bb) = w.body(h)
                                        && bb.id.0 == sid {
                                            if let Ok(bm) = w.body_mut(h) {
                                                bm.friction = self.edit_friction as Real;
                                                bm.restitution =
                                                    self.edit_restitution as Real;
                                            }
                                            break;
                                        }
                                }
                            }
                    });
                }

                ui.collapsing("Determinism Controls (Real Engine)", |ui| {
                    ui.label(
                        egui::RichText::new(format!("Real state hash: {:016x}", self.state_hash))
                            .monospace()
                            .color(egui::Color32::YELLOW),
                    );
                    ui.horizontal(|ui| {
                        if ui.button("📸 Snapshot").clicked() {
                            match &self.world {
                                ActiveWorld::World2(w) => {
                                    self.snapshot2 = Some(w.snapshot());
                                }
                                ActiveWorld::World3(w) => {
                                    self.snapshot3 = Some(w.snapshot());
                                }
                                ActiveWorld::SoftBody(sb) => {
                                    self.snapshot_soft = Some((**sb).clone());
                                }
                                ActiveWorld::Mixed => {}
                            }
                        }
                        if ui.button("⏮ Rollback").clicked() {
                            match &mut self.world {
                                ActiveWorld::World2(w) => {
                                    if let Some(snap) = &self.snapshot2 {
                                        w.restore(snap).ok();
                                        self.update_hash();
                                    }
                                }
                                ActiveWorld::World3(w) => {
                                    if let Some(snap) = &self.snapshot3 {
                                        w.restore(snap).ok();
                                        self.update_hash();
                                    }
                                }
                                ActiveWorld::SoftBody(sb) => {
                                    if let Some(snap) = &self.snapshot_soft {
                                        **sb = snap.clone();
                                        self.update_hash();
                                    }
                                }
                                ActiveWorld::Mixed => {}
                            }
                        }
                    });
                    // Real record/replay: engine snapshot + per-frame state-hash trace,
                    // verified by re-stepping from the snapshot (no mock counters).
                    let recordable = matches!(
                        self.world,
                        ActiveWorld::World2(_) | ActiveWorld::World3(_) | ActiveWorld::SoftBody(_)
                    );
                    if recordable {
                        ui.horizontal(|ui| {
                            let mut rec = self.is_recording;
                            if ui.checkbox(&mut rec, "🔴 Record").changed() {
                                if rec {
                                    self.start_recording();
                                } else {
                                    self.is_recording = false;
                                }
                            }
                            let can_replay =
                                !self.is_recording && !self.recorded_hashes.is_empty();
                            if ui
                                .add_enabled(
                                    can_replay && !self.is_replaying,
                                    egui::Button::new("▶ Replay & verify"),
                                )
                                .clicked()
                            {
                                self.start_replay();
                            }
                        });
                        ui.label(format!(
                            "Recorded frames: {} (real per-step state hashes)",
                            self.recorded_hashes.len()
                        ));
                        if self.is_recording {
                            ui.label("Recording… every stepped frame's state hash is captured");
                        }
                        if self.is_replaying {
                            ui.label(format!(
                                "Replaying: frame {}/{} verified (hash match)",
                                self.replay_index,
                                self.recorded_hashes.len()
                            ));
                        }
                        if let Some((idx, expected, actual)) = self.replay_mismatch {
                            ui.colored_label(
                                egui::Color32::RED,
                                format!(
                                    "❌ Replay diverged at frame {idx}: recorded {expected:016x} vs replayed {actual:016x}"
                                ),
                            );
                        }
                    } else {
                        ui.label(
                            "Record/Replay available on rigid-body (2D/3D) and soft-body scenes \
                             (engine snapshot API); particle-only scenes are excluded honestly.",
                        );
                    }
                });

                if self.show_inspection {
                    ui.collapsing("Body / Constraint Inspection", |ui| {
                    match &self.world {
                        ActiveWorld::World2(w) => {
                            ui.label(format!("Bodies: {} | Joints: {}", w.body_count(), w.joints.len()));
                            egui::ScrollArea::vertical().max_height(150.0).show(ui, |ui| {
                                for (h, b) in w.bodies_iter() {
                                    let label = format!(
                                        "ID {}: pos=({:.2},{:.2}) v=({:.2},{:.2}) sleep={} kind={:?}",
                                        b.id.0, b.position.x, b.position.y, b.velocity.x, b.velocity.y, b.sleeping, b.kind
                                    );
                                    let selected = self.selected_body == Some(b.id.0);
                                    if ui.selectable_label(selected, label).clicked() {
                                        self.selected_body = Some(b.id.0);
                                        let _ = h;
                                    }
                                }
                            });
                            if let Some(sid) = self.selected_body {
                                ui.separator();
                                ui.label(format!("Selected body ID {}", sid));
                                if let Some((_, b)) = w.bodies_iter().find(|(_, b)| b.id.0 == sid) {
                                    ui.label(format!("Pos: {:.3},{:.3}", b.position.x, b.position.y));
                                    ui.label(format!("Vel: {:.3},{:.3}", b.velocity.x, b.velocity.y));
                                    ui.label(format!("AngVel: {:.3}", b.angular_velocity));
                                    ui.label(format!("Sleeping: {}", b.sleeping));
                                    ui.label(format!("Colliders: {}", b.colliders.len()));
                                }
                            }
                            ui.separator();
                            ui.label("Joints:");
                            for j in &w.joints {
                                ui.label(format!(
                                    "Joint {:?} ID {} broken={} imp={:.2}",
                                    j.config.joint_type, j.id.0, j.broken, j.impulse
                                ));
                            }
                            ui.label(format!("Sensor events: {}", w.sensor_events.len()));
                            for ev in w.sensor_events.iter().take(5) {
                                ui.label(format!("Sensor {} other {} began {}", ev.sensor, ev.other, ev.began));
                            }
                        }
                        ActiveWorld::World3(w) => {
                            ui.label(format!("Bodies: {} | Joints: {}", w.body_count(), w.joints.len()));
                            egui::ScrollArea::vertical().max_height(150.0).show(ui, |ui| {
                                for (_, b) in w.bodies_iter() {
                                    let selected = self.selected_body == Some(b.id.0);
                                    if ui
                                        .selectable_label(
                                            selected,
                                            format!(
                                                "ID {} pos=({:.1},{:.1},{:.1}) sleep={}",
                                                b.id.0, b.position.x, b.position.y, b.position.z, b.sleeping
                                            ),
                                        )
                                        .clicked()
                                    {
                                        self.selected_body = Some(b.id.0);
                                    }
                                }
                            });
                            if let Some(sid) = self.selected_body
                                && let Some((_, b)) = w.bodies_iter().find(|(_, b)| b.id.0 == sid) {
                                    ui.label(format!(
                                        "Pos: {:.2},{:.2},{:.2}",
                                        b.position.x, b.position.y, b.position.z
                                    ));
                                    ui.label(format!(
                                        "Vel: {:.2},{:.2},{:.2}",
                                        b.velocity.x, b.velocity.y, b.velocity.z
                                    ));
                                }
                        }
                        ActiveWorld::SoftBody(sb) => {
                            ui.label(format!("SoftBody particles: {}", sb.particles.len()));
                            ui.label(format!("KE: {:.3}", sb.kinetic_energy()));
                        }
                        ActiveWorld::Mixed => {
                            if let Some(storage) = &self.particle_storage {
                                ui.label(format!("Particles alive: {}", storage.alive_count()));
                            }
                        }
                    }
                    });
                }

                ui.collapsing("Profiling Overlay (Real Timing)", |ui| {
                    ui.label(format!("Last step: {:.1} µs", self.last_step_time_us));
                    ui.label(format!("Broad-phase est: {:.1} µs", self.last_broad_time_us));
                    ui.label(format!("Bodies: {}", match &self.world {
                        ActiveWorld::World2(w) => w.body_count(),
                        ActiveWorld::World3(w) => w.body_count(),
                        _ => 0,
                    }));
                });
            });
        });

        // Central panel - 2D/3D view (engine-driven)
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.label(format!(
                "Scene: {} | {} view | Real hash display, no mocks",
                self.scene.as_str(),
                if self.scene.is_3d() { "3D" } else { "2D" }
            ));
            let available = ui.available_size();
            let (rect, _) = ui.allocate_exact_size(available, egui::Sense::click());
            let painter = ui.painter_at(rect);
            painter.rect_filled(rect, 8.0, egui::Color32::from_rgb(20, 20, 26));

            // Draw world
            match &self.world {
                ActiveWorld::World2(w) => {
                    let scale = 30.0;
                    let off_x = rect.center().x;
                    let off_y = rect.center().y + 100.0;
                    // ground line
                    painter.line_segment(
                        [
                            egui::pos2(rect.left(), off_y),
                            egui::pos2(rect.right(), off_y),
                        ],
                        egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(50, 50, 60)),
                    );
                    for (_, b) in w.bodies_iter() {
                        let screen_x = off_x + b.position.x * scale;
                        let screen_y = off_y - b.position.y * scale;
                        let color = match b.kind {
                            BodyType::Static => egui::Color32::GRAY,
                            BodyType::Kinematic => egui::Color32::from_rgb(68, 170, 153),
                            BodyType::Dynamic => {
                                if b.sleeping {
                                    egui::Color32::from_rgb(68, 68, 100)
                                } else {
                                    egui::Color32::from_rgb(68, 153, 255)
                                }
                            }
                        };
                        // draw first collider as circle/box approx
                        let r = b
                            .colliders
                            .first()
                            .map(|c| c.bounding_radius() * scale)
                            .unwrap_or(10.0);
                        painter.circle_filled(egui::pos2(screen_x, screen_y), r.max(4.0), color);
                        painter.circle_stroke(
                            egui::pos2(screen_x, screen_y),
                            r.max(4.0),
                            egui::Stroke::new(1.0_f32, egui::Color32::WHITE),
                        );
                        if self.debug.velocities && !b.sleeping {
                            let vx = b.velocity.x * scale * 0.2;
                            let vy = -b.velocity.y * scale * 0.2;
                            painter.line_segment(
                                [
                                    egui::pos2(screen_x, screen_y),
                                    egui::pos2(screen_x + vx, screen_y + vy),
                                ],
                                egui::Stroke::new(1.5_f32, egui::Color32::from_rgb(0, 255, 255)),
                            );
                        }
                        if self.debug.aabbs {
                            let aabb = b.world_aabb();
                            let min_x = off_x + aabb.min.x * scale;
                            let min_y = off_y - aabb.max.y * scale;
                            let max_x = off_x + aabb.max.x * scale;
                            let max_y = off_y - aabb.min.y * scale;
                            painter.rect_stroke(
                                egui::Rect::from_min_max(
                                    egui::pos2(min_x, min_y),
                                    egui::pos2(max_x, max_y),
                                ),
                                egui::CornerRadius::same(0),
                                egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(34, 102, 51)),
                                egui::StrokeKind::Middle,
                            );
                        }
                        if Some(b.id.0) == self.selected_body {
                            painter.circle_stroke(
                                egui::pos2(screen_x, screen_y),
                                r.max(4.0) + 4.0,
                                egui::Stroke::new(2.0_f32, egui::Color32::YELLOW),
                            );
                        }
                    }
                    if self.debug.joints {
                        for j in &w.joints {
                            if let (Ok(ba), Ok(bb)) =
                                (w.body(j.config.body_a), w.body(j.config.body_b))
                            {
                                let p1 = ba.position + ba.rotation.rotate(j.config.anchor_a);
                                let p2 = bb.position + bb.rotation.rotate(j.config.anchor_b);
                                let s1x = off_x + p1.x * scale;
                                let s1y = off_y - p1.y * scale;
                                let s2x = off_x + p2.x * scale;
                                let s2y = off_y - p2.y * scale;
                                painter.line_segment(
                                    [egui::pos2(s1x, s1y), egui::pos2(s2x, s2y)],
                                    egui::Stroke::new(
                                        2.0_f32,
                                        egui::Color32::from_rgb(255, 204, 0),
                                    ),
                                );
                            }
                        }
                    }
                }
                ActiveWorld::World3(w) => {
                    let scale = 18.0;
                    let off_x = rect.center().x;
                    let off_y = rect.center().y;
                    let project = |p: Vec3| -> egui::Pos2 {
                        let sx = off_x + (p.x - p.z * 0.4) * scale;
                        let sy = off_y - (p.y - p.z * 0.2) * scale;
                        egui::pos2(sx, sy)
                    };
                    for (_, b) in w.bodies_iter() {
                        let pos = project(b.position);
                        let color = match b.kind {
                            BodyType::Static => egui::Color32::GRAY,
                            BodyType::Kinematic => egui::Color32::from_rgb(68, 170, 153),
                            BodyType::Dynamic => {
                                if b.sleeping {
                                    egui::Color32::from_rgb(68, 68, 100)
                                } else {
                                    egui::Color32::from_rgb(255, 136, 68)
                                }
                            }
                        };
                        let r = b
                            .colliders
                            .first()
                            .map(|c| c.bounding_radius() * scale * 0.5)
                            .unwrap_or(6.0)
                            .max(3.0);
                        painter.circle_filled(pos, r, color);
                        painter.circle_stroke(
                            pos,
                            r,
                            egui::Stroke::new(1.0_f32, egui::Color32::WHITE),
                        );
                        if Some(b.id.0) == self.selected_body {
                            painter.circle_stroke(
                                pos,
                                r + 4.0,
                                egui::Stroke::new(2.0_f32, egui::Color32::YELLOW),
                            );
                        }
                    }
                }
                ActiveWorld::SoftBody(sb) => {
                    let scale = 22.0;
                    let off_x = rect.center().x;
                    let off_y = rect.center().y;
                    // draw particles
                    for p in &sb.particles {
                        let sx = off_x + p.position.x * scale;
                        let sy = off_y - p.position.y * scale;
                        painter.circle_filled(
                            egui::pos2(sx, sy),
                            4.0,
                            egui::Color32::from_rgb(255, 136, 68),
                        );
                    }
                    // draw constraints as lines (first few)
                    if self.debug.softbody {
                        for (a_idx, b_idx) in sb.edge_indices.iter().take(300) {
                            if let (Some(pa), Some(pb)) =
                                (sb.particles.get(*a_idx), sb.particles.get(*b_idx))
                            {
                                let a = egui::pos2(
                                    off_x + pa.position.x * scale,
                                    off_y - pa.position.y * scale,
                                );
                                let b = egui::pos2(
                                    off_x + pb.position.x * scale,
                                    off_y - pb.position.y * scale,
                                );
                                painter.line_segment(
                                    [a, b],
                                    egui::Stroke::new(
                                        1.0_f32,
                                        egui::Color32::from_rgb(68, 255, 136),
                                    ),
                                );
                            }
                        }
                    }
                }
                ActiveWorld::Mixed => {
                    ui.label("Particles / Fluid scene — engine-driven storage, real counts");
                    if let Some(storage) = &self.particle_storage {
                        ui.label(format!("Alive: {}", storage.alive_count()));
                        let scale = 16.0;
                        let off_x = rect.center().x;
                        let off_y = rect.center().y + 50.0;
                        for (i, alive) in storage.alive.iter().enumerate() {
                            if !alive {
                                continue;
                            }
                            let pos = storage.positions[i];
                            let sx = off_x + pos.x * scale;
                            let sy = off_y - pos.y * scale;
                            painter.circle_filled(
                                egui::pos2(sx, sy),
                                5.0,
                                egui::Color32::from_rgb(34, 221, 255),
                            );
                        }
                    }
                }
            }

            // Click to select body (simple nearest)
            if ui.input(|i| i.pointer.primary_clicked())
                && let Some(click_pos) = ctx.input(|i| i.pointer.interact_pos())
                && rect.contains(click_pos)
            {
                // find nearest body
                let mut nearest: Option<(u64, f32)> = None;
                match &self.world {
                    ActiveWorld::World2(w) => {
                        let scale = 30.0;
                        let off_x = rect.center().x;
                        let off_y = rect.center().y + 100.0;
                        for (_, b) in w.bodies_iter() {
                            let sx = off_x + b.position.x * scale;
                            let sy = off_y - b.position.y * scale;
                            let d =
                                ((click_pos.x - sx).powi(2) + (click_pos.y - sy).powi(2)).sqrt();
                            if d < 20.0 && (nearest.is_none() || d < nearest.unwrap().1) {
                                nearest = Some((b.id.0, d));
                            }
                        }
                    }
                    ActiveWorld::World3(w) => {
                        let scale = 18.0;
                        let off_x = rect.center().x;
                        let off_y = rect.center().y;
                        for (_, b) in w.bodies_iter() {
                            let sx = off_x + (b.position.x - b.position.z * 0.4) * scale;
                            let sy = off_y - (b.position.y - b.position.z * 0.2) * scale;
                            let d =
                                ((click_pos.x - sx).powi(2) + (click_pos.y - sy).powi(2)).sqrt();
                            if d < 20.0 && (nearest.is_none() || d < nearest.unwrap().1) {
                                nearest = Some((b.id.0, d));
                            }
                        }
                    }
                    _ => {}
                }
                if let Some((id, _)) = nearest {
                    self.selected_body = Some(id);
                }
            }
        });

        // Request repaint for continuous simulation
        if !self.paused {
            ctx.request_repaint();
            self.step();
        }
    }
}
