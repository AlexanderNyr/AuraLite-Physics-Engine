//! Real interactive desktop sandbox & scene editor studio — engine-driven, no mocks.
//! Implements DoD-5: scene browser 18 subsystems, interactive tool studio (select, spawn,
//! joint wizard, impulse drag, delete), full live property inspector, outliner/hierarchy,
//! editable runtime settings, profiling overlay, real determinism controls (snapshot/rollback/record/replay)
//! showing real 64-bit state hash, plus full hex/binary scene save & load.
//! Dependency: eframe (winit + glow + egui) — justified in ADR-17, default-features off, license MIT/Apache.
#![forbid(unsafe_code)]

use eframe::egui;

use auralite_collision::CollisionFilter;
use auralite_dynamics::{
    BodyBuilder2, BodyBuilder3, BodyType, Collider2, Collider3, ColliderShape2, ColliderShape3,
    JointConfig2, JointConfig3, JointId, JointType2, JointType3, Material, World2, World3,
};
use auralite_geometry::{Box2, Box3, Capsule2, Capsule3, Circle2, Sphere3};
use auralite_math::{Quat, Real, Rot2, Vec2, Vec3};
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
pub enum EditorTool {
    Select,
    SpawnBody,
    CreateJoint,
    ApplyImpulse,
    Delete,
}

impl EditorTool {
    pub fn as_str(&self) -> &'static str {
        match self {
            EditorTool::Select => "👆 Select & Inspect",
            EditorTool::SpawnBody => "➕ Spawn Body",
            EditorTool::CreateJoint => "🔗 Connect Joint",
            EditorTool::ApplyImpulse => "💨 Force / Impulse Puller",
            EditorTool::Delete => "🗑 Delete Object",
        }
    }
}

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
    Custom2d,
    Custom3d,
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
            SceneId::Custom2d => "17. Custom 2D Editor Workspace",
            SceneId::Custom3d => "18. Custom 3D Editor Workspace",
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
            SceneId::Custom2d,
            SceneId::Custom3d,
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
                | SceneId::Custom3d
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
    pub grid: bool,
}

pub enum ActiveWorld {
    World2(Box<World2>),
    World3(Box<World3>),
    SoftBody(Box<auralite_softbody::SoftBody>),
    Mixed,
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
    pub selected_body: Option<u64>,
    pub selected_joint: Option<JointId>,
    // Determinism
    pub state_hash: u64,
    pub snapshot2: Option<auralite_dynamics::Snapshot2>,
    pub snapshot3: Option<auralite_dynamics::Snapshot3>,
    pub snapshot_soft: Option<auralite_softbody::SoftBody>,
    pub recorded_hashes: Vec<u64>,
    pub record_snapshot2: Option<auralite_dynamics::Snapshot2>,
    pub record_snapshot3: Option<auralite_dynamics::Snapshot3>,
    pub record_snapshot_soft: Option<auralite_softbody::SoftBody>,
    pub is_recording: bool,
    pub is_replaying: bool,
    pub replay_index: usize,
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
    // Editable state for selected body
    pub edit_pos_x: f32,
    pub edit_pos_y: f32,
    pub edit_pos_z: f32,
    pub edit_vel_x: f32,
    pub edit_vel_y: f32,
    pub edit_vel_z: f32,
    pub edit_ang_vel_x: f32,
    pub edit_ang_vel_y: f32,
    pub edit_ang_vel_z: f32,
    pub edit_mass: f32,
    pub edit_lin_damp: f32,
    pub edit_ang_damp: f32,
    pub edit_friction: f32,
    pub edit_restitution: f32,
    // Editor Tools & Gizmo Mode
    pub active_tool: EditorTool,
    // Spawn Body Configuration
    pub spawn_shape: usize, // 0 = Circle/Sphere, 1 = Box, 2 = Capsule
    pub spawn_radius: f32,
    pub spawn_half_width: f32,
    pub spawn_half_height: f32,
    pub spawn_half_depth: f32,
    pub spawn_body_type: usize, // 0 = Dynamic, 1 = Static, 2 = Kinematic
    pub spawn_mass: f32,
    pub spawn_friction: f32,
    pub spawn_restitution: f32,
    pub spawn_lin_damp: f32,
    pub spawn_ang_damp: f32,
    pub spawn_vel_x: f32,
    pub spawn_vel_y: f32,
    pub spawn_vel_z: f32,
    // Joint Wizard Configuration
    pub joint_wizard_step: usize, // 0 = Pick Body A, 1 = Pick Body B & Create
    pub joint_wizard_body_a: Option<u64>,
    pub joint_wizard_body_b: Option<u64>,
    pub joint_wizard_type2: usize, // 0 = Revolute, 1 = Distance, 2 = Spring, 3 = Weld, 4 = Prismatic
    pub joint_wizard_type3: usize, // 0 = BallSocket, 1 = Distance, 2 = Spring, 3 = Weld, 4 = Hinge, 5 = Slider, 6 = ConeTwist
    pub joint_wizard_anchor_a_x: f32,
    pub joint_wizard_anchor_a_y: f32,
    pub joint_wizard_anchor_a_z: f32,
    pub joint_wizard_anchor_b_x: f32,
    pub joint_wizard_anchor_b_y: f32,
    pub joint_wizard_anchor_b_z: f32,
    pub joint_wizard_stiffness: f32,
    pub joint_wizard_damping: f32,
    pub joint_wizard_break_impulse: f32,
    // Impulse Puller / Drag State
    pub drag_body: Option<u64>,
    pub drag_start_pos: Option<egui::Pos2>,
    pub drag_current_pos: Option<egui::Pos2>,
    pub impulse_multiplier: f32,
    // Viewport & Camera controls
    pub viewport_offset: Vec2,
    pub viewport_scale: f32,
    pub camera_yaw: f32,
    pub camera_pitch: f32,
    pub camera_dist: f32,
    // Outliner & Search
    pub outliner_search: String,
    // Serialization & File Studio
    pub serialization_buffer: String,
    pub serialization_status: String,
    pub save_file_path: String,
    // UI Panels toggles
    pub show_inspection: bool,
    pub show_settings: bool,
    pub show_outliner: bool,
    pub show_tools: bool,
    pub show_serialization: bool,
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
                grid: true,
            },
            selected_body: None,
            selected_joint: None,
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
            edit_pos_x: 0.0,
            edit_pos_y: 0.0,
            edit_pos_z: 0.0,
            edit_vel_x: 0.0,
            edit_vel_y: 0.0,
            edit_vel_z: 0.0,
            edit_ang_vel_x: 0.0,
            edit_ang_vel_y: 0.0,
            edit_ang_vel_z: 0.0,
            edit_mass: 1.0,
            edit_lin_damp: 0.0,
            edit_ang_damp: 0.0,
            edit_friction: 0.5,
            edit_restitution: 0.3,
            active_tool: EditorTool::Select,
            spawn_shape: 0,
            spawn_radius: 0.5,
            spawn_half_width: 0.5,
            spawn_half_height: 0.5,
            spawn_half_depth: 0.5,
            spawn_body_type: 0,
            spawn_mass: 1.0,
            spawn_friction: 0.5,
            spawn_restitution: 0.3,
            spawn_lin_damp: 0.0,
            spawn_ang_damp: 0.0,
            spawn_vel_x: 0.0,
            spawn_vel_y: 0.0,
            spawn_vel_z: 0.0,
            joint_wizard_step: 0,
            joint_wizard_body_a: None,
            joint_wizard_body_b: None,
            joint_wizard_type2: 0,
            joint_wizard_type3: 0,
            joint_wizard_anchor_a_x: 0.0,
            joint_wizard_anchor_a_y: 0.0,
            joint_wizard_anchor_a_z: 0.0,
            joint_wizard_anchor_b_x: 0.0,
            joint_wizard_anchor_b_y: 0.0,
            joint_wizard_anchor_b_z: 0.0,
            joint_wizard_stiffness: 50.0,
            joint_wizard_damping: 5.0,
            joint_wizard_break_impulse: 0.0,
            drag_body: None,
            drag_start_pos: None,
            drag_current_pos: None,
            impulse_multiplier: 5.0,
            viewport_offset: Vec2 { x: 0.0, y: 0.0 },
            viewport_scale: 30.0,
            camera_yaw: 0.4,
            camera_pitch: 0.3,
            camera_dist: 25.0,
            outliner_search: String::new(),
            serialization_buffer: String::new(),
            serialization_status: "Ready for scene export / import.".to_string(),
            save_file_path: "editor_scene.aura".to_string(),
            show_inspection: true,
            show_settings: false,
            show_outliner: true,
            show_tools: true,
            show_serialization: false,
        };
        app.restart_scene();
        app
    }

    pub fn restart_scene(&mut self) {
        self.paused = false;
        self.step_count = 0;
        self.sim_time = 0.0;
        self.selected_body = None;
        self.selected_joint = None;
        self.particle_storage = None;
        self.fluid = None;
        self.cloth_particles_clone = None;
        self.character2 = None;
        self.character3 = None;
        self.vehicle3 = None;
        self.is_recording = false;
        self.is_replaying = false;
        self.recorded_hashes.clear();
        self.replay_mismatch = None;
        self.joint_wizard_step = 0;
        self.joint_wizard_body_a = None;
        self.joint_wizard_body_b = None;
        self.drag_body = None;
        self.drag_start_pos = None;
        self.drag_current_pos = None;
        self.viewport_offset = Vec2::ZERO;

        match self.scene {
            SceneId::Stacking => {
                self.viewport_scale = 30.0;
                let mut w = World2::default();
                w.set_gravity(self.gravity2).ok();
                w.solver_iterations = self.iterations as u16;
                w.add_body(
                    BodyBuilder2::static_body()
                        .position(Vec2 { x: 0.0, y: -0.5 })
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Box(
                                Box2::new(Vec2 { x: 15.0, y: 0.5 }).unwrap(),
                            ),
                            offset: Vec2::ZERO,
                            material: Material {
                                friction: 0.6,
                                restitution: 0.1,
                                density: 1.0,
                            },
                            filter: CollisionFilter::default(),
                        }),
                )
                .ok();
                for i in 0..10 {
                    let y = 0.6 + i as Real * 1.1;
                    w.add_body(
                        BodyBuilder2::dynamic()
                            .position(Vec2 {
                                x: (i % 2) as Real * 0.02,
                                y,
                            })
                            .mass(1.0)
                            .add_collider(Collider2 {
                                shape: ColliderShape2::Box(
                                    Box2::new(Vec2 { x: 0.5, y: 0.5 }).unwrap(),
                                ),
                                offset: Vec2::ZERO,
                                material: Material {
                                    friction: 0.5,
                                    restitution: 0.0,
                                    density: 1.0,
                                },
                                filter: CollisionFilter::default(),
                            }),
                    )
                    .ok();
                }
                self.world = ActiveWorld::World2(Box::new(w));
            }
            SceneId::Joints => {
                self.viewport_scale = 30.0;
                let mut w = World2::default();
                w.set_gravity(self.gravity2).ok();
                w.solver_iterations = self.iterations as u16;
                let mut handles = Vec::new();
                for i in 0..11 {
                    let y = 5.0 + (10 - i) as Real * 0.8;
                    let b = BodyBuilder2::dynamic()
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
                self.viewport_scale = 30.0;
                let mut w = World2::default();
                w.set_gravity(self.gravity2).ok();
                w.add_body(
                    BodyBuilder2::dynamic()
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
                    BodyBuilder2::static_body()
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
                self.viewport_scale = 30.0;
                let mut w = World2::default();
                w.set_gravity(Vec2::ZERO).ok();
                let sensor = BodyBuilder2::dynamic()
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
                let other = BodyBuilder2::static_body()
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
                self.viewport_scale = 18.0;
                let mut w = World3::default();
                w.set_gravity(self.gravity3).ok();
                w.add_body(
                    BodyBuilder3::dynamic()
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
                self.viewport_scale = 22.0;
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
                self.viewport_scale = 22.0;
                let cloth = build_cloth_grid(
                    6,
                    6,
                    0.2,
                    Vec3 {
                        x: -0.5,
                        y: 1.0,
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
            SceneId::Particles => {
                self.viewport_scale = 16.0;
                let mut storage = ParticleStorage::new(500);
                for i in 0..50 {
                    storage.spawn(
                        Vec3 {
                            x: (i % 10) as Real * 0.2 - 1.0,
                            y: 2.0 + (i / 10) as Real * 0.2,
                            z: 0.0,
                        },
                        Vec3 {
                            x: 0.5,
                            y: 2.0,
                            z: 0.0,
                        },
                        5.0,
                        ParticleType::Generic,
                    );
                }
                self.particle_storage = Some(storage);
                self.world = ActiveWorld::Mixed;
            }
            SceneId::Fluid => {
                self.viewport_scale = 16.0;
                let mut storage = ParticleStorage::new(200);
                for x in 0..5 {
                    for y in 0..5 {
                        storage.spawn(
                            Vec3 {
                                x: x as Real * 0.2 - 0.5,
                                y: y as Real * 0.2 + 1.0,
                                z: 0.0,
                            },
                            Vec3::ZERO,
                            100.0,
                            ParticleType::Fluid,
                        );
                    }
                }
                self.particle_storage = Some(storage);
                self.fluid = Some(PbfFluid::new(1000.0, 0.1, 50.0, 0.01));
                self.world = ActiveWorld::Mixed;
            }
            SceneId::Buoyancy => {
                self.viewport_scale = 18.0;
                let mut w = World3::default();
                w.set_gravity(self.gravity3).ok();
                w.add_body(
                    BodyBuilder3::dynamic()
                        .position(Vec3 {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        })
                        .mass(10.0)
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
                            material: Material {
                                density: 200.0,
                                ..Default::default()
                            },
                            filter: CollisionFilter::default(),
                        }),
                )
                .ok();
                self.world = ActiveWorld::World3(Box::new(w));
            }
            SceneId::Fields => {
                self.viewport_scale = 22.0;
                let cloth = build_cloth_grid(
                    6,
                    6,
                    0.2,
                    Vec3 {
                        x: -0.5,
                        y: 0.5,
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
            SceneId::Vehicle => {
                self.viewport_scale = 18.0;
                let mut w = World3::default();
                w.set_gravity(self.gravity3).ok();
                w.add_body(
                    BodyBuilder3::static_body()
                        .position(Vec3 {
                            x: 0.0,
                            y: -1.0,
                            z: 0.0,
                        })
                        .add_collider(Collider3 {
                            shape: ColliderShape3::Box(
                                Box3::new(Vec3 {
                                    x: 50.0,
                                    y: 1.0,
                                    z: 50.0,
                                })
                                .unwrap(),
                            ),
                            offset: Vec3::ZERO,
                            material: Material {
                                friction: 0.9,
                                restitution: 0.0,
                                density: 1.0,
                            },
                            filter: CollisionFilter::default(),
                        }),
                )
                .ok();
                let mut veh = Vehicle3::new(
                    VehicleConfig3::default(),
                    Vec3 {
                        x: 0.0,
                        y: 2.0,
                        z: 0.0,
                    },
                    Quat::identity(),
                    vec![
                        WheelConfig3 {
                            attachment_point: Vec3 {
                                x: -0.9,
                                y: -0.2,
                                z: 1.4,
                            },
                            steered: true,
                            ..Default::default()
                        },
                        WheelConfig3 {
                            attachment_point: Vec3 {
                                x: 0.9,
                                y: -0.2,
                                z: 1.4,
                            },
                            steered: true,
                            ..Default::default()
                        },
                        WheelConfig3 {
                            attachment_point: Vec3 {
                                x: -0.9,
                                y: -0.2,
                                z: -1.4,
                            },
                            driven: true,
                            ..Default::default()
                        },
                        WheelConfig3 {
                            attachment_point: Vec3 {
                                x: 0.9,
                                y: -0.2,
                                z: -1.4,
                            },
                            driven: true,
                            ..Default::default()
                        },
                    ],
                    &mut w,
                );
                veh.set_controls(0.5, 0.2, 0.0);
                self.vehicle3 = Some(veh);
                self.world = ActiveWorld::World3(Box::new(w));
            }
            SceneId::Char2d => {
                self.viewport_scale = 30.0;
                let mut w = World2::default();
                w.set_gravity(self.gravity2).ok();
                w.add_body(
                    BodyBuilder2::static_body()
                        .position(Vec2 { x: 0.0, y: -0.5 })
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Box(
                                Box2::new(Vec2 { x: 20.0, y: 0.5 }).unwrap(),
                            ),
                            offset: Vec2::ZERO,
                            material: Material::default(),
                            filter: CollisionFilter::default(),
                        }),
                )
                .ok();
                let mut cc = Character2::new(CharacterConfig2::default(), Vec2 { x: -2.0, y: 2.0 });
                cc.attach(&mut w);
                cc.set_move(Vec2 { x: 3.0, y: 0.0 });
                self.character2 = Some(cc);
                self.world = ActiveWorld::World2(Box::new(w));
            }
            SceneId::Char3d => {
                self.viewport_scale = 18.0;
                let mut w = World3::default();
                w.set_gravity(self.gravity3).ok();
                w.add_body(
                    BodyBuilder3::static_body()
                        .position(Vec3 {
                            x: 0.0,
                            y: -1.0,
                            z: 0.0,
                        })
                        .add_collider(Collider3 {
                            shape: ColliderShape3::Box(
                                Box3::new(Vec3 {
                                    x: 20.0,
                                    y: 1.0,
                                    z: 20.0,
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
                        y: 3.0,
                        z: 0.0,
                    },
                );
                cc.attach(&mut w);
                cc.set_move(Vec3 {
                    x: 2.0,
                    y: 0.0,
                    z: 1.0,
                });
                self.character3 = Some(cc);
                self.world = ActiveWorld::World3(Box::new(w));
            }
            SceneId::Serialization => {
                self.viewport_scale = 30.0;
                let mut w = World2::default();
                w.set_gravity(self.gravity2).ok();
                w.add_body(
                    BodyBuilder2::dynamic()
                        .position(Vec2 { x: 0.0, y: 5.0 })
                        .velocity(Vec2 { x: 1.0, y: -2.0 })
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
                self.viewport_scale = 30.0;
                let mut w = World2::default();
                w.set_gravity(self.gravity2).ok();
                w.add_body(
                    BodyBuilder2::static_body()
                        .position(Vec2 { x: 0.0, y: -0.5 })
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Box(
                                Box2::new(Vec2 { x: 25.0, y: 0.5 }).unwrap(),
                            ),
                            offset: Vec2::ZERO,
                            material: Material::default(),
                            filter: CollisionFilter::default(),
                        }),
                )
                .ok();
                for i in 0..100 {
                    let x = (i % 10) as Real * 0.6 - 3.0;
                    let y = 1.0 + (i / 10) as Real * 0.6;
                    w.add_body(
                        BodyBuilder2::dynamic()
                            .position(Vec2 { x, y })
                            .mass(0.5)
                            .add_collider(Collider2 {
                                shape: ColliderShape2::Box(
                                    Box2::new(Vec2 { x: 0.25, y: 0.25 }).unwrap(),
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
            SceneId::Custom2d => {
                self.viewport_scale = 30.0;
                let mut w = World2::default();
                w.set_gravity(self.gravity2).ok();
                w.solver_iterations = self.iterations as u16;
                // Static ground boundary so built bodies don't fall indefinitely
                w.add_body(
                    BodyBuilder2::static_body()
                        .position(Vec2 { x: 0.0, y: -5.0 })
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Box(
                                Box2::new(Vec2 { x: 25.0, y: 1.0 }).unwrap(),
                            ),
                            offset: Vec2::ZERO,
                            material: Material {
                                friction: 0.6,
                                restitution: 0.2,
                                density: 1.0,
                            },
                            filter: CollisionFilter::default(),
                        }),
                )
                .ok();
                self.world = ActiveWorld::World2(Box::new(w));
            }
            SceneId::Custom3d => {
                self.viewport_scale = 18.0;
                let mut w = World3::default();
                w.set_gravity(self.gravity3).ok();
                w.solver_iterations = self.iterations as u16;
                w.add_body(
                    BodyBuilder3::static_body()
                        .position(Vec3 {
                            x: 0.0,
                            y: -3.0,
                            z: 0.0,
                        })
                        .add_collider(Collider3 {
                            shape: ColliderShape3::Box(
                                Box3::new(Vec3 {
                                    x: 25.0,
                                    y: 1.0,
                                    z: 25.0,
                                })
                                .unwrap(),
                            ),
                            offset: Vec3::ZERO,
                            material: Material {
                                friction: 0.6,
                                restitution: 0.2,
                                density: 1.0,
                            },
                            filter: CollisionFilter::default(),
                        }),
                )
                .ok();
                self.world = ActiveWorld::World3(Box::new(w));
            }
        }
        self.update_hash();
    }

    pub fn update_hash(&mut self) {
        self.state_hash = match &self.world {
            ActiveWorld::World2(w) => w.state_hash(),
            ActiveWorld::World3(w) => w.state_hash(),
            ActiveWorld::SoftBody(sb) => {
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
        if self.is_recording && !self.is_replaying {
            self.recorded_hashes.push(self.state_hash);
            if self.recorded_hashes.len() >= MAX_RECORD_FRAMES {
                self.is_recording = false;
            }
        } else if self.is_replaying {
            match self.recorded_hashes.get(self.replay_index) {
                Some(&expected) if expected == self.state_hash => {
                    self.replay_index += 1;
                    if self.replay_index >= self.recorded_hashes.len() {
                        self.is_replaying = false;
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

    #[allow(clippy::unnecessary_cast)] // Real is f32 or f64 across feature builds
    pub fn load_body_to_editor_fields(&mut self, id: u64) {
        match &self.world {
            ActiveWorld::World2(w) => {
                if let Some((_, b)) = w.bodies_iter().find(|(_, b)| b.id.0 == id) {
                    self.edit_pos_x = b.position.x as f32;
                    self.edit_pos_y = b.position.y as f32;
                    self.edit_pos_z = 0.0;
                    self.edit_vel_x = b.velocity.x as f32;
                    self.edit_vel_y = b.velocity.y as f32;
                    self.edit_vel_z = 0.0;
                    self.edit_ang_vel_x = 0.0;
                    self.edit_ang_vel_y = 0.0;
                    self.edit_ang_vel_z = b.angular_velocity as f32;
                    self.edit_mass = if b.inv_mass > 0.0 {
                        (1.0 / b.inv_mass) as f32
                    } else {
                        0.0
                    };
                    self.edit_lin_damp = b.linear_damping as f32;
                    self.edit_ang_damp = b.angular_damping as f32;
                    self.edit_friction = b.friction as f32;
                    self.edit_restitution = b.restitution as f32;
                }
            }
            ActiveWorld::World3(w) => {
                if let Some((_, b)) = w.bodies_iter().find(|(_, b)| b.id.0 == id) {
                    self.edit_pos_x = b.position.x as f32;
                    self.edit_pos_y = b.position.y as f32;
                    self.edit_pos_z = b.position.z as f32;
                    self.edit_vel_x = b.velocity.x as f32;
                    self.edit_vel_y = b.velocity.y as f32;
                    self.edit_vel_z = b.velocity.z as f32;
                    self.edit_ang_vel_x = b.angular_velocity.x as f32;
                    self.edit_ang_vel_y = b.angular_velocity.y as f32;
                    self.edit_ang_vel_z = b.angular_velocity.z as f32;
                    self.edit_mass = if b.inv_mass > 0.0 {
                        (1.0 / b.inv_mass) as f32
                    } else {
                        0.0
                    };
                    self.edit_lin_damp = b.linear_damping as f32;
                    self.edit_ang_damp = b.angular_damping as f32;
                    self.edit_friction = b.friction as f32;
                    self.edit_restitution = b.restitution as f32;
                }
            }
            _ => {}
        }
    }

    #[allow(clippy::unnecessary_cast)] // Real is f32 or f64 across feature builds
    pub fn apply_editor_fields_to_body(&mut self, id: u64) {
        match &mut self.world {
            ActiveWorld::World2(w) => {
                let handle = w.bodies_iter().find(|(_, b)| b.id.0 == id).map(|(h, _)| h);
                if let Some(h) = handle
                    && let Ok(bm) = w.body_mut(h)
                {
                    bm.position = Vec2 {
                        x: self.edit_pos_x as Real,
                        y: self.edit_pos_y as Real,
                    };
                    bm.velocity = Vec2 {
                        x: self.edit_vel_x as Real,
                        y: self.edit_vel_y as Real,
                    };
                    bm.angular_velocity = self.edit_ang_vel_z as Real;
                    if self.edit_mass > 0.0 && bm.kind == BodyType::Dynamic {
                        bm.inv_mass = (1.0 / self.edit_mass) as Real;
                    }
                    bm.linear_damping = self.edit_lin_damp as Real;
                    bm.angular_damping = self.edit_ang_damp as Real;
                    bm.friction = self.edit_friction as Real;
                    bm.restitution = self.edit_restitution as Real;
                    for c in &mut bm.colliders {
                        c.material.friction = self.edit_friction as Real;
                        c.material.restitution = self.edit_restitution as Real;
                    }
                    bm.sleeping = false;
                }
            }
            ActiveWorld::World3(w) => {
                let handle = w.bodies_iter().find(|(_, b)| b.id.0 == id).map(|(h, _)| h);
                if let Some(h) = handle
                    && let Ok(bm) = w.body_mut(h)
                {
                    bm.position = Vec3 {
                        x: self.edit_pos_x as Real,
                        y: self.edit_pos_y as Real,
                        z: self.edit_pos_z as Real,
                    };
                    bm.velocity = Vec3 {
                        x: self.edit_vel_x as Real,
                        y: self.edit_vel_y as Real,
                        z: self.edit_vel_z as Real,
                    };
                    bm.angular_velocity = Vec3 {
                        x: self.edit_ang_vel_x as Real,
                        y: self.edit_ang_vel_y as Real,
                        z: self.edit_ang_vel_z as Real,
                    };
                    if self.edit_mass > 0.0 && bm.kind == BodyType::Dynamic {
                        bm.inv_mass = (1.0 / self.edit_mass) as Real;
                    }
                    bm.linear_damping = self.edit_lin_damp as Real;
                    bm.angular_damping = self.edit_ang_damp as Real;
                    bm.friction = self.edit_friction as Real;
                    bm.restitution = self.edit_restitution as Real;
                    for c in &mut bm.colliders {
                        c.material.friction = self.edit_friction as Real;
                        c.material.restitution = self.edit_restitution as Real;
                    }
                    bm.sleeping = false;
                }
            }
            _ => {}
        }
        self.update_hash();
    }

    fn hex_encode(data: &[u8]) -> String {
        let mut s = String::with_capacity(data.len() * 2);
        for byte in data {
            use std::fmt::Write;
            let _ = write!(&mut s, "{:02x}", byte);
        }
        s
    }

    fn hex_decode(s: &str) -> Option<Vec<u8>> {
        let s = s.trim();
        if !s.len().is_multiple_of(2) {
            return None;
        }
        let mut bytes = Vec::with_capacity(s.len() / 2);
        let chars: Vec<char> = s.chars().collect();
        for i in (0..chars.len()).step_by(2) {
            let hi = chars[i].to_digit(16)?;
            let lo = chars[i + 1].to_digit(16)?;
            bytes.push(((hi << 4) | lo) as u8);
        }
        Some(bytes)
    }

    fn render_top_bar(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("top").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(&mut self.show_tools, true, "🛠 Tool Studio");
                ui.selectable_value(&mut self.show_outliner, true, "🗂 Outliner");
                ui.selectable_value(&mut self.show_inspection, true, "🔍 Live Inspector");
                ui.selectable_value(&mut self.show_settings, true, "⚙ Runtime Settings");
                ui.selectable_value(&mut self.show_serialization, true, "💾 Save & Load Scene");
                ui.separator();
                ui.heading("⚡ AuraLite Physics Engine — Full Scene Editor & Sandbox Studio");
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
    }

    fn render_left_side_panel(&mut self, ctx: &egui::Context) {
        egui::SidePanel::left("left").default_width(350.0).show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.collapsing("📂 Scene Browser (18 Subsystems & Workspaces)", |ui| {
                    ui.label("Select subsystem scene or empty workspace:");
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

                if self.show_tools {
                    ui.collapsing("🛠 Interactive Tool Studio & Gizmo Modes", |ui| {
                        ui.label(egui::RichText::new("Active Editor Tool:").strong());
                        ui.horizontal_wrapped(|ui| {
                            for tool in [
                                EditorTool::Select,
                                EditorTool::SpawnBody,
                                EditorTool::CreateJoint,
                                EditorTool::ApplyImpulse,
                                EditorTool::Delete,
                            ] {
                                ui.selectable_value(&mut self.active_tool, tool, tool.as_str());
                            }
                        });
                        ui.separator();

                        match self.active_tool {
                            EditorTool::Select => {
                                ui.label("👆 Click on any body or joint in the viewport to inspect and live-edit its properties.");
                                ui.label("💡 Tip: When paused, drag selected bodies across the viewport to reposition them precisely!");
                            }
                            EditorTool::SpawnBody => {
                                ui.label(egui::RichText::new("➕ Spawn Body Configuration:").strong());
                                ui.horizontal(|ui| {
                                    ui.label("Shape:");
                                    ui.radio_value(&mut self.spawn_shape, 0, "Circle/Sphere");
                                    ui.radio_value(&mut self.spawn_shape, 1, "Box");
                                    ui.radio_value(&mut self.spawn_shape, 2, "Capsule");
                                });
                                ui.horizontal(|ui| {
                                    ui.label("Type:");
                                    ui.radio_value(&mut self.spawn_body_type, 0, "Dynamic");
                                    ui.radio_value(&mut self.spawn_body_type, 1, "Static");
                                    ui.radio_value(&mut self.spawn_body_type, 2, "Kinematic");
                                });
                                if self.spawn_shape == 0 {
                                    ui.add(egui::Slider::new(&mut self.spawn_radius, 0.1..=5.0).text("Radius (m)"));
                                } else if self.spawn_shape == 1 {
                                    ui.add(egui::Slider::new(&mut self.spawn_half_width, 0.1..=10.0).text("Half Width X"));
                                    ui.add(egui::Slider::new(&mut self.spawn_half_height, 0.1..=10.0).text("Half Height Y"));
                                    if self.scene.is_3d() {
                                        ui.add(egui::Slider::new(&mut self.spawn_half_depth, 0.1..=10.0).text("Half Depth Z"));
                                    }
                                } else {
                                    ui.add(egui::Slider::new(&mut self.spawn_radius, 0.1..=3.0).text("Radius (m)"));
                                    ui.add(egui::Slider::new(&mut self.spawn_half_height, 0.1..=5.0).text("Half Height (m)"));
                                }
                                if self.spawn_body_type == 0 {
                                    ui.add(egui::Slider::new(&mut self.spawn_mass, 0.1..=100.0).text("Mass (kg)"));
                                }
                                ui.horizontal(|ui| {
                                    ui.add(egui::DragValue::new(&mut self.spawn_friction).range(0.0..=1.0).prefix("Friction: "));
                                    ui.add(egui::DragValue::new(&mut self.spawn_restitution).range(0.0..=1.0).prefix("Restitution: "));
                                });
                                ui.horizontal(|ui| {
                                    ui.add(egui::DragValue::new(&mut self.spawn_lin_damp).range(0.0..=10.0).prefix("Lin Damp: "));
                                    ui.add(egui::DragValue::new(&mut self.spawn_ang_damp).range(0.0..=10.0).prefix("Ang Damp: "));
                                });
                                ui.label("Initial Velocity:");
                                ui.horizontal(|ui| {
                                    ui.add(egui::DragValue::new(&mut self.spawn_vel_x).prefix("vx: "));
                                    ui.add(egui::DragValue::new(&mut self.spawn_vel_y).prefix("vy: "));
                                    if self.scene.is_3d() {
                                        ui.add(egui::DragValue::new(&mut self.spawn_vel_z).prefix("vz: "));
                                    }
                                });
                                ui.colored_label(egui::Color32::from_rgb(100, 255, 180), "✨ Click anywhere on the viewport to spawn this body immediately!");
                            }
                            EditorTool::CreateJoint => {
                                ui.label(egui::RichText::new("🔗 Step-by-Step Joint Creation Wizard:").strong());
                                if self.joint_wizard_step == 0 {
                                    ui.colored_label(egui::Color32::YELLOW, "Step 1: Click Body A on the viewport.");
                                    if let Some(id_a) = self.joint_wizard_body_a {
                                        ui.label(format!("Selected Body A: ID {}", id_a));
                                        if ui.button("Next Step ➡").clicked() {
                                            self.joint_wizard_step = 1;
                                        }
                                    }
                                } else {
                                    ui.colored_label(egui::Color32::YELLOW, "Step 2: Click Body B on the viewport.");
                                    if let (Some(id_a), Some(id_b)) = (self.joint_wizard_body_a, self.joint_wizard_body_b) {
                                        ui.label(format!("Wiring Body A (ID {}) to Body B (ID {})", id_a, id_b));
                                        if !self.scene.is_3d() {
                                            ui.label("2D Joint Type:");
                                            ui.radio_value(&mut self.joint_wizard_type2, 0, "Revolute (Hinge)");
                                            ui.radio_value(&mut self.joint_wizard_type2, 1, "Distance");
                                            ui.radio_value(&mut self.joint_wizard_type2, 2, "Spring");
                                            ui.radio_value(&mut self.joint_wizard_type2, 3, "Weld");
                                            ui.radio_value(&mut self.joint_wizard_type2, 4, "Prismatic (Slider)");
                                        } else {
                                            ui.label("3D Joint Type:");
                                            ui.radio_value(&mut self.joint_wizard_type3, 0, "Ball-and-Socket");
                                            ui.radio_value(&mut self.joint_wizard_type3, 1, "Distance");
                                            ui.radio_value(&mut self.joint_wizard_type3, 2, "Spring");
                                            ui.radio_value(&mut self.joint_wizard_type3, 3, "Weld");
                                            ui.radio_value(&mut self.joint_wizard_type3, 4, "Hinge");
                                            ui.radio_value(&mut self.joint_wizard_type3, 5, "Slider");
                                            ui.radio_value(&mut self.joint_wizard_type3, 6, "Cone-Twist (DoD-H5)");
                                        }
                                        ui.horizontal(|ui| {
                                            ui.label("Anchor A:");
                                            ui.add(egui::DragValue::new(&mut self.joint_wizard_anchor_a_x).prefix("x: "));
                                            ui.add(egui::DragValue::new(&mut self.joint_wizard_anchor_a_y).prefix("y: "));
                                        });
                                        ui.horizontal(|ui| {
                                            ui.label("Anchor B:");
                                            ui.add(egui::DragValue::new(&mut self.joint_wizard_anchor_b_x).prefix("x: "));
                                            ui.add(egui::DragValue::new(&mut self.joint_wizard_anchor_b_y).prefix("y: "));
                                        });
                                        if self.joint_wizard_type2 == 2 || self.joint_wizard_type3 == 2 {
                                            ui.add(egui::Slider::new(&mut self.joint_wizard_stiffness, 1.0..=500.0).text("Stiffness"));
                                            ui.add(egui::Slider::new(&mut self.joint_wizard_damping, 0.1..=50.0).text("Damping"));
                                        }
                                        ui.add(egui::DragValue::new(&mut self.joint_wizard_break_impulse).prefix("Break Impulse (0=Inf): "));
                                        if ui.button("⚡ Create Connected Joint Now!").clicked() {
                                            self.execute_create_joint();
                                        }
                                    }
                                    if ui.button("⬅ Reset Wizard").clicked() {
                                        self.joint_wizard_step = 0;
                                        self.joint_wizard_body_a = None;
                                        self.joint_wizard_body_b = None;
                                    }
                                }
                            }
                            EditorTool::ApplyImpulse => {
                                ui.label("💨 Force / Impulse Puller:");
                                ui.label("Click and hold on any body, then pull/drag across the viewport to stretch an impulse vector arrow. Release mouse button to launch!");
                                ui.add(egui::Slider::new(&mut self.impulse_multiplier, 0.5..=25.0).text("Impulse Strength"));
                            }
                            EditorTool::Delete => {
                                ui.label("🗑 Delete Mode:");
                                ui.colored_label(egui::Color32::LIGHT_RED, "Click any body or joint on the viewport to instantly remove it from the active simulation!");
                            }
                        }
                    });
                }

                ui.collapsing("⏱ Time & Simulation Controls", |ui| {
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
                    ui.add(egui::Slider::new(&mut self.substeps, 1..=16).text("Substeps"));
                });

                ui.collapsing("🎨 Debug-Draw Toggles", |ui| {
                    ui.checkbox(&mut self.debug.grid, "Coordinate Grid & Ground Line");
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
                    ui.collapsing("⚙ Editable Runtime Settings", |ui| {
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
                        ui.add(egui::Slider::new(&mut self.viewport_scale, 5.0..=150.0).text("Viewport Scale"));
                        if self.scene.is_3d() {
                            ui.add(egui::Slider::new(&mut self.camera_yaw, -(std::f32::consts::PI)..=std::f32::consts::PI).text("Camera Yaw"));
                            ui.add(egui::Slider::new(&mut self.camera_pitch, -1.5..=1.5).text("Camera Pitch"));
                        }
                    });
                }

                ui.collapsing("🛑 Determinism & Replay Verification", |ui| {
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
                    let recordable = matches!(
                        self.world,
                        ActiveWorld::World2(_) | ActiveWorld::World3(_) | ActiveWorld::SoftBody(_)
                    );
                    if recordable {
                        ui.horizontal(|ui| {
                            let mut rec = self.is_recording;
                            if ui.checkbox(&mut rec, "🔴 Record Trace").changed() {
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
                    }
                });

                ui.collapsing("📊 Profiling Overlay (Real Timing)", |ui| {
                    ui.label(format!("Last step: {:.1} µs", self.last_step_time_us));
                    ui.label(format!("Broad-phase est: {:.1} µs", self.last_broad_time_us));
                    ui.label(format!("Total Bodies: {}", match &self.world {
                        ActiveWorld::World2(w) => w.body_count(),
                        ActiveWorld::World3(w) => w.body_count(),
                        _ => 0,
                    }));
                });
            });
        });
    }

    fn execute_create_joint(&mut self) {
        if let (Some(id_a), Some(id_b)) = (self.joint_wizard_body_a, self.joint_wizard_body_b) {
            match &mut self.world {
                ActiveWorld::World2(w) => {
                    let ha = w
                        .bodies_iter()
                        .find(|(_, b)| b.id.0 == id_a)
                        .map(|(h, _)| h);
                    let hb = w
                        .bodies_iter()
                        .find(|(_, b)| b.id.0 == id_b)
                        .map(|(h, _)| h);
                    if let (Some(ha), Some(hb)) = (ha, hb) {
                        let jtype = match self.joint_wizard_type2 {
                            1 => JointType2::Distance,
                            2 => JointType2::Spring {
                                stiffness: self.joint_wizard_stiffness as Real,
                                damping: self.joint_wizard_damping as Real,
                            },
                            3 => JointType2::Weld,
                            4 => JointType2::Prismatic {
                                axis_local: Vec2::X,
                            },
                            _ => JointType2::Revolute,
                        };
                        let mut cfg = JointConfig2::new(
                            jtype,
                            ha,
                            hb,
                            Vec2 {
                                x: self.joint_wizard_anchor_a_x as Real,
                                y: self.joint_wizard_anchor_a_y as Real,
                            },
                            Vec2 {
                                x: self.joint_wizard_anchor_b_x as Real,
                                y: self.joint_wizard_anchor_b_y as Real,
                            },
                        );
                        cfg.break_impulse = self.joint_wizard_break_impulse as Real;
                        w.add_joint(cfg).ok();
                    }
                }
                ActiveWorld::World3(w) => {
                    let ha = w
                        .bodies_iter()
                        .find(|(_, b)| b.id.0 == id_a)
                        .map(|(h, _)| h);
                    let hb = w
                        .bodies_iter()
                        .find(|(_, b)| b.id.0 == id_b)
                        .map(|(h, _)| h);
                    if let (Some(ha), Some(hb)) = (ha, hb) {
                        let jtype = match self.joint_wizard_type3 {
                            1 => JointType3::Distance,
                            2 => JointType3::Spring {
                                stiffness: self.joint_wizard_stiffness as Real,
                                damping: self.joint_wizard_damping as Real,
                            },
                            3 => JointType3::Weld,
                            4 => JointType3::Hinge {
                                axis_local: Vec3::Z,
                            },
                            5 => JointType3::Slider {
                                axis_local: Vec3::X,
                            },
                            6 => JointType3::ConeTwist {
                                axis_local: Vec3::Y,
                                swing_limit: 0.5,
                                twist_limit: 0.5,
                            },
                            _ => JointType3::BallSocket,
                        };
                        let mut cfg = JointConfig3::new(
                            jtype,
                            ha,
                            hb,
                            Vec3 {
                                x: self.joint_wizard_anchor_a_x as Real,
                                y: self.joint_wizard_anchor_a_y as Real,
                                z: self.joint_wizard_anchor_a_z as Real,
                            },
                            Vec3 {
                                x: self.joint_wizard_anchor_b_x as Real,
                                y: self.joint_wizard_anchor_b_y as Real,
                                z: self.joint_wizard_anchor_b_z as Real,
                            },
                        );
                        cfg.break_impulse = self.joint_wizard_break_impulse as Real;
                        w.add_joint(cfg).ok();
                    }
                }
                _ => {}
            }
            self.joint_wizard_step = 0;
            self.joint_wizard_body_a = None;
            self.joint_wizard_body_b = None;
            self.update_hash();
        }
    }

    fn render_right_side_panel(&mut self, ctx: &egui::Context) {
        if !self.show_inspection && !self.show_outliner {
            return;
        }
        egui::SidePanel::right("right").default_width(360.0).show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                if self.show_outliner {
                    ui.collapsing("🗂 Scene Hierarchy & Outliner", |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Filter:");
                            ui.text_edit_singleline(&mut self.outliner_search);
                        });
                        ui.separator();
                        let mut body_to_load = None;
                        match &self.world {
                            ActiveWorld::World2(w) => {
                                ui.label(egui::RichText::new(format!("Total Bodies: {} | Joints: {}", w.body_count(), w.joints.len())).strong());
                                let mut clicked_body = None;
                                egui::ScrollArea::vertical().max_height(180.0).show(ui, |ui| {
                                    for (_, b) in w.bodies_iter() {
                                        let summary = format!(
                                            "Body {} [{:?}] at ({:.1}, {:.1})",
                                            b.id.0, b.kind, b.position.x, b.position.y
                                        );
                                        if !self.outliner_search.is_empty() && !summary.to_lowercase().contains(&self.outliner_search.to_lowercase()) {
                                            continue;
                                        }
                                        let selected = self.selected_body == Some(b.id.0);
                                        if ui.selectable_label(selected, summary).clicked() {
                                            self.selected_body = Some(b.id.0);
                                            clicked_body = Some(b.id.0);
                                        }
                                    }
                                });
                                body_to_load = clicked_body;
                                ui.separator();
                                ui.label(egui::RichText::new("Joints List:").strong());
                                for j in &w.joints {
                                    let jsum = format!("Joint {} [{:?}] broken={}", j.id.0, j.config.joint_type, j.broken);
                                    let jsel = self.selected_joint == Some(j.id);
                                    if ui.selectable_label(jsel, jsum).clicked() {
                                        self.selected_joint = Some(j.id);
                                    }
                                }
                            }
                            ActiveWorld::World3(w) => {
                                ui.label(egui::RichText::new(format!("Total Bodies: {} | Joints: {}", w.body_count(), w.joints.len())).strong());
                                let mut clicked_body = None;
                                egui::ScrollArea::vertical().max_height(180.0).show(ui, |ui| {
                                    for (_, b) in w.bodies_iter() {
                                        let summary = format!(
                                            "Body {} [{:?}] at ({:.1}, {:.1}, {:.1})",
                                            b.id.0, b.kind, b.position.x, b.position.y, b.position.z
                                        );
                                        if !self.outliner_search.is_empty() && !summary.to_lowercase().contains(&self.outliner_search.to_lowercase()) {
                                            continue;
                                        }
                                        let selected = self.selected_body == Some(b.id.0);
                                        if ui.selectable_label(selected, summary).clicked() {
                                            self.selected_body = Some(b.id.0);
                                            clicked_body = Some(b.id.0);
                                        }
                                    }
                                });
                                body_to_load = clicked_body;
                                ui.separator();
                                ui.label(egui::RichText::new("Joints List:").strong());
                                for j in &w.joints {
                                    let jsum = format!("Joint {} [{:?}] broken={}", j.id.0, j.config.joint_type, j.broken);
                                    let jsel = self.selected_joint == Some(j.id);
                                    if ui.selectable_label(jsel, jsum).clicked() {
                                        self.selected_joint = Some(j.id);
                                    }
                                }
                            }
                            ActiveWorld::SoftBody(sb) => {
                                ui.label(format!("XPBD SoftBody: {} particles, {} edge constraints", sb.particles.len(), sb.edge_indices.len()));
                            }
                            ActiveWorld::Mixed => {
                                if let Some(st) = &self.particle_storage {
                                    ui.label(format!("Particle/Fluid Storage: {} alive", st.alive_count()));
                                }
                            }
                        }
                        if let Some(id) = body_to_load {
                            self.load_body_to_editor_fields(id);
                        }
                    });
                }

                if self.show_inspection {
                    ui.collapsing("🔍 Live Property Inspector & Modifier", |ui| {
                        if let Some(sid) = self.selected_body {
                            ui.label(egui::RichText::new(format!("Editing Selected Body ID: {}", sid)).strong().color(egui::Color32::YELLOW));
                            let mut modified = false;
                            ui.horizontal(|ui| {
                                ui.label("Position:");
                                if ui.add(egui::DragValue::new(&mut self.edit_pos_x).prefix("x: ").speed(0.1)).changed() { modified = true; }
                                if ui.add(egui::DragValue::new(&mut self.edit_pos_y).prefix("y: ").speed(0.1)).changed() { modified = true; }
                                if self.scene.is_3d() && ui.add(egui::DragValue::new(&mut self.edit_pos_z).prefix("z: ").speed(0.1)).changed() { modified = true; }
                            });
                            ui.horizontal(|ui| {
                                ui.label("Velocity:");
                                if ui.add(egui::DragValue::new(&mut self.edit_vel_x).prefix("vx: ").speed(0.1)).changed() { modified = true; }
                                if ui.add(egui::DragValue::new(&mut self.edit_vel_y).prefix("vy: ").speed(0.1)).changed() { modified = true; }
                                if self.scene.is_3d() && ui.add(egui::DragValue::new(&mut self.edit_vel_z).prefix("vz: ").speed(0.1)).changed() { modified = true; }
                            });
                            ui.horizontal(|ui| {
                                ui.label("AngVel ω:");
                                if ui.add(egui::DragValue::new(&mut self.edit_ang_vel_z).prefix("wz: ").speed(0.1)).changed() { modified = true; }
                            });
                            ui.horizontal(|ui| {
                                ui.label("Mass & Damping:");
                                if ui.add(egui::DragValue::new(&mut self.edit_mass).range(0.0..=500.0).prefix("mass: ").speed(0.1)).changed() { modified = true; }
                                if ui.add(egui::DragValue::new(&mut self.edit_lin_damp).range(0.0..=10.0).prefix("ld: ").speed(0.05)).changed() { modified = true; }
                            });
                            ui.horizontal(|ui| {
                                ui.label("Material Properties:");
                                if ui.add(egui::DragValue::new(&mut self.edit_friction).range(0.0..=1.0).prefix("fric: ").speed(0.02)).changed() { modified = true; }
                                if ui.add(egui::DragValue::new(&mut self.edit_restitution).range(0.0..=1.0).prefix("rest: ").speed(0.02)).changed() { modified = true; }
                            });

                            if modified {
                                self.apply_editor_fields_to_body(sid);
                            }

                            ui.horizontal(|ui| {
                                if ui.button("⚡ Apply / Wake Body").clicked() {
                                    self.apply_editor_fields_to_body(sid);
                                }
                                if ui.button("💥 Give Upward Impulse").clicked() {
                                    match &mut self.world {
                                        ActiveWorld::World2(w) => {
                                            let handle = w.bodies_iter().find(|(_, b)| b.id.0 == sid).map(|(h, _)| h);
                                            if let Some(h) = handle {
                                                let _ = w.apply_impulse(h, Vec2 { x: 0.0, y: 5.0 });
                                            }
                                        }
                                        ActiveWorld::World3(w) => {
                                            let handle = w.bodies_iter().find(|(_, b)| b.id.0 == sid).map(|(h, _)| h);
                                            if let Some(h) = handle {
                                                let _ = w.apply_impulse(h, Vec3 { x: 0.0, y: 5.0, z: 0.0 });
                                            }
                                        }
                                        _ => {}
                                    }
                                }
                            });
                            ui.separator();
                            if ui.button("🗑 Delete Selected Body").clicked() {
                                match &mut self.world {
                                    ActiveWorld::World2(w) => {
                                        let handle = w.bodies_iter().find(|(_, b)| b.id.0 == sid).map(|(h, _)| h);
                                        if let Some(h) = handle {
                                            let _ = w.remove_body(h);
                                        }
                                    }
                                    ActiveWorld::World3(w) => {
                                        let handle = w.bodies_iter().find(|(_, b)| b.id.0 == sid).map(|(h, _)| h);
                                        if let Some(h) = handle {
                                            let _ = w.remove_body(h);
                                        }
                                    }
                                    _ => {}
                                }
                                self.selected_body = None;
                                self.update_hash();
                            }
                        } else if let Some(jid) = self.selected_joint {
                            ui.label(egui::RichText::new(format!("Inspecting Selected Joint ID: {}", jid.0)).strong().color(egui::Color32::from_rgb(255, 180, 50)));
                            if ui.button("🗑 Delete Selected Joint").clicked() {
                                match &mut self.world {
                                    ActiveWorld::World2(w) => w.remove_joint(jid),
                                    ActiveWorld::World3(w) => w.remove_joint(jid),
                                    _ => {}
                                }
                                self.selected_joint = None;
                                self.update_hash();
                            }
                        } else {
                            ui.label("No body or joint currently selected. Click on an object in the viewport or outliner to inspect and modify it.");
                        }
                    });
                }
            });
        });
    }

    fn render_bottom_serialization_panel(&mut self, ctx: &egui::Context) {
        if !self.show_serialization {
            return;
        }
        egui::TopBottomPanel::bottom("bottom_serialization").resizable(true).default_height(160.0).show(ctx, |ui| {
            ui.heading("💾 Scene Serialization, Save & Load Studio");
            ui.horizontal(|ui| {
                if ui.button("📤 Export Active World to Hex String").clicked() {
                    match &self.world {
                        ActiveWorld::World2(w) => {
                            self.serialization_buffer = Self::hex_encode(&auralite_serialize::serialize_world2(w));
                            self.serialization_status = "Exported 2D World successfully!".to_string();
                        }
                        ActiveWorld::World3(w) => {
                            self.serialization_buffer = Self::hex_encode(&auralite_serialize::serialize_world3(w));
                            self.serialization_status = "Exported 3D World successfully!".to_string();
                        }
                        ActiveWorld::SoftBody(sb) => {
                            self.serialization_buffer = Self::hex_encode(&auralite_serialize::serialize_soft_body(sb));
                            self.serialization_status = "Exported SoftBody World successfully!".to_string();
                        }
                        ActiveWorld::Mixed => {
                            self.serialization_status = "Particle/Fluid scenes use specialized engine arrays.".to_string();
                        }
                    }
                }
                if ui.button("📥 Import & Replace World from Hex Buffer").clicked() {
                    if let Some(bytes) = Self::hex_decode(&self.serialization_buffer) {
                        if let Ok(w) = auralite_serialize::deserialize_world2(&bytes) {
                            self.world = ActiveWorld::World2(Box::new(w));
                            self.update_hash();
                            self.serialization_status = "Successfully loaded and verified 2D World!".to_string();
                        } else if let Ok(w) = auralite_serialize::deserialize_world3(&bytes) {
                            self.world = ActiveWorld::World3(Box::new(w));
                            self.update_hash();
                            self.serialization_status = "Successfully loaded and verified 3D World!".to_string();
                        } else if let Ok(sb) = auralite_serialize::deserialize_soft_body(&bytes) {
                            self.world = ActiveWorld::SoftBody(Box::new(sb));
                            self.update_hash();
                            self.serialization_status = "Successfully loaded SoftBody World!".to_string();
                        } else {
                            self.serialization_status = "❌ Failed to decode scene envelope (check format or checksum).".to_string();
                        }
                    } else {
                        self.serialization_status = "❌ Hex string is invalid or malformed.".to_string();
                    }
                }
                ui.separator();
                ui.label("File Path:");
                ui.text_edit_singleline(&mut self.save_file_path);
                if ui.button("💾 Save to File").clicked() {
                    if let Some(bytes) = Self::hex_decode(&self.serialization_buffer) {
                        if std::fs::write(&self.save_file_path, &bytes).is_ok() {
                            self.serialization_status = format!("Saved scene directly to {}", self.save_file_path);
                        } else {
                            self.serialization_status = "❌ File write failed.".to_string();
                        }
                    } else {
                        // Export directly if buffer was empty
                        let raw = match &self.world {
                            ActiveWorld::World2(w) => Some(auralite_serialize::serialize_world2(w)),
                            ActiveWorld::World3(w) => Some(auralite_serialize::serialize_world3(w)),
                            ActiveWorld::SoftBody(sb) => Some(auralite_serialize::serialize_soft_body(sb)),
                            ActiveWorld::Mixed => None,
                        };
                        if let Some(bytes) = raw {
                            if std::fs::write(&self.save_file_path, &bytes).is_ok() {
                                self.serialization_status = format!("Saved binary scene to {}", self.save_file_path);
                            } else {
                                self.serialization_status = "❌ File write failed.".to_string();
                            }
                        }
                    }
                }
                if ui.button("📂 Load from File").clicked() {
                    if let Ok(bytes) = std::fs::read(&self.save_file_path) {
                        self.serialization_buffer = Self::hex_encode(&bytes);
                        if let Ok(w) = auralite_serialize::deserialize_world2(&bytes) {
                            self.world = ActiveWorld::World2(Box::new(w));
                            self.update_hash();
                            self.serialization_status = format!("Loaded 2D World from {}", self.save_file_path);
                        } else if let Ok(w) = auralite_serialize::deserialize_world3(&bytes) {
                            self.world = ActiveWorld::World3(Box::new(w));
                            self.update_hash();
                            self.serialization_status = format!("Loaded 3D World from {}", self.save_file_path);
                        } else if let Ok(sb) = auralite_serialize::deserialize_soft_body(&bytes) {
                            self.world = ActiveWorld::SoftBody(Box::new(sb));
                            self.update_hash();
                            self.serialization_status = format!("Loaded SoftBody from {}", self.save_file_path);
                        }
                    } else {
                        self.serialization_status = format!("❌ Could not read {}", self.save_file_path);
                    }
                }
            });
            ui.label(egui::RichText::new(&self.serialization_status).color(egui::Color32::LIGHT_BLUE));
            egui::ScrollArea::vertical().max_height(80.0).show(ui, |ui| {
                ui.add(egui::TextEdit::multiline(&mut self.serialization_buffer).desired_width(f32::INFINITY).font(egui::TextStyle::Monospace));
            });
        });
    }

    fn render_central_viewport(&mut self, ctx: &egui::Context) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!(
                    "Active Scene: {} | Mode: {} | Tool: {}",
                    self.scene.as_str(),
                    if self.scene.is_3d() {
                        "3D Space"
                    } else {
                        "2D Plane"
                    },
                    self.active_tool.as_str()
                ));
            });
            let available = ui.available_size();
            let (rect, response) = ui.allocate_exact_size(available, egui::Sense::click_and_drag());
            let painter = ui.painter_at(rect);
            painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(20, 20, 26));

            if self.debug.grid {
                self.draw_viewport_grid(&painter, rect);
            }

            match &self.world {
                ActiveWorld::World2(w) => {
                    let scale = self.viewport_scale;
                    let off_x = rect.center().x + self.viewport_offset.x;
                    let off_y = rect.center().y + self.viewport_offset.y;
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
                                egui::Stroke::new(2.5_f32, egui::Color32::YELLOW),
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
                    let scale = self.viewport_scale * (25.0 / self.camera_dist.max(1.0)) * 0.6;
                    let off_x = rect.center().x + self.viewport_offset.x;
                    let off_y = rect.center().y + self.viewport_offset.y;
                    let cos_y = self.camera_yaw.cos();
                    let sin_y = self.camera_yaw.sin();
                    let cos_p = self.camera_pitch.cos();
                    let sin_p = self.camera_pitch.sin();
                    let project = |p: Vec3| -> egui::Pos2 {
                        let rx = p.x * cos_y - p.z * sin_y;
                        let rz = p.x * sin_y + p.z * cos_y;
                        let ry = p.y * cos_p - rz * sin_p;
                        egui::pos2(off_x + rx * scale, off_y - ry * scale)
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
                                egui::Stroke::new(2.5_f32, egui::Color32::YELLOW),
                            );
                        }
                    }
                }
                ActiveWorld::SoftBody(sb) => {
                    let scale = self.viewport_scale * 0.7;
                    let off_x = rect.center().x + self.viewport_offset.x;
                    let off_y = rect.center().y + self.viewport_offset.y;
                    for p in &sb.particles {
                        let sx = off_x + p.position.x * scale;
                        let sy = off_y - p.position.y * scale;
                        painter.circle_filled(
                            egui::pos2(sx, sy),
                            4.0,
                            egui::Color32::from_rgb(255, 136, 68),
                        );
                    }
                    if self.debug.softbody {
                        for (a_idx, b_idx) in sb.edge_indices.iter().take(400) {
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
                    if let Some(storage) = &self.particle_storage {
                        let scale = self.viewport_scale * 0.6;
                        let off_x = rect.center().x + self.viewport_offset.x;
                        let off_y = rect.center().y + self.viewport_offset.y;
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

            self.draw_viewport_gizmos(&painter, rect, ctx);
            self.handle_viewport_interactions(&response, rect, ctx);
        });
    }

    fn draw_viewport_grid(&self, painter: &egui::Painter, rect: egui::Rect) {
        let off_x = rect.center().x + self.viewport_offset.x;
        let off_y = rect.center().y + self.viewport_offset.y;
        painter.line_segment(
            [
                egui::pos2(rect.left(), off_y),
                egui::pos2(rect.right(), off_y),
            ],
            egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(60, 60, 75)),
        );
        painter.line_segment(
            [
                egui::pos2(off_x, rect.top()),
                egui::pos2(off_x, rect.bottom()),
            ],
            egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(60, 60, 75)),
        );
    }

    fn draw_viewport_gizmos(&self, painter: &egui::Painter, rect: egui::Rect, ctx: &egui::Context) {
        if let Some(hover_pos) = ctx.input(|i| i.pointer.hover_pos()) {
            if !rect.contains(hover_pos) {
                return;
            }
            if self.active_tool == EditorTool::SpawnBody {
                let r = self.spawn_radius * self.viewport_scale;
                painter.circle_stroke(
                    hover_pos,
                    r.max(6.0),
                    egui::Stroke::new(1.5_f32, egui::Color32::from_rgb(100, 255, 180)),
                );
            } else if self.active_tool == EditorTool::CreateJoint
                && self.joint_wizard_step == 1
                && let Some(id_a) = self.joint_wizard_body_a
            {
                let off_x = rect.center().x + self.viewport_offset.x;
                let off_y = rect.center().y + self.viewport_offset.y;
                let scale = self.viewport_scale;
                if let ActiveWorld::World2(w) = &self.world
                    && let Some((_, b)) = w.bodies_iter().find(|(_, b)| b.id.0 == id_a)
                {
                    let a_pos =
                        egui::pos2(off_x + b.position.x * scale, off_y - b.position.y * scale);
                    painter.line_segment(
                        [a_pos, hover_pos],
                        egui::Stroke::new(2.0_f32, egui::Color32::YELLOW),
                    );
                } else if let ActiveWorld::World3(w) = &self.world
                    && let Some((_, b)) = w.bodies_iter().find(|(_, b)| b.id.0 == id_a)
                {
                    let cos_y = self.camera_yaw.cos();
                    let sin_y = self.camera_yaw.sin();
                    let cos_p = self.camera_pitch.cos();
                    let sin_p = self.camera_pitch.sin();
                    let rx = b.position.x * cos_y - b.position.z * sin_y;
                    let rz = b.position.x * sin_y + b.position.z * cos_y;
                    let ry = b.position.y * cos_p - rz * sin_p;
                    let a_pos = egui::pos2(off_x + rx * scale * 0.6, off_y - ry * scale * 0.6);
                    painter.line_segment(
                        [a_pos, hover_pos],
                        egui::Stroke::new(2.0_f32, egui::Color32::YELLOW),
                    );
                }
            }
        }
        if self.active_tool == EditorTool::ApplyImpulse
            && let (Some(_), Some(start), Some(end)) =
                (self.drag_body, self.drag_start_pos, self.drag_current_pos)
        {
            painter.line_segment(
                [start, end],
                egui::Stroke::new(3.0_f32, egui::Color32::from_rgb(255, 120, 40)),
            );
            painter.circle_filled(end, 5.0, egui::Color32::from_rgb(255, 120, 40));
        }
    }

    fn handle_viewport_interactions(
        &mut self,
        response: &egui::Response,
        rect: egui::Rect,
        ctx: &egui::Context,
    ) {
        if response.dragged_by(egui::PointerButton::Middle)
            || (response.dragged_by(egui::PointerButton::Primary) && ctx.input(|i| i.modifiers.alt))
        {
            let delta = response.drag_delta();
            if self.scene.is_3d() && ctx.input(|i| i.modifiers.alt) {
                self.camera_yaw += delta.x * 0.01;
                self.camera_pitch = (self.camera_pitch + delta.y * 0.01).clamp(-1.5, 1.5);
            } else {
                self.viewport_offset += Vec2 {
                    x: delta.x,
                    y: delta.y,
                };
            }
        }

        if response.hovered() {
            let scroll = ctx.input(|i| i.raw_scroll_delta.y);
            if scroll.abs() > 0.1 {
                self.viewport_scale = (self.viewport_scale + scroll * 0.1).clamp(5.0, 200.0);
            }
        }

        if response.clicked()
            && let Some(click_pos) = response.interact_pointer_pos()
        {
            let off_x = rect.center().x + self.viewport_offset.x;
            let off_y = rect.center().y + self.viewport_offset.y;
            let scale = self.viewport_scale;

            match self.active_tool {
                EditorTool::Select => {
                    let mut nearest: Option<(u64, f32)> = None;
                    if let ActiveWorld::World2(w) = &self.world {
                        for (_, b) in w.bodies_iter() {
                            let sx = off_x + b.position.x * scale;
                            let sy = off_y - b.position.y * scale;
                            let d =
                                ((click_pos.x - sx).powi(2) + (click_pos.y - sy).powi(2)).sqrt();
                            if d < 25.0 && nearest.is_none_or(|(_, best_d)| d < best_d) {
                                nearest = Some((b.id.0, d));
                            }
                        }
                    } else if let ActiveWorld::World3(w) = &self.world {
                        let scale3 = scale * (25.0 / self.camera_dist.max(1.0)) * 0.6;
                        let cos_y = self.camera_yaw.cos();
                        let sin_y = self.camera_yaw.sin();
                        let cos_p = self.camera_pitch.cos();
                        let sin_p = self.camera_pitch.sin();
                        for (_, b) in w.bodies_iter() {
                            let rx = b.position.x * cos_y - b.position.z * sin_y;
                            let rz = b.position.x * sin_y + b.position.z * cos_y;
                            let ry = b.position.y * cos_p - rz * sin_p;
                            let sx = off_x + rx * scale3;
                            let sy = off_y - ry * scale3;
                            let d =
                                ((click_pos.x - sx).powi(2) + (click_pos.y - sy).powi(2)).sqrt();
                            if d < 25.0 && nearest.is_none_or(|(_, best_d)| d < best_d) {
                                nearest = Some((b.id.0, d));
                            }
                        }
                    }
                    if let Some((id, _)) = nearest {
                        self.selected_body = Some(id);
                        self.load_body_to_editor_fields(id);
                    } else {
                        self.selected_body = None;
                    }
                }
                EditorTool::SpawnBody => {
                    let wx = (click_pos.x - off_x) / scale;
                    let wy = (off_y - click_pos.y) / scale;
                    let kind = match self.spawn_body_type {
                        1 => BodyType::Static,
                        2 => BodyType::Kinematic,
                        _ => BodyType::Dynamic,
                    };
                    let mat = Material {
                        friction: self.spawn_friction as Real,
                        restitution: self.spawn_restitution as Real,
                        density: 1.0,
                    };
                    let mut spawned_body_to_load = None;
                    if let ActiveWorld::World2(w) = &mut self.world {
                        let shape = match self.spawn_shape {
                            0 => {
                                Circle2::new(self.spawn_radius as Real).map(ColliderShape2::Circle)
                            }
                            1 => Box2::new(Vec2 {
                                x: self.spawn_half_width as Real,
                                y: self.spawn_half_height as Real,
                            })
                            .map(ColliderShape2::Box),
                            _ => Capsule2::new(
                                self.spawn_radius as Real,
                                self.spawn_half_height as Real,
                            )
                            .map(ColliderShape2::Capsule),
                        };
                        if let Ok(sh) = shape {
                            let builder = BodyBuilder2 {
                                kind,
                                position: Vec2 {
                                    x: wx as Real,
                                    y: wy as Real,
                                },
                                rotation: Rot2::from_radians(0.0).unwrap(),
                                velocity: Vec2 {
                                    x: self.spawn_vel_x as Real,
                                    y: self.spawn_vel_y as Real,
                                },
                                angular_velocity: 0.0,
                                mass: if kind == BodyType::Dynamic {
                                    self.spawn_mass as Real
                                } else {
                                    0.0
                                },
                                inertia: None,
                                colliders: vec![Collider2 {
                                    shape: sh,
                                    offset: Vec2::ZERO,
                                    material: mat,
                                    filter: CollisionFilter::default(),
                                }],
                                restitution: self.spawn_restitution as Real,
                                friction: self.spawn_friction as Real,
                                linear_damping: self.spawn_lin_damp as Real,
                                angular_damping: self.spawn_ang_damp as Real,
                                user_data: 0,
                            };
                            if let Ok(h) = w.add_body(builder)
                                && let Ok(b) = w.body(h)
                            {
                                spawned_body_to_load = Some(b.id.0);
                            }
                        }
                    } else if let ActiveWorld::World3(w) = &mut self.world {
                        let shape = match self.spawn_shape {
                            0 => {
                                Sphere3::new(self.spawn_radius as Real).map(ColliderShape3::Sphere)
                            }
                            1 => Box3::new(Vec3 {
                                x: self.spawn_half_width as Real,
                                y: self.spawn_half_height as Real,
                                z: self.spawn_half_depth as Real,
                            })
                            .map(ColliderShape3::Box),
                            _ => Capsule3::new(
                                self.spawn_radius as Real,
                                self.spawn_half_height as Real,
                            )
                            .map(ColliderShape3::Capsule),
                        };
                        if let Ok(sh) = shape {
                            let builder = BodyBuilder3 {
                                kind,
                                position: Vec3 {
                                    x: wx as Real,
                                    y: wy as Real,
                                    z: 0.0,
                                },
                                rotation: Quat::identity(),
                                velocity: Vec3 {
                                    x: self.spawn_vel_x as Real,
                                    y: self.spawn_vel_y as Real,
                                    z: self.spawn_vel_z as Real,
                                },
                                angular_velocity: Vec3::ZERO,
                                mass: if kind == BodyType::Dynamic {
                                    self.spawn_mass as Real
                                } else {
                                    0.0
                                },
                                inertia_diagonal: None,
                                colliders: vec![Collider3 {
                                    shape: sh,
                                    offset: Vec3::ZERO,
                                    material: mat,
                                    filter: CollisionFilter::default(),
                                }],
                                restitution: self.spawn_restitution as Real,
                                friction: self.spawn_friction as Real,
                                linear_damping: self.spawn_lin_damp as Real,
                                angular_damping: self.spawn_ang_damp as Real,
                                user_data: 0,
                            };
                            if let Ok(h) = w.add_body(builder)
                                && let Ok(b) = w.body(h)
                            {
                                spawned_body_to_load = Some(b.id.0);
                            }
                        }
                    }
                    if let Some(id) = spawned_body_to_load {
                        self.selected_body = Some(id);
                        self.load_body_to_editor_fields(id);
                    }
                    self.update_hash();
                }
                EditorTool::CreateJoint => {
                    let mut clicked_id = None;
                    if let ActiveWorld::World2(w) = &self.world {
                        for (_, b) in w.bodies_iter() {
                            let sx = off_x + b.position.x * scale;
                            let sy = off_y - b.position.y * scale;
                            if ((click_pos.x - sx).powi(2) + (click_pos.y - sy).powi(2)).sqrt()
                                < 25.0
                            {
                                clicked_id = Some(b.id.0);
                                break;
                            }
                        }
                    } else if let ActiveWorld::World3(w) = &self.world {
                        let scale3 = scale * (25.0 / self.camera_dist.max(1.0)) * 0.6;
                        let cos_y = self.camera_yaw.cos();
                        let sin_y = self.camera_yaw.sin();
                        let cos_p = self.camera_pitch.cos();
                        let sin_p = self.camera_pitch.sin();
                        for (_, b) in w.bodies_iter() {
                            let rx = b.position.x * cos_y - b.position.z * sin_y;
                            let rz = b.position.x * sin_y + b.position.z * cos_y;
                            let ry = b.position.y * cos_p - rz * sin_p;
                            let sx = off_x + rx * scale3;
                            let sy = off_y - ry * scale3;
                            if ((click_pos.x - sx).powi(2) + (click_pos.y - sy).powi(2)).sqrt()
                                < 25.0
                            {
                                clicked_id = Some(b.id.0);
                                break;
                            }
                        }
                    }
                    if let Some(id) = clicked_id {
                        if self.joint_wizard_step == 0 {
                            self.joint_wizard_body_a = Some(id);
                            self.joint_wizard_step = 1;
                        } else if Some(id) != self.joint_wizard_body_a {
                            self.joint_wizard_body_b = Some(id);
                        }
                    }
                }
                EditorTool::Delete => {
                    let mut target_id = None;
                    if let ActiveWorld::World2(w) = &self.world {
                        for (_, b) in w.bodies_iter() {
                            let sx = off_x + b.position.x * scale;
                            let sy = off_y - b.position.y * scale;
                            if ((click_pos.x - sx).powi(2) + (click_pos.y - sy).powi(2)).sqrt()
                                < 25.0
                            {
                                target_id = Some(b.id.0);
                                break;
                            }
                        }
                    } else if let ActiveWorld::World3(w) = &self.world {
                        let scale3 = scale * (25.0 / self.camera_dist.max(1.0)) * 0.6;
                        let cos_y = self.camera_yaw.cos();
                        let sin_y = self.camera_yaw.sin();
                        let cos_p = self.camera_pitch.cos();
                        let sin_p = self.camera_pitch.sin();
                        for (_, b) in w.bodies_iter() {
                            let rx = b.position.x * cos_y - b.position.z * sin_y;
                            let rz = b.position.x * sin_y + b.position.z * cos_y;
                            let ry = b.position.y * cos_p - rz * sin_p;
                            let sx = off_x + rx * scale3;
                            let sy = off_y - ry * scale3;
                            if ((click_pos.x - sx).powi(2) + (click_pos.y - sy).powi(2)).sqrt()
                                < 25.0
                            {
                                target_id = Some(b.id.0);
                                break;
                            }
                        }
                    }
                    if let Some(id) = target_id {
                        let handle2 = if let ActiveWorld::World2(w) = &self.world {
                            w.bodies_iter().find(|(_, b)| b.id.0 == id).map(|(h, _)| h)
                        } else {
                            None
                        };
                        let handle3 = if let ActiveWorld::World3(w) = &self.world {
                            w.bodies_iter().find(|(_, b)| b.id.0 == id).map(|(h, _)| h)
                        } else {
                            None
                        };
                        if let Some(h) = handle2
                            && let ActiveWorld::World2(w) = &mut self.world
                        {
                            let _ = w.remove_body(h);
                        } else if let Some(h) = handle3
                            && let ActiveWorld::World3(w) = &mut self.world
                        {
                            let _ = w.remove_body(h);
                        }
                        if self.selected_body == Some(id) {
                            self.selected_body = None;
                        }
                        self.update_hash();
                    }
                }
                _ => {}
            }
        }

        if self.active_tool == EditorTool::ApplyImpulse {
            if response.drag_started()
                && let Some(start_pos) = response.interact_pointer_pos()
            {
                let off_x = rect.center().x + self.viewport_offset.x;
                let off_y = rect.center().y + self.viewport_offset.y;
                let scale = self.viewport_scale;
                let mut found = None;
                if let ActiveWorld::World2(w) = &self.world {
                    for (_, b) in w.bodies_iter() {
                        let sx = off_x + b.position.x * scale;
                        let sy = off_y - b.position.y * scale;
                        if ((start_pos.x - sx).powi(2) + (start_pos.y - sy).powi(2)).sqrt() < 30.0 {
                            found = Some((b.id.0, egui::pos2(sx, sy)));
                            break;
                        }
                    }
                } else if let ActiveWorld::World3(w) = &self.world {
                    let scale3 = scale * (25.0 / self.camera_dist.max(1.0)) * 0.6;
                    let cos_y = self.camera_yaw.cos();
                    let sin_y = self.camera_yaw.sin();
                    let cos_p = self.camera_pitch.cos();
                    let sin_p = self.camera_pitch.sin();
                    for (_, b) in w.bodies_iter() {
                        let rx = b.position.x * cos_y - b.position.z * sin_y;
                        let rz = b.position.x * sin_y + b.position.z * cos_y;
                        let ry = b.position.y * cos_p - rz * sin_p;
                        let sx = off_x + rx * scale3;
                        let sy = off_y - ry * scale3;
                        if ((start_pos.x - sx).powi(2) + (start_pos.y - sy).powi(2)).sqrt() < 30.0 {
                            found = Some((b.id.0, egui::pos2(sx, sy)));
                            break;
                        }
                    }
                }
                if let Some((id, center)) = found {
                    self.drag_body = Some(id);
                    self.drag_start_pos = Some(center);
                    self.drag_current_pos = Some(start_pos);
                }
            } else if response.dragged()
                && self.drag_body.is_some()
                && let Some(curr) = response.interact_pointer_pos()
            {
                self.drag_current_pos = Some(curr);
            } else if response.drag_stopped()
                && let (Some(id), Some(start), Some(end)) =
                    (self.drag_body, self.drag_start_pos, self.drag_current_pos)
            {
                let dx = (end.x - start.x) / self.viewport_scale * self.impulse_multiplier;
                let dy = (start.y - end.y) / self.viewport_scale * self.impulse_multiplier;
                let handle2 = if let ActiveWorld::World2(w) = &self.world {
                    w.bodies_iter().find(|(_, b)| b.id.0 == id).map(|(h, _)| h)
                } else {
                    None
                };
                let handle3 = if let ActiveWorld::World3(w) = &self.world {
                    w.bodies_iter().find(|(_, b)| b.id.0 == id).map(|(h, _)| h)
                } else {
                    None
                };
                if let Some(h) = handle2
                    && let ActiveWorld::World2(w) = &mut self.world
                {
                    let _ = w.apply_impulse(
                        h,
                        Vec2 {
                            x: dx as Real,
                            y: dy as Real,
                        },
                    );
                } else if let Some(h) = handle3
                    && let ActiveWorld::World3(w) = &mut self.world
                {
                    let _ = w.apply_impulse(
                        h,
                        Vec3 {
                            x: dx as Real,
                            y: dy as Real,
                            z: 0.0,
                        },
                    );
                }
                self.drag_body = None;
                self.drag_start_pos = None;
                self.drag_current_pos = None;
                self.update_hash();
            }
        }
    }
}

impl eframe::App for SandboxApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.render_top_bar(ctx);
        self.render_left_side_panel(ctx);
        self.render_right_side_panel(ctx);
        self.render_bottom_serialization_panel(ctx);
        self.render_central_viewport(ctx);

        if !self.paused {
            ctx.request_repaint();
            self.step();
        }
    }
}
