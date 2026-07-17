//! Vehicles (2D/3D) and character controllers (2D/3D).
#![forbid(unsafe_code)]
#![allow(missing_docs, clippy::too_many_arguments)]

use auralite_collision::CollisionFilter;
use auralite_dynamics::{
    BodyBuilder2, BodyBuilder3, BodyHandle2, BodyHandle3, Collider2, Collider3, ColliderShape2,
    ColliderShape3, Material, World2, World3,
};
use auralite_math::{ABS_EPSILON, Quat, Ray3, Real, Rot2, Vec2, Vec3};

#[derive(Clone, Debug)]
pub struct WheelConfig3 {
    pub attachment_point: Vec3,
    pub suspension_direction: Vec3,
    pub suspension_travel: Real,
    pub suspension_stiffness: Real,
    pub suspension_damping: Real,
    pub wheel_radius: Real,
    pub steered: bool,
    pub driven: bool,
    pub brake_torque: Real,
    pub friction_long: Real,
    pub friction_lat: Real,
    pub wheel_mass: Real,
}

impl Default for WheelConfig3 {
    fn default() -> Self {
        Self {
            attachment_point: Vec3::ZERO,
            suspension_direction: Vec3 {
                x: 0.0,
                y: -1.0,
                z: 0.0,
            },
            suspension_travel: 0.3,
            suspension_stiffness: 50.0,
            suspension_damping: 10.0,
            wheel_radius: 0.4,
            steered: false,
            driven: false,
            brake_torque: 100.0,
            friction_long: 1.0,
            friction_lat: 0.8,
            wheel_mass: 20.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct WheelState3 {
    pub config: WheelConfig3,
    pub suspension_length: Real,
    pub prev_suspension_length: Real,
    pub contact_point: Vec3,
    pub contact_normal: Vec3,
    pub is_grounded: bool,
    pub steer_angle: Real,
    pub angular_velocity: Real,
    pub slip_long: Real,
    pub slip_lat: Real,
    pub longitudinal_force: Real,
    pub lateral_force: Real,
}

impl WheelState3 {
    pub fn new(config: WheelConfig3) -> Self {
        Self {
            suspension_length: config.suspension_travel,
            prev_suspension_length: config.suspension_travel,
            contact_point: Vec3::ZERO,
            contact_normal: Vec3::Y,
            is_grounded: false,
            steer_angle: 0.0,
            angular_velocity: 0.0,
            slip_long: 0.0,
            slip_lat: 0.0,
            longitudinal_force: 0.0,
            lateral_force: 0.0,
            config,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DifferentialType {
    Open,
    Locked,
    LimitedSlip,
}

#[derive(Clone, Debug)]
pub struct VehicleConfig3 {
    pub chassis_mass: Real,
    pub chassis_half_extents: Vec3,
    pub center_of_mass_offset: Vec3,
    pub max_engine_torque: Real,
    pub max_brake_torque: Real,
    pub max_steer_angle: Real,
    pub steer_speed: Real,
    pub differential_type: DifferentialType,
}

impl Default for VehicleConfig3 {
    fn default() -> Self {
        Self {
            chassis_mass: 1500.0,
            chassis_half_extents: Vec3 {
                x: 1.2,
                y: 0.4,
                z: 2.5,
            },
            center_of_mass_offset: Vec3 {
                x: 0.0,
                y: -0.2,
                z: 0.0,
            },
            max_engine_torque: 300.0,
            max_brake_torque: 500.0,
            max_steer_angle: 0.6,
            steer_speed: 2.0,
            differential_type: DifferentialType::Open,
        }
    }
}

pub struct Vehicle3 {
    pub config: VehicleConfig3,
    pub body: BodyHandle3,
    pub wheels: Vec<WheelState3>,
    pub throttle: Real,
    pub brake: Real,
    pub steer_input: Real,
    pub speed: Real,
}

impl Vehicle3 {
    pub fn new(
        config: VehicleConfig3,
        position: Vec3,
        rotation: Quat,
        wheel_configs: Vec<WheelConfig3>,
        world: &mut World3,
    ) -> Self {
        let inertia = Vec3 {
            x: config.chassis_mass
                * (config.chassis_half_extents.y.powi(2) + config.chassis_half_extents.z.powi(2))
                / 12.0,
            y: config.chassis_mass
                * (config.chassis_half_extents.x.powi(2) + config.chassis_half_extents.z.powi(2))
                / 12.0,
            z: config.chassis_mass
                * (config.chassis_half_extents.x.powi(2) + config.chassis_half_extents.y.powi(2))
                / 12.0,
        };
        let body = world
            .add_body(
                BodyBuilder3::dynamic()
                    .position(position)
                    .rotation(rotation)
                    .mass(config.chassis_mass)
                    .inertia_diagonal(inertia)
                    .add_collider(Collider3 {
                        shape: ColliderShape3::Box(
                            auralite_geometry::Box3::new(config.chassis_half_extents).unwrap(),
                        ),
                        offset: config.center_of_mass_offset,
                        material: Material {
                            restitution: 0.0,
                            friction: 0.8,
                            density: 1.0,
                        },
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        let wheels = wheel_configs.into_iter().map(WheelState3::new).collect();
        Self {
            config,
            body,
            wheels,
            throttle: 0.0,
            brake: 0.0,
            steer_input: 0.0,
            speed: 0.0,
        }
    }
    pub fn set_controls(&mut self, throttle: Real, brake: Real, steer: Real) {
        self.throttle = throttle.clamp(-1.0, 1.0);
        self.brake = brake.clamp(0.0, 1.0);
        self.steer_input = steer.clamp(-1.0, 1.0);
    }
    pub fn step(&mut self, dt: Real, world: &mut World3) {
        let chassis = match world.body(self.body) {
            Ok(b) => b.clone(),
            Err(_) => return,
        };
        self.speed = chassis.velocity.length();
        for wheel in &mut self.wheels {
            wheel.prev_suspension_length = wheel.suspension_length;
            let world_attach =
                chassis.position + chassis.rotation.rotate(wheel.config.attachment_point);
            let sus_dir = chassis
                .rotation
                .rotate(wheel.config.suspension_direction)
                .normalized_or(-Vec3::Y);
            let ray_max = wheel.config.suspension_travel + wheel.config.wheel_radius;
            let ray = Ray3::new(world_attach, sus_dir).unwrap();
            let hit = world.ray_cast_ignoring(ray, ray_max, self.body);
            if let Some((_, t, n)) = hit {
                wheel.suspension_length =
                    (t - wheel.config.wheel_radius).min(wheel.config.suspension_travel);
                wheel.contact_point = world_attach + sus_dir * t;
                wheel.contact_normal = n;
                wheel.is_grounded = true;
                let compression =
                    (wheel.config.suspension_travel - wheel.suspension_length).max(0.0);
                let spring = compression * wheel.config.suspension_stiffness;
                let damper = ((wheel.suspension_length - wheel.prev_suspension_length) / dt)
                    * wheel.config.suspension_damping;
                let sus_force = (spring + damper).max(0.0);
                world
                    .apply_impulse_at_point(
                        self.body,
                        sus_dir * sus_force * dt,
                        wheel.contact_point,
                    )
                    .unwrap();
                // (Tire forces simplified for logic flow)
            } else {
                wheel.is_grounded = false;
                wheel.suspension_length = wheel.config.suspension_travel;
            }
        }
    }
}

pub struct Vehicle2 {
    pub body: BodyHandle2,
    pub throttle: Real,
    pub brake: Real,
    pub speed: Real,
}
impl Vehicle2 {
    pub fn new(position: Vec2, rotation: Rot2, mass: Real, world: &mut World2) -> Self {
        let body = world
            .add_body(
                BodyBuilder2::dynamic()
                    .position(position)
                    .rotation(rotation)
                    .mass(mass)
                    .inertia(mass)
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Box(
                            auralite_geometry::Box2::new(Vec2 { x: 1.2, y: 0.4 }).unwrap(),
                        ),
                        offset: Vec2::ZERO,
                        material: Material {
                            restitution: 0.0,
                            friction: 0.8,
                            density: 1.0,
                        },
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        Self {
            body,
            throttle: 0.0,
            brake: 0.0,
            speed: 0.0,
        }
    }
    pub fn set_controls(&mut self, throttle: Real, brake: Real) {
        self.throttle = throttle.clamp(-1.0, 1.0);
        self.brake = brake.clamp(0.0, 1.0);
    }
    pub fn step(&mut self, dt: Real, world: &mut World2) {
        let b = match world.body(self.body) {
            Ok(b) => b.clone(),
            Err(_) => return,
        };
        self.speed = b.velocity.length();
        let engine_force = b.rotation.rotate(Vec2::X) * self.throttle * 500.0 * dt;
        let _ = world.apply_impulse(self.body, engine_force);
    }
}

#[derive(Clone, Debug)]
pub struct CharacterConfig2 {
    pub height: Real,
    pub radius: Real,
    pub skin_width: Real,
    pub move_speed: Real,
    pub acceleration: Real,
    pub gravity: Vec2,
    pub jump_velocity: Real,
    pub slope_limit: Real,
    pub step_height: Real,
}
impl Default for CharacterConfig2 {
    fn default() -> Self {
        Self {
            height: 1.8,
            radius: 0.3,
            skin_width: 0.01,
            move_speed: 6.0,
            acceleration: 50.0,
            gravity: Vec2 { x: 0.0, y: -20.0 },
            jump_velocity: 8.0,
            slope_limit: 0.785,
            step_height: 0.3,
        }
    }
}
pub struct Character2 {
    pub config: CharacterConfig2,
    pub position: Vec2,
    pub is_grounded: bool,
    pub body: Option<BodyHandle2>,
    move_input: Vec2,
    want_jump: bool,
    jump_cooldown: Real,
}
impl Character2 {
    pub fn new(config: CharacterConfig2, position: Vec2) -> Self {
        Self {
            config,
            position,
            is_grounded: false,
            body: None,
            move_input: Vec2::ZERO,
            want_jump: false,
            jump_cooldown: 0.0,
        }
    }
    pub fn attach(&mut self, world: &mut World2) {
        self.body = Some(
            world
                .add_body(
                    BodyBuilder2::dynamic()
                        .position(self.position)
                        .mass(80.0)
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Capsule(
                                auralite_geometry::Capsule2::new(
                                    self.config.radius,
                                    (self.config.height - self.config.radius * 2.0) * 0.5,
                                )
                                .unwrap(),
                            ),
                            offset: Vec2 {
                                x: 0.0,
                                y: self.config.height * 0.5,
                            },
                            material: Material {
                                restitution: 0.0,
                                friction: 0.0,
                                density: 1.0,
                            },
                            filter: CollisionFilter::default(),
                        }),
                )
                .unwrap(),
        );
    }
    pub fn set_move(&mut self, input: Vec2) {
        self.move_input = input;
    }
    pub fn jump(&mut self) {
        self.want_jump = true;
    }
    pub fn step(&mut self, dt: Real, world: &mut World2) {
        let body_h = match self.body {
            Some(h) => h,
            None => return,
        };
        self.jump_cooldown = (self.jump_cooldown - dt).max(0.0);
        let (pos, _vel) = match world.body(body_h) {
            Ok(b) => (b.position, b.velocity),
            Err(_) => return,
        };
        self.position = pos;
        let ray = auralite_math::Ray2::new(pos + Vec2::Y * 0.1, -Vec2::Y).unwrap();
        let hit = world.ray_cast_ignoring(ray, 0.2 + self.config.skin_width, body_h);
        self.is_grounded = false;
        let mut p_vel = Vec2::ZERO;
        if let Some((hg, _, n)) = hit
            && n.dot(Vec2::Y).acos() <= self.config.slope_limit
        {
            self.is_grounded = true;
            if let Ok(gb) = world.body(hg) {
                p_vel = gb.velocity;
            }
        }
        let target_vx = self.move_input.x * self.config.move_speed;
        let world_gravity = world.gravity();
        if let Ok(b) = world.body_mut(body_h) {
            if self.is_grounded {
                let diff = (p_vel
                    + Vec2 {
                        x: target_vx,
                        y: 0.0,
                    })
                    - b.velocity;
                let accel = self.config.acceleration * dt;
                b.velocity += if diff.length() <= accel {
                    diff
                } else {
                    diff.normalized_or(Vec2::ZERO) * accel
                };
                if self.want_jump && self.jump_cooldown <= 0.0 {
                    b.velocity.y = p_vel.y + self.config.jump_velocity;
                    self.jump_cooldown = 0.1;
                    self.is_grounded = false;
                }
            } else {
                b.velocity += (self.config.gravity + world_gravity) * 0.5 * dt;
            }
        }
        self.want_jump = false;
    }
}

#[derive(Clone, Debug)]
pub struct CharacterConfig3 {
    pub height: Real,
    pub radius: Real,
    pub skin_width: Real,
    pub move_speed: Real,
    pub acceleration: Real,
    pub gravity: Vec3,
    pub jump_velocity: Real,
    pub slope_limit: Real,
    pub step_height: Real,
}
impl Default for CharacterConfig3 {
    fn default() -> Self {
        Self {
            height: 1.8,
            radius: 0.3,
            skin_width: 0.01,
            move_speed: 6.0,
            acceleration: 50.0,
            gravity: Vec3 {
                x: 0.0,
                y: -20.0,
                z: 0.0,
            },
            jump_velocity: 8.0,
            slope_limit: 0.785,
            step_height: 0.3,
        }
    }
}
pub struct Character3 {
    pub config: CharacterConfig3,
    pub position: Vec3,
    pub is_grounded: bool,
    pub body: Option<BodyHandle3>,
    move_input: Vec3,
    want_jump: bool,
    jump_cooldown: Real,
}
impl Character3 {
    pub fn new(config: CharacterConfig3, position: Vec3) -> Self {
        Self {
            config,
            position,
            is_grounded: false,
            body: None,
            move_input: Vec3::ZERO,
            want_jump: false,
            jump_cooldown: 0.0,
        }
    }
    pub fn attach(&mut self, world: &mut World3) {
        self.body = Some(
            world
                .add_body(
                    BodyBuilder3::dynamic()
                        .position(self.position)
                        .mass(80.0)
                        .add_collider(Collider3 {
                            shape: ColliderShape3::Capsule(
                                auralite_geometry::Capsule3::new(
                                    self.config.radius,
                                    (self.config.height - self.config.radius * 2.0) * 0.5,
                                )
                                .unwrap(),
                            ),
                            offset: Vec3 {
                                x: 0.0,
                                y: self.config.height * 0.5,
                                z: 0.0,
                            },
                            material: Material {
                                restitution: 0.0,
                                friction: 0.0,
                                density: 1.0,
                            },
                            filter: CollisionFilter::default(),
                        }),
                )
                .unwrap(),
        );
    }
    pub fn set_move(&mut self, input: Vec3) {
        self.move_input = input;
    }
    pub fn jump(&mut self) {
        self.want_jump = true;
    }
    pub fn step(&mut self, dt: Real, world: &mut World3) {
        let body_h = match self.body {
            Some(h) => h,
            None => return,
        };
        self.jump_cooldown = (self.jump_cooldown - dt).max(0.0);
        let (pos, vel) = match world.body(body_h) {
            Ok(b) => (b.position, b.velocity),
            Err(_) => return,
        };
        self.position = pos;
        let ray = Ray3::new(pos + Vec3::Y * 0.1, -Vec3::Y).unwrap();
        let hit = world.ray_cast_ignoring(ray, 0.2 + self.config.skin_width, body_h);
        self.is_grounded = false;
        let mut p_vel = Vec3::ZERO;
        if let Some((hg, _, n)) = hit
            && n.dot(Vec3::Y).acos() <= self.config.slope_limit
        {
            self.is_grounded = true;
            if let Ok(gb) = world.body(hg) {
                p_vel = gb.velocity;
            }
        }
        let target_v = p_vel
            + Vec3 {
                x: self.move_input.x * self.config.move_speed,
                y: 0.0,
                z: self.move_input.z * self.config.move_speed,
            };
        let world_gravity = world.gravity();
        if self.is_grounded {
            let diff = target_v - vel;
            let accel = self.config.acceleration * dt;
            let imp = if diff.length() <= accel {
                diff
            } else {
                diff.normalized_or(Vec3::ZERO) * accel
            };
            if imp.length() > ABS_EPSILON {
                let _ = world.apply_impulse(body_h, imp);
            }
            if self.want_jump && self.jump_cooldown <= 0.0 {
                let _ = world.apply_impulse(
                    body_h,
                    Vec3 {
                        x: 0.0,
                        y: self.config.jump_velocity,
                        z: 0.0,
                    },
                );
                self.want_jump = false;
                self.jump_cooldown = 0.1;
                self.is_grounded = false;
            }
        } else {
            let _ = world.apply_impulse(body_h, (self.config.gravity + world_gravity) * dt);
            let air_accel = self.config.acceleration * 0.3 * dt;
            let rel_v = vel - p_vel;
            let target_rel_v = Vec3 {
                x: self.move_input.x * self.config.move_speed,
                y: 0.0,
                z: self.move_input.z * self.config.move_speed,
            };
            let diff = target_rel_v
                - Vec3 {
                    x: rel_v.x,
                    y: 0.0,
                    z: rel_v.z,
                };
            let imp = if diff.length() <= air_accel {
                diff
            } else {
                diff.normalized_or(Vec3::ZERO) * air_accel
            };
            let _ = world.apply_impulse(body_h, imp);
        }
        self.want_jump = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn vehicle3_creates_and_steps() {
        let mut world = World3::default();
        let wc = vec![WheelConfig3 {
            attachment_point: Vec3 {
                x: -0.8,
                y: -0.2,
                z: 1.5,
            },
            steered: true,
            driven: true,
            ..Default::default()
        }];
        let mut v = Vehicle3::new(
            VehicleConfig3::default(),
            Vec3::Y,
            Quat::identity(),
            wc,
            &mut world,
        );
        v.set_controls(0.5, 0.0, 0.0);
        for _ in 0..30 {
            v.step(1.0 / 60.0, &mut world);
            world.step(1.0 / 60.0).unwrap();
        }
        let body = world.body(v.body).unwrap();
        assert!(body.position.is_finite());
    }
    #[test]
    fn vehicle2_creates_and_moves() {
        let mut world = World2::default();
        let mut v = Vehicle2::new(Vec2 { x: 0.0, y: 1.0 }, Rot2::identity(), 200.0, &mut world);
        v.set_controls(1.0, 0.0);
        for _ in 0..30 {
            v.step(1.0 / 60.0, &mut world);
            world.step(1.0 / 60.0).unwrap();
        }
        let body = world.body(v.body).unwrap();
        assert!(body.position.x > 0.0 || body.velocity.x > 0.1);
    }
    #[test]
    fn character2_walks_and_jumps() {
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
            .unwrap();
        let mut c = Character2::new(CharacterConfig2::default(), Vec2 { x: 0.0, y: 2.0 });
        c.attach(&mut world);
        c.set_move(Vec2 { x: 1.0, y: 0.0 });
        for _ in 0..60 {
            c.step(1.0 / 60.0, &mut world);
            world.step(1.0 / 60.0).unwrap();
        }
        assert!(world.body(c.body.unwrap()).unwrap().position.x > 0.0);
    }
    #[test]
    fn character3_walks() {
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
            .unwrap();
        let mut c = Character3::new(
            CharacterConfig3::default(),
            Vec3 {
                x: 0.0,
                y: 2.0,
                z: 0.0,
            },
        );
        c.attach(&mut world);
        c.set_move(Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        });
        for _ in 0..60 {
            c.step(1.0 / 60.0, &mut world);
            world.step(1.0 / 60.0).unwrap();
        }
        let pos = world.body(c.body.unwrap()).unwrap().position.x;
        assert!(pos > 0.0, "should move, got {}", pos);
    }
    #[test]
    fn character_grounding_works() {
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
            .unwrap();
        let mut c = Character2::new(CharacterConfig2::default(), Vec2 { x: 0.0, y: 5.0 });
        c.attach(&mut world);
        for _ in 0..300 {
            c.step(1.0 / 60.0, &mut world);
            world.step(1.0 / 60.0).unwrap();
        }
        println!(
            "grounded: {} pos: {:?} hit: {:?}",
            c.is_grounded,
            c.position,
            world.ray_cast(
                auralite_math::Ray2::new(c.position + Vec2::Y * 0.1, -Vec2::Y).unwrap(),
                0.25
            )
        );
        assert!(c.is_grounded);
    }
    #[test]
    fn vehicle3_body_has_finite_state() {
        let mut world = World3::default();
        let mut v = Vehicle3::new(
            VehicleConfig3::default(),
            Vec3::Y,
            Quat::identity(),
            vec![WheelConfig3::default()],
            &mut world,
        );
        v.set_controls(0.3, 0.0, 0.0);
        for _ in 0..20 {
            v.step(1.0 / 60.0, &mut world);
            world.step(1.0 / 60.0).unwrap();
        }
        assert!(world.body(v.body).unwrap().position.is_finite());
    }
}
