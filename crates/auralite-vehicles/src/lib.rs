//! Vehicles (2D/3D) and character controllers (2D/3D).
#![forbid(unsafe_code)]
#![allow(missing_docs, clippy::too_many_arguments)]

use auralite_collision::CollisionFilter;
use auralite_dynamics::{
    BodyBuilder2, BodyBuilder3, BodyHandle2, BodyHandle3, Collider2, Collider3, ColliderShape2,
    ColliderShape3, Material, World2, World3,
};
use auralite_math::{ABS_EPSILON, Quat, Real, Rot2, Vec2, Vec3};

// ─── 3D Vehicle ──────────────────────────────────────────────────────────────

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
                * (config.chassis_half_extents.y * config.chassis_half_extents.y
                    + config.chassis_half_extents.z * config.chassis_half_extents.z)
                / 12.0,
            y: config.chassis_mass
                * (config.chassis_half_extents.x * config.chassis_half_extents.x
                    + config.chassis_half_extents.z * config.chassis_half_extents.z)
                / 12.0,
            z: config.chassis_mass
                * (config.chassis_half_extents.x * config.chassis_half_extents.x
                    + config.chassis_half_extents.y * config.chassis_half_extents.y)
                / 12.0,
        };

        let builder = BodyBuilder3::dynamic()
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
            });

        let body = world.add_body(builder).unwrap();
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

        // Process each wheel
        for wheel in &mut self.wheels {
            wheel.prev_suspension_length = wheel.suspension_length;
            let local_attach = wheel.config.attachment_point;
            let world_attach = chassis.position + chassis.rotation.rotate(local_attach);
            let sus_dir = chassis
                .rotation
                .rotate(wheel.config.suspension_direction)
                .normalized_or(-Vec3::Y);
            let ray_max = wheel.config.suspension_travel + wheel.config.wheel_radius;

            // Ray-ground intersection
            let hit = ray_plane_intersection(world_attach, sus_dir, Vec3::Y, 0.0);
            let hit_distance = hit.unwrap_or(ray_max);
            let is_hit = hit_distance < ray_max;

            if is_hit {
                wheel.suspension_length =
                    (hit_distance - wheel.config.wheel_radius).min(wheel.config.suspension_travel);
                wheel.contact_point = world_attach + sus_dir * hit_distance;
                wheel.contact_normal = Vec3::Y;
                wheel.is_grounded = true;

                // Suspension force
                let compression =
                    (wheel.config.suspension_travel - wheel.suspension_length).max(0.0);
                let spring_force = compression * wheel.config.suspension_stiffness;
                let sus_vel =
                    (wheel.suspension_length - wheel.prev_suspension_length) / dt.max(ABS_EPSILON);
                let damper_force = sus_vel * wheel.config.suspension_damping;
                let suspension_force = (spring_force + damper_force).max(0.0);

                // Apply suspension as CoM impulse
                let sus_impulse = sus_dir * suspension_force * dt;
                let _ = world.apply_impulse(self.body, sus_impulse);

                // Steering
                if wheel.config.steered {
                    let target = self.steer_input * self.config.max_steer_angle;
                    let rate = self.config.steer_speed * dt;
                    if (wheel.steer_angle - target).abs() > rate {
                        wheel.steer_angle += rate * target.signum();
                    } else {
                        wheel.steer_angle = target;
                    }
                }

                // Wheel forward/right directions
                let chassis_fwd = chassis
                    .rotation
                    .rotate(Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: -1.0,
                    })
                    .normalized_or(Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: -1.0,
                    });
                let chassis_right = chassis.rotation.rotate(Vec3::X).normalized_or(Vec3::X);
                let (wf, wr) = if wheel.config.steered && wheel.steer_angle.abs() > ABS_EPSILON {
                    let c = wheel.steer_angle.cos();
                    let s = wheel.steer_angle.sin();
                    let fwd = (chassis_fwd * c + chassis_right * s).normalized_or(chassis_fwd);
                    let right = fwd.cross(wheel.contact_normal).normalized_or(chassis_right);
                    (fwd, right)
                } else {
                    (chassis_fwd, chassis_right)
                };

                // Wheel velocity at contact
                let r = wheel.contact_point - chassis.position;
                let vel_at = chassis.velocity + chassis.angular_velocity.cross(r);

                // Slip
                let long_vel = vel_at.dot(wf);
                let wheel_circ_vel = wheel.angular_velocity * wheel.config.wheel_radius;
                wheel.slip_long = if self.speed > 0.5 {
                    (long_vel - wheel_circ_vel) / self.speed
                } else {
                    0.0
                };
                wheel.slip_lat = vel_at.dot(wr) / self.speed.max(1.0);

                // Drive/brake torque
                let engine_torque = if wheel.config.driven {
                    self.throttle * self.config.max_engine_torque
                } else {
                    0.0
                };
                let brake_torque = if self.brake > 0.0 {
                    -self.brake * wheel.config.brake_torque
                } else {
                    0.0
                };
                let net_torque = engine_torque + brake_torque;
                let w_inertia = 0.5
                    * wheel.config.wheel_mass
                    * wheel.config.wheel_radius
                    * wheel.config.wheel_radius;
                wheel.angular_velocity += net_torque * dt / w_inertia.max(ABS_EPSILON);

                // Tire forces (simplified Pacejka)
                let fz = suspension_force.max(0.0);
                let f_long = fz * wheel.config.friction_long * wheel.slip_long.tanh() * 0.5;
                wheel.longitudinal_force = f_long;
                let f_lat = -fz * wheel.config.friction_lat * (3.0 * wheel.slip_lat).tanh();
                wheel.lateral_force = f_lat;

                // Apply tire impulse at CoM (simplified)
                let tire_impulse = (wf * f_long + wr * f_lat) * dt;
                let _ = world.apply_impulse(self.body, tire_impulse);
            } else {
                wheel.is_grounded = false;
                wheel.suspension_length = wheel.config.suspension_travel;
                wheel.slip_long = 0.0;
                wheel.slip_lat = 0.0;
                wheel.longitudinal_force = 0.0;
                wheel.lateral_force = 0.0;
            }
        }

        // Air drag
        if let Ok(b) = world.body(self.body) {
            let vel = b.velocity;
            let speed = vel.length();
            if speed > 0.1 {
                let drag = vel / -speed * 0.3 * speed * speed * dt;
                let _ = world.apply_impulse(self.body, drag);
            }
        }
    }
}

fn ray_plane_intersection(
    origin: Vec3,
    dir: Vec3,
    plane_normal: Vec3,
    plane_offset: Real,
) -> Option<Real> {
    let denom = dir.dot(plane_normal);
    if denom.abs() <= ABS_EPSILON {
        return None;
    }
    let t = -(origin.dot(plane_normal) - plane_offset) / denom;
    if t >= 0.0 { Some(t) } else { None }
}

// ─── 2D Vehicle ──────────────────────────────────────────────────────────────

pub struct Vehicle2 {
    pub body: BodyHandle2,
    pub throttle: Real,
    pub brake: Real,
    pub speed: Real,
}

impl Vehicle2 {
    pub fn new(position: Vec2, rotation: Rot2, mass: Real, world: &mut World2) -> Self {
        let inertia = mass * 1.0;
        let builder = BodyBuilder2::dynamic()
            .position(position)
            .rotation(rotation)
            .mass(mass)
            .inertia(inertia)
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
            });
        let body = world.add_body(builder).unwrap();
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

// ─── Character Controller 2D ─────────────────────────────────────────────────

#[derive(Clone, Debug)]
pub struct CharacterConfig2 {
    pub height: Real,
    pub radius: Real,
    pub skin_width: Real,
    pub move_speed: Real,
    pub acceleration: Real,
    pub gravity: Vec2,
    pub jump_velocity: Real,
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
        let b = world
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
            .unwrap();
        self.body = Some(b);
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

        let b = match world.body(body_h) {
            Ok(b) => b.clone(),
            Err(_) => return,
        };
        self.position = b.position;
        let ground_y = self.config.radius + self.config.skin_width;
        self.is_grounded = self.position.y <= ground_y
            || (b.velocity.y.abs() < 0.1 && (self.position.y - ground_y).abs() < 0.1);

        let target_vx = self.move_input.x * self.config.move_speed;

        if let Ok(b) = world.body_mut(body_h) {
            if self.is_grounded {
                let diff = target_vx - b.velocity.x;
                let accel = self.config.acceleration * dt;
                if diff.abs() <= accel {
                    b.velocity.x = target_vx;
                } else {
                    b.velocity.x += accel * diff.signum();
                }

                if self.want_jump && self.jump_cooldown <= 0.0 {
                    b.velocity.y = self.config.jump_velocity;
                    self.want_jump = false;
                    self.jump_cooldown = 0.1;
                    self.is_grounded = false;
                }
            } else {
                b.velocity += self.config.gravity * dt;
                if self.position.y <= ground_y {
                    b.velocity.y = 0.0;
                    self.is_grounded = true;
                }
            }
        }
        self.want_jump = false;
    }
}

// ─── Character Controller 3D ─────────────────────────────────────────────────

#[derive(Clone, Debug)]
pub struct CharacterConfig3 {
    pub height: Real,
    pub radius: Real,
    pub skin_width: Real,
    pub move_speed: Real,
    pub acceleration: Real,
    pub gravity: Vec3,
    pub jump_velocity: Real,
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
        let b = world
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
            .unwrap();
        self.body = Some(b);
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

        // Read chassis state
        let (pos, vel) = match world.body(body_h) {
            Ok(b) => (b.position, b.velocity),
            Err(_) => return,
        };
        self.position = pos;
        let ground_y = self.config.radius + self.config.skin_width;
        self.is_grounded = pos.y <= ground_y && vel.y <= 0.0;

        let target_v = Vec3 {
            x: self.move_input.x * self.config.move_speed,
            y: 0.0,
            z: self.move_input.z * self.config.move_speed,
        };

        // Apply velocity changes through impulses
        if self.is_grounded {
            // Horizontal control
            let diff_x = target_v.x - vel.x;
            let diff_z = target_v.z - vel.z;
            let accel = self.config.acceleration * dt;

            let imp_x = if diff_x.abs() <= accel {
                target_v.x - vel.x
            } else {
                accel * diff_x.signum()
            };
            let imp_z = if diff_z.abs() <= accel {
                target_v.z - vel.z
            } else {
                accel * diff_z.signum()
            };

            if imp_x.abs() > ABS_EPSILON || imp_z.abs() > ABS_EPSILON {
                let _ = world.apply_impulse(
                    body_h,
                    Vec3 {
                        x: imp_x,
                        y: 0.0,
                        z: imp_z,
                    },
                );
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
            }
        } else {
            // In air: gravity + air control
            let _ = world.apply_impulse(body_h, self.config.gravity * dt);
            let air_accel = self.config.acceleration * 0.3 * dt;
            let diff_x = target_v.x - vel.x;
            let diff_z = target_v.z - vel.z;
            if diff_x.abs() > ABS_EPSILON {
                let imp_x = if diff_x.abs() <= air_accel {
                    diff_x
                } else {
                    air_accel * diff_x.signum()
                };
                let _ = world.apply_impulse(
                    body_h,
                    Vec3 {
                        x: imp_x,
                        y: 0.0,
                        z: 0.0,
                    },
                );
            }
            if diff_z.abs() > ABS_EPSILON {
                let imp_z = if diff_z.abs() <= air_accel {
                    diff_z
                } else {
                    air_accel * diff_z.signum()
                };
                let _ = world.apply_impulse(
                    body_h,
                    Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: imp_z,
                    },
                );
            }
        }
        self.want_jump = false;
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vehicle3_creates_and_steps() {
        let mut world = World3::default();
        world
            .set_gravity(Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            })
            .unwrap();
        let wc = vec![
            WheelConfig3 {
                attachment_point: Vec3 {
                    x: -0.8,
                    y: -0.2,
                    z: 1.5,
                },
                steered: true,
                driven: true,
                ..Default::default()
            },
            WheelConfig3 {
                attachment_point: Vec3 {
                    x: 0.8,
                    y: -0.2,
                    z: 1.5,
                },
                steered: true,
                driven: true,
                ..Default::default()
            },
            WheelConfig3 {
                attachment_point: Vec3 {
                    x: -0.8,
                    y: -0.2,
                    z: -1.5,
                },
                steered: false,
                driven: true,
                ..Default::default()
            },
            WheelConfig3 {
                attachment_point: Vec3 {
                    x: 0.8,
                    y: -0.2,
                    z: -1.5,
                },
                steered: false,
                driven: true,
                ..Default::default()
            },
        ];
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
        assert!(body.velocity.is_finite());
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
        assert!(
            body.position.x > 0.0 || body.velocity.x > 0.1,
            "2D vehicle should move"
        );
    }

    #[test]
    fn character2_walks_and_jumps() {
        let mut world = World2::default();
        let mut c = Character2::new(CharacterConfig2::default(), Vec2 { x: 0.0, y: 2.0 });
        c.attach(&mut world);
        c.set_move(Vec2 { x: 1.0, y: 0.0 });
        for _ in 0..30 {
            c.step(1.0 / 60.0, &mut world);
            world.step(1.0 / 60.0).unwrap();
        }
        assert!(
            world.body(c.body.unwrap()).unwrap().position.x > 0.0,
            "should move right"
        );
        c.jump();
        for _ in 0..30 {
            c.step(1.0 / 60.0, &mut world);
            world.step(1.0 / 60.0).unwrap();
        }
        assert!(world.body(c.body.unwrap()).unwrap().position.is_finite());
    }

    #[test]
    fn character3_walks() {
        let mut world = World3::default();
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
        for _ in 0..30 {
            c.step(1.0 / 60.0, &mut world);
            world.step(1.0 / 60.0).unwrap();
        }
        assert!(world.body(c.body.unwrap()).unwrap().position.x > 0.0);
    }

    #[test]
    fn character_grounding_works() {
        let mut world = World2::default();
        let mut c = Character2::new(CharacterConfig2::default(), Vec2 { x: 0.0, y: 100.0 });
        c.attach(&mut world);
        for _ in 0..300 {
            c.step(1.0 / 60.0, &mut world);
            world.step(1.0 / 60.0).unwrap();
        }
        assert!(c.is_grounded, "should be grounded after falling");
    }

    #[test]
    fn vehicle3_body_has_finite_state() {
        let mut world = World3::default();
        let wc = vec![WheelConfig3::default(); 2];
        let mut v = Vehicle3::new(
            VehicleConfig3::default(),
            Vec3::Y,
            Quat::identity(),
            wc,
            &mut world,
        );
        v.set_controls(0.3, 0.0, 0.0);
        for _ in 0..20 {
            v.step(1.0 / 60.0, &mut world);
            world.step(1.0 / 60.0).unwrap();
        }
        let body = world.body(v.body).unwrap();
        assert!(body.position.is_finite());
        assert!(body.velocity.is_finite());
    }
}
