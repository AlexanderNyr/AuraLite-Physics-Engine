//! Joint/constraint types for 2D and 3D.
//! Joint solving is integrated into the world step in lib.rs.

use crate::{Body2, BodyHandle2, BodyType, Pool};
use auralite_math::{ABS_EPSILON, Real, Vec2, Vec3};

/// A stable joint ID within a world.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct JointId(pub u64);

/// Joint break event.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointBreakEvent {
    pub joint_id: JointId,
    pub impulse: Real,
}

/// Configuration for a 2D joint.
#[derive(Clone, Debug)]
pub struct JointConfig2 {
    pub joint_type: JointType2,
    pub body_a: BodyHandle2,
    pub body_b: BodyHandle2,
    pub anchor_a: Vec2,
    pub anchor_b: Vec2,
    pub limits: JointLimits,
    pub motor: JointMotor,
    pub break_impulse: Real,
    pub user_data: u64,
}

impl JointConfig2 {
    pub fn new(
        joint_type: JointType2,
        body_a: BodyHandle2,
        body_b: BodyHandle2,
        anchor_a: Vec2,
        anchor_b: Vec2,
    ) -> Self {
        Self {
            joint_type,
            body_a,
            body_b,
            anchor_a,
            anchor_b,
            limits: JointLimits::default(),
            motor: JointMotor::default(),
            break_impulse: 0.0,
            user_data: 0,
        }
    }
}

/// 2D joint type.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JointType2 {
    Weld,
    Distance,
    Spring { stiffness: Real, damping: Real },
    Revolute,
    Prismatic { axis_local: Vec2 },
}

/// Joint angle/position limits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointLimits {
    pub min: Real,
    pub max: Real,
    pub enabled: bool,
}
impl Default for JointLimits {
    fn default() -> Self {
        Self {
            min: -Real::INFINITY,
            max: Real::INFINITY,
            enabled: false,
        }
    }
}

/// Joint motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointMotor {
    pub target_speed: Real,
    pub max_force: Real,
    pub enabled: bool,
}
impl Default for JointMotor {
    fn default() -> Self {
        Self {
            target_speed: 0.0,
            max_force: 0.0,
            enabled: false,
        }
    }
}

/// Runtime 2D joint state.
#[derive(Clone, Debug)]
pub struct Joint2 {
    pub config: JointConfig2,
    pub impulse: Real,
    pub accumulated_position_error: Real,
    pub broken: bool,
}

impl Joint2 {
    pub fn new(config: JointConfig2) -> Self {
        Self {
            config,
            impulse: 0.0,
            accumulated_position_error: 0.0,
            broken: false,
        }
    }

    /// Extract body data needed for solving without holding references.
    fn get_body_data(&self, bodies: &Pool<Body2>) -> Option<JointBodyData> {
        let a = bodies.get(self.config.body_a)?;
        let b = bodies.get(self.config.body_b)?;
        Some(JointBodyData {
            pos_a: a.position,
            rot_a: a.rotation,
            pos_b: b.position,
            rot_b: b.rotation,
            vel_a: a.velocity,
            vel_b: b.velocity,
            ang_a: a.angular_velocity,
            ang_b: b.angular_velocity,
            inv_mass_a: a.effective_inv_mass(),
            inv_mass_b: b.effective_inv_mass(),
            inv_inertia_a: a.effective_inv_inertia(),
            inv_inertia_b: b.effective_inv_inertia(),
            kind_a: a.kind,
            kind_b: b.kind,
            sleeping_a: a.sleeping,
            sleeping_b: b.sleeping,
        })
    }
}

#[expect(dead_code)]
struct JointBodyData {
    pos_a: Vec2,
    pos_b: Vec2,
    rot_a: crate::Rot2,
    rot_b: crate::Rot2,
    vel_a: Vec2,
    vel_b: Vec2,
    ang_a: Real,
    ang_b: Real,
    inv_mass_a: Real,
    inv_mass_b: Real,
    inv_inertia_a: Real,
    inv_inertia_b: Real,
    kind_a: BodyType,
    kind_b: BodyType,
    sleeping_a: bool,
    sleeping_b: bool,
}

impl Joint2 {
    /// Solve this joint constraint, applying impulses to connected bodies.
    /// Returns the impulse magnitude applied.
    pub fn solve(&mut self, bodies: &mut Pool<Body2>) -> Real {
        if self.broken {
            return 0.0;
        }
        let data = match self.get_body_data(bodies) {
            Some(d) => d,
            None => {
                self.broken = true;
                return 0.0;
            }
        };
        if data.sleeping_a && data.sleeping_b {
            return 0.0;
        }

        let world_a = data.pos_a + data.rot_a.rotate(self.config.anchor_a);
        let world_b = data.pos_b + data.rot_b.rotate(self.config.anchor_b);

        match self.config.joint_type {
            JointType2::Weld => {
                let error = world_b - world_a;
                let total_im = data.inv_mass_a + data.inv_mass_b;
                if total_im <= ABS_EPSILON {
                    return 0.0;
                }
                let impulse = error * 0.5 / total_im;
                let imp = impulse.length();
                if self.config.break_impulse > 0.0 && imp > self.config.break_impulse {
                    self.broken = true;
                    return imp;
                }
                self.impulse += imp;
                crate::apply_impulse2(
                    bodies,
                    self.config.body_a,
                    self.config.body_b,
                    impulse,
                    world_a,
                );
                imp
            }
            JointType2::Distance => {
                let diff = world_b - world_a;
                let dist = diff.length();
                if dist <= ABS_EPSILON {
                    return 0.0;
                }
                let dir = diff / dist;
                let rest = (self.config.anchor_b - self.config.anchor_a).length();
                let target = if self.config.limits.enabled {
                    dist.clamp(self.config.limits.min, self.config.limits.max)
                } else {
                    rest
                };
                let error = dist - target;
                if error.abs() <= ABS_EPSILON {
                    return 0.0;
                }
                let total_im = data.inv_mass_a + data.inv_mass_b;
                if total_im <= ABS_EPSILON {
                    return 0.0;
                }
                let impulse = dir * error * 0.5 / total_im;
                let imp = impulse.length();
                if self.config.break_impulse > 0.0 && imp > self.config.break_impulse {
                    self.broken = true;
                    return imp;
                }
                self.impulse += imp;
                self.accumulated_position_error += error.abs();
                crate::apply_impulse2(
                    bodies,
                    self.config.body_a,
                    self.config.body_b,
                    impulse,
                    world_a,
                );
                imp
            }
            JointType2::Spring { stiffness, damping } => {
                let diff = world_b - world_a;
                let dist = diff.length();
                if dist <= ABS_EPSILON {
                    return 0.0;
                }
                let dir = diff / dist;
                let rest = (self.config.anchor_b - self.config.anchor_a).length();
                let spring_force = (dist - rest) * stiffness;
                let rel_vel = (data.vel_b - data.vel_a).dot(dir);
                let damping_force = rel_vel * damping;
                let total_force = spring_force + damping_force;
                let total_im = data.inv_mass_a + data.inv_mass_b;
                if total_im <= ABS_EPSILON {
                    return 0.0;
                }
                let imp_mag = total_force * 0.016666668 / total_im.max(ABS_EPSILON);
                let impulse = dir * imp_mag.min(1000.0);
                if self.config.break_impulse > 0.0 && imp_mag.abs() > self.config.break_impulse {
                    self.broken = true;
                    return imp_mag.abs();
                }
                self.impulse += imp_mag.abs();
                self.accumulated_position_error += (dist - rest).abs();
                crate::apply_impulse2(
                    bodies,
                    self.config.body_a,
                    self.config.body_b,
                    impulse,
                    world_a,
                );
                imp_mag.abs()
            }
            JointType2::Revolute => {
                let error = world_b - world_a;
                let total_im = data.inv_mass_a + data.inv_mass_b;
                if total_im <= ABS_EPSILON {
                    return 0.0;
                }
                let impulse = error * 0.5 / total_im;
                let imp = impulse.length();
                if self.config.break_impulse > 0.0 && imp > self.config.break_impulse {
                    self.broken = true;
                    return imp;
                }
                self.impulse += imp;
                self.accumulated_position_error += error.length();
                crate::apply_impulse2(
                    bodies,
                    self.config.body_a,
                    self.config.body_b,
                    impulse,
                    world_a,
                );
                if self.config.motor.enabled {
                    let target_vel = self.config.motor.target_speed;
                    let max_torque = self.config.motor.max_force;
                    let cur_vel = data.ang_b - data.ang_a;
                    let vel_error = target_vel - cur_vel;
                    let torque = vel_error.clamp(-max_torque, max_torque);
                    if let Some(a) = bodies.get_mut(self.config.body_a) {
                        if a.kind == BodyType::Dynamic && !a.sleeping {
                            a.angular_velocity -= torque * a.effective_inv_inertia();
                        }
                    }
                    if let Some(b) = bodies.get_mut(self.config.body_b) {
                        if b.kind == BodyType::Dynamic && !b.sleeping {
                            b.angular_velocity += torque * b.effective_inv_inertia();
                        }
                    }
                }
                imp
            }
            JointType2::Prismatic { axis_local } => {
                let world_axis = data.rot_a.rotate(axis_local).normalized_or(Vec2::X);
                let offset = world_b - world_a;
                let along = offset.dot(world_axis);
                let lateral = offset - world_axis * along;
                if lateral.length_squared() > ABS_EPSILON {
                    let total_im = data.inv_mass_a + data.inv_mass_b;
                    if total_im > ABS_EPSILON {
                        let impulse = lateral * 0.5 / total_im;
                        let imp = impulse.length();
                        if self.config.break_impulse > 0.0 && imp > self.config.break_impulse {
                            self.broken = true;
                            return imp;
                        }
                        crate::apply_impulse2(
                            bodies,
                            self.config.body_a,
                            self.config.body_b,
                            impulse,
                            world_a,
                        );
                    }
                }
                if self.config.limits.enabled {
                    let clamped = along.clamp(self.config.limits.min, self.config.limits.max);
                    let error = along - clamped;
                    if error.abs() > ABS_EPSILON {
                        let total_im = data.inv_mass_a + data.inv_mass_b;
                        if total_im > ABS_EPSILON {
                            let impulse = world_axis * error * 0.5 / total_im;
                            if self.config.break_impulse > 0.0
                                && impulse.length() > self.config.break_impulse
                            {
                                self.broken = true;
                                return impulse.length();
                            }
                            crate::apply_impulse2(
                                bodies,
                                self.config.body_a,
                                self.config.body_b,
                                impulse,
                                world_a,
                            );
                        }
                    }
                }
                if self.config.motor.enabled {
                    let target_vel = self.config.motor.target_speed;
                    let max_force = self.config.motor.max_force;
                    let rel_vel = (data.vel_b - data.vel_a).dot(world_axis);
                    let vel_error = target_vel - rel_vel;
                    let force = vel_error.clamp(-max_force, max_force);
                    let impulse = world_axis * force * 0.016666668;
                    crate::apply_impulse2(
                        bodies,
                        self.config.body_a,
                        self.config.body_b,
                        impulse,
                        world_a,
                    );
                }
                0.0
            }
        }
    }

    pub fn set_limits(&mut self, limits: JointLimits) {
        self.config.limits = limits;
    }
    pub fn set_motor(&mut self, motor: JointMotor) {
        self.config.motor = motor;
    }
    pub fn set_break_impulse(&mut self, impulse: Real) {
        self.config.break_impulse = impulse;
    }
}

/// 3D joint configuration.
#[derive(Clone, Debug)]
pub struct JointConfig3 {
    pub joint_type: JointType3,
    pub body_a: crate::BodyHandle3,
    pub body_b: crate::BodyHandle3,
    pub anchor_a: Vec3,
    pub anchor_b: Vec3,
    pub limits: JointLimits,
    pub motor: JointMotor,
    pub break_impulse: Real,
    pub user_data: u64,
}

/// 3D joint type.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JointType3 {
    BallSocket,
    Distance,
    Spring { stiffness: Real, damping: Real },
}

/// Runtime 3D joint state.
#[derive(Clone, Debug)]
pub struct Joint3 {
    pub config: JointConfig3,
    pub impulse: Real,
    pub accumulated_position_error: Real,
    pub broken: bool,
}

impl Joint3 {
    pub fn new(config: JointConfig3) -> Self {
        Self {
            config,
            impulse: 0.0,
            accumulated_position_error: 0.0,
            broken: false,
        }
    }
}
