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
    pub id: JointId,
    pub config: JointConfig2,
    pub impulse: Real,
    pub accumulated_position_error: Real,
    pub broken: bool,
}

impl Joint2 {
    pub fn new(id: JointId, config: JointConfig2) -> Self {
        Self {
            id,
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

fn perp(v: Vec2) -> Vec2 {
    Vec2 { x: -v.y, y: v.x }
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
                let world_a = data.pos_a + data.rot_a.rotate(self.config.anchor_a);
                let world_b = data.pos_b + data.rot_b.rotate(self.config.anchor_b);
                let error = world_b - world_a;
                let rel_vel = (data.vel_b + perp(world_b - data.pos_b) * data.ang_b)
                    - (data.vel_a + perp(world_a - data.pos_a) * data.ang_a);

                let total_im = data.inv_mass_a + data.inv_mass_b;
                if total_im <= ABS_EPSILON {
                    return 0.0;
                }

                let bias = error * 0.2 / 0.016666668;
                let impulse = (bias - rel_vel) * 0.1 / total_im;
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
                let world_a = data.pos_a + data.rot_a.rotate(self.config.anchor_a);
                let world_b = data.pos_b + data.rot_b.rotate(self.config.anchor_b);
                let diff = world_b - world_a;
                let dist = diff.length();
                if dist <= ABS_EPSILON {
                    return 0.0;
                }
                let dir = diff / dist;
                let rest = (self.config.anchor_b - self.config.anchor_a).length();
                let error = dist - rest;

                let rel_vel = (data.vel_b - data.vel_a).dot(dir);
                let total_im = data.inv_mass_a + data.inv_mass_b;
                if total_im <= ABS_EPSILON {
                    return 0.0;
                }

                let lambda = (error * 0.1 - rel_vel * 0.01) / total_im;
                if self.config.break_impulse > 0.0 && lambda.abs() >= self.config.break_impulse {
                    self.broken = true;
                    return lambda.abs();
                }
                crate::apply_impulse2(
                    bodies,
                    self.config.body_a,
                    self.config.body_b,
                    dir * lambda,
                    world_a,
                );
                lambda.abs()
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
    Weld,
    Hinge { axis_local: Vec3 },
    Slider { axis_local: Vec3 },
}

/// Runtime 3D joint state.
#[derive(Clone, Debug)]
pub struct Joint3 {
    pub id: JointId,
    pub config: JointConfig3,
    pub impulse: Real,
    pub accumulated_position_error: Real,
    pub broken: bool,
}

impl Joint3 {
    pub fn new(id: JointId, config: JointConfig3) -> Self {
        Self {
            id,
            config,
            impulse: 0.0,
            accumulated_position_error: 0.0,
            broken: false,
        }
    }

    /// Solve 3D constraint.
    pub fn solve(&mut self, bodies: &mut Pool<crate::Body3>) -> Real {
        if self.broken {
            return 0.0;
        }
        let (ba_pos, ba_rot, ba_vel, _ba_ang, ba_im, _ba_ii, _ba_kind, ba_sleep) = {
            let a = match bodies.get(self.config.body_a) {
                Some(a) => a,
                None => {
                    self.broken = true;
                    return 0.0;
                }
            };
            (
                a.position,
                a.rotation,
                a.velocity,
                a.angular_velocity,
                a.effective_inv_mass(),
                a.inv_inertia_diagonal,
                a.kind,
                a.sleeping,
            )
        };
        let (bb_pos, bb_rot, bb_vel, _bb_ang, bb_im, _bb_ii, _bb_kind, bb_sleep) = {
            let b = match bodies.get(self.config.body_b) {
                Some(b) => b,
                None => {
                    self.broken = true;
                    return 0.0;
                }
            };
            (
                b.position,
                b.rotation,
                b.velocity,
                b.angular_velocity,
                b.effective_inv_mass(),
                b.inv_inertia_diagonal,
                b.kind,
                b.sleeping,
            )
        };

        if ba_sleep && bb_sleep {
            return 0.0;
        }

        let world_a = ba_pos + ba_rot.rotate(self.config.anchor_a);
        let world_b = bb_pos + bb_rot.rotate(self.config.anchor_b);

        match self.config.joint_type {
            JointType3::BallSocket | JointType3::Weld => {
                let error = world_b - world_a;
                let total_im = ba_im + bb_im;
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
                self.accumulated_position_error += imp;
                crate::apply_impulse3(
                    bodies,
                    self.config.body_a,
                    self.config.body_b,
                    impulse,
                    world_a,
                );
                imp
            }
            JointType3::Distance => {
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
                let total_im = ba_im + bb_im;
                if total_im <= ABS_EPSILON {
                    return 0.0;
                }
                let impulse = dir * error * 0.5 / total_im;
                let imp = impulse.length();
                if self.config.break_impulse > 0.0 && imp >= self.config.break_impulse {
                    self.broken = true;
                    return imp;
                }
                crate::apply_impulse3(
                    bodies,
                    self.config.body_a,
                    self.config.body_b,
                    impulse,
                    world_a,
                );
                imp
            }
            JointType3::Spring { stiffness, damping } => {
                let diff = world_b - world_a;
                let dist = diff.length();
                if dist <= ABS_EPSILON {
                    return 0.0;
                }
                let dir = diff / dist;
                let rest = (self.config.anchor_b - self.config.anchor_a).length();
                let spring_force = (dist - rest) * stiffness;
                let rel_vel = (bb_vel - ba_vel).dot(dir);
                let damping_force = rel_vel * damping;
                let total_force = spring_force + damping_force;
                let impulse = dir * total_force * 0.016666668;
                crate::apply_impulse3(
                    bodies,
                    self.config.body_a,
                    self.config.body_b,
                    impulse,
                    world_a,
                );
                impulse.length()
            }
            JointType3::Hinge { axis_local } => {
                let error = world_b - world_a;
                let total_im = ba_im + bb_im;
                let mut imp = 0.0;
                if total_im > ABS_EPSILON {
                    let impulse = error * 0.5 / total_im;
                    imp = impulse.length();
                    if self.config.break_impulse > 0.0 && imp > self.config.break_impulse {
                        self.broken = true;
                        return imp;
                    }
                    self.impulse += imp;
                    self.accumulated_position_error += imp;
                    crate::apply_impulse3(
                        bodies,
                        self.config.body_a,
                        self.config.body_b,
                        impulse,
                        world_a,
                    );
                }
                if self.config.motor.enabled {
                    let target_vel = self.config.motor.target_speed;
                    let max_force = self.config.motor.max_force;
                    let world_axis = ba_rot.rotate(axis_local).normalized_or(Vec3::X);
                    if let (Some(a), Some(b)) = (
                        bodies.get(self.config.body_a),
                        bodies.get(self.config.body_b),
                    ) {
                        let rel_vel = (b.angular_velocity - a.angular_velocity).dot(world_axis);
                        let vel_error = target_vel - rel_vel;
                        let torque = vel_error.clamp(-max_force, max_force);
                        let ang_imp = world_axis * torque * 0.016666668;
                        if let Some(am) = bodies.get_mut(self.config.body_a) {
                            if am.kind == crate::BodyType::Dynamic {
                                am.angular_velocity -= ang_imp;
                            }
                        }
                        if let Some(bm) = bodies.get_mut(self.config.body_b) {
                            if bm.kind == crate::BodyType::Dynamic {
                                bm.angular_velocity += ang_imp;
                            }
                        }
                    }
                }
                imp
            }
            JointType3::Slider { axis_local } => {
                let world_axis = ba_rot.rotate(axis_local).normalized_or(Vec3::X);
                let offset = world_b - world_a;
                let lateral = offset - world_axis * offset.dot(world_axis);
                let total_im = ba_im + bb_im;
                let mut imp = 0.0;
                if total_im > ABS_EPSILON {
                    let impulse = lateral * 0.5 / total_im;
                    imp = impulse.length();
                    if self.config.break_impulse > 0.0 && imp > self.config.break_impulse {
                        self.broken = true;
                        return imp;
                    }
                    self.impulse += imp;
                    self.accumulated_position_error += imp;
                    crate::apply_impulse3(
                        bodies,
                        self.config.body_a,
                        self.config.body_b,
                        impulse,
                        world_a,
                    );
                }
                if self.config.motor.enabled {
                    let target_vel = self.config.motor.target_speed;
                    let max_force = self.config.motor.max_force;
                    if let (Some(a), Some(b)) = (
                        bodies.get(self.config.body_a),
                        bodies.get(self.config.body_b),
                    ) {
                        let rel_vel = (b.velocity - a.velocity).dot(world_axis);
                        let vel_error = target_vel - rel_vel;
                        let force = vel_error.clamp(-max_force, max_force);
                        let lin_imp = world_axis * force * 0.016666668;
                        if let Some(am) = bodies.get_mut(self.config.body_a) {
                            if am.kind == crate::BodyType::Dynamic {
                                am.velocity -= lin_imp * am.inv_mass;
                            }
                        }
                        if let Some(bm) = bodies.get_mut(self.config.body_b) {
                            if bm.kind == crate::BodyType::Dynamic {
                                bm.velocity += lin_imp * bm.inv_mass;
                            }
                        }
                    }
                }
                imp
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyBuilder2, BodyBuilder3, World2, World3};
    use auralite_math::{Vec2, Vec3};

    #[test]
    fn joint2_break_impulse_breaks_under_excess_force() {
        let mut w = World2::default();
        let b1 = w.add_body(BodyBuilder2::static_body()).unwrap();
        let b2 = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 10.0, y: 0.0 })
                    .velocity(Vec2 { x: 100.0, y: 0.0 }),
            )
            .unwrap();
        let j = w
            .add_joint(JointConfig2 {
                joint_type: JointType2::Distance,
                body_a: b1,
                body_b: b2,
                anchor_a: Vec2::ZERO,
                anchor_b: Vec2::ZERO,
                limits: JointLimits::default(),
                motor: JointMotor::default(),
                break_impulse: 1.0,
                user_data: 0,
            })
            .unwrap();
        for _ in 0..10 {
            w.step(0.016).unwrap();
        }
        assert!(
            w.joint(j).unwrap().broken,
            "Joint2 should break under high impulse"
        );
    }

    #[test]
    fn joint3_break_impulse_breaks_under_excess_force() {
        let mut w = World3::default();
        let b1 = w.add_body(BodyBuilder3::static_body()).unwrap();
        let b2 = w
            .add_body(
                BodyBuilder3::dynamic()
                    .position(Vec3 {
                        x: 10.0,
                        y: 0.0,
                        z: 0.0,
                    })
                    .velocity(Vec3 {
                        x: 100.0,
                        y: 0.0,
                        z: 0.0,
                    }),
            )
            .unwrap();
        let j = w
            .add_joint(JointConfig3 {
                joint_type: JointType3::Distance,
                body_a: b1,
                body_b: b2,
                anchor_a: Vec3::ZERO,
                anchor_b: Vec3::ZERO,
                limits: JointLimits::default(),
                motor: JointMotor::default(),
                break_impulse: 5.0,
                user_data: 0,
            })
            .unwrap();
        w.step(0.016).unwrap();
        assert!(
            w.joint(j).unwrap().broken,
            "Joint3 should break under high impulse"
        );
    }

    #[test]
    fn joint3_hinge_motor_converges_to_target_speed() {
        let mut w = World3::default();
        w.gravity = Vec3::ZERO;
        let b1 = w.add_body(BodyBuilder3::static_body()).unwrap();
        let b2 = w.add_body(BodyBuilder3::dynamic()).unwrap();
        let mut motor = JointMotor::default();
        motor.enabled = true;
        motor.target_speed = 5.0;
        motor.max_force = 100.0;
        w.add_joint(JointConfig3 {
            joint_type: JointType3::Hinge {
                axis_local: Vec3::Z,
            },
            body_a: b1,
            body_b: b2,
            anchor_a: Vec3::ZERO,
            anchor_b: Vec3::ZERO,
            limits: JointLimits::default(),
            motor,
            break_impulse: 0.0,
            user_data: 0,
        })
        .unwrap();
        for _ in 0..30 {
            w.step(0.016).unwrap();
        }
        let ang_z = w.body(b2).unwrap().angular_velocity.z;
        assert!(
            (ang_z - 5.0).abs() < 1.0,
            "Hinge motor must drive angular velocity, got {}",
            ang_z
        );
    }

    #[test]
    fn joint3_slider_motor_converges_to_target_speed() {
        let mut w = World3::default();
        w.gravity = Vec3::ZERO;
        let b1 = w.add_body(BodyBuilder3::static_body()).unwrap();
        let b2 = w.add_body(BodyBuilder3::dynamic()).unwrap();
        let mut motor = JointMotor::default();
        motor.enabled = true;
        motor.target_speed = 5.0;
        motor.max_force = 100.0;
        w.add_joint(JointConfig3 {
            joint_type: JointType3::Slider {
                axis_local: Vec3::X,
            },
            body_a: b1,
            body_b: b2,
            anchor_a: Vec3::ZERO,
            anchor_b: Vec3::ZERO,
            limits: JointLimits::default(),
            motor,
            break_impulse: 0.0,
            user_data: 0,
        })
        .unwrap();
        for _ in 0..30 {
            w.step(0.016).unwrap();
        }
        let vel_x = w.body(b2).unwrap().velocity.x;
        assert!(
            (vel_x - 5.0).abs() < 1.0,
            "Slider motor must drive linear velocity, got {}",
            vel_x
        );
    }
}
