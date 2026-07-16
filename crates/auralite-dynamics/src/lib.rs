//! Native dimension-separated rigid-body worlds.
#![forbid(unsafe_code)]
use auralite_core::{Handle, Pool, StableId, hash_bytes};
use auralite_math::{CONTACT_SLOP, Real, Vec2, Vec3};

/// Body motion classification.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyType {
    /// Never moves.
    Static,
    /// Velocity-driven, infinite mass.
    Kinematic,
    /// Force/impulse-driven.
    Dynamic,
}
/// World construction/step error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WorldError {
    /// Input was non-finite or non-positive where required.
    InvalidInput,
    /// A handle was stale.
    StaleHandle,
}
/// Native 2D rigid body (circle collider in the current vertical slice).
#[derive(Clone, Debug)]
pub struct Body2 {
    /// Stable deterministic identity.
    pub id: StableId,
    /// Motion type.
    pub kind: BodyType,
    /// Center position.
    pub position: Vec2,
    /// Linear velocity.
    pub velocity: Vec2,
    /// Circle radius.
    pub radius: Real,
    /// Inverse mass.
    pub inv_mass: Real,
    /// Coefficient of restitution.
    pub restitution: Real,
    /// Whether sleeping.
    pub sleeping: bool,
}
/// Native 3D rigid body (sphere collider in the current vertical slice).
#[derive(Clone, Debug)]
pub struct Body3 {
    /// Stable deterministic identity.
    pub id: StableId,
    /// Motion type.
    pub kind: BodyType,
    /// Center position.
    pub position: Vec3,
    /// Linear velocity.
    pub velocity: Vec3,
    /// Sphere radius.
    pub radius: Real,
    /// Inverse mass.
    pub inv_mass: Real,
    /// Coefficient of restitution.
    pub restitution: Real,
    /// Whether sleeping.
    pub sleeping: bool,
}
/// 2D body handle; cannot be passed to a 3D world.
pub type BodyHandle2 = Handle<Body2>;
/// 3D body handle; cannot be passed to a 2D world.
pub type BodyHandle3 = Handle<Body3>;
/// 2D body builder.
pub struct BodyBuilder2 {
    kind: BodyType,
    position: Vec2,
    velocity: Vec2,
    radius: Real,
    mass: Real,
    restitution: Real,
}
impl BodyBuilder2 {
    /// Creates a dynamic body builder.
    #[must_use]
    pub fn dynamic() -> Self {
        Self {
            kind: BodyType::Dynamic,
            position: Vec2::ZERO,
            velocity: Vec2::ZERO,
            radius: 0.5,
            mass: 1.0,
            restitution: 0.0,
        }
    }
    /// Sets position.
    #[must_use]
    pub fn position(mut self, v: Vec2) -> Self {
        self.position = v;
        self
    }
    /// Sets velocity.
    #[must_use]
    pub fn velocity(mut self, v: Vec2) -> Self {
        self.velocity = v;
        self
    }
    /// Sets radius.
    #[must_use]
    pub fn radius(mut self, v: Real) -> Self {
        self.radius = v;
        self
    }
    /// Sets mass.
    #[must_use]
    pub fn mass(mut self, v: Real) -> Self {
        self.mass = v;
        self
    }
    /// Sets restitution.
    #[must_use]
    pub fn restitution(mut self, v: Real) -> Self {
        self.restitution = v;
        self
    }
}
/// 3D body builder.
pub struct BodyBuilder3 {
    kind: BodyType,
    position: Vec3,
    velocity: Vec3,
    radius: Real,
    mass: Real,
    restitution: Real,
}
impl BodyBuilder3 {
    /// Creates a dynamic body builder.
    #[must_use]
    pub fn dynamic() -> Self {
        Self {
            kind: BodyType::Dynamic,
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            radius: 0.5,
            mass: 1.0,
            restitution: 0.0,
        }
    }
    /// Sets position.
    #[must_use]
    pub fn position(mut self, v: Vec3) -> Self {
        self.position = v;
        self
    }
    /// Sets velocity.
    #[must_use]
    pub fn velocity(mut self, v: Vec3) -> Self {
        self.velocity = v;
        self
    }
    /// Sets radius.
    #[must_use]
    pub fn radius(mut self, v: Real) -> Self {
        self.radius = v;
        self
    }
    /// Sets mass.
    #[must_use]
    pub fn mass(mut self, v: Real) -> Self {
        self.mass = v;
        self
    }
    /// Sets restitution.
    #[must_use]
    pub fn restitution(mut self, v: Real) -> Self {
        self.restitution = v;
        self
    }
}

/// Snapshot of canonical 2D dynamic state.
#[derive(Clone, Debug)]
pub struct Snapshot2 {
    states: Vec<(u64, Vec2, Vec2, bool)>,
    step: u64,
}
/// Snapshot of canonical 3D dynamic state.
#[derive(Clone, Debug)]
pub struct Snapshot3 {
    states: Vec<(u64, Vec3, Vec3, bool)>,
    step: u64,
}
/// Native 2D simulation world with an infinite ground half-space at y=0.
pub struct World2 {
    gravity: Vec2,
    bodies: Pool<Body2>,
    next_id: u64,
    step: u64,
}
impl Default for World2 {
    fn default() -> Self {
        Self {
            gravity: Vec2 { x: 0.0, y: -9.81 },
            bodies: Pool::default(),
            next_id: 1,
            step: 0,
        }
    }
}
impl World2 {
    /// Sets finite gravity.
    pub fn set_gravity(&mut self, g: Vec2) -> Result<(), WorldError> {
        if !g.is_finite() {
            return Err(WorldError::InvalidInput);
        }
        self.gravity = g;
        Ok(())
    }
    /// Adds a validated body.
    pub fn add_body(&mut self, b: BodyBuilder2) -> Result<BodyHandle2, WorldError> {
        if !b.position.is_finite()
            || !b.velocity.is_finite()
            || b.radius <= 0.0
            || b.mass <= 0.0
            || !(0.0..=1.0).contains(&b.restitution)
        {
            return Err(WorldError::InvalidInput);
        }
        let id = StableId(self.next_id);
        self.next_id += 1;
        Ok(self.bodies.insert(Body2 {
            id,
            kind: b.kind,
            position: b.position,
            velocity: b.velocity,
            radius: b.radius,
            inv_mass: 1.0 / b.mass,
            restitution: b.restitution,
            sleeping: false,
        }))
    }
    /// Removes a body; stale handles fail.
    pub fn remove_body(&mut self, h: BodyHandle2) -> Result<Body2, WorldError> {
        self.bodies.remove(h).ok_or(WorldError::StaleHandle)
    }
    /// Gets a body.
    pub fn body(&self, h: BodyHandle2) -> Result<&Body2, WorldError> {
        self.bodies.get(h).ok_or(WorldError::StaleHandle)
    }
    /// Applies an impulse to a dynamic body.
    pub fn apply_impulse(&mut self, h: BodyHandle2, j: Vec2) -> Result<(), WorldError> {
        if !j.is_finite() {
            return Err(WorldError::InvalidInput);
        }
        let b = self.bodies.get_mut(h).ok_or(WorldError::StaleHandle)?;
        if b.kind == BodyType::Dynamic {
            b.velocity += j * b.inv_mass;
            b.sleeping = false;
        }
        Ok(())
    }
    /// Advances by a positive finite fixed time step. Ground contact uses bounded impulse/projection.
    pub fn step(&mut self, dt: Real) -> Result<(), WorldError> {
        if !(dt > 0.0 && dt.is_finite()) {
            return Err(WorldError::InvalidInput);
        }
        for (_, b) in self.bodies.iter_mut() {
            if b.kind == BodyType::Dynamic && !b.sleeping {
                b.velocity += self.gravity * dt;
                b.position += b.velocity * dt;
                let depth = b.radius - b.position.y;
                if depth > 0.0 {
                    b.position.y += depth;
                    if b.velocity.y < 0.0 {
                        b.velocity.y = -b.velocity.y * b.restitution;
                    }
                    if b.velocity.length_squared() < 1.0e-6 && depth <= CONTACT_SLOP + b.radius {
                        b.velocity = Vec2::ZERO;
                        b.sleeping = true;
                    }
                }
                if !b.position.is_finite() || !b.velocity.is_finite() {
                    return Err(WorldError::InvalidInput);
                }
            }
        }
        self.step += 1;
        Ok(())
    }
    /// Captures full dynamic state.
    #[must_use]
    pub fn snapshot(&self) -> Snapshot2 {
        Snapshot2 {
            states: self
                .bodies
                .iter()
                .map(|(_, b)| (b.id.0, b.position, b.velocity, b.sleeping))
                .collect(),
            step: self.step,
        }
    }
    /// Restores a snapshot when body identity sets match.
    pub fn restore(&mut self, s: &Snapshot2) -> Result<(), WorldError> {
        if s.states.len() != self.bodies.len() {
            return Err(WorldError::InvalidInput);
        }
        for (_, b) in self.bodies.iter_mut() {
            let (_, p, v, sleep) = s
                .states
                .iter()
                .find(|x| x.0 == b.id.0)
                .ok_or(WorldError::InvalidInput)?;
            b.position = *p;
            b.velocity = *v;
            b.sleeping = *sleep;
        }
        self.step = s.step;
        Ok(())
    }
    /// Canonical state hash for same-build replay.
    #[must_use]
    pub fn state_hash(&self) -> u64 {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&self.step.to_le_bytes());
        for (_, b) in self.bodies.iter() {
            bytes.extend_from_slice(&b.id.0.to_le_bytes());
            bytes.extend_from_slice(&b.position.x.to_bits().to_le_bytes());
            bytes.extend_from_slice(&b.position.y.to_bits().to_le_bytes());
            bytes.extend_from_slice(&b.velocity.x.to_bits().to_le_bytes());
            bytes.extend_from_slice(&b.velocity.y.to_bits().to_le_bytes());
            bytes.push(u8::from(b.sleeping));
        }
        hash_bytes(&bytes)
    }
}
/// Native 3D simulation world with an infinite ground half-space at y=0.
pub struct World3 {
    gravity: Vec3,
    bodies: Pool<Body3>,
    next_id: u64,
    step: u64,
}
impl Default for World3 {
    fn default() -> Self {
        Self {
            gravity: Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
            bodies: Pool::default(),
            next_id: 1,
            step: 0,
        }
    }
}
impl World3 {
    /// Adds a validated body.
    pub fn add_body(&mut self, b: BodyBuilder3) -> Result<BodyHandle3, WorldError> {
        if !b.position.is_finite()
            || !b.velocity.is_finite()
            || b.radius <= 0.0
            || b.mass <= 0.0
            || !(0.0..=1.0).contains(&b.restitution)
        {
            return Err(WorldError::InvalidInput);
        }
        let id = StableId(self.next_id);
        self.next_id += 1;
        Ok(self.bodies.insert(Body3 {
            id,
            kind: b.kind,
            position: b.position,
            velocity: b.velocity,
            radius: b.radius,
            inv_mass: 1.0 / b.mass,
            restitution: b.restitution,
            sleeping: false,
        }))
    }
    /// Gets a body.
    pub fn body(&self, h: BodyHandle3) -> Result<&Body3, WorldError> {
        self.bodies.get(h).ok_or(WorldError::StaleHandle)
    }
    /// Advances a positive finite fixed step.
    pub fn step(&mut self, dt: Real) -> Result<(), WorldError> {
        if !(dt > 0.0 && dt.is_finite()) {
            return Err(WorldError::InvalidInput);
        }
        for (_, b) in self.bodies.iter_mut() {
            if b.kind == BodyType::Dynamic && !b.sleeping {
                b.velocity += self.gravity * dt;
                b.position += b.velocity * dt;
                let d = b.radius - b.position.y;
                if d > 0.0 {
                    b.position.y += d;
                    if b.velocity.y < 0.0 {
                        b.velocity.y = -b.velocity.y * b.restitution;
                    }
                    if b.velocity.length_squared() < 1.0e-6 {
                        b.velocity = Vec3::ZERO;
                        b.sleeping = true;
                    }
                }
                if !b.position.is_finite() || !b.velocity.is_finite() {
                    return Err(WorldError::InvalidInput);
                }
            }
        }
        self.step += 1;
        Ok(())
    }
    /// Captures state.
    #[must_use]
    pub fn snapshot(&self) -> Snapshot3 {
        Snapshot3 {
            states: self
                .bodies
                .iter()
                .map(|(_, b)| (b.id.0, b.position, b.velocity, b.sleeping))
                .collect(),
            step: self.step,
        }
    }
    /// Restores state.
    pub fn restore(&mut self, s: &Snapshot3) -> Result<(), WorldError> {
        if s.states.len() != self.bodies.len() {
            return Err(WorldError::InvalidInput);
        }
        for (_, b) in self.bodies.iter_mut() {
            let (_, p, v, sl) = s
                .states
                .iter()
                .find(|x| x.0 == b.id.0)
                .ok_or(WorldError::InvalidInput)?;
            b.position = *p;
            b.velocity = *v;
            b.sleeping = *sl;
        }
        self.step = s.step;
        Ok(())
    }
    /// Canonical state hash.
    #[must_use]
    pub fn state_hash(&self) -> u64 {
        let mut x = Vec::new();
        x.extend_from_slice(&self.step.to_le_bytes());
        for (_, b) in self.bodies.iter() {
            x.extend_from_slice(&b.id.0.to_le_bytes());
            for v in [
                b.position.x,
                b.position.y,
                b.position.z,
                b.velocity.x,
                b.velocity.y,
                b.velocity.z,
            ] {
                x.extend_from_slice(&v.to_bits().to_le_bytes());
            }
            x.push(u8::from(b.sleeping));
        }
        hash_bytes(&x)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn falling_2d_rests() {
        let mut w = World2::default();
        let h = w
            .add_body(BodyBuilder2::dynamic().position(Vec2 { x: 0.0, y: 4.0 }))
            .unwrap();
        for _ in 0..600 {
            w.step(1.0 / 60.0).unwrap();
        }
        assert!((w.body(h).unwrap().position.y - 0.5).abs() < 1.0e-5);
        assert!(w.body(h).unwrap().sleeping);
    }
    #[test]
    fn rollback_replays_bitwise() {
        let mut w = World3::default();
        w.add_body(BodyBuilder3::dynamic().position(Vec3 {
            x: 1.0,
            y: 10.0,
            z: 2.0,
        }))
        .unwrap();
        for _ in 0..30 {
            w.step(1.0 / 60.0).unwrap();
        }
        let s = w.snapshot();
        for _ in 0..100 {
            w.step(1.0 / 60.0).unwrap();
        }
        let expected = w.state_hash();
        w.restore(&s).unwrap();
        for _ in 0..100 {
            w.step(1.0 / 60.0).unwrap();
        }
        assert_eq!(expected, w.state_hash());
    }
    #[test]
    fn invalid_dt_does_not_mutate() {
        let mut w = World2::default();
        let h = w.state_hash();
        assert_eq!(w.step(Real::NAN), Err(WorldError::InvalidInput));
        assert_eq!(h, w.state_hash());
    }
}
