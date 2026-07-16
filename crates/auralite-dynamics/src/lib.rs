//! Native dimension-separated rigid-body worlds with full rotation, solver, colliders, sleeping, and sensors.
#![forbid(unsafe_code)]
#![allow(
    missing_docs,
    clippy::too_many_arguments,
    clippy::type_complexity,
    clippy::collapsible_if,
    clippy::redundant_closure_call,
    clippy::field_reassign_with_default
)]
#[cfg(not(any(feature = "multithread", feature = "single-thread")))]
compile_error!("auralite-dynamics requires either the 'multithread' or 'single-thread' feature");

pub mod joints;

use auralite_collision::{
    BroadPhase2, CollisionFilter, DynamicTree2, FeatureId, Manifold2, PairDecision,
};
use auralite_core::{Handle, Pool, StableId, hash_bytes};
use auralite_geometry::{
    Box2, Box3, Capsule2, Capsule3, Circle2, ConvexHull3, ConvexPolygon, Edge2, Edge3, Sphere3,
    TriangleMesh,
};
use auralite_math::{ABS_EPSILON, CONTACT_SLOP, Real, Rot2, Vec2};
use auralite_math::{Quat, Vec3};
pub use joints::{
    Joint2, JointBreakEvent, JointConfig2, JointId, JointLimits, JointMotor, JointType2,
};
use std::collections::VecDeque;

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
    /// A body or shape operation failed.
    Internal,
}

/// Material properties for a collider.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material {
    /// Coefficient of restitution [0, 1].
    pub restitution: Real,
    /// Coefficient of friction (Coulomb).
    pub friction: Real,
    /// Density for mass calculation.
    pub density: Real,
}
impl Default for Material {
    fn default() -> Self {
        Self {
            restitution: 0.0,
            friction: 0.5,
            density: 1.0,
        }
    }
}

/// How two materials combine.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum CombineMode {
    /// Multiply the two values.
    Multiply,
    /// Average the two values.
    #[default]
    Average,
    /// Take the minimum.
    Min,
    /// Take the maximum.
    Max,
    /// Use the first body's value.
    First,
}
/// Combine two material values per the mode.
pub fn combine(a: Real, b: Real, mode: CombineMode) -> Real {
    match mode {
        CombineMode::Multiply => a * b,
        CombineMode::Average => (a + b) * 0.5,
        CombineMode::Min => a.min(b),
        CombineMode::Max => a.max(b),
        CombineMode::First => a,
    }
}

// ─── Collider2 ───────────────────────────────────────────────────────────────

/// A 2D collider: a shape + material + local transform relative to the body.
#[derive(Clone, Debug, PartialEq)]
pub struct Collider2 {
    /// Shape discriminant and data.
    pub shape: ColliderShape2,
    /// Local offset from body center.
    pub offset: Vec2,
    /// Material properties.
    pub material: Material,
    /// Collision filter.
    pub filter: CollisionFilter,
}

/// 2D collider shape variants.
#[derive(Clone, Debug, PartialEq)]
pub enum ColliderShape2 {
    /// Circle.
    Circle(Circle2),
    /// Box (axis-aligned in local space).
    Box(Box2),
    /// Capsule aligned with local Y axis.
    Capsule(Capsule2),
    /// Convex polygon.
    ConvexPolygon(ConvexPolygon),
    /// Edge segment (zero area).
    Edge(Edge2),
}

impl Collider2 {
    /// Compute the AABB in world space given the body transform and this collider's local offset.
    #[must_use]
    pub fn world_aabb(&self, body_pos: Vec2, body_rot: Rot2) -> auralite_math::Aabb2 {
        // Compose transforms: world position = body_pos + body_rot.rotate(offset)
        let world_center = body_pos + body_rot.rotate(self.offset);
        let r = self.bounding_radius();
        let h = Vec2 { x: r, y: r };
        auralite_math::Aabb2::new(world_center - h, world_center + h)
            .unwrap_or(auralite_math::Aabb2::new(world_center, world_center).unwrap())
    }
    /// Approximate bounding radius about body origin for broad phase.
    #[must_use]
    pub fn bounding_radius(&self) -> Real {
        let local_r = match &self.shape {
            ColliderShape2::Circle(c) => c.radius(),
            ColliderShape2::Box(b) => b.half_extents().length(),
            ColliderShape2::Capsule(cap) => cap.bounding_radius(),
            ColliderShape2::ConvexPolygon(poly) => poly.bounding_radius(),
            ColliderShape2::Edge(e) => e.bounding_radius(),
        };
        local_r + self.offset.length()
    }
    /// Local-space center of mass offset for this collider (density-weighted).
    #[must_use]
    pub fn local_center(&self) -> Vec2 {
        self.offset
    }
}

// ─── Collider3 ───────────────────────────────────────────────────────────────

/// A 3D collider.
#[derive(Clone, Debug, PartialEq)]
pub struct Collider3 {
    /// Shape.
    pub shape: ColliderShape3,
    /// Local offset.
    pub offset: Vec3,
    /// Material.
    pub material: Material,
    /// Collision filter.
    pub filter: CollisionFilter,
}

/// 3D collider shape variants.
#[derive(Clone, Debug, PartialEq)]
pub enum ColliderShape3 {
    /// Sphere.
    Sphere(Sphere3),
    /// Box (axis-aligned in local space).
    Box(Box3),
    /// Capsule along local Y.
    Capsule(Capsule3),
    /// Convex hull.
    ConvexHull(ConvexHull3),
    /// Triangle mesh (static-only).
    TriangleMesh(TriangleMesh),
    /// Edge segment.
    Edge(Edge3),
}

impl Collider3 {
    /// World AABB.
    #[must_use]
    pub fn world_aabb(&self, body_pos: Vec3, body_rot: Quat) -> auralite_math::Aabb3 {
        let world_center = body_pos + body_rot.rotate(self.offset);
        let r = self.bounding_radius();
        let h = Vec3 { x: r, y: r, z: r };
        auralite_math::Aabb3::new(world_center - h, world_center + h)
            .unwrap_or(auralite_math::Aabb3::new(world_center, world_center).unwrap())
    }
    /// Bounding radius.
    #[must_use]
    pub fn bounding_radius(&self) -> Real {
        let local_r = match &self.shape {
            ColliderShape3::Sphere(s) => s.radius(),
            ColliderShape3::Box(b) => b.half_extents().length(),
            ColliderShape3::Capsule(cap) => cap.bounding_radius(),
            ColliderShape3::ConvexHull(hull) => hull.bounding_radius(),
            ColliderShape3::TriangleMesh(mesh) => mesh.bounding_radius(),
            ColliderShape3::Edge(e) => e.bounding_radius(),
        };
        local_r + self.offset.length()
    }
}

// ─── Body2 ────────────────────────────────────────────────────────────────────

/// Native 2D rigid body with rotation, inertia, and colliders.
#[derive(Clone, Debug, PartialEq)]
pub struct Body2 {
    /// Stable deterministic identity.
    pub id: StableId,
    /// Motion type.
    pub kind: BodyType,
    /// Center position.
    pub position: Vec2,
    /// Rotation.
    pub rotation: Rot2,
    /// Linear velocity.
    pub velocity: Vec2,
    /// Angular velocity (scalar in 2D).
    pub angular_velocity: Real,
    /// Inverse mass (zero for static/kinematic).
    pub inv_mass: Real,
    /// Inverse inertia (zero for static/kinematic).
    pub inv_inertia: Real,
    /// Colliders attached to this body.
    pub colliders: Vec<Collider2>,
    /// Coefficient of restitution (override).
    pub restitution: Real,
    /// Coefficient of friction (override).
    pub friction: Real,
    /// Whether sleeping.
    pub sleeping: bool,
    /// Accumulated force.
    pub force: Vec2,
    /// Accumulated torque.
    pub torque: Real,
    /// Force/torque damping factor per step (0..1).
    pub linear_damping: Real,
    pub angular_damping: Real,
    /// User data.
    pub user_data: u64,
}

impl Body2 {
    /// World-space AABB encompassing all colliders.
    #[must_use]
    pub fn world_aabb(&self) -> auralite_math::Aabb2 {
        if self.colliders.is_empty() {
            return auralite_math::Aabb2::new(self.position, self.position).unwrap();
        }
        let mut min = Vec2 {
            x: Real::INFINITY,
            y: Real::INFINITY,
        };
        let mut max = Vec2 {
            x: Real::NEG_INFINITY,
            y: Real::NEG_INFINITY,
        };
        for c in &self.colliders {
            let a = c.world_aabb(self.position, self.rotation);
            min = Vec2 {
                x: min.x.min(a.min.x),
                y: min.y.min(a.min.y),
            };
            max = Vec2 {
                x: max.x.max(a.max.x),
                y: max.y.max(a.max.y),
            };
        }
        auralite_math::Aabb2::new(min, max).unwrap()
    }
    /// Total inverse mass (0 for static/kinematic).
    #[must_use]
    pub fn effective_inv_mass(&self) -> Real {
        if self.kind == BodyType::Dynamic {
            self.inv_mass
        } else {
            0.0
        }
    }
    #[must_use]
    pub fn effective_inv_inertia(&self) -> Real {
        if self.kind == BodyType::Dynamic {
            self.inv_inertia
        } else {
            0.0
        }
    }
    /// Apply an impulse at a world-space point.
    pub fn apply_impulse(&mut self, impulse: Vec2, point: Vec2) {
        if self.kind != BodyType::Dynamic || self.sleeping {
            return;
        }
        let im = self.effective_inv_mass();
        self.velocity += impulse * im;
        let r = point - self.position;
        self.angular_velocity += r.cross(impulse) * self.effective_inv_inertia();
    }
}

// ─── Body3 ────────────────────────────────────────────────────────────────────

/// Native 3D rigid body with rotation, inertia, and colliders.
#[derive(Clone, Debug)]
pub struct Body3 {
    /// Stable identity.
    pub id: StableId,
    /// Motion type.
    pub kind: BodyType,
    /// Position.
    pub position: Vec3,
    /// Orientation (unit quaternion).
    pub rotation: Quat,
    /// Linear velocity.
    pub velocity: Vec3,
    /// Angular velocity.
    pub angular_velocity: Vec3,
    /// Inverse mass.
    pub inv_mass: Real,
    /// Inverse inertia tensor in world space (diagonal for principal axes).
    pub inv_inertia_diagonal: Vec3,
    /// Colliders.
    pub colliders: Vec<Collider3>,
    /// Restitution override.
    pub restitution: Real,
    /// Friction override.
    pub friction: Real,
    /// Whether sleeping.
    pub sleeping: bool,
    /// Accumulated force.
    pub force: Vec3,
    /// Accumulated torque.
    pub torque: Vec3,
    /// Damping.
    pub linear_damping: Real,
    pub angular_damping: Real,
    /// User data.
    pub user_data: u64,
}

impl Body3 {
    /// World AABB.
    #[must_use]
    pub fn world_aabb(&self) -> auralite_math::Aabb3 {
        if self.colliders.is_empty() {
            return auralite_math::Aabb3::new(self.position, self.position).unwrap();
        }
        let mut min = Vec3 {
            x: Real::INFINITY,
            y: Real::INFINITY,
            z: Real::INFINITY,
        };
        let mut max = Vec3 {
            x: Real::NEG_INFINITY,
            y: Real::NEG_INFINITY,
            z: Real::NEG_INFINITY,
        };
        for c in &self.colliders {
            let a = c.world_aabb(self.position, self.rotation);
            min = Vec3 {
                x: min.x.min(a.min.x),
                y: min.y.min(a.min.y),
                z: min.z.min(a.min.z),
            };
            max = Vec3 {
                x: max.x.max(a.max.x),
                y: max.y.max(a.max.y),
                z: max.z.max(a.max.z),
            };
        }
        auralite_math::Aabb3::new(min, max).unwrap()
    }
    #[must_use]
    pub fn effective_inv_mass(&self) -> Real {
        if self.kind == BodyType::Dynamic {
            self.inv_mass
        } else {
            0.0
        }
    }
    pub fn apply_impulse(&mut self, impulse: Vec3, point: Vec3) {
        if self.kind != BodyType::Dynamic || self.sleeping {
            return;
        }
        let im = self.effective_inv_mass();
        self.velocity += impulse * im;
        let r = point - self.position;
        // Angular impulse = I^{-1} * (r × F)
        let torque_impulse = r.cross(impulse);
        self.angular_velocity += Vec3 {
            x: torque_impulse.x * self.inv_inertia_diagonal.x,
            y: torque_impulse.y * self.inv_inertia_diagonal.y,
            z: torque_impulse.z * self.inv_inertia_diagonal.z,
        };
    }
}

// ─── Handles ──────────────────────────────────────────────────────────────────

pub type BodyHandle2 = Handle<Body2>;
pub type BodyHandle3 = Handle<Body3>;
pub type ColliderHandle2 = Handle<Collider2>;
pub type ColliderHandle3 = Handle<Collider3>;

// ─── Builders ─────────────────────────────────────────────────────────────────

/// 2D body builder.
pub struct BodyBuilder2 {
    kind: BodyType,
    position: Vec2,
    rotation: Rot2,
    velocity: Vec2,
    angular_velocity: Real,
    mass: Real,
    inertia: Option<Real>,
    colliders: Vec<Collider2>,
    restitution: Real,
    friction: Real,
    linear_damping: Real,
    angular_damping: Real,
    user_data: u64,
}
impl BodyBuilder2 {
    pub fn new() -> Self {
        Self::dynamic()
    }
    pub fn dynamic() -> Self {
        Self {
            kind: BodyType::Dynamic,
            position: Vec2::ZERO,
            rotation: Rot2::identity(),
            velocity: Vec2::ZERO,
            angular_velocity: 0.0,
            mass: 1.0,
            inertia: None,
            colliders: Vec::new(),
            restitution: 0.0,
            friction: 0.5,
            linear_damping: 0.0,
            angular_damping: 0.0,
            user_data: 0,
        }
    }
    pub fn static_body() -> Self {
        Self {
            kind: BodyType::Static,
            ..Self::dynamic()
        }
    }
    pub fn kinematic() -> Self {
        Self {
            kind: BodyType::Kinematic,
            ..Self::dynamic()
        }
    }
    pub fn position(mut self, v: Vec2) -> Self {
        self.position = v;
        self
    }
    pub fn rotation(mut self, r: Rot2) -> Self {
        self.rotation = r;
        self
    }
    pub fn velocity(mut self, v: Vec2) -> Self {
        self.velocity = v;
        self
    }
    pub fn angular_velocity(mut self, v: Real) -> Self {
        self.angular_velocity = v;
        self
    }
    pub fn mass(mut self, v: Real) -> Self {
        self.mass = v;
        self
    }
    pub fn inertia(mut self, i: Real) -> Self {
        self.inertia = Some(i);
        self
    }
    pub fn restitution(mut self, v: Real) -> Self {
        self.restitution = v;
        self
    }
    pub fn friction(mut self, v: Real) -> Self {
        self.friction = v;
        self
    }
    pub fn linear_damping(mut self, v: Real) -> Self {
        self.linear_damping = v;
        self
    }
    pub fn angular_damping(mut self, v: Real) -> Self {
        self.angular_damping = v;
        self
    }
    pub fn user_data(mut self, v: u64) -> Self {
        self.user_data = v;
        self
    }
    pub fn add_collider(mut self, c: Collider2) -> Self {
        self.colliders.push(c);
        self
    }
}
impl Default for BodyBuilder2 {
    fn default() -> Self {
        Self::new()
    }
}

/// 3D body builder.
pub struct BodyBuilder3 {
    kind: BodyType,
    position: Vec3,
    rotation: Quat,
    velocity: Vec3,
    angular_velocity: Vec3,
    mass: Real,
    inertia_diagonal: Option<Vec3>,
    colliders: Vec<Collider3>,
    restitution: Real,
    friction: Real,
    linear_damping: Real,
    angular_damping: Real,
    user_data: u64,
}
impl BodyBuilder3 {
    pub fn new() -> Self {
        Self::dynamic()
    }
    pub fn dynamic() -> Self {
        Self {
            kind: BodyType::Dynamic,
            position: Vec3::ZERO,
            rotation: Quat::identity(),
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            mass: 1.0,
            inertia_diagonal: None,
            colliders: Vec::new(),
            restitution: 0.0,
            friction: 0.5,
            linear_damping: 0.0,
            angular_damping: 0.0,
            user_data: 0,
        }
    }
    pub fn static_body() -> Self {
        Self {
            kind: BodyType::Static,
            ..Self::dynamic()
        }
    }
    pub fn kinematic() -> Self {
        Self {
            kind: BodyType::Kinematic,
            ..Self::dynamic()
        }
    }
    pub fn position(mut self, v: Vec3) -> Self {
        self.position = v;
        self
    }
    pub fn rotation(mut self, r: Quat) -> Self {
        self.rotation = r;
        self
    }
    pub fn velocity(mut self, v: Vec3) -> Self {
        self.velocity = v;
        self
    }
    pub fn angular_velocity(mut self, v: Vec3) -> Self {
        self.angular_velocity = v;
        self
    }
    pub fn mass(mut self, v: Real) -> Self {
        self.mass = v;
        self
    }
    pub fn inertia_diagonal(mut self, i: Vec3) -> Self {
        self.inertia_diagonal = Some(i);
        self
    }
    pub fn restitution(mut self, v: Real) -> Self {
        self.restitution = v;
        self
    }
    pub fn friction(mut self, v: Real) -> Self {
        self.friction = v;
        self
    }
    pub fn linear_damping(mut self, v: Real) -> Self {
        self.linear_damping = v;
        self
    }
    pub fn angular_damping(mut self, v: Real) -> Self {
        self.angular_damping = v;
        self
    }
    pub fn user_data(mut self, v: u64) -> Self {
        self.user_data = v;
        self
    }
    pub fn add_collider(mut self, c: Collider3) -> Self {
        self.colliders.push(c);
        self
    }
}
impl Default for BodyBuilder3 {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Contact Constraints ──────────────────────────────────────────────────────

/// A contact constraint ready for the solver.
#[derive(Clone, Debug)]
pub struct ContactConstraint2 {
    pub body_a: BodyHandle2,
    pub body_b: BodyHandle2,
    pub normal: Vec2,      // A-to-B normal
    pub tangent: Vec2,     // friction direction (perpendicular to normal)
    pub point: Vec2,       // world-space contact point
    pub penetration: Real, // non-negative
    pub restitution: Real,
    pub friction: Real,
    pub normal_impulse: Real, // accumulated (warm start)
    pub tangent_impulse: Real,
    pub feature_id: FeatureId,
    /// Effective mass and bias for the solver.
    pub effective_mass_normal: Real,
    pub effective_mass_tangent: Real,
    pub bias: Real, // penetration correction velocity
}

impl ContactConstraint2 {
    fn new(
        body_a: BodyHandle2,
        body_b: BodyHandle2,
        normal: Vec2,
        point: Vec2,
        penetration: Real,
        restitution: Real,
        friction: Real,
        feature_id: FeatureId,
        w2: &World2,
    ) -> Self {
        let tangent = Vec2 {
            x: -normal.y,
            y: normal.x,
        };
        // Compute effective mass
        let (ia, ii_a) = get_inv2(w2, body_a);
        let (ib, ii_b) = get_inv2(w2, body_b);
        let ra = point - body_pos2(w2, body_a);
        let rb = point - body_pos2(w2, body_b);

        let mn = compute_effective_mass_2d(ia, ii_a, ib, ii_b, ra, rb, normal);
        let mt = compute_effective_mass_2d(ia, ii_a, ib, ii_b, ra, rb, tangent);

        // Baumgarte penetration bias
        let bias = if penetration > CONTACT_SLOP {
            let dt: Real = 0.016666668; // 1/60
            (penetration - CONTACT_SLOP) * 0.2 / dt.max(ABS_EPSILON)
        } else {
            0.0
        };

        Self {
            body_a,
            body_b,
            normal,
            tangent,
            point,
            penetration,
            restitution,
            friction,
            normal_impulse: 0.0,
            tangent_impulse: 0.0,
            feature_id,
            effective_mass_normal: mn,
            effective_mass_tangent: mt,
            bias,
        }
    }
    /// Apply warm-start impulses from manifold.
    fn warm_start(&mut self, manifold: &Manifold2) {
        for mp in &manifold.points {
            if mp.feature == self.feature_id {
                self.normal_impulse = mp.normal_impulse;
                self.tangent_impulse = mp.tangent_impulse;
                break;
            }
        }
    }
}

fn get_inv2(w2: &World2, h: BodyHandle2) -> (Real, Real) {
    w2.bodies.get(h).map_or((0.0, 0.0), |b| {
        (b.effective_inv_mass(), b.effective_inv_inertia())
    })
}
fn body_pos2(w2: &World2, h: BodyHandle2) -> Vec2 {
    w2.bodies.get(h).map_or(Vec2::ZERO, |b| b.position)
}

fn compute_effective_mass_2d(
    ia: Real,
    ii_a: Real,
    ib: Real,
    ii_b: Real,
    ra: Vec2,
    rb: Vec2,
    n: Vec2,
) -> Real {
    let ra_cross_n = ra.cross(n);
    let rb_cross_n = rb.cross(n);
    let sum = ia + ib + ii_a * ra_cross_n * ra_cross_n + ii_b * rb_cross_n * rb_cross_n;
    if sum > ABS_EPSILON { 1.0 / sum } else { 0.0 }
}

/// 3D contact constraint.
#[derive(Clone, Debug)]
#[expect(dead_code)]
pub struct ContactConstraint3 {
    pub body_a: BodyHandle3,
    pub body_b: BodyHandle3,
    pub normal: Vec3,
    pub point: Vec3,
    pub penetration: Real,
    pub restitution: Real,
    pub friction: Real,
    pub normal_impulse: Real,
    pub tangent_impulse_u: Real,
    pub tangent_impulse_v: Real,
    pub feature_id: FeatureId,
    pub effective_mass_normal: Real,
    pub bias: Real,
    tangent1: Vec3,
    tangent2: Vec3,
    effective_mass_tangent_u: Real,
    effective_mass_tangent_v: Real,
}

// ─── Solver ───────────────────────────────────────────────────────────────────

/// Run one step of the sequential impulse solver for 2D contacts.
fn solve_contacts_2d(constraints: &mut [ContactConstraint2], bodies: &mut Pool<Body2>) {
    for _iteration in 0..10 {
        let mut max_pen: Real = 0.0;
        for c in constraints.iter_mut() {
            let (va, wa) = get_vel2(bodies, c.body_a);
            let (vb, wb) = get_vel2(bodies, c.body_b);
            let ra = c.point - body_pos_from_pool(bodies, c.body_a);
            let rb = c.point - body_pos_from_pool(bodies, c.body_b);

            // Relative velocity at contact
            let dv = (vb + perp(rb) * wb) - (va + perp(ra) * wa);

            // Normal impulse
            let dvn = dv.dot(c.normal);
            let dn = -dvn + (-dvn).min(0.0) * c.restitution + c.bias;
            let lambda_n = dn * c.effective_mass_normal;
            let new_impulse = (c.normal_impulse + lambda_n).max(0.0);
            let delta_n = new_impulse - c.normal_impulse;
            c.normal_impulse = new_impulse;

            // Apply normal impulse
            apply_impulse2(bodies, c.body_a, c.body_b, c.normal * delta_n, c.point);

            // Friction (tangent)
            let (va2, wa2) = get_vel2(bodies, c.body_a);
            let (vb2, wb2) = get_vel2(bodies, c.body_b);
            let ra2 = c.point - body_pos_from_pool(bodies, c.body_a);
            let rb2 = c.point - body_pos_from_pool(bodies, c.body_b);
            let dv2 = (vb2 + perp(rb2) * wb2) - (va2 + perp(ra2) * wa2);
            let dvt = dv2.dot(c.tangent);
            let max_friction = c.friction * c.normal_impulse.max(0.0);
            let lambda_t = -dvt * c.effective_mass_tangent;
            let new_tangent = (c.tangent_impulse + lambda_t).clamp(-max_friction, max_friction);
            let delta_t = new_tangent - c.tangent_impulse;
            c.tangent_impulse = new_tangent;

            apply_impulse2(bodies, c.body_a, c.body_b, c.tangent * delta_t, c.point);

            max_pen = max_pen.max(c.penetration);
        }
        if max_pen < CONTACT_SLOP * 2.0 {
            break;
        }
    }
}

fn perp(v: Vec2) -> Vec2 {
    Vec2 { x: -v.y, y: v.x }
}

fn get_vel2(pool: &Pool<Body2>, h: BodyHandle2) -> (Vec2, Real) {
    pool.get(h)
        .map_or((Vec2::ZERO, 0.0), |b| (b.velocity, b.angular_velocity))
}

fn body_pos_from_pool(pool: &Pool<Body2>, h: BodyHandle2) -> Vec2 {
    pool.get(h).map_or(Vec2::ZERO, |b| b.position)
}

pub(crate) fn apply_impulse2(
    pool: &mut Pool<Body2>,
    ha: BodyHandle2,
    hb: BodyHandle2,
    impulse: Vec2,
    point: Vec2,
) {
    if let Some(ba) = pool.get_mut(ha) {
        if ba.kind == BodyType::Dynamic && !ba.sleeping {
            let im = ba.effective_inv_mass();
            ba.velocity += impulse * im;
            let r = point - ba.position;
            ba.angular_velocity += r.cross(impulse) * ba.effective_inv_inertia();
        }
    }
    if let Some(bb) = pool.get_mut(hb) {
        if bb.kind == BodyType::Dynamic && !bb.sleeping {
            let im = bb.effective_inv_mass();
            bb.velocity -= impulse * im;
            let r = point - bb.position;
            bb.angular_velocity -= r.cross(impulse) * bb.effective_inv_inertia();
        }
    }
}

// ─── Snapshot ─────────────────────────────────────────────────────────────────

#[derive(Clone, Debug)]
pub struct Snapshot2 {
    states: Vec<(u64, Vec2, Rot2, Vec2, Real, Real, Real, bool)>,
    step: u64,
}
#[derive(Clone, Debug)]
pub struct Snapshot3 {
    states: Vec<(u64, Vec3, Quat, Vec3, Vec3, Real, Vec3, bool)>,
    step: u64,
}

// ─── Sensor events ────────────────────────────────────────────────────────────

/// A single sensor event.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorEvent {
    /// Sensor body handle index.
    pub sensor: u64,
    /// Other body handle index.
    pub other: u64,
    /// Whether this is a begin (true) or end (false) event.
    pub began: bool,
}

// ─── World2 ───────────────────────────────────────────────────────────────────

/// Native 2D simulation world with gravity, broad/narrow phase, solver, sleeping, and sensors.
pub struct World2 {
    gravity: Vec2,
    bodies: Pool<Body2>,
    next_id: u64,
    step: u64,
    broad_phase: BroadPhase2,
    dynamic_tree: DynamicTree2,
    pub solver_iterations: u16,
    pub sleep_threshold: Real,
    pub restitution_mode: CombineMode,
    pub friction_mode: CombineMode,
    prev_manifolds: Vec<Manifold2>,
    prev_sensor_pairs: Vec<(u64, u64)>,
    pub sensor_events: VecDeque<SensorEvent>,
    pub joints: Vec<Joint2>,
    pub joint_break_events: VecDeque<JointBreakEvent>,
}

impl Default for World2 {
    fn default() -> Self {
        Self {
            gravity: Vec2 { x: 0.0, y: -9.81 },
            bodies: Pool::default(),
            next_id: 1,
            step: 0,
            broad_phase: BroadPhase2::default(),
            dynamic_tree: DynamicTree2::new(0.02, 1.0 / 60.0).unwrap(),
            solver_iterations: 10,
            sleep_threshold: 1.0e-6,
            restitution_mode: CombineMode::Average,
            friction_mode: CombineMode::Average,
            prev_manifolds: Vec::new(),
            prev_sensor_pairs: Vec::new(),
            sensor_events: VecDeque::new(),
            joints: Vec::new(),
            joint_break_events: VecDeque::new(),
        }
    }
}

impl World2 {
    pub fn set_gravity(&mut self, g: Vec2) -> Result<(), WorldError> {
        if !g.is_finite() {
            return Err(WorldError::InvalidInput);
        }
        self.gravity = g;
        Ok(())
    }
    /// Add a body with its colliders.
    pub fn add_body(&mut self, b: BodyBuilder2) -> Result<BodyHandle2, WorldError> {
        if !b.position.is_finite()
            || !b.velocity.is_finite()
            || b.mass <= 0.0
            || !b.restitution.is_finite()
            || !b.friction.is_finite()
        {
            return Err(WorldError::InvalidInput);
        }
        // Compute auto inertia from colliders if not specified
        let inertia = b.inertia.unwrap_or_else(|| {
            let mut i = 0.0;
            for c in &b.colliders {
                let m = c.material.density * b.mass;
                let r = c.offset.length();
                i += m * r * r;
            }
            if i <= ABS_EPSILON { b.mass * 0.1 } else { i }
        });
        let id = StableId(self.next_id);
        self.next_id += 1;
        let h = self.bodies.insert(Body2 {
            id,
            kind: b.kind,
            position: b.position,
            rotation: b.rotation,
            velocity: b.velocity,
            angular_velocity: b.angular_velocity,
            inv_mass: if b.kind == BodyType::Dynamic {
                1.0 / b.mass
            } else {
                0.0
            },
            inv_inertia: if b.kind == BodyType::Dynamic {
                1.0 / inertia
            } else {
                0.0
            },
            colliders: b.colliders,
            restitution: b.restitution,
            friction: b.friction,
            sleeping: false,
            force: Vec2::ZERO,
            torque: 0.0,
            linear_damping: b.linear_damping,
            angular_damping: b.angular_damping,
            user_data: b.user_data,
        });
        Ok(h)
    }
    pub fn remove_body(&mut self, h: BodyHandle2) -> Result<Body2, WorldError> {
        self.bodies.remove(h).ok_or(WorldError::StaleHandle)
    }
    pub fn body(&self, h: BodyHandle2) -> Result<&Body2, WorldError> {
        self.bodies.get(h).ok_or(WorldError::StaleHandle)
    }
    pub fn body_mut(&mut self, h: BodyHandle2) -> Result<&mut Body2, WorldError> {
        self.bodies.get_mut(h).ok_or(WorldError::StaleHandle)
    }
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
    pub fn apply_force(&mut self, h: BodyHandle2, f: Vec2) -> Result<(), WorldError> {
        if !f.is_finite() {
            return Err(WorldError::InvalidInput);
        }
        let b = self.bodies.get_mut(h).ok_or(WorldError::StaleHandle)?;
        if b.kind == BodyType::Dynamic {
            b.force += f;
            b.sleeping = false;
        }
        Ok(())
    }
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    /// Add a joint between two bodies.
    pub fn add_joint(&mut self, config: JointConfig2) -> Result<JointId, WorldError> {
        if self.bodies.get(config.body_a).is_none() || self.bodies.get(config.body_b).is_none() {
            return Err(WorldError::StaleHandle);
        }
        let id = JointId(self.next_id);
        self.next_id += 1;
        self.joints.push(Joint2::new(config));
        Ok(id)
    }

    pub fn joint(&self, _id: JointId) -> Option<&Joint2> {
        self.joints.iter().find(|j| !j.broken)
    }
    pub fn joint_mut(&mut self, _id: JointId) -> Option<&mut Joint2> {
        self.joints.iter_mut().find(|j| !j.broken)
    }
    pub fn remove_joint(&mut self, _id: JointId) {
        self.joints.retain(|j| j.broken);
    }

    /// Step simulation.
    pub fn step(&mut self, dt: Real) -> Result<(), WorldError> {
        if !(dt > 0.0 && dt.is_finite()) {
            return Err(WorldError::InvalidInput);
        }

        let body_handles: Vec<BodyHandle2> = self.bodies.iter().map(|(h, _)| h).collect();

        // ── 1. Integrate velocities (apply gravity + forces) ──
        for &h in &body_handles {
            if let Some(b) = self.bodies.get_mut(h) {
                if b.kind == BodyType::Dynamic && !b.sleeping {
                    b.velocity += (self.gravity + b.force * b.inv_mass) * dt;
                    b.angular_velocity += b.torque * b.inv_inertia * dt;
                    let ld: Real = (1.0 - b.linear_damping).max(0.0);
                    let ad: Real = (1.0 - b.angular_damping).max(0.0);
                    #[allow(clippy::assign_op_pattern)]
                    {
                        b.velocity = b.velocity * ld;
                    }
                    #[allow(clippy::assign_op_pattern)]
                    {
                        b.angular_velocity = b.angular_velocity * ad;
                    }
                    b.force = Vec2::ZERO;
                    b.torque = 0.0;
                }
            }
        }

        // ── 2. Integrate positions ──
        for &h in &body_handles {
            if let Some(b) = self.bodies.get_mut(h) {
                if b.kind == BodyType::Dynamic && !b.sleeping {
                    b.position += b.velocity * dt;
                    b.rotation = auralite_math::Rot2::from_radians(
                        rot2_angle(b.rotation) + b.angular_velocity * dt,
                    )
                    .unwrap_or(b.rotation);
                    if !b.position.is_finite() || !b.velocity.is_finite() {
                        return Err(WorldError::InvalidInput);
                    }
                }
            }
        }

        // ── 3. Implicit ground contact (infinite half-space at y=0) ──
        // Project bodies out of ground and reflect velocity with restitution.
        // Only applied if collider filters would accept collision with the ground (layer 1, mask 1).
        for &h in &body_handles {
            if let Some(b) = self.bodies.get_mut(h) {
                if b.kind != BodyType::Dynamic || b.sleeping {
                    continue;
                }
                let mut any_collides_ground = false;
                for c in &b.colliders {
                    let ground_filter = CollisionFilter {
                        layers: 1,
                        mask: 1,
                        group: 0,
                        sensor: false,
                    };
                    any_collides_ground = c.filter.decide(ground_filter) != PairDecision::Ignore;
                    if any_collides_ground {
                        break;
                    }
                }
                if !any_collides_ground {
                    continue;
                }
                for c in &b.colliders {
                    let r = match &c.shape {
                        ColliderShape2::Circle(circ) => circ.radius(),
                        ColliderShape2::Box(bx) => bx.half_extents().y,
                        ColliderShape2::Capsule(cap) => cap.radius + cap.half_height,
                        ColliderShape2::ConvexPolygon(_) => 0.0,
                        ColliderShape2::Edge(_) => 0.0,
                    };
                    let world_y = b.position.y + c.offset.y;
                    let penetration = r - world_y;
                    if penetration > 0.0 && r > ABS_EPSILON {
                        b.position.y += penetration;
                        if b.velocity.y < 0.0 {
                            let combined_rest = combine(b.restitution, 0.0, self.restitution_mode);
                            b.velocity.y = -b.velocity.y * combined_rest;
                        }
                        if b.velocity.length_squared() < self.sleep_threshold
                            && penetration <= CONTACT_SLOP + r
                        {
                            b.velocity = Vec2::ZERO;
                            b.angular_velocity = 0.0;
                            b.sleeping = true;
                        }
                    }
                }
            }
        }

        // ── 4. Update broad phase ──
        self.broad_phase = BroadPhase2::default();
        self.dynamic_tree = DynamicTree2::new(0.02, dt).unwrap();
        for &h in &body_handles {
            if let Some(b) = self.bodies.get(h) {
                let aabb = b.world_aabb();
                let id = b.id.0;
                self.broad_phase.update(id, aabb);
                self.dynamic_tree.update(id, aabb, b.velocity);
            }
        }

        // ── 4. Broad phase pairs detection ──
        let pairs = self.dynamic_tree.pairs();

        // ── 5. Build contacts from pairs ──
        let mut constraints: Vec<ContactConstraint2> = Vec::new();
        let mut current_sensor_pairs: Vec<(u64, u64)> = Vec::new();
        // Build body id -> handle mapping
        let mut id_to_handle: Vec<(u64, BodyHandle2)> = body_handles
            .iter()
            .filter_map(|&h| self.bodies.get(h).map(|b| (b.id.0, h)))
            .collect();
        id_to_handle.sort_by_key(|x| x.0);

        for &(ida, idb) in &pairs {
            // Find handles
            let ha = id_to_handle.iter().find(|x| x.0 == ida).map(|x| x.1);
            let hb = id_to_handle.iter().find(|x| x.0 == idb).map(|x| x.1);
            let (ha, hb) = match (ha, hb) {
                (Some(a), Some(b)) => (a, b),
                _ => continue,
            };

            let filter_pair = |a: &Body2, b: &Body2| -> PairDecision {
                for ca in &a.colliders {
                    for cb in &b.colliders {
                        let d = ca.filter.decide(cb.filter);
                        if d != PairDecision::Ignore {
                            return d;
                        }
                    }
                }
                PairDecision::Ignore
            };

            let (body_a, body_b) = match (self.bodies.get(ha), self.bodies.get(hb)) {
                (Some(a), Some(b)) => (a, b),
                _ => continue,
            };
            if body_a.sleeping && body_b.sleeping {
                continue;
            }

            let decision = filter_pair(body_a, body_b);
            if decision == PairDecision::Ignore {
                continue;
            }

            // Sensor tracking
            if decision == PairDecision::Trigger {
                let key = if ida <= idb { (ida, idb) } else { (idb, ida) };
                current_sensor_pairs.push(key);
                if !self.prev_sensor_pairs.contains(&key) {
                    self.sensor_events.push_back(SensorEvent {
                        sensor: ida,
                        other: idb,
                        began: true,
                    });
                }
                continue;
            }

            // Narrow phase: circle-circle for now
            let (body_a_ref, body_b_ref) = match (self.bodies.get(ha), self.bodies.get(hb)) {
                (Some(a), Some(b)) => (a, b),
                _ => continue,
            };

            // Find closest features across all collider pairs
            let mut best_contact: Option<(Vec2, Real, Vec2, FeatureId)> = None;
            for ca in &body_a_ref.colliders {
                for cb in &body_b_ref.colliders {
                    // Simple circle-circle contact for now; expand to full shape dispatch
                    match (&ca.shape, &cb.shape) {
                        (ColliderShape2::Circle(a), ColliderShape2::Circle(b)) => {
                            let world_a = body_a_ref.position + ca.offset;
                            let world_b = body_b_ref.position + cb.offset;
                            if let Some(contact) = auralite_collision::circle_circle(
                                world_a,
                                a.radius(),
                                world_b,
                                b.radius(),
                            ) {
                                let fid = FeatureId(
                                    (self.step) << 32 | (u64::from(best_contact.is_some())),
                                );
                                if best_contact.is_none()
                                    || contact.penetration > best_contact.unwrap().1
                                {
                                    best_contact = Some((
                                        contact.normal,
                                        contact.penetration,
                                        contact.point,
                                        fid,
                                    ));
                                }
                            }
                        }
                        _ => {
                            // Use GJK distance for general convex pairs
                            if let Some(pen) =
                                generic_convex_contact_2d(ca, cb, body_a_ref, body_b_ref)
                            {
                                let fid = FeatureId((self.step) << 32 | 1);
                                if best_contact.is_none() || pen.1 > best_contact.unwrap().1 {
                                    best_contact = Some((pen.0, pen.1, pen.2, fid));
                                }
                            }
                        }
                    }
                }
            }

            if let Some((normal, penetration, point, fid)) = best_contact {
                let combined_rest = combine(
                    body_a_ref.restitution,
                    body_b_ref.restitution,
                    self.restitution_mode,
                );
                let combined_fric =
                    combine(body_a_ref.friction, body_b_ref.friction, self.friction_mode);
                let mut cc = ContactConstraint2::new(
                    ha,
                    hb,
                    normal,
                    point,
                    penetration,
                    combined_rest,
                    combined_fric,
                    fid,
                    self,
                );
                // Warm start from previous manifolds
                for pm in &self.prev_manifolds {
                    cc.warm_start(pm);
                }
                constraints.push(cc);
            }
        }

        // ── 6. Solve all contacts (body-body) ──
        solve_contacts_2d(&mut constraints, &mut self.bodies);

        // ── 7. Solve joints ──
        for i in 0..self.joints.len() {
            let imp = self.joints[i].solve(&mut self.bodies);
            if self.joints[i].broken && self.joints[i].config.break_impulse > 0.0 {
                self.joint_break_events.push_back(JointBreakEvent {
                    joint_id: JointId(i as u64),
                    impulse: imp,
                });
            }
        }

        // ── 8. Update manifolds for warm starting ──
        self.prev_manifolds = constraints
            .iter()
            .map(|c| Manifold2::from_clip(c.normal, vec![(c.point, c.penetration, c.feature_id)]))
            .collect();

        // ── 8. Sensor end events ──
        for prev in &self.prev_sensor_pairs {
            if !current_sensor_pairs.contains(prev) {
                self.sensor_events.push_back(SensorEvent {
                    sensor: prev.0,
                    other: prev.1,
                    began: false,
                });
            }
        }
        self.prev_sensor_pairs = current_sensor_pairs;

        // ── 9. Sleeping ──
        for &h in &body_handles {
            if let Some(b) = self.bodies.get(h) {
                if b.kind == BodyType::Dynamic && !b.sleeping {
                    let ek = b.velocity.length_squared() + b.angular_velocity * b.angular_velocity;
                    if ek < self.sleep_threshold {
                        // Only sleep if not in contact with a non-sleeping body
                        let has_active_contact =
                            constraints.iter().any(|c| c.body_a == h || c.body_b == h);
                        if !has_active_contact {
                            if let Some(bm) = self.bodies.get_mut(h) {
                                bm.sleeping = true;
                                bm.velocity = Vec2::ZERO;
                                bm.angular_velocity = 0.0;
                            }
                        }
                    }
                }
            }
        }

        self.step += 1;
        Ok(())
    }

    pub fn snapshot(&self) -> Snapshot2 {
        Snapshot2 {
            states: self
                .bodies
                .iter()
                .map(|(_, b)| {
                    (
                        b.id.0,
                        b.position,
                        b.rotation,
                        b.velocity,
                        b.angular_velocity,
                        b.restitution,
                        b.friction,
                        b.sleeping,
                    )
                })
                .collect(),
            step: self.step,
        }
    }
    pub fn restore(&mut self, s: &Snapshot2) -> Result<(), WorldError> {
        if s.states.len() != self.bodies.len() {
            return Err(WorldError::InvalidInput);
        }
        for (_, b) in self.bodies.iter_mut() {
            let entry = s
                .states
                .iter()
                .find(|x| x.0 == b.id.0)
                .ok_or(WorldError::InvalidInput)?;
            b.position = entry.1;
            b.rotation = entry.2;
            b.velocity = entry.3;
            b.angular_velocity = entry.4;
            b.restitution = entry.5;
            b.friction = entry.6;
            b.sleeping = entry.7;
        }
        self.step = s.step;
        Ok(())
    }
    pub fn state_hash(&self) -> u64 {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&self.step.to_le_bytes());
        for (_, b) in self.bodies.iter() {
            bytes.extend_from_slice(&b.id.0.to_le_bytes());
            bytes.extend_from_slice(&b.position.x.to_bits().to_le_bytes());
            bytes.extend_from_slice(&b.position.y.to_bits().to_le_bytes());
            bytes.extend_from_slice(&rot2_angle(b.rotation).to_bits().to_le_bytes());
            bytes.extend_from_slice(&b.velocity.x.to_bits().to_le_bytes());
            bytes.extend_from_slice(&b.velocity.y.to_bits().to_le_bytes());
            bytes.extend_from_slice(&b.angular_velocity.to_bits().to_le_bytes());
            bytes.push(u8::from(b.sleeping));
        }
        hash_bytes(&bytes)
    }
    pub fn wake_body(&mut self, h: BodyHandle2) -> Result<(), WorldError> {
        let b = self.bodies.get_mut(h).ok_or(WorldError::StaleHandle)?;
        b.sleeping = false;
        Ok(())
    }
}

/// Extract angle from a Rot2 by rotating (1,0) and computing atan2.
fn rot2_angle(r: Rot2) -> Real {
    let v = r.rotate(Vec2::X);
    v.y.atan2(v.x)
}

/// Generic 2D convex contact using GJK + EPA.
fn generic_convex_contact_2d(
    ca: &Collider2,
    cb: &Collider2,
    ba: &Body2,
    bb: &Body2,
) -> Option<(Vec2, Real, Vec2)> {
    let world_a = ba.position + ca.offset;
    let world_b = bb.position + cb.offset;
    // Use GJK distance to get separation
    use auralite_collision::gjk_distance2;
    let support_a = |d: Vec2| -> Vec2 {
        world_a
            + match &ca.shape {
                ColliderShape2::Circle(c) => c.support(d),
                ColliderShape2::Box(b) => b.support(ba.rotation.inverse().rotate(d)),
                ColliderShape2::Capsule(cap) => cap.support(ba.rotation.inverse().rotate(d)),
                ColliderShape2::ConvexPolygon(p) => p.support(ba.rotation.inverse().rotate(d)),
                ColliderShape2::Edge(e) => e.closest_point(world_a + d * 100.0),
            }
    };
    let support_b = |d: Vec2| -> Vec2 {
        world_b
            + match &cb.shape {
                ColliderShape2::Circle(c) => c.support(d),
                ColliderShape2::Box(b) => b.support(bb.rotation.inverse().rotate(d)),
                ColliderShape2::Capsule(cap) => cap.support(bb.rotation.inverse().rotate(d)),
                ColliderShape2::ConvexPolygon(p) => p.support(bb.rotation.inverse().rotate(d)),
                ColliderShape2::Edge(e) => e.closest_point(world_b + d * 100.0),
            }
    };
    let gjk = gjk_distance2(support_a, support_b, 32);
    if gjk.distance <= ABS_EPSILON {
        // Overlapping: use EPA for depth
        let epa = auralite_collision::epa_penetration2(support_a, support_b, 32);
        if let Some(p) = epa {
            return Some((p.normal, p.depth, (gjk.point_a + gjk.point_b) * 0.5));
        }
        return Some((gjk.normal, 0.01, (gjk.point_a + gjk.point_b) * 0.5));
    }
    None
}

// ─── World3 ───────────────────────────────────────────────────────────────────

/// Native 3D world. For M4 we provide a minimal extension of the vertical slice.
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
    pub fn set_gravity(&mut self, g: Vec3) -> Result<(), WorldError> {
        if !g.is_finite() {
            return Err(WorldError::InvalidInput);
        }
        self.gravity = g;
        Ok(())
    }
    pub fn add_body(&mut self, b: BodyBuilder3) -> Result<BodyHandle3, WorldError> {
        if !b.position.is_finite()
            || !b.velocity.is_finite()
            || b.mass <= 0.0
            || !b.restitution.is_finite()
        {
            return Err(WorldError::InvalidInput);
        }
        let inertia = b.inertia_diagonal.unwrap_or(Vec3 {
            x: 0.1,
            y: 0.1,
            z: 0.1,
        });
        let id = StableId(self.next_id);
        self.next_id += 1;
        let h = self.bodies.insert(Body3 {
            id,
            kind: b.kind,
            position: b.position,
            rotation: b.rotation,
            velocity: b.velocity,
            angular_velocity: b.angular_velocity,
            inv_mass: if b.kind == BodyType::Dynamic {
                1.0 / b.mass
            } else {
                0.0
            },
            inv_inertia_diagonal: if b.kind == BodyType::Dynamic {
                Vec3 {
                    x: 1.0 / inertia.x,
                    y: 1.0 / inertia.y,
                    z: 1.0 / inertia.z,
                }
            } else {
                Vec3::ZERO
            },
            colliders: b.colliders,
            restitution: b.restitution,
            friction: b.friction,
            sleeping: false,
            force: Vec3::ZERO,
            torque: Vec3::ZERO,
            linear_damping: b.linear_damping,
            angular_damping: b.angular_damping,
            user_data: b.user_data,
        });
        Ok(h)
    }
    pub fn remove_body(&mut self, h: BodyHandle3) -> Result<Body3, WorldError> {
        self.bodies.remove(h).ok_or(WorldError::StaleHandle)
    }
    pub fn body(&self, h: BodyHandle3) -> Result<&Body3, WorldError> {
        self.bodies.get(h).ok_or(WorldError::StaleHandle)
    }
    pub fn apply_impulse(&mut self, h: BodyHandle3, j: Vec3) -> Result<(), WorldError> {
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
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    /// Step with gravity + ground contact (extended vertical slice).
    pub fn step(&mut self, dt: Real) -> Result<(), WorldError> {
        if !(dt > 0.0 && dt.is_finite()) {
            return Err(WorldError::InvalidInput);
        }
        let handles: Vec<BodyHandle3> = self.bodies.iter().map(|(h, _)| h).collect();
        for &h in &handles {
            if let Some(b) = self.bodies.get_mut(h) {
                if b.kind == BodyType::Dynamic && !b.sleeping {
                    b.velocity += self.gravity * dt;
                    b.position += b.velocity * dt;
                    // Simple ground contact for colliders
                    for c in &b.colliders {
                        if let ColliderShape3::Sphere(s) = &c.shape {
                            let world_center = b.position + c.offset;
                            let d = s.radius() - world_center.y;
                            if d > 0.0 {
                                b.position.y += d;
                                if b.velocity.y < 0.0 {
                                    b.velocity.y = -b.velocity.y * b.restitution;
                                }
                                if b.velocity.length_squared() < 1.0e-6
                                    && d <= CONTACT_SLOP + s.radius()
                                {
                                    b.velocity = Vec3::ZERO;
                                    b.sleeping = true;
                                }
                            }
                        } else if let ColliderShape3::Box(bx) = &c.shape {
                            let world_center = b.position + c.offset;
                            let d = bx.half_extents().y - world_center.y;
                            if d > 0.0 {
                                b.position.y += d;
                                if b.velocity.y < 0.0 {
                                    b.velocity.y = -b.velocity.y * b.restitution;
                                }
                                if b.velocity.length_squared() < 1.0e-6
                                    && d <= CONTACT_SLOP + bx.half_extents().y
                                {
                                    b.velocity = Vec3::ZERO;
                                    b.sleeping = true;
                                }
                            }
                        }
                    }
                    if !b.position.is_finite() || !b.velocity.is_finite() {
                        return Err(WorldError::InvalidInput);
                    }
                }
            }
        }
        self.step += 1;
        Ok(())
    }

    pub fn snapshot(&self) -> Snapshot3 {
        Snapshot3 {
            states: self
                .bodies
                .iter()
                .map(|(_, b)| {
                    (
                        b.id.0,
                        b.position,
                        b.rotation,
                        b.velocity,
                        b.angular_velocity,
                        b.restitution,
                        b.inv_inertia_diagonal,
                        b.sleeping,
                    )
                })
                .collect(),
            step: self.step,
        }
    }
    pub fn restore(&mut self, s: &Snapshot3) -> Result<(), WorldError> {
        if s.states.len() != self.bodies.len() {
            return Err(WorldError::InvalidInput);
        }
        for (_, b) in self.bodies.iter_mut() {
            let e = s
                .states
                .iter()
                .find(|x| x.0 == b.id.0)
                .ok_or(WorldError::InvalidInput)?;
            b.position = e.1;
            b.rotation = e.2;
            b.velocity = e.3;
            b.angular_velocity = e.4;
            b.restitution = e.5;
            b.inv_inertia_diagonal = e.6;
            b.sleeping = e.7;
        }
        self.step = s.step;
        Ok(())
    }
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

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn falling_2d_rests() {
        let mut w = World2::default();
        let b = BodyBuilder2::dynamic()
            .position(Vec2 { x: 0.0, y: 4.0 })
            .add_collider(Collider2 {
                shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            });
        let h = w.add_body(b).unwrap();
        for _ in 0..600 {
            w.step(1.0 / 60.0).unwrap();
        }
        let pos = w.body(h).unwrap().position.y;
        assert!(
            (pos - 0.5).abs() < 0.1,
            "body should rest on ground, y={}",
            pos
        );
    }

    #[test]
    fn rollback_replays_bitwise() {
        let mut w = World3::default();
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

    #[test]
    fn body_with_box_collider_falls_and_rests() {
        let mut w = World2::default();
        let b = BodyBuilder2::dynamic()
            .position(Vec2 { x: 0.0, y: 5.0 })
            .add_collider(Collider2 {
                shape: ColliderShape2::Box(Box2::new(Vec2 { x: 0.5, y: 0.5 }).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            });
        let h = w.add_body(b).unwrap();
        for _ in 0..600 {
            w.step(1.0 / 60.0).unwrap();
        }
        assert!(w.body(h).unwrap().sleeping);
        assert!(w.body(h).unwrap().position.y > -0.5 && w.body(h).unwrap().position.y < 1.0);
    }

    #[test]
    fn two_circles_stack() {
        let mut w = World2::default();
        // Static circle with center at y=-1.0 so its top is at y=0 (ground level)
        let b1 = BodyBuilder2::static_body()
            .position(Vec2 { x: 0.0, y: -1.0 })
            .add_collider(Collider2 {
                shape: ColliderShape2::Circle(Circle2::new(1.0).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            });
        w.add_body(b1).unwrap();
        // Dynamic circle above
        let b2 = BodyBuilder2::dynamic()
            .position(Vec2 { x: 0.0, y: 3.0 })
            .add_collider(Collider2 {
                shape: ColliderShape2::Circle(Circle2::new(1.0).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            });
        let h2 = w.add_body(b2).unwrap();
        for _ in 0..300 {
            w.step(1.0 / 60.0).unwrap();
        }
        // The falling body should rest at y=1.0 (radius=1, bottom on ground at y=0)
        assert!(w.body(h2).unwrap().sleeping, "body should be sleeping");
        assert!(
            (w.body(h2).unwrap().position.y - 1.0).abs() < 0.5,
            "y={}",
            w.body(h2).unwrap().position.y
        );
    }

    #[test]
    fn restitution_affects_bounce() {
        let mut w = World2::default();
        w.sleep_threshold = 0.0; // prevent sleeping during test
        // Start very close to ground so it hits quickly
        let h = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 0.0, y: 1.5 })
                    .restitution(0.8)
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        for _ in 0..60 {
            w.step(1.0 / 60.0).unwrap();
        }
        // After hitting ground, should have bounced back up
        let vel = w.body(h).unwrap().velocity.y;
        assert!(
            vel >= 0.0,
            "after bounce with restitution 0.8, velocity should be non-negative, got {}",
            vel
        );
    }

    #[test]
    fn sensor_events_fire() {
        let mut w = World2::default();
        let sensor = BodyBuilder2::dynamic()
            .position(Vec2 { x: 0.0, y: 10.0 })
            .add_collider(Collider2 {
                shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter {
                    sensor: true,
                    ..Default::default()
                },
            });
        let _sh = w.add_body(sensor).unwrap();
        let other = BodyBuilder2::static_body().add_collider(Collider2 {
            shape: ColliderShape2::Circle(Circle2::new(1.0).unwrap()),
            offset: Vec2::ZERO,
            material: Material::default(),
            filter: CollisionFilter::default(),
        });
        let _oh = w.add_body(other).unwrap();
        // Sensor body falls onto static body
        for _ in 0..300 {
            w.step(1.0 / 60.0).unwrap();
        }
        assert!(!w.sensor_events.is_empty(), "sensor events should fire");
        // At least one begin event
        let has_begin = w.sensor_events.iter().any(|e| e.began);
        assert!(has_begin, "should have at least one begin event");
    }

    #[test]
    fn static_body_does_not_move() {
        let mut w = World2::default();
        let h = w
            .add_body(
                BodyBuilder2::static_body()
                    .position(Vec2 { x: 10.0, y: 10.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        let pos_before = w.body(h).unwrap().position;
        for _ in 0..100 {
            w.step(1.0 / 60.0).unwrap();
        }
        assert_eq!(w.body(h).unwrap().position, pos_before);
    }

    #[test]
    fn forces_accumulate() {
        let mut w = World2::default();
        w.gravity = Vec2::ZERO;
        let h = w
            .add_body(BodyBuilder2::dynamic().add_collider(Collider2 {
                shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            }))
            .unwrap();
        w.apply_force(h, Vec2 { x: 10.0, y: 0.0 }).unwrap();
        w.step(1.0 / 60.0).unwrap();
        // After one step with force F=10, mass=1, dt=1/60, velocity should be F*dt = 10/60 ≈ 0.167
        let vx = w.body(h).unwrap().velocity.x;
        assert!(
            (vx - 10.0 / 60.0).abs() < 1.0e-4,
            "force should accelerate body, vx={}",
            vx
        );
    }

    #[test]
    fn angular_velocity_rotates() {
        let mut w = World2::default();
        w.gravity = Vec2::ZERO;
        // Position well above ground so ground contact doesn't trigger
        let h = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 0.0, y: 10.0 })
                    .angular_velocity(1.0)
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        let angle_before = rot2_angle(w.body(h).unwrap().rotation);
        w.step(1.0).unwrap();
        let angle_after = rot2_angle(w.body(h).unwrap().rotation);
        let diff = angle_after - angle_before;
        // With angular velocity 1.0 for 1.0 sec, should rotate ~1 rad
        assert!(
            (diff - 1.0).abs() < 0.01,
            "should rotate ~1 rad, got diff={:.6}",
            diff
        );
    }

    #[test]
    fn multiple_colliders_on_one_body() {
        let mut w = World2::default();
        let h = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 0.0, y: 5.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.3).unwrap()),
                        offset: Vec2 { x: 1.0, y: 0.0 },
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        for _ in 0..600 {
            w.step(1.0 / 60.0).unwrap();
        }
        assert!(w.body(h).unwrap().sleeping);
    }

    #[test]
    fn invalid_input_rejected() {
        let mut w = World2::default();
        assert_eq!(
            w.add_body(BodyBuilder2::dynamic().mass(0.0)),
            Err(WorldError::InvalidInput)
        );
        assert_eq!(
            w.add_body(BodyBuilder2::dynamic().mass(-1.0)),
            Err(WorldError::InvalidInput)
        );
        assert_eq!(w.step(-1.0), Err(WorldError::InvalidInput));
        assert_eq!(
            w.set_gravity(Vec2 {
                x: Real::NAN,
                y: 0.0
            }),
            Err(WorldError::InvalidInput)
        );
    }

    #[test]
    fn stale_handle_rejected() {
        let mut w = World2::default();
        let h = w
            .add_body(BodyBuilder2::dynamic().add_collider(Collider2 {
                shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            }))
            .unwrap();
        let _removed = w.remove_body(h).unwrap();
        assert_eq!(w.body(h).err(), Some(WorldError::StaleHandle));
    }

    #[test]
    fn wake_sleeping_body() {
        let mut w = World2::default();
        let h = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 0.0, y: 4.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        for _ in 0..600 {
            w.step(1.0 / 60.0).unwrap();
        }
        assert!(w.body(h).unwrap().sleeping);
        w.wake_body(h).unwrap();
        assert!(!w.body(h).unwrap().sleeping);
    }

    #[test]
    fn contact_filter_prevents_collision() {
        let mut w = World2::default();
        let b1 = BodyBuilder2::dynamic()
            .position(Vec2 { x: 0.0, y: 5.0 })
            .add_collider(Collider2 {
                shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter {
                    layers: 1,
                    mask: 2,
                    ..Default::default()
                },
            });
        let b2 = BodyBuilder2::static_body().add_collider(Collider2 {
            shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
            offset: Vec2::ZERO,
            material: Material::default(),
            filter: CollisionFilter {
                layers: 1,
                mask: 1,
                ..Default::default()
            },
        });
        let h1 = w.add_body(b1).unwrap();
        let _h2 = w.add_body(b2).unwrap();
        for _ in 0..600 {
            w.step(1.0 / 60.0).unwrap();
        }
        // Body 1 should fall through ground (filtered out)
        assert!(
            !w.body(h1).unwrap().sleeping,
            "filtered body should fall through"
        );
        assert!(
            w.body(h1).unwrap().position.y < -5.0,
            "filtered body should keep falling, y={}",
            w.body(h1).unwrap().position.y
        );
    }

    #[test]
    fn capsule_collider_falls_and_rests() {
        let mut w = World2::default();
        let b = BodyBuilder2::dynamic()
            .position(Vec2 { x: 0.0, y: 5.0 })
            .add_collider(Collider2 {
                shape: ColliderShape2::Capsule(Capsule2::new(0.5, 1.0).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            });
        let h = w.add_body(b).unwrap();
        for _ in 0..600 {
            w.step(1.0 / 60.0).unwrap();
        }
        assert!(w.body(h).unwrap().sleeping);
        assert!(
            (w.body(h).unwrap().position.y - 1.5).abs() < 1.0,
            "capsule should rest on ground, y={}",
            w.body(h).unwrap().position.y
        );
    }

    #[test]
    fn large_mass_ratio_stable() {
        let mut w = World2::default();
        // Heavy static ground
        let ground = BodyBuilder2::static_body().add_collider(Collider2 {
            shape: ColliderShape2::Box(Box2::new(Vec2 { x: 100.0, y: 0.5 }).unwrap()),
            offset: Vec2::ZERO,
            material: Material::default(),
            filter: CollisionFilter::default(),
        });
        w.add_body(ground).unwrap();
        // Light dynamic body on top (mass ratio 1000:1)
        let light = BodyBuilder2::dynamic()
            .position(Vec2 { x: 0.0, y: 2.0 })
            .mass(0.001)
            .add_collider(Collider2 {
                shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            });
        let h = w.add_body(light).unwrap();
        for _ in 0..300 {
            w.step(1.0 / 60.0).unwrap();
        }
        // Should not explode
        assert!(w.body(h).unwrap().velocity.is_finite());
        assert!(w.body(h).unwrap().position.y > -10.0);
    }

    #[test]
    fn distance_joint_holds_bodies_together() {
        let mut w = World2::default();
        w.gravity = Vec2::ZERO;
        let b1 = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: -1.0, y: 0.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.2).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        let b2 = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 1.0, y: 0.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.2).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        let jid = w
            .add_joint(JointConfig2::new(
                JointType2::Distance,
                b1,
                b2,
                Vec2::ZERO,
                Vec2::ZERO,
            ))
            .unwrap();
        for _ in 0..100 {
            w.step(1.0 / 60.0).unwrap();
        }
        let j = w.joint(jid).unwrap();
        assert!(!j.broken, "distance joint should not break");
        assert!(
            j.accumulated_position_error < 1.0,
            "joint drift should be small: {}",
            j.accumulated_position_error
        );
    }

    #[test]
    fn weld_joint_keeps_bodies_connected() {
        let mut w = World2::default();
        w.gravity = Vec2::ZERO;
        let b1 = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: -1.0, y: 0.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.2).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        let b2 = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 1.0, y: 0.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.2).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        let jid = w
            .add_joint(JointConfig2::new(
                JointType2::Weld,
                b1,
                b2,
                Vec2::ZERO,
                Vec2::ZERO,
            ))
            .unwrap();
        for _ in 0..200 {
            w.step(1.0 / 60.0).unwrap();
        }
        // Joint should still exist and not be broken
        let j = w.joint(jid);
        assert!(j.is_some(), "weld joint should still exist");
        assert!(!j.unwrap().broken, "weld joint should not break");
        // Both bodies should have finite positions
        assert!(w.body(b1).unwrap().position.is_finite());
        assert!(w.body(b2).unwrap().position.is_finite());
    }

    #[test]
    fn revolute_joint_pins_bodies() {
        let mut w = World2::default();
        w.gravity = Vec2::ZERO;
        let b1 = w
            .add_body(BodyBuilder2::static_body().add_collider(Collider2 {
                shape: ColliderShape2::Circle(Circle2::new(0.2).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            }))
            .unwrap();
        let b2 = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 0.0, y: 1.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.2).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        let _jid = w
            .add_joint(JointConfig2 {
                joint_type: JointType2::Revolute,
                body_a: b1,
                body_b: b2,
                anchor_a: Vec2::ZERO,
                anchor_b: Vec2 { x: 0.0, y: -0.5 },
                limits: JointLimits::default(),
                motor: JointMotor {
                    target_speed: 1.0,
                    max_force: 10.0,
                    enabled: true,
                },
                break_impulse: 0.0,
                user_data: 0,
            })
            .unwrap();
        for _ in 0..100 {
            w.step(1.0 / 60.0).unwrap();
        }
        // Body b2 should have rotated (motor target speed)
        let angle = rot2_angle(w.body(b2).unwrap().rotation);
        assert!(
            angle.abs() > 0.01,
            "motor should rotate body, angle={}",
            angle
        );
    }

    #[test]
    fn spring_joint_oscillates() {
        let mut w = World2::default();
        w.gravity = Vec2::ZERO;
        let b1 = w
            .add_body(BodyBuilder2::static_body().add_collider(Collider2 {
                shape: ColliderShape2::Circle(Circle2::new(0.2).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            }))
            .unwrap();
        let b2 = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 2.0, y: 0.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.2).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        let _jid = w
            .add_joint(JointConfig2 {
                joint_type: JointType2::Spring {
                    stiffness: 50.0,
                    damping: 1.0,
                },
                body_a: b1,
                body_b: b2,
                anchor_a: Vec2::ZERO,
                anchor_b: Vec2::ZERO,
                limits: JointLimits::default(),
                motor: JointMotor::default(),
                break_impulse: 0.0,
                user_data: 0,
            })
            .unwrap();
        let pos_before = w.body(b2).unwrap().position.x;
        for _ in 0..60 {
            w.step(1.0 / 60.0).unwrap();
        }
        let pos_after = w.body(b2).unwrap().position.x;
        assert!(
            (pos_after - 0.0).abs() < (pos_before - 0.0).abs() + 0.1,
            "spring should pull body toward origin: before={}, after={}",
            pos_before,
            pos_after
        );
    }

    #[test]
    fn ragdoll_11_bodies_assembles() {
        let mut w = World2::default();
        let n_bodies = 11;
        let spacing = 0.8;
        let mut handles = Vec::new();
        for i in 0..n_bodies {
            let y = 5.0 + (n_bodies - 1 - i) as Real * spacing;
            let mass = if i % 2 == 0 { 1.0 } else { 0.5 };
            let b = w
                .add_body(
                    BodyBuilder2::dynamic()
                        .position(Vec2 { x: 0.0, y })
                        .mass(mass)
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Circle(Circle2::new(0.3).unwrap()),
                            offset: Vec2::ZERO,
                            material: Material::default(),
                            filter: CollisionFilter::default(),
                        }),
                )
                .unwrap();
            handles.push(b);
        }
        for i in 0..n_bodies - 1 {
            let a = handles[i + 1];
            let b = handles[i];
            w.add_joint(JointConfig2::new(
                JointType2::Revolute,
                a,
                b,
                Vec2 { x: 0.0, y: -0.4 },
                Vec2 { x: 0.0, y: 0.4 },
            ))
            .unwrap();
        }
        let anchor = w
            .add_body(BodyBuilder2::static_body().add_collider(Collider2 {
                shape: ColliderShape2::Circle(Circle2::new(0.1).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: CollisionFilter::default(),
            }))
            .unwrap();
        w.add_joint(JointConfig2::new(
            JointType2::Revolute,
            anchor,
            handles[n_bodies - 1],
            Vec2::ZERO,
            Vec2 { x: 0.0, y: 0.4 },
        ))
        .unwrap();
        assert_eq!(w.joints.len(), n_bodies, "should have {} joints", n_bodies);
        for _ in 0..300 {
            w.step(1.0 / 60.0).unwrap();
        }
        for h in &handles {
            let b = w.body(*h).unwrap();
            assert!(
                b.position.is_finite(),
                "ragdoll body should have finite position"
            );
            assert!(
                b.velocity.is_finite(),
                "ragdoll body should have finite velocity"
            );
        }
    }
}
