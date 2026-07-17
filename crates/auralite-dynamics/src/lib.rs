//! Native dimension-separated rigid-body worlds with full rotation, solver, colliders, sleeping, and sensors.
#![forbid(unsafe_code)]
#![allow(
    clippy::too_many_arguments,
    clippy::type_complexity,
    clippy::collapsible_if,
    clippy::redundant_closure_call,
    clippy::field_reassign_with_default
)]
#[cfg(not(any(feature = "multithread", feature = "single-thread")))]
compile_error!("auralite-dynamics requires either the 'multithread' or 'single-thread' feature");

/// item.
pub mod joints;
pub mod lockstep;
use auralite_collision::{
    CollisionFilter, DynamicTree2, DynamicTree3, FeatureId, Manifold2, Manifold3, PairDecision,
};
use auralite_core::{Handle, Pool, StableId, hash_bytes};
use auralite_geometry::{
    Box2, Box3, Capsule2, Capsule3, Circle2, ConvexHull3, ConvexPolygon, Edge2, Edge3, Sphere3,
    TriangleMesh,
};
use auralite_math::{ABS_EPSILON, CONTACT_SLOP, Quat, Ray2, Ray3, Real, Rot2, Vec2, Vec3};
/// item.
pub use joints::{
    Joint2, Joint3, JointBreakEvent, JointConfig2, JointConfig3, JointId, JointLimits, JointMotor,
    JointType2, JointType3,
};
pub use lockstep::{InputEvent, InputRecorder};
use std::collections::{HashMap, VecDeque};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// BodyType enumeration.
pub enum BodyType {
    /// Static variant.
    Static,
    /// Kinematic variant.
    Kinematic,
    /// Dynamic variant.
    Dynamic,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// WorldError enumeration.
pub enum WorldError {
    /// InvalidInput variant.
    InvalidInput,
    /// StaleHandle variant.
    StaleHandle,
    /// Internal variant.
    Internal,
}
#[derive(Clone, Copy, Debug, PartialEq)]
/// Material — represents material in the dynamics system.
pub struct Material {
    /// restitution field.
    pub restitution: Real,
    /// friction field.
    pub friction: Real,
    /// density field.
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
#[derive(Clone, Copy, Debug, PartialEq, Default)]
/// CombineMode enumeration.
pub enum CombineMode {
    /// Multiply variant.
    Multiply,
    #[default]
    /// Average variant.
    Average,
    /// Min variant.
    Min,
    /// Max variant.
    Max,
    /// First variant.
    First,
}
/// combine — performs combine operation.
pub fn combine(a: Real, b: Real, mode: CombineMode) -> Real {
    match mode {
        CombineMode::Multiply => a * b,
        CombineMode::Average => (a + b) * 0.5,
        CombineMode::Min => a.min(b),
        CombineMode::Max => a.max(b),
        CombineMode::First => a,
    }
}

#[derive(Clone, Debug, PartialEq)]
/// Collider2 — represents collider2 in the dynamics system.
pub struct Collider2 {
    /// shape field.
    pub shape: ColliderShape2,
    /// offset field.
    pub offset: Vec2,
    /// material field.
    pub material: Material,
    /// filter field.
    pub filter: CollisionFilter,
}
#[derive(Clone, Debug, PartialEq)]
/// ColliderShape2 enumeration.
pub enum ColliderShape2 {
    /// Circle variant.
    Circle(Circle2),
    /// Box variant.
    Box(Box2),
    /// Capsule variant.
    Capsule(Capsule2),
    /// ConvexPolygon variant.
    ConvexPolygon(ConvexPolygon),
    /// Edge variant.
    Edge(Edge2),
}
impl ColliderShape2 {
    /// ray_intersection — performs ray intersection operation.
    pub fn ray_intersection(&self, r: Ray2) -> Option<(Real, Vec2)> {
        match self {
            ColliderShape2::Circle(c) => c.ray_intersection(r),
            ColliderShape2::Box(b) => b.ray_intersection(r),
            ColliderShape2::Capsule(c) => c.ray_intersection(r),
            ColliderShape2::ConvexPolygon(p) => p.ray_intersection(r),
            ColliderShape2::Edge(e) => e.ray_intersection(r),
        }
    }
}
impl Collider2 {
    #[must_use]
    /// world_aabb — performs world aabb operation.
    pub fn world_aabb(&self, body_pos: Vec2, body_rot: Rot2) -> auralite_math::Aabb2 {
        let world_center = body_pos + body_rot.rotate(self.offset);
        let r = self.bounding_radius();
        let h = Vec2 { x: r, y: r };
        auralite_math::Aabb2::new(world_center - h, world_center + h)
            .unwrap_or_else(|_| auralite_math::Aabb2::new(world_center, world_center).unwrap())
    }
    #[must_use]
    /// bounding_radius — performs bounding radius operation.
    pub fn bounding_radius(&self) -> Real {
        (match &self.shape {
            ColliderShape2::Circle(c) => c.radius(),
            ColliderShape2::Box(b) => b.half_extents().length(),
            ColliderShape2::Capsule(cap) => cap.bounding_radius(),
            ColliderShape2::ConvexPolygon(poly) => poly.bounding_radius(),
            ColliderShape2::Edge(e) => e.bounding_radius(),
        }) + self.offset.length()
    }
}
#[derive(Clone, Debug, PartialEq)]
/// Collider3 — represents collider3 in the dynamics system.
pub struct Collider3 {
    /// shape field.
    pub shape: ColliderShape3,
    /// offset field.
    pub offset: Vec3,
    /// material field.
    pub material: Material,
    /// filter field.
    pub filter: CollisionFilter,
}
#[derive(Clone, Debug, PartialEq)]
/// ColliderShape3 enumeration.
pub enum ColliderShape3 {
    /// Sphere variant.
    Sphere(Sphere3),
    /// Box variant.
    Box(Box3),
    /// Capsule variant.
    Capsule(Capsule3),
    /// ConvexHull variant.
    ConvexHull(ConvexHull3),
    /// TriangleMesh variant.
    TriangleMesh(TriangleMesh),
    /// Edge variant.
    Edge(Edge3),
}
impl ColliderShape3 {
    /// support — performs support operation.
    pub fn support(&self, direction: Vec3) -> Vec3 {
        match self {
            ColliderShape3::Sphere(s) => s.support(direction),
            ColliderShape3::Box(b) => b.support(direction),
            ColliderShape3::Capsule(c) => c.support(direction),
            ColliderShape3::ConvexHull(h) => h.support(direction),
            ColliderShape3::Edge(e) => e.closest_point(direction * 100.0),
            ColliderShape3::TriangleMesh(_) => Vec3::ZERO,
        }
    }
    /// ray_intersection — performs ray intersection operation.
    pub fn ray_intersection(&self, r: Ray3) -> Option<(Real, Vec3)> {
        match self {
            ColliderShape3::Sphere(s) => s.ray_intersection(r),
            ColliderShape3::Box(b) => b.ray_intersection(r),
            ColliderShape3::Capsule(c) => c.ray_intersection(r),
            ColliderShape3::ConvexHull(h) => h.ray_intersection(r),
            ColliderShape3::TriangleMesh(m) => m.ray_intersection(r),
            ColliderShape3::Edge(e) => e.ray_intersection(r),
        }
    }
    /// volume — performs volume operation.
    pub fn volume(&self) -> Real {
        match self {
            ColliderShape3::Sphere(s) => s.volume(),
            ColliderShape3::Box(b) => b.volume(),
            ColliderShape3::Capsule(c) => c.volume(),
            ColliderShape3::ConvexHull(h) => h.volume(),
            _ => 0.0,
        }
    }
}
impl Collider3 {
    #[must_use]
    /// world_aabb — performs world aabb operation.
    pub fn world_aabb(&self, body_pos: Vec3, body_rot: Quat) -> auralite_math::Aabb3 {
        let world_center = body_pos + body_rot.rotate(self.offset);
        let r = self.bounding_radius();
        let h = Vec3 { x: r, y: r, z: r };
        auralite_math::Aabb3::new(world_center - h, world_center + h)
            .unwrap_or_else(|_| auralite_math::Aabb3::new(world_center, world_center).unwrap())
    }
    #[must_use]
    /// bounding_radius — performs bounding radius operation.
    pub fn bounding_radius(&self) -> Real {
        (match &self.shape {
            ColliderShape3::Sphere(s) => s.radius(),
            ColliderShape3::Box(b) => b.half_extents().length(),
            ColliderShape3::Capsule(cap) => cap.bounding_radius(),
            ColliderShape3::ConvexHull(hull) => hull.bounding_radius(),
            ColliderShape3::TriangleMesh(mesh) => mesh.bounding_radius(),
            ColliderShape3::Edge(e) => e.bounding_radius(),
        }) + self.offset.length()
    }
}

#[derive(Clone, Debug, PartialEq)]
/// Body2 — represents body2 in the dynamics system.
pub struct Body2 {
    /// id field.
    pub id: StableId,
    /// kind field.
    pub kind: BodyType,
    /// position field.
    pub position: Vec2,
    /// rotation field.
    pub rotation: Rot2,
    /// velocity field.
    pub velocity: Vec2,
    /// angular_velocity field.
    pub angular_velocity: Real,
    /// inv_mass field.
    pub inv_mass: Real,
    /// inv_inertia field.
    pub inv_inertia: Real,
    /// colliders field.
    pub colliders: Vec<Collider2>,
    /// restitution field.
    pub restitution: Real,
    /// friction field.
    pub friction: Real,
    /// sleeping field.
    pub sleeping: bool,
    /// force field.
    pub force: Vec2,
    /// torque field.
    pub torque: Real,
    /// linear_damping field.
    pub linear_damping: Real,
    /// angular_damping field.
    pub angular_damping: Real,
    /// user_data field.
    pub user_data: u64,
}
impl Body2 {
    #[must_use]
    /// world_aabb — performs world aabb operation.
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
            min.x = min.x.min(a.min.x);
            min.y = min.y.min(a.min.y);
            max.x = max.x.max(a.max.x);
            max.y = max.y.max(a.max.y);
        }
        auralite_math::Aabb2::new(min, max).unwrap()
    }
    #[must_use]
    /// effective_inv_mass — performs effective inv mass operation.
    pub fn effective_inv_mass(&self) -> Real {
        if self.kind == BodyType::Dynamic {
            self.inv_mass
        } else {
            0.0
        }
    }
    #[must_use]
    /// effective_inv_inertia — performs effective inv inertia operation.
    pub fn effective_inv_inertia(&self) -> Real {
        if self.kind == BodyType::Dynamic {
            self.inv_inertia
        } else {
            0.0
        }
    }
    /// apply_impulse — performs apply impulse operation.
    pub fn apply_impulse(&mut self, impulse: Vec2, point: Vec2) {
        if self.kind != BodyType::Dynamic || self.sleeping {
            return;
        }
        self.velocity += impulse * self.effective_inv_mass();
        let r = point - self.position;
        self.angular_velocity += r.cross(impulse) * self.effective_inv_inertia();
    }
}
#[derive(Clone, Debug, PartialEq)]
/// Body3 — represents body3 in the dynamics system.
pub struct Body3 {
    /// id field.
    pub id: StableId,
    /// kind field.
    pub kind: BodyType,
    /// position field.
    pub position: Vec3,
    /// rotation field.
    pub rotation: Quat,
    /// velocity field.
    pub velocity: Vec3,
    /// angular_velocity field.
    pub angular_velocity: Vec3,
    /// inv_mass field.
    pub inv_mass: Real,
    /// inv_inertia_diagonal field.
    pub inv_inertia_diagonal: Vec3,
    /// colliders field.
    pub colliders: Vec<Collider3>,
    /// restitution field.
    pub restitution: Real,
    /// friction field.
    pub friction: Real,
    /// sleeping field.
    pub sleeping: bool,
    /// force field.
    pub force: Vec3,
    /// torque field.
    pub torque: Vec3,
    /// linear_damping field.
    pub linear_damping: Real,
    /// angular_damping field.
    pub angular_damping: Real,
    /// user_data field.
    pub user_data: u64,
}
impl Body3 {
    #[must_use]
    /// world_aabb — performs world aabb operation.
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
            min.x = min.x.min(a.min.x);
            min.y = min.y.min(a.min.y);
            min.z = min.z.min(a.min.z);
            max.x = max.x.max(a.max.x);
            max.y = max.y.max(a.max.y);
            max.z = max.z.max(a.max.z);
        }
        auralite_math::Aabb3::new(min, max).unwrap()
    }
    #[must_use]
    /// effective_inv_mass — performs effective inv mass operation.
    pub fn effective_inv_mass(&self) -> Real {
        if self.kind == BodyType::Dynamic {
            self.inv_mass
        } else {
            0.0
        }
    }
    /// apply_impulse — performs apply impulse operation.
    pub fn apply_impulse(&mut self, impulse: Vec3, point: Vec3) {
        if self.kind != BodyType::Dynamic || self.sleeping {
            return;
        }
        self.velocity += impulse * self.effective_inv_mass();
        let r = point - self.position;
        let torque_impulse = r.cross(impulse);
        let local_torque = self.rotation.inverse().rotate(torque_impulse);
        let local_ang_vel_change = local_torque * self.inv_inertia_diagonal;
        self.angular_velocity += self.rotation.rotate(local_ang_vel_change);
    }
}

/// Type alias for BodyHandle2.
pub type BodyHandle2 = Handle<Body2>;
/// Type alias for BodyHandle3.
pub type BodyHandle3 = Handle<Body3>;
/// Type alias for ColliderHandle2.
pub type ColliderHandle2 = Handle<Collider2>;
/// Type alias for ColliderHandle3.
pub type ColliderHandle3 = Handle<Collider3>;
pub(crate) fn apply_impulse2(
    pool: &mut Pool<Body2>,
    ha: BodyHandle2,
    hb: BodyHandle2,
    impulse: Vec2,
    point: Vec2,
) {
    if let Some(ba) = pool.get_mut(ha) {
        ba.apply_impulse(impulse, point);
    }
    if let Some(bb) = pool.get_mut(hb) {
        bb.apply_impulse(-impulse, point);
    }
}
pub(crate) fn apply_impulse3(
    pool: &mut Pool<Body3>,
    ha: BodyHandle3,
    hb: BodyHandle3,
    impulse: Vec3,
    point: Vec3,
) {
    if let Some(ba) = pool.get_mut(ha) {
        ba.apply_impulse(impulse, point);
    }
    if let Some(bb) = pool.get_mut(hb) {
        bb.apply_impulse(-impulse, point);
    }
}

/// Builder for constructing 2D bodies (`Static`, `Kinematic`, `Dynamic`).
///
/// # Example
/// ```
/// use auralite_dynamics::BodyBuilder2;
/// use auralite_math::Vec2;
/// let builder = BodyBuilder2::dynamic().position(Vec2 { x: 1.0, y: 2.0 }).mass(5.0);
/// assert_eq!(builder.position.x, 1.0);
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct BodyBuilder2 {
    /// kind field.
    pub kind: BodyType,
    /// position field.
    pub position: Vec2,
    /// rotation field.
    pub rotation: Rot2,
    /// velocity field.
    pub velocity: Vec2,
    /// angular_velocity field.
    pub angular_velocity: Real,
    /// mass field.
    pub mass: Real,
    /// inertia field.
    pub inertia: Option<Real>,
    /// colliders field.
    pub colliders: Vec<Collider2>,
    /// restitution field.
    pub restitution: Real,
    /// friction field.
    pub friction: Real,
    /// linear_damping field.
    pub linear_damping: Real,
    /// angular_damping field.
    pub angular_damping: Real,
    /// user_data field.
    pub user_data: u64,
}
impl BodyBuilder2 {
    /// new — performs new operation.
    pub fn new() -> Self {
        Self::dynamic()
    }
    /// dynamic — performs dynamic operation.
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
    /// static_body — performs static body operation.
    pub fn static_body() -> Self {
        let mut b = Self::dynamic();
        b.kind = BodyType::Static;
        b
    }
    /// position — performs position operation.
    pub fn position(mut self, v: Vec2) -> Self {
        self.position = v;
        self
    }
    /// rotation — performs rotation operation.
    pub fn rotation(mut self, r: Rot2) -> Self {
        self.rotation = r;
        self
    }
    /// velocity — performs velocity operation.
    pub fn velocity(mut self, v: Vec2) -> Self {
        self.velocity = v;
        self
    }
    /// mass — performs mass operation.
    pub fn mass(mut self, v: Real) -> Self {
        self.mass = v;
        self
    }
    /// restitution — performs restitution operation.
    pub fn restitution(mut self, v: Real) -> Self {
        self.restitution = v;
        self
    }
    /// inertia — performs inertia operation.
    pub fn inertia(mut self, i: Real) -> Self {
        self.inertia = Some(i);
        self
    }
    /// add_collider — performs add collider operation.
    pub fn add_collider(mut self, c: Collider2) -> Self {
        self.colliders.push(c);
        self
    }
    /// angular_velocity — performs angular velocity operation.
    pub fn angular_velocity(mut self, v: Real) -> Self {
        self.angular_velocity = v;
        self
    }
    /// friction — performs friction operation.
    pub fn friction(mut self, v: Real) -> Self {
        self.friction = v;
        self
    }
    /// linear_damping — performs linear damping operation.
    pub fn linear_damping(mut self, v: Real) -> Self {
        self.linear_damping = v;
        self
    }
    /// angular_damping — performs angular damping operation.
    pub fn angular_damping(mut self, v: Real) -> Self {
        self.angular_damping = v;
        self
    }
    /// user_data — performs user data operation.
    pub fn user_data(mut self, v: u64) -> Self {
        self.user_data = v;
        self
    }
}
impl Default for BodyBuilder2 {
    fn default() -> Self {
        Self::new()
    }
}
/// Builder for constructing 3D bodies (`Static`, `Kinematic`, `Dynamic`).
///
/// # Example
/// ```
/// use auralite_dynamics::BodyBuilder3;
/// use auralite_math::Vec3;
/// let builder = BodyBuilder3::dynamic().position(Vec3 { x: 1.0, y: 2.0, z: 3.0 }).mass(5.0);
/// assert_eq!(builder.position.x, 1.0);
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct BodyBuilder3 {
    /// kind field.
    pub kind: BodyType,
    /// position field.
    pub position: Vec3,
    /// rotation field.
    pub rotation: Quat,
    /// velocity field.
    pub velocity: Vec3,
    /// angular_velocity field.
    pub angular_velocity: Vec3,
    /// mass field.
    pub mass: Real,
    /// inertia_diagonal field.
    pub inertia_diagonal: Option<Vec3>,
    /// colliders field.
    pub colliders: Vec<Collider3>,
    /// restitution field.
    pub restitution: Real,
    /// friction field.
    pub friction: Real,
    /// linear_damping field.
    pub linear_damping: Real,
    /// angular_damping field.
    pub angular_damping: Real,
    /// user_data field.
    pub user_data: u64,
}
impl BodyBuilder3 {
    /// new — performs new operation.
    pub fn new() -> Self {
        Self::dynamic()
    }
    /// dynamic — performs dynamic operation.
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
    /// static_body — performs static body operation.
    pub fn static_body() -> Self {
        let mut b = Self::dynamic();
        b.kind = BodyType::Static;
        b
    }
    /// position — performs position operation.
    pub fn position(mut self, v: Vec3) -> Self {
        self.position = v;
        self
    }
    /// rotation — performs rotation operation.
    pub fn rotation(mut self, r: Quat) -> Self {
        self.rotation = r;
        self
    }
    /// mass — performs mass operation.
    pub fn mass(mut self, v: Real) -> Self {
        self.mass = v;
        self
    }
    /// inertia_diagonal — performs inertia diagonal operation.
    pub fn inertia_diagonal(mut self, i: Vec3) -> Self {
        self.inertia_diagonal = Some(i);
        self
    }
    /// add_collider — performs add collider operation.
    pub fn add_collider(mut self, c: Collider3) -> Self {
        self.colliders.push(c);
        self
    }
    /// velocity — performs velocity operation.
    pub fn velocity(mut self, v: Vec3) -> Self {
        self.velocity = v;
        self
    }
    /// angular_velocity — performs angular velocity operation.
    pub fn angular_velocity(mut self, v: Vec3) -> Self {
        self.angular_velocity = v;
        self
    }
    /// restitution — performs restitution operation.
    pub fn restitution(mut self, v: Real) -> Self {
        self.restitution = v;
        self
    }
    /// friction — performs friction operation.
    pub fn friction(mut self, v: Real) -> Self {
        self.friction = v;
        self
    }
    /// linear_damping — performs linear damping operation.
    pub fn linear_damping(mut self, v: Real) -> Self {
        self.linear_damping = v;
        self
    }
    /// angular_damping — performs angular damping operation.
    pub fn angular_damping(mut self, v: Real) -> Self {
        self.angular_damping = v;
        self
    }
    /// user_data — performs user data operation.
    pub fn user_data(mut self, v: u64) -> Self {
        self.user_data = v;
        self
    }
}
impl Default for BodyBuilder3 {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug)]
/// ContactConstraint2 — represents contactconstraint2 in the dynamics system.
pub struct ContactConstraint2 {
    /// body_a field.
    pub body_a: BodyHandle2,
    /// body_b field.
    pub body_b: BodyHandle2,
    /// normal field.
    pub normal: Vec2,
    /// tangent field.
    pub tangent: Vec2,
    /// point field.
    pub point: Vec2,
    /// penetration field.
    pub penetration: Real,
    /// restitution field.
    pub restitution: Real,
    /// friction field.
    pub friction: Real,
    /// normal_impulse field.
    pub normal_impulse: Real,
    /// tangent_impulse field.
    pub tangent_impulse: Real,
    /// feature_id field.
    pub feature_id: FeatureId,
    /// effective_mass_normal field.
    pub effective_mass_normal: Real,
    /// effective_mass_tangent field.
    pub effective_mass_tangent: Real,
    /// bias field.
    pub bias: Real,
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
        let (ia, ii_a) = w2.bodies.get(body_a).map_or((0.0, 0.0), |b| {
            (b.effective_inv_mass(), b.effective_inv_inertia())
        });
        let (ib, ii_b) = w2.bodies.get(body_b).map_or((0.0, 0.0), |b| {
            (b.effective_inv_mass(), b.effective_inv_inertia())
        });
        let ra = point - w2.bodies.get(body_a).map_or(Vec2::ZERO, |b| b.position);
        let rb = point - w2.bodies.get(body_b).map_or(Vec2::ZERO, |b| b.position);
        let mn = compute_effective_mass_2d(ia, ii_a, ib, ii_b, ra, rb, normal);
        let mt = compute_effective_mass_2d(ia, ii_a, ib, ii_b, ra, rb, tangent);
        let bias = if penetration > CONTACT_SLOP {
            (penetration - CONTACT_SLOP) * 0.2 / (0.016666668 * w2.solver_iterations.max(1) as Real)
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
    fn warm_start(&mut self, manifold: &Manifold2, bodies: &mut Pool<Body2>) {
        for mp in &manifold.points {
            if mp.feature == self.feature_id {
                self.normal_impulse = mp.normal_impulse;
                self.tangent_impulse = mp.tangent_impulse;
                let imp = self.normal * self.normal_impulse + self.tangent * self.tangent_impulse;
                if let Some(ba) = bodies.get_mut(self.body_a) {
                    ba.apply_impulse(-imp, self.point);
                }
                if let Some(bb) = bodies.get_mut(self.body_b) {
                    bb.apply_impulse(imp, self.point);
                }
                break;
            }
        }
    }
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
fn solve_contacts_2d_once(constraints: &mut [ContactConstraint2], bodies: &mut Pool<Body2>) {
    for c in constraints.iter_mut() {
        let (va, wa) = bodies
            .get(c.body_a)
            .map_or((Vec2::ZERO, 0.0), |b| (b.velocity, b.angular_velocity));
        let (vb, wb) = bodies
            .get(c.body_b)
            .map_or((Vec2::ZERO, 0.0), |b| (b.velocity, b.angular_velocity));
        let ra = c.point - bodies.get(c.body_a).map_or(Vec2::ZERO, |b| b.position);
        let rb = c.point - bodies.get(c.body_b).map_or(Vec2::ZERO, |b| b.position);
        let rel_v = (vb + Vec2 { x: -rb.y, y: rb.x } * wb) - (va + Vec2 { x: -ra.y, y: ra.x } * wa);
        let dvn = rel_v.dot(c.normal);
        let lambda_n = (-(1.0 + c.restitution) * dvn + c.bias) * c.effective_mass_normal;
        let old_n = c.normal_impulse;
        c.normal_impulse = (old_n + lambda_n).max(0.0);
        let delta_n = c.normal_impulse - old_n;
        if let Some(ba) = bodies.get_mut(c.body_a) {
            ba.apply_impulse(-c.normal * delta_n, c.point);
        }
        if let Some(bb) = bodies.get_mut(c.body_b) {
            bb.apply_impulse(c.normal * delta_n, c.point);
        }

        // Solve Coulomb friction (tangent impulse)
        let (va, wa) = bodies
            .get(c.body_a)
            .map_or((Vec2::ZERO, 0.0), |b| (b.velocity, b.angular_velocity));
        let (vb, wb) = bodies
            .get(c.body_b)
            .map_or((Vec2::ZERO, 0.0), |b| (b.velocity, b.angular_velocity));
        let rel_v_t =
            (vb + Vec2 { x: -rb.y, y: rb.x } * wb) - (va + Vec2 { x: -ra.y, y: ra.x } * wa);
        let dvt = rel_v_t.dot(c.tangent);
        let lambda_t = -dvt * c.effective_mass_tangent;
        let max_f = c.friction * c.normal_impulse;
        let old_t = c.tangent_impulse;
        c.tangent_impulse = (old_t + lambda_t).clamp(-max_f, max_f);
        let delta_t = c.tangent_impulse - old_t;
        if let Some(ba) = bodies.get_mut(c.body_a) {
            ba.apply_impulse(-c.tangent * delta_t, c.point);
        }
        if let Some(bb) = bodies.get_mut(c.body_b) {
            bb.apply_impulse(c.tangent * delta_t, c.point);
        }
    }
}

#[derive(Clone, Debug)]
/// ContactConstraint3 — represents contactconstraint3 in the dynamics system.
pub struct ContactConstraint3 {
    /// body_a field.
    pub body_a: BodyHandle3,
    /// body_b field.
    pub body_b: BodyHandle3,
    /// normal field.
    pub normal: Vec3,
    /// tangent1 field.
    pub tangent1: Vec3,
    /// tangent2 field.
    pub tangent2: Vec3,
    /// point field.
    pub point: Vec3,
    /// penetration field.
    pub penetration: Real,
    /// restitution field.
    pub restitution: Real,
    /// friction field.
    pub friction: Real,
    /// normal_impulse field.
    pub normal_impulse: Real,
    /// tangent1_impulse field.
    pub tangent1_impulse: Real,
    /// tangent2_impulse field.
    pub tangent2_impulse: Real,
    /// feature_id field.
    pub feature_id: FeatureId,
    /// effective_mass_normal field.
    pub effective_mass_normal: Real,
    /// effective_mass_tangent1 field.
    pub effective_mass_tangent1: Real,
    /// effective_mass_tangent2 field.
    pub effective_mass_tangent2: Real,
    /// bias field.
    pub bias: Real,
}

impl ContactConstraint3 {
    fn new(
        body_a: BodyHandle3,
        body_b: BodyHandle3,
        normal: Vec3,
        point: Vec3,
        penetration: Real,
        restitution: Real,
        friction: Real,
        feature_id: FeatureId,
        w3: &World3,
    ) -> Self {
        let (tangent1, tangent2) = if normal.x.abs() < 0.9 {
            let t1 = Vec3::X.cross(normal).normalized_or(Vec3::Y);
            (t1, normal.cross(t1).normalized_or(Vec3::Z))
        } else {
            let t1 = Vec3::Y.cross(normal).normalized_or(Vec3::Z);
            (t1, normal.cross(t1).normalized_or(Vec3::Y))
        };
        let (ia, ii_a) = w3.bodies.get(body_a).map_or((0.0, Vec3::ZERO), |b| {
            (b.effective_inv_mass(), b.inv_inertia_diagonal)
        });
        let (ib, ii_b) = w3.bodies.get(body_b).map_or((0.0, Vec3::ZERO), |b| {
            (b.effective_inv_mass(), b.inv_inertia_diagonal)
        });
        let ra = point - w3.bodies.get(body_a).map_or(Vec3::ZERO, |b| b.position);
        let rb = point - w3.bodies.get(body_b).map_or(Vec3::ZERO, |b| b.position);
        let mn = compute_effective_mass_3d(ia, ii_a, ib, ii_b, ra, rb, normal);
        let mt1 = compute_effective_mass_3d(ia, ii_a, ib, ii_b, ra, rb, tangent1);
        let mt2 = compute_effective_mass_3d(ia, ii_a, ib, ii_b, ra, rb, tangent2);
        let bias = if penetration > CONTACT_SLOP {
            (penetration - CONTACT_SLOP) * 0.2 / (0.016666668 * w3.solver_iterations.max(1) as Real)
        } else {
            0.0
        };
        Self {
            body_a,
            body_b,
            normal,
            tangent1,
            tangent2,
            point,
            penetration,
            restitution,
            friction,
            normal_impulse: 0.0,
            tangent1_impulse: 0.0,
            tangent2_impulse: 0.0,
            feature_id,
            effective_mass_normal: mn,
            effective_mass_tangent1: mt1,
            effective_mass_tangent2: mt2,
            bias,
        }
    }
    fn warm_start(&mut self, manifold: &Manifold3, bodies: &mut Pool<Body3>) {
        for mp in &manifold.points {
            if mp.feature == self.feature_id {
                self.normal_impulse = mp.normal_impulse;
                self.tangent1_impulse = mp.tangent_impulse;
                let imp = self.normal * self.normal_impulse + self.tangent1 * self.tangent1_impulse;
                if let Some(ba) = bodies.get_mut(self.body_a) {
                    ba.apply_impulse(-imp, self.point);
                }
                if let Some(bb) = bodies.get_mut(self.body_b) {
                    bb.apply_impulse(imp, self.point);
                }
                break;
            }
        }
    }
}

fn compute_effective_mass_3d(
    ia: Real,
    inv_ii_a: Vec3,
    ib: Real,
    inv_ii_b: Vec3,
    ra: Vec3,
    rb: Vec3,
    n: Vec3,
) -> Real {
    let ra_cross_n = ra.cross(n);
    let rb_cross_n = rb.cross(n);
    let sum = ia
        + ib
        + ra_cross_n.x * ra_cross_n.x * inv_ii_a.x
        + ra_cross_n.y * ra_cross_n.y * inv_ii_a.y
        + ra_cross_n.z * ra_cross_n.z * inv_ii_a.z
        + rb_cross_n.x * rb_cross_n.x * inv_ii_b.x
        + rb_cross_n.y * rb_cross_n.y * inv_ii_b.y
        + rb_cross_n.z * rb_cross_n.z * inv_ii_b.z;
    if sum > crate::ABS_EPSILON {
        1.0 / sum
    } else {
        0.0
    }
}

fn solve_contacts_3d_once(constraints: &mut [ContactConstraint3], bodies: &mut Pool<Body3>) {
    for c in constraints.iter_mut() {
        let (va, wa) = bodies.get(c.body_a).map_or((Vec3::ZERO, Vec3::ZERO), |b| {
            (b.velocity, b.angular_velocity)
        });
        let (vb, wb) = bodies.get(c.body_b).map_or((Vec3::ZERO, Vec3::ZERO), |b| {
            (b.velocity, b.angular_velocity)
        });
        let ra = c.point - bodies.get(c.body_a).map_or(Vec3::ZERO, |b| b.position);
        let rb = c.point - bodies.get(c.body_b).map_or(Vec3::ZERO, |b| b.position);
        let rel_v = (vb + wb.cross(rb)) - (va + wa.cross(ra));

        let dvn = rel_v.dot(c.normal);
        let lambda_n = (-(1.0 + c.restitution) * dvn + c.bias) * c.effective_mass_normal;
        let old_n = c.normal_impulse;
        c.normal_impulse = (old_n + lambda_n).max(0.0);
        let delta_n = c.normal_impulse - old_n;
        if let Some(ba) = bodies.get_mut(c.body_a) {
            ba.apply_impulse(-c.normal * delta_n, c.point);
        }
        if let Some(bb) = bodies.get_mut(c.body_b) {
            bb.apply_impulse(c.normal * delta_n, c.point);
        }

        let (va, wa) = bodies.get(c.body_a).map_or((Vec3::ZERO, Vec3::ZERO), |b| {
            (b.velocity, b.angular_velocity)
        });
        let (vb, wb) = bodies.get(c.body_b).map_or((Vec3::ZERO, Vec3::ZERO), |b| {
            (b.velocity, b.angular_velocity)
        });
        let rel_v_t1 = (vb + wb.cross(rb)) - (va + wa.cross(ra));
        let dvt1 = rel_v_t1.dot(c.tangent1);
        let lambda_t1 = -dvt1 * c.effective_mass_tangent1;
        let max_f = c.friction * c.normal_impulse;
        let old_t1 = c.tangent1_impulse;
        c.tangent1_impulse = (old_t1 + lambda_t1).clamp(-max_f, max_f);
        let delta_t1 = c.tangent1_impulse - old_t1;
        if let Some(ba) = bodies.get_mut(c.body_a) {
            ba.apply_impulse(-c.tangent1 * delta_t1, c.point);
        }
        if let Some(bb) = bodies.get_mut(c.body_b) {
            bb.apply_impulse(c.tangent1 * delta_t1, c.point);
        }

        let (va, wa) = bodies.get(c.body_a).map_or((Vec3::ZERO, Vec3::ZERO), |b| {
            (b.velocity, b.angular_velocity)
        });
        let (vb, wb) = bodies.get(c.body_b).map_or((Vec3::ZERO, Vec3::ZERO), |b| {
            (b.velocity, b.angular_velocity)
        });
        let rel_v_t2 = (vb + wb.cross(rb)) - (va + wa.cross(ra));
        let dvt2 = rel_v_t2.dot(c.tangent2);
        let lambda_t2 = -dvt2 * c.effective_mass_tangent2;
        let old_t2 = c.tangent2_impulse;
        c.tangent2_impulse = (old_t2 + lambda_t2).clamp(-max_f, max_f);
        let delta_t2 = c.tangent2_impulse - old_t2;
        if let Some(ba) = bodies.get_mut(c.body_a) {
            ba.apply_impulse(-c.tangent2 * delta_t2, c.point);
        }
        if let Some(bb) = bodies.get_mut(c.body_b) {
            bb.apply_impulse(c.tangent2 * delta_t2, c.point);
        }
    }
}

/// Snapshot2 — represents snapshot2 in the dynamics system.
pub struct Snapshot2 {
    /// states field.
    pub states: Vec<(u64, Vec2, Rot2, Vec2, Real, Real, Real, bool)>,
    /// step field.
    pub step: u64,
}
/// Snapshot3 — represents snapshot3 in the dynamics system.
pub struct Snapshot3 {
    /// states field.
    pub states: Vec<(u64, Vec3, Quat, Vec3, Vec3, Real, Vec3, bool)>,
    /// step field.
    pub step: u64,
}
/// Sensor event — begin/stay/end per step in deterministic order.
/// H6: stay event emitted for ongoing trigger pairs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorEvent {
    /// Sensor body StableId.
    pub sensor: u64,
    /// Other body StableId.
    pub other: u64,
    /// True if begin event, false if end (unless is_stay).
    pub began: bool,
    /// True if this is a stay event (ongoing trigger). When is_stay=true, began is false to distinguish from begin.
    /// Stay is emitted each step for pairs present in both prev and current sensor sets, in deterministic sorted order.
    pub is_stay: bool,
}

impl SensorEvent {
    /// Returns true if this is a begin event.
    pub fn is_begin(&self) -> bool {
        self.began && !self.is_stay
    }
    /// Returns true if this is an end event.
    pub fn is_end(&self) -> bool {
        !self.began && !self.is_stay
    }
    /// Returns true if this is a stay event.
    pub fn is_stay(&self) -> bool {
        self.is_stay
    }
}

/// A 2D rigid body world containing bodies, joints, and spatial tree.
///
/// # Example
/// ```
/// use auralite_dynamics::{World2, BodyBuilder2};
/// use auralite_math::Vec2;
///
/// let mut world = World2::default();
/// let body = world.add_body(BodyBuilder2::dynamic().position(Vec2 { x: 0.0, y: 10.0 })).unwrap();
/// world.step(0.016).unwrap();
/// assert!(world.body(body).unwrap().position.y < 10.0);
/// ```
#[derive(Clone)]
pub struct World2 {
    gravity: Vec2,
    bodies: Pool<Body2>,
    next_id: u64,
    step: u64,
    dynamic_tree: DynamicTree2,
    /// solver_iterations field.
    pub solver_iterations: u16,
    /// sleep_threshold field.
    pub sleep_threshold: Real,
    /// restitution_mode field.
    pub restitution_mode: CombineMode,
    /// friction_mode field.
    pub friction_mode: CombineMode,
    prev_manifolds: Vec<Manifold2>,
    prev_sensor_pairs: Vec<(u64, u64)>,
    /// sensor_events field.
    pub sensor_events: VecDeque<SensorEvent>,
    /// joints field.
    pub joints: Vec<Joint2>,
    /// joint_break_events field.
    pub joint_break_events: VecDeque<JointBreakEvent>,
    scratch_pairs: Vec<(u64, u64)>,
    scratch_handles: Vec<BodyHandle2>,
    scratch_constraints: Vec<ContactConstraint2>,
    scratch_raw_contacts: Vec<(
        BodyHandle2,
        BodyHandle2,
        Vec2,
        Vec<(Vec2, Real, FeatureId)>,
        Real,
        Real,
    )>,
    scratch_id_to_h: HashMap<u64, BodyHandle2>,
    scratch_sensor_pairs: Vec<(u64, u64)>,
}
impl Default for World2 {
    fn default() -> Self {
        Self {
            gravity: Vec2 { x: 0.0, y: -9.81 },
            bodies: Pool::default(),
            next_id: 1,
            step: 0,
            dynamic_tree: DynamicTree2::new(0.02, 1.0 / 60.0).unwrap(),
            solver_iterations: 10,
            sleep_threshold: 1.0e-4,
            restitution_mode: CombineMode::Average,
            friction_mode: CombineMode::Average,
            prev_manifolds: Vec::new(),
            prev_sensor_pairs: Vec::new(),
            sensor_events: VecDeque::new(),
            joints: Vec::new(),
            joint_break_events: VecDeque::new(),
            scratch_pairs: Vec::new(),
            scratch_handles: Vec::new(),
            scratch_constraints: Vec::new(),
            scratch_raw_contacts: Vec::new(),
            scratch_id_to_h: HashMap::new(),
            scratch_sensor_pairs: Vec::new(),
        }
    }
}
struct PairChunkTask2<'a> {
    pairs: &'a [(u64, u64)],
    id_to_h: &'a HashMap<u64, BodyHandle2>,
    bodies: &'a Pool<Body2>,
    prev_sensor_pairs: &'a [(u64, u64)],
    sensor_pairs: Vec<(u64, u64)>,
    sensor_events: Vec<SensorEvent>,
    raw_contacts: Vec<(
        BodyHandle2,
        BodyHandle2,
        Vec2,
        Vec<(Vec2, Real, FeatureId)>,
        Real,
        Real,
    )>,
}

fn process_chunk2(task: &mut PairChunkTask2) {
    for &(ida, idb) in task.pairs {
        let ha = match task.id_to_h.get(&ida) {
            Some(&h) => h,
            None => continue,
        };
        let hb = match task.id_to_h.get(&idb) {
            Some(&h) => h,
            None => continue,
        };
        let (ba, bb) = match (task.bodies.get(ha), task.bodies.get(hb)) {
            (Some(a), Some(b)) => (a, b),
            _ => continue,
        };
        if ba.sleeping && bb.sleeping {
            continue;
        }
        let mut decision = PairDecision::Ignore;
        for ca in &ba.colliders {
            for cb in &bb.colliders {
                let d = ca.filter.decide(cb.filter);
                if d != PairDecision::Ignore {
                    decision = d;
                    break;
                }
            }
            if decision != PairDecision::Ignore {
                break;
            }
        }
        if decision == PairDecision::Ignore {
            continue;
        }
        if decision == PairDecision::Trigger {
            let key = if ida <= idb { (ida, idb) } else { (idb, ida) };
            task.sensor_pairs.push(key);
            if !task.prev_sensor_pairs.contains(&key) {
                task.sensor_events.push(SensorEvent {
                    sensor: ida,
                    other: idb,
                    began: true,
                    is_stay: false,
                });
            }
            continue;
        }
        for (ia, ca) in ba.colliders.iter().enumerate() {
            for (ib, cb) in bb.colliders.iter().enumerate() {
                if let Some((n, pen, p)) = generic_convex_contact_2d(ca, cb, ba, bb) {
                    let pts = generate_clip_points_2d(ca, cb, ba, bb, n, p, pen, ia, ib);
                    let rest = ca.material.restitution.max(cb.material.restitution);
                    let fric = (ca.material.friction * cb.material.friction).sqrt();
                    task.raw_contacts.push((ha, hb, n, pts, rest, fric));
                }
            }
        }
    }
}

impl World2 {
    /// step — performs step operation.
    pub fn step(&mut self, dt: Real) -> Result<(), WorldError> {
        #[cfg(feature = "multithread")]
        {
            self.step_with_scheduler(dt, &mut auralite_core::ThreadPoolScheduler)
        }
        #[cfg(not(feature = "multithread"))]
        {
            self.step_with_scheduler(dt, &mut auralite_core::SingleThreadScheduler)
        }
    }

    /// step_with_scheduler — performs step with scheduler operation.
    pub fn step_with_scheduler(
        &mut self,
        dt: Real,
        scheduler: &mut impl auralite_core::Scheduler,
    ) -> Result<(), WorldError> {
        if !(dt > 0.0 && dt.is_finite()) {
            return Err(WorldError::InvalidInput);
        }
        self.scratch_handles.clear();
        self.scratch_handles
            .extend(self.bodies.iter().map(|(h, _)| h));
        let num_handles = self.scratch_handles.len();
        for i in 0..num_handles {
            let h = self.scratch_handles[i];
            if let Some(b) = self.bodies.get_mut(h) {
                if b.kind == BodyType::Dynamic && !b.sleeping {
                    b.velocity += (self.gravity + b.force * b.inv_mass) * dt;
                    b.angular_velocity += b.torque * b.inv_inertia * dt;
                    if b.linear_damping > 0.0 {
                        b.velocity *= (1.0 - b.linear_damping * dt).max(0.0);
                    }
                    if b.angular_damping > 0.0 {
                        b.angular_velocity *= (1.0 - b.angular_damping * dt).max(0.0);
                    }
                    if !b.velocity.is_finite() {
                        return Err(WorldError::InvalidInput);
                    }
                    b.force = Vec2::ZERO;
                    b.torque = 0.0;
                }
            }
        }
        self.dynamic_tree = DynamicTree2::new(0.02, dt).unwrap();
        for i in 0..num_handles {
            let h = self.scratch_handles[i];
            if let Some(b) = self.bodies.get(h) {
                self.dynamic_tree.update(b.id.0, b.world_aabb(), b.velocity);
            }
        }
        self.dynamic_tree.collect_pairs(&mut self.scratch_pairs);
        self.scratch_constraints.clear();
        self.scratch_id_to_h.clear();
        for i in 0..num_handles {
            let h = self.scratch_handles[i];
            if let Some(b) = self.bodies.get(h) {
                self.scratch_id_to_h.insert(b.id.0, h);
            }
        }
        let mut current_sensor_pairs = Vec::new();
        self.scratch_raw_contacts.clear();

        if self.scratch_pairs.len() > 16 {
            let chunk_size = self.scratch_pairs.len().div_ceil(4);
            let mut tasks: Vec<PairChunkTask2> = self
                .scratch_pairs
                .chunks(chunk_size.max(1))
                .map(|chunk| PairChunkTask2 {
                    pairs: chunk,
                    id_to_h: &self.scratch_id_to_h,
                    bodies: &self.bodies,
                    prev_sensor_pairs: &self.prev_sensor_pairs,
                    sensor_pairs: Vec::new(),
                    sensor_events: Vec::new(),
                    raw_contacts: Vec::new(),
                })
                .collect();
            scheduler.run_slice(&mut tasks, process_chunk2);
            for mut task in tasks {
                current_sensor_pairs.append(&mut task.sensor_pairs);
                for e in task.sensor_events {
                    self.sensor_events.push_back(e);
                }
                self.scratch_raw_contacts.append(&mut task.raw_contacts);
            }
        } else {
            let mut task = PairChunkTask2 {
                pairs: &self.scratch_pairs,
                id_to_h: &self.scratch_id_to_h,
                bodies: &self.bodies,
                prev_sensor_pairs: &self.prev_sensor_pairs,
                sensor_pairs: Vec::new(),
                sensor_events: Vec::new(),
                raw_contacts: Vec::new(),
            };
            process_chunk2(&mut task);
            current_sensor_pairs = task.sensor_pairs;
            for e in task.sensor_events {
                self.sensor_events.push_back(e);
            }
            self.scratch_raw_contacts = task.raw_contacts;
        }

        for i in 0..self.scratch_raw_contacts.len() {
            let (ha, hb, n, ref pts, rest, fric) = self.scratch_raw_contacts[i];
            for &(cp, cpen, cfid) in pts {
                let mut cc = ContactConstraint2::new(ha, hb, n, cp, cpen, rest, fric, cfid, self);
                for pm in &self.prev_manifolds {
                    cc.warm_start(pm, &mut self.bodies);
                }
                self.scratch_constraints.push(cc);
            }
        }
        for _ in 0..self.solver_iterations {
            solve_contacts_2d_once(&mut self.scratch_constraints, &mut self.bodies);
            for j in &mut self.joints {
                j.solve(&mut self.bodies);
            }
        }
        for i in 0..num_handles {
            let h = self.scratch_handles[i];
            if let Some(b) = self.bodies.get_mut(h) {
                if b.kind == BodyType::Dynamic && !b.sleeping {
                    b.position += b.velocity * dt;
                    if !b.position.is_finite() {
                        return Err(WorldError::InvalidInput);
                    }
                    let angle = rot2_angle(b.rotation) + b.angular_velocity * dt;
                    b.rotation = Rot2::from_radians(angle).unwrap_or(b.rotation);
                    let mut max_p: Real = 0.0;
                    for c in &b.colliders {
                        let r = match &c.shape {
                            ColliderShape2::Circle(circ) => circ.radius(),
                            ColliderShape2::Box(bx) => bx.half_extents().y,
                            ColliderShape2::Capsule(cap) => cap.radius + cap.half_height,
                            _ => 0.0,
                        };
                        let wy = b.position.y + b.rotation.rotate(c.offset).y;
                        if r - wy > max_p {
                            let p = r - wy;
                            b.position.y += p;
                            if b.velocity.y < 0.0 {
                                b.velocity.y = -b.velocity.y * b.restitution;
                            } else if b.velocity.length_squared() < self.sleep_threshold
                                && b.angular_velocity.abs() < self.sleep_threshold
                            {
                                b.sleeping = true;
                                b.velocity = Vec2::ZERO;
                                b.angular_velocity = 0.0;
                            }
                            max_p = max_p.max(p);
                        }
                    }
                    let has_contact_support = max_p > 0.0
                        || self.scratch_constraints.iter().any(|c| {
                            (c.body_a == h || c.body_b == h)
                                && c.normal_impulse > crate::ABS_EPSILON
                        });
                    if has_contact_support
                        && b.velocity.length_squared() < self.sleep_threshold
                        && b.angular_velocity.abs() < self.sleep_threshold
                    {
                        b.sleeping = true;
                        b.velocity = Vec2::ZERO;
                        b.angular_velocity = 0.0;
                    }
                }
            }
        }
        self.prev_manifolds.clear();
        self.prev_manifolds.extend(
            self.scratch_constraints.iter().map(|c| {
                Manifold2::from_clip(c.normal, vec![(c.point, c.penetration, c.feature_id)])
            }),
        );
        // H6: emit stay events for ongoing pairs (intersection) in deterministic sorted order
        {
            let mut staying: Vec<(u64, u64)> = Vec::new();
            for key in &self.prev_sensor_pairs {
                if current_sensor_pairs.contains(key) {
                    staying.push(*key);
                }
            }
            staying.sort_unstable();
            for (s, o) in staying {
                self.sensor_events.push_back(SensorEvent {
                    sensor: s,
                    other: o,
                    began: false,
                    is_stay: true,
                });
            }
        }
        for prev in &self.prev_sensor_pairs {
            if !current_sensor_pairs.contains(prev) {
                self.sensor_events.push_back(SensorEvent {
                    sensor: prev.0,
                    other: prev.1,
                    began: false,
                    is_stay: false,
                });
            }
        }
        self.scratch_sensor_pairs.clear();
        self.scratch_sensor_pairs.extend(current_sensor_pairs);
        core::mem::swap(&mut self.prev_sensor_pairs, &mut self.scratch_sensor_pairs);
        self.step += 1;
        Ok(())
    }
    /// add_body — performs add body operation.
    pub fn add_body(&mut self, b: BodyBuilder2) -> Result<BodyHandle2, WorldError> {
        let id = StableId(self.next_id);
        self.next_id += 1;
        Ok(self.bodies.insert(Body2 {
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
                1.0 / b.inertia.unwrap_or(0.1)
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
        }))
    }
    /// body — performs body operation.
    pub fn body(&self, h: BodyHandle2) -> Result<&Body2, WorldError> {
        self.bodies.get(h).ok_or(WorldError::StaleHandle)
    }
    /// body_mut — performs body mut operation.
    pub fn body_mut(&mut self, h: BodyHandle2) -> Result<&mut Body2, WorldError> {
        self.bodies.get_mut(h).ok_or(WorldError::StaleHandle)
    }
    /// add_joint — performs add joint operation.
    pub fn add_joint(&mut self, config: JointConfig2) -> Result<JointId, WorldError> {
        let id = JointId(self.next_id);
        self.next_id += 1;
        self.joints.push(Joint2::new(id, config));
        Ok(id)
    }
    /// joint — performs joint operation.
    pub fn joint(&self, id: JointId) -> Option<&Joint2> {
        self.joints.iter().find(|j| j.id == id)
    }
    /// remove_body — performs remove body operation.
    pub fn remove_body(&mut self, h: BodyHandle2) -> Result<Body2, WorldError> {
        self.bodies.remove(h).ok_or(WorldError::StaleHandle)
    }
    /// apply_force — performs apply force operation.
    pub fn apply_force(&mut self, h: BodyHandle2, f: Vec2) -> Result<(), WorldError> {
        if let Some(b) = self.bodies.get_mut(h) {
            b.force += f;
            b.sleeping = false;
        }
        Ok(())
    }
    /// apply_impulse — performs apply impulse operation.
    pub fn apply_impulse(&mut self, h: BodyHandle2, j: Vec2) -> Result<(), WorldError> {
        if let Some(b) = self.bodies.get_mut(h) {
            b.velocity += j * b.inv_mass;
            b.sleeping = false;
        }
        Ok(())
    }
    /// state_hash — performs state hash operation.
    pub fn state_hash(&self) -> u64 {
        let mut b = Vec::new();
        b.extend_from_slice(&self.step.to_le_bytes());
        for (_, body) in self.bodies.iter() {
            b.extend_from_slice(&body.id.0.to_le_bytes());
            b.extend_from_slice(&body.position.x.to_bits().to_le_bytes());
            b.extend_from_slice(&body.position.y.to_bits().to_le_bytes());
            b.extend_from_slice(&rot2_angle(body.rotation).to_bits().to_le_bytes());
            b.extend_from_slice(&body.velocity.x.to_bits().to_le_bytes());
            b.extend_from_slice(&body.velocity.y.to_bits().to_le_bytes());
            b.extend_from_slice(&body.angular_velocity.to_bits().to_le_bytes());
            b.push(u8::from(body.sleeping));
            b.push(body.kind as u8);
        }
        hash_bytes(&b)
    }
    /// wake_body — performs wake body operation.
    pub fn wake_body(&mut self, h: BodyHandle2) -> Result<(), WorldError> {
        if let Some(b) = self.bodies.get_mut(h) {
            b.sleeping = false;
        }
        Ok(())
    }
    /// snapshot — performs snapshot operation.
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
    /// restore — performs restore operation.
    pub fn restore(&mut self, s: &Snapshot2) -> Result<(), WorldError> {
        self.step = s.step;
        for (_, b) in self.bodies.iter_mut() {
            if let Some(e) = s.states.iter().find(|x| x.0 == b.id.0) {
                b.position = e.1;
                b.rotation = e.2;
                b.velocity = e.3;
                b.angular_velocity = e.4;
                b.restitution = e.5;
                b.friction = e.6;
                b.sleeping = e.7;
            }
        }
        Ok(())
    }
    /// body_handles — performs body handles operation.
    pub fn body_handles(&self) -> Vec<BodyHandle2> {
        self.bodies.iter().map(|(h, _)| h).collect()
    }
    /// body_count — performs body count operation.
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }
    /// step_count — performs step count operation.
    pub fn step_count(&self) -> u64 {
        self.step
    }
    /// set_gravity — performs set gravity operation.
    pub fn set_gravity(&mut self, g: Vec2) -> Result<(), WorldError> {
        self.gravity = g;
        Ok(())
    }
    /// gravity — performs gravity operation.
    pub fn gravity(&self) -> Vec2 {
        self.gravity
    }
    /// bodies_iter — performs bodies iter operation.
    pub fn bodies_iter(&self) -> impl Iterator<Item = (BodyHandle2, &Body2)> {
        self.bodies.iter()
    }
    /// set_step_count — performs set step count operation.
    pub fn set_step_count(&mut self, step: u64) {
        self.step = step;
    }
    /// insert_restored_body — performs insert restored body operation.
    pub fn insert_restored_body(&mut self, body: Body2) -> BodyHandle2 {
        if body.id.0 >= self.next_id {
            self.next_id = body.id.0 + 1;
        }
        self.bodies.insert(body)
    }
    /// rebuild_tree — performs rebuild tree operation.
    pub fn rebuild_tree(&mut self) {
        self.dynamic_tree = DynamicTree2::new(0.02, 1.0 / 60.0).unwrap();
        for (_, b) in self.bodies.iter() {
            self.dynamic_tree.update(b.id.0, b.world_aabb(), b.velocity);
        }
    }
    /// serialize_bodies — performs serialize bodies operation.
    pub fn serialize_bodies(&self) -> Vec<u8> {
        let mut b = Vec::new();
        for (_, body) in self.bodies.iter() {
            b.extend_from_slice(&body.id.0.to_le_bytes());
            b.push(body.kind as u8);
            b.extend_from_slice(&body.position.x.to_bits().to_le_bytes());
            b.extend_from_slice(&body.position.y.to_bits().to_le_bytes());
            b.extend_from_slice(&rot2_angle(body.rotation).to_bits().to_le_bytes());
            b.extend_from_slice(&body.velocity.x.to_bits().to_le_bytes());
            b.extend_from_slice(&body.velocity.y.to_bits().to_le_bytes());
            b.extend_from_slice(&body.angular_velocity.to_bits().to_le_bytes());
            b.extend_from_slice(&body.inv_mass.to_bits().to_le_bytes());
            b.extend_from_slice(&body.inv_inertia.to_bits().to_le_bytes());
            b.extend_from_slice(&body.restitution.to_bits().to_le_bytes());
            b.extend_from_slice(&body.friction.to_bits().to_le_bytes());
            b.push(u8::from(body.sleeping));
        }
        b
    }
    /// serialize_joints — performs serialize joints operation.
    pub fn serialize_joints(&self) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&(self.joints.len() as u32).to_le_bytes());
        for j in &self.joints {
            b.push(match j.config.joint_type {
                JointType2::Weld => 0,
                JointType2::Distance => 1,
                JointType2::Spring { .. } => 2,
                JointType2::Revolute => 3,
                JointType2::Prismatic { .. } => 4,
            });
            b.extend_from_slice(&j.config.anchor_a.x.to_bits().to_le_bytes());
            b.extend_from_slice(&j.config.anchor_a.y.to_bits().to_le_bytes());
            b.extend_from_slice(&j.config.anchor_b.x.to_bits().to_le_bytes());
            b.extend_from_slice(&j.config.anchor_b.y.to_bits().to_le_bytes());
            b.extend_from_slice(&j.config.break_impulse.to_bits().to_le_bytes());
            b.push(u8::from(j.broken));
            b.extend_from_slice(&j.impulse.to_bits().to_le_bytes());
            b.extend_from_slice(&j.accumulated_position_error.to_bits().to_le_bytes());
        }
        b
    }
    /// ray_cast — performs ray cast operation.
    pub fn ray_cast(&self, ray: Ray2, max_t: Real) -> Option<(BodyHandle2, Real, Vec2)> {
        self.ray_cast_ignoring(ray, max_t, BodyHandle2::new(u32::MAX, u32::MAX))
    }
    /// ray_cast_ignoring — performs ray cast ignoring operation.
    pub fn ray_cast_ignoring(
        &self,
        ray: Ray2,
        max_t: Real,
        ignore: BodyHandle2,
    ) -> Option<(BodyHandle2, Real, Vec2)> {
        let mut best_t = max_t;
        let mut best_hit = None;
        let candidates = self.dynamic_tree.ray_cast(ray, max_t);
        for id in candidates {
            if let Some((h, b)) = self.bodies.iter().find(|(_, b)| b.id.0 == id) {
                if h == ignore {
                    continue;
                }
                for c in &b.colliders {
                    let center = b.position + b.rotation.rotate(c.offset);
                    let local_origin = b.rotation.inverse().rotate(ray.origin - center);
                    let local_dir = b.rotation.inverse().rotate(ray.direction);
                    if let Ok(local_ray) = Ray2::new(local_origin, local_dir) {
                        if let Some((t, local_normal)) = c.shape.ray_intersection(local_ray) {
                            if t >= 0.0 && t < best_t {
                                best_t = t;
                                let world_normal =
                                    b.rotation.rotate(local_normal).normalized_or(Vec2::Y);
                                best_hit = Some((h, t, world_normal));
                            }
                        }
                    }
                }
            }
        }
        best_hit
    }
}

/// A 3D rigid body world containing bodies, joints, and spatial tree.
///
/// # Example
/// ```
/// use auralite_dynamics::{World3, BodyBuilder3};
/// use auralite_math::Vec3;
///
/// let mut world = World3::default();
/// let body = world.add_body(BodyBuilder3::dynamic().position(Vec3 { x: 0.0, y: 10.0, z: 0.0 })).unwrap();
/// world.step(0.016).unwrap();
/// assert!(world.body(body).unwrap().position.y < 10.0);
/// ```
#[derive(Clone)]
pub struct World3 {
    gravity: Vec3,
    bodies: Pool<Body3>,
    next_id: u64,
    step: u64,
    dynamic_tree: DynamicTree3,
    /// solver_iterations field.
    pub solver_iterations: u16,
    /// sleep_threshold field.
    pub sleep_threshold: Real,
    /// joints field.
    pub joints: Vec<Joint3>,
    /// prev_manifolds field.
    pub prev_manifolds: Vec<Manifold3>,
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
            dynamic_tree: DynamicTree3::new(0.02, 1.0 / 60.0).unwrap(),
            solver_iterations: 10,
            sleep_threshold: 1.0e-6,
            joints: Vec::new(),
            prev_manifolds: Vec::new(),
        }
    }
}
struct PairChunkTask3<'a> {
    pairs: &'a [(u64, u64)],
    id_to_h: &'a HashMap<u64, BodyHandle3>,
    bodies: &'a Pool<Body3>,
    raw_contacts: Vec<(
        BodyHandle3,
        BodyHandle3,
        Vec3,
        Vec<(Vec3, Real, FeatureId)>,
        Real,
        Real,
    )>,
}

fn process_chunk3(task: &mut PairChunkTask3) {
    for &(ida, idb) in task.pairs {
        let ha = match task.id_to_h.get(&ida) {
            Some(&h) => h,
            None => continue,
        };
        let hb = match task.id_to_h.get(&idb) {
            Some(&h) => h,
            None => continue,
        };
        let (ba, bb) = match (task.bodies.get(ha), task.bodies.get(hb)) {
            (Some(a), Some(b)) => (a, b),
            _ => continue,
        };
        if ba.sleeping && bb.sleeping {
            continue;
        }
        for (ia, ca) in ba.colliders.iter().enumerate() {
            for (ib, cb) in bb.colliders.iter().enumerate() {
                if let Some((n, pen, p)) = generic_convex_contact_3d(ca, cb, ba, bb) {
                    let pts = generate_clip_points_3d(ca, cb, ba, bb, n, p, pen, ia, ib);
                    let rest = ca.material.restitution.max(cb.material.restitution);
                    let fric = (ca.material.friction * cb.material.friction).sqrt();
                    task.raw_contacts.push((ha, hb, n, pts, rest, fric));
                }
            }
        }
    }
}

impl World3 {
    /// step — performs step operation.
    pub fn step(&mut self, dt: Real) -> Result<(), WorldError> {
        #[cfg(feature = "multithread")]
        {
            self.step_with_scheduler(dt, &mut auralite_core::ThreadPoolScheduler)
        }
        #[cfg(not(feature = "multithread"))]
        {
            self.step_with_scheduler(dt, &mut auralite_core::SingleThreadScheduler)
        }
    }

    /// step_with_scheduler — performs step with scheduler operation.
    pub fn step_with_scheduler(
        &mut self,
        dt: Real,
        scheduler: &mut impl auralite_core::Scheduler,
    ) -> Result<(), WorldError> {
        let handles: Vec<BodyHandle3> = self.bodies.iter().map(|(h, _)| h).collect();
        for &h in &handles {
            if let Some(b) = self.bodies.get_mut(h) {
                if b.kind == BodyType::Dynamic && !b.sleeping {
                    b.velocity += (self.gravity + b.force * b.inv_mass) * dt;
                    b.angular_velocity += b.torque * b.inv_inertia_diagonal * dt;
                    if b.linear_damping > 0.0 {
                        b.velocity *= (1.0 - b.linear_damping * dt).max(0.0);
                    }
                    if b.angular_damping > 0.0 {
                        b.angular_velocity *= (1.0 - b.angular_damping * dt).max(0.0);
                    }
                    b.force = Vec3::ZERO;
                    b.torque = Vec3::ZERO;
                }
            }
        }
        self.dynamic_tree = DynamicTree3::new(0.02, dt).unwrap();
        for &h in &handles {
            if let Some(b) = self.bodies.get(h) {
                self.dynamic_tree.update(b.id.0, b.world_aabb(), b.velocity);
            }
        }
        let pairs = self.dynamic_tree.pairs();
        let mut constraints = Vec::new();
        let mut id_to_h = HashMap::new();
        for &h in &handles {
            if let Some(b) = self.bodies.get(h) {
                id_to_h.insert(b.id.0, h);
            }
        }
        let mut raw_contacts = Vec::new();

        if pairs.len() > 16 {
            let chunk_size = pairs.len().div_ceil(4);
            let mut tasks: Vec<PairChunkTask3> = pairs
                .chunks(chunk_size.max(1))
                .map(|chunk| PairChunkTask3 {
                    pairs: chunk,
                    id_to_h: &id_to_h,
                    bodies: &self.bodies,
                    raw_contacts: Vec::new(),
                })
                .collect();
            scheduler.run_slice(&mut tasks, process_chunk3);
            for mut task in tasks {
                raw_contacts.append(&mut task.raw_contacts);
            }
        } else {
            let mut task = PairChunkTask3 {
                pairs: &pairs,
                id_to_h: &id_to_h,
                bodies: &self.bodies,
                raw_contacts: Vec::new(),
            };
            process_chunk3(&mut task);
            raw_contacts = task.raw_contacts;
        }

        for (ha, hb, n, pts, rest, fric) in raw_contacts {
            for (cp, cpen, cfid) in pts {
                let mut cc = ContactConstraint3::new(ha, hb, n, cp, cpen, rest, fric, cfid, self);
                for pm in &self.prev_manifolds {
                    cc.warm_start(pm, &mut self.bodies);
                }
                constraints.push(cc);
            }
        }
        for _ in 0..self.solver_iterations {
            solve_contacts_3d_once(&mut constraints, &mut self.bodies);
            for j in &mut self.joints {
                j.solve(&mut self.bodies);
            }
        }
        self.prev_manifolds = constraints
            .iter()
            .map(|c| Manifold3::from_clip(c.normal, vec![(c.point, c.penetration, c.feature_id)]))
            .collect();
        for &h in &handles {
            if let Some(b) = self.bodies.get_mut(h) {
                if b.kind == BodyType::Dynamic && !b.sleeping {
                    b.position += b.velocity * dt;
                    let ang_vel_q = Quat {
                        x: b.angular_velocity.x,
                        y: b.angular_velocity.y,
                        z: b.angular_velocity.z,
                        w: 0.0,
                    };
                    let dq = ang_vel_q * b.rotation;
                    b.rotation = Quat {
                        x: b.rotation.x + dq.x * 0.5 * dt,
                        y: b.rotation.y + dq.y * 0.5 * dt,
                        z: b.rotation.z + dq.z * 0.5 * dt,
                        w: b.rotation.w + dq.w * 0.5 * dt,
                    }
                    .normalized_or(Quat::identity());
                    let mut max_p: Real = 0.0;
                    for c in &b.colliders {
                        let r = c.bounding_radius();
                        let wy = b.position.y + b.rotation.rotate(c.offset).y;
                        if wy < r {
                            let p = r - wy;
                            b.position.y += p;
                            if b.velocity.y < 0.0 {
                                b.velocity.y = -b.velocity.y * b.restitution;
                            } else if b.velocity.length_squared() < self.sleep_threshold
                                && b.angular_velocity.length_squared() < self.sleep_threshold
                            {
                                b.sleeping = true;
                                b.velocity = Vec3::ZERO;
                                b.angular_velocity = Vec3::ZERO;
                            }
                            max_p = max_p.max(p);
                        }
                    }
                    let has_contact_support = max_p > 0.0;
                    if has_contact_support
                        && b.velocity.length_squared() < self.sleep_threshold
                        && b.angular_velocity.length_squared() < self.sleep_threshold
                    {
                        b.sleeping = true;
                        b.velocity = Vec3::ZERO;
                        b.angular_velocity = Vec3::ZERO;
                    }
                }
            }
        }
        self.step += 1;
        Ok(())
    }
    /// add_body — performs add body operation.
    pub fn add_body(&mut self, b: BodyBuilder3) -> Result<BodyHandle3, WorldError> {
        let id = StableId(self.next_id);
        self.next_id += 1;
        Ok(self.bodies.insert(Body3 {
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
            inv_inertia_diagonal: b.inertia_diagonal.unwrap_or(Vec3 {
                x: 10.0,
                y: 10.0,
                z: 10.0,
            }),
            colliders: b.colliders,
            restitution: b.restitution,
            friction: b.friction,
            sleeping: false,
            force: Vec3::ZERO,
            torque: Vec3::ZERO,
            linear_damping: b.linear_damping,
            angular_damping: b.angular_damping,
            user_data: b.user_data,
        }))
    }
    /// body — performs body operation.
    pub fn body(&self, h: BodyHandle3) -> Result<&Body3, WorldError> {
        self.bodies.get(h).ok_or(WorldError::StaleHandle)
    }
    /// body_handles — performs body handles operation.
    pub fn body_handles(&self) -> Vec<BodyHandle3> {
        self.bodies.iter().map(|(h, _)| h).collect()
    }
    /// body_count — performs body count operation.
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }
    /// step_count — performs step count operation.
    pub fn step_count(&self) -> u64 {
        self.step
    }
    /// gravity — performs gravity operation.
    pub fn gravity(&self) -> Vec3 {
        self.gravity
    }
    /// bodies_iter — performs bodies iter operation.
    pub fn bodies_iter(&self) -> impl Iterator<Item = (BodyHandle3, &Body3)> {
        self.bodies.iter()
    }
    /// set_gravity — performs set gravity operation.
    pub fn set_gravity(&mut self, g: Vec3) -> Result<(), WorldError> {
        self.gravity = g;
        Ok(())
    }
    /// set_step_count — performs set step count operation.
    pub fn set_step_count(&mut self, step: u64) {
        self.step = step;
    }
    /// insert_restored_body — performs insert restored body operation.
    pub fn insert_restored_body(&mut self, body: Body3) -> BodyHandle3 {
        if body.id.0 >= self.next_id {
            self.next_id = body.id.0 + 1;
        }
        self.bodies.insert(body)
    }
    /// rebuild_tree — performs rebuild tree operation.
    pub fn rebuild_tree(&mut self) {
        self.dynamic_tree = DynamicTree3::new(0.02, 1.0 / 60.0).unwrap();
        for (_, b) in self.bodies.iter() {
            self.dynamic_tree.update(b.id.0, b.world_aabb(), b.velocity);
        }
    }
    /// snapshot — performs snapshot operation.
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
    /// restore — performs restore operation.
    pub fn restore(&mut self, s: &Snapshot3) -> Result<(), WorldError> {
        self.step = s.step;
        for (_, b) in self.bodies.iter_mut() {
            if let Some(e) = s.states.iter().find(|x| x.0 == b.id.0) {
                b.position = e.1;
                b.rotation = e.2;
                b.velocity = e.3;
                b.angular_velocity = e.4;
                b.restitution = e.5;
                b.inv_inertia_diagonal = e.6;
                b.sleeping = e.7;
            }
        }
        Ok(())
    }
    /// state_hash — performs state hash operation.
    pub fn state_hash(&self) -> u64 {
        let mut b = Vec::new();
        b.extend_from_slice(&self.step.to_le_bytes());
        for (_, body) in self.bodies.iter() {
            b.extend_from_slice(&body.id.0.to_le_bytes());
            b.extend_from_slice(&body.position.x.to_bits().to_le_bytes());
            b.extend_from_slice(&body.position.y.to_bits().to_le_bytes());
            b.extend_from_slice(&body.position.z.to_bits().to_le_bytes());
            b.extend_from_slice(&body.rotation.x.to_bits().to_le_bytes());
            b.extend_from_slice(&body.rotation.y.to_bits().to_le_bytes());
            b.extend_from_slice(&body.rotation.z.to_bits().to_le_bytes());
            b.extend_from_slice(&body.rotation.w.to_bits().to_le_bytes());
            b.extend_from_slice(&body.velocity.x.to_bits().to_le_bytes());
            b.extend_from_slice(&body.velocity.y.to_bits().to_le_bytes());
            b.extend_from_slice(&body.velocity.z.to_bits().to_le_bytes());
            b.extend_from_slice(&body.angular_velocity.x.to_bits().to_le_bytes());
            b.extend_from_slice(&body.angular_velocity.y.to_bits().to_le_bytes());
            b.extend_from_slice(&body.angular_velocity.z.to_bits().to_le_bytes());
            b.push(u8::from(body.sleeping));
            b.push(body.kind as u8);
        }
        hash_bytes(&b)
    }
    /// add_joint — performs add joint operation.
    pub fn add_joint(&mut self, config: JointConfig3) -> Result<JointId, WorldError> {
        let id = JointId(self.next_id);
        self.next_id += 1;
        self.joints.push(Joint3::new(id, config));
        Ok(id)
    }
    /// joint — performs joint operation.
    pub fn joint(&self, id: JointId) -> Option<&Joint3> {
        self.joints.iter().find(|j| j.id == id)
    }
    /// remove_joint — performs remove joint operation.
    pub fn remove_joint(&mut self, id: JointId) {
        self.joints.retain(|j| j.id != id);
    }
    /// remove_body — performs remove body operation.
    pub fn remove_body(&mut self, h: BodyHandle3) -> Result<Body3, WorldError> {
        self.bodies.remove(h).ok_or(WorldError::StaleHandle)
    }
    /// apply_force — performs apply force operation.
    pub fn apply_force(&mut self, h: BodyHandle3, f: Vec3) -> Result<(), WorldError> {
        if let Some(b) = self.bodies.get_mut(h) {
            b.force += f;
        }
        Ok(())
    }
    /// apply_impulse — performs apply impulse operation.
    pub fn apply_impulse(&mut self, h: BodyHandle3, j: Vec3) -> Result<(), WorldError> {
        if let Some(b) = self.bodies.get_mut(h) {
            b.velocity += j * b.inv_mass;
            b.sleeping = false;
        }
        Ok(())
    }
    /// apply_impulse_at_point — performs apply impulse at point operation.
    pub fn apply_impulse_at_point(
        &mut self,
        h: BodyHandle3,
        impulse: Vec3,
        point: Vec3,
    ) -> Result<(), WorldError> {
        if let Some(b) = self.bodies.get_mut(h) {
            b.apply_impulse(impulse, point);
            b.sleeping = false;
        }
        Ok(())
    }
    /// serialize_bodies — performs serialize bodies operation.
    pub fn serialize_bodies(&self) -> Vec<u8> {
        let mut b = Vec::new();
        for (_, body) in self.bodies.iter() {
            b.extend_from_slice(&body.id.0.to_le_bytes());
            b.push(body.kind as u8);
            b.extend_from_slice(&body.position.x.to_bits().to_le_bytes());
            b.extend_from_slice(&body.position.y.to_bits().to_le_bytes());
            b.extend_from_slice(&body.position.z.to_bits().to_le_bytes());
            b.extend_from_slice(&body.velocity.x.to_bits().to_le_bytes());
            b.extend_from_slice(&body.velocity.y.to_bits().to_le_bytes());
            b.extend_from_slice(&body.velocity.z.to_bits().to_le_bytes());
            b.extend_from_slice(&body.restitution.to_bits().to_le_bytes());
            b.push(u8::from(body.sleeping));
        }
        b
    }
    /// serialize_joints — performs serialize joints operation.
    pub fn serialize_joints(&self) -> Vec<u8> {
        Vec::new()
    }
    /// ray_cast — performs ray cast operation.
    pub fn ray_cast(&self, ray: Ray3, max_t: Real) -> Option<(BodyHandle3, Real, Vec3)> {
        self.ray_cast_ignoring(ray, max_t, BodyHandle3::new(u32::MAX, u32::MAX))
    }
    /// ray_cast_ignoring — performs ray cast ignoring operation.
    pub fn ray_cast_ignoring(
        &self,
        ray: Ray3,
        max_t: Real,
        ignore: BodyHandle3,
    ) -> Option<(BodyHandle3, Real, Vec3)> {
        let mut best_t = max_t;
        let mut best_hit = None;
        let candidates = self.dynamic_tree.ray_cast(ray, max_t);
        for id in candidates {
            if let Some((h, b)) = self.bodies.iter().find(|(_, b)| b.id.0 == id) {
                if h == ignore {
                    continue;
                }
                for c in &b.colliders {
                    let center = b.position + b.rotation.rotate(c.offset);
                    let local_origin = b.rotation.inverse().rotate(ray.origin - center);
                    let local_dir = b.rotation.inverse().rotate(ray.direction);
                    if let Ok(local_ray) = Ray3::new(local_origin, local_dir) {
                        if let Some((t, local_normal)) = c.shape.ray_intersection(local_ray) {
                            if t >= 0.0 && t < best_t {
                                best_t = t;
                                let world_normal =
                                    b.rotation.rotate(local_normal).normalized_or(Vec3::Y);
                                best_hit = Some((h, t, world_normal));
                            }
                        }
                    }
                }
            }
        }
        best_hit
    }
}

fn rot2_angle(r: Rot2) -> Real {
    let v = r.rotate(Vec2::X);
    v.y.atan2(v.x)
}
fn generate_clip_points_2d(
    ca: &Collider2,
    cb: &Collider2,
    ba: &Body2,
    bb: &Body2,
    n: Vec2,
    p: Vec2,
    pen: Real,
    ia: usize,
    ib: usize,
) -> Vec<(Vec2, Real, FeatureId)> {
    let world_a = ba.position + ba.rotation.rotate(ca.offset);
    let world_b = bb.position + bb.rotation.rotate(cb.offset);
    let get_verts = |shape: &ColliderShape2, pos: Vec2, rot: Rot2| -> Option<Vec<Vec2>> {
        match shape {
            ColliderShape2::Box(bx) => {
                let he = bx.half_extents();
                Some(vec![
                    pos + rot.rotate(Vec2 { x: -he.x, y: -he.y }),
                    pos + rot.rotate(Vec2 { x: he.x, y: -he.y }),
                    pos + rot.rotate(Vec2 { x: he.x, y: he.y }),
                    pos + rot.rotate(Vec2 { x: -he.x, y: he.y }),
                ])
            }
            ColliderShape2::ConvexPolygon(poly) => Some(
                poly.vertices()
                    .iter()
                    .map(|&v| pos + rot.rotate(v))
                    .collect(),
            ),
            _ => None,
        }
    };
    if let (Some(va), Some(vb)) = (
        get_verts(&ca.shape, world_a, ba.rotation),
        get_verts(&cb.shape, world_b, bb.rotation),
    ) {
        let mut clipped = auralite_collision::narrow::clip_contacts2(n, &va, None, &vb, None, 0.0);
        if !clipped.is_empty() {
            for (_, cpen, _) in &mut clipped {
                *cpen = (*cpen).min(pen);
            }
            return clipped;
        }
    }
    vec![(p, pen, FeatureId((ia as u64) << 32 | ib as u64))]
}

fn generic_convex_contact_2d(
    ca: &Collider2,
    cb: &Collider2,
    ba: &Body2,
    bb: &Body2,
) -> Option<(Vec2, Real, Vec2)> {
    let world_a = ba.position + ba.rotation.rotate(ca.offset);
    let world_b = bb.position + bb.rotation.rotate(cb.offset);
    let support_a = |d: Vec2| -> Vec2 {
        let local_d = ba.rotation.inverse().rotate(d);
        world_a
            + ba.rotation.rotate(match &ca.shape {
                ColliderShape2::Circle(c) => c.support(local_d),
                ColliderShape2::Box(b) => b.support(local_d),
                ColliderShape2::Capsule(cap) => cap.support(local_d),
                ColliderShape2::ConvexPolygon(p) => p.support(local_d),
                ColliderShape2::Edge(e) => e.support(local_d),
            })
    };
    let support_b = |d: Vec2| -> Vec2 {
        let local_d = bb.rotation.inverse().rotate(d);
        world_b
            + bb.rotation.rotate(match &cb.shape {
                ColliderShape2::Circle(c) => c.support(local_d),
                ColliderShape2::Box(b) => b.support(local_d),
                ColliderShape2::Capsule(cap) => cap.support(local_d),
                ColliderShape2::ConvexPolygon(p) => p.support(local_d),
                ColliderShape2::Edge(e) => e.support(local_d),
            })
    };
    let gjk = auralite_collision::gjk_distance2(support_a, support_b, 32);
    if gjk.distance <= ABS_EPSILON {
        let epa = auralite_collision::epa_penetration2(support_a, support_b, 32);
        if let Some(p) = epa {
            let contact_pt = if gjk.distance > 0.0 && gjk.point_a.is_finite() {
                (gjk.point_a + gjk.point_b) * 0.5
            } else {
                // When overlapping in GJK, use EPA support points along contact normal
                (support_a(-p.normal) + support_b(p.normal)) * 0.5
            };
            return Some((p.normal, p.depth, contact_pt));
        }
    }
    None
}

fn generate_clip_points_3d(
    _ca: &Collider3,
    _cb: &Collider3,
    _ba: &Body3,
    _bb: &Body3,
    _n: Vec3,
    p: Vec3,
    pen: Real,
    ia: usize,
    ib: usize,
) -> Vec<(Vec3, Real, FeatureId)> {
    vec![(p, pen, FeatureId((ia as u64) << 32 | ib as u64))]
}

fn generic_convex_contact_3d(
    ca: &Collider3,
    cb: &Collider3,
    ba: &Body3,
    bb: &Body3,
) -> Option<(Vec3, Real, Vec3)> {
    let world_a = ba.position + ba.rotation.rotate(ca.offset);
    let world_b = bb.position + bb.rotation.rotate(cb.offset);
    let support_a = |d: Vec3| -> Vec3 {
        let local_d = ba.rotation.inverse().rotate(d);
        world_a + ba.rotation.rotate(ca.shape.support(local_d))
    };
    let support_b = |d: Vec3| -> Vec3 {
        let local_d = bb.rotation.inverse().rotate(d);
        world_b + bb.rotation.rotate(cb.shape.support(local_d))
    };
    let gjk = auralite_collision::gjk_distance3(support_a, support_b, 32);
    if gjk.distance <= ABS_EPSILON {
        let epa = auralite_collision::epa_penetration3(support_a, support_b, 32);
        if let Some(p) = epa {
            let contact_pt = if gjk.distance > 0.0 && gjk.point_a.is_finite() {
                (gjk.point_a + gjk.point_b) * 0.5
            } else {
                (support_a(-p.normal) + support_b(p.normal)) * 0.5
            };
            return Some((p.normal, p.depth, contact_pt));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn falling_2d_rests() {
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
        assert!((w.body(h).unwrap().position.y - 0.5).abs() < 0.1);
    }
    #[test]
    fn forces_accumulate() {
        let mut w = World2::default();
        w.gravity = Vec2::ZERO;
        let h = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 0.0, y: 10.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        w.apply_force(h, Vec2 { x: 10.0, y: 0.0 }).unwrap();
        w.step(1.0 / 60.0).unwrap();
        assert!((w.body(h).unwrap().velocity.x - 10.0 / 60.0).abs() < 1.0e-4);
    }
    #[test]
    fn distance_joint_holds_bodies_together() {
        let mut w = World2::default();
        w.gravity = Vec2::ZERO;
        let b1 = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: -1.0, y: 10.0 })
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
                    .position(Vec2 { x: 1.0, y: 10.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.2).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        w.add_joint(JointConfig2::new(
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
        let dist = (w.body(b1).unwrap().position - w.body(b2).unwrap().position).length();
        assert!(dist < 2.1, "distance should stay small, got {}", dist);
    }
    #[test]
    fn two_circles_stack() {
        let mut w = World2::default();
        w.add_body(
            BodyBuilder2::static_body()
                .position(Vec2 { x: 0.0, y: -1.0 })
                .add_collider(Collider2 {
                    shape: ColliderShape2::Circle(Circle2::new(1.0).unwrap()),
                    offset: Vec2::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
                }),
        )
        .unwrap();
        let h2 = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 0.0, y: 3.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(1.0).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        for _ in 0..600 {
            w.step(1.0 / 60.0).unwrap();
        }
        assert!(w.body(h2).unwrap().sleeping);
    }
    #[test]
    fn restitution_affects_bounce() {
        let mut w = World2::default();
        w.sleep_threshold = 0.0;
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
        for _ in 0..40 {
            w.step(1.0 / 60.0).unwrap();
        }
        assert!(w.body(h).unwrap().velocity.y > 0.0);
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
        for _ in 0..10 {
            w.step(0.016).unwrap();
        }
        let snapshot = w.clone();
        for _ in 0..10 {
            w.step(0.016).unwrap();
        }
        let h1 = w.state_hash();
        w = snapshot;
        for _ in 0..10 {
            w.step(0.016).unwrap();
        }
        let h2 = w.state_hash();
        assert_eq!(
            h1, h2,
            "Snapshot rollback and replay in 3D must yield bitwise identical state hash"
        );
    }

    #[test]
    fn rollback_replays_bitwise_2d() {
        let mut w = World2::default();
        w.add_body(
            BodyBuilder2::dynamic()
                .position(Vec2 { x: 1.0, y: 10.0 })
                .add_collider(Collider2 {
                    shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                    offset: Vec2::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
                }),
        )
        .unwrap();
        for _ in 0..10 {
            w.step(0.016).unwrap();
        }
        let snapshot = w.clone();
        for _ in 0..10 {
            w.step(0.016).unwrap();
        }
        let h1 = w.state_hash();
        w = snapshot;
        for _ in 0..10 {
            w.step(0.016).unwrap();
        }
        let h2 = w.state_hash();
        assert_eq!(
            h1, h2,
            "Snapshot rollback and replay in 2D must yield bitwise identical state hash"
        );
    }
    #[test]
    fn distance_joint_3d() {
        let mut w = World3::default();
        let b1 = w
            .add_body(BodyBuilder3::dynamic().position(Vec3::ZERO))
            .unwrap();
        let b2 = w
            .add_body(BodyBuilder3::dynamic().position(Vec3 {
                x: 2.0,
                y: 0.0,
                z: 0.0,
            }))
            .unwrap();
        w.add_joint(JointConfig3 {
            joint_type: JointType3::Distance,
            body_a: b1,
            body_b: b2,
            anchor_a: Vec3::ZERO,
            anchor_b: Vec3::ZERO,
            limits: JointLimits::default(),
            motor: JointMotor::default(),
            break_impulse: 0.0,
            user_data: 0,
        })
        .unwrap();
        for _ in 0..60 {
            w.step(0.016).unwrap();
        }
        let dist = (w.body(b1).unwrap().position - w.body(b2).unwrap().position).length();
        assert!(dist < 0.5, "bodies should be pulled together, got {}", dist);
    }

    #[test]
    fn tossed_body_does_not_sleep_at_apex_2d() {
        let mut w = World2::default();
        let h = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 { x: 0.0, y: 10.0 })
                    .velocity(Vec2 { x: 0.0, y: 5.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        // Step until ~apex of jump
        for _ in 0..30 {
            w.step(0.016).unwrap();
            let b = w.body(h).unwrap();
            assert!(!b.sleeping, "Airborne body must never fall asleep at apex");
        }
    }

    #[test]
    fn tossed_body_does_not_sleep_at_apex_3d() {
        let mut w = World3::default();
        let h = w
            .add_body(
                BodyBuilder3::dynamic()
                    .position(Vec3 {
                        x: 0.0,
                        y: 10.0,
                        z: 0.0,
                    })
                    .velocity(Vec3 {
                        x: 0.0,
                        y: 5.0,
                        z: 0.0,
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
            w.step(0.016).unwrap();
            let b = w.body(h).unwrap();
            assert!(
                !b.sleeping,
                "Airborne 3D body must never fall asleep at apex"
            );
        }
    }

    #[test]
    fn linear_damping_decay_2d() {
        let mut w = World2::default();
        w.gravity = Vec2::ZERO;
        let h = w
            .add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2::ZERO)
                    .velocity(Vec2 { x: 10.0, y: 0.0 })
                    .linear_damping(0.5)
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
        let vx = w.body(h).unwrap().velocity.x;
        assert!(
            vx < 7.0 && vx > 0.1,
            "Velocity should decay due to linear damping, got {}",
            vx
        );
    }

    #[test]
    fn linear_damping_decay_3d() {
        let mut w = World3::default();
        w.gravity = Vec3::ZERO;
        let h = w
            .add_body(
                BodyBuilder3::dynamic()
                    .position(Vec3::ZERO)
                    .velocity(Vec3 {
                        x: 10.0,
                        y: 0.0,
                        z: 0.0,
                    })
                    .linear_damping(0.5)
                    .add_collider(Collider3 {
                        shape: ColliderShape3::Sphere(Sphere3::new(0.5).unwrap()),
                        offset: Vec3::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        for _ in 0..60 {
            w.step(1.0 / 60.0).unwrap();
        }
        let vx = w.body(h).unwrap().velocity.x;
        assert!(
            vx < 7.0 && vx > 0.1,
            "Velocity should decay due to linear damping, got {}",
            vx
        );
    }

    #[test]
    fn long_run_determinism_suite_10k_steps_2d() {
        let build_scene = || -> World2 {
            let mut w = World2::default();
            w.add_body(
                BodyBuilder2::static_body()
                    .position(Vec2 { x: 0.0, y: -5.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Box(
                            auralite_geometry::Box2::new(Vec2 { x: 50.0, y: 1.0 }).unwrap(),
                        ),
                        offset: Vec2::ZERO,
                        material: Material {
                            restitution: 0.3,
                            friction: 0.5,
                            ..Default::default()
                        },
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
            for i in 0..5 {
                w.add_body(
                    BodyBuilder2::dynamic()
                        .position(Vec2 {
                            x: (i as Real - 2.0) * 1.5,
                            y: 5.0 + i as Real * 2.0,
                        })
                        .velocity(Vec2 {
                            x: (i as Real - 2.0) * 0.5,
                            y: -1.0,
                        })
                        .linear_damping(0.01)
                        .angular_damping(0.01)
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                            offset: Vec2::ZERO,
                            material: Material {
                                restitution: 0.4,
                                friction: 0.4,
                                ..Default::default()
                            },
                            filter: CollisionFilter::default(),
                        }),
                )
                .unwrap();
            }
            w
        };

        // Run 1: 10,000 steps continuous
        let mut w1 = build_scene();
        let mut hashes_run1 = Vec::new();
        for step in 1..=10_000 {
            w1.step(0.016).unwrap();
            if step % 2500 == 0 || step == 10_000 {
                hashes_run1.push((step, w1.state_hash()));
            }
        }

        // Run 2: independent reproduction
        let mut w2 = build_scene();
        let mut hashes_run2 = Vec::new();
        for step in 1..=10_000 {
            w2.step(0.016).unwrap();
            if step % 2500 == 0 || step == 10_000 {
                hashes_run2.push((step, w2.state_hash()));
            }
        }
        assert_eq!(
            hashes_run1, hashes_run2,
            "10,000-step continuous run 1 vs run 2 must be bitwise identical across all checkpoints"
        );

        // Run 3: run to step 5000, snapshot, roll back, run to 10,000
        let mut w3 = build_scene();
        for _ in 1..=5_000 {
            w3.step(0.016).unwrap();
        }
        let snapshot = w3.clone();
        for _ in 5_001..=7_500 {
            w3.step(0.016).unwrap();
        }
        w3 = snapshot; // Rollback to step 5000 exactly
        for _ in 5_001..=10_000 {
            w3.step(0.016).unwrap();
        }
        assert_eq!(
            w3.state_hash(),
            w1.state_hash(),
            "Run 3 snapshot-rollback state at step 10,000 must bitwise match continuous Run 1"
        );
    }

    #[test]
    fn long_run_determinism_suite_10k_steps_3d() {
        let build_scene = || -> World3 {
            let mut w = World3::default();
            w.add_body(
                BodyBuilder3::static_body()
                    .position(Vec3 {
                        x: 0.0,
                        y: -5.0,
                        z: 0.0,
                    })
                    .add_collider(Collider3 {
                        shape: ColliderShape3::Box(
                            auralite_geometry::Box3::new(Vec3 {
                                x: 50.0,
                                y: 1.0,
                                z: 50.0,
                            })
                            .unwrap(),
                        ),
                        offset: Vec3::ZERO,
                        material: Material {
                            restitution: 0.3,
                            friction: 0.5,
                            ..Default::default()
                        },
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
            for i in 0..5 {
                w.add_body(
                    BodyBuilder3::dynamic()
                        .position(Vec3 {
                            x: (i as Real - 2.0) * 1.5,
                            y: 5.0 + i as Real * 2.0,
                            z: (i as Real - 2.0) * 0.5,
                        })
                        .velocity(Vec3 {
                            x: (i as Real - 2.0) * 0.5,
                            y: -1.0,
                            z: 0.2,
                        })
                        .linear_damping(0.01)
                        .angular_damping(0.01)
                        .add_collider(Collider3 {
                            shape: ColliderShape3::Sphere(Sphere3::new(0.5).unwrap()),
                            offset: Vec3::ZERO,
                            material: Material {
                                restitution: 0.4,
                                friction: 0.4,
                                ..Default::default()
                            },
                            filter: CollisionFilter::default(),
                        }),
                )
                .unwrap();
            }
            w
        };

        let mut w1 = build_scene();
        let mut hashes_run1 = Vec::new();
        for step in 1..=10_000 {
            w1.step(0.016).unwrap();
            if step % 2500 == 0 || step == 10_000 {
                hashes_run1.push((step, w1.state_hash()));
            }
        }

        let mut w2 = build_scene();
        let mut hashes_run2 = Vec::new();
        for step in 1..=10_000 {
            w2.step(0.016).unwrap();
            if step % 2500 == 0 || step == 10_000 {
                hashes_run2.push((step, w2.state_hash()));
            }
        }
        assert_eq!(
            hashes_run1, hashes_run2,
            "10,000-step continuous 3D run 1 vs run 2 must be bitwise identical across all checkpoints"
        );

        let mut w3 = build_scene();
        for _ in 1..=5_000 {
            w3.step(0.016).unwrap();
        }
        let snapshot = w3.clone();
        for _ in 5_001..=7_500 {
            w3.step(0.016).unwrap();
        }
        w3 = snapshot;
        for _ in 5_001..=10_000 {
            w3.step(0.016).unwrap();
        }
        assert_eq!(
            w3.state_hash(),
            w1.state_hash(),
            "3D Run 3 snapshot-rollback state at step 10,000 must bitwise match continuous Run 1"
        );
    }

    #[test]
    fn steady_state_step_allocation_budget_2d() {
        let mut w = World2::default();
        w.add_body(
            BodyBuilder2::static_body()
                .position(Vec2 { x: 0.0, y: -5.0 })
                .add_collider(Collider2 {
                    shape: ColliderShape2::Box(
                        auralite_geometry::Box2::new(Vec2 { x: 50.0, y: 1.0 }).unwrap(),
                    ),
                    offset: Vec2::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
                }),
        )
        .unwrap();
        for i in 0..5 {
            w.add_body(
                BodyBuilder2::dynamic()
                    .position(Vec2 {
                        x: (i as Real - 2.0) * 1.5,
                        y: 5.0 + i as Real * 2.0,
                    })
                    .velocity(Vec2 { x: 0.0, y: -1.0 })
                    .add_collider(Collider2 {
                        shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()),
                        offset: Vec2::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();
        }
        for _ in 0..50 {
            w.step(0.016).unwrap();
        }
        let cap_handles = w.scratch_handles.capacity();
        let cap_constraints = w.scratch_constraints.capacity();
        let cap_raw = w.scratch_raw_contacts.capacity();
        let cap_pairs = w.scratch_pairs.capacity();
        let cap_sensor = w.scratch_sensor_pairs.capacity();
        let cap_prev_man = w.prev_manifolds.capacity();
        let cap_prev_sen = w.prev_sensor_pairs.capacity();

        for _ in 0..50 {
            w.step(0.016).unwrap();
        }
        assert_eq!(
            w.scratch_handles.capacity(),
            cap_handles,
            "scratch_handles must not reallocate during steady state"
        );
        assert_eq!(
            w.scratch_constraints.capacity(),
            cap_constraints,
            "scratch_constraints must not reallocate during steady state"
        );
        assert_eq!(
            w.scratch_raw_contacts.capacity(),
            cap_raw,
            "scratch_raw_contacts must not reallocate during steady state"
        );
        assert_eq!(
            w.scratch_pairs.capacity(),
            cap_pairs,
            "scratch_pairs must not reallocate during steady state"
        );
        assert_eq!(
            w.scratch_sensor_pairs.capacity(),
            cap_sensor,
            "scratch_sensor_pairs must not reallocate during steady state"
        );
        assert_eq!(
            w.prev_manifolds.capacity(),
            cap_prev_man,
            "prev_manifolds must not reallocate during steady state"
        );
        assert_eq!(
            w.prev_sensor_pairs.capacity(),
            cap_prev_sen,
            "prev_sensor_pairs must not reallocate during steady state"
        );
    }
}
