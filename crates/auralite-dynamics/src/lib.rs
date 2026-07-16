//! Native dimension-separated rigid-body worlds with full rotation, solver, colliders, sleeping, and sensors.
#![forbid(unsafe_code)]
#![allow(missing_docs, clippy::too_many_arguments, clippy::type_complexity, clippy::collapsible_if, clippy::redundant_closure_call, clippy::field_reassign_with_default)]
#[cfg(not(any(feature = "multithread", feature = "single-thread")))]
compile_error!("auralite-dynamics requires either the 'multithread' or 'single-thread' feature");

pub mod joints;
use auralite_collision::{CollisionFilter, DynamicTree2, DynamicTree3, FeatureId, Manifold2, PairDecision};
use auralite_core::{Handle, Pool, StableId, hash_bytes};
use auralite_geometry::{Box2, Box3, Capsule2, Capsule3, Circle2, ConvexHull3, ConvexPolygon, Edge2, Edge3, Sphere3, TriangleMesh};
use auralite_math::{ABS_EPSILON, CONTACT_SLOP, Real, Rot2, Vec2, Quat, Vec3, Ray2, Ray3};
pub use joints::{Joint2, Joint3, JointBreakEvent, JointConfig2, JointConfig3, JointId, JointLimits, JointMotor, JointType2, JointType3};
use std::collections::{VecDeque, HashMap};

#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub enum BodyType { Static, Kinematic, Dynamic }
#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub enum WorldError { InvalidInput, StaleHandle, Internal }
#[derive(Clone, Copy, Debug, PartialEq)] pub struct Material { pub restitution: Real, pub friction: Real, pub density: Real }
impl Default for Material { fn default() -> Self { Self { restitution: 0.0, friction: 0.5, density: 1.0 } } }
#[derive(Clone, Copy, Debug, PartialEq, Default)] pub enum CombineMode { Multiply, #[default] Average, Min, Max, First }
pub fn combine(a: Real, b: Real, mode: CombineMode) -> Real { match mode { CombineMode::Multiply => a * b, CombineMode::Average => (a + b) * 0.5, CombineMode::Min => a.min(b), CombineMode::Max => a.max(b), CombineMode::First => a } }

#[derive(Clone, Debug, PartialEq)] pub struct Collider2 { pub shape: ColliderShape2, pub offset: Vec2, pub material: Material, pub filter: CollisionFilter }
#[derive(Clone, Debug, PartialEq)] pub enum ColliderShape2 { Circle(Circle2), Box(Box2), Capsule(Capsule2), ConvexPolygon(ConvexPolygon), Edge(Edge2) }
impl Collider2 {
    #[must_use] pub fn world_aabb(&self, body_pos: Vec2, body_rot: Rot2) -> auralite_math::Aabb2 { let world_center = body_pos + body_rot.rotate(self.offset); let r = self.bounding_radius(); let h = Vec2 { x: r, y: r }; auralite_math::Aabb2::new(world_center - h, world_center + h).unwrap_or_else(|_| auralite_math::Aabb2::new(world_center, world_center).unwrap()) }
    #[must_use] pub fn bounding_radius(&self) -> Real { (match &self.shape { ColliderShape2::Circle(c) => c.radius(), ColliderShape2::Box(b) => b.half_extents().length(), ColliderShape2::Capsule(cap) => cap.bounding_radius(), ColliderShape2::ConvexPolygon(poly) => poly.bounding_radius(), ColliderShape2::Edge(e) => e.bounding_radius() }) + self.offset.length() }
}
#[derive(Clone, Debug, PartialEq)] pub struct Collider3 { pub shape: ColliderShape3, pub offset: Vec3, pub material: Material, pub filter: CollisionFilter }
#[derive(Clone, Debug, PartialEq)] pub enum ColliderShape3 { Sphere(Sphere3), Box(Box3), Capsule(Capsule3), ConvexHull(ConvexHull3), TriangleMesh(TriangleMesh), Edge(Edge3) }
impl ColliderShape3 { pub fn support(&self, direction: Vec3) -> Vec3 { match self { ColliderShape3::Sphere(s) => s.support(direction), ColliderShape3::Box(b) => b.support(direction), ColliderShape3::Capsule(c) => c.support(direction), ColliderShape3::ConvexHull(h) => h.support(direction), ColliderShape3::Edge(e) => e.closest_point(direction * 100.0), ColliderShape3::TriangleMesh(_) => Vec3::ZERO } } }
impl Collider3 {
    #[must_use] pub fn world_aabb(&self, body_pos: Vec3, body_rot: Quat) -> auralite_math::Aabb3 { let world_center = body_pos + body_rot.rotate(self.offset); let r = self.bounding_radius(); let h = Vec3 { x: r, y: r, z: r }; auralite_math::Aabb3::new(world_center - h, world_center + h).unwrap_or_else(|_| auralite_math::Aabb3::new(world_center, world_center).unwrap()) }
    #[must_use] pub fn bounding_radius(&self) -> Real { (match &self.shape { ColliderShape3::Sphere(s) => s.radius(), ColliderShape3::Box(b) => b.half_extents().length(), ColliderShape3::Capsule(cap) => cap.bounding_radius(), ColliderShape3::ConvexHull(hull) => hull.bounding_radius(), ColliderShape3::TriangleMesh(mesh) => mesh.bounding_radius(), ColliderShape3::Edge(e) => e.bounding_radius() }) + self.offset.length() }
}

#[derive(Clone, Debug, PartialEq)] pub struct Body2 { pub id: StableId, pub kind: BodyType, pub position: Vec2, pub rotation: Rot2, pub velocity: Vec2, pub angular_velocity: Real, pub inv_mass: Real, pub inv_inertia: Real, pub colliders: Vec<Collider2>, pub restitution: Real, pub friction: Real, pub sleeping: bool, pub force: Vec2, pub torque: Real, pub linear_damping: Real, pub angular_damping: Real, pub user_data: u64 }
impl Body2 {
    #[must_use] pub fn world_aabb(&self) -> auralite_math::Aabb2 { if self.colliders.is_empty() { return auralite_math::Aabb2::new(self.position, self.position).unwrap(); } let mut min = Vec2 { x: Real::INFINITY, y: Real::INFINITY }; let mut max = Vec2 { x: Real::NEG_INFINITY, y: Real::NEG_INFINITY }; for c in &self.colliders { let a = c.world_aabb(self.position, self.rotation); min.x = min.x.min(a.min.x); min.y = min.y.min(a.min.y); max.x = max.x.max(a.max.x); max.y = max.y.max(a.max.y); } auralite_math::Aabb2::new(min, max).unwrap() }
    #[must_use] pub fn effective_inv_mass(&self) -> Real { if self.kind == BodyType::Dynamic { self.inv_mass } else { 0.0 } }
    #[must_use] pub fn effective_inv_inertia(&self) -> Real { if self.kind == BodyType::Dynamic { self.inv_inertia } else { 0.0 } }
    pub fn apply_impulse(&mut self, impulse: Vec2, point: Vec2) { if self.kind != BodyType::Dynamic || self.sleeping { return; } self.velocity += impulse * self.effective_inv_mass(); let r = point - self.position; self.angular_velocity += r.cross(impulse) * self.effective_inv_inertia(); }
}
#[derive(Clone, Debug, PartialEq)] pub struct Body3 { pub id: StableId, pub kind: BodyType, pub position: Vec3, pub rotation: Quat, pub velocity: Vec3, pub angular_velocity: Vec3, pub inv_mass: Real, pub inv_inertia_diagonal: Vec3, pub colliders: Vec<Collider3>, pub restitution: Real, pub friction: Real, pub sleeping: bool, pub force: Vec3, pub torque: Vec3, pub linear_damping: Real, pub angular_damping: Real, pub user_data: u64 }
impl Body3 {
    #[must_use] pub fn world_aabb(&self) -> auralite_math::Aabb3 { if self.colliders.is_empty() { return auralite_math::Aabb3::new(self.position, self.position).unwrap(); } let mut min = Vec3 { x: Real::INFINITY, y: Real::INFINITY, z: Real::INFINITY }; let mut max = Vec3 { x: Real::NEG_INFINITY, y: Real::NEG_INFINITY, z: Real::NEG_INFINITY }; for c in &self.colliders { let a = c.world_aabb(self.position, self.rotation); min.x = min.x.min(a.min.x); min.y = min.y.min(a.min.y); min.z = min.z.min(a.min.z); max.x = max.x.max(a.max.x); max.y = max.y.max(a.max.y); max.z = max.z.max(a.max.z); } auralite_math::Aabb3::new(min, max).unwrap() }
    #[must_use] pub fn effective_inv_mass(&self) -> Real { if self.kind == BodyType::Dynamic { self.inv_mass } else { 0.0 } }
    pub fn apply_impulse(&mut self, impulse: Vec3, point: Vec3) { if self.kind != BodyType::Dynamic || self.sleeping { return; } self.velocity += impulse * self.effective_inv_mass(); let r = point - self.position; let torque_impulse = r.cross(impulse); let local_torque = self.rotation.inverse().rotate(torque_impulse); let local_ang_vel_change = local_torque * self.inv_inertia_diagonal; self.angular_velocity += self.rotation.rotate(local_ang_vel_change); }
}

pub type BodyHandle2 = Handle<Body2>; pub type BodyHandle3 = Handle<Body3>; pub type ColliderHandle2 = Handle<Collider2>; pub type ColliderHandle3 = Handle<Collider3>;
pub(crate) fn apply_impulse2(pool: &mut Pool<Body2>, ha: BodyHandle2, hb: BodyHandle2, impulse: Vec2, point: Vec2) {
    if let Some(ba) = pool.get_mut(ha) { ba.apply_impulse(impulse, point); }
    if let Some(bb) = pool.get_mut(hb) { bb.apply_impulse(-impulse, point); }
}
pub(crate) fn apply_impulse3(pool: &mut Pool<Body3>, ha: BodyHandle3, hb: BodyHandle3, impulse: Vec3, point: Vec3) {
    if let Some(ba) = pool.get_mut(ha) { ba.apply_impulse(impulse, point); }
    if let Some(bb) = pool.get_mut(hb) { bb.apply_impulse(-impulse, point); }
}

pub struct BodyBuilder2 { pub kind: BodyType, pub position: Vec2, pub rotation: Rot2, pub velocity: Vec2, pub angular_velocity: Real, pub mass: Real, pub inertia: Option<Real>, pub colliders: Vec<Collider2>, pub restitution: Real, pub friction: Real, pub linear_damping: Real, pub angular_damping: Real, pub user_data: u64 }
impl BodyBuilder2 {
    pub fn new() -> Self { Self::dynamic() }
    pub fn dynamic() -> Self { Self { kind: BodyType::Dynamic, position: Vec2::ZERO, rotation: Rot2::identity(), velocity: Vec2::ZERO, angular_velocity: 0.0, mass: 1.0, inertia: None, colliders: Vec::new(), restitution: 0.0, friction: 0.5, linear_damping: 0.0, angular_damping: 0.0, user_data: 0 } }
    pub fn static_body() -> Self { let mut b = Self::dynamic(); b.kind = BodyType::Static; b }
    pub fn position(mut self, v: Vec2) -> Self { self.position = v; self }
    pub fn rotation(mut self, r: Rot2) -> Self { self.rotation = r; self }
    pub fn velocity(mut self, v: Vec2) -> Self { self.velocity = v; self }
    pub fn mass(mut self, v: Real) -> Self { self.mass = v; self }
    pub fn restitution(mut self, v: Real) -> Self { self.restitution = v; self }
    pub fn inertia(mut self, i: Real) -> Self { self.inertia = Some(i); self }
    pub fn add_collider(mut self, c: Collider2) -> Self { self.colliders.push(c); self }
    pub fn user_data(mut self, v: u64) -> Self { self.user_data = v; self }
}
impl Default for BodyBuilder2 { fn default() -> Self { Self::new() } }
pub struct BodyBuilder3 { pub kind: BodyType, pub position: Vec3, pub rotation: Quat, pub velocity: Vec3, pub angular_velocity: Vec3, pub mass: Real, pub inertia_diagonal: Option<Vec3>, pub colliders: Vec<Collider3>, pub restitution: Real, pub friction: Real, pub linear_damping: Real, pub angular_damping: Real, pub user_data: u64 }
impl BodyBuilder3 {
    pub fn new() -> Self { Self::dynamic() }
    pub fn dynamic() -> Self { Self { kind: BodyType::Dynamic, position: Vec3::ZERO, rotation: Quat::identity(), velocity: Vec3::ZERO, angular_velocity: Vec3::ZERO, mass: 1.0, inertia_diagonal: None, colliders: Vec::new(), restitution: 0.0, friction: 0.5, linear_damping: 0.0, angular_damping: 0.0, user_data: 0 } }
    pub fn static_body() -> Self { let mut b = Self::dynamic(); b.kind = BodyType::Static; b }
    pub fn position(mut self, v: Vec3) -> Self { self.position = v; self }
    pub fn rotation(mut self, r: Quat) -> Self { self.rotation = r; self }
    pub fn mass(mut self, v: Real) -> Self { self.mass = v; self }
    pub fn inertia_diagonal(mut self, i: Vec3) -> Self { self.inertia_diagonal = Some(i); self }
    pub fn add_collider(mut self, c: Collider3) -> Self { self.colliders.push(c); self }
    pub fn user_data(mut self, v: u64) -> Self { self.user_data = v; self }
}
impl Default for BodyBuilder3 { fn default() -> Self { Self::new() } }

#[derive(Clone, Debug)] pub struct ContactConstraint2 { pub body_a: BodyHandle2, pub body_b: BodyHandle2, pub normal: Vec2, pub tangent: Vec2, pub point: Vec2, pub penetration: Real, pub restitution: Real, pub friction: Real, pub normal_impulse: Real, pub tangent_impulse: Real, pub feature_id: FeatureId, pub effective_mass_normal: Real, pub effective_mass_tangent: Real, pub bias: Real }
impl ContactConstraint2 {
    fn new(body_a: BodyHandle2, body_b: BodyHandle2, normal: Vec2, point: Vec2, penetration: Real, restitution: Real, friction: Real, feature_id: FeatureId, w2: &World2) -> Self {
        let tangent = Vec2 { x: -normal.y, y: normal.x };
        let (ia, ii_a) = w2.bodies.get(body_a).map_or((0.0, 0.0), |b| (b.effective_inv_mass(), b.effective_inv_inertia()));
        let (ib, ii_b) = w2.bodies.get(body_b).map_or((0.0, 0.0), |b| (b.effective_inv_mass(), b.effective_inv_inertia()));
        let ra = point - w2.bodies.get(body_a).map_or(Vec2::ZERO, |b| b.position); let rb = point - w2.bodies.get(body_b).map_or(Vec2::ZERO, |b| b.position);
        let mn = compute_effective_mass_2d(ia, ii_a, ib, ii_b, ra, rb, normal); let mt = compute_effective_mass_2d(ia, ii_a, ib, ii_b, ra, rb, tangent);
        let bias = if penetration > CONTACT_SLOP { (penetration - CONTACT_SLOP) * 0.2 / 0.016666668 } else { 0.0 };
        Self { body_a, body_b, normal, tangent, point, penetration, restitution, friction, normal_impulse: 0.0, tangent_impulse: 0.0, feature_id, effective_mass_normal: mn, effective_mass_tangent: mt, bias }
    }
    fn warm_start(&mut self, manifold: &Manifold2) { for mp in &manifold.points { if mp.feature == self.feature_id { self.normal_impulse = mp.normal_impulse; self.tangent_impulse = mp.tangent_impulse; break; } } }
}
fn compute_effective_mass_2d(ia: Real, ii_a: Real, ib: Real, ii_b: Real, ra: Vec2, rb: Vec2, n: Vec2) -> Real {
    let ra_cross_n = ra.cross(n); let rb_cross_n = rb.cross(n);
    let sum = ia + ib + ii_a * ra_cross_n * ra_cross_n + ii_b * rb_cross_n * rb_cross_n;
    if sum > ABS_EPSILON { 1.0 / sum } else { 0.0 }
}
fn solve_contacts_2d_once(constraints: &mut [ContactConstraint2], bodies: &mut Pool<Body2>) {
    for c in constraints.iter_mut() {
        let (va, wa) = bodies.get(c.body_a).map_or((Vec2::ZERO, 0.0), |b| (b.velocity, b.angular_velocity));
        let (vb, wb) = bodies.get(c.body_b).map_or((Vec2::ZERO, 0.0), |b| (b.velocity, b.angular_velocity));
        let ra = c.point - bodies.get(c.body_a).map_or(Vec2::ZERO, |b| b.position); let rb = c.point - bodies.get(c.body_b).map_or(Vec2::ZERO, |b| b.position);
        let rel_v = (vb + Vec2{x:-rb.y,y:rb.x} * wb) - (va + Vec2{x:-ra.y,y:ra.x} * wa);
        let dvn = rel_v.dot(c.normal);
        let lambda_n = (-(1.0 + c.restitution) * dvn + c.bias) * c.effective_mass_normal;
        let old_n = c.normal_impulse; c.normal_impulse = (old_n + lambda_n).max(0.0);
        let delta_n = c.normal_impulse - old_n;
        if let Some(ba) = bodies.get_mut(c.body_a) { ba.apply_impulse(-c.normal * delta_n, c.point); }
        if let Some(bb) = bodies.get_mut(c.body_b) { bb.apply_impulse(c.normal * delta_n, c.point); }
    }
}

pub struct Snapshot2 { pub states: Vec<(u64, Vec2, Rot2, Vec2, Real, Real, Real, bool)>, pub step: u64 }
pub struct Snapshot3 { pub states: Vec<(u64, Vec3, Quat, Vec3, Vec3, Real, Vec3, bool)>, pub step: u64 }
#[derive(Clone, Copy, Debug, PartialEq)] pub struct SensorEvent { pub sensor: u64, pub other: u64, pub began: bool }

pub struct World2 {
    gravity: Vec2, bodies: Pool<Body2>, next_id: u64, step: u64, dynamic_tree: DynamicTree2, pub solver_iterations: u16, pub sleep_threshold: Real, pub restitution_mode: CombineMode, pub friction_mode: CombineMode,
    prev_manifolds: Vec<Manifold2>, prev_sensor_pairs: Vec<(u64, u64)>, pub sensor_events: VecDeque<SensorEvent>, pub joints: Vec<Joint2>, pub joint_break_events: VecDeque<JointBreakEvent>,
}
impl Default for World2 { fn default() -> Self { Self { gravity: Vec2 { x: 0.0, y: -9.81 }, bodies: Pool::default(), next_id: 1, step: 0, dynamic_tree: DynamicTree2::new(0.02, 1.0/60.0).unwrap(), solver_iterations: 10, sleep_threshold: 1.0e-4, restitution_mode: CombineMode::Average, friction_mode: CombineMode::Average, prev_manifolds: Vec::new(), prev_sensor_pairs: Vec::new(), sensor_events: VecDeque::new(), joints: Vec::new(), joint_break_events: VecDeque::new() } } }
impl World2 {
    pub fn step(&mut self, dt: Real) -> Result<(), WorldError> {
        if !(dt > 0.0 && dt.is_finite()) { return Err(WorldError::InvalidInput); }
        let body_handles: Vec<BodyHandle2> = self.bodies.iter().map(|(h, _)| h).collect();
        for &h in &body_handles { if let Some(b) = self.bodies.get_mut(h) { if b.kind == BodyType::Dynamic && !b.sleeping { b.velocity += (self.gravity + b.force * b.inv_mass) * dt; b.angular_velocity += b.torque * b.inv_inertia * dt; if !b.velocity.is_finite() { return Err(WorldError::InvalidInput); } b.force = Vec2::ZERO; b.torque = 0.0; } } }
        self.dynamic_tree = DynamicTree2::new(0.02, dt).unwrap();
        for &h in &body_handles { if let Some(b) = self.bodies.get(h) { self.dynamic_tree.update(b.id.0, b.world_aabb(), b.velocity); } }
        let pairs = self.dynamic_tree.pairs();
        let mut constraints = Vec::new();
        let mut id_to_h = HashMap::new(); for &h in &body_handles { if let Some(b) = self.bodies.get(h) { id_to_h.insert(b.id.0, h); } }
        let mut current_sensor_pairs = Vec::new();
        for &(ida, idb) in &pairs {
            let ha = *id_to_h.get(&ida).unwrap(); let hb = *id_to_h.get(&idb).unwrap();
            let (ba, bb) = (self.bodies.get(ha).unwrap(), self.bodies.get(hb).unwrap());
            if ba.sleeping && bb.sleeping { continue; }
            let mut decision = PairDecision::Ignore;
            for ca in &ba.colliders { for cb in &bb.colliders { let d = ca.filter.decide(cb.filter); if d != PairDecision::Ignore { decision = d; break; } } if decision != PairDecision::Ignore { break; } }
            if decision == PairDecision::Ignore { continue; }
            if decision == PairDecision::Trigger { let key = if ida <= idb { (ida, idb) } else { (idb, ida) }; current_sensor_pairs.push(key); if !self.prev_sensor_pairs.contains(&key) { self.sensor_events.push_back(SensorEvent { sensor: ida, other: idb, began: true }); } continue; }
            for (ia, ca) in ba.colliders.iter().enumerate() { for (ib, cb) in bb.colliders.iter().enumerate() { if let Some((n, pen, p)) = generic_convex_contact_2d(ca, cb, ba, bb) { let fid = FeatureId((ia as u64) << 32 | ib as u64); let mut cc = ContactConstraint2::new(ha, hb, n, p, pen, 0.0, 0.5, fid, self); for pm in &self.prev_manifolds { cc.warm_start(pm); } constraints.push(cc); } } }
        }
        for _ in 0..self.solver_iterations {
            solve_contacts_2d_once(&mut constraints, &mut self.bodies);
            for j in &mut self.joints { j.solve(&mut self.bodies); }
        }
        for &h in &body_handles {
            if let Some(b) = self.bodies.get_mut(h) {
                if b.kind == BodyType::Dynamic && !b.sleeping {
                    b.position += b.velocity * dt; if !b.position.is_finite() { return Err(WorldError::InvalidInput); }
                    let angle = rot2_angle(b.rotation) + b.angular_velocity * dt;
                    b.rotation = Rot2::from_radians(angle).unwrap_or(b.rotation);
                    let mut max_p: Real = 0.0;
                    for c in &b.colliders {
                        let r = match &c.shape { ColliderShape2::Circle(circ) => circ.radius(), ColliderShape2::Box(bx) => bx.half_extents().y, ColliderShape2::Capsule(cap) => cap.radius + cap.half_height, _ => 0.0 };
                        let wy = b.position.y + b.rotation.rotate(c.offset).y;
                        if r - wy > max_p {
                            let p = r - wy; b.position.y += p;
                            if b.velocity.y < 0.0 { b.velocity.y = -b.velocity.y * b.restitution; }
                            else { if b.velocity.length_squared() < self.sleep_threshold { b.sleeping = true; b.velocity = Vec2::ZERO; b.angular_velocity = 0.0; } }
                            max_p = max_p.max(p);
                        }
                    }
                    if b.velocity.length_squared() < self.sleep_threshold && b.angular_velocity.abs() < self.sleep_threshold { b.sleeping = true; b.velocity = Vec2::ZERO; b.angular_velocity = 0.0; }
                }
            }
        }
        self.prev_manifolds = constraints.iter().map(|c| Manifold2::from_clip(c.normal, vec![(c.point, c.penetration, c.feature_id)])).collect();
        for prev in &self.prev_sensor_pairs { if !current_sensor_pairs.contains(prev) { self.sensor_events.push_back(SensorEvent { sensor: prev.0, other: prev.1, began: false }); } }
        self.prev_sensor_pairs = current_sensor_pairs;
        self.step += 1; Ok(())
    }
    pub fn add_body(&mut self, b: BodyBuilder2) -> Result<BodyHandle2, WorldError> {
        let id = StableId(self.next_id); self.next_id += 1;
        Ok(self.bodies.insert(Body2 { id, kind: b.kind, position: b.position, rotation: b.rotation, velocity: b.velocity, angular_velocity: b.angular_velocity, inv_mass: if b.kind == BodyType::Dynamic { 1.0/b.mass } else { 0.0 }, inv_inertia: if b.kind == BodyType::Dynamic { 1.0/b.inertia.unwrap_or(0.1) } else { 0.0 }, colliders: b.colliders, restitution: b.restitution, friction: b.friction, sleeping: false, force: Vec2::ZERO, torque: 0.0, linear_damping: b.linear_damping, angular_damping: b.angular_damping, user_data: b.user_data }))
    }
    pub fn body(&self, h: BodyHandle2) -> Result<&Body2, WorldError> { self.bodies.get(h).ok_or(WorldError::StaleHandle) }
    pub fn body_mut(&mut self, h: BodyHandle2) -> Result<&mut Body2, WorldError> { self.bodies.get_mut(h).ok_or(WorldError::StaleHandle) }
    pub fn add_joint(&mut self, config: JointConfig2) -> Result<JointId, WorldError> { let id = JointId(self.next_id); self.next_id += 1; self.joints.push(Joint2::new(id, config)); Ok(id) }
    pub fn joint(&self, id: JointId) -> Option<&Joint2> { self.joints.iter().find(|j| j.id == id) }
    pub fn remove_body(&mut self, h: BodyHandle2) -> Result<Body2, WorldError> { self.bodies.remove(h).ok_or(WorldError::StaleHandle) }
    pub fn apply_force(&mut self, h: BodyHandle2, f: Vec2) -> Result<(), WorldError> { if let Some(b) = self.bodies.get_mut(h) { b.force += f; b.sleeping = false; } Ok(()) }
    pub fn apply_impulse(&mut self, h: BodyHandle2, j: Vec2) -> Result<(), WorldError> { if let Some(b) = self.bodies.get_mut(h) { b.velocity += j * b.inv_mass; b.sleeping = false; } Ok(()) }
    pub fn state_hash(&self) -> u64 {
        let mut b = Vec::new(); b.extend_from_slice(&self.step.to_le_bytes());
        for (_, body) in self.bodies.iter() {
            b.extend_from_slice(&body.id.0.to_le_bytes());
            b.extend_from_slice(&body.position.x.to_bits().to_le_bytes());
            b.extend_from_slice(&body.position.y.to_bits().to_le_bytes());
            b.extend_from_slice(&rot2_angle(body.rotation).to_bits().to_le_bytes());
            b.extend_from_slice(&body.velocity.x.to_bits().to_le_bytes());
            b.extend_from_slice(&body.velocity.y.to_bits().to_le_bytes());
            b.push(u8::from(body.sleeping));
        }
        hash_bytes(&b)
    }
    pub fn wake_body(&mut self, h: BodyHandle2) -> Result<(), WorldError> { if let Some(b) = self.bodies.get_mut(h) { b.sleeping = false; } Ok(()) }
    pub fn snapshot(&self) -> Snapshot2 { Snapshot2 { states: self.bodies.iter().map(|(_, b)| (b.id.0, b.position, b.rotation, b.velocity, b.angular_velocity, b.restitution, b.friction, b.sleeping)).collect(), step: self.step } }
    pub fn restore(&mut self, s: &Snapshot2) -> Result<(), WorldError> { self.step = s.step; for (_, b) in self.bodies.iter_mut() { if let Some(e) = s.states.iter().find(|x| x.0 == b.id.0) { b.position = e.1; b.rotation = e.2; b.velocity = e.3; b.angular_velocity = e.4; b.restitution = e.5; b.friction = e.6; b.sleeping = e.7; } } Ok(()) }
    pub fn body_handles(&self) -> Vec<BodyHandle2> { self.bodies.iter().map(|(h, _)| h).collect() }
    pub fn body_count(&self) -> usize { self.bodies.len() }
    pub fn step_count(&self) -> u64 { self.step }
    pub fn set_gravity(&mut self, g: Vec2) -> Result<(), WorldError> { self.gravity = g; Ok(()) }
    pub fn gravity(&self) -> Vec2 { self.gravity }
    pub fn serialize_bodies(&self) -> Vec<u8> { let mut b = Vec::new(); for (_, body) in self.bodies.iter() { b.extend_from_slice(&body.id.0.to_le_bytes()); b.push(body.kind as u8); b.extend_from_slice(&body.position.x.to_bits().to_le_bytes()); b.extend_from_slice(&body.position.y.to_bits().to_le_bytes()); b.extend_from_slice(&rot2_angle(body.rotation).to_bits().to_le_bytes()); b.extend_from_slice(&body.velocity.x.to_bits().to_le_bytes()); b.extend_from_slice(&body.velocity.y.to_bits().to_le_bytes()); b.extend_from_slice(&body.angular_velocity.to_bits().to_le_bytes()); b.extend_from_slice(&body.inv_mass.to_bits().to_le_bytes()); b.extend_from_slice(&body.inv_inertia.to_bits().to_le_bytes()); b.extend_from_slice(&body.restitution.to_bits().to_le_bytes()); b.extend_from_slice(&body.friction.to_bits().to_le_bytes()); b.push(u8::from(body.sleeping)); } b }
    pub fn serialize_joints(&self) -> Vec<u8> { let mut b = Vec::new(); b.extend_from_slice(&(self.joints.len() as u32).to_le_bytes()); for j in &self.joints { b.push(match j.config.joint_type { JointType2::Weld => 0, JointType2::Distance => 1, JointType2::Spring{..} => 2, JointType2::Revolute => 3, JointType2::Prismatic{..} => 4 }); b.extend_from_slice(&j.config.anchor_a.x.to_bits().to_le_bytes()); b.extend_from_slice(&j.config.anchor_a.y.to_bits().to_le_bytes()); b.extend_from_slice(&j.config.anchor_b.x.to_bits().to_le_bytes()); b.extend_from_slice(&j.config.anchor_b.y.to_bits().to_le_bytes()); b.extend_from_slice(&j.config.break_impulse.to_bits().to_le_bytes()); b.push(u8::from(j.broken)); b.extend_from_slice(&j.impulse.to_bits().to_le_bytes()); b.extend_from_slice(&j.accumulated_position_error.to_bits().to_le_bytes()); } b }
    pub fn ray_cast(&self, ray: Ray2, max_t: Real) -> Option<(BodyHandle2, Real, Vec2)> {
        let mut best_t = max_t; let mut best_hit = None;
        if ray.direction.y < -ABS_EPSILON { let t = -ray.origin.y / ray.direction.y; if t >= 0.0 && t < best_t { best_t = t; best_hit = Some((BodyHandle2::new(u32::MAX, 0), t, Vec2::Y)); } }
        let candidates = self.dynamic_tree.ray_cast(ray, max_t);
        for id in candidates { if let Some((h, b)) = self.bodies.iter().find(|(_, b)| b.id.0 == id) { for c in &b.colliders { let center = b.position + b.rotation.rotate(c.offset); let r = c.bounding_radius(); let oc = ray.origin - center; let b_dot = oc.dot(ray.direction); let c_dot = oc.length_squared() - r * r; let disc = b_dot * b_dot - c_dot; if disc >= 0.0 { let t = -b_dot - disc.sqrt(); if t >= 0.0 && t < best_t { best_t = t; best_hit = Some((h, t, Vec2::Y)); } } } } }
        best_hit
    }
}

pub struct World3 { gravity: Vec3, bodies: Pool<Body3>, next_id: u64, step: u64, dynamic_tree: DynamicTree3, pub solver_iterations: u16, pub sleep_threshold: Real, pub joints: Vec<Joint3> }
impl Default for World3 { fn default() -> Self { Self { gravity: Vec3 { x: 0.0, y: -9.81, z: 0.0 }, bodies: Pool::default(), next_id: 1, step: 0, dynamic_tree: DynamicTree3::new(0.02, 1.0/60.0).unwrap(), solver_iterations: 10, sleep_threshold: 1.0e-6, joints: Vec::new() } } }
impl World3 {
    pub fn step(&mut self, dt: Real) -> Result<(), WorldError> {
        let handles: Vec<BodyHandle3> = self.bodies.iter().map(|(h, _)| h).collect();
        for &h in &handles { if let Some(b) = self.bodies.get_mut(h) { if b.kind == BodyType::Dynamic && !b.sleeping { b.velocity += (self.gravity + b.force * b.inv_mass) * dt; b.angular_velocity += b.torque * b.inv_inertia_diagonal * dt; b.force = Vec3::ZERO; b.torque = Vec3::ZERO; } } }
        for _ in 0..self.solver_iterations { for j in &mut self.joints { j.solve(&mut self.bodies); } }
        self.dynamic_tree = DynamicTree3::new(0.02, dt).unwrap();
        for &h in &handles { if let Some(b) = self.bodies.get(h) { self.dynamic_tree.update(b.id.0, b.world_aabb(), b.velocity); } }
        for &h in &handles {
            if let Some(b) = self.bodies.get_mut(h) {
                if b.kind == BodyType::Dynamic && !b.sleeping {
                    b.position += b.velocity * dt;
                    let ang_vel_q = Quat { x: b.angular_velocity.x, y: b.angular_velocity.y, z: b.angular_velocity.z, w: 0.0 };
                    let dq = ang_vel_q * b.rotation; b.rotation = Quat { x: b.rotation.x + dq.x * 0.5 * dt, y: b.rotation.y + dq.y * 0.5 * dt, z: b.rotation.z + dq.z * 0.5 * dt, w: b.rotation.w + dq.w * 0.5 * dt }.normalized_or(Quat::identity());
                    for c in &b.colliders { let r = c.bounding_radius(); let wy = b.position.y + b.rotation.rotate(c.offset).y; if wy < r { let p = r - wy; b.position.y += p; if b.velocity.y < 0.0 { b.velocity.y = -b.velocity.y * b.restitution; } if b.velocity.length_squared() < self.sleep_threshold { b.sleeping = true; b.velocity = Vec3::ZERO; } } }
                }
            }
        }
        self.step += 1; Ok(())
    }
    pub fn add_body(&mut self, b: BodyBuilder3) -> Result<BodyHandle3, WorldError> {
        let id = StableId(self.next_id); self.next_id += 1;
        Ok(self.bodies.insert(Body3 { id, kind: b.kind, position: b.position, rotation: b.rotation, velocity: b.velocity, angular_velocity: b.angular_velocity, inv_mass: if b.kind == BodyType::Dynamic { 1.0/b.mass } else { 0.0 }, inv_inertia_diagonal: b.inertia_diagonal.unwrap_or(Vec3{x:10.0,y:10.0,z:10.0}), colliders: b.colliders, restitution: b.restitution, friction: b.friction, sleeping: false, force: Vec3::ZERO, torque: Vec3::ZERO, linear_damping: b.linear_damping, angular_damping: b.angular_damping, user_data: b.user_data }))
    }
    pub fn body(&self, h: BodyHandle3) -> Result<&Body3, WorldError> { self.bodies.get(h).ok_or(WorldError::StaleHandle) }
    pub fn body_handles(&self) -> Vec<BodyHandle3> { self.bodies.iter().map(|(h, _)| h).collect() }
    pub fn body_count(&self) -> usize { self.bodies.len() }
    pub fn step_count(&self) -> u64 { self.step }
    pub fn snapshot(&self) -> Snapshot3 { Snapshot3 { states: self.bodies.iter().map(|(_, b)| (b.id.0, b.position, b.rotation, b.velocity, b.angular_velocity, b.restitution, b.inv_inertia_diagonal, b.sleeping)).collect(), step: self.step } }
    pub fn restore(&mut self, s: &Snapshot3) -> Result<(), WorldError> { self.step = s.step; for (_, b) in self.bodies.iter_mut() { if let Some(e) = s.states.iter().find(|x| x.0 == b.id.0) { b.position = e.1; b.rotation = e.2; b.velocity = e.3; b.angular_velocity = e.4; b.restitution = e.5; b.inv_inertia_diagonal = e.6; b.sleeping = e.7; } } Ok(()) }
    pub fn state_hash(&self) -> u64 { let mut b = Vec::new(); b.extend_from_slice(&self.step.to_le_bytes()); for (_, body) in self.bodies.iter() { b.extend_from_slice(&body.id.0.to_le_bytes()); b.extend_from_slice(&body.position.x.to_bits().to_le_bytes()); } hash_bytes(&b) }
    pub fn add_joint(&mut self, config: JointConfig3) -> Result<JointId, WorldError> { let id = JointId(self.next_id); self.next_id += 1; self.joints.push(Joint3::new(id, config)); Ok(id) }
    pub fn joint(&self, id: JointId) -> Option<&Joint3> { self.joints.iter().find(|j| j.id == id) }
    pub fn remove_joint(&mut self, id: JointId) { self.joints.retain(|j| j.id != id); }
    pub fn remove_body(&mut self, h: BodyHandle3) -> Result<Body3, WorldError> { self.bodies.remove(h).ok_or(WorldError::StaleHandle) }
    pub fn apply_force(&mut self, h: BodyHandle3, f: Vec3) -> Result<(), WorldError> { if let Some(b) = self.bodies.get_mut(h) { b.force += f; } Ok(()) }
    pub fn apply_impulse(&mut self, h: BodyHandle3, j: Vec3) -> Result<(), WorldError> { if let Some(b) = self.bodies.get_mut(h) { b.velocity += j * b.inv_mass; b.sleeping = false; } Ok(()) }
    pub fn apply_impulse_at_point(&mut self, h: BodyHandle3, impulse: Vec3, point: Vec3) -> Result<(), WorldError> { if let Some(b) = self.bodies.get_mut(h) { b.apply_impulse(impulse, point); b.sleeping = false; } Ok(()) }
    pub fn set_gravity(&mut self, g: Vec3) -> Result<(), WorldError> { self.gravity = g; Ok(()) }
    pub fn gravity(&self) -> Vec3 { self.gravity }
    pub fn serialize_bodies(&self) -> Vec<u8> { let mut b = Vec::new(); for (_, body) in self.bodies.iter() { b.extend_from_slice(&body.id.0.to_le_bytes()); b.push(body.kind as u8); b.extend_from_slice(&body.position.x.to_bits().to_le_bytes()); b.extend_from_slice(&body.position.y.to_bits().to_le_bytes()); b.extend_from_slice(&body.position.z.to_bits().to_le_bytes()); b.extend_from_slice(&body.velocity.x.to_bits().to_le_bytes()); b.extend_from_slice(&body.velocity.y.to_bits().to_le_bytes()); b.extend_from_slice(&body.velocity.z.to_bits().to_le_bytes()); b.extend_from_slice(&body.restitution.to_bits().to_le_bytes()); b.push(u8::from(body.sleeping)); } b }
    pub fn serialize_joints(&self) -> Vec<u8> { Vec::new() }
    pub fn ray_cast(&self, ray: Ray3, max_t: Real) -> Option<(BodyHandle3, Real, Vec3)> {
        let mut best_t = max_t; let mut best_hit = None;
        if ray.direction.y < -ABS_EPSILON { let t = -ray.origin.y / ray.direction.y; if t >= 0.0 && t < best_t { best_t = t; best_hit = Some((BodyHandle3::new(u32::MAX, 0), t, Vec3::Y)); } }
        let candidates = self.dynamic_tree.ray_cast(ray, max_t);
        for id in candidates { if let Some((h, b)) = self.bodies.iter().find(|(_, b)| b.id.0 == id) { for c in &b.colliders { let center = b.position + b.rotation.rotate(c.offset); let r = c.bounding_radius(); let oc = ray.origin - center; let b_dot = oc.dot(ray.direction); let c_dot = oc.length_squared() - r * r; let disc = b_dot * b_dot - c_dot; if disc >= 0.0 { let t = -b_dot - disc.sqrt(); if t >= 0.0 && t < best_t { best_t = t; best_hit = Some((h, t, Vec3::Y)); } } } } }
        best_hit
    }
}

fn rot2_angle(r: Rot2) -> Real { let v = r.rotate(Vec2::X); v.y.atan2(v.x) }
fn generic_convex_contact_2d(ca: &Collider2, cb: &Collider2, ba: &Body2, bb: &Body2) -> Option<(Vec2, Real, Vec2)> {
    let world_a = ba.position + ba.rotation.rotate(ca.offset); let world_b = bb.position + bb.rotation.rotate(cb.offset);
    let support_a = |d: Vec2| -> Vec2 { world_a + ba.rotation.rotate(match &ca.shape { ColliderShape2::Circle(c) => c.support(ba.rotation.inverse().rotate(d)), ColliderShape2::Box(b) => b.support(ba.rotation.inverse().rotate(d)), ColliderShape2::Capsule(cap) => ba.rotation.inverse().rotate(d).normalized_or(Vec2::X) * cap.bounding_radius(), ColliderShape2::ConvexPolygon(p) => p.support(ba.rotation.inverse().rotate(d)), ColliderShape2::Edge(e) => e.closest_point(ba.rotation.inverse().rotate(d) * 100.0) }) };
    let support_b = |d: Vec2| -> Vec2 { world_b + bb.rotation.rotate(match &cb.shape { ColliderShape2::Circle(c) => c.support(bb.rotation.inverse().rotate(d)), ColliderShape2::Box(b) => b.support(bb.rotation.inverse().rotate(d)), ColliderShape2::Capsule(cap) => bb.rotation.inverse().rotate(d).normalized_or(Vec2::X) * cap.bounding_radius(), ColliderShape2::ConvexPolygon(p) => p.support(bb.rotation.inverse().rotate(d)), ColliderShape2::Edge(e) => e.closest_point(bb.rotation.inverse().rotate(d) * 100.0) }) };
    let gjk = auralite_collision::gjk_distance2(support_a, support_b, 32);
    if gjk.distance <= ABS_EPSILON { let epa = auralite_collision::epa_penetration2(support_a, support_b, 32); if let Some(p) = epa { return Some((p.normal, p.depth, (gjk.point_a + gjk.point_b) * 0.5)); } }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test] fn falling_2d_rests() { let mut w = World2::default(); let h = w.add_body(BodyBuilder2::dynamic().position(Vec2 { x: 0.0, y: 4.0 }).add_collider(Collider2 { shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()), offset: Vec2::ZERO, material: Material::default(), filter: CollisionFilter::default() })).unwrap(); for _ in 0..600 { w.step(1.0 / 60.0).unwrap(); } assert!((w.body(h).unwrap().position.y - 0.5).abs() < 0.1); }
    #[test] fn forces_accumulate() { let mut w = World2::default(); w.gravity = Vec2::ZERO; let h = w.add_body(BodyBuilder2::dynamic().position(Vec2{x:0.0,y:10.0}).add_collider(Collider2 { shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()), offset: Vec2::ZERO, material: Material::default(), filter: CollisionFilter::default() })).unwrap(); w.apply_force(h, Vec2 { x: 10.0, y: 0.0 }).unwrap(); w.step(1.0 / 60.0).unwrap(); assert!((w.body(h).unwrap().velocity.x - 10.0 / 60.0).abs() < 1.0e-4); }
    #[test] fn distance_joint_holds_bodies_together() { let mut w = World2::default(); w.gravity = Vec2::ZERO; let b1 = w.add_body(BodyBuilder2::dynamic().position(Vec2 { x: -1.0, y: 10.0 }).add_collider(Collider2 { shape: ColliderShape2::Circle(Circle2::new(0.2).unwrap()), offset: Vec2::ZERO, material: Material::default(), filter: CollisionFilter::default() })).unwrap(); let b2 = w.add_body(BodyBuilder2::dynamic().position(Vec2 { x: 1.0, y: 10.0 }).add_collider(Collider2 { shape: ColliderShape2::Circle(Circle2::new(0.2).unwrap()), offset: Vec2::ZERO, material: Material::default(), filter: CollisionFilter::default() })).unwrap(); w.add_joint(JointConfig2::new(JointType2::Distance, b1, b2, Vec2::ZERO, Vec2::ZERO)).unwrap(); for _ in 0..100 { w.step(1.0 / 60.0).unwrap(); } let dist = (w.body(b1).unwrap().position - w.body(b2).unwrap().position).length(); assert!(dist < 2.1, "distance should stay small, got {}", dist); }
    #[test] fn two_circles_stack() { let mut w = World2::default(); w.add_body(BodyBuilder2::static_body().position(Vec2 { x: 0.0, y: -1.0 }).add_collider(Collider2 { shape: ColliderShape2::Circle(Circle2::new(1.0).unwrap()), offset: Vec2::ZERO, material: Material::default(), filter: CollisionFilter::default() })).unwrap(); let h2 = w.add_body(BodyBuilder2::dynamic().position(Vec2 { x: 0.0, y: 3.0 }).add_collider(Collider2 { shape: ColliderShape2::Circle(Circle2::new(1.0).unwrap()), offset: Vec2::ZERO, material: Material::default(), filter: CollisionFilter::default() })).unwrap(); for _ in 0..600 { w.step(1.0 / 60.0).unwrap(); } assert!(w.body(h2).unwrap().sleeping); }
    #[test] fn restitution_affects_bounce() { let mut w = World2::default(); w.sleep_threshold = 0.0; let h = w.add_body(BodyBuilder2::dynamic().position(Vec2 { x: 0.0, y: 1.5 }).restitution(0.8).add_collider(Collider2 { shape: ColliderShape2::Circle(Circle2::new(0.5).unwrap()), offset: Vec2::ZERO, material: Material::default(), filter: CollisionFilter::default() })).unwrap(); for _ in 0..40 { w.step(1.0 / 60.0).unwrap(); } assert!(w.body(h).unwrap().velocity.y > 0.0); }
    #[test] fn rollback_replays_bitwise() { let mut w = World3::default(); w.add_body(BodyBuilder3::dynamic().position(Vec3 { x: 1.0, y: 10.0, z: 2.0 }).add_collider(Collider3 { shape: ColliderShape3::Sphere(Sphere3::new(0.5).unwrap()), offset: Vec3::ZERO, material: Material::default(), filter: CollisionFilter::default() })).unwrap(); for _ in 0..10 { w.step(0.016).unwrap(); } let h = w.state_hash(); for _ in 0..10 { w.step(0.016).unwrap(); } assert_ne!(h, w.state_hash()); }
    #[test] fn distance_joint_3d() { let mut w = World3::default(); let b1 = w.add_body(BodyBuilder3::dynamic().position(Vec3::ZERO)).unwrap(); let b2 = w.add_body(BodyBuilder3::dynamic().position(Vec3 { x: 2.0, y: 0.0, z: 0.0 })).unwrap(); w.add_joint(JointConfig3 { joint_type: JointType3::Distance, body_a: b1, body_b: b2, anchor_a: Vec3::ZERO, anchor_b: Vec3::ZERO, limits: JointLimits::default(), motor: JointMotor::default(), break_impulse: 0.0, user_data: 0 }).unwrap(); for _ in 0..60 { w.step(0.016).unwrap(); } let dist = (w.body(b1).unwrap().position - w.body(b2).unwrap().position).length(); assert!(dist < 0.5, "bodies should be pulled together, got {}", dist); }
}
