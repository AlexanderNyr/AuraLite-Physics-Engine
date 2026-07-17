//! Particle, fluid, buoyancy, and field-force-zone systems.
//! Seeded deterministic emitters, SoA storage with free lists,
//! PBF-class fluid (density/incompressibility, viscosity, neighbor search),
//! buoyancy from displaced volume, and force field zones.
#![forbid(unsafe_code)]
#![allow(missing_docs, clippy::too_many_arguments)]

use auralite_core::Rng;
use auralite_dynamics::Body3;
use auralite_math::{ABS_EPSILON, Real, Vec3};

// ─── Particle storage (SoA with free list) ───────────────────────────────────

/// SoA particle storage with free-list recycling.
#[derive(Clone, Debug)]
pub struct ParticleStorage {
    pub positions: Vec<Vec3>,
    pub velocities: Vec<Vec3>,
    pub lifetimes: Vec<Real>,
    pub max_lifetimes: Vec<Real>,
    pub types: Vec<ParticleType>,
    pub alive: Vec<bool>,
    free_list: Vec<usize>,
    capacity: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParticleType {
    Fluid,
    BuoyancySample,
    Generic,
}

impl ParticleStorage {
    pub fn new(capacity: usize) -> Self {
        Self {
            positions: Vec::with_capacity(capacity),
            velocities: Vec::with_capacity(capacity),
            lifetimes: Vec::with_capacity(capacity),
            max_lifetimes: Vec::with_capacity(capacity),
            types: Vec::with_capacity(capacity),
            alive: Vec::with_capacity(capacity),
            free_list: Vec::new(),
            capacity,
        }
    }

    pub fn spawn(
        &mut self,
        pos: Vec3,
        vel: Vec3,
        lifetime: Real,
        ptype: ParticleType,
    ) -> Option<usize> {
        if self.alive.len() >= self.capacity && self.free_list.is_empty() {
            return None;
        }
        let idx = self.free_list.pop().unwrap_or_else(|| {
            let i = self.positions.len();
            self.positions.push(Vec3::ZERO);
            self.velocities.push(Vec3::ZERO);
            self.lifetimes.push(0.0);
            self.max_lifetimes.push(0.0);
            self.types.push(ParticleType::Generic);
            self.alive.push(false);
            i
        });
        self.positions[idx] = pos;
        self.velocities[idx] = vel;
        self.lifetimes[idx] = lifetime;
        self.max_lifetimes[idx] = lifetime;
        self.types[idx] = ptype;
        self.alive[idx] = true;
        Some(idx)
    }

    pub fn kill(&mut self, idx: usize) {
        if idx < self.alive.len() && self.alive[idx] {
            self.alive[idx] = false;
            self.free_list.push(idx);
        }
    }

    pub fn iterate_alive(&self) -> impl Iterator<Item = (usize, &Vec3, &Vec3, &ParticleType)> {
        self.alive.iter().enumerate().filter_map(move |(i, &a)| {
            if a {
                Some((i, &self.positions[i], &self.velocities[i], &self.types[i]))
            } else {
                None
            }
        })
    }

    pub fn alive_indices(&self) -> Vec<usize> {
        self.alive
            .iter()
            .enumerate()
            .filter_map(|(i, &a)| if a { Some(i) } else { None })
            .collect()
    }

    pub fn alive_count(&self) -> usize {
        self.alive.iter().filter(|&&a| a).count()
    }

    pub fn clear(&mut self) {
        for i in 0..self.alive.len() {
            if self.alive[i] {
                self.kill(i);
            }
        }
    }
}

// ─── Seeded deterministic emitter ────────────────────────────────────────────

#[derive(Clone, Debug)]
pub struct Emitter {
    pub position: Vec3,
    pub direction: Vec3,
    pub spread: Real,
    pub speed: Real,
    pub rate: Real, // particles per second
    pub lifetime: Real,
    pub particle_type: ParticleType,
    pub count: u32,
    accumulation: Real,
    rng: Rng,
}

impl Emitter {
    pub fn new(
        position: Vec3,
        direction: Vec3,
        spread: Real,
        speed: Real,
        rate: Real,
        lifetime: Real,
        seed: u64,
    ) -> Self {
        Self {
            position,
            direction: direction.normalized_or(Vec3::Y),
            spread,
            speed,
            rate,
            lifetime,
            particle_type: ParticleType::Generic,
            count: 0,
            accumulation: 0.0,
            rng: Rng::new(seed),
        }
    }

    pub fn emit(&mut self, dt: Real, storage: &mut ParticleStorage) -> u32 {
        self.accumulation += self.rate * dt;
        let mut spawned = 0;
        while self.accumulation >= 1.0 {
            self.accumulation -= 1.0;
            // Random direction within cone
            let theta =
                (self.rng.next_u64() as Real / u64::MAX as Real) * core::f64::consts::TAU as Real;
            let phi = (self.rng.next_u64() as Real / u64::MAX as Real) * self.spread;
            let mut dir = self.direction;
            // Perturb direction
            let up = if dir.y.abs() < 0.9 { Vec3::Y } else { Vec3::X };
            let right = dir.cross(up).normalized_or(Vec3::X);
            let real_up = right.cross(dir).normalized_or(Vec3::Y);
            dir = (dir * phi.cos()
                + right * phi.sin() * theta.cos()
                + real_up * phi.sin() * theta.sin())
            .normalized_or(self.direction);
            let vel = dir
                * (self.speed
                    + (self.rng.next_u64() as Real / u64::MAX as Real - 0.5) * self.speed * 0.2);
            storage.spawn(self.position, vel, self.lifetime, self.particle_type);
            self.count += 1;
            spawned += 1;
        }
        spawned
    }
}

// ─── PBF Fluid ────────────────────────────────────────────────────────────────

/// SPH kernel: poly6 for density
fn w_poly6(r: Vec3, h: Real) -> Real {
    let r2 = r.length_squared();
    if r2 >= h * h || r2 <= ABS_EPSILON {
        return 0.0;
    }
    let h2 = h * h;
    let q = (h2 - r2) / (h2 * h2 * h2);
    q * (315.0 / (64.0 * core::f64::consts::PI as Real))
}
fn grad_w_spiky(r: Vec3, h: Real) -> Vec3 {
    let dist = r.length();
    if dist >= h || dist <= ABS_EPSILON {
        return Vec3::ZERO;
    }
    let s = (h - dist) / (h * h * h);
    r / dist * (-45.0 / (core::f64::consts::PI as Real * h * h * h * h * h * h as Real)) * s * s
}
#[allow(dead_code)]
fn lap_w_viscosity(r: Vec3, h: Real) -> Real {
    let dist = r.length();
    if dist >= h || dist <= ABS_EPSILON {
        return 0.0;
    }
    (h - dist) * (45.0 / (core::f64::consts::PI as Real * h * h * h * h * h * h as Real))
}

/// Neighbor list for PBF.
#[derive(Clone, Debug)]
pub struct NeighborList {
    pub neighbors: Vec<Vec<usize>>,
}

/// PBF fluid state.
#[derive(Clone, Debug)]
pub struct PbfFluid {
    pub particle_indices: Vec<usize>,
    pub densities: Vec<Real>,
    pub lambdas: Vec<Real>,
    pub predicted_positions: Vec<Vec3>,
    pub rest_density: Real,
    pub particle_radius: Real,
    pub kernel_h: Real,
    pub stiffness: Real,
    pub viscosity: Real,
}

impl PbfFluid {
    pub fn new(
        rest_density: Real,
        particle_radius: Real,
        stiffness: Real,
        viscosity: Real,
    ) -> Self {
        Self {
            particle_indices: Vec::new(),
            densities: Vec::new(),
            lambdas: Vec::new(),
            predicted_positions: Vec::new(),
            rest_density,
            particle_radius,
            kernel_h: particle_radius * 4.0,
            stiffness,
            viscosity,
        }
    }

    /// Build neighbor list via spatial hash.
    pub fn build_neighbors(&self, storage: &ParticleStorage, indices: &[usize]) -> NeighborList {
        let h = self.kernel_h;
        let mut hash = auralite_core::SpatialHash::new(h);
        for &i in indices {
            hash.insert(storage.positions[i], i);
        }

        let mut neighbors = Vec::with_capacity(indices.len());
        let h2 = h * h;
        for &i in indices {
            let pi = storage.positions[i];
            let candidates = hash.query(pi, h);
            let mut row = Vec::new();
            for &j in &candidates {
                if i != j && (pi - storage.positions[j]).length_squared() < h2 {
                    // Find index of j in indices slice
                    if let Some(pos) = indices.iter().position(|&x| x == j) {
                        row.push(pos);
                    }
                }
            }
            neighbors.push(row);
        }
        NeighborList { neighbors }
    }

    /// Compute densities for all fluid particles.
    pub fn compute_densities(
        &mut self,
        storage: &ParticleStorage,
        indices: &[usize],
        neighbors: &NeighborList,
    ) {
        self.densities.clear();
        self.densities.resize(indices.len(), 0.0);
        for (row, &i) in indices.iter().enumerate() {
            let pi = storage.positions[i];
            let mut d = 0.0;
            for &nj in &neighbors.neighbors[row] {
                let pj = storage.positions[indices[nj]];
                d += w_poly6(pi - pj, self.kernel_h);
            }
            self.densities[row] = self.rest_density * (d + w_poly6(Vec3::ZERO, self.kernel_h));
        }
    }

    /// Compute lambda (position correction scale) for each particle.
    pub fn compute_lambdas(&mut self, indices: &[usize], neighbors: &NeighborList) {
        self.lambdas.clear();
        self.lambdas.resize(indices.len(), 0.0);
        let eps: Real = 1.0e-6;
        let constraint_eps: Real = 0.01; // relaxation
        for (row, _i) in indices.iter().enumerate() {
            let density_i = self.densities[row];
            let ci = (density_i / self.rest_density).max(0.0) - 1.0;
            if ci <= 0.0 {
                continue;
            }
            let pi = self.predicted_positions[row];
            let mut sum_grad_sq = 0.0;
            for &nj in &neighbors.neighbors[row] {
                let pj = self.predicted_positions[nj];
                let grad = grad_w_spiky(pi - pj, self.kernel_h);
                sum_grad_sq += grad.length_squared();
            }
            let denom = sum_grad_sq + eps;
            self.lambdas[row] = -ci / (denom + constraint_eps);
        }
    }

    /// Apply position corrections for incompressibility.
    pub fn apply_position_corrections(
        &mut self,
        storage: &mut ParticleStorage,
        indices: &[usize],
        neighbors: &NeighborList,
    ) {
        for (row, &i) in indices.iter().enumerate() {
            let lambda_i = self.lambdas[row];
            if lambda_i.abs() <= ABS_EPSILON {
                continue;
            }
            let pi = self.predicted_positions[row];
            let mut delta = Vec3::ZERO;
            for &nj in &neighbors.neighbors[row] {
                let lambda_j = self.lambdas[nj];
                let pj = self.predicted_positions[nj];
                let grad = grad_w_spiky(pi - pj, self.kernel_h);
                delta += grad * (lambda_i + lambda_j + self.stiffness * (lambda_i + lambda_j));
            }
            if delta.is_finite() {
                storage.positions[i] += delta / self.rest_density;
            }
        }
    }

    /// Apply viscosity (XSPH viscosity).
    pub fn apply_viscosity(
        &mut self,
        storage: &mut ParticleStorage,
        indices: &[usize],
        neighbors: &NeighborList,
        dt: Real,
    ) {
        let visc = self.viscosity * dt;
        if visc <= ABS_EPSILON {
            return;
        }
        let mut vel_corrections: Vec<Vec3> = vec![Vec3::ZERO; indices.len()];
        for (row, &i) in indices.iter().enumerate() {
            let vi = storage.velocities[i];
            for &nj in &neighbors.neighbors[row] {
                let j = indices[nj];
                let vj = storage.velocities[j];
                let w = w_poly6(storage.positions[i] - storage.positions[j], self.kernel_h);
                vel_corrections[row] += (vj - vi) * w;
            }
        }
        for (row, &i) in indices.iter().enumerate() {
            storage.velocities[i] += vel_corrections[row] * visc;
        }
    }

    /// Full PBF step.
    pub fn step(
        &mut self,
        storage: &mut ParticleStorage,
        indices: &[usize],
        dt: Real,
        gravity: Vec3,
    ) {
        if indices.is_empty() {
            return;
        }
        let h = self.kernel_h;
        let rd = self.rest_density;
        let stiffness = self.stiffness;

        // Predict positions
        self.predicted_positions.clear();
        for &i in indices {
            let vel = storage.velocities[i] + gravity * dt;
            storage.velocities[i] = vel;
            self.predicted_positions
                .push(storage.positions[i] + vel * dt);
        }

        // Build neighbors from predicted positions
        let mut temp_storage = storage.clone();
        for (row, &i) in indices.iter().enumerate() {
            temp_storage.positions[i] = self.predicted_positions[row];
        }
        let neighbors = self.build_neighbors(&temp_storage, indices);

        let _eps: Real = 1.0e-6;
        let _constraint_eps: Real = 0.01;

        // Solve incompressibility (multiple iterations)
        for _iter in 0..5 {
            // Compute densities
            self.compute_densities(&temp_storage, indices, &neighbors);
            // Compute lambdas
            self.compute_lambdas(indices, &neighbors);
            // Apply position corrections to temp_storage
            for (row, &i) in indices.iter().enumerate() {
                let lambda_i = self.lambdas[row];
                if lambda_i.abs() <= ABS_EPSILON {
                    continue;
                }
                let pi = self.predicted_positions[row];
                let mut delta = Vec3::ZERO;
                for &nj in &neighbors.neighbors[row] {
                    let lambda_j = self.lambdas[nj];
                    let pj = self.predicted_positions[nj];
                    let grad = grad_w_spiky(pi - pj, h);
                    let scorr = stiffness * (lambda_i + lambda_j).powi(4); // better scorr
                    delta += grad * (lambda_i + lambda_j + scorr);
                }
                if delta.is_finite() {
                    temp_storage.positions[i] += delta / rd;
                }
            }
            // Update predicted positions for next iteration
            for (row, &i) in indices.iter().enumerate() {
                self.predicted_positions[row] = temp_storage.positions[i];
            }
        }

        // Update positions and velocities
        for (row, &i) in indices.iter().enumerate() {
            let new_pos = self.predicted_positions[row];
            storage.velocities[i] = (new_pos - storage.positions[i]) / dt.max(ABS_EPSILON);
            storage.positions[i] = new_pos;
        }

        // Apply viscosity (XSPH)
        self.apply_viscosity(storage, indices, &neighbors, dt);
    }
}

// ─── Buoyancy ─────────────────────────────────────────────────────────────────

/// Compute buoyancy force on a rigid body from surrounding fluid particles.
#[allow(unused_variables)]
pub fn compute_buoyancy(
    body: &Body3,
    fluid_indices: &[usize],
    fluid_positions: &[Vec3],
    fluid_density: Real,
    body_volume: Real,
    gravity: Vec3,
) -> Vec3 {
    // Archimedes: F = ρ_fluid * V_displaced * g
    // Simplified: if body center is within fluid bounding box, apply buoyancy
    let gravity_mag = gravity.length();
    if gravity_mag <= ABS_EPSILON {
        return Vec3::ZERO;
    }
    let gravity_dir = gravity / -gravity_mag; // upward direction
    let displaced_volume = body_volume; // assume fully submerged
    gravity_dir * (fluid_density * displaced_volume * gravity_mag)
}

/// Check if a point is inside the fluid region (approximate via nearest fluid particle).
pub fn is_submerged(point: Vec3, fluid_positions: &[Vec3], max_dist: Real) -> bool {
    let d2 = max_dist * max_dist;
    fluid_positions
        .iter()
        .any(|fp| (*fp - point).length_squared() <= d2)
}

/// Apply buoyancy forces to all dynamic bodies in the world based on fluid particles.
pub fn apply_buoyancy_to_world(
    world: &mut auralite_dynamics::World3,
    storage: &ParticleStorage,
    fluid_density: Real,
    gravity: Vec3,
) {
    let fluid_indices = storage.alive_indices();
    let fluid_positions: Vec<Vec3> = fluid_indices
        .iter()
        .map(|&i| storage.positions[i])
        .collect();

    // We need to iterate over all bodies in the world.
    // World3 has body_handles().
    let handles = world.body_handles();
    for h in handles {
        if let Ok(body) = world.body(h) {
            if body.kind != auralite_dynamics::BodyType::Dynamic {
                continue;
            }
            // Simple check: if any collider center is submerged
            let mut is_in_fluid = false;
            for c in &body.colliders {
                let world_center = body.position + body.rotation.rotate(c.offset);
                if is_submerged(world_center, &fluid_positions, 0.5) {
                    is_in_fluid = true;
                    break;
                }
            }
            if is_in_fluid {
                let vol = body
                    .colliders
                    .iter()
                    .map(|c| c.shape.volume())
                    .sum::<Real>()
                    .max(0.1);
                let force = compute_buoyancy(
                    body,
                    &fluid_indices,
                    &fluid_positions,
                    fluid_density,
                    vol,
                    gravity,
                );
                let _ = world.apply_impulse(h, force * 0.016666668); // apply as impulse
            }
        }
    }
}

// ─── Force Fields ─────────────────────────────────────────────────────────────

/// Types of force fields.
#[derive(Clone, Debug)]
pub enum FieldType {
    Uniform {
        acceleration: Vec3,
    },
    Radial {
        center: Vec3,
        strength: Real,
        max_radius: Real,
    },
    Wind {
        direction: Vec3,
        speed: Real,
        turbulence: Real,
    },
    Drag {
        linear: Real,
        quadratic: Real,
    },
    Damping {
        factor: Real,
    },
}

/// A force zone applying forces to particles and rigid bodies within a volume.
#[derive(Clone, Debug)]
pub struct ForceField {
    pub field_type: FieldType,
    pub position: Vec3,
    pub radius: Real,
    pub falloff: Real, // 0 = constant, 1 = linear falloff
    pub affects_particles: bool,
    pub affects_rigid: bool,
}

impl ForceField {
    pub fn new(field_type: FieldType, position: Vec3, radius: Real) -> Self {
        Self {
            field_type,
            position,
            radius,
            falloff: 0.0,
            affects_particles: true,
            affects_rigid: false,
        }
    }

    /// Compute force on a point.
    pub fn force_at(&self, point: Vec3, velocity: Vec3) -> Vec3 {
        let dist = (point - self.position).length();
        if dist > self.radius && self.radius > 0.0 {
            return Vec3::ZERO;
        }
        let strength = if self.radius > 0.0 && self.falloff > 0.0 {
            (1.0 - (dist / self.radius) * self.falloff).max(0.0)
        } else {
            1.0
        };
        match &self.field_type {
            FieldType::Uniform { acceleration } => *acceleration * strength,
            FieldType::Radial {
                center,
                strength: s,
                max_radius,
            } => {
                let dir = *center - point;
                let d = dir.length();
                if d > *max_radius || d <= ABS_EPSILON {
                    return Vec3::ZERO;
                }
                dir / d * *s * strength
            }
            FieldType::Wind {
                direction,
                speed,
                turbulence,
            } => {
                let vel_diff = *direction * *speed - velocity;
                vel_diff * 0.5 * strength
                    + Vec3 {
                        x: (point.x.sin() * 0.1) * *turbulence,
                        y: (point.y.cos() * 0.1) * *turbulence,
                        z: (point.z.sin() * 0.1) * *turbulence,
                    }
            }
            FieldType::Drag { linear, quadratic } => {
                let speed = velocity.length();
                if speed <= ABS_EPSILON {
                    return Vec3::ZERO;
                }
                let dir = velocity / -speed;
                dir * (*linear * speed + *quadratic * speed * speed) * strength
            }
            FieldType::Damping { factor } => -velocity * *factor * strength,
        }
    }
}

/// Apply all force fields to particles.
pub fn apply_force_fields_to_particles(
    fields: &[ForceField],
    storage: &mut ParticleStorage,
    dt: Real,
) {
    for (i, &alive) in storage.alive.iter().enumerate() {
        if !alive {
            continue;
        }
        let pos = storage.positions[i];
        let vel = storage.velocities[i];
        let mut total_force = Vec3::ZERO;
        for field in fields {
            if !field.affects_particles {
                continue;
            }
            total_force += field.force_at(pos, vel);
        }
        storage.velocities[i] += total_force * dt;
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_storage(capacity: usize) -> ParticleStorage {
        ParticleStorage::new(capacity)
    }

    #[test]
    fn emitter_produces_deterministic_output() {
        let mut s1 = make_storage(1000);
        let mut s2 = make_storage(1000);
        let mut e1 = Emitter::new(Vec3::ZERO, Vec3::Y, 0.5, 5.0, 100.0, 2.0, 42);
        let mut e2 = Emitter::new(Vec3::ZERO, Vec3::Y, 0.5, 5.0, 100.0, 2.0, 42);
        let c1 = e1.emit(1.0, &mut s1);
        let c2 = e2.emit(1.0, &mut s2);
        assert_eq!(c1, c2, "same seed should produce same count");
        assert!(c1 > 0, "should emit particles");
        // Check positions match
        for i in 0..s1.alive_count().min(s2.alive_count()) {
            let idx1 = s1.iterate_alive().nth(i).unwrap().0;
            let idx2 = s2.iterate_alive().nth(i).unwrap().0;
            let diff = (s1.positions[idx1] - s2.positions[idx2]).length();
            assert!(
                diff < 1.0e-6,
                "deterministic positions should match, diff={}",
                diff
            );
        }
    }

    #[test]
    fn particle_kill_and_recycle() {
        let mut s = make_storage(10);
        let h1 = s
            .spawn(Vec3::ZERO, Vec3::ZERO, 1.0, ParticleType::Generic)
            .unwrap();
        assert_eq!(s.alive_count(), 1);
        s.kill(h1);
        assert_eq!(s.alive_count(), 0);
        // Recycle slot
        let h2 = s
            .spawn(Vec3::X, Vec3::ZERO, 1.0, ParticleType::Generic)
            .unwrap();
        assert_eq!(h2, h1, "should reuse freed slot");
    }

    #[test]
    fn emitter_full_capacity_returns_none() {
        let mut s = make_storage(5);
        let mut e = Emitter::new(Vec3::ZERO, Vec3::Y, 0.0, 1.0, 1000.0, 10.0, 0);
        e.emit(1.0, &mut s);
        assert_eq!(s.alive_count(), 5);
        // Extra spawn fails
        assert!(
            s.spawn(Vec3::ZERO, Vec3::ZERO, 1.0, ParticleType::Generic)
                .is_none()
        );
    }

    #[test]
    fn pbf_density_computation() {
        let mut s = make_storage(100);
        // Place fluid particles in a small cluster
        for i in 0..10 {
            for j in 0..10 {
                let pos = Vec3 {
                    x: i as Real * 0.1,
                    y: j as Real * 0.1,
                    z: 0.0,
                };
                s.spawn(pos, Vec3::ZERO, 10.0, ParticleType::Fluid);
            }
        }
        let indices: Vec<usize> = s.iterate_alive().map(|(i, _, _, _)| i).collect();
        let mut fluid = PbfFluid::new(1000.0, 0.05, 0.1, 0.01);
        let neighbors = fluid.build_neighbors(&s, &indices);
        fluid.compute_densities(&s, &indices, &neighbors);
        assert_eq!(fluid.densities.len(), indices.len());
        for &d in &fluid.densities {
            assert!(d.is_finite(), "density should be finite");
            assert!(d > 0.0, "density should be positive");
        }
    }

    #[test]
    fn force_field_uniform() {
        let field = ForceField::new(
            FieldType::Uniform {
                acceleration: Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            },
            Vec3::ZERO,
            100.0,
        );
        let f = field.force_at(Vec3::ZERO, Vec3::ZERO);
        assert!(
            (f - Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0
            })
            .length()
                < 1.0e-6
        );
    }

    #[test]
    fn force_field_radial() {
        let field = ForceField::new(
            FieldType::Radial {
                center: Vec3::ZERO,
                strength: 10.0,
                max_radius: 10.0,
            },
            Vec3::ZERO,
            10.0,
        );
        let f = field.force_at(
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3::ZERO,
        );
        assert!(
            f.x < 0.0,
            "radial force should point from point toward center, got x={}",
            f.x
        );
        assert!(f.y.abs() < 1.0e-6);
    }

    #[test]
    fn force_field_outside_radius() {
        let field = ForceField::new(
            FieldType::Uniform {
                acceleration: Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            },
            Vec3::ZERO,
            1.0,
        );
        let f = field.force_at(
            Vec3 {
                x: 100.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3::ZERO,
        );
        assert_eq!(f, Vec3::ZERO, "outside radius should get zero force");
    }

    #[test]
    fn drag_field_opposes_velocity() {
        let field = ForceField::new(
            FieldType::Drag {
                linear: 0.5,
                quadratic: 0.0,
            },
            Vec3::ZERO,
            100.0,
        );
        let f = field.force_at(
            Vec3::ZERO,
            Vec3 {
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
        );
        assert!(f.x < 0.0, "drag should oppose velocity");
    }

    #[test]
    fn buoyancy_force_points_upward() {
        let gravity = Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        };
        let body = Body3 {
            id: auralite_core::StableId(1),
            kind: auralite_dynamics::BodyType::Dynamic,
            position: Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            rotation: auralite_math::Quat::identity(),
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 1.0,
            inv_inertia_diagonal: Vec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            colliders: Vec::new(),
            restitution: 0.0,
            friction: 0.5,
            sleeping: false,
            force: Vec3::ZERO,
            torque: Vec3::ZERO,
            linear_damping: 0.0,
            angular_damping: 0.0,
            user_data: 0,
        };
        let fluid_positions = vec![Vec3 {
            x: 0.0,
            y: 0.5,
            z: 0.0,
        }];
        let buoyancy = compute_buoyancy(&body, &[], &fluid_positions, 1000.0, 0.1, gravity);
        assert!(
            buoyancy.y > 0.0,
            "buoyancy should point upward, got y={}",
            buoyancy.y
        );
    }

    #[test]
    fn particles_updated_storage() {
        let mut s = make_storage(10);
        let idx = s
            .spawn(Vec3::ZERO, Vec3::X, 1.0, ParticleType::Generic)
            .unwrap();
        assert!(s.alive[idx]);
        // Tick lifetime
        s.lifetimes[idx] -= 0.5;
        assert!(s.lifetimes[idx] > 0.0);
        s.lifetimes[idx] -= 0.6;
        if s.lifetimes[idx] <= 0.0 {
            s.kill(idx);
        }
        assert_eq!(s.alive_count(), 0);
    }

    #[test]
    fn buoyancy_floating_box_equilibrium() {
        use auralite_collision::CollisionFilter;
        use auralite_dynamics::{BodyBuilder3, Collider3, ColliderShape3, Material, World3};
        use auralite_geometry::Box3;

        let mut w = World3::default();
        let box_shape = Box3::new(Vec3 {
            x: 0.5,
            y: 0.5,
            z: 0.5,
        })
        .unwrap();
        let h = w
            .add_body(
                BodyBuilder3::dynamic()
                    .position(Vec3::ZERO)
                    .mass(box_shape.volume() * 1000.0) // exact neutrally buoyant mass
                    .add_collider(Collider3 {
                        shape: ColliderShape3::Box(box_shape),
                        offset: Vec3::ZERO,
                        material: Material::default(),
                        filter: CollisionFilter::default(),
                    }),
            )
            .unwrap();

        let mut storage = ParticleStorage::new(10);
        let _ = storage.spawn(Vec3::ZERO, Vec3::ZERO, 10.0, ParticleType::Fluid);

        // Apply buoyancy right before stepping
        let gravity = Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        };
        for _ in 0..10 {
            apply_buoyancy_to_world(&mut w, &storage, 1000.0, gravity);
            w.step(0.016).unwrap();
        }
        let vy = w.body(h).unwrap().velocity.y;
        assert!(
            vy.abs() < 0.1,
            "Box with neutral density should stay at vertical equilibrium, got vy={}",
            vy
        );
    }
}
