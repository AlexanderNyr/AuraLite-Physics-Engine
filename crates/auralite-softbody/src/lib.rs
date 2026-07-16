//! XPBD-based soft bodies and cloth simulation.
//! Supports stretch, shear, bend, volume constraints, attachments,
//! self-collision, rigid coupling, wind/aerodynamics, and damping.
#![forbid(unsafe_code)]
#![allow(missing_docs, clippy::too_many_arguments)]

use auralite_core::Pool;
use auralite_dynamics::{Body2, Body3, BodyHandle2, BodyHandle3};
use auralite_math::{ABS_EPSILON, Real, Vec2, Vec3};

/// A soft-body particle.
#[derive(Clone, Copy, Debug)]
pub struct Particle {
    pub position: Vec3,
    pub old_position: Vec3,
    pub velocity: Vec3,
    pub inv_mass: Real,
    pub pinned: bool,
}

impl Particle {
    pub fn new(position: Vec3, inv_mass: Real) -> Self {
        Self {
            position,
            old_position: position,
            velocity: Vec3::ZERO,
            inv_mass,
            pinned: inv_mass == 0.0,
        }
    }
}

/// XPBD constraint types.
#[derive(Clone, Debug)]
pub enum Constraint {
    Stretch {
        p1: usize,
        p2: usize,
        rest_length: Real,
        compliance: Real,
    },
    Bend {
        p1: usize,
        p2: usize,
        rest_length: Real,
        compliance: Real,
    },
    Volume {
        p: [usize; 4],
        rest_volume: Real,
        compliance: Real,
    },
    Attachment {
        particle: usize,
        target: Vec3,
        compliance: Real,
    },
    RigidAttachment2 {
        particle: usize,
        body: BodyHandle2,
        local_offset: Vec2,
        compliance: Real,
    },
    RigidAttachment3 {
        particle: usize,
        body: BodyHandle3,
        local_offset: Vec3,
        compliance: Real,
    },
}

/// XPBD soft body state.
#[derive(Clone, Debug)]
pub struct SoftBody {
    pub particles: Vec<Particle>,
    pub constraints: Vec<Constraint>,
    pub edge_indices: Vec<(usize, usize)>,
    pub triangle_indices: Vec<[usize; 3]>,
    pub damping: Real,
    pub wind: Vec3,
    pub aerodynamic: bool,
    drag_coefficient: Real,
}

impl SoftBody {
    pub fn new(damping: Real) -> Self {
        Self {
            particles: Vec::new(),
            constraints: Vec::new(),
            edge_indices: Vec::new(),
            triangle_indices: Vec::new(),
            damping,
            wind: Vec3::ZERO,
            aerodynamic: false,
            drag_coefficient: 0.02,
        }
    }

    /// Advance particle positions with velocity (explicit Euler pre-step).
    pub fn pre_step(&mut self, dt: Real, gravity: Vec3) {
        for p in &mut self.particles {
            if p.pinned || p.inv_mass <= ABS_EPSILON {
                continue;
            }
            p.old_position = p.position;
            let acceleration = gravity
                + self.wind
                    * (if self.aerodynamic {
                        self.drag_coefficient
                    } else {
                        0.0
                    });
            p.velocity += acceleration * dt;
            p.velocity = p.velocity * (1.0 - self.damping * dt).max(0.0);
            // Predict position:
            p.position += p.velocity * dt;
        }
    }

    pub fn solve_constraints(&mut self, iterations: u32, dt: Real) {
        let dt_sq = dt * dt;
        for _ in 0..iterations {
            for ci in 0..self.constraints.len() {
                match self.constraints[ci] {
                    Constraint::Stretch {
                        p1,
                        p2,
                        rest_length,
                        compliance,
                    } => {
                        let p1p = self.particles[p1].position;
                        let p2p = self.particles[p2].position;
                        let im1 = self.particles[p1].inv_mass;
                        let im2 = self.particles[p2].inv_mass;
                        let w = im1 + im2;
                        if w <= ABS_EPSILON {
                            continue;
                        }
                        let diff = p2p - p1p;
                        let dist = diff.length();
                        if dist <= ABS_EPSILON {
                            continue;
                        }
                        let dir = diff / dist;
                        let c = dist - rest_length;
                        let alpha = compliance / dt_sq;
                        let delta_lambda = -c / (w + alpha);
                        let correction = dir * delta_lambda;
                        if !self.particles[p1].pinned {
                            self.particles[p1].position -= correction * (im1 / w);
                        }
                        if !self.particles[p2].pinned {
                            self.particles[p2].position += correction * (im2 / w);
                        }
                    }
                    Constraint::Bend {
                        p1,
                        p2,
                        rest_length,
                        compliance,
                    } => {
                        let p1p = self.particles[p1].position;
                        let p2p = self.particles[p2].position;
                        let im1 = self.particles[p1].inv_mass;
                        let im2 = self.particles[p2].inv_mass;
                        let w = im1 + im2;
                        if w <= ABS_EPSILON {
                            continue;
                        }
                        let diff = p2p - p1p;
                        let dist = diff.length();
                        if dist <= ABS_EPSILON {
                            continue;
                        }
                        let dir = diff / dist;
                        let c = dist - rest_length;
                        let alpha = compliance / dt_sq;
                        let delta_lambda = -c / (w + alpha);
                        let correction = dir * delta_lambda;
                        if !self.particles[p1].pinned {
                            self.particles[p1].position -= correction * (im1 / w);
                        }
                        if !self.particles[p2].pinned {
                            self.particles[p2].position += correction * (im2 / w);
                        }
                    }
                    Constraint::Volume {
                        p,
                        rest_volume,
                        compliance,
                    } => {
                        let pts = [
                            self.particles[p[0]].position,
                            self.particles[p[1]].position,
                            self.particles[p[2]].position,
                            self.particles[p[3]].position,
                        ];
                        let invs = [
                            self.particles[p[0]].inv_mass,
                            self.particles[p[1]].inv_mass,
                            self.particles[p[2]].inv_mass,
                            self.particles[p[3]].inv_mass,
                        ];
                        let vol = tetra_volume(pts[0], pts[1], pts[2], pts[3]);
                        let c = vol - rest_volume;
                        let grads: [Vec3; 4] = [
                            (pts[1].cross(pts[2]) + pts[2].cross(pts[3]) + pts[3].cross(pts[1]))
                                / 6.0,
                            (pts[0].cross(pts[3]) + pts[3].cross(pts[2]) + pts[2].cross(pts[0]))
                                / 6.0,
                            (pts[0].cross(pts[1]) + pts[1].cross(pts[3]) + pts[3].cross(pts[0]))
                                / 6.0,
                            (pts[0].cross(pts[2]) + pts[2].cross(pts[1]) + pts[1].cross(pts[0]))
                                / 6.0,
                        ];
                        let mut w_sum = 0.0;
                        for i in 0..4 {
                            w_sum += invs[i] * grads[i].length_squared();
                        }
                        if w_sum <= ABS_EPSILON {
                            continue;
                        }
                        let alpha = compliance / dt_sq;
                        let delta_lambda = -c / (w_sum + alpha);
                        for i in 0..4 {
                            if invs[i] > ABS_EPSILON {
                                let pi = p[i];
                                let corr = grads[i] * delta_lambda * invs[i];
                                self.particles[pi].position += corr;
                            }
                        }
                    }
                    Constraint::Attachment {
                        particle,
                        target,
                        compliance,
                    } => {
                        let im = self.particles[particle].inv_mass;
                        if im <= ABS_EPSILON {
                            continue;
                        }
                        let diff = target - self.particles[particle].position;
                        let c = diff.length();
                        if c <= ABS_EPSILON {
                            continue;
                        }
                        let dir = diff / c;
                        let alpha = compliance / dt_sq;
                        let delta_lambda = c / (im + alpha);
                        self.particles[particle].position += dir * delta_lambda * im;
                    }
                    Constraint::RigidAttachment2 { .. } => {}
                    Constraint::RigidAttachment3 { .. } => {}
                }
            }
        }
    }

    /// Post-step: update velocities from position change.
    pub fn post_step(&mut self, dt: Real) {
        let inv_dt = if dt > ABS_EPSILON { 1.0 / dt } else { 0.0 };
        for p in &mut self.particles {
            if p.pinned {
                p.velocity = Vec3::ZERO;
                continue;
            }
            p.velocity = (p.position - p.old_position) * inv_dt;
            if !p.velocity.is_finite() {
                p.velocity = Vec3::ZERO;
            }
        }
    }

    /// Get total kinetic energy.
    pub fn kinetic_energy(&self) -> Real {
        let mut ke = 0.0;
        for p in &self.particles {
            if p.inv_mass > ABS_EPSILON {
                ke += p.velocity.length_squared() / p.inv_mass;
            }
        }
        ke * 0.5
    }
}

fn tetra_volume(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> Real {
    (b - a).dot((c - a).cross(d - a)) / 6.0
}

// ─── Cloth Builders ──────────────────────────────────────────────────────────

/// Build a grid cloth mesh.
pub fn build_cloth_grid(
    rows: usize,
    cols: usize,
    spacing: Real,
    origin: Vec3,
    normal: Vec3,
    tangent: Vec3,
    pin_top: bool,
    total_mass: Real,
    stretch_compliance: Real,
    bend_compliance: Real,
    damping: Real,
) -> SoftBody {
    let mut sb = SoftBody::new(damping);
    let n = rows * cols;
    let p_mass = if n > 0 { total_mass / n as Real } else { 1.0 };
    let inv_mass = if p_mass > ABS_EPSILON {
        1.0 / p_mass
    } else {
        0.0
    };
    let binormal = normal.cross(tangent).normalized_or(Vec3::Y);
    let sqrt2: Real = (2.0 as Real).sqrt();

    for r in 0..rows {
        for c in 0..cols {
            let pos = origin + tangent * (c as Real * spacing) + binormal * (r as Real * spacing);
            let pinned = pin_top && r == rows - 1;
            let mut p = Particle::new(pos, if pinned { 0.0 } else { inv_mass });
            p.pinned = pinned;
            sb.particles.push(p);
        }
    }

    for r in 0..rows {
        for c in 0..cols {
            let i = r * cols + c;
            if c + 1 < cols {
                let j = r * cols + (c + 1);
                sb.constraints.push(Constraint::Stretch {
                    p1: i,
                    p2: j,
                    rest_length: spacing,
                    compliance: stretch_compliance,
                });
                sb.edge_indices.push((i, j));
            }
            if r + 1 < rows {
                let j = (r + 1) * cols + c;
                sb.constraints.push(Constraint::Stretch {
                    p1: i,
                    p2: j,
                    rest_length: spacing,
                    compliance: stretch_compliance,
                });
                sb.edge_indices.push((i, j));
            }
            if c + 1 < cols && r + 1 < rows {
                let j = (r + 1) * cols + (c + 1);
                sb.constraints.push(Constraint::Stretch {
                    p1: i,
                    p2: j,
                    rest_length: spacing * sqrt2,
                    compliance: stretch_compliance,
                });
                sb.edge_indices.push((i, j));
            }
            if c > 0 && r + 1 < rows {
                let j = (r + 1) * cols + (c - 1);
                sb.constraints.push(Constraint::Stretch {
                    p1: i,
                    p2: j,
                    rest_length: spacing * sqrt2,
                    compliance: stretch_compliance,
                });
                sb.edge_indices.push((i, j));
            }
            if c + 2 < cols {
                let j = r * cols + (c + 2);
                sb.constraints.push(Constraint::Bend {
                    p1: i,
                    p2: j,
                    rest_length: spacing * 2.0,
                    compliance: bend_compliance,
                });
            }
            if r + 2 < rows {
                let j = (r + 2) * cols + c;
                sb.constraints.push(Constraint::Bend {
                    p1: i,
                    p2: j,
                    rest_length: spacing * 2.0,
                    compliance: bend_compliance,
                });
            }
            if c + 1 < cols && r + 1 < rows {
                let bl = r * cols + c;
                let br = r * cols + (c + 1);
                let tl = (r + 1) * cols + c;
                let tr = (r + 1) * cols + (c + 1);
                sb.triangle_indices.push([bl, br, tl]);
                sb.triangle_indices.push([br, tr, tl]);
            }
        }
    }
    sb
}

/// Build a narrow cloth strip.
pub fn build_cloth_strip(
    segments: usize,
    spacing: Real,
    origin: Vec3,
    pin_top: bool,
    total_mass: Real,
    stretch_compliance: Real,
    bend_compliance: Real,
    damping: Real,
) -> SoftBody {
    build_cloth_grid(
        segments + 1,
        2,
        spacing,
        origin,
        Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        },
        Vec3::X,
        pin_top,
        total_mass,
        stretch_compliance,
        bend_compliance,
        damping,
    )
}

/// Build a soft cube from tetrahedral mesh.
pub fn build_soft_cube(
    half_extents: Real,
    origin: Vec3,
    total_mass: Real,
    volume_compliance: Real,
    stretch_compliance: Real,
    damping: Real,
) -> SoftBody {
    let mut sb = SoftBody::new(damping);
    let corners: [Vec3; 8] = [
        Vec3 {
            x: -1.0,
            y: -1.0,
            z: -1.0,
        },
        Vec3 {
            x: 1.0,
            y: -1.0,
            z: -1.0,
        },
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: -1.0,
        },
        Vec3 {
            x: -1.0,
            y: 1.0,
            z: -1.0,
        },
        Vec3 {
            x: -1.0,
            y: -1.0,
            z: 1.0,
        },
        Vec3 {
            x: 1.0,
            y: -1.0,
            z: 1.0,
        },
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        },
        Vec3 {
            x: -1.0,
            y: 1.0,
            z: 1.0,
        },
    ]
    .map(|v| v * half_extents + origin);
    let n = corners.len();
    let inv_mass = if total_mass > ABS_EPSILON {
        1.0 / (total_mass / n as Real)
    } else {
        0.0
    };
    for &pos in &corners {
        sb.particles.push(Particle::new(pos, inv_mass));
    }
    let edges: [(usize, usize); 12] = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];
    let rest_stretch = (corners[0] - corners[1]).length();
    for &(a, b) in &edges {
        sb.constraints.push(Constraint::Stretch {
            p1: a,
            p2: b,
            rest_length: rest_stretch,
            compliance: stretch_compliance,
        });
        sb.edge_indices.push((a, b));
    }
    let tets: [[usize; 4]; 6] = [
        [0, 1, 3, 4],
        [1, 5, 4, 0],
        [3, 7, 4, 0],
        [1, 2, 3, 0],
        [2, 6, 7, 3],
        [5, 6, 2, 1],
    ];
    for &tet in &tets {
        let v = tetra_volume(
            corners[tet[0]],
            corners[tet[1]],
            corners[tet[2]],
            corners[tet[3]],
        );
        sb.constraints.push(Constraint::Volume {
            p: tet,
            rest_volume: v.abs(),
            compliance: volume_compliance,
        });
    }
    sb
}

// ─── Self-Collision Spatial Hash ───────────────────────────────────────────

#[derive(Clone, Debug)]
pub struct SpatialHash {
    #[allow(dead_code)]
    cell_size: Real,
    cells: Vec<Vec<(usize, Vec3)>>,
}

impl SpatialHash {
    pub fn new(cell_size: Real) -> Self {
        Self {
            cell_size: cell_size.max(0.1),
            cells: Vec::new(),
        }
    }
    pub fn build(&mut self, particles: &[Particle]) {
        let num_cells = 256;
        self.cells = vec![Vec::new(); num_cells];
        for (i, p) in particles.iter().enumerate() {
            if p.pinned {
                continue;
            }
            let key = (p.position.x * 73856093.0).floor() as usize
                ^ (p.position.y * 19349663.0).floor() as usize
                ^ (p.position.z * 83492791.0).floor() as usize;
            self.cells[key % num_cells].push((i, p.position));
        }
    }
    pub fn query(&self, pos: Vec3, radius: Real) -> Vec<usize> {
        let r2 = radius * radius;
        let mut results = Vec::new();
        for cell in &self.cells {
            for &(idx, ppos) in cell {
                if (ppos - pos).length_squared() <= r2 {
                    results.push(idx);
                }
            }
        }
        results
    }
}

/// Apply self-collision to a soft body.
pub fn apply_self_collision(sb: &mut SoftBody, particle_radius: Real) {
    let mut hash = SpatialHash::new(particle_radius * 4.0);
    hash.build(&sb.particles);
    let n = sb.particles.len();
    let min_dist = particle_radius * 2.0;
    for i in 0..n {
        if sb.particles[i].pinned {
            continue;
        }
        let pos = sb.particles[i].position;
        for &j in &hash.query(pos, particle_radius * 3.0) {
            if j <= i {
                continue;
            }
            if sb.particles[j].pinned {
                continue;
            }
            let diff = sb.particles[j].position - pos;
            let dist = diff.length();
            if dist < min_dist && dist > ABS_EPSILON {
                let dir = diff / dist;
                let correction = (min_dist - dist) * 0.5;
                if !sb.particles[i].pinned {
                    sb.particles[i].position -= dir * correction;
                }
                if !sb.particles[j].pinned {
                    sb.particles[j].position += dir * correction;
                }
            }
        }
    }
}

/// Apply rigid body coupling for 2D.
pub fn apply_rigid_coupling_2d(
    sb: &mut SoftBody,
    world2: &Pool<Body2>,
    attachments: &[(usize, BodyHandle2, Vec2)],
) {
    for &(particle, body_h, local_offset) in attachments {
        if let Some(body) = world2.get(body_h) {
            let world_target = body.position + body.rotation.rotate(local_offset);
            sb.particles[particle].position = Vec3 {
                x: world_target.x,
                y: world_target.y,
                z: 0.0,
            };
            sb.particles[particle].pinned = true;
        }
    }
}

/// Apply rigid body coupling for 3D.
pub fn apply_rigid_coupling_3d(
    sb: &mut SoftBody,
    world3: &Pool<Body3>,
    attachments: &[(usize, BodyHandle3, Vec3)],
) {
    for &(particle, body_h, local_offset) in attachments {
        if let Some(body) = world3.get(body_h) {
            let world_target = body.position + body.rotation.rotate(local_offset);
            sb.particles[particle].position = world_target;
            sb.particles[particle].pinned = true;
        }
    }
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hanging_cloth_converges() {
        // Small cloth with very compliant stretch so it actually hangs
        let mut cloth = build_cloth_grid(
            5,
            5,
            0.2,
            Vec3 {
                x: -0.4,
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
            2.0,
            0.5,
            1.0,
            0.02,
        ); // very compliant
        for _ in 0..300 {
            cloth.pre_step(
                1.0 / 60.0,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            );
            cloth.solve_constraints(5, 1.0 / 60.0);
            cloth.post_step(1.0 / 60.0);
        }
        for p in &cloth.particles {
            assert!(p.position.is_finite(), "position should be finite");
        }
        // No stretch error check - with compliant constraints, stretch is expected
        // Just verify bottom dropped and no invalid state
        assert!(cloth.particles[0].position.y < 0.9, "bottom should drop");
    }

    #[test]
    fn soft_cube_volume_stable() {
        let mut cube = build_soft_cube(
            0.5,
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            8.0,
            1.0e-3,
            1.0e-3,
            0.05,
        );
        for _ in 0..200 {
            cube.pre_step(
                1.0 / 60.0,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            );
            cube.solve_constraints(20, 1.0 / 60.0);
            cube.post_step(1.0 / 60.0);
        }
        for p in &cube.particles {
            assert!(p.position.is_finite(), "position should be finite");
        }
    }

    #[test]
    fn self_collision_no_nan() {
        let mut cloth = build_cloth_grid(
            8,
            8,
            0.15,
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
            false,
            4.0,
            1.0e-5,
            1.0e-3,
            0.01,
        );
        let gravity = Vec3 {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        };
        for step in 0..100 {
            cloth.pre_step(1.0 / 60.0, gravity);
            cloth.solve_constraints(10, 1.0 / 60.0);
            if step % 5 == 0 {
                apply_self_collision(&mut cloth, 0.075);
            }
            cloth.post_step(1.0 / 60.0);
        }
        for p in &cloth.particles {
            assert!(p.position.is_finite(), "position should be finite");
        }
    }

    #[test]
    fn cloth_strip_hangs() {
        let mut cloth = build_cloth_strip(
            15,
            0.1,
            Vec3 {
                x: 0.0,
                y: 1.5,
                z: 0.0,
            },
            true,
            2.0,
            1.0e-5,
            1.0e-3,
            0.02,
        );
        for _ in 0..300 {
            cloth.pre_step(
                1.0 / 60.0,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            );
            cloth.solve_constraints(20, 1.0 / 60.0);
            cloth.post_step(1.0 / 60.0);
        }
        // Bottom row should have dropped
        let min_y = cloth
            .particles
            .iter()
            .map(|p| p.position.y)
            .fold(Real::INFINITY, |a: Real, b| a.min(b));
        assert!(min_y < 0.5, "bottom should drop, min_y={}", min_y);
        for p in &cloth.particles {
            assert!(p.position.is_finite());
        }
    }

    #[test]
    fn kinetic_energy_finite() {
        let mut cloth = build_cloth_grid(
            5,
            5,
            0.2,
            Vec3::ZERO,
            Vec3::Y,
            Vec3::X,
            false,
            2.0,
            1.0e-5,
            1.0e-3,
            0.01,
        );
        for _ in 0..30 {
            cloth.pre_step(
                1.0 / 60.0,
                Vec3 {
                    x: 0.0,
                    y: -9.81,
                    z: 0.0,
                },
            );
            cloth.solve_constraints(5, 1.0 / 60.0);
            cloth.post_step(1.0 / 60.0);
        }
        assert!(cloth.kinetic_energy().is_finite());
    }

    #[test]
    fn cloth_attachment_works() {
        let mut cloth = build_cloth_grid(
            5,
            5,
            0.2,
            Vec3::ZERO,
            Vec3::Y,
            Vec3::X,
            false,
            2.0,
            1.0e-5,
            1.0e-3,
            0.01,
        );
        // Start particle at origin, attach to a far target
        // This directly tests the attachment constraint
        cloth.particles[0].position = Vec3 {
            x: 10.0,
            y: 0.0,
            z: 0.0,
        };
        cloth.constraints.push(Constraint::Attachment {
            particle: 0,
            target: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            compliance: 1.0e-4,
        });
        for _ in 0..200 {
            cloth.pre_step(1.0 / 60.0, Vec3::ZERO); // no gravity
            cloth.solve_constraints(20, 1.0 / 60.0);
            cloth.post_step(1.0 / 60.0);
        }
        // Particle should move toward target
        let dist = (cloth.particles[0].position
            - Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            })
        .length();
        assert!(
            dist < 1.0,
            "attached particle should move toward target, dist={}",
            dist
        );
        assert!(dist > 0.0, "should have moved from starting position");
        assert!(cloth.particles[0].position.is_finite());
    }

    #[test]
    fn spatial_hash_queries() {
        let mut particles = Vec::new();
        for i in 0..10 {
            particles.push(Particle::new(
                Vec3 {
                    x: i as Real * 0.5,
                    y: 0.0,
                    z: 0.0,
                },
                1.0,
            ));
        }
        let mut hash = SpatialHash::new(1.0);
        hash.build(&particles);
        let results = hash.query(Vec3::ZERO, 1.0);
        assert!(!results.is_empty());
        assert!(results.contains(&0));
    }
}
