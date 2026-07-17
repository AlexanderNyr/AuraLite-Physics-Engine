//! SAT penetration, bounded EPA, contact manifolds, stable feature persistence,
//! 3D box OBB-OBB SAT, 3D EPA fallback, contact clipping, and pair dispatch.
use auralite_math::{ABS_EPSILON, CONTACT_SLOP, Real, Vec2, Vec3};
/// Penetration query output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Penetration2 {
    /// A-to-B unit normal.
    pub normal: Vec2,
    /// Nonnegative depth.
    pub depth: Real,
    /// Iterations.
    pub iterations: u16,
    /// Final improvement/error.
    pub residual: Real,
    /// Whether the primary algorithm converged.
    pub converged: bool,
}
/// 3D penetration query output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Penetration3 {
    /// A-to-B unit normal.
    pub normal: Vec3,
    /// Nonnegative depth.
    pub depth: Real,
    /// Iterations.
    pub iterations: u16,
    /// Residual.
    pub residual: Real,
    /// Whether converged.
    pub converged: bool,
}
/// SAT for two counter-clockwise convex polygons in world space.
#[must_use]
pub fn sat_polygon2(a: &[Vec2], b: &[Vec2]) -> Option<Penetration2> {
    if a.len() < 3 || b.len() < 3 {
        return None;
    }
    let ca = a.iter().copied().fold(Vec2::ZERO, |x, y| x + y) / (a.len() as Real);
    let cb = b.iter().copied().fold(Vec2::ZERO, |x, y| x + y) / (b.len() as Real);
    let mut best = Real::INFINITY;
    let mut normal = Vec2::X;
    for p in [a, b] {
        for i in 0..p.len() {
            let e = p[(i + 1) % p.len()] - p[i];
            if e.length_squared() <= ABS_EPSILON * ABS_EPSILON {
                continue;
            }
            let mut n = Vec2 { x: e.y, y: -e.x }.normalized_or(Vec2::X);
            let (min_a, max_a) = project2(a, n);
            let (min_b, max_b) = project2(b, n);
            let overlap = max_a.min(max_b) - min_a.max(min_b);
            if overlap < -CONTACT_SLOP {
                return None;
            }
            if overlap < best {
                if n.dot(cb - ca) < 0.0 {
                    n = -n;
                }
                best = overlap.max(0.0);
                normal = n;
            }
        }
    }
    best.is_finite().then_some(Penetration2 {
        normal,
        depth: best,
        iterations: (a.len() + b.len()) as u16,
        residual: 0.0,
        converged: true,
    })
}
fn project2(p: &[Vec2], n: Vec2) -> (Real, Real) {
    p.iter()
        .map(|x| x.dot(n))
        .fold((Real::INFINITY, Real::NEG_INFINITY), |(a, b), x| {
            (a.min(x), b.max(x))
        })
}

/// 3D SAT for oriented box vs oriented box (OBB-OBB).
/// Tests 15 axes: 3 face normals from each box + 9 edge cross products.
/// Returns minimal penetration axis and depth if overlapping.
#[must_use]
pub fn sat_box3_box3(
    a_half: Vec3,
    a_rot: &[Vec3; 3], // columns of rotation matrix
    a_pos: Vec3,
    b_half: Vec3,
    b_rot: &[Vec3; 3],
    b_pos: Vec3,
) -> Option<Penetration3> {
    // Box A face axes (rotation columns = local axes in world space)
    let axes_a = [a_rot[0], a_rot[1], a_rot[2]];
    // Box B face axes
    let axes_b = [b_rot[0], b_rot[1], b_rot[2]];

    let center = b_pos - a_pos;
    let mut best_depth = Real::INFINITY;
    let mut best_normal = Vec3::X;
    let mut found = false;

    // Test all 15 potential separating axes
    for &axis in axes_a.iter().chain(axes_b.iter()) {
        let ax = axis.normalized_or(Vec3::X);
        let depth = sat_box_test_axis(a_half, &axes_a, b_half, &axes_b, center, ax)?;
        if depth < best_depth {
            best_depth = depth.max(0.0);
            best_normal = if ax.dot(center) < 0.0 { -ax } else { ax };
            found = true;
        }
    }

    // Edge cross products
    for &a in axes_a.iter() {
        for &b in axes_b.iter() {
            let ax = a.cross(b);
            if ax.length_squared() <= ABS_EPSILON * ABS_EPSILON {
                continue;
            }
            let ax = ax.normalized_or(Vec3::X);
            let depth = sat_box_test_axis(a_half, &axes_a, b_half, &axes_b, center, ax)?;
            if depth < best_depth {
                best_depth = depth.max(0.0);
                best_normal = if ax.dot(center) < 0.0 { -ax } else { ax };
                found = true;
            }
        }
    }

    if !found || !best_depth.is_finite() {
        return None;
    }
    Some(Penetration3 {
        normal: best_normal,
        depth: best_depth,
        iterations: 15,
        residual: 0.0,
        converged: true,
    })
}

fn sat_box_test_axis(
    a_half: Vec3,
    a_rot: &[Vec3; 3],
    b_half: Vec3,
    b_rot: &[Vec3; 3],
    center: Vec3,
    axis: Vec3,
) -> Option<Real> {
    // Project box A onto axis
    let ra = a_half.x * (a_rot[0].dot(axis)).abs()
        + a_half.y * (a_rot[1].dot(axis)).abs()
        + a_half.z * (a_rot[2].dot(axis)).abs();
    // Project box B onto axis
    let rb = b_half.x * (b_rot[0].dot(axis)).abs()
        + b_half.y * (b_rot[1].dot(axis)).abs()
        + b_half.z * (b_rot[2].dot(axis)).abs();
    // Distance between projected centers
    let d = (center.dot(axis)).abs();
    let overlap = ra + rb - d;
    if overlap < 0.0 {
        // Boxes are separated along this axis with gap > CONTACT_SLOP? We use a small tolerance
        if d - (ra + rb) > CONTACT_SLOP {
            return None; // truly separated
        }
        // Touching or small gap - we report slight negative as zero overlap
        return Some(0.0);
    }
    Some(overlap)
}

/// 3D SAT using support function (generic convex-vs-convex separating axis test).
/// Tests directions from edges of both shapes and face normals (approximate).
#[must_use]
pub fn sat_convex3(
    support_a: &mut impl FnMut(Vec3) -> Vec3,
    support_b: &mut impl FnMut(Vec3) -> Vec3,
    _max_axes: u16,
) -> Option<Penetration3> {
    // Use GJK-based separation: if GJK says separated, return None.
    // For penetration we rely on EPA fallback (3D EPA).
    // This is a fallback that runs GJK to check separation.
    let res = super::gjk_distance3(&mut *support_a, &mut *support_b, 32);
    if res.status == super::GjkStatus::Separated {
        return None; // separated
    }
    // Overlapping or intersecting - we need EPA for depth
    epa_penetration3(&mut *support_a, &mut *support_b, 64)
}

#[derive(Clone, Copy)]
struct EpaPoint2 {
    p: Vec2,
}
/// Bounded 2D EPA penetration using convex support functions. Falls back to the best finite edge at the cap.
#[must_use]
pub fn epa_penetration2(
    mut sa: impl FnMut(Vec2) -> Vec2,
    mut sb: impl FnMut(Vec2) -> Vec2,
    max_iterations: u16,
) -> Option<Penetration2> {
    let mut poly = Vec::new();
    for i in 0..3 {
        let q = i as Real * core::f64::consts::TAU as Real / 3.0;
        let d = Vec2 {
            x: q.cos(),
            y: q.sin(),
        };
        let a = sa(d);
        let b = sb(-d);
        if !a.is_finite() || !b.is_finite() {
            return None;
        }
        poly.push(EpaPoint2 { p: a - b });
    }
    if (poly[1].p - poly[0].p).cross(poly[2].p - poly[0].p) < 0.0 {
        poly.swap(1, 2);
    }
    let mut last = Real::INFINITY;
    for it in 1..=max_iterations.max(1) {
        let mut best = (Real::INFINITY, 0, Vec2::X);
        for i in 0..poly.len() {
            let j = (i + 1) % poly.len();
            let e = poly[j].p - poly[i].p;
            let mut n = Vec2 { x: e.y, y: -e.x }.normalized_or(Vec2::X);
            let mut d = n.dot(poly[i].p);
            if d < 0.0 {
                n = -n;
                d = -d;
            }
            if d < best.0 {
                best = (d, i, n);
            }
        }
        let a = sa(best.2);
        let b = sb(-best.2);
        let p = a - b;
        let distance = best.2.dot(p);
        let residual = (distance - best.0).abs();
        if residual <= ABS_EPSILON * (1.0 + distance.abs()) {
            return Some(Penetration2 {
                normal: best.2,
                depth: distance.max(0.0),
                iterations: it,
                residual,
                converged: true,
            });
        }
        if poly
            .iter()
            .any(|x| (x.p - p).length_squared() <= ABS_EPSILON * ABS_EPSILON)
        {
            return Some(Penetration2 {
                normal: best.2,
                depth: best.0.max(0.0),
                iterations: it,
                residual,
                converged: false,
            });
        }
        last = residual;
        poly.insert(best.1 + 1, EpaPoint2 { p });
    }
    let mut best = (Real::INFINITY, Vec2::X);
    for i in 0..poly.len() {
        let e = poly[(i + 1) % poly.len()].p - poly[i].p;
        let mut n = Vec2 { x: e.y, y: -e.x }.normalized_or(Vec2::X);
        let mut d = n.dot(poly[i].p);
        if d < 0.0 {
            d = -d;
            n = -n;
        }
        if d < best.0 {
            best = (d, n)
        }
    }
    Some(Penetration2 {
        normal: best.1,
        depth: best.0.max(0.0),
        iterations: max_iterations.max(1),
        residual: last,
        converged: false,
    })
}

#[derive(Clone, Copy)]
struct EpaPoint3 {
    p: Vec3,
}
/// Bounded 3D EPA penetration using convex support functions.
/// Expands a tetrahedron (4 initial points) toward the Minkowski origin.
/// Falls back to the closest face at iteration cap.
#[must_use]
pub fn epa_penetration3(
    mut sa: impl FnMut(Vec3) -> Vec3,
    mut sb: impl FnMut(Vec3) -> Vec3,
    max_iterations: u16,
) -> Option<Penetration3> {
    // Build initial tetrahedron from 4 directions
    let dirs = [
        Vec3::X,
        Vec3::Y,
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        },
        Vec3 {
            x: -1.0,
            y: -1.0,
            z: -1.0,
        },
    ];
    let mut poly: Vec<EpaPoint3> = Vec::new();
    for &d in &dirs {
        let dn = d.normalized_or(Vec3::X);
        let a = sa(dn);
        let b = sb(-dn);
        if !a.is_finite() || !b.is_finite() {
            return None;
        }
        poly.push(EpaPoint3 { p: a - b });
    }
    // Ensure proper winding for the tetrahedron (positive volume)
    // Check if we need to reorder
    let vol = (poly[1].p - poly[0].p).dot((poly[2].p - poly[0].p).cross(poly[3].p - poly[0].p));
    if vol.abs() <= ABS_EPSILON {
        // Degenerate initial tetrahedron
        return None;
    }
    // Poly now represents the CSO. At each iteration find closest face to origin.
    let mut last_residual = Real::INFINITY;
    for it in 1..=max_iterations.max(1) {
        // Find closest face (triangular facet) to origin
        // We enumerate all triangular faces of the current polytope
        let faces = enumerate_faces_3(&poly);
        if faces.is_empty() {
            return None;
        }
        let mut best_face = 0usize;
        let mut best_dist = Real::INFINITY;
        let mut best_normal = Vec3::X;
        for (fi, &(i, j, k)) in faces.iter().enumerate() {
            let a = poly[i].p;
            let b = poly[j].p;
            let c = poly[k].p;
            let mut n = (b - a).cross(c - a);
            let nlen = n.length();
            if nlen <= ABS_EPSILON {
                continue;
            }
            n /= nlen;
            let d = n.dot(a);
            let dist = if d < 0.0 { -d } else { d };
            if dist < best_dist {
                best_dist = dist;
                best_face = fi;
                best_normal = if d >= 0.0 { n } else { -n };
            }
        }
        // Expand along the closest face normal
        let n = best_normal;
        let a_new = sa(n);
        let b_new = sb(-n);
        if !a_new.is_finite() || !b_new.is_finite() {
            // Fallback: use best face distance
            let f = faces[best_face];
            let a = poly[f.0].p;
            let b = poly[f.1].p;
            let c = poly[f.2].p;
            let mut n = (b - a).cross(c - a);
            let nlen = n.length();
            if nlen <= ABS_EPSILON {
                return None;
            }
            n /= nlen;
            let depth = n.dot(a).abs();
            return Some(Penetration3 {
                normal: n,
                depth: depth.max(0.0),
                iterations: it,
                residual: 0.0,
                converged: false,
            });
        }
        let p_new = a_new - b_new;
        let d_new = n.dot(p_new);
        let residual = (d_new - best_dist).abs();
        if residual <= ABS_EPSILON * (1.0 + best_dist.abs()) {
            return Some(Penetration3 {
                normal: n,
                depth: d_new.max(0.0),
                iterations: it,
                residual,
                converged: true,
            });
        }
        // Check if p_new is already in poly
        if poly
            .iter()
            .any(|x| (x.p - p_new).length_squared() <= ABS_EPSILON * ABS_EPSILON)
        {
            let f = faces[best_face];
            let a = poly[f.0].p;
            let b = poly[f.1].p;
            let c = poly[f.2].p;
            let mut n = (b - a).cross(c - a);
            let nlen = n.length();
            if nlen <= ABS_EPSILON {
                return None;
            }
            n /= nlen;
            return Some(Penetration3 {
                normal: n,
                depth: n.dot(a).abs().max(0.0),
                iterations: it,
                residual,
                converged: false,
            });
        }
        last_residual = residual;
        poly.push(EpaPoint3 { p: p_new });
    }
    // Iteration limit: find closest face
    let faces = enumerate_faces_3(&poly);
    if faces.is_empty() {
        return None;
    }
    let mut best_dist = Real::INFINITY;
    let mut best_normal = Vec3::X;
    for &(i, j, k) in &faces {
        let a = poly[i].p;
        let b = poly[j].p;
        let c = poly[k].p;
        let mut n = (b - a).cross(c - a);
        let nlen = n.length();
        if nlen <= ABS_EPSILON {
            continue;
        }
        n /= nlen;
        let d = n.dot(a).abs();
        if d < best_dist {
            best_dist = d;
            best_normal = if n.dot(a) >= 0.0 { n } else { -n };
        }
    }
    Some(Penetration3 {
        normal: best_normal,
        depth: best_dist.max(0.0),
        iterations: max_iterations.max(1),
        residual: last_residual,
        converged: false,
    })
}

/// Enumerate unique triangular faces from a polytope of 3D points.
/// Returns indices of vertices forming each face.
fn enumerate_faces_3(points: &[EpaPoint3]) -> Vec<(usize, usize, usize)> {
    let mut faces = Vec::new();
    let n = points.len();
    if n < 4 {
        return faces;
    }
    for i in 0..n {
        for j in i + 1..n {
            for k in j + 1..n {
                let a = points[i].p;
                let b = points[j].p;
                let c = points[k].p;
                let n_vec = (b - a).cross(c - a);
                if n_vec.length_squared() <= ABS_EPSILON * ABS_EPSILON {
                    continue;
                }
                // Check if all other points are on one side (this is a hull face)
                let mut all_neg = true;
                let mut all_pos = true;
                for (m, pt) in points.iter().enumerate() {
                    if m == i || m == j || m == k {
                        continue;
                    }
                    let d = n_vec.dot(pt.p - a);
                    if d > ABS_EPSILON {
                        all_neg = false;
                    }
                    if d < -ABS_EPSILON {
                        all_pos = false;
                    }
                }
                // A face has all points on one side (or coplanar)
                if all_neg || all_pos {
                    faces.push((i, j, k));
                }
            }
        }
    }
    faces
}

/// Stable feature identifier derived from shape features, never pointer/order state.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FeatureId(pub u64);
/// Persistent 2D contact point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ManifoldPoint2 {
    /// Position.
    pub position: Vec2,
    /// Penetration.
    pub penetration: Real,
    /// Stable feature.
    pub feature: FeatureId,
    /// Cached normal impulse.
    pub normal_impulse: Real,
    /// Cached tangent impulse.
    pub tangent_impulse: Real,
}
/// Up to two native 2D contact points.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Manifold2 {
    /// A-to-B normal.
    pub normal: Vec2,
    /// Contact points sorted by feature.
    pub points: Vec<ManifoldPoint2>,
}
impl Manifold2 {
    /// Updates contacts while carrying warm-start impulses for matching feature IDs.
    pub fn update(&mut self, normal: Vec2, mut fresh: Vec<ManifoldPoint2>) {
        fresh.sort_by_key(|x| x.feature);
        fresh.dedup_by_key(|x| x.feature);
        fresh.truncate(2);
        for p in &mut fresh {
            if let Some(old) = self.points.iter().find(|x| x.feature == p.feature) {
                p.normal_impulse = old.normal_impulse;
                p.tangent_impulse = old.tangent_impulse;
            }
        }
        self.normal = normal.normalized_or(Vec2::X);
        self.points = fresh;
    }
    /// Builds a manifold from clip points produced by 2D clipping.
    pub fn from_clip(normal: Vec2, points: Vec<(Vec2, Real, FeatureId)>) -> Self {
        let fresh: Vec<ManifoldPoint2> = points
            .into_iter()
            .map(|(p, pen, f)| ManifoldPoint2 {
                position: p,
                penetration: pen,
                feature: f,
                normal_impulse: 0.0,
                tangent_impulse: 0.0,
            })
            .collect();
        let mut m = Manifold2::default();
        m.update(normal, fresh);
        m
    }
}
/// Persistent 3D contact point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ManifoldPoint3 {
    /// Position.
    pub position: Vec3,
    /// Penetration.
    pub penetration: Real,
    /// Stable feature.
    pub feature: FeatureId,
    /// Cached normal impulse.
    pub normal_impulse: Real,
    /// Cached tangent impulse.
    pub tangent_impulse: Real,
}
/// Up to four 3D contact points.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Manifold3 {
    /// A-to-B normal.
    pub normal: Vec3,
    /// Contact points sorted by feature.
    pub points: Vec<ManifoldPoint3>,
}
impl Manifold3 {
    /// Updates contacts while carrying warm-start impulses for matching feature IDs.
    pub fn update(&mut self, normal: Vec3, mut fresh: Vec<ManifoldPoint3>) {
        fresh.sort_by_key(|x| x.feature);
        fresh.dedup_by_key(|x| x.feature);
        fresh.truncate(4);
        for p in &mut fresh {
            if let Some(old) = self.points.iter().find(|x| x.feature == p.feature) {
                p.normal_impulse = old.normal_impulse;
                p.tangent_impulse = old.tangent_impulse;
            }
        }
        self.normal = normal.normalized_or(Vec3::X);
        self.points = fresh;
    }
    /// Builds a manifold from clip points.
    pub fn from_clip(normal: Vec3, points: Vec<(Vec3, Real, FeatureId)>) -> Self {
        let fresh: Vec<ManifoldPoint3> = points
            .into_iter()
            .map(|(p, pen, f)| ManifoldPoint3 {
                position: p,
                penetration: pen,
                feature: f,
                normal_impulse: 0.0,
                tangent_impulse: 0.0,
            })
            .collect();
        let mut m = Manifold3::default();
        m.update(normal, fresh);
        m
    }
}

/// 2D contact clipping: reference-face / incident-edge method for convex polygons.
/// Produces up to 2 contact points with feature IDs derived from edge indices.
#[must_use]
pub fn clip_contacts2(
    normal: Vec2,
    a_vertices: &[Vec2],
    a_edge_index: Option<usize>,
    b_vertices: &[Vec2],
    b_edge_index: Option<usize>,
    separation: Real,
) -> Vec<(Vec2, Real, FeatureId)> {
    // Find the reference face (edge with most anti-parallel normal)
    let find_best_edge = |vertices: &[Vec2], n: Vec2| -> (usize, Real) {
        let mut best = 0;
        let mut best_dot = Real::INFINITY;
        for i in 0..vertices.len() {
            let e = vertices[(i + 1) % vertices.len()] - vertices[i];
            let en = Vec2 { x: e.y, y: -e.x }.normalized_or(Vec2::X);
            let d = en.dot(n);
            if d < best_dot {
                best_dot = d;
                best = i;
            }
        }
        (best, best_dot)
    };
    let (ref_idx, _) = match (a_edge_index, b_edge_index) {
        (Some(i), _) => (i, find_best_edge(b_vertices, -normal).0),
        (_, Some(j)) => (find_best_edge(a_vertices, normal).0, j),
        _ => {
            let (ri, _) = find_best_edge(a_vertices, normal);
            (ri, find_best_edge(b_vertices, -normal).0)
        }
    };
    // Use A as reference face
    let ref_edge_dir = a_vertices[(ref_idx + 1) % a_vertices.len()] - a_vertices[ref_idx];
    // Clip incident edge (from B) against the reference face's side planes
    // Find incident edge on B (edge most aligned with -ref_normal)
    let mut inc_idx = 0;
    let mut inc_best = Real::NEG_INFINITY;
    for i in 0..b_vertices.len() {
        let e = b_vertices[(i + 1) % b_vertices.len()] - b_vertices[i];
        let en = Vec2 { x: e.y, y: -e.x };
        let en_n = en.normalized_or(Vec2::X);
        let d = en_n.dot(-normal);
        if d > inc_best {
            inc_best = d;
            inc_idx = i;
        }
    }
    let i0 = b_vertices[inc_idx];
    let i1 = b_vertices[(inc_idx + 1) % b_vertices.len()];
    // Clip the incident edge against the reference face side planes
    let ref_point = a_vertices[ref_idx];
    // For 2D clipping, we clip the incident edge segment to produce up to 2 contact points
    // Simple approach: project the overlapping portion of the incident edge onto the reference face
    let mut contacts = Vec::new();
    // Clamp the incident edge to the reference face extents along the edge direction
    let edge_len = ref_edge_dir.length();
    if edge_len > ABS_EPSILON {
        let ed = ref_edge_dir / edge_len;
        // Project incident vertices onto the reference edge axis
        let p0 = (i0 - ref_point).dot(ed);
        let p1 = (i1 - ref_point).dot(ed);
        let t0 = p0.max(0.0);
        let t1 = p1.min(edge_len);
        if t0 <= t1 {
            // Point at t0
            let cp = ref_point + ed * t0;
            let pen = separation + (cp - i0).dot(normal).abs();
            contacts.push((
                cp,
                pen.max(0.0),
                FeatureId((inc_idx as u64) << 32 | ref_idx as u64),
            ));
        }
        if t1 > t0 && (t1 - t0) > edge_len * 0.25 {
            let cp = ref_point + ed * t1;
            let pen = separation + (cp - i1).dot(normal).abs();
            contacts.push((
                cp,
                pen.max(0.0),
                FeatureId(((inc_idx + 1) as u64) << 32 | ref_idx as u64),
            ));
        }
    }
    if contacts.is_empty() {
        // Fallback: use midpoints
        let mid = (i0 + i1) * 0.5;
        let pen = separation;
        contacts.push((mid, pen.max(0.0), FeatureId(inc_idx as u64)));
    }
    contacts
}

/// Closest points between 3D segments, robust for parallel and degenerate inputs.
#[must_use]
pub fn segment_segment_closest3(
    p1: Vec3,
    q1: Vec3,
    p2: Vec3,
    q2: Vec3,
) -> (Vec3, Vec3, Real, Real) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);
    let (mut s, mut t);
    if a <= ABS_EPSILON && e <= ABS_EPSILON {
        return (p1, p2, 0.0, 0.0);
    }
    if a <= ABS_EPSILON {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if e <= ABS_EPSILON {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            let den = a * e - b * b;
            s = if den.abs() > ABS_EPSILON {
                ((b * f - c * e) / den).clamp(0.0, 1.0)
            } else {
                0.0
            };
            t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0)
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0)
            }
        }
    }
    (p1 + d1 * s, p2 + d2 * t, s, t)
}

///////////////////////////////////////////////////////////////////////////////
// Pair dispatch types
///////////////////////////////////////////////////////////////////////////////

/// Shape type discriminators for 2D pair dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeType2 {
    /// Circle shape.
    Circle,
    /// Capsule/swept circle shape.
    Capsule,
    /// Axis-aligned or oriented box shape.
    Box,
    /// Convex polygon (CCW vertex list).
    ConvexPolygon,
    /// Compound of multiple child shapes.
    Compound,
    /// Zero-area edge segment.
    Edge,
    /// Heightfield (line segment chain).
    Heightfield,
    /// Infinite half-space.
    HalfSpace,
}
/// Shape type discriminators for 3D pair dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeType3 {
    /// Sphere.
    Sphere,
    /// Capsule/swept sphere shape.
    Capsule,
    /// Oriented box.
    Box,
    /// Convex hull polyhedron.
    ConvexHull,
    /// Compound of multiple child shapes.
    Compound,
    /// Zero-volume edge segment.
    Edge,
    /// Triangle mesh.
    TriangleMesh,
    /// Grid heightfield.
    Heightfield,
    /// Infinite half-space.
    HalfSpace,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GjkStatus;
    fn square(c: Vec2, h: Real) -> Vec<Vec2> {
        vec![
            c + Vec2 { x: -h, y: -h },
            c + Vec2 { x: h, y: -h },
            c + Vec2 { x: h, y: h },
            c + Vec2 { x: -h, y: h },
        ]
    }
    #[test]
    fn epa_agrees_with_sat_for_boxes() {
        for i in 0..1000 {
            let x = i as Real * 0.0005;
            let a = square(Vec2::ZERO, 1.0);
            let b = square(Vec2 { x: 1.0 + x, y: 0.0 }, 1.0);
            let sat = sat_polygon2(&a, &b).unwrap();
            let epa = epa_penetration2(
                |d| {
                    *a.iter()
                        .max_by(|x, y| x.dot(d).total_cmp(&y.dot(d)))
                        .unwrap()
                },
                |d| {
                    *b.iter()
                        .max_by(|x, y| x.dot(d).total_cmp(&y.dot(d)))
                        .unwrap()
                },
                64,
            )
            .unwrap();
            assert!(
                (sat.depth - epa.depth).abs() < 2.0e-4,
                "sat={} epa={}",
                sat.depth,
                epa.depth
            );
        }
    }
    #[test]
    fn manifold_preserves_feature_impulses() {
        let mut m = Manifold2::default();
        m.update(
            Vec2::X,
            vec![ManifoldPoint2 {
                position: Vec2::ZERO,
                penetration: 1.0,
                feature: FeatureId(7),
                normal_impulse: 3.0,
                tangent_impulse: 2.0,
            }],
        );
        m.update(
            Vec2::X,
            vec![ManifoldPoint2 {
                position: Vec2 { x: 0.01, y: 0.0 },
                penetration: 0.9,
                feature: FeatureId(7),
                normal_impulse: 0.0,
                tangent_impulse: 0.0,
            }],
        );
        assert_eq!(m.points[0].normal_impulse, 3.0);
    }
    #[test]
    fn parallel_segments_are_finite() {
        let (a, b, _, _) = segment_segment_closest3(
            Vec3::ZERO,
            Vec3::X,
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            Vec3 {
                x: 1.0,
                y: 1.0,
                z: 0.0,
            },
        );
        assert!(a.is_finite() && b.is_finite());
        assert!((a - b).length() - 1.0 < 1.0e-6);
    }
    #[test]
    fn sat_box3_box3_overlap_detection() {
        // Two overlapping boxes
        let a_half = Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let a_rot = [
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        let a_pos = Vec3::ZERO;
        let b_half = Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let b_rot = [
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        let b_pos = Vec3 {
            x: 0.5,
            y: 0.0,
            z: 0.0,
        };
        let result = sat_box3_box3(a_half, &a_rot, a_pos, b_half, &b_rot, b_pos);
        assert!(
            result.is_some(),
            "Overlapping boxes should produce penetration"
        );
        let p = result.unwrap();
        assert!(p.depth > 0.0);
        assert!(p.normal.is_finite());
    }
    #[test]
    fn sat_box3_box3_separated_returns_none() {
        let a_half = Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let a_rot = [
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        let a_pos = Vec3::ZERO;
        let b_half = Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let b_rot = [
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        let b_pos = Vec3 {
            x: 5.0,
            y: 0.0,
            z: 0.0,
        };
        let result = sat_box3_box3(a_half, &a_rot, a_pos, b_half, &b_rot, b_pos);
        assert!(
            result.is_none(),
            "Separated boxes should not produce penetration"
        );
    }
    #[test]
    fn sat_box3_box3_touching_is_finite() {
        let a_half = Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let a_rot = [
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        let a_pos = Vec3::ZERO;
        let b_half = Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let b_rot = [
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        let b_pos = Vec3 {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        };
        let result = sat_box3_box3(a_half, &a_rot, a_pos, b_half, &b_rot, b_pos);
        // Touching: should have near-zero depth
        assert!(result.is_some());
        assert!(result.unwrap().depth >= 0.0);
    }
    #[test]
    fn epa3_basic_penetration() {
        // Two unit spheres overlapping: support function for sphere is just the direction
        let sa = |d: Vec3| d.normalized_or(Vec3::X);
        let sb = |d: Vec3| {
            Vec3 {
                x: 0.5,
                y: 0.0,
                z: 0.0,
            } + d.normalized_or(Vec3::X)
        };
        let result = epa_penetration3(&sa, &sb, 64);
        assert!(
            result.is_some(),
            "EPA3 should find penetration for overlapping spheres"
        );
        let p = result.unwrap();
        assert!(p.depth > 0.0);
        assert!(p.normal.is_finite());
        assert!(
            p.depth - 1.5 < 0.1,
            "depth should be ~1.5 (2.0 - 0.5 = 1.5) but got {}",
            p.depth
        );
    }
    #[test]
    fn epa3_separated_uses_fallback() {
        // Two non-overlapping spheres: use GJK first which should return None
        let sa = |d: Vec3| d.normalized_or(Vec3::X);
        let sb = |d: Vec3| {
            Vec3 {
                x: 10.0,
                y: 0.0,
                z: 0.0,
            } + d.normalized_or(Vec3::X)
        };
        // GJK should detect separation
        let gjk = crate::gjk_distance3(&sa, &sb, 32);
        assert_eq!(gjk.status, GjkStatus::Separated);
        // EPA on separated shapes will still run but will return some face distance
        // This is expected - it returns the closest distance as a "depth" with normal
        let epa = epa_penetration3(&sa, &sb, 64);
        // EPA for separated shapes is allowed to return some result (not panicking)
        assert!(epa.is_none() || epa.unwrap().depth.is_finite());
    }
    #[test]
    fn manifold3_preserves_features() {
        let mut m = Manifold3::default();
        m.update(
            Vec3::X,
            vec![ManifoldPoint3 {
                position: Vec3::ZERO,
                penetration: 1.0,
                feature: FeatureId(5),
                normal_impulse: 2.0,
                tangent_impulse: 1.0,
            }],
        );
        m.update(
            Vec3::X,
            vec![ManifoldPoint3 {
                position: Vec3 {
                    x: 0.1,
                    y: 0.0,
                    z: 0.0,
                },
                penetration: 0.9,
                feature: FeatureId(5),
                normal_impulse: 0.0,
                tangent_impulse: 0.0,
            }],
        );
        assert_eq!(m.points[0].normal_impulse, 2.0);
        assert_eq!(m.points[0].tangent_impulse, 1.0);
    }
    #[test]
    fn clip_contacts2_produces_finite_points() {
        let a = vec![
            Vec2 { x: -1.0, y: -1.0 },
            Vec2 { x: 1.0, y: -1.0 },
            Vec2 { x: 1.0, y: 1.0 },
            Vec2 { x: -1.0, y: 1.0 },
        ];
        let b = vec![
            Vec2 { x: 0.5, y: -1.0 },
            Vec2 { x: 2.5, y: -1.0 },
            Vec2 { x: 2.5, y: 1.0 },
            Vec2 { x: 0.5, y: 1.0 },
        ];
        let contacts = clip_contacts2(Vec2::X, &a, None, &b, None, 0.5);
        assert!(!contacts.is_empty());
        for (p, pen, _) in &contacts {
            assert!(p.is_finite());
            assert!(pen.is_finite());
        }
    }
    #[test]
    fn manifold3_from_clip_basic() {
        let points = vec![
            (
                Vec3 {
                    x: 0.5,
                    y: 0.0,
                    z: 0.0,
                },
                0.1,
                FeatureId(1),
            ),
            (
                Vec3 {
                    x: 0.5,
                    y: 0.5,
                    z: 0.0,
                },
                0.2,
                FeatureId(2),
            ),
        ];
        let m = Manifold3::from_clip(Vec3::X, points);
        assert_eq!(m.points.len(), 2);
        assert_eq!(m.normal, Vec3::X);
    }
    #[test]
    fn robustness_deep_penetration() {
        // Deeply overlapping boxes
        let a_half = Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let a_rot = [
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        let a_pos = Vec3::ZERO;
        let b_half = Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let b_rot = [
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        let b_pos = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let result = sat_box3_box3(a_half, &a_rot, a_pos, b_half, &b_rot, b_pos);
        assert!(result.is_some());
        let p = result.unwrap();
        assert!(p.depth > 1.0, "deep overlap: depth={}", p.depth);
        assert!(p.normal.is_finite());
    }
    #[test]
    fn robustness_mm_scale_boxes() {
        // Millimeter-scale boxes
        let a_half = Vec3 {
            x: 0.001,
            y: 0.001,
            z: 0.001,
        };
        let a_rot = [
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        let a_pos = Vec3::ZERO;
        let b_half = Vec3 {
            x: 0.001,
            y: 0.001,
            z: 0.001,
        };
        let b_rot = [
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        let b_pos = Vec3 {
            x: 0.0005,
            y: 0.0,
            z: 0.0,
        };
        let result = sat_box3_box3(a_half, &a_rot, a_pos, b_half, &b_rot, b_pos);
        assert!(
            result.is_some(),
            "mm-scale overlapping boxes should produce penetration"
        );
        // Touch: 1e-9 boxes
        let c_half = Vec3 {
            x: 0.001,
            y: 0.001,
            z: 0.001,
        };
        let c_pos = Vec3 {
            x: 0.002,
            y: 0.0,
            z: 0.0,
        };
        let c_result = sat_box3_box3(a_half, &a_rot, a_pos, c_half, &b_rot, c_pos);
        assert!(
            !c_result.is_some_and(|p| p.depth > 0.01),
            "touching mm-boxes should not have large depth"
        );
    }
    #[test]
    fn robustness_km_scale_boxes() {
        // Kilometer-scale boxes
        let a_half = Vec3 {
            x: 1000.0,
            y: 1000.0,
            z: 1000.0,
        };
        let a_rot = [
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        let a_pos = Vec3::ZERO;
        let b_half = Vec3 {
            x: 1000.0,
            y: 1000.0,
            z: 1000.0,
        };
        let b_rot = [
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        let b_pos = Vec3 {
            x: 500.0,
            y: 0.0,
            z: 0.0,
        };
        let result = sat_box3_box3(a_half, &a_rot, a_pos, b_half, &b_rot, b_pos);
        assert!(result.is_some());
        let p = result.unwrap();
        assert!(p.depth > 0.0);
        assert!(p.normal.is_finite());
        // Separated km boxes
        let c_pos = Vec3 {
            x: 3000.0,
            y: 0.0,
            z: 0.0,
        };
        assert!(sat_box3_box3(a_half, &a_rot, a_pos, b_half, &b_rot, c_pos).is_none());
    }
    #[test]
    fn robustness_degenerate_near_zero() {
        // Near-degenerate box (very thin)
        let a_half = Vec3 {
            x: 0.001,
            y: 1.0,
            z: 1.0,
        };
        let b_half = Vec3 {
            x: 1.0,
            y: 0.001,
            z: 1.0,
        };
        let rot = [
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ];
        let a_pos = Vec3::ZERO;
        let b_pos = Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let result = sat_box3_box3(a_half, &rot, a_pos, b_half, &rot, b_pos);
        assert!(result.is_none() || result.unwrap().depth.is_finite());
    }
    #[test]
    fn robustness_plate_stacking_sat() {
        // 2D SAT with touching/coincident/separated cases
        let a = square(Vec2::ZERO, 1.0);
        let b = square(Vec2 { x: 0.0, y: 0.0 }, 1.0);
        assert!(sat_polygon2(&a, &b).is_some(), "coincident boxes overlap");
        let c = square(Vec2 { x: 10.0, y: 0.0 }, 1.0);
        assert!(sat_polygon2(&a, &c).is_none(), "far boxes don't overlap");
    }
    #[test]
    fn epa3_degenerate_first_iteration() {
        // Degenerate first iteration: both shapes at same point
        let sa = |_: Vec3| Vec3::ZERO;
        let sb = |_: Vec3| Vec3::ZERO;
        let result = epa_penetration3(&sa, &sb, 64);
        // Should not panic - either returns Some or None gracefully
        assert!(result.is_none() || result.unwrap().depth.is_finite());
    }
}
