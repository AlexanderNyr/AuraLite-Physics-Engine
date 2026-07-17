//! Extended M2 shape catalog, hull construction, compounds, meshes, heightfields, and BVH.
use crate::{GeometryError, MassProperties2, MassProperties3};
use auralite_math::{
    ABS_EPSILON, Aabb2, Aabb3, Plane2, Plane3, Ray2, Ray3, Real, Segment2, Segment3, Transform2,
    Transform3, Triangle3, Vec2, Vec3,
};
const PI: Real = core::f64::consts::PI as Real;
fn positive(v: Real) -> bool {
    v.is_finite() && v > 0.0
}
fn ray_circle(ray: Ray2, c: Vec2, r: Real) -> Option<Real> {
    let m = ray.origin - c;
    let b = m.dot(ray.direction);
    let q = m.length_squared() - r * r;
    if q > 0.0 && b > 0.0 {
        return None;
    }
    let disc = b * b - q;
    if disc < 0.0 {
        return None;
    }
    Some((-b - disc.sqrt()).max(0.0))
}
fn ray_sphere(ray: Ray3, c: Vec3, r: Real) -> Option<Real> {
    let m = ray.origin - c;
    let b = m.dot(ray.direction);
    let q = m.length_squared() - r * r;
    if q > 0.0 && b > 0.0 {
        return None;
    }
    let disc = b * b - q;
    if disc < 0.0 {
        return None;
    }
    Some((-b - disc.sqrt()).max(0.0))
}

/// A solid 2D circle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Circle2 {
    radius: Real,
}
impl Circle2 {
    /// Validates positive radius.
    pub fn new(radius: Real) -> Result<Self, GeometryError> {
        if positive(radius) {
            Ok(Self { radius })
        } else {
            Err(GeometryError::InvalidInput)
        }
    }
    /// Radius.
    #[must_use]
    pub const fn radius(self) -> Real {
        self.radius
    }
    /// Support point.
    #[must_use]
    pub fn support(self, d: Vec2) -> Vec2 {
        d.normalized_or(Vec2::X) * self.radius
    }
    /// Bounds.
    #[must_use]
    pub fn aabb(self) -> Aabb2 {
        let h = Vec2 {
            x: self.radius,
            y: self.radius,
        };
        Aabb2::new(-h, h).expect("valid circle")
    }
    /// Bounding radius.
    #[must_use]
    pub const fn bounding_radius(self) -> Real {
        self.radius
    }
    /// Containment.
    #[must_use]
    pub fn contains(self, p: Vec2) -> bool {
        p.is_finite() && p.length_squared() <= self.radius * self.radius
    }
    /// Closest point in/on shape.
    #[must_use]
    pub fn closest_point(self, p: Vec2) -> Vec2 {
        if self.contains(p) {
            p
        } else {
            p.normalized_or(Vec2::X) * self.radius
        }
    }
    /// Ray hit.
    #[must_use]
    pub fn ray_t(self, r: Ray2) -> Option<Real> {
        ray_circle(r, Vec2::ZERO, self.radius)
    }
    /// Exact ray hit and outward normal.
    #[must_use]
    pub fn ray_intersection(self, r: Ray2) -> Option<(Real, Vec2)> {
        let t = self.ray_t(r)?;
        let p = r.origin + r.direction * t;
        Some((t, p.normalized_or(Vec2::Y)))
    }
    /// Mass properties.
    pub fn mass_properties(self, density: Real) -> Result<MassProperties2, GeometryError> {
        if !positive(density) {
            return Err(GeometryError::InvalidInput);
        }
        let m = PI * self.radius * self.radius * density;
        Ok(MassProperties2 {
            mass: m,
            center: Vec2::ZERO,
            inertia: m * self.radius * self.radius * 0.5,
        })
    }
    /// Uniform scaling.
    pub fn scaled(self, s: Real) -> Result<Self, GeometryError> {
        Self::new(self.radius * s)
    }
}
/// A solid 3D sphere.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sphere3 {
    /// Radius.
    pub radius: Real,
}
impl Sphere3 {
    /// Validates positive radius.
    pub fn new(radius: Real) -> Result<Self, GeometryError> {
        if positive(radius) {
            Ok(Self { radius })
        } else {
            Err(GeometryError::InvalidInput)
        }
    }
    /// Radius.
    #[must_use]
    pub const fn radius(self) -> Real {
        self.radius
    }
    /// Support point.
    #[must_use]
    pub fn support(self, d: Vec3) -> Vec3 {
        d.normalized_or(Vec3::X) * self.radius
    }
    /// Bounds.
    #[must_use]
    pub fn aabb(self) -> Aabb3 {
        let h = Vec3 {
            x: self.radius,
            y: self.radius,
            z: self.radius,
        };
        Aabb3::new(-h, h).expect("valid sphere")
    }
    /// Bounding radius.
    #[must_use]
    pub const fn bounding_radius(self) -> Real {
        self.radius
    }
    /// Containment.
    #[must_use]
    pub fn contains(self, p: Vec3) -> bool {
        p.is_finite() && p.length_squared() <= self.radius * self.radius
    }
    /// Closest point.
    #[must_use]
    pub fn closest_point(self, p: Vec3) -> Vec3 {
        if self.contains(p) {
            p
        } else {
            p.normalized_or(Vec3::X) * self.radius
        }
    }
    /// Ray hit.
    #[must_use]
    pub fn ray_t(self, r: Ray3) -> Option<Real> {
        ray_sphere(r, Vec3::ZERO, self.radius)
    }
    /// Exact ray hit and outward normal.
    #[must_use]
    pub fn ray_intersection(self, r: Ray3) -> Option<(Real, Vec3)> {
        let t = self.ray_t(r)?;
        let p = r.origin + r.direction * t;
        Some((t, p.normalized_or(Vec3::Y)))
    }
    /// Volume of the sphere.
    #[must_use]
    pub fn volume(self) -> Real {
        4.0 * PI * self.radius.powi(3) / 3.0
    }
    /// Mass properties.
    pub fn mass_properties(self, density: Real) -> Result<MassProperties3, GeometryError> {
        if !positive(density) {
            return Err(GeometryError::InvalidInput);
        }
        let m = 4.0 * PI * self.radius.powi(3) * density / 3.0;
        let i = 0.4 * m * self.radius * self.radius;
        Ok(MassProperties3 {
            mass: m,
            center: Vec3::ZERO,
            inertia_diagonal: Vec3 { x: i, y: i, z: i },
        })
    }
    /// Uniform scaling.
    pub fn scaled(self, s: Real) -> Result<Self, GeometryError> {
        Self::new(self.radius * s)
    }
}

/// A finite zero-area 2D edge, with no mass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Edge2 {
    segment: Segment2,
}
impl Edge2 {
    /// Constructs a non-degenerate edge.
    pub fn new(a: Vec2, b: Vec2) -> Result<Self, GeometryError> {
        Ok(Self {
            segment: Segment2::new(a, b)?,
        })
    }
    /// Endpoints.
    #[must_use]
    pub const fn endpoints(self) -> (Vec2, Vec2) {
        (self.segment.a, self.segment.b)
    }
    /// True analytic support point.
    #[must_use]
    pub fn support(self, d: Vec2) -> Vec2 {
        if self.segment.a.dot(d) >= self.segment.b.dot(d) {
            self.segment.a
        } else {
            self.segment.b
        }
    }
    /// Closest point.
    #[must_use]
    pub fn closest_point(self, p: Vec2) -> Vec2 {
        self.segment.closest_point(p).0
    }
    /// Bounds.
    #[must_use]
    pub fn aabb(self) -> Aabb2 {
        Aabb2::new(
            Vec2 {
                x: self.segment.a.x.min(self.segment.b.x),
                y: self.segment.a.y.min(self.segment.b.y),
            },
            Vec2 {
                x: self.segment.a.x.max(self.segment.b.x),
                y: self.segment.a.y.max(self.segment.b.y),
            },
        )
        .expect("valid edge")
    }
    /// Bounding radius about origin.
    #[must_use]
    pub fn bounding_radius(self) -> Real {
        self.segment.a.length().max(self.segment.b.length())
    }
    /// Ray-edge intersection.
    #[must_use]
    pub fn ray_t(self, ray: Ray2) -> Option<Real> {
        let e = self.segment.b - self.segment.a;
        let det = ray.direction.cross(e);
        if det.abs() <= ABS_EPSILON {
            return None;
        }
        let q = self.segment.a - ray.origin;
        let t = q.cross(e) / det;
        let u = q.cross(ray.direction) / det;
        if t >= 0.0 && (0.0..=1.0).contains(&u) {
            Some(t)
        } else {
            None
        }
    }
    /// Exact ray intersection and normal across edge segment.
    #[must_use]
    pub fn ray_intersection(self, ray: Ray2) -> Option<(Real, Vec2)> {
        let t = self.ray_t(ray)?;
        let e = self.segment.b - self.segment.a;
        let n = Vec2 { x: e.y, y: -e.x }.normalized_or(Vec2::Y);
        let normal = if n.dot(ray.direction) <= 0.0 { n } else { -n };
        Some((t, normal))
    }
    /// Uniform scaling.
    pub fn scaled(self, s: Real) -> Result<Self, GeometryError> {
        if !positive(s) {
            Err(GeometryError::InvalidInput)
        } else {
            Self::new(self.segment.a * s, self.segment.b * s)
        }
    }
}

/// A finite zero-volume 3D segment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Edge3 {
    segment: Segment3,
}
impl Edge3 {
    /// Constructs an edge.
    pub fn new(a: Vec3, b: Vec3) -> Result<Self, GeometryError> {
        Ok(Self {
            segment: Segment3::new(a, b)?,
        })
    }
    /// Closest point.
    #[must_use]
    pub fn closest_point(self, p: Vec3) -> Vec3 {
        self.segment.closest_point(p).0
    }
    /// Ray intersection (1D segment in 3D has 0 surface area).
    #[must_use]
    pub fn ray_intersection(self, _r: Ray3) -> Option<(Real, Vec3)> {
        None
    }
    /// Bounds.
    #[must_use]
    pub fn aabb(self) -> Aabb3 {
        Aabb3::new(
            Vec3 {
                x: self.segment.a.x.min(self.segment.b.x),
                y: self.segment.a.y.min(self.segment.b.y),
                z: self.segment.a.z.min(self.segment.b.z),
            },
            Vec3 {
                x: self.segment.a.x.max(self.segment.b.x),
                y: self.segment.a.y.max(self.segment.b.y),
                z: self.segment.a.z.max(self.segment.b.z),
            },
        )
        .expect("valid edge")
    }
    /// Bounding radius.
    #[must_use]
    pub fn bounding_radius(self) -> Real {
        self.segment.a.length().max(self.segment.b.length())
    }
    /// Uniform scaling.
    pub fn scaled(self, s: Real) -> Result<Self, GeometryError> {
        if !positive(s) {
            Err(GeometryError::InvalidInput)
        } else {
            Self::new(self.segment.a * s, self.segment.b * s)
        }
    }
}

/// A convex polyhedron built from a finite point cloud using enumerated supporting faces.
#[derive(Clone, Debug, PartialEq)]
pub struct ConvexHull3 {
    /// Vertices.
    pub vertices: Vec<Vec3>,
    /// Faces.
    pub faces: Vec<[usize; 3]>,
    /// Bounding box.
    pub aabb: Aabb3,
}
impl ConvexHull3 {
    /// Builds a hull. Duplicate points are removed; coplanar-only input is rejected. Face enumeration is O(n^4), intended as a robust reference builder.
    pub fn build(points: &[Vec3]) -> Result<Self, GeometryError> {
        let mut v = Vec::new();
        for &p in points {
            if !p.is_finite() {
                return Err(GeometryError::InvalidInput);
            }
            if !v
                .iter()
                .any(|q: &Vec3| (*q - p).length_squared() <= ABS_EPSILON * ABS_EPSILON)
            {
                v.push(p);
            }
        }
        if v.len() < 4 {
            return Err(GeometryError::InvalidPolygon);
        }
        let mut faces = Vec::new();
        for i in 0..v.len() {
            for j in i + 1..v.len() {
                for k in j + 1..v.len() {
                    let n = (v[j] - v[i]).cross(v[k] - v[i]);
                    if n.length_squared() <= ABS_EPSILON * ABS_EPSILON {
                        continue;
                    }
                    let mut pos = false;
                    let mut neg = false;
                    for (m, p) in v.iter().enumerate() {
                        if m == i || m == j || m == k {
                            continue;
                        }
                        let d = n.dot(*p - v[i]);
                        pos |= d > ABS_EPSILON;
                        neg |= d < -ABS_EPSILON;
                    }
                    if pos && neg {
                        continue;
                    }
                    let f = if pos { [i, k, j] } else { [i, j, k] };
                    faces.push(f);
                }
            }
        }
        if faces.len() < 4 {
            return Err(GeometryError::InvalidPolygon);
        }
        let min = Vec3 {
            x: v.iter().map(|p| p.x).fold(Real::INFINITY, Real::min),
            y: v.iter().map(|p| p.y).fold(Real::INFINITY, Real::min),
            z: v.iter().map(|p| p.z).fold(Real::INFINITY, Real::min),
        };
        let max = Vec3 {
            x: v.iter().map(|p| p.x).fold(Real::NEG_INFINITY, Real::max),
            y: v.iter().map(|p| p.y).fold(Real::NEG_INFINITY, Real::max),
            z: v.iter().map(|p| p.z).fold(Real::NEG_INFINITY, Real::max),
        };
        Ok(Self {
            vertices: v,
            faces,
            aabb: Aabb3::new(min, max)?,
        })
    }
    /// Hull vertices.
    #[must_use]
    pub fn vertices(&self) -> &[Vec3] {
        &self.vertices
    }
    /// Triangular supporting faces.
    #[must_use]
    pub fn faces(&self) -> &[[usize; 3]] {
        &self.faces
    }
    /// Support map.
    #[must_use]
    pub fn support(&self, d: Vec3) -> Vec3 {
        let mut b = self.vertices[0];
        let mut q = b.dot(d);
        for &p in &self.vertices[1..] {
            let x = p.dot(d);
            if x > q {
                q = x;
                b = p;
            }
        }
        b
    }
    /// Bounds.
    #[must_use]
    pub const fn aabb(&self) -> Aabb3 {
        self.aabb
    }
    /// Bounding radius.
    #[must_use]
    pub fn bounding_radius(&self) -> Real {
        self.vertices
            .iter()
            .map(|p| p.length())
            .fold(0.0, Real::max)
    }
    /// Half-space containment.
    #[must_use]
    pub fn contains(&self, p: Vec3) -> bool {
        self.faces.iter().all(|f| {
            let a = self.vertices[f[0]];
            let n = (self.vertices[f[1]] - a).cross(self.vertices[f[2]] - a);
            n.dot(p - a) <= ABS_EPSILON
        })
    }
    /// Closest point over boundary triangles, preserving interior points.
    #[must_use]
    pub fn closest_point(&self, p: Vec3) -> Vec3 {
        if self.contains(p) {
            return p;
        }
        self.faces
            .iter()
            .map(|f| {
                Triangle3::new(
                    self.vertices[f[0]],
                    self.vertices[f[1]],
                    self.vertices[f[2]],
                )
                .expect("hull face")
                .closest_point(p)
            })
            .min_by(|a, b| {
                (p - *a)
                    .length_squared()
                    .total_cmp(&(p - *b).length_squared())
            })
            .expect("faces")
    }
    /// Closest ray hit.
    #[must_use]
    pub fn ray_t(&self, r: Ray3) -> Option<Real> {
        self.faces
            .iter()
            .filter_map(|f| {
                Triangle3::new(
                    self.vertices[f[0]],
                    self.vertices[f[1]],
                    self.vertices[f[2]],
                )
                .ok()?
                .ray_intersection(r)
                .map(|x| x.0)
            })
            .min_by(Real::total_cmp)
    }
    /// Exact ray intersection and outward normal.
    #[must_use]
    pub fn ray_intersection(&self, r: Ray3) -> Option<(Real, Vec3)> {
        self.faces
            .iter()
            .filter_map(|f| {
                Triangle3::new(
                    self.vertices[f[0]],
                    self.vertices[f[1]],
                    self.vertices[f[2]],
                )
                .ok()?
                .ray_intersection(r)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
    }
    /// Uniform scaling.
    pub fn scaled(&self, s: Real) -> Result<Self, GeometryError> {
        if !positive(s) {
            return Err(GeometryError::InvalidInput);
        }
        Self::build(&self.vertices.iter().map(|p| *p * s).collect::<Vec<_>>())
    }
    /// Volume of the convex hull.
    #[must_use]
    pub fn volume(&self) -> Real {
        self.mass_properties(1.0).map(|m| m.mass).unwrap_or(0.0)
    }
    /// Signed-tetrahedra numerical mass properties; accurate for consistently oriented closed hull faces.
    pub fn mass_properties(&self, density: Real) -> Result<MassProperties3, GeometryError> {
        if !positive(density) {
            return Err(GeometryError::InvalidInput);
        }
        let mut volume = 0.0;
        let mut center = Vec3::ZERO;
        for f in &self.faces {
            let a = self.vertices[f[0]];
            let b = self.vertices[f[1]];
            let c = self.vertices[f[2]];
            let n = (b - a).cross(c - a).normalized_or(Vec3::X);
            let coplanar_vertices = self
                .vertices
                .iter()
                .filter(|p| n.dot(**p - a).abs() <= ABS_EPSILON * 4.0)
                .count();
            // Enumerating every supporting triple covers an n-gon facet (n-2) times.
            let weight = 1.0 / (coplanar_vertices.saturating_sub(2).max(1) as Real);
            let v = a.dot(b.cross(c)) * weight / 6.0;
            volume += v;
            center += (a + b + c) * (v / 4.0);
        }
        if volume.abs() <= ABS_EPSILON {
            return Err(GeometryError::InvalidPolygon);
        }
        center /= volume;
        let mass = volume.abs() * density;
        let mut diag = Vec3::ZERO;
        for &p in &self.vertices {
            let q = p - center;
            diag += Vec3 {
                x: q.y * q.y + q.z * q.z,
                y: q.x * q.x + q.z * q.z,
                z: q.x * q.x + q.y * q.y,
            };
        }
        diag *= mass / self.vertices.len() as Real;
        Ok(MassProperties3 {
            mass,
            center,
            inertia_diagonal: diag,
        })
    }
}

/// A triangle mesh BVH node.
#[derive(Clone, Debug, PartialEq)]
pub struct BvhNode {
    /// Bounds.
    pub bounds: Aabb3,
    /// Child indices, absent for a leaf.
    pub children: Option<(usize, usize)>,
    /// Triangle indices for leaves.
    pub triangles: Vec<usize>,
}
/// Validated triangle mesh with deterministic median-split BVH.
#[derive(Clone, Debug, PartialEq)]
pub struct TriangleMesh {
    /// Vertices.
    pub vertices: Vec<Vec3>,
    /// Triangle indices.
    pub indices: Vec<[u32; 3]>,
    /// BVH nodes.
    pub nodes: Vec<BvhNode>,
    /// Root node index.
    pub root: usize,
}
impl TriangleMesh {
    /// Validates indices/triangles and builds a BVH.
    pub fn new(vertices: Vec<Vec3>, indices: Vec<[u32; 3]>) -> Result<Self, GeometryError> {
        if vertices.is_empty() || indices.is_empty() || vertices.iter().any(|p| !p.is_finite()) {
            return Err(GeometryError::InvalidInput);
        }
        for f in &indices {
            let [a, b, c] = f.map(|x| x as usize);
            if a >= vertices.len()
                || b >= vertices.len()
                || c >= vertices.len()
                || Triangle3::new(vertices[a], vertices[b], vertices[c]).is_err()
            {
                return Err(GeometryError::InvalidInput);
            }
        }
        let mut m = Self {
            vertices,
            indices,
            nodes: Vec::new(),
            root: 0,
        };
        m.root = m.build_node((0..m.indices.len()).collect());
        Ok(m)
    }
    fn tri_bounds(&self, i: usize) -> Aabb3 {
        let f = self.indices[i];
        let a = self.vertices[f[0] as usize];
        let b = self.vertices[f[1] as usize];
        let c = self.vertices[f[2] as usize];
        Aabb3::new(
            Vec3 {
                x: a.x.min(b.x).min(c.x),
                y: a.y.min(b.y).min(c.y),
                z: a.z.min(b.z).min(c.z),
            },
            Vec3 {
                x: a.x.max(b.x).max(c.x),
                y: a.y.max(b.y).max(c.y),
                z: a.z.max(b.z).max(c.z),
            },
        )
        .expect("triangle")
    }
    fn build_node(&mut self, mut tris: Vec<usize>) -> usize {
        let first = self.tri_bounds(tris[0]);
        let mut min = first.min;
        let mut max = first.max;
        for &i in &tris[1..] {
            let a = self.tri_bounds(i);
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
        let idx = self.nodes.len();
        self.nodes.push(BvhNode {
            bounds: Aabb3::new(min, max).expect("bounds"),
            children: None,
            triangles: Vec::new(),
        });
        if tris.len() <= 4 {
            self.nodes[idx].triangles = tris;
        } else {
            let extent = max - min;
            let axis = if extent.x >= extent.y && extent.x >= extent.z {
                0
            } else if extent.y >= extent.z {
                1
            } else {
                2
            };
            tris.sort_by(|&a, &b| {
                let ca = self.tri_bounds(a);
                let cb = self.tri_bounds(b);
                let va = match axis {
                    0 => ca.min.x + ca.max.x,
                    1 => ca.min.y + ca.max.y,
                    _ => ca.min.z + ca.max.z,
                };
                let vb = match axis {
                    0 => cb.min.x + cb.max.x,
                    1 => cb.min.y + cb.max.y,
                    _ => cb.min.z + cb.max.z,
                };
                va.total_cmp(&vb).then(a.cmp(&b))
            });
            let r = tris.split_off(tris.len() / 2);
            let l = self.build_node(tris);
            let r = self.build_node(r);
            self.nodes[idx].children = Some((l, r));
        }
        idx
    }
    /// BVH nodes.
    #[must_use]
    pub fn bvh(&self) -> &[BvhNode] {
        &self.nodes
    }
    /// Bounds.
    #[must_use]
    pub fn aabb(&self) -> Aabb3 {
        self.nodes[self.root].bounds
    }
    /// Bounding radius.
    #[must_use]
    pub fn bounding_radius(&self) -> Real {
        self.vertices
            .iter()
            .map(|p| p.length())
            .fold(0.0, Real::max)
    }
    /// Closest point by exact triangle traversal.
    #[must_use]
    pub fn closest_point(&self, p: Vec3) -> Vec3 {
        self.indices
            .iter()
            .map(|f| {
                Triangle3::new(
                    self.vertices[f[0] as usize],
                    self.vertices[f[1] as usize],
                    self.vertices[f[2] as usize],
                )
                .expect("valid")
                .closest_point(p)
            })
            .min_by(|a, b| {
                (p - *a)
                    .length_squared()
                    .total_cmp(&(p - *b).length_squared())
            })
            .expect("triangles")
    }
    /// Closest ray hit (exact triangle reference; BVH representation is available for accelerated traversal integration).
    #[must_use]
    pub fn ray_t(&self, r: Ray3) -> Option<Real> {
        self.indices
            .iter()
            .filter_map(|f| {
                Triangle3::new(
                    self.vertices[f[0] as usize],
                    self.vertices[f[1] as usize],
                    self.vertices[f[2] as usize],
                )
                .ok()?
                .ray_intersection(r)
                .map(|x| x.0)
            })
            .min_by(Real::total_cmp)
    }
    /// Exact ray hit and normal across mesh triangles.
    #[must_use]
    pub fn ray_intersection(&self, r: Ray3) -> Option<(Real, Vec3)> {
        self.indices
            .iter()
            .filter_map(|f| {
                Triangle3::new(
                    self.vertices[f[0] as usize],
                    self.vertices[f[1] as usize],
                    self.vertices[f[2] as usize],
                )
                .ok()?
                .ray_intersection(r)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
    }
    /// Uniform scaling and BVH rebuild.
    pub fn scaled(&self, s: Real) -> Result<Self, GeometryError> {
        if !positive(s) {
            return Err(GeometryError::InvalidInput);
        }
        Self::new(
            self.vertices.iter().map(|p| *p * s).collect(),
            self.indices.clone(),
        )
    }
    /// BVH-accelerated ray hit query. Returns the closest triangle hit distance.
    #[must_use]
    pub fn ray_t_bvh(&self, r: Ray3) -> Option<Real> {
        self._ray_bvh_recursive(r, self.root, Real::INFINITY)
    }
    #[allow(clippy::collapsible_if)]
    fn _ray_bvh_recursive(&self, r: Ray3, node_idx: usize, mut best: Real) -> Option<Real> {
        let node = &self.nodes[node_idx];
        // Check if ray hits this node's bounds
        let (lo, _hi) = ray_aabb3_interval(r, node.bounds)?;
        if lo > best {
            return None;
        }
        match node.children {
            Some((left_child, right_child)) => {
                let left = self._ray_bvh_recursive(r, left_child, best);
                if let Some(t) = left {
                    best = best.min(t);
                }
                let right = self._ray_bvh_recursive(r, right_child, best);
                right.or(left)
            }
            None => {
                // Leaf node: check triangles
                let mut closest: Option<Real> = None;
                for &ti in &node.triangles {
                    let tri_idx = self.indices[ti];
                    if let Ok(tri) = Triangle3::new(
                        self.vertices[tri_idx[0] as usize],
                        self.vertices[tri_idx[1] as usize],
                        self.vertices[tri_idx[2] as usize],
                    ) {
                        if let Some((t, _)) = tri.ray_intersection(r) {
                            if t <= best {
                                best = t;
                                closest = Some(t);
                            }
                        }
                    }
                }
                closest
            }
        }
    }
    /// BVH-accelerated closest point query.
    #[must_use]
    pub fn closest_point_bvh(&self, p: Vec3) -> Vec3 {
        let mut best_dist = Real::INFINITY;
        let mut best_pt = p;
        self._closest_bvh_recursive(p, self.root, &mut best_dist, &mut best_pt);
        best_pt
    }
    fn _closest_bvh_recursive(
        &self,
        p: Vec3,
        node_idx: usize,
        best_dist: &mut Real,
        best_pt: &mut Vec3,
    ) {
        let node = &self.nodes[node_idx];
        // Early exit if the node bounds are farther than current best
        let d_bound = closest_point_aabb3(p, node.bounds);
        if d_bound >= *best_dist {
            return;
        }
        match node.children {
            Some((left_child, right_child)) => {
                // Process closer child first
                let dl = closest_point_aabb3(p, self.nodes[left_child].bounds);
                let dr = closest_point_aabb3(p, self.nodes[right_child].bounds);
                if dl <= dr {
                    self._closest_bvh_recursive(p, left_child, best_dist, best_pt);
                    self._closest_bvh_recursive(p, right_child, best_dist, best_pt);
                } else {
                    self._closest_bvh_recursive(p, right_child, best_dist, best_pt);
                    self._closest_bvh_recursive(p, left_child, best_dist, best_pt);
                }
            }
            None => {
                for &ti in &node.triangles {
                    let tri_idx = self.indices[ti];
                    if let Ok(tri) = Triangle3::new(
                        self.vertices[tri_idx[0] as usize],
                        self.vertices[tri_idx[1] as usize],
                        self.vertices[tri_idx[2] as usize],
                    ) {
                        let pt = tri.closest_point(p);
                        let d = (pt - p).length_squared();
                        if d < *best_dist {
                            *best_dist = d;
                            *best_pt = pt;
                        }
                    }
                }
            }
        }
    }
}

/// Returns the squared distance from point p to the AABB.
fn closest_point_aabb3(p: Vec3, a: Aabb3) -> Real {
    let c = (a.min + a.max) * 0.5;
    let h = (a.max - a.min) * 0.5;
    let d = Vec3 {
        x: (p.x - c.x).abs() - h.x,
        y: (p.y - c.y).abs() - h.y,
        z: (p.z - c.z).abs() - h.z,
    };
    let d2 = Vec3 {
        x: if d.x > 0.0 { d.x } else { 0.0 },
        y: if d.y > 0.0 { d.y } else { 0.0 },
        z: if d.z > 0.0 { d.z } else { 0.0 },
    };
    d2.length_squared()
}
/// Returns the intersection interval of a ray with an AABB.
fn ray_aabb3_interval(r: Ray3, a: Aabb3) -> Option<(Real, Real)> {
    let mut lo = 0.0f32 as Real;
    let mut hi = Real::INFINITY;
    for (o, d, mn, mx) in [
        (r.origin.x, r.direction.x, a.min.x, a.max.x),
        (r.origin.y, r.direction.y, a.min.y, a.max.y),
        (r.origin.z, r.direction.z, a.min.z, a.max.z),
    ] {
        if d.abs() < 1.0e-8 {
            if o < mn || o > mx {
                return None;
            }
        } else {
            let mut a = (mn - o) / d;
            let mut b = (mx - o) / d;
            if a > b {
                core::mem::swap(&mut a, &mut b);
            }
            lo = lo.max(a);
            hi = hi.min(b);
            if lo > hi {
                return None;
            }
        }
    }
    Some((lo, hi))
}

/// A 2D heightfield chain sampled at uniform x spacing.
#[derive(Clone, Debug, PartialEq)]
pub struct Heightfield2 {
    heights: Vec<Real>,
    spacing: Real,
}
impl Heightfield2 {
    /// Validates at least two finite samples and positive spacing.
    pub fn new(heights: Vec<Real>, spacing: Real) -> Result<Self, GeometryError> {
        if heights.len() >= 2 && positive(spacing) && heights.iter().all(|x| x.is_finite()) {
            Ok(Self { heights, spacing })
        } else {
            Err(GeometryError::InvalidInput)
        }
    }
    /// Number of edge segments.
    #[must_use]
    pub fn segment_count(&self) -> usize {
        self.heights.len() - 1
    }
    /// Returns an edge.
    #[must_use]
    pub fn segment(&self, i: usize) -> Option<Edge2> {
        (i < self.segment_count()).then(|| {
            Edge2::new(
                Vec2 {
                    x: i as Real * self.spacing,
                    y: self.heights[i],
                },
                Vec2 {
                    x: (i + 1) as Real * self.spacing,
                    y: self.heights[i + 1],
                },
            )
            .expect("samples")
        })
    }
    /// Bounds.
    #[must_use]
    pub fn aabb(&self) -> Aabb2 {
        let lo = self.heights.iter().copied().fold(Real::INFINITY, Real::min);
        let hi = self
            .heights
            .iter()
            .copied()
            .fold(Real::NEG_INFINITY, Real::max);
        Aabb2::new(
            Vec2 { x: 0.0, y: lo },
            Vec2 {
                x: (self.heights.len() - 1) as Real * self.spacing,
                y: hi,
            },
        )
        .expect("heightfield")
    }
    /// Closest point.
    #[must_use]
    pub fn closest_point(&self, p: Vec2) -> Vec2 {
        (0..self.segment_count())
            .map(|i| self.segment(i).expect("index").closest_point(p))
            .min_by(|a, b| {
                (p - *a)
                    .length_squared()
                    .total_cmp(&(p - *b).length_squared())
            })
            .expect("segments")
    }
    /// Closest ray hit.
    #[must_use]
    pub fn ray_t(&self, r: Ray2) -> Option<Real> {
        (0..self.segment_count())
            .filter_map(|i| self.segment(i)?.ray_t(r))
            .min_by(Real::total_cmp)
    }
    /// Bounding-circle radius about the local origin.
    #[must_use]
    pub fn bounding_radius(&self) -> Real {
        let a = self.aabb();
        a.min.length().max(a.max.length())
    }
    /// Treats the chain as the top of a solid region within its sampled x range.
    #[must_use]
    pub fn contains(&self, p: Vec2) -> bool {
        if !p.is_finite() || p.x < 0.0 || p.x > (self.heights.len() - 1) as Real * self.spacing {
            return false;
        }
        let i = ((p.x / self.spacing).floor() as usize).min(self.segment_count() - 1);
        let t = (p.x - i as Real * self.spacing) / self.spacing;
        let y = self.heights[i] * (1.0 - t) + self.heights[i + 1] * t;
        p.y <= y
    }
    /// Uniform scaling.
    pub fn scaled(&self, s: Real) -> Result<Self, GeometryError> {
        if !positive(s) {
            return Err(GeometryError::InvalidInput);
        }
        Self::new(
            self.heights.iter().map(|x| *x * s).collect(),
            self.spacing * s,
        )
    }
}
/// A 3D regular-grid heightfield represented as an accelerated triangle mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct Heightfield3 {
    mesh: TriangleMesh,
    rows: usize,
    cols: usize,
    spacing: Real,
}
impl Heightfield3 {
    /// Builds a row-major grid with at least 2×2 finite heights.
    pub fn new(
        rows: usize,
        cols: usize,
        heights: Vec<Real>,
        spacing: Real,
    ) -> Result<Self, GeometryError> {
        if rows < 2
            || cols < 2
            || heights.len() != rows * cols
            || !positive(spacing)
            || heights.iter().any(|x| !x.is_finite())
        {
            return Err(GeometryError::InvalidInput);
        }
        let vertices = heights
            .iter()
            .enumerate()
            .map(|(i, &y)| Vec3 {
                x: (i % cols) as Real * spacing,
                y,
                z: (i / cols) as Real * spacing,
            })
            .collect();
        let mut indices = Vec::new();
        for r in 0..rows - 1 {
            for c in 0..cols - 1 {
                let a = (r * cols + c) as u32;
                let b = a + 1;
                let d = ((r + 1) * cols + c) as u32;
                let e = d + 1;
                indices.push([a, d, b]);
                indices.push([b, d, e]);
            }
        }
        Ok(Self {
            mesh: TriangleMesh::new(vertices, indices)?,
            rows,
            cols,
            spacing,
        })
    }
    /// Underlying accelerated mesh.
    #[must_use]
    pub const fn mesh(&self) -> &TriangleMesh {
        &self.mesh
    }
    /// Grid dimensions and spacing.
    #[must_use]
    pub const fn layout(&self) -> (usize, usize, Real) {
        (self.rows, self.cols, self.spacing)
    }
    /// Local bounds.
    #[must_use]
    pub fn aabb(&self) -> Aabb3 {
        self.mesh.aabb()
    }
    /// Bounding-sphere radius about local origin.
    #[must_use]
    pub fn bounding_radius(&self) -> Real {
        self.mesh.bounding_radius()
    }
    /// Closest surface point.
    #[must_use]
    pub fn closest_point(&self, p: Vec3) -> Vec3 {
        self.mesh.closest_point(p)
    }
    /// Closest forward ray hit.
    #[must_use]
    pub fn ray_t(&self, r: Ray3) -> Option<Real> {
        self.mesh.ray_t(r)
    }
    /// Exact ray hit and normal.
    #[must_use]
    pub fn ray_intersection(&self, r: Ray3) -> Option<(Real, Vec3)> {
        self.mesh.ray_intersection(r)
    }
    /// Uniformly scales sample spacing and heights, rebuilding acceleration.
    pub fn scaled(&self, s: Real) -> Result<Self, GeometryError> {
        if !positive(s) {
            return Err(GeometryError::InvalidInput);
        }
        let heights = self.mesh.vertices.iter().map(|v| v.y * s).collect();
        Self::new(self.rows, self.cols, heights, self.spacing * s)
    }
}

/// A transformed 2D child shape used by compounds.
#[derive(Clone, Debug, PartialEq)]
pub struct CompoundCircle2 {
    /// Child shape.
    pub shape: Circle2,
    /// Child transform.
    pub transform: Transform2,
    /// Child density.
    pub density: Real,
}
/// A compound of solid circles with exact combined center and parallel-axis inertia.
#[derive(Clone, Debug, PartialEq)]
pub struct Compound2 {
    children: Vec<CompoundCircle2>,
}
impl Compound2 {
    /// Validates nonempty children and densities.
    pub fn new(children: Vec<CompoundCircle2>) -> Result<Self, GeometryError> {
        if children.is_empty()
            || children
                .iter()
                .any(|c| !positive(c.density) || !c.transform.translation.is_finite())
        {
            Err(GeometryError::InvalidInput)
        } else {
            Ok(Self { children })
        }
    }
    /// Combined mass properties.
    pub fn mass_properties(&self) -> Result<MassProperties2, GeometryError> {
        let props = self
            .children
            .iter()
            .map(|c| {
                c.shape
                    .mass_properties(c.density)
                    .map(|p| (p, c.transform.translation))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let mass: Real = props.iter().map(|x| x.0.mass).sum();
        let center = props.iter().fold(Vec2::ZERO, |a, x| a + x.1 * x.0.mass) / mass;
        let inertia = props
            .iter()
            .map(|x| x.0.inertia + x.0.mass * (x.1 - center).length_squared())
            .sum();
        Ok(MassProperties2 {
            mass,
            center,
            inertia,
        })
    }
    /// Bounds.
    #[must_use]
    pub fn aabb(&self) -> Aabb2 {
        let mut min = Vec2 {
            x: Real::INFINITY,
            y: Real::INFINITY,
        };
        let mut max = -min;
        for c in &self.children {
            let r = c.shape.radius();
            let a = c.transform.translation - Vec2 { x: r, y: r };
            let b = c.transform.translation + Vec2 { x: r, y: r };
            min = Vec2 {
                x: min.x.min(a.x),
                y: min.y.min(a.y),
            };
            max = Vec2 {
                x: max.x.max(b.x),
                y: max.y.max(b.y),
            };
        }
        Aabb2::new(min, max).expect("compound")
    }
}

/// Infinite 2D half-space shape; mass and finite bounds are intentionally undefined.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HalfSpace2 {
    /// Boundary.
    pub plane: Plane2,
}
impl HalfSpace2 {
    /// Constructs from a validated plane.
    #[must_use]
    pub const fn new(plane: Plane2) -> Self {
        Self { plane }
    }
    /// Containment on the negative side.
    #[must_use]
    pub fn contains(self, p: Vec2) -> bool {
        self.plane.signed_distance(p) <= 0.0
    }
    /// Closest point.
    #[must_use]
    pub fn closest_point(self, p: Vec2) -> Vec2 {
        if self.contains(p) {
            p
        } else {
            self.plane.closest_point(p)
        }
    }
    /// Ray boundary hit.
    #[must_use]
    pub fn ray_t(self, r: Ray2) -> Option<Real> {
        self.plane.ray_t(r)
    }
}
/// Infinite 3D half-space shape; mass and finite bounds are intentionally undefined.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HalfSpace3 {
    /// Boundary.
    pub plane: Plane3,
}
impl HalfSpace3 {
    /// Constructs from a validated plane.
    #[must_use]
    pub const fn new(plane: Plane3) -> Self {
        Self { plane }
    }
    /// Containment on negative side.
    #[must_use]
    pub fn contains(self, p: Vec3) -> bool {
        self.plane.signed_distance(p) <= 0.0
    }
    /// Closest point.
    #[must_use]
    pub fn closest_point(self, p: Vec3) -> Vec3 {
        if self.contains(p) {
            p
        } else {
            p - self.plane.normal * self.plane.signed_distance(p)
        }
    }
    /// Ray boundary hit.
    #[must_use]
    pub fn ray_t(self, r: Ray3) -> Option<Real> {
        self.plane.ray_t(r)
    }
}

impl TriangleMesh {
    /// Closed-mesh mass properties from signed tetrahedra. Open/inconsistently wound meshes are rejected when signed volume is near zero. Diagonal inertia uses deterministic tetra-centroid quadrature and is documented as first-order numerical integration.
    pub fn mass_properties(&self, density: Real) -> Result<MassProperties3, GeometryError> {
        if !positive(density) {
            return Err(GeometryError::InvalidInput);
        }
        let mut volume = 0.0;
        let mut weighted = Vec3::ZERO;
        let mut tetra = Vec::with_capacity(self.indices.len());
        for f in &self.indices {
            let a = self.vertices[f[0] as usize];
            let b = self.vertices[f[1] as usize];
            let c = self.vertices[f[2] as usize];
            let v = a.dot(b.cross(c)) / 6.0;
            volume += v;
            let center = (a + b + c) * 0.25;
            weighted += center * v;
            tetra.push((v, center));
        }
        if volume.abs() <= ABS_EPSILON {
            return Err(GeometryError::InvalidPolygon);
        }
        let center = weighted / volume;
        let mass = volume.abs() * density;
        let mut inertia = Vec3::ZERO;
        for (v, p) in tetra {
            let dm = v.abs() * density;
            let q = p - center;
            inertia += Vec3 {
                x: dm * (q.y * q.y + q.z * q.z),
                y: dm * (q.x * q.x + q.z * q.z),
                z: dm * (q.x * q.x + q.y * q.y),
            };
        }
        Ok(MassProperties3 {
            mass,
            center,
            inertia_diagonal: inertia,
        })
    }
    /// Point containment for closed meshes using an odd/even +X ray rule. Boundary points within tolerance are inside.
    #[must_use]
    pub fn contains(&self, p: Vec3) -> bool {
        if (self.closest_point(p) - p).length_squared() <= ABS_EPSILON * ABS_EPSILON {
            return true;
        }
        let ray = Ray3::new(p, Vec3::X).expect("unit direction");
        let mut hits: Vec<Real> = self
            .indices
            .iter()
            .filter_map(|f| {
                Triangle3::new(
                    self.vertices[f[0] as usize],
                    self.vertices[f[1] as usize],
                    self.vertices[f[2] as usize],
                )
                .ok()?
                .ray_intersection(ray)
                .map(|x| x.0)
            })
            .collect();
        hits.sort_by(Real::total_cmp);
        hits.dedup_by(|a, b| (*a - *b).abs() <= ABS_EPSILON * 4.0);
        hits.len() % 2 == 1
    }
}
/// A transformed 3D sphere child for compound mass/bounds/query aggregation.
#[derive(Clone, Debug, PartialEq)]
pub struct CompoundSphere3 {
    /// Shape.
    pub shape: Sphere3,
    /// Child transform.
    pub transform: Transform3,
    /// Density.
    pub density: Real,
}
/// A validated 3D compound with parallel-axis mass aggregation.
#[derive(Clone, Debug, PartialEq)]
pub struct Compound3 {
    children: Vec<CompoundSphere3>,
}
impl Compound3 {
    /// Validates a nonempty set of finite children.
    pub fn new(children: Vec<CompoundSphere3>) -> Result<Self, GeometryError> {
        if children.is_empty()
            || children
                .iter()
                .any(|c| !positive(c.density) || !c.transform.translation.is_finite())
        {
            Err(GeometryError::InvalidInput)
        } else {
            Ok(Self { children })
        }
    }
    /// Combined mass properties.
    pub fn mass_properties(&self) -> Result<MassProperties3, GeometryError> {
        let p = self
            .children
            .iter()
            .map(|c| {
                c.shape
                    .mass_properties(c.density)
                    .map(|x| (x, c.transform.translation))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let mass = p.iter().map(|x| x.0.mass).sum();
        let center = p.iter().fold(Vec3::ZERO, |a, x| a + x.1 * x.0.mass) / mass;
        let mut i = Vec3::ZERO;
        for (x, pos) in p {
            let q = pos - center;
            i += x.inertia_diagonal
                + Vec3 {
                    x: x.mass * (q.y * q.y + q.z * q.z),
                    y: x.mass * (q.x * q.x + q.z * q.z),
                    z: x.mass * (q.x * q.x + q.y * q.y),
                };
        }
        Ok(MassProperties3 {
            mass,
            center,
            inertia_diagonal: i,
        })
    }
    /// Bounds.
    #[must_use]
    pub fn aabb(&self) -> Aabb3 {
        let mut min = Vec3 {
            x: Real::INFINITY,
            y: Real::INFINITY,
            z: Real::INFINITY,
        };
        let mut max = -min;
        for c in &self.children {
            let r = c.shape.radius();
            let h = Vec3 { x: r, y: r, z: r };
            let a = c.transform.translation - h;
            let b = c.transform.translation + h;
            min = Vec3 {
                x: min.x.min(a.x),
                y: min.y.min(a.y),
                z: min.z.min(a.z),
            };
            max = Vec3 {
                x: max.x.max(b.x),
                y: max.y.max(b.y),
                z: max.z.max(b.z),
            };
        }
        Aabb3::new(min, max).expect("compound")
    }
    /// Containment in any child.
    #[must_use]
    pub fn contains(&self, p: Vec3) -> bool {
        self.children
            .iter()
            .any(|c| c.shape.contains(c.transform.inverse().transform_point(p)))
    }
    /// Closest point over children.
    #[must_use]
    pub fn closest_point(&self, p: Vec3) -> Vec3 {
        self.children
            .iter()
            .map(|c| {
                c.transform.transform_point(
                    c.shape
                        .closest_point(c.transform.inverse().transform_point(p)),
                )
            })
            .min_by(|a, b| {
                (p - *a)
                    .length_squared()
                    .total_cmp(&(p - *b).length_squared())
            })
            .expect("children")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sphere_circle_complete_contract() {
        let c = Circle2::new(2.0).unwrap();
        assert!(c.contains(Vec2::ZERO));
        assert_eq!(
            c.ray_t(Ray2::new(Vec2 { x: -3.0, y: 0.0 }, Vec2::X).unwrap()),
            Some(1.0)
        );
        assert!((c.mass_properties(1.0).unwrap().mass - 4.0 * PI).abs() < 1.0e-5);
        let s = Sphere3::new(2.0).unwrap();
        assert_eq!(
            s.support(Vec3::X),
            Vec3 {
                x: 2.0,
                y: 0.0,
                z: 0.0
            }
        );
        assert!(s.scaled(-1.0).is_err());
    }
    #[test]
    fn edge_and_heightfield_queries() {
        let e = Edge2::new(Vec2::ZERO, Vec2 { x: 2.0, y: 0.0 }).unwrap();
        assert_eq!(
            e.closest_point(Vec2 { x: 3.0, y: 1.0 }),
            Vec2 { x: 2.0, y: 0.0 }
        );
        let h = Heightfield2::new(vec![0.0, 1.0, 0.0], 1.0).unwrap();
        assert_eq!(h.segment_count(), 2);
        assert!(
            h.ray_t(Ray2::new(Vec2 { x: 0.5, y: 2.0 }, Vec2 { x: 0.0, y: -1.0 }).unwrap())
                .is_some()
        );
    }
    #[test]
    fn tetra_hull_support_contains_mass() {
        let h = ConvexHull3::build(&[
            Vec3::ZERO,
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ])
        .unwrap();
        assert!(h.contains(Vec3 {
            x: 0.1,
            y: 0.1,
            z: 0.1
        }));
        assert_eq!(h.support(Vec3::X), Vec3::X);
        let p = h.mass_properties(6.0).unwrap();
        assert!((p.mass - 1.0).abs() < 1.0e-5);
        assert!(
            h.ray_t(
                Ray3::new(
                    Vec3 {
                        x: 0.1,
                        y: 0.1,
                        z: 2.0
                    },
                    Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: -1.0
                    }
                )
                .unwrap()
            )
            .is_some()
        );
    }
    #[test]
    fn mesh_builds_bvh_and_queries() {
        let m = TriangleMesh::new(
            vec![
                Vec3::ZERO,
                Vec3::X,
                Vec3::Y,
                Vec3 {
                    x: 1.0,
                    y: 1.0,
                    z: 0.0,
                },
            ],
            vec![[0, 1, 2], [1, 3, 2]],
        )
        .unwrap();
        assert!(!m.bvh().is_empty());
        assert_eq!(
            m.closest_point(Vec3 {
                x: 0.5,
                y: 0.5,
                z: 1.0
            })
            .z,
            0.0
        );
        assert!(
            m.ray_t(
                Ray3::new(
                    Vec3 {
                        x: 0.5,
                        y: 0.5,
                        z: 1.0
                    },
                    Vec3 {
                        x: 0.0,
                        y: 0.0,
                        z: -1.0
                    }
                )
                .unwrap()
            )
            .is_some()
        );
    }
    #[test]
    fn compound_parallel_axis() {
        let c = Circle2::new(1.0).unwrap();
        let x = Compound2::new(vec![
            CompoundCircle2 {
                shape: c,
                transform: Transform2::new(
                    Vec2 { x: -2.0, y: 0.0 },
                    auralite_math::Rot2::identity(),
                )
                .unwrap(),
                density: 1.0,
            },
            CompoundCircle2 {
                shape: c,
                transform: Transform2::new(
                    Vec2 { x: 2.0, y: 0.0 },
                    auralite_math::Rot2::identity(),
                )
                .unwrap(),
                density: 1.0,
            },
        ])
        .unwrap();
        let p = x.mass_properties().unwrap();
        assert_eq!(p.center, Vec2::ZERO);
        assert!(p.inertia > p.mass * 4.0);
    }
    #[test]
    fn rejects_malformed_assets() {
        assert!(ConvexHull3::build(&[Vec3::ZERO, Vec3::X, Vec3::Y]).is_err());
        assert!(TriangleMesh::new(vec![Vec3::ZERO], vec![[0, 1, 2]]).is_err());
        assert!(Heightfield3::new(2, 2, vec![0.0; 3], 1.0).is_err());
    }
    #[test]
    fn capsule_and_polygon_full_contract() {
        let c = crate::Capsule3::new(0.5, 1.0).unwrap();
        assert!(c.contains(Vec3::ZERO));
        assert!(
            (c.closest_point(Vec3 {
                x: 2.0,
                y: 0.0,
                z: 0.0
            })
            .x - 0.5)
                .abs()
                < 1.0e-6
        );
        assert!(
            c.ray_t(
                Ray3::new(
                    Vec3 {
                        x: -2.0,
                        y: 0.0,
                        z: 0.0
                    },
                    Vec3::X
                )
                .unwrap()
            )
            .is_some()
        );
        assert!(c.mass_properties(2.0).unwrap().mass > 0.0);
        let p = crate::ConvexPolygon::new(vec![
            Vec2 { x: -1.0, y: -1.0 },
            Vec2 { x: 1.0, y: -1.0 },
            Vec2 { x: 1.0, y: 1.0 },
            Vec2 { x: -1.0, y: 1.0 },
        ])
        .unwrap();
        assert_eq!(
            p.closest_point(Vec2 { x: 3.0, y: 0.0 }),
            Vec2 { x: 1.0, y: 0.0 }
        );
        assert!(
            p.ray_t(Ray2::new(Vec2 { x: -2.0, y: 0.0 }, Vec2::X).unwrap())
                .is_some()
        );
        assert!(p.scaled(0.0).is_err());
    }
    #[test]
    fn closed_mesh_mass_and_compound3() {
        let v = vec![
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3 {
                x: 1.0,
                y: 1.0,
                z: 0.0,
            },
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 1.0,
            },
            Vec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 1.0,
            },
        ];
        let f = vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [3, 7, 6],
            [3, 6, 2],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ];
        let m = TriangleMesh::new(v, f).unwrap();
        let p = m.mass_properties(2.0).unwrap();
        assert!((p.mass - 2.0).abs() < 1.0e-5);
        assert!(m.contains(Vec3 {
            x: 0.5,
            y: 0.5,
            z: 0.5
        }));
        let s = Sphere3::new(1.0).unwrap();
        let c = Compound3::new(vec![CompoundSphere3 {
            shape: s,
            transform: Transform3::identity(),
            density: 1.0,
        }])
        .unwrap();
        assert!(c.contains(Vec3::ZERO));
        assert!(c.mass_properties().unwrap().mass > 4.0);
    }
    #[test]
    fn randomized_support_equals_vertex_reference() {
        let seed = 0x5A17_u64;
        let mut x = seed;
        let hull = ConvexHull3::build(&[
            Vec3::ZERO,
            Vec3::X,
            Vec3::Y,
            Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        ])
        .unwrap();
        for _ in 0..10_000 {
            x ^= x >> 12;
            x ^= x << 25;
            x ^= x >> 27;
            let a = (x >> 32) as Real;
            x ^= x >> 12;
            x ^= x << 25;
            x ^= x >> 27;
            let b = (x >> 32) as Real;
            let d = Vec3 {
                x: a.sin(),
                y: a.cos(),
                z: b.sin(),
            };
            let s = hull.support(d);
            assert!(
                hull.vertices().iter().all(|v| s.dot(d) >= v.dot(d)),
                "seed={seed:#x}"
            );
        }
    }
    #[test]
    fn mesh_bvh_ray_and_closest_agree_with_bruteforce() {
        // Build a larger mesh and verify BVH-accelerated queries match brute force
        let mut v = Vec::new();
        for x in 0..5 {
            for z in 0..5 {
                let y = ((x as Real).sin() * (z as Real).cos() * 0.5).max(0.0);
                v.push(Vec3 {
                    x: x as Real,
                    y,
                    z: z as Real,
                });
            }
        }
        let mut indices = Vec::new();
        for r in 0..4 {
            for c in 0..4 {
                let a = (r * 5 + c) as u32;
                let b = a + 1;
                let d = ((r + 1) * 5 + c) as u32;
                let e = d + 1;
                indices.push([a, d, b]);
                indices.push([b, d, e]);
            }
        }
        let m = TriangleMesh::new(v, indices).unwrap();
        // Test BVH ray against brute force
        let ray = Ray3::new(
            Vec3 {
                x: 2.5,
                y: 5.0,
                z: 2.5,
            },
            Vec3 {
                x: 0.0,
                y: -1.0,
                z: 0.0,
            },
        )
        .unwrap();
        let bvh_hit = m.ray_t_bvh(ray);
        let brute_hit = m.ray_t(ray);
        assert_eq!(bvh_hit, brute_hit, "BVH ray should match brute force");

        // Test BVH closest point against brute force
        let pt = Vec3 {
            x: 2.5,
            y: 5.0,
            z: 2.5,
        };
        let bvh_closest = m.closest_point_bvh(pt);
        let brute_closest = m.closest_point(pt);
        let diff = (bvh_closest - brute_closest).length();
        assert!(
            diff < 1.0e-6,
            "BVH closest should match brute force, diff={diff}"
        );
    }
    #[test]
    fn hull_cube_mass_regression() {
        let mut v = Vec::new();
        for x in [-1.0, 1.0] {
            for y in [-1.0, 1.0] {
                for z in [-1.0, 1.0] {
                    v.push(Vec3 { x, y, z });
                }
            }
        }
        let h = ConvexHull3::build(&v).unwrap();
        let p = h.mass_properties(1.0).unwrap();
        assert!(
            (p.mass - 8.0).abs() < 1.0e-4,
            "mass={} faces={}",
            p.mass,
            h.faces().len()
        );
    }
}
