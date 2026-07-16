//! Deterministic balanced dynamic AABB trees with fat, velocity-predicted leaves.
use auralite_math::{ABS_EPSILON, Aabb2, Aabb3, Ray2, Ray3, Real, Vec2, Vec3};
fn union2(a: Aabb2, b: Aabb2) -> Aabb2 {
    Aabb2::new(
        Vec2 {
            x: a.min.x.min(b.min.x),
            y: a.min.y.min(b.min.y),
        },
        Vec2 {
            x: a.max.x.max(b.max.x),
            y: a.max.y.max(b.max.y),
        },
    )
    .expect("finite")
}
fn union3(a: Aabb3, b: Aabb3) -> Aabb3 {
    Aabb3::new(
        Vec3 {
            x: a.min.x.min(b.min.x),
            y: a.min.y.min(b.min.y),
            z: a.min.z.min(b.min.z),
        },
        Vec3 {
            x: a.max.x.max(b.max.x),
            y: a.max.y.max(b.max.y),
            z: a.max.z.max(b.max.z),
        },
    )
    .expect("finite")
}
fn ray_aabb2(r: Ray2, a: Aabb2, max: Real) -> bool {
    let mut lo = 0.0f32 as Real;
    let mut hi = max;
    for (o, d, mn, mx) in [
        (r.origin.x, r.direction.x, a.min.x, a.max.x),
        (r.origin.y, r.direction.y, a.min.y, a.max.y),
    ] {
        if d.abs() < 1.0e-8 {
            if o < mn || o > mx {
                return false;
            }
        } else {
            let mut x = (mn - o) / d;
            let mut y = (mx - o) / d;
            if x > y {
                core::mem::swap(&mut x, &mut y);
            }
            lo = lo.max(x);
            hi = hi.min(y);
            if lo > hi {
                return false;
            }
        }
    }
    true
}
fn ray_aabb3(r: Ray3, a: Aabb3, max: Real) -> bool {
    let mut lo = 0.0f32 as Real;
    let mut hi = max;
    for (o, d, mn, mx) in [
        (r.origin.x, r.direction.x, a.min.x, a.max.x),
        (r.origin.y, r.direction.y, a.min.y, a.max.y),
        (r.origin.z, r.direction.z, a.min.z, a.max.z),
    ] {
        if d.abs() < 1.0e-8 {
            if o < mn || o > mx {
                return false;
            }
        } else {
            let mut x = (mn - o) / d;
            let mut y = (mx - o) / d;
            if x > y {
                core::mem::swap(&mut x, &mut y);
            }
            lo = lo.max(x);
            hi = hi.min(y);
            if lo > hi {
                return false;
            }
        }
    }
    true
}
#[derive(Clone, Debug)]
struct Node2 {
    aabb: Aabb2,
    kind: Kind,
}
#[derive(Clone, Debug)]
struct Node3 {
    aabb: Aabb3,
    kind: Kind,
}
#[derive(Clone, Debug)]
enum Kind {
    Leaf(u64),
    Branch(usize, usize),
}
/// Dynamic 2D AABB tree. Mutations rebuild a height-balanced deterministic hierarchy from stable-ID ordered fat leaves.
#[derive(Default, Clone, Debug)]
pub struct DynamicTree2 {
    leaves: Vec<(u64, Aabb2)>,
    nodes: Vec<Node2>,
    root: Option<usize>,
    margin: Real,
    prediction: Real,
}
impl DynamicTree2 {
    /// Creates a tree with nonnegative fixed margin and velocity prediction horizon.
    pub fn new(margin: Real, prediction: Real) -> Result<Self, TreeError> {
        if margin.is_finite() && margin >= 0.0 && prediction.is_finite() && prediction >= 0.0 {
            Ok(Self {
                leaves: Vec::new(),
                nodes: Vec::new(),
                root: None,
                margin,
                prediction,
            })
        } else {
            Err(TreeError)
        }
    }
    fn fatten(&self, a: Aabb2, v: Vec2) -> Aabb2 {
        let mut min = a.min
            - Vec2 {
                x: self.margin,
                y: self.margin,
            };
        let mut max = a.max
            + Vec2 {
                x: self.margin,
                y: self.margin,
            };
        let d = v * self.prediction;
        if d.x < 0.0 {
            min.x += d.x
        } else {
            max.x += d.x
        }
        if d.y < 0.0 {
            min.y += d.y
        } else {
            max.y += d.y
        }
        Aabb2::new(min, max).expect("finite")
    }
    /// Adds or updates a stable ID and deterministically rebalances.
    pub fn update(&mut self, id: u64, aabb: Aabb2, velocity: Vec2) {
        let fat = self.fatten(aabb, velocity);
        if let Some(x) = self.leaves.iter_mut().find(|x| x.0 == id) {
            x.1 = fat
        } else {
            self.leaves.push((id, fat));
            self.leaves.sort_unstable_by_key(|x| x.0);
        }
        self.rebuild();
    }
    /// Removes an ID.
    pub fn remove(&mut self, id: u64) -> bool {
        let n = self.leaves.len();
        self.leaves.retain(|x| x.0 != id);
        let removed = n != self.leaves.len();
        if removed {
            self.rebuild();
        }
        removed
    }
    fn rebuild(&mut self) {
        self.nodes.clear();
        if self.leaves.is_empty() {
            self.root = None;
            return;
        }
        fn build(nodes: &mut Vec<Node2>, leaves: &[(u64, Aabb2)]) -> usize {
            if leaves.len() == 1 {
                let i = nodes.len();
                nodes.push(Node2 {
                    aabb: leaves[0].1,
                    kind: Kind::Leaf(leaves[0].0),
                });
                return i;
            }
            let m = leaves.len() / 2;
            let l = build(nodes, &leaves[..m]);
            let r = build(nodes, &leaves[m..]);
            let i = nodes.len();
            nodes.push(Node2 {
                aabb: union2(nodes[l].aabb, nodes[r].aabb),
                kind: Kind::Branch(l, r),
            });
            i
        }
        self.root = Some(build(&mut self.nodes, &self.leaves));
    }
    /// Queries overlapping leaf IDs in stable order.
    #[must_use]
    pub fn query(&self, a: Aabb2) -> Vec<u64> {
        let mut out = Vec::new();
        if let Some(root) = self.root {
            let mut stack = vec![root];
            while let Some(i) = stack.pop() {
                let n = &self.nodes[i];
                if !n.aabb.overlaps(a) {
                    continue;
                }
                match n.kind {
                    Kind::Leaf(id) => out.push(id),
                    Kind::Branch(l, r) => {
                        stack.push(r);
                        stack.push(l);
                    }
                }
            }
        }
        out.sort_unstable();
        out
    }
    /// Ray-casts tree bounds and returns candidate IDs in stable order.
    #[must_use]
    pub fn ray_cast(&self, r: Ray2, max: Real) -> Vec<u64> {
        let mut out = Vec::new();
        if let Some(root) = self.root {
            let mut s = vec![root];
            while let Some(i) = s.pop() {
                let n = &self.nodes[i];
                if !ray_aabb2(r, n.aabb, max) {
                    continue;
                }
                match n.kind {
                    Kind::Leaf(id) => out.push(id),
                    Kind::Branch(l, q) => {
                        s.push(q);
                        s.push(l)
                    }
                }
            }
        }
        out.sort_unstable();
        out
    }
    /// Canonical overlapping pairs.
    #[must_use]
    pub fn pairs(&self) -> Vec<(u64, u64)> {
        let mut out = Vec::new();
        for (i, (a, aa)) in self.leaves.iter().enumerate() {
            for (b, bb) in &self.leaves[i + 1..] {
                if aa.overlaps(*bb) {
                    out.push((*a, *b));
                }
            }
        }
        out
    }
    /// Tree height; balanced rebuild guarantees ceil(log2(n))+1 or less.
    #[must_use]
    pub fn height(&self) -> usize {
        fn h(n: &[Node2], i: usize) -> usize {
            match n[i].kind {
                Kind::Leaf(_) => 1,
                Kind::Branch(a, b) => 1 + h(n, a).max(h(n, b)),
            }
        }
        self.root.map_or(0, |r| h(&self.nodes, r))
    }
    /// AABB query that also returns candidate IDs and their AABBs for distance evaluation.
    #[must_use]
    pub fn query_with_aabbs(&self, a: Aabb2) -> Vec<(u64, Aabb2)> {
        let ids = self.query(a);
        ids.into_iter()
            .filter_map(|id| self.leaves.iter().find(|x| x.0 == id).map(|x| (id, x.1)))
            .collect()
    }
    /// Shape cast: sweep this AABB through `translation` and return potential overlaps at the end position.
    /// Uses Minkowski sum approach: expand the query AABB by the swept AABB's half-extents.
    #[must_use]
    pub fn shape_cast(&self, shape_aabb: Aabb2, translation: Vec2) -> Vec<u64> {
        if translation.length_squared() <= ABS_EPSILON * ABS_EPSILON {
            return self.query(shape_aabb);
        }
        // Swept AABB = union of start and end AABB
        let end_aabb = Aabb2::new(shape_aabb.min + translation, shape_aabb.max + translation)
            .unwrap_or(shape_aabb);
        let swept = Aabb2::new(
            Vec2 {
                x: shape_aabb.min.x.min(end_aabb.min.x),
                y: shape_aabb.min.y.min(end_aabb.min.y),
            },
            Vec2 {
                x: shape_aabb.max.x.max(end_aabb.max.x),
                y: shape_aabb.max.y.max(end_aabb.max.y),
            },
        )
        .unwrap_or(shape_aabb);
        self.query(swept)
    }
}
/// Dynamic 3D AABB tree with the same deterministic policy.
#[derive(Default, Clone, Debug)]
pub struct DynamicTree3 {
    leaves: Vec<(u64, Aabb3)>,
    nodes: Vec<Node3>,
    root: Option<usize>,
    margin: Real,
    prediction: Real,
}
impl DynamicTree3 {
    /// Creates a tree.
    pub fn new(margin: Real, prediction: Real) -> Result<Self, TreeError> {
        if margin.is_finite() && margin >= 0.0 && prediction.is_finite() && prediction >= 0.0 {
            Ok(Self {
                leaves: Vec::new(),
                nodes: Vec::new(),
                root: None,
                margin,
                prediction,
            })
        } else {
            Err(TreeError)
        }
    }
    fn fatten(&self, a: Aabb3, v: Vec3) -> Aabb3 {
        let h = Vec3 {
            x: self.margin,
            y: self.margin,
            z: self.margin,
        };
        let mut min = a.min - h;
        let mut max = a.max + h;
        let d = v * self.prediction;
        for (x, mn, mx) in [
            (d.x, &mut min.x, &mut max.x),
            (d.y, &mut min.y, &mut max.y),
            (d.z, &mut min.z, &mut max.z),
        ] {
            if x < 0.0 { *mn += x } else { *mx += x }
        }
        Aabb3::new(min, max).expect("finite")
    }
    /// Adds/updates.
    pub fn update(&mut self, id: u64, a: Aabb3, v: Vec3) {
        let f = self.fatten(a, v);
        if let Some(x) = self.leaves.iter_mut().find(|x| x.0 == id) {
            x.1 = f
        } else {
            self.leaves.push((id, f));
            self.leaves.sort_unstable_by_key(|x| x.0)
        }
        self.rebuild()
    }
    /// Removes.
    pub fn remove(&mut self, id: u64) -> bool {
        let n = self.leaves.len();
        self.leaves.retain(|x| x.0 != id);
        if n != self.leaves.len() {
            self.rebuild();
            true
        } else {
            false
        }
    }
    fn rebuild(&mut self) {
        self.nodes.clear();
        fn b(n: &mut Vec<Node3>, l: &[(u64, Aabb3)]) -> usize {
            if l.len() == 1 {
                let i = n.len();
                n.push(Node3 {
                    aabb: l[0].1,
                    kind: Kind::Leaf(l[0].0),
                });
                i
            } else {
                let m = l.len() / 2;
                let a = b(n, &l[..m]);
                let c = b(n, &l[m..]);
                let i = n.len();
                n.push(Node3 {
                    aabb: union3(n[a].aabb, n[c].aabb),
                    kind: Kind::Branch(a, c),
                });
                i
            }
        }
        self.root = (!self.leaves.is_empty()).then(|| b(&mut self.nodes, &self.leaves));
    }
    /// Overlap query.
    #[must_use]
    pub fn query(&self, a: Aabb3) -> Vec<u64> {
        let mut o = Vec::new();
        if let Some(r) = self.root {
            let mut s = vec![r];
            while let Some(i) = s.pop() {
                let n = &self.nodes[i];
                if !n.aabb.overlaps(a) {
                    continue;
                }
                match n.kind {
                    Kind::Leaf(id) => o.push(id),
                    Kind::Branch(a, b) => {
                        s.push(b);
                        s.push(a)
                    }
                }
            }
        }
        o.sort_unstable();
        o
    }
    /// Ray candidates.
    #[must_use]
    pub fn ray_cast(&self, r: Ray3, max: Real) -> Vec<u64> {
        let mut o = Vec::new();
        if let Some(i) = self.root {
            let mut s = vec![i];
            while let Some(i) = s.pop() {
                let n = &self.nodes[i];
                if !ray_aabb3(r, n.aabb, max) {
                    continue;
                }
                match n.kind {
                    Kind::Leaf(id) => o.push(id),
                    Kind::Branch(a, b) => {
                        s.push(b);
                        s.push(a)
                    }
                }
            }
        }
        o.sort_unstable();
        o
    }
    /// Pairs.
    #[must_use]
    pub fn pairs(&self) -> Vec<(u64, u64)> {
        let mut o = Vec::new();
        for (i, (a, x)) in self.leaves.iter().enumerate() {
            for (b, y) in &self.leaves[i + 1..] {
                if x.overlaps(*y) {
                    o.push((*a, *b))
                }
            }
        }
        o
    }
    /// Height.
    #[must_use]
    pub fn height(&self) -> usize {
        fn h(n: &[Node3], i: usize) -> usize {
            match n[i].kind {
                Kind::Leaf(_) => 1,
                Kind::Branch(a, b) => 1 + h(n, a).max(h(n, b)),
            }
        }
        self.root.map_or(0, |r| h(&self.nodes, r))
    }
    /// AABB query that also returns candidate IDs and their AABBs.
    #[must_use]
    pub fn query_with_aabbs(&self, a: Aabb3) -> Vec<(u64, Aabb3)> {
        let ids = self.query(a);
        ids.into_iter()
            .filter_map(|id| self.leaves.iter().find(|x| x.0 == id).map(|x| (id, x.1)))
            .collect()
    }
    /// Shape cast: sweep this AABB through `translation` and return potential overlaps.
    #[must_use]
    pub fn shape_cast(&self, shape_aabb: Aabb3, translation: Vec3) -> Vec<u64> {
        if translation.length_squared() <= ABS_EPSILON * ABS_EPSILON {
            return self.query(shape_aabb);
        }
        let end_aabb = Aabb3::new(shape_aabb.min + translation, shape_aabb.max + translation)
            .unwrap_or(shape_aabb);
        let swept = Aabb3::new(
            Vec3 {
                x: shape_aabb.min.x.min(end_aabb.min.x),
                y: shape_aabb.min.y.min(end_aabb.min.y),
                z: shape_aabb.min.z.min(end_aabb.min.z),
            },
            Vec3 {
                x: shape_aabb.max.x.max(end_aabb.max.x),
                y: shape_aabb.max.y.max(end_aabb.max.y),
                z: shape_aabb.max.z.max(end_aabb.max.z),
            },
        )
        .unwrap_or(shape_aabb);
        self.query(swept)
    }
}
/// Invalid tree configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TreeError;
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn differential_hundred_thousand_checks() {
        let mut t = DynamicTree2::new(0.0, 0.0).unwrap();
        let mut seed = 7u64;
        let mut boxes = Vec::new();
        for id in 0..448u64 {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            let x = (seed % 1000) as Real / 10.0;
            let a = Aabb2::new(
                Vec2 { x, y: x * 0.37 },
                Vec2 {
                    x: x + 1.0,
                    y: x * 0.37 + 1.0,
                },
            )
            .unwrap();
            boxes.push((id, a));
            t.update(id, a, Vec2::ZERO);
        }
        let pairs = t.pairs();
        let brute: Vec<_> = boxes
            .iter()
            .enumerate()
            .flat_map(|(i, a)| {
                boxes[i + 1..]
                    .iter()
                    .filter_map(move |b| a.1.overlaps(b.1).then_some((a.0, b.0)))
            })
            .collect();
        assert_eq!(pairs, brute);
        assert_eq!(boxes.len() * (boxes.len() - 1) / 2, 100_128);
        assert!(t.height() <= 10);
    }
    #[test]
    fn insertion_order_does_not_change_pairs() {
        let a = Aabb3::new(
            Vec3::ZERO,
            Vec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
        )
        .unwrap();
        let mut x = DynamicTree3::new(0.1, 0.2).unwrap();
        let mut y = DynamicTree3::new(0.1, 0.2).unwrap();
        for id in [9, 1, 7, 2] {
            x.update(id, a, Vec3::ZERO)
        }
        for id in [2, 7, 1, 9] {
            y.update(id, a, Vec3::ZERO)
        }
        assert_eq!(x.pairs(), y.pairs());
        assert_eq!(x.query(a), vec![1, 2, 7, 9]);
    }
    #[test]
    fn dynamic_tree_shape_cast_candidates() {
        let mut t = DynamicTree2::new(0.0, 0.0).unwrap();
        t.update(
            1,
            Aabb2::new(Vec2 { x: 0.0, y: 0.0 }, Vec2 { x: 1.0, y: 1.0 }).unwrap(),
            Vec2::ZERO,
        );
        t.update(
            2,
            Aabb2::new(Vec2 { x: 10.0, y: 0.0 }, Vec2 { x: 11.0, y: 1.0 }).unwrap(),
            Vec2::ZERO,
        );
        let shape = Aabb2::new(Vec2::ZERO, Vec2 { x: 1.0, y: 1.0 }).unwrap();
        // Shape cast from origin to x=10 should find id 2
        let hits = t.shape_cast(shape, Vec2 { x: 12.0, y: 0.0 });
        assert!(hits.contains(&2), "shape cast should find target at x=10");
    }
    #[test]
    fn dynamic_tree_query_with_aabbs() {
        let mut t = DynamicTree2::new(0.1, 0.2).unwrap();
        let a = Aabb2::new(Vec2::ZERO, Vec2 { x: 1.0, y: 1.0 }).unwrap();
        t.update(42, a, Vec2::ZERO);
        let q = t.query_with_aabbs(a);
        assert_eq!(q.len(), 1);
        assert_eq!(q[0].0, 42);
    }
}
