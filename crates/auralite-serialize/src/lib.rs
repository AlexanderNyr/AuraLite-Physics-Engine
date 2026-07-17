//! Versioned, quota-bounded binary envelope with typed serialization for
//! all AuraLite state types: worlds, bodies, shapes, joints, soft-bodies,
//! particles, snapshots, RNG seeds.
//!
//! # Format
//! All multi-byte integers are little-endian. Payloads are tagged with a
//! 1-byte type tag followed by a 4-byte length, then the payload data.
#![forbid(unsafe_code)]
#![allow(missing_docs, dead_code)]

use auralite_core::{StableId, hash_bytes};
use auralite_dynamics::joints::{JointId, JointType2, JointType3};
use auralite_dynamics::{
    Body2, Body3, BodyHandle2, BodyHandle3, BodyType, Collider2, Collider3, ColliderShape2,
    ColliderShape3, Joint2, Joint3, JointConfig2, JointConfig3, JointLimits, JointMotor, Material,
    World2, World3,
};
use auralite_math::{Quat, Real, Rot2, Vec2, Vec3};

// ─── Envelope ────────────────────────────────────────────────────────────────

pub const MAGIC: [u8; 4] = *b"AURA";
pub const VERSION: u16 = 2;
pub const MAX_PAYLOAD: usize = 64 * 1024 * 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Truncated,
    BadMagic,
    UnsupportedVersion,
    InvalidLength,
    TypedPayloadMismatch,
    UnsupportedTypeTag,
    ChecksumMismatch,
    InvalidEnumDiscriminant,
}

#[must_use]
pub fn encode(payload: &[u8]) -> Vec<u8> {
    let checksum = hash_bytes(payload);
    let mut out = Vec::with_capacity(18 + payload.len());
    out.extend_from_slice(&MAGIC);
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.extend_from_slice(&(payload.len() as u32).to_le_bytes());
    out.extend_from_slice(&checksum.to_le_bytes());
    out.extend_from_slice(payload);
    out
}

pub fn decode(input: &[u8], quota: usize) -> Result<&[u8], Error> {
    if input.len() < 18 {
        return Err(Error::Truncated);
    }
    if input[..4] != MAGIC {
        return Err(Error::BadMagic);
    }
    let version = u16::from_le_bytes([input[4], input[5]]);
    if version > VERSION {
        return Err(Error::UnsupportedVersion);
    }
    let n = u32::from_le_bytes([input[6], input[7], input[8], input[9]]) as usize;
    if n > quota || n > MAX_PAYLOAD || n + 18 > input.len() {
        return Err(Error::InvalidLength);
    }
    let stored_checksum = u64::from_le_bytes([
        input[10], input[11], input[12], input[13], input[14], input[15], input[16], input[17],
    ]);
    let payload = &input[18..18 + n];
    let computed = hash_bytes(payload);
    if stored_checksum != computed {
        return Err(Error::ChecksumMismatch);
    }
    Ok(payload)
}

// ─── Type Tags ───────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TypeTag {
    World2State = 1,
    World3State = 2,
    JointConfig2 = 3,
    Joint2State = 4,
    Snapshot2 = 5,
    Snapshot3 = 6,
    Body2 = 7,
    Body3 = 8,
    Collider2 = 9,
    Collider3 = 10,
    SoftBody = 11,
    ParticleStorage = 12,
    RngState = 13,
    ForceField = 14,
    CombinedSnapshot2 = 15,
    CombinedSnapshot3 = 16,
    Joint3State = 17,
    JointConfig3 = 18,
}

impl TypeTag {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            1 => Self::World2State,
            2 => Self::World3State,
            3 => Self::JointConfig2,
            4 => Self::Joint2State,
            5 => Self::Snapshot2,
            6 => Self::Snapshot3,
            7 => Self::Body2,
            8 => Self::Body3,
            9 => Self::Collider2,
            10 => Self::Collider3,
            11 => Self::SoftBody,
            12 => Self::ParticleStorage,
            13 => Self::RngState,
            14 => Self::ForceField,
            15 => Self::CombinedSnapshot2,
            16 => Self::CombinedSnapshot3,
            17 => Self::Joint3State,
            18 => Self::JointConfig3,
            _ => return None,
        })
    }
}

// ─── Serialization helpers ───────────────────────────────────────────────────

fn write_u8(buf: &mut Vec<u8>, v: u8) {
    buf.push(v);
}
fn write_u32(buf: &mut Vec<u8>, v: u32) {
    buf.extend_from_slice(&v.to_le_bytes());
}
fn write_u64(buf: &mut Vec<u8>, v: u64) {
    buf.extend_from_slice(&v.to_le_bytes());
}
fn write_f32(buf: &mut Vec<u8>, v: Real) {
    buf.extend_from_slice(&v.to_bits().to_le_bytes());
}
fn write_bool(buf: &mut Vec<u8>, v: bool) {
    buf.push(u8::from(v));
}

fn write_vec2(buf: &mut Vec<u8>, v: Vec2) {
    write_f32(buf, v.x);
    write_f32(buf, v.y);
}
fn write_vec3(buf: &mut Vec<u8>, v: Vec3) {
    write_f32(buf, v.x);
    write_f32(buf, v.y);
    write_f32(buf, v.z);
}

fn read_u8(data: &[u8], pos: &mut usize) -> Result<u8, Error> {
    if *pos >= data.len() {
        return Err(Error::Truncated);
    }
    let v = data[*pos];
    *pos += 1;
    Ok(v)
}
fn read_u32(data: &[u8], pos: &mut usize) -> Result<u32, Error> {
    if *pos + 4 > data.len() {
        return Err(Error::Truncated);
    }
    let v = u32::from_le_bytes([data[*pos], data[*pos + 1], data[*pos + 2], data[*pos + 3]]);
    *pos += 4;
    Ok(v)
}
fn read_u64(data: &[u8], pos: &mut usize) -> Result<u64, Error> {
    if *pos + 8 > data.len() {
        return Err(Error::Truncated);
    }
    let v = u64::from_le_bytes([
        data[*pos],
        data[*pos + 1],
        data[*pos + 2],
        data[*pos + 3],
        data[*pos + 4],
        data[*pos + 5],
        data[*pos + 6],
        data[*pos + 7],
    ]);
    *pos += 8;
    Ok(v)
}
fn read_f32(data: &[u8], pos: &mut usize) -> Result<Real, Error> {
    if *pos + 4 > data.len() {
        return Err(Error::Truncated);
    }
    let bits = u32::from_le_bytes([data[*pos], data[*pos + 1], data[*pos + 2], data[*pos + 3]]);
    *pos += 4;
    Ok(Real::from_bits(bits))
}
fn read_bool(data: &[u8], pos: &mut usize) -> Result<bool, Error> {
    let v = read_u8(data, pos)?;
    Ok(v != 0)
}
fn read_vec2(data: &[u8], pos: &mut usize) -> Result<Vec2, Error> {
    let x = read_f32(data, pos)?;
    let y = read_f32(data, pos)?;
    Ok(Vec2 { x, y })
}
fn read_vec3(data: &[u8], pos: &mut usize) -> Result<Vec3, Error> {
    let x = read_f32(data, pos)?;
    let y = read_f32(data, pos)?;
    let z = read_f32(data, pos)?;
    Ok(Vec3 { x, y, z })
}

fn write_typed_payload(tag: TypeTag, payload: &[u8]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(5 + payload.len());
    buf.push(tag as u8);
    write_u32(&mut buf, payload.len() as u32);
    buf.extend_from_slice(payload);
    buf
}

fn read_typed_payload<'a>(
    data: &'a [u8],
    pos: &mut usize,
    expected: TypeTag,
) -> Result<&'a [u8], Error> {
    let tag_byte = read_u8(data, pos)?;
    if TypeTag::from_u8(tag_byte) != Some(expected) {
        return Err(Error::TypedPayloadMismatch);
    }
    let len = read_u32(data, pos)? as usize;
    if *pos + len > data.len() {
        return Err(Error::Truncated);
    }
    let slice = &data[*pos..*pos + len];
    *pos += len;
    Ok(slice)
}

// ─── Collider2 serialization ─────────────────────────────────────────────────

pub fn serialize_collider2(c: &Collider2) -> Vec<u8> {
    let mut buf = Vec::new();
    // Body type discriminant + shape discriminant
    match &c.shape {
        ColliderShape2::Circle(circ) => {
            write_u8(&mut buf, 0);
            write_f32(&mut buf, circ.radius());
        }
        ColliderShape2::Box(bx) => {
            write_u8(&mut buf, 1);
            write_vec2(&mut buf, bx.half_extents());
        }
        ColliderShape2::Capsule(cap) => {
            write_u8(&mut buf, 2);
            write_f32(&mut buf, cap.radius);
            write_f32(&mut buf, cap.half_height);
        }
        ColliderShape2::ConvexPolygon(poly) => {
            write_u8(&mut buf, 3);
            write_u32(&mut buf, poly.vertices().len() as u32);
            for v in poly.vertices() {
                write_vec2(&mut buf, *v);
            }
        }
        ColliderShape2::Edge(edge) => {
            write_u8(&mut buf, 4);
            let (a, b) = edge.endpoints();
            write_vec2(&mut buf, a);
            write_vec2(&mut buf, b);
        }
    }
    write_vec2(&mut buf, c.offset);
    write_f32(&mut buf, c.material.restitution);
    write_f32(&mut buf, c.material.friction);
    write_f32(&mut buf, c.material.density);
    write_u64(&mut buf, c.filter.layers);
    write_u64(&mut buf, c.filter.mask);
    write_i32_as_u32(&mut buf, c.filter.group);
    write_bool(&mut buf, c.filter.sensor);
    write_typed_payload(TypeTag::Collider2, &buf)
}

fn write_i32_as_u32(buf: &mut Vec<u8>, v: i32) {
    write_u32(buf, v as u32);
}
fn read_u32_as_i32(data: &[u8], pos: &mut usize) -> Result<i32, Error> {
    let v = read_u32(data, pos)?;
    Ok(v as i32)
}

pub fn deserialize_collider2(data: &[u8]) -> Result<Collider2, Error> {
    let payload = read_typed_payload(data, &mut 0, TypeTag::Collider2)?;
    let mut pos = 0;
    let shape_disc = read_u8(payload, &mut pos)?;
    let shape = match shape_disc {
        0 => {
            let r = read_f32(payload, &mut pos)?;
            ColliderShape2::Circle(
                auralite_geometry::Circle2::new(r).map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        1 => {
            let h = read_vec2(payload, &mut pos)?;
            ColliderShape2::Box(
                auralite_geometry::Box2::new(h).map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        2 => {
            let r = read_f32(payload, &mut pos)?;
            let hh = read_f32(payload, &mut pos)?;
            ColliderShape2::Capsule(
                auralite_geometry::Capsule2::new(r, hh)
                    .map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        3 => {
            let n = read_u32(payload, &mut pos)? as usize;
            let mut vertices = Vec::with_capacity(n);
            for _ in 0..n {
                vertices.push(read_vec2(payload, &mut pos)?);
            }
            ColliderShape2::ConvexPolygon(
                auralite_geometry::ConvexPolygon::new(vertices)
                    .map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        4 => {
            let a = read_vec2(payload, &mut pos)?;
            let b = read_vec2(payload, &mut pos)?;
            ColliderShape2::Edge(
                auralite_geometry::Edge2::new(a, b).map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        _ => return Err(Error::UnsupportedTypeTag),
    };
    let offset = read_vec2(payload, &mut pos)?;
    let restitution = read_f32(payload, &mut pos)?;
    let friction = read_f32(payload, &mut pos)?;
    let density = read_f32(payload, &mut pos)?;
    let layers = read_u64(payload, &mut pos)?;
    let mask = read_u64(payload, &mut pos)?;
    let group = read_u32_as_i32(payload, &mut pos)?;
    let sensor = read_bool(payload, &mut pos)?;
    Ok(Collider2 {
        shape,
        offset,
        material: Material {
            restitution,
            friction,
            density,
        },
        filter: auralite_collision::CollisionFilter {
            layers,
            mask,
            group,
            sensor,
        },
    })
}

// ─── Body2 serialization ────────────────────────────────────────────────────

pub fn serialize_body2(b: &Body2) -> Vec<u8> {
    let mut buf = Vec::new();
    write_u64(&mut buf, b.id.0);
    write_u8(&mut buf, b.kind as u8);
    write_vec2(&mut buf, b.position);
    write_f32(&mut buf, b.rotation.c);
    write_f32(&mut buf, b.rotation.s);
    write_vec2(&mut buf, b.velocity);
    write_f32(&mut buf, b.angular_velocity);
    write_f32(&mut buf, b.inv_mass);
    write_f32(&mut buf, b.inv_inertia);
    write_u32(&mut buf, b.colliders.len() as u32);
    for c in &b.colliders {
        buf.extend_from_slice(&serialize_collider2_inner(c));
    }
    write_f32(&mut buf, b.restitution);
    write_f32(&mut buf, b.friction);
    write_bool(&mut buf, b.sleeping);
    write_vec2(&mut buf, b.force);
    write_f32(&mut buf, b.torque);
    write_f32(&mut buf, b.linear_damping);
    write_f32(&mut buf, b.angular_damping);
    write_u64(&mut buf, b.user_data);
    write_typed_payload(TypeTag::Body2, &buf)
}

fn serialize_collider2_inner(c: &Collider2) -> Vec<u8> {
    let mut buf = Vec::new();
    match &c.shape {
        ColliderShape2::Circle(circ) => {
            write_u8(&mut buf, 0);
            write_f32(&mut buf, circ.radius());
        }
        ColliderShape2::Box(bx) => {
            write_u8(&mut buf, 1);
            write_vec2(&mut buf, bx.half_extents());
        }
        ColliderShape2::Capsule(cap) => {
            write_u8(&mut buf, 2);
            write_f32(&mut buf, cap.radius);
            write_f32(&mut buf, cap.half_height);
        }
        ColliderShape2::ConvexPolygon(poly) => {
            write_u8(&mut buf, 3);
            write_u32(&mut buf, poly.vertices().len() as u32);
            for v in poly.vertices() {
                write_vec2(&mut buf, *v);
            }
        }
        ColliderShape2::Edge(edge) => {
            write_u8(&mut buf, 4);
            let (a, b) = edge.endpoints();
            write_vec2(&mut buf, a);
            write_vec2(&mut buf, b);
        }
    }
    write_vec2(&mut buf, c.offset);
    write_f32(&mut buf, c.material.restitution);
    write_f32(&mut buf, c.material.friction);
    write_f32(&mut buf, c.material.density);
    write_u64(&mut buf, c.filter.layers);
    write_u64(&mut buf, c.filter.mask);
    write_u32(&mut buf, c.filter.group as u32);
    write_bool(&mut buf, c.filter.sensor);
    buf
}

pub fn deserialize_body2(data: &[u8]) -> Result<Body2, Error> {
    let payload = read_typed_payload(data, &mut 0, TypeTag::Body2)?;
    let mut pos = 0;
    let id = StableId(read_u64(payload, &mut pos)?);
    let kind = match read_u8(payload, &mut pos)? {
        0 => BodyType::Static,
        1 => BodyType::Kinematic,
        2 => BodyType::Dynamic,
        _ => return Err(Error::InvalidEnumDiscriminant),
    };
    let position = read_vec2(payload, &mut pos)?;
    let rc = read_f32(payload, &mut pos)?;
    let rs = read_f32(payload, &mut pos)?;
    let rotation = Rot2 { c: rc, s: rs };
    let velocity = read_vec2(payload, &mut pos)?;
    let angular_velocity = read_f32(payload, &mut pos)?;
    let inv_mass = read_f32(payload, &mut pos)?;
    let inv_inertia = read_f32(payload, &mut pos)?;
    let n_colliders = read_u32(payload, &mut pos)? as usize;
    let mut colliders = Vec::with_capacity(n_colliders);
    for _ in 0..n_colliders {
        colliders.push(deserialize_collider2_inner(payload, &mut pos)?);
    }
    let restitution = read_f32(payload, &mut pos)?;
    let friction = read_f32(payload, &mut pos)?;
    let sleeping = read_bool(payload, &mut pos)?;
    let force = read_vec2(payload, &mut pos)?;
    let torque = read_f32(payload, &mut pos)?;
    let linear_damping = read_f32(payload, &mut pos)?;
    let angular_damping = read_f32(payload, &mut pos)?;
    let user_data = read_u64(payload, &mut pos)?;
    Ok(Body2 {
        id,
        kind,
        position,
        rotation,
        velocity,
        angular_velocity,
        inv_mass,
        inv_inertia,
        colliders,
        restitution,
        friction,
        sleeping,
        force,
        torque,
        linear_damping,
        angular_damping,
        user_data,
    })
}

fn deserialize_collider2_inner(data: &[u8], pos: &mut usize) -> Result<Collider2, Error> {
    let shape_disc = read_u8(data, pos)?;
    let shape = match shape_disc {
        0 => {
            let r = read_f32(data, pos)?;
            ColliderShape2::Circle(
                auralite_geometry::Circle2::new(r).map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        1 => {
            let h = read_vec2(data, pos)?;
            ColliderShape2::Box(
                auralite_geometry::Box2::new(h).map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        2 => {
            let r = read_f32(data, pos)?;
            let hh = read_f32(data, pos)?;
            ColliderShape2::Capsule(
                auralite_geometry::Capsule2::new(r, hh)
                    .map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        3 => {
            let n = read_u32(data, pos)? as usize;
            let mut vertices = Vec::with_capacity(n);
            for _ in 0..n {
                vertices.push(read_vec2(data, pos)?);
            }
            ColliderShape2::ConvexPolygon(
                auralite_geometry::ConvexPolygon::new(vertices)
                    .map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        4 => {
            let a = read_vec2(data, pos)?;
            let b = read_vec2(data, pos)?;
            ColliderShape2::Edge(
                auralite_geometry::Edge2::new(a, b).map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        _ => return Err(Error::UnsupportedTypeTag),
    };
    let offset = read_vec2(data, pos)?;
    let restitution = read_f32(data, pos)?;
    let friction = read_f32(data, pos)?;
    let density = read_f32(data, pos)?;
    let layers = read_u64(data, pos)?;
    let mask = read_u64(data, pos)?;
    let group = read_u32(data, pos)? as i32;
    let sensor = read_bool(data, pos)?;
    Ok(Collider2 {
        shape,
        offset,
        material: Material {
            restitution,
            friction,
            density,
        },
        filter: auralite_collision::CollisionFilter {
            layers,
            mask,
            group,
            sensor,
        },
    })
}

/// Extract angle from Rot2 by rotating (1,0) and computing atan2.
fn rot2_angle(r: Rot2) -> Real {
    let v = r.rotate(Vec2::X);
    v.y.atan2(v.x)
}

pub fn serialize_collider3(c: &Collider3) -> Vec<u8> {
    let mut buf = Vec::new();
    match &c.shape {
        ColliderShape3::Sphere(s) => {
            write_u8(&mut buf, 0);
            write_f32(&mut buf, s.radius);
        }
        ColliderShape3::Box(bx) => {
            write_u8(&mut buf, 1);
            write_vec3(&mut buf, bx.half);
        }
        ColliderShape3::Capsule(cap) => {
            write_u8(&mut buf, 2);
            write_f32(&mut buf, cap.radius);
            write_f32(&mut buf, cap.half_height);
        }
        ColliderShape3::ConvexHull(hull) => {
            write_u8(&mut buf, 3);
            write_u32(&mut buf, hull.vertices.len() as u32);
            for v in &hull.vertices {
                write_vec3(&mut buf, *v);
            }
        }
        ColliderShape3::TriangleMesh(mesh) => {
            write_u8(&mut buf, 4);
            write_u32(&mut buf, mesh.vertices.len() as u32);
            for v in &mesh.vertices {
                write_vec3(&mut buf, *v);
            }
            write_u32(&mut buf, mesh.indices.len() as u32);
            for f in &mesh.indices {
                write_u32(&mut buf, f[0]);
                write_u32(&mut buf, f[1]);
                write_u32(&mut buf, f[2]);
            }
        }
        ColliderShape3::Edge(edge) => {
            write_u8(&mut buf, 5);
            let a = edge.closest_point(Vec3::ZERO);
            let b = edge.closest_point(Vec3::X);
            write_vec3(&mut buf, a);
            write_vec3(&mut buf, b);
        }
    }
    write_vec3(&mut buf, c.offset);
    write_f32(&mut buf, c.material.restitution);
    write_f32(&mut buf, c.material.friction);
    write_f32(&mut buf, c.material.density);
    write_u64(&mut buf, c.filter.layers);
    write_u64(&mut buf, c.filter.mask);
    write_i32_as_u32(&mut buf, c.filter.group);
    write_bool(&mut buf, c.filter.sensor);
    write_typed_payload(TypeTag::Collider3, &buf)
}

pub fn deserialize_collider3(data: &[u8]) -> Result<Collider3, Error> {
    let payload = read_typed_payload(data, &mut 0, TypeTag::Collider3)?;
    let mut pos = 0;
    let shape_disc = read_u8(payload, &mut pos)?;
    let shape = match shape_disc {
        0 => {
            let r = read_f32(payload, &mut pos)?;
            ColliderShape3::Sphere(
                auralite_geometry::Sphere3::new(r).map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        1 => {
            let h = read_vec3(payload, &mut pos)?;
            ColliderShape3::Box(
                auralite_geometry::Box3::new(h).map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        2 => {
            let r = read_f32(payload, &mut pos)?;
            let hh = read_f32(payload, &mut pos)?;
            ColliderShape3::Capsule(
                auralite_geometry::Capsule3::new(r, hh)
                    .map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        3 => {
            let n = read_u32(payload, &mut pos)? as usize;
            let mut vertices = Vec::with_capacity(n);
            for _ in 0..n {
                vertices.push(read_vec3(payload, &mut pos)?);
            }
            ColliderShape3::ConvexHull(
                auralite_geometry::ConvexHull3::build(&vertices)
                    .map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        4 => {
            let n_v = read_u32(payload, &mut pos)? as usize;
            let mut vertices = Vec::with_capacity(n_v);
            for _ in 0..n_v {
                vertices.push(read_vec3(payload, &mut pos)?);
            }
            let n_i = read_u32(payload, &mut pos)? as usize;
            let mut indices = Vec::with_capacity(n_i);
            for _ in 0..n_i {
                let i0 = read_u32(payload, &mut pos)?;
                let i1 = read_u32(payload, &mut pos)?;
                let i2 = read_u32(payload, &mut pos)?;
                indices.push([i0, i1, i2]);
            }
            ColliderShape3::TriangleMesh(
                auralite_geometry::TriangleMesh::new(vertices, indices)
                    .map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        5 => {
            let a = read_vec3(payload, &mut pos)?;
            let b = read_vec3(payload, &mut pos)?;
            ColliderShape3::Edge(
                auralite_geometry::Edge3::new(a, b).map_err(|_| Error::InvalidEnumDiscriminant)?,
            )
        }
        _ => return Err(Error::UnsupportedTypeTag),
    };
    let offset = read_vec3(payload, &mut pos)?;
    let restitution = read_f32(payload, &mut pos)?;
    let friction = read_f32(payload, &mut pos)?;
    let density = read_f32(payload, &mut pos)?;
    let layers = read_u64(payload, &mut pos)?;
    let mask = read_u64(payload, &mut pos)?;
    let group = read_u32_as_i32(payload, &mut pos)?;
    let sensor = read_bool(payload, &mut pos)?;
    Ok(Collider3 {
        shape,
        offset,
        material: Material {
            restitution,
            friction,
            density,
        },
        filter: auralite_collision::CollisionFilter {
            layers,
            mask,
            group,
            sensor,
        },
    })
}

pub fn serialize_body3(b: &Body3) -> Vec<u8> {
    let mut buf = Vec::new();
    write_u64(&mut buf, b.id.0);
    write_u8(&mut buf, b.kind as u8);
    write_vec3(&mut buf, b.position);
    write_f32(&mut buf, b.rotation.x);
    write_f32(&mut buf, b.rotation.y);
    write_f32(&mut buf, b.rotation.z);
    write_f32(&mut buf, b.rotation.w);
    write_vec3(&mut buf, b.velocity);
    write_vec3(&mut buf, b.angular_velocity);
    write_f32(&mut buf, b.inv_mass);
    write_vec3(&mut buf, b.inv_inertia_diagonal);
    write_u32(&mut buf, b.colliders.len() as u32);
    for c in &b.colliders {
        let col_bytes = serialize_collider3(c);
        write_u32(&mut buf, col_bytes.len() as u32);
        buf.extend_from_slice(&col_bytes);
    }
    write_f32(&mut buf, b.restitution);
    write_f32(&mut buf, b.friction);
    write_bool(&mut buf, b.sleeping);
    write_vec3(&mut buf, b.force);
    write_vec3(&mut buf, b.torque);
    write_f32(&mut buf, b.linear_damping);
    write_f32(&mut buf, b.angular_damping);
    write_u64(&mut buf, b.user_data);
    write_typed_payload(TypeTag::Body3, &buf)
}

pub fn deserialize_body3(data: &[u8]) -> Result<Body3, Error> {
    let payload = read_typed_payload(data, &mut 0, TypeTag::Body3)?;
    let mut pos = 0;
    let id = StableId(read_u64(payload, &mut pos)?);
    let kind = match read_u8(payload, &mut pos)? {
        0 => BodyType::Static,
        1 => BodyType::Kinematic,
        2 => BodyType::Dynamic,
        _ => return Err(Error::InvalidEnumDiscriminant),
    };
    let position = read_vec3(payload, &mut pos)?;
    let rx = read_f32(payload, &mut pos)?;
    let ry = read_f32(payload, &mut pos)?;
    let rz = read_f32(payload, &mut pos)?;
    let rw = read_f32(payload, &mut pos)?;
    let rotation = Quat {
        x: rx,
        y: ry,
        z: rz,
        w: rw,
    };
    let velocity = read_vec3(payload, &mut pos)?;
    let angular_velocity = read_vec3(payload, &mut pos)?;
    let inv_mass = read_f32(payload, &mut pos)?;
    let inv_inertia_diagonal = read_vec3(payload, &mut pos)?;
    let n_colliders = read_u32(payload, &mut pos)? as usize;
    let mut colliders = Vec::with_capacity(n_colliders);
    for _ in 0..n_colliders {
        let clen = read_u32(payload, &mut pos)? as usize;
        if pos + clen > payload.len() {
            return Err(Error::Truncated);
        }
        colliders.push(deserialize_collider3(&payload[pos..pos + clen])?);
        pos += clen;
    }
    let restitution = read_f32(payload, &mut pos)?;
    let friction = read_f32(payload, &mut pos)?;
    let sleeping = read_bool(payload, &mut pos)?;
    let force = read_vec3(payload, &mut pos)?;
    let torque = read_vec3(payload, &mut pos)?;
    let linear_damping = read_f32(payload, &mut pos)?;
    let angular_damping = read_f32(payload, &mut pos)?;
    let user_data = read_u64(payload, &mut pos)?;

    Ok(Body3 {
        id,
        kind,
        position,
        rotation,
        velocity,
        angular_velocity,
        inv_mass,
        inv_inertia_diagonal,
        colliders,
        restitution,
        friction,
        sleeping,
        force,
        torque,
        linear_damping,
        angular_damping,
        user_data,
    })
}

// ─── World2 snapshot serialization ───────────────────────────────────────────

pub fn serialize_world2(world: &World2) -> Vec<u8> {
    let mut buf = Vec::new();
    write_u64(&mut buf, world.step_count());
    write_vec2(&mut buf, world.gravity());
    let mut bodies_data = Vec::new();
    for (_, body) in world.bodies_iter() {
        let b_bytes = serialize_body2(body);
        write_u32(&mut bodies_data, b_bytes.len() as u32);
        bodies_data.extend_from_slice(&b_bytes);
    }
    write_u32(&mut buf, bodies_data.len() as u32);
    buf.extend_from_slice(&bodies_data);

    let mut joints_data = Vec::new();
    for j in &world.joints {
        let j_bytes = serialize_joint2(j);
        write_u32(&mut joints_data, j_bytes.len() as u32);
        joints_data.extend_from_slice(&j_bytes);
    }
    write_u32(&mut buf, joints_data.len() as u32);
    buf.extend_from_slice(&joints_data);
    write_typed_payload(TypeTag::World2State, &buf)
}

pub fn serialize_world3(world: &World3) -> Vec<u8> {
    let mut buf = Vec::new();
    write_u64(&mut buf, world.step_count());
    write_vec3(&mut buf, world.gravity());
    let mut bodies_data = Vec::new();
    for (_, body) in world.bodies_iter() {
        let b_bytes = serialize_body3(body);
        write_u32(&mut bodies_data, b_bytes.len() as u32);
        bodies_data.extend_from_slice(&b_bytes);
    }
    write_u32(&mut buf, bodies_data.len() as u32);
    buf.extend_from_slice(&bodies_data);

    let mut joints_data = Vec::new();
    for j in &world.joints {
        let j_bytes = serialize_joint3(j);
        write_u32(&mut joints_data, j_bytes.len() as u32);
        joints_data.extend_from_slice(&j_bytes);
    }
    write_u32(&mut buf, joints_data.len() as u32);
    buf.extend_from_slice(&joints_data);
    write_typed_payload(TypeTag::World3State, &buf)
}

/// Deserialize 2D world state into components that can be restored.
pub fn deserialize_world2_state(data: &[u8]) -> Result<(u64, Vec2, Vec<u8>, Vec<u8>), Error> {
    let payload = read_typed_payload(data, &mut 0, TypeTag::World2State)?;
    let mut pos = 0;
    let step = read_u64(payload, &mut pos)?;
    let gravity = read_vec2(payload, &mut pos)?;
    let bodies_len = read_u32(payload, &mut pos)? as usize;
    if pos + bodies_len > payload.len() {
        return Err(Error::Truncated);
    }
    let bodies_data = payload[pos..pos + bodies_len].to_vec();
    pos += bodies_len;
    let joints_len = read_u32(payload, &mut pos)? as usize;
    if pos + joints_len > payload.len() {
        return Err(Error::Truncated);
    }
    let joints_data = payload[pos..pos + joints_len].to_vec();
    Ok((step, gravity, bodies_data, joints_data))
}

/// Deserialize 3D world state into components that can be restored.
pub fn deserialize_world3_state(data: &[u8]) -> Result<(u64, Vec3, Vec<u8>, Vec<u8>), Error> {
    let payload = read_typed_payload(data, &mut 0, TypeTag::World3State)?;
    let mut pos = 0;
    let step = read_u64(payload, &mut pos)?;
    let gravity = read_vec3(payload, &mut pos)?;
    let bodies_len = read_u32(payload, &mut pos)? as usize;
    if pos + bodies_len > payload.len() {
        return Err(Error::Truncated);
    }
    let bodies_data = payload[pos..pos + bodies_len].to_vec();
    pos += bodies_len;
    let joints_len = read_u32(payload, &mut pos)? as usize;
    if pos + joints_len > payload.len() {
        return Err(Error::Truncated);
    }
    let joints_data = payload[pos..pos + joints_len].to_vec();
    Ok((step, gravity, bodies_data, joints_data))
}

pub fn deserialize_world2(data: &[u8]) -> Result<World2, Error> {
    let (step, gravity, bodies_data, joints_data) = deserialize_world2_state(data)?;
    let mut w = World2::default();
    let _ = w.set_gravity(gravity);
    w.set_step_count(step);
    let mut pos = 0;
    while pos + 4 <= bodies_data.len() {
        let blen = read_u32(&bodies_data, &mut pos)? as usize;
        if pos + blen > bodies_data.len() {
            return Err(Error::Truncated);
        }
        let body = deserialize_body2(&bodies_data[pos..pos + blen])?;
        pos += blen;
        w.insert_restored_body(body);
    }
    let mut jpos = 0;
    while jpos + 4 <= joints_data.len() {
        let jlen = read_u32(&joints_data, &mut jpos)? as usize;
        if jpos + jlen > joints_data.len() {
            return Err(Error::Truncated);
        }
        let joint = deserialize_joint2(&joints_data[jpos..jpos + jlen])?;
        jpos += jlen;
        w.joints.push(joint);
    }
    w.rebuild_tree();
    Ok(w)
}

pub fn deserialize_world3(data: &[u8]) -> Result<World3, Error> {
    let (step, gravity, bodies_data, joints_data) = deserialize_world3_state(data)?;
    let mut w = World3::default();
    let _ = w.set_gravity(gravity);
    w.set_step_count(step);
    let mut pos = 0;
    while pos + 4 <= bodies_data.len() {
        let blen = read_u32(&bodies_data, &mut pos)? as usize;
        if pos + blen > bodies_data.len() {
            return Err(Error::Truncated);
        }
        let body = deserialize_body3(&bodies_data[pos..pos + blen])?;
        pos += blen;
        w.insert_restored_body(body);
    }
    let mut jpos = 0;
    while jpos + 4 <= joints_data.len() {
        let jlen = read_u32(&joints_data, &mut jpos)? as usize;
        if jpos + jlen > joints_data.len() {
            return Err(Error::Truncated);
        }
        let joint = deserialize_joint3(&joints_data[jpos..jpos + jlen])?;
        jpos += jlen;
        w.joints.push(joint);
    }
    w.rebuild_tree();
    Ok(w)
}

// ─── Joint serialization ────────────────────────────────────────────────────

pub fn serialize_joint2(j: &Joint2) -> Vec<u8> {
    let c = &j.config;
    let mut buf = Vec::new();
    let jt_disc = match c.joint_type {
        JointType2::Weld => 0u8,
        JointType2::Distance => 1u8,
        JointType2::Spring { .. } => 2u8,
        JointType2::Revolute => 3u8,
        JointType2::Prismatic { .. } => 4u8,
    };
    write_u8(&mut buf, jt_disc);
    write_u64(&mut buf, j.id.0);
    write_u32(&mut buf, c.body_a.index());
    write_u32(&mut buf, c.body_a.generation());
    write_u32(&mut buf, c.body_b.index());
    write_u32(&mut buf, c.body_b.generation());
    write_vec2(&mut buf, c.anchor_a);
    write_vec2(&mut buf, c.anchor_b);
    match c.joint_type {
        JointType2::Spring { stiffness, damping } => {
            write_f32(&mut buf, stiffness);
            write_f32(&mut buf, damping);
        }
        JointType2::Prismatic { axis_local } => {
            write_vec2(&mut buf, axis_local);
        }
        _ => {}
    }
    write_f32(&mut buf, c.limits.min);
    write_f32(&mut buf, c.limits.max);
    write_bool(&mut buf, c.limits.enabled);
    write_f32(&mut buf, c.motor.target_speed);
    write_f32(&mut buf, c.motor.max_force);
    write_bool(&mut buf, c.motor.enabled);
    write_f32(&mut buf, c.break_impulse);
    write_u64(&mut buf, c.user_data);
    write_f32(&mut buf, j.impulse);
    write_f32(&mut buf, j.accumulated_position_error);
    write_bool(&mut buf, j.broken);
    write_typed_payload(TypeTag::Joint2State, &buf)
}

fn write_i32(buf: &mut Vec<u8>, v: i32) {
    buf.extend_from_slice(&v.to_le_bytes());
}

pub fn deserialize_joint2(data: &[u8]) -> Result<Joint2, Error> {
    let payload = read_typed_payload(data, &mut 0, TypeTag::Joint2State)?;
    let mut pos = 0;
    let jt_disc = read_u8(payload, &mut pos)?;
    let id = JointId(read_u64(payload, &mut pos)?);
    let ba_i = read_u32(payload, &mut pos)?;
    let ba_g = read_u32(payload, &mut pos)?;
    let bb_i = read_u32(payload, &mut pos)?;
    let bb_g = read_u32(payload, &mut pos)?;
    let anchor_a = read_vec2(payload, &mut pos)?;
    let anchor_b = read_vec2(payload, &mut pos)?;
    let joint_type = match jt_disc {
        0 => JointType2::Weld,
        1 => JointType2::Distance,
        2 => {
            let stiffness = read_f32(payload, &mut pos)?;
            let damping = read_f32(payload, &mut pos)?;
            JointType2::Spring { stiffness, damping }
        }
        3 => JointType2::Revolute,
        4 => {
            let axis_local = read_vec2(payload, &mut pos)?;
            JointType2::Prismatic { axis_local }
        }
        _ => return Err(Error::InvalidEnumDiscriminant),
    };
    let l_min = read_f32(payload, &mut pos)?;
    let l_max = read_f32(payload, &mut pos)?;
    let l_enabled = read_bool(payload, &mut pos)?;
    let m_speed = read_f32(payload, &mut pos)?;
    let m_force = read_f32(payload, &mut pos)?;
    let m_enabled = read_bool(payload, &mut pos)?;
    let break_impulse = read_f32(payload, &mut pos)?;
    let user_data = read_u64(payload, &mut pos)?;
    let impulse = read_f32(payload, &mut pos)?;
    let pos_error = read_f32(payload, &mut pos)?;
    let broken = read_bool(payload, &mut pos)?;

    Ok(Joint2 {
        id,
        config: JointConfig2 {
            joint_type,
            body_a: BodyHandle2::new(ba_i, ba_g),
            body_b: BodyHandle2::new(bb_i, bb_g),
            anchor_a,
            anchor_b,
            limits: JointLimits {
                min: l_min,
                max: l_max,
                enabled: l_enabled,
            },
            motor: JointMotor {
                target_speed: m_speed,
                max_force: m_force,
                enabled: m_enabled,
            },
            break_impulse,
            user_data,
        },
        impulse,
        accumulated_position_error: pos_error,
        broken,
    })
}

pub fn serialize_joint3(j: &Joint3) -> Vec<u8> {
    let c = &j.config;
    let mut buf = Vec::new();
    let jt_disc = match c.joint_type {
        JointType3::BallSocket => 0u8,
        JointType3::Distance => 1u8,
        JointType3::Spring { .. } => 2u8,
        JointType3::Weld => 3u8,
        JointType3::Hinge { .. } => 4u8,
        JointType3::Slider { .. } => 5u8,
        JointType3::ConeTwist { .. } => 6u8,
    };
    write_u8(&mut buf, jt_disc);
    write_u64(&mut buf, j.id.0);
    write_u32(&mut buf, c.body_a.index());
    write_u32(&mut buf, c.body_a.generation());
    write_u32(&mut buf, c.body_b.index());
    write_u32(&mut buf, c.body_b.generation());
    write_vec3(&mut buf, c.anchor_a);
    write_vec3(&mut buf, c.anchor_b);
    match c.joint_type {
        JointType3::Spring { stiffness, damping } => {
            write_f32(&mut buf, stiffness);
            write_f32(&mut buf, damping);
        }
        JointType3::Hinge { axis_local } | JointType3::Slider { axis_local } => {
            write_vec3(&mut buf, axis_local);
        }
        _ => {}
    }
    write_f32(&mut buf, c.limits.min);
    write_f32(&mut buf, c.limits.max);
    write_bool(&mut buf, c.limits.enabled);
    write_f32(&mut buf, c.motor.target_speed);
    write_f32(&mut buf, c.motor.max_force);
    write_bool(&mut buf, c.motor.enabled);
    write_f32(&mut buf, c.break_impulse);
    write_u64(&mut buf, c.user_data);
    write_f32(&mut buf, j.impulse);
    write_f32(&mut buf, j.accumulated_position_error);
    write_bool(&mut buf, j.broken);
    write_typed_payload(TypeTag::Joint3State, &buf)
}

pub fn deserialize_joint3(data: &[u8]) -> Result<Joint3, Error> {
    let payload = read_typed_payload(data, &mut 0, TypeTag::Joint3State)?;
    let mut pos = 0;
    let jt_disc = read_u8(payload, &mut pos)?;
    let id = JointId(read_u64(payload, &mut pos)?);
    let ba_i = read_u32(payload, &mut pos)?;
    let ba_g = read_u32(payload, &mut pos)?;
    let bb_i = read_u32(payload, &mut pos)?;
    let bb_g = read_u32(payload, &mut pos)?;
    let anchor_a = read_vec3(payload, &mut pos)?;
    let anchor_b = read_vec3(payload, &mut pos)?;
    let joint_type = match jt_disc {
        0 => JointType3::BallSocket,
        1 => JointType3::Distance,
        2 => {
            let stiffness = read_f32(payload, &mut pos)?;
            let damping = read_f32(payload, &mut pos)?;
            JointType3::Spring { stiffness, damping }
        }
        3 => JointType3::Weld,
        4 => {
            let axis_local = read_vec3(payload, &mut pos)?;
            JointType3::Hinge { axis_local }
        }
        5 => {
            let axis_local = read_vec3(payload, &mut pos)?;
            JointType3::Slider { axis_local }
        }
        _ => return Err(Error::InvalidEnumDiscriminant),
    };
    let l_min = read_f32(payload, &mut pos)?;
    let l_max = read_f32(payload, &mut pos)?;
    let l_enabled = read_bool(payload, &mut pos)?;
    let m_speed = read_f32(payload, &mut pos)?;
    let m_force = read_f32(payload, &mut pos)?;
    let m_enabled = read_bool(payload, &mut pos)?;
    let break_impulse = read_f32(payload, &mut pos)?;
    let user_data = read_u64(payload, &mut pos)?;
    let impulse = read_f32(payload, &mut pos)?;
    let pos_error = read_f32(payload, &mut pos)?;
    let broken = read_bool(payload, &mut pos)?;

    Ok(Joint3 {
        id,
        config: JointConfig3 {
            joint_type,
            body_a: BodyHandle3::new(ba_i, ba_g),
            body_b: BodyHandle3::new(bb_i, bb_g),
            anchor_a,
            anchor_b,
            limits: JointLimits {
                min: l_min,
                max: l_max,
                enabled: l_enabled,
            },
            motor: JointMotor {
                target_speed: m_speed,
                max_force: m_force,
                enabled: m_enabled,
            },
            break_impulse,
            user_data,
        },
        impulse,
        accumulated_position_error: pos_error,
        broken,
    })
}

// ─── SoftBody & ParticleStorage serialization ───────────────────────────────

pub fn serialize_soft_body(sb: &auralite_softbody::SoftBody) -> Vec<u8> {
    let mut buf = Vec::new();
    write_u32(&mut buf, sb.particles.len() as u32);
    for p in &sb.particles {
        write_vec3(&mut buf, p.position);
        write_vec3(&mut buf, p.old_position);
        write_vec3(&mut buf, p.velocity);
        write_f32(&mut buf, p.inv_mass);
        write_bool(&mut buf, p.pinned);
    }
    write_f32(&mut buf, sb.damping);
    write_vec3(&mut buf, sb.wind);
    write_bool(&mut buf, sb.aerodynamic);
    write_typed_payload(TypeTag::SoftBody, &buf)
}

pub fn deserialize_soft_body(data: &[u8]) -> Result<auralite_softbody::SoftBody, Error> {
    let payload = read_typed_payload(data, &mut 0, TypeTag::SoftBody)?;
    let mut pos = 0;
    let n_p = read_u32(payload, &mut pos)? as usize;
    let mut particles = Vec::with_capacity(n_p);
    for _ in 0..n_p {
        let position = read_vec3(payload, &mut pos)?;
        let old_position = read_vec3(payload, &mut pos)?;
        let velocity = read_vec3(payload, &mut pos)?;
        let inv_mass = read_f32(payload, &mut pos)?;
        let pinned = read_bool(payload, &mut pos)?;
        particles.push(auralite_softbody::Particle {
            position,
            old_position,
            velocity,
            inv_mass,
            pinned,
        });
    }
    let damping = read_f32(payload, &mut pos)?;
    let wind = read_vec3(payload, &mut pos)?;
    let aerodynamic = read_bool(payload, &mut pos)?;
    let mut sb = auralite_softbody::SoftBody::new(damping);
    sb.particles = particles;
    sb.wind = wind;
    sb.aerodynamic = aerodynamic;
    Ok(sb)
}

pub fn serialize_particle_storage(ps: &auralite_particles::ParticleStorage) -> Vec<u8> {
    let mut buf = Vec::new();
    write_u32(&mut buf, ps.capacity as u32);
    write_u32(&mut buf, ps.positions.len() as u32);
    for i in 0..ps.positions.len() {
        write_vec3(&mut buf, ps.positions[i]);
        write_vec3(&mut buf, ps.velocities[i]);
        write_f32(&mut buf, ps.lifetimes[i]);
        write_f32(&mut buf, ps.max_lifetimes[i]);
        write_u8(
            &mut buf,
            match ps.types[i] {
                auralite_particles::ParticleType::Generic => 0,
                auralite_particles::ParticleType::Fluid => 1,
                auralite_particles::ParticleType::BuoyancySample => 2,
            },
        );
        write_bool(&mut buf, ps.alive[i]);
    }
    write_typed_payload(TypeTag::ParticleStorage, &buf)
}

pub fn deserialize_particle_storage(
    data: &[u8],
) -> Result<auralite_particles::ParticleStorage, Error> {
    let payload = read_typed_payload(data, &mut 0, TypeTag::ParticleStorage)?;
    let mut pos = 0;
    let capacity = read_u32(payload, &mut pos)? as usize;
    let n = read_u32(payload, &mut pos)? as usize;
    let mut ps = auralite_particles::ParticleStorage::new(capacity);
    ps.positions.clear();
    ps.velocities.clear();
    ps.lifetimes.clear();
    ps.max_lifetimes.clear();
    ps.types.clear();
    ps.alive.clear();
    for _ in 0..n {
        ps.positions.push(read_vec3(payload, &mut pos)?);
        ps.velocities.push(read_vec3(payload, &mut pos)?);
        ps.lifetimes.push(read_f32(payload, &mut pos)?);
        ps.max_lifetimes.push(read_f32(payload, &mut pos)?);
        ps.types.push(match read_u8(payload, &mut pos)? {
            0 => auralite_particles::ParticleType::Generic,
            1 => auralite_particles::ParticleType::Fluid,
            2 => auralite_particles::ParticleType::BuoyancySample,
            _ => return Err(Error::InvalidEnumDiscriminant),
        });
        ps.alive.push(read_bool(payload, &mut pos)?);
    }
    Ok(ps)
}

// ─── RNG serialization ──────────────────────────────────────────────────────

pub fn serialize_rng(rng: &auralite_core::Rng) -> Vec<u8> {
    let mut buf = Vec::new();
    write_u64(&mut buf, rng.state());
    write_typed_payload(TypeTag::RngState, &buf)
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use auralite_collision::CollisionFilter;

    #[test]
    fn envelope_round_trip() {
        let data = b"hello world";
        let enc = encode(data);
        let dec = decode(&enc, 100).unwrap();
        assert_eq!(dec, data);
    }

    #[test]
    fn checksum_detects_corruption() {
        let data = b"test data";
        let mut enc = encode(data);
        enc[18] ^= 0xFF; // corrupt one byte
        assert_eq!(decode(&enc, 100), Err(Error::ChecksumMismatch));
    }

    #[test]
    fn truncated_fails() {
        let enc = encode(b"abc");
        for n in 0..enc.len() {
            assert!(decode(&enc[..n], 100).is_err());
        }
    }

    #[test]
    fn collider2_serialization_round_trip() {
        let c = Collider2 {
            shape: ColliderShape2::Circle(auralite_geometry::Circle2::new(1.5).unwrap()),
            offset: Vec2 { x: 0.1, y: 0.2 },
            material: Material {
                restitution: 0.3,
                friction: 0.4,
                density: 2.0,
            },
            filter: CollisionFilter {
                layers: 1,
                mask: 3,
                group: -1,
                sensor: true,
            },
        };
        let data = serialize_collider2(&c);
        let restored = deserialize_collider2(&data).unwrap();
        assert_eq!(restored.offset.x, c.offset.x);
        assert_eq!(restored.material.restitution, c.material.restitution);
        assert_eq!(restored.filter.sensor, c.filter.sensor);
        assert_eq!(restored.filter.group, c.filter.group);
    }

    #[test]
    fn body2_serialization_round_trip() {
        let b = Body2 {
            id: StableId(42),
            kind: BodyType::Dynamic,
            position: Vec2 { x: 1.0, y: 2.0 },
            rotation: Rot2::from_radians(0.5).unwrap(),
            velocity: Vec2 { x: 3.0, y: 4.0 },
            angular_velocity: 0.1,
            inv_mass: 0.5,
            inv_inertia: 0.2,
            colliders: vec![],
            restitution: 0.1,
            friction: 0.2,
            sleeping: false,
            force: Vec2::ZERO,
            torque: 0.0,
            linear_damping: 0.01,
            angular_damping: 0.02,
            user_data: 7,
        };
        let data = serialize_body2(&b);
        let restored = deserialize_body2(&data).unwrap();
        assert_eq!(restored.id.0, b.id.0);
        assert_eq!(restored.kind, b.kind);
        assert!((restored.position.x - b.position.x).abs() < 1e-6);
        assert!((restored.velocity.y - b.velocity.y).abs() < 1e-6);
        assert_eq!(restored.restitution, b.restitution);
        assert_eq!(restored.user_data, b.user_data);
    }

    #[test]
    fn typed_payload_tag_check() {
        let data = write_typed_payload(TypeTag::RngState, &[1, 2, 3]);
        let result = read_typed_payload(&data, &mut 0, TypeTag::RngState);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), &[1, 2, 3]);

        let result2 = read_typed_payload(&data, &mut 0, TypeTag::World2State);
        assert_eq!(result2, Err(Error::TypedPayloadMismatch));
    }

    #[test]
    fn body2_with_colliders_round_trip() {
        let b = Body2 {
            id: StableId(100),
            kind: BodyType::Dynamic,
            position: Vec2 { x: 10.0, y: 20.0 },
            rotation: Rot2::from_radians(1.0).unwrap(),
            velocity: Vec2 { x: 1.0, y: 0.0 },
            angular_velocity: 0.5,
            inv_mass: 0.1,
            inv_inertia: 0.05,
            colliders: vec![
                Collider2 {
                    shape: ColliderShape2::Box(
                        auralite_geometry::Box2::new(Vec2 { x: 0.5, y: 0.5 }).unwrap(),
                    ),
                    offset: Vec2::ZERO,
                    material: Material::default(),
                    filter: CollisionFilter::default(),
                },
                Collider2 {
                    shape: ColliderShape2::Capsule(
                        auralite_geometry::Capsule2::new(0.3, 0.5).unwrap(),
                    ),
                    offset: Vec2 { x: 0.0, y: 0.5 },
                    material: Material {
                        restitution: 0.5,
                        friction: 1.0,
                        density: 1.0,
                    },
                    filter: CollisionFilter::default(),
                },
            ],
            restitution: 0.3,
            friction: 0.7,
            sleeping: true,
            force: Vec2 { x: 0.0, y: -9.81 },
            torque: 1.0,
            linear_damping: 0.05,
            angular_damping: 0.1,
            user_data: 99,
        };
        let data = serialize_body2(&b);
        let restored = deserialize_body2(&data).unwrap();
        assert_eq!(restored.id.0, b.id.0);
        assert_eq!(restored.colliders.len(), b.colliders.len());
        assert_eq!(restored.sleeping, b.sleeping);
        assert_eq!(restored.linear_damping, b.linear_damping);
        assert_eq!(restored.user_data, b.user_data);
    }

    #[test]
    fn rng_serialization() {
        let rng = auralite_core::Rng::new(12345);
        let data = serialize_rng(&rng);
        let _ = read_typed_payload(&data, &mut 0, TypeTag::RngState).unwrap();
        // Just verify it round-trips through envelope
        let enc = encode(&data);
        let dec = decode(&enc, 1000).unwrap();
        assert_eq!(dec.len(), data.len());
    }

    #[test]
    fn body3_serialization_round_trip() {
        let b = Body3 {
            id: StableId(42),
            kind: BodyType::Dynamic,
            position: Vec3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            rotation: Quat::identity(),
            velocity: Vec3 {
                x: -1.0,
                y: 0.5,
                z: 2.0,
            },
            angular_velocity: Vec3 {
                x: 0.1,
                y: 0.2,
                z: 0.3,
            },
            inv_mass: 0.2,
            inv_inertia_diagonal: Vec3 {
                x: 0.1,
                y: 0.1,
                z: 0.1,
            },
            colliders: vec![Collider3 {
                shape: ColliderShape3::Sphere(auralite_geometry::Sphere3::new(1.0).unwrap()),
                offset: Vec3::ZERO,
                material: Material {
                    restitution: 0.5,
                    friction: 0.4,
                    density: 1.0,
                },
                filter: CollisionFilter::default(),
            }],
            restitution: 0.5,
            friction: 0.4,
            sleeping: false,
            force: Vec3 {
                x: 0.0,
                y: -9.81,
                z: 0.0,
            },
            torque: Vec3::ZERO,
            linear_damping: 0.02,
            angular_damping: 0.03,
            user_data: 777,
        };
        let data = serialize_body3(&b);
        let restored = deserialize_body3(&data).unwrap();
        assert_eq!(restored.id.0, b.id.0);
        assert_eq!(restored.colliders.len(), b.colliders.len());
        assert_eq!(restored.linear_damping, b.linear_damping);
        assert_eq!(restored.user_data, b.user_data);
    }

    #[test]
    fn joint3_serialization_round_trip() {
        let j = Joint3 {
            id: JointId(101),
            config: JointConfig3 {
                joint_type: JointType3::Distance,
                body_a: BodyHandle3::new(1, 0),
                body_b: BodyHandle3::new(2, 0),
                anchor_a: Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                anchor_b: Vec3 {
                    x: -1.0,
                    y: 0.0,
                    z: 0.0,
                },
                limits: JointLimits {
                    min: 0.5,
                    max: 2.5,
                    enabled: true,
                },
                motor: JointMotor {
                    target_speed: 1.0,
                    max_force: 50.0,
                    enabled: true,
                },
                break_impulse: 10.0,
                user_data: 555,
            },
            impulse: 1.23,
            accumulated_position_error: 0.45,
            broken: false,
        };
        let data = serialize_joint3(&j);
        let restored = deserialize_joint3(&data).unwrap();
        assert_eq!(restored.id.0, j.id.0);
        assert_eq!(restored.config.break_impulse, j.config.break_impulse);
        assert_eq!(restored.config.limits.max, j.config.limits.max);
        assert_eq!(restored.impulse, j.impulse);
    }

    #[test]
    fn softbody_serialization_round_trip() {
        let mut sb = auralite_softbody::SoftBody::new(0.05);
        sb.particles.push(auralite_softbody::Particle::new(
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            1.0,
        ));
        sb.wind = Vec3 {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        };
        sb.aerodynamic = true;
        let data = serialize_soft_body(&sb);
        let restored = deserialize_soft_body(&data).unwrap();
        assert_eq!(restored.particles.len(), sb.particles.len());
        assert_eq!(restored.wind, sb.wind);
        assert_eq!(restored.aerodynamic, sb.aerodynamic);
    }

    #[test]
    fn particle_storage_round_trip() {
        let mut ps = auralite_particles::ParticleStorage::new(20);
        let _ = ps.spawn(
            Vec3::ZERO,
            Vec3::Y,
            2.0,
            auralite_particles::ParticleType::Fluid,
        );
        let data = serialize_particle_storage(&ps);
        let restored = deserialize_particle_storage(&data).unwrap();
        assert_eq!(restored.positions.len(), ps.positions.len());
        assert_eq!(restored.types[0], auralite_particles::ParticleType::Fluid);
    }

    #[test]
    fn world2_snapshot_round_trip_replays_bitwise() {
        let mut w = World2::default();
        w.add_body(
            auralite_dynamics::BodyBuilder2::dynamic()
                .position(Vec2 { x: 0.0, y: 5.0 })
                .velocity(Vec2 { x: 1.0, y: 0.0 })
                .add_collider(Collider2 {
                    shape: ColliderShape2::Circle(auralite_geometry::Circle2::new(0.5).unwrap()),
                    offset: Vec2::ZERO,
                    material: Material::default(),
                    filter: auralite_collision::CollisionFilter::default(),
                }),
        )
        .unwrap();
        for _ in 0..30 {
            w.step(0.016).unwrap();
        }
        let data = serialize_world2(&w);
        let mut w_rest = deserialize_world2(&data).unwrap();
        for (h, b) in w.bodies_iter() {
            let b2 = w_rest.body(h).unwrap();
            println!(
                "body diff pos={:?} vs {:?} rot={:?} vs {:?} vel={:?} vs {:?}",
                b.position, b2.position, b.rotation, b2.rotation, b.velocity, b2.velocity
            );
        }
        println!("hash w={} w_rest={}", w.state_hash(), w_rest.state_hash());
        assert_eq!(w.state_hash(), w_rest.state_hash());
        for _ in 0..30 {
            w.step(0.016).unwrap();
            w_rest.step(0.016).unwrap();
        }
        assert_eq!(
            w.state_hash(),
            w_rest.state_hash(),
            "World2 restored from serialized snapshot must replay bitwise identically"
        );
    }

    #[test]
    fn world3_snapshot_round_trip_replays_bitwise() {
        let mut w = World3::default();
        let b1 = w
            .add_body(
                auralite_dynamics::BodyBuilder3::dynamic()
                    .position(Vec3 {
                        x: 0.0,
                        y: 5.0,
                        z: 0.0,
                    })
                    .velocity(Vec3 {
                        x: 1.0,
                        y: 0.0,
                        z: 0.5,
                    })
                    .add_collider(Collider3 {
                        shape: ColliderShape3::Sphere(
                            auralite_geometry::Sphere3::new(0.5).unwrap(),
                        ),
                        offset: Vec3::ZERO,
                        material: Material::default(),
                        filter: auralite_collision::CollisionFilter::default(),
                    }),
            )
            .unwrap();
        let b2 = w
            .add_body(
                auralite_dynamics::BodyBuilder3::dynamic()
                    .position(Vec3 {
                        x: 2.0,
                        y: 5.0,
                        z: 0.0,
                    })
                    .add_collider(Collider3 {
                        shape: ColliderShape3::Sphere(
                            auralite_geometry::Sphere3::new(0.5).unwrap(),
                        ),
                        offset: Vec3::ZERO,
                        material: Material::default(),
                        filter: auralite_collision::CollisionFilter::default(),
                    }),
            )
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
            user_data: 42,
        })
        .unwrap();
        for _ in 0..30 {
            w.step(0.016).unwrap();
        }
        let data = serialize_world3(&w);
        let mut w_rest = deserialize_world3(&data).unwrap();
        for (h, b) in w.bodies_iter() {
            let b2 = w_rest.body(h).unwrap();
            println!(
                "body diff pos={:?} vs {:?} rot={:?} vs {:?} vel={:?} vs {:?}",
                b.position, b2.position, b.rotation, b2.rotation, b.velocity, b2.velocity
            );
        }
        println!("hash w={} w_rest={}", w.state_hash(), w_rest.state_hash());
        assert_eq!(w.state_hash(), w_rest.state_hash());
        for _ in 0..30 {
            w.step(0.016).unwrap();
            w_rest.step(0.016).unwrap();
        }
        assert_eq!(
            w.state_hash(),
            w_rest.state_hash(),
            "World3 restored from serialized snapshot must replay bitwise identically"
        );
    }
}
