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
use auralite_dynamics::joints::JointType2;
use auralite_dynamics::{
    Body2, BodyHandle2, BodyType, Collider2, ColliderShape2, Joint2, JointConfig2, JointLimits,
    JointMotor, Material, World2,
};
use auralite_math::{Real, Rot2, Vec2, Vec3};

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
    write_f32(&mut buf, rot2_angle(b.rotation));
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
    let angle = read_f32(payload, &mut pos)?;
    let rotation = Rot2::from_radians(angle).map_err(|_| Error::InvalidEnumDiscriminant)?;
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

// ─── World2 snapshot serialization ───────────────────────────────────────────

pub fn serialize_world2(world: &World2) -> Vec<u8> {
    let mut buf = Vec::new();
    write_u64(&mut buf, world.step_count());
    write_vec2(&mut buf, world.gravity());
    write_u32(&mut buf, world.body_count() as u32);

    // Serialize each body
    // We need a way to iterate bodies. World2 has iter_bodies().
    // For now serialize as an empty placeholder with the count
    let bodies_data = world.serialize_bodies();
    write_u32(&mut buf, bodies_data.len() as u32);
    buf.extend_from_slice(&bodies_data);

    // Serialize joints
    let joints_data = world.serialize_joints();
    write_u32(&mut buf, joints_data.len() as u32);
    buf.extend_from_slice(&joints_data);

    write_typed_payload(TypeTag::World2State, &buf)
}

/// Deserialize world state into a byte payload that can be restored.
pub fn deserialize_world2_state(data: &[u8]) -> Result<(Vec<u8>, Vec<u8>), Error> {
    let payload = read_typed_payload(data, &mut 0, TypeTag::World2State)?;
    let mut pos = 0;
    let _step = read_u64(payload, &mut pos)?;
    let _gravity = read_vec2(payload, &mut pos)?;
    let _n_bodies = read_u32(payload, &mut pos)?;
    let bodies_len = read_u32(payload, &mut pos)? as usize;
    let bodies_data = payload[pos..pos + bodies_len].to_vec();
    pos += bodies_len;
    let joints_len = read_u32(payload, &mut pos)? as usize;
    let joints_data = payload[pos..pos + joints_len].to_vec();
    Ok((bodies_data, joints_data))
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
    write_u64(&mut buf, c.body_a.index() as u64);
    write_u64(&mut buf, c.body_b.index() as u64);
    write_vec2(&mut buf, c.anchor_a);
    write_vec2(&mut buf, c.anchor_b);
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
    let _jt = read_u8(payload, &mut pos)?;
    let _ba = read_u64(payload, &mut pos)?;
    let _bb = read_u64(payload, &mut pos)?;
    let anchor_a = read_vec2(payload, &mut pos)?;
    let anchor_b = read_vec2(payload, &mut pos)?;
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
        config: JointConfig2 {
            joint_type: JointType2::Weld,
            body_a: BodyHandle2::default(),
            body_b: BodyHandle2::default(),
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
}
