//! Recorded replay structures — pure std, no external deps, engine-generated.
//! Used to generate the watermarked HTML replay viewer (H1 fix).

use auralite_dynamics::{BodyType, World2, World3};
use auralite_math::{Vec2, Vec3};

/// One body snapshot for replay (2D)
#[derive(Clone, Debug)]
pub struct ReplayBody2 {
    pub id: u64,
    pub x: f32,
    pub y: f32,
    pub angle: f32,
    pub sleeping: bool,
    pub kind: u8,    // 0 static, 1 kinematic, 2 dynamic
    pub radius: f32, // approx for rendering
}

/// One frame of 2D replay
#[derive(Clone, Debug)]
pub struct ReplayFrame2 {
    pub step: u64,
    pub sim_time: f32,
    pub hash: u64,
    pub bodies: Vec<ReplayBody2>,
}

/// Collection for one scene (2D)
#[derive(Clone, Debug)]
pub struct SceneReplay2 {
    pub name: String,
    pub frames: Vec<ReplayFrame2>,
}

/// 3D replay body
#[derive(Clone, Debug)]
pub struct ReplayBody3 {
    pub id: u64,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub sleeping: bool,
    pub kind: u8,
    pub radius: f32,
}

#[derive(Clone, Debug)]
pub struct ReplayFrame3 {
    pub step: u64,
    pub sim_time: f32,
    pub hash: u64,
    pub bodies: Vec<ReplayBody3>,
}

#[derive(Clone, Debug)]
pub struct SceneReplay3 {
    pub name: String,
    pub frames: Vec<ReplayFrame3>,
}

/// Unified replay that can hold either 2D or 3D
#[derive(Clone, Debug)]
pub enum SceneReplay {
    Dim2(SceneReplay2),
    Dim3(SceneReplay3),
}

impl SceneReplay {
    pub fn name(&self) -> &str {
        match self {
            SceneReplay::Dim2(s) => &s.name,
            SceneReplay::Dim3(s) => &s.name,
        }
    }
    pub fn frame_count(&self) -> usize {
        match self {
            SceneReplay::Dim2(s) => s.frames.len(),
            SceneReplay::Dim3(s) => s.frames.len(),
        }
    }
}

/// Record World2 state into ReplayBody2
pub fn record_world2(world: &World2) -> Vec<ReplayBody2> {
    let mut out = Vec::new();
    for (_, b) in world.bodies_iter() {
        let kind = match b.kind {
            BodyType::Static => 0,
            BodyType::Kinematic => 1,
            BodyType::Dynamic => 2,
        };
        // Approximate radius from first collider or 0.5
        let radius = b
            .colliders
            .first()
            .map(|c| c.bounding_radius() as f32)
            .unwrap_or(0.5);
        let angle = {
            let v = b.rotation.rotate(Vec2::X);
            v.y.atan2(v.x) as f32
        };
        out.push(ReplayBody2 {
            id: b.id.0,
            x: b.position.x as f32,
            y: b.position.y as f32,
            angle,
            sleeping: b.sleeping,
            kind,
            radius,
        });
    }
    out
}

pub fn record_world3(world: &World3) -> Vec<ReplayBody3> {
    let mut out = Vec::new();
    for (_, b) in world.bodies_iter() {
        let kind = match b.kind {
            BodyType::Static => 0,
            BodyType::Kinematic => 1,
            BodyType::Dynamic => 2,
        };
        let radius = b
            .colliders
            .first()
            .map(|c| c.bounding_radius() as f32)
            .unwrap_or(0.5);
        out.push(ReplayBody3 {
            id: b.id.0,
            x: b.position.x as f32,
            y: b.position.y as f32,
            z: b.position.z as f32,
            sleeping: b.sleeping,
            kind,
            radius,
        });
    }
    out
}

/// Build a JSON string manually (no serde, zero-dep) for embedding in HTML.
/// Structure: { scenes: [ { name, dim:2/3, frames: [ {step, time, hash, bodies:[...]} ] } ] }
pub fn build_replays_json(replays: &[SceneReplay]) -> String {
    let mut json = String::new();
    json.push_str("{\"scenes\":[");
    for (si, scene) in replays.iter().enumerate() {
        if si > 0 {
            json.push(',');
        }
        match scene {
            SceneReplay::Dim2(s) => {
                json.push_str(&format!("{{\"name\":{:?},\"dim\":2,\"frames\":[", s.name));
                for (fi, frame) in s.frames.iter().enumerate() {
                    if fi > 0 {
                        json.push(',');
                    }
                    json.push_str(&format!(
                        "{{\"step\":{},\"time\":{:.4},\"hash\":\"{:016x}\",\"bodies\":[",
                        frame.step, frame.sim_time, frame.hash
                    ));
                    for (bi, b) in frame.bodies.iter().enumerate() {
                        if bi > 0 {
                            json.push(',');
                        }
                        json.push_str(&format!(
                            "{{\"id\":{},\"x\":{:.4},\"y\":{:.4},\"a\":{:.4},\"s\":{},\"k\":{},\"r\":{:.3}}}",
                            b.id, b.x, b.y, b.angle, b.sleeping as u8, b.kind, b.radius
                        ));
                    }
                    json.push_str("]}");
                }
                json.push_str("]}");
            }
            SceneReplay::Dim3(s) => {
                json.push_str(&format!("{{\"name\":{:?},\"dim\":3,\"frames\":[", s.name));
                for (fi, frame) in s.frames.iter().enumerate() {
                    if fi > 0 {
                        json.push(',');
                    }
                    json.push_str(&format!(
                        "{{\"step\":{},\"time\":{:.4},\"hash\":\"{:016x}\",\"bodies\":[",
                        frame.step, frame.sim_time, frame.hash
                    ));
                    for (bi, b) in frame.bodies.iter().enumerate() {
                        if bi > 0 {
                            json.push(',');
                        }
                        json.push_str(&format!(
                            "{{\"id\":{},\"x\":{:.4},\"y\":{:.4},\"z\":{:.4},\"s\":{},\"k\":{},\"r\":{:.3}}}",
                            b.id, b.x, b.y, b.z, b.sleeping as u8, b.kind, b.radius
                        ));
                    }
                    json.push_str("]}");
                }
                json.push_str("]}");
            }
        }
    }
    json.push_str("]}");
    json
}
