//! Visualizer and recorded-replay viewer generator for AuraLite worlds.
//! - SvgVisualizer renders real World2/World3 state (engine-driven)
//! - generate_recorded_replay_viewer produces watermarked HTML that plays back ENGINE-RECORDED trajectories + real hashes (H1 fix)
#![forbid(unsafe_code)]

use auralite_dynamics::{BodyType, World2, World3};
use auralite_math::{Vec2, Vec3};

/// SVG visualizer (real engine data, not mocked)
pub struct SvgVisualizer {
    pub width: f32,
    pub height: f32,
    pub scale: f32,
    pub offset: Vec2,
}

impl Default for SvgVisualizer {
    fn default() -> Self {
        Self::new()
    }
}

impl SvgVisualizer {
    pub fn new() -> Self {
        Self {
            width: 800.0,
            height: 600.0,
            scale: 50.0,
            offset: Vec2 { x: 400.0, y: 500.0 },
        }
    }

    pub fn render2d(&self, world: &World2) -> String {
        let mut svg = format!(
            "<svg width=\"{}\" height=\"{}\" viewBox=\"0 0 {} {}\" xmlns=\"http://www.w3.org/2000/svg\">",
            self.width, self.height, self.width, self.height
        );
        svg.push_str("<rect width=\"100%\" height=\"100%\" fill=\"#1a1a1a\" />");

        let gy = self.offset.y;
        svg.push_str(&format!(
            "<line x1=\"0\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"#333333\" stroke-width=\"2\" />",
            gy, self.width, gy
        ));

        for h in world.body_handles() {
            if let Ok(b) = world.body(h) {
                let color = match b.kind {
                    BodyType::Static => "#555555",
                    BodyType::Kinematic => "#44aa99",
                    BodyType::Dynamic => {
                        if b.sleeping {
                            "#444466"
                        } else {
                            "#4499ff"
                        }
                    }
                };
                let angle_deg = rot2_to_degrees(b.rotation);
                for c in &b.colliders {
                    let world_pos = b.position + b.rotation.rotate(c.offset);
                    let screen_x = self.offset.x + world_pos.x * self.scale;
                    let screen_y = self.offset.y - world_pos.y * self.scale;
                    match &c.shape {
                        auralite_dynamics::ColliderShape2::Circle(circ) => {
                            let r = circ.radius() * self.scale;
                            svg.push_str(&format!(
                                "<circle cx=\"{}\" cy=\"{}\" r=\"{}\" fill=\"{}\" opacity=\"0.8\" stroke=\"white\" stroke-width=\"1\" />",
                                screen_x, screen_y, r, color
                            ));
                        }
                        auralite_dynamics::ColliderShape2::Box(bx) => {
                            let hw = bx.half_extents().x * self.scale;
                            let hh = bx.half_extents().y * self.scale;
                            svg.push_str(&format!(
                                "<g transform=\"translate({}, {}) rotate({})\"><rect x=\"{}\" y=\"{}\" width=\"{}\" height=\"{}\" fill=\"{}\" opacity=\"0.8\" stroke=\"white\" stroke-width=\"1\" /></g>",
                                screen_x,
                                screen_y,
                                -angle_deg,
                                -hw,
                                -hh,
                                hw * 2.0,
                                hh * 2.0,
                                color
                            ));
                        }
                        auralite_dynamics::ColliderShape2::Capsule(cap) => {
                            let r = cap.radius * self.scale;
                            let hh = cap.half_height * self.scale;
                            svg.push_str(&format!(
                                "<g transform=\"translate({}, {}) rotate({})\"><circle cx=\"0\" cy=\"{}\" r=\"{}\" fill=\"{}\"/><circle cx=\"0\" cy=\"{}\" r=\"{}\" fill=\"{}\"/><rect x=\"{}\" y=\"{}\" width=\"{}\" height=\"{}\" fill=\"{}\"/></g>",
                                screen_x,
                                screen_y,
                                -angle_deg,
                                -hh,
                                r,
                                color,
                                hh,
                                r,
                                color,
                                -r,
                                -hh,
                                r * 2.0,
                                hh * 2.0,
                                color
                            ));
                        }
                        auralite_dynamics::ColliderShape2::ConvexPolygon(poly) => {
                            let mut pts_str = String::new();
                            for v in poly.vertices() {
                                let wp = b.position + b.rotation.rotate(c.offset + *v);
                                let sx = self.offset.x + wp.x * self.scale;
                                let sy = self.offset.y - wp.y * self.scale;
                                pts_str.push_str(&format!("{},{} ", sx, sy));
                            }
                            svg.push_str(&format!(
                                "<polygon points=\"{}\" fill=\"{}\" opacity=\"0.8\" stroke=\"white\" stroke-width=\"1\" />",
                                pts_str.trim(),
                                color
                            ));
                        }
                        auralite_dynamics::ColliderShape2::Edge(edge) => {
                            let (p1, p2) = edge.endpoints();
                            let wp1 = b.position + b.rotation.rotate(c.offset + p1);
                            let wp2 = b.position + b.rotation.rotate(c.offset + p2);
                            let sx1 = self.offset.x + wp1.x * self.scale;
                            let sy1 = self.offset.y - wp1.y * self.scale;
                            let sx2 = self.offset.x + wp2.x * self.scale;
                            let sy2 = self.offset.y - wp2.y * self.scale;
                            svg.push_str(&format!(
                                "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"white\" stroke-width=\"2\" />",
                                sx1, sy1, sx2, sy2
                            ));
                        }
                    }
                }
            }
        }
        svg.push_str("</svg>");
        svg
    }

    pub fn render3d(&self, world: &World3) -> String {
        let mut svg = format!(
            "<svg width=\"{}\" height=\"{}\" viewBox=\"0 0 {} {}\" xmlns=\"http://www.w3.org/2000/svg\">",
            self.width, self.height, self.width, self.height
        );
        svg.push_str("<rect width=\"100%\" height=\"100%\" fill=\"#1a1a1a\" />");
        let project = |p: Vec3| -> (f32, f32) {
            let sx = self.offset.x + (p.x - p.z * 0.4) * self.scale;
            let sy = self.offset.y - (p.y - p.z * 0.2) * self.scale;
            (sx, sy)
        };
        for (_, b) in world.bodies_iter() {
            let color = match b.kind {
                BodyType::Static => "#555555",
                BodyType::Kinematic => "#44aa99",
                BodyType::Dynamic => {
                    if b.sleeping {
                        "#444466"
                    } else {
                        "#ff8844"
                    }
                }
            };
            for c in &b.colliders {
                let wp = b.position + b.rotation.rotate(c.offset);
                let (sx, sy) = project(wp);
                match &c.shape {
                    auralite_dynamics::ColliderShape3::Sphere(s) => {
                        let r = s.radius * self.scale;
                        svg.push_str(&format!(
                            "<circle cx=\"{}\" cy=\"{}\" r=\"{}\" fill=\"{}\" opacity=\"0.8\" stroke=\"white\" stroke-width=\"1\" />",
                            sx, sy, r, color
                        ));
                    }
                    auralite_dynamics::ColliderShape3::Box(bx) => {
                        let hx = bx.half.x;
                        let hy = bx.half.y;
                        let hz = bx.half.z;
                        let corners = [
                            Vec3 {
                                x: -hx,
                                y: -hy,
                                z: -hz,
                            },
                            Vec3 {
                                x: hx,
                                y: -hy,
                                z: -hz,
                            },
                            Vec3 {
                                x: hx,
                                y: hy,
                                z: -hz,
                            },
                            Vec3 {
                                x: -hx,
                                y: hy,
                                z: -hz,
                            },
                            Vec3 {
                                x: -hx,
                                y: -hy,
                                z: hz,
                            },
                            Vec3 {
                                x: hx,
                                y: -hy,
                                z: hz,
                            },
                            Vec3 {
                                x: hx,
                                y: hy,
                                z: hz,
                            },
                            Vec3 {
                                x: -hx,
                                y: hy,
                                z: hz,
                            },
                        ];
                        let proj_corners: Vec<(f32, f32)> = corners
                            .iter()
                            .map(|&co| project(b.position + b.rotation.rotate(c.offset + co)))
                            .collect();
                        let edges = [
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
                        for (i1, i2) in edges {
                            let (x1, y1) = proj_corners[i1];
                            let (x2, y2) = proj_corners[i2];
                            svg.push_str(&format!(
                                "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"{}\" stroke-width=\"1.5\" />",
                                x1, y1, x2, y2, color
                            ));
                        }
                    }
                    _ => {
                        svg.push_str(&format!(
                            "<circle cx=\"{}\" cy=\"{}\" r=\"5\" fill=\"{}\" />",
                            sx, sy, color
                        ));
                    }
                }
            }
        }
        svg.push_str("</svg>");
        svg
    }
}

fn rot2_to_degrees(r: auralite_math::Rot2) -> f32 {
    let v = r.rotate(Vec2::X);
    v.y.atan2(v.x) * 180.0 / core::f64::consts::PI as f32
}

pub fn generate_recorded_replay_viewer(replays_json: &str) -> String {
    format!(
        r##"<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>AuraLite Physics Engine — Recorded Replay Viewer (Engine-Generated)</title>
<style>
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{ background: #0e0e12; color: #e0e0e8; font-family: 'Segoe UI', sans-serif; display: flex; flex-direction: column; height: 100vh; overflow: hidden; }}
  header {{ background: #1a1a22; padding: 12px 20px; border-bottom: 1px solid #2a2a36; display: flex; align-items: center; justify-content: space-between; }}
  header h1 {{ font-size: 1.1rem; color: #ffcc00; }}
  .watermark {{ position: absolute; top: 50%; left: 50%; transform: translate(-50%,-50%) rotate(-20deg); font-size: 3rem; color: rgba(255, 204, 0, 0.08); font-weight: 900; pointer-events: none; user-select: none; z-index: 10; border: 4px solid rgba(255,204,0,0.08); padding: 10px 20px; }}
  .badge {{ background: #332200; color: #ffcc00; padding: 4px 10px; border-radius: 4px; font-size: 0.75rem; font-weight: bold; border: 1px solid #664400; }}
  .layout {{ display: flex; flex: 1; overflow: hidden; }}
  aside {{ width: 340px; background: #16161c; border-right: 1px solid #2a2a36; display: flex; flex-direction: column; overflow-y: auto; }}
  .panel {{ padding: 14px; border-bottom: 1px solid #22222c; }}
  .panel h2 {{ font-size: 0.8rem; color: #8888a0; text-transform: uppercase; margin-bottom: 8px; }}
  select, button, input[type="range"] {{ width: 100%; padding: 8px; background: #22222e; color: #fff; border: 1px solid #333344; border-radius: 6px; font-size: 0.85rem; margin-bottom: 6px; cursor: pointer; }}
  button.primary {{ background: #1a4a8a; border-color: #3377cc; font-weight: bold; }}
  .stat {{ font-family: monospace; font-size: 0.8rem; line-height: 1.6; }}
  .stat-label {{ color: #8888a0; }}
  .stat-val {{ color: #44ffaa; font-weight: bold; }}
  main {{ flex: 1; position: relative; background: #0c0c0f; display: flex; align-items: center; justify-content: center; }}
  canvas {{ background: #14141a; border: 1px solid #262634; border-radius: 8px; box-shadow: 0 10px 30px rgba(0,0,0,0.5); }}
  .overlay {{ position: absolute; top: 16px; left: 16px; background: rgba(18,18,24,0.9); border: 1px solid #2e2e3e; padding: 10px 14px; border-radius: 8px; pointer-events: none; font-family: monospace; font-size: 0.8rem; backdrop-filter: blur(6px); }}
</style>
</head>
<body>
<header>
  <h1>⚡ AuraLite — Recorded Replay Viewer (Engine-Generated Trajectories)</h1>
  <div><span class="badge">RECORDED REPLAY — NOT LIVE SIMULATION</span> <span class="badge" style="background:#221122;color:#cc88ff;border-color:#664477;margin-left:8px;">REAL STATE HASHES</span></div>
</header>
<div class="layout">
  <aside>
    <div class="panel">
      <h2>Watermark Declaration</h2>
      <div style="background:#332200;border:1px solid #664400;padding:8px;border-radius:6px;color:#ffdd66;font-size:0.8rem;">
        This HTML viewer plays back <b>engine-recorded</b> per-frame trajectories and <b>real 64-bit state hashes</b> produced by <code>cargo run -p auralite-sandbox --release</code>.
        No physics executes in JavaScript. All numbers are from Rust engine execution. See <code>docs/generated/scenes.html</code> regeneration in <code>crates/auralite-sandbox/src/main.rs</code> + <code>replay.rs</code>.
      </div>
      <h2 style="margin-top:12px;">Select Scene (Engine-Recorded)</h2>
      <select id="sceneSelect"></select>
      <div style="display:flex;gap:6px;">
        <button id="btnPlayPause" class="primary" style="flex:1;">⏸ Pause</button>
        <button id="btnReset" style="flex:1;">🔄 Reset</button>
      </div>
      <button id="btnStep">⏭ Step 1 Frame</button>
      <label style="font-size:0.8rem;color:#8888a0;display:block;margin-top:8px;">Frame Scrub: <span id="frameLabel" style="color:#fff;">0 / 0</span></label>
      <input type="range" id="frameSlider" min="0" max="100" value="0" step="1">
      <label style="font-size:0.8rem;color:#8888a0;display:block;margin-top:8px;">Playback Speed: <span id="speedLabel" style="color:#fff;">1.0x</span></label>
      <input type="range" id="speedSlider" min="0.1" max="3.0" step="0.1" value="1.0">
    </div>
    <div class="panel">
      <h2>Frame Info (Real Engine Hash)</h2>
      <div class="stat">
        <div><span class="stat-label">Scene:</span> <span class="stat-val" id="statScene">-</span></div>
        <div><span class="stat-label">Step:</span> <span class="stat-val" id="statStep">0</span></div>
        <div><span class="stat-label">Sim Time:</span> <span class="stat-val" id="statTime">0.00 s</span></div>
        <div><span class="stat-label">Bodies:</span> <span class="stat-val" id="statBodies">0</span></div>
        <div><span class="stat-label">State Hash (FNV-1a real):</span> <span class="stat-val" id="statHash" style="color:#ffdd44;">0x0</span></div>
      </div>
    </div>
    <div class="panel">
      <h2>Provenance</h2>
      <div style="font-size:0.75rem;color:#8888a0;line-height:1.5;">
        Generated by <code>auralite-sandbox</code> headless runner. Each frame's body positions and hash are from <code>World2::state_hash()</code> / <code>World3::state_hash()</code> during real stepping. No pseudo-hash, no counter*1337.
        Regenerate via: <code>cargo run -p auralite-sandbox --release</code> → <code>docs/generated/scenes.html</code> (single canonical path).
      </div>
    </div>
  </aside>
  <main>
    <div class="watermark">RECORDED REPLAY — NOT LIVE SIMULATION</div>
    <canvas id="canvas" width="900" height="680"></canvas>
    <div class="overlay">
      <div>Frame: <span id="overlayFrame" style="color:#44ffaa;">0</span> / <span id="overlayTotal">0</span></div>
      <div>Hash: <span id="overlayHash" style="color:#ffdd44;">0x0</span></div>
      <div style="margin-top:6px;color:#8888a0;font-size:0.7rem;">Engine-recorded playback</div>
    </div>
  </main>
</div>
<script>
const REPLAY_DATA = {json_data};

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
let currentSceneIdx = 0;
let currentFrameIdx = 0;
let playing = true;
let playbackSpeed = 1.0;
let lastTime = 0;
let accum = 0;

function initScenes() {{
  const sel = document.getElementById('sceneSelect');
  REPLAY_DATA.scenes.forEach((s, idx) => {{
    const opt = document.createElement('option');
    opt.value = idx;
    opt.textContent = `${{idx+1}}. ${{s.name}} (${{s.dim}}D, ${{s.frames.length}} frames)`;
    sel.appendChild(opt);
  }});
  sel.addEventListener('change', (e) => {{
    currentSceneIdx = parseInt(e.target.value);
    currentFrameIdx = 0;
    updateUI();
    draw();
  }});
}}

function getCurrentScene() {{
  return REPLAY_DATA.scenes[currentSceneIdx];
}}
function getCurrentFrame() {{
  const sc = getCurrentScene();
  if (!sc || sc.frames.length===0) return null;
  return sc.frames[Math.min(currentFrameIdx, sc.frames.length-1)];
}}

function updateUI() {{
  const scene = getCurrentScene();
  const frame = getCurrentFrame();
  if (!scene || !frame) return;
  document.getElementById('statScene').textContent = scene.name;
  document.getElementById('statStep').textContent = frame.step;
  document.getElementById('statTime').textContent = frame.time.toFixed(3) + ' s';
  document.getElementById('statBodies').textContent = frame.bodies.length;
  document.getElementById('statHash').textContent = '0x' + frame.hash;
  document.getElementById('overlayFrame').textContent = currentFrameIdx;
  document.getElementById('overlayTotal').textContent = scene.frames.length-1;
  document.getElementById('overlayHash').textContent = '0x' + frame.hash;
  document.getElementById('frameLabel').textContent = `${{currentFrameIdx}} / ${{scene.frames.length-1}}`;
  const slider = document.getElementById('frameSlider');
  slider.max = scene.frames.length-1;
  slider.value = currentFrameIdx;
}}

function draw() {{
  const scene = getCurrentScene();
  const frame = getCurrentFrame();
  if (!scene || !frame) return;
  ctx.clearRect(0,0,canvas.width,canvas.height);
  ctx.strokeStyle = '#333344';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(40, 600);
  ctx.lineTo(860, 600);
  ctx.stroke();

  if (scene.dim === 2) {{
    const scale = 22;
    const offX = 450;
    const offY = 550;
    frame.bodies.forEach(b => {{
      const sx = offX + b.x * scale;
      const sy = offY - b.y * scale;
      const sleeping = b.s;
      const kind = b.k;
      let color = kind===0 ? '#555555' : kind===1 ? '#44aa99' : (sleeping ? '#444466' : '#4499ff');
      if (kind===2 && scene.name.includes('3D')) color = '#ff8844';
      ctx.fillStyle = color;
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 1;
      const r = Math.max(3, b.r * scale * 0.6);
      ctx.beginPath();
      ctx.arc(sx, sy, r, 0, Math.PI*2);
      ctx.fill();
      ctx.stroke();
      if (b.a !== undefined) {{
        ctx.strokeStyle = '#ffffff';
        ctx.beginPath();
        ctx.moveTo(sx, sy);
        ctx.lineTo(sx + Math.cos(b.a)*r, sy - Math.sin(b.a)*r);
        ctx.stroke();
      }}
      if (sleeping) {{
        ctx.fillStyle = '#8888ff';
        ctx.fillRect(sx-2, sy-2, 4, 4);
      }}
    }});
  }} else {{
    const scale = 18;
    const offX = 450;
    const offY = 500;
    const project = (x,y,z) => {{
      const sx = offX + (x - z*0.4)*scale;
      const sy = offY - (y - z*0.2)*scale;
      return [sx, sy];
    }};
    frame.bodies.forEach(b => {{
      const [sx, sy] = project(b.x, b.y, b.z);
      const color = b.k===0 ? '#555555' : b.k===1 ? '#44aa99' : (b.s ? '#444466' : '#ff8844');
      ctx.fillStyle = color;
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1;
      const r = Math.max(3, b.r * scale * 0.5);
      ctx.beginPath();
      ctx.arc(sx, sy, r, 0, Math.PI*2);
      ctx.fill();
      ctx.stroke();
    }});
  }}
}}

document.getElementById('btnPlayPause').addEventListener('click', (e) => {{
  playing = !playing;
  e.target.textContent = playing ? '⏸ Pause' : '▶ Play';
}});
document.getElementById('btnReset').addEventListener('click', () => {{
  currentFrameIdx = 0;
  updateUI();
  draw();
}});
document.getElementById('btnStep').addEventListener('click', () => {{
  const scene = getCurrentScene();
  if (!scene) return;
  if (currentFrameIdx < scene.frames.length-1) {{
    currentFrameIdx++;
    updateUI();
    draw();
  }}
}});
document.getElementById('frameSlider').addEventListener('input', (e) => {{
  currentFrameIdx = parseInt(e.target.value);
  updateUI();
  draw();
}});
document.getElementById('speedSlider').addEventListener('input', (e) => {{
  playbackSpeed = parseFloat(e.target.value);
  document.getElementById('speedLabel').textContent = playbackSpeed.toFixed(1) + 'x';
}});

function loop(ts) {{
  if (!lastTime) lastTime = ts;
  const dt = ts - lastTime;
  lastTime = ts;
  if (playing) {{
    accum += dt * playbackSpeed;
    if (accum > 1000/60) {{
      accum = 0;
      const scene = getCurrentScene();
      if (scene) {{
        currentFrameIdx = (currentFrameIdx + 1) % scene.frames.length;
        updateUI();
        draw();
      }}
    }}
  }}
  requestAnimationFrame(loop);
}}

initScenes();
updateUI();
draw();
requestAnimationFrame(loop);
</script>
</body>
</html>
"##,
        json_data = replays_json
    )
}
