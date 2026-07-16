// PBF Fluid compute shader for AuraLite Physics Engine
// WGSL — WebGPU Shading Language

struct Particle {
    position: vec3<f32>,
    velocity: vec3<f32>,
    density: f32,
    lambda: f32,
};

@group(0) @binding(0) var<storage, read_write> particles: array<Particle>;
@group(0) @binding(1) var<uniform> params: FluidParams;

struct FluidParams {
    rest_density: f32,
    kernel_h: f32,
    stiffness: f32,
    dt: f32,
    num_particles: u32,
};

// Poly6 kernel for density
fn w_poly6(r: vec3<f32>, h: f32) -> f32 {
    let r2 = dot(r, r);
    if r2 >= h * h || r2 <= 1e-10 { return 0.0; }
    let h2 = h * h;
    let q = (h2 - r2) / (h2 * h2 * h2);
    return q * (315.0 / (64.0 * 3.14159265));
}

// Spiky gradient kernel
fn grad_w_spiky(r: vec3<f32>, h: f32) -> vec3<f32> {
    let dist = length(r);
    if dist >= h || dist <= 1e-10 { return vec3<f32>(0.0, 0.0, 0.0); }
    let s = (h - dist) / (h * h * h);
    return r / dist * (-45.0 / (3.14159265 * h * h * h * h * h * h)) * s * s;
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) id: vec3<u32>) {
    let i = id.x;
    if i >= params.num_particles { return; }
    
    let pi = particles[i].position;
    let h = params.kernel_h;
    let h2 = h * h;
    
    // Compute density
    var density = w_poly6(vec3<f32>(0.0, 0.0, 0.0), h); // self contribution
    for (var j = 0u; j < params.num_particles; j = j + 1u) {
        if j == i { continue; }
        let pj = particles[j].position;
        let diff = pi - pj;
        if dot(diff, diff) < h2 {
            density = density + w_poly6(diff, h);
        }
    }
    particles[i].density = params.rest_density * density;
}
