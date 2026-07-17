# Vehicles Guide

## Vehicle3

Ray-cast vehicle with 4-wheel suspension, true shape ray casts with `ray_cast_ignoring`.

`VehicleConfig3 { chassis_mass, chassis_half_extents, center_of_mass_offset, max_engine_torque, ... }`

`WheelConfig3 { suspension_stiffness, damping, max_travel, radius, ... }`

`Vehicle3::new(config, position, rotation, wheel_configs, &mut world)` creates chassis body and wheels.

`set_controls(throttle, steering, brake)`, `step(dt, &mut world)` does ray casts for each wheel, applies suspension forces.

Example in `crates/auralite-sandbox/src/main.rs` scene_vehicle3.

## Vehicle2

Similar for 2D.

## Character Controllers

Slope-aware grounding: `n.dot(Vec3::Y).acos() <= slope_limit`, skin-width stepping, self-filtering via `ray_cast_ignoring`.

`Character2::new(config, position)`, `attach(&mut world)`, `set_move(Vec2)`, `jump()`, `step(dt, &mut world)`, `is_grounded`

`Character3` similar with `Vec3`.

Tests `character2_walks_and_jumps`, `character3_walks`, `character_grounding_works`.

