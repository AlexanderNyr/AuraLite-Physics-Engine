# Character Controller Guide

Kinematic character controllers (KCC) provide a robust way to move entities through the physics world without the complexity of a purely dynamic body.

## 2D Controller
```rust
let config = CharacterConfig2::default();
let mut cc = Character2::new(config, start_pos);
cc.attach(&mut world);

// Control
cc.set_move(Vec2 { x: 1.0, y: 0.0 });
if space_pressed { cc.jump(); }

// Step
cc.step(dt, &mut world);
```

## Features
- **Slope Limits**: Prevents climbing steep surfaces.
- **Platform Riding**: Automatically inherits velocity from moving platforms.
- **Ray-cast Grounding**: Precise ground detection even on moving geometry.
- **Skin Width**: Prevents jitter and stuck states.
