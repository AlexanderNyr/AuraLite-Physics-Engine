//! Global integration tests for AuraLite Physics Engine.
use auralite_dynamics::*;
use auralite_math::*;

#[test]
fn test_long_running_stacking() {
    let mut world = World2::default();
    
    // Add ground
    let ground = BodyBuilder2::static_body()
        .position(Vec2 { x: 0.0, y: -0.5 })
        .add_collider(Collider2 {
            shape: ColliderShape2::Box(auralite_geometry::Box2::new(Vec2 { x: 100.0, y: 0.5 }).unwrap()),
            offset: Vec2::ZERO,
            material: Material::default(),
            filter: auralite_collision::CollisionFilter::default(),
        });
    world.add_body(ground).unwrap();

    // Add stack
    for i in 0..10 {
        let b = BodyBuilder2::dynamic()
            .position(Vec2 { x: 0.0, y: 1.0 + i as Real * 1.1 })
            .add_collider(Collider2 {
                shape: ColliderShape2::Box(auralite_geometry::Box2::new(Vec2 { x: 0.5, y: 0.5 }).unwrap()),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: auralite_collision::CollisionFilter::default(),
            });
        world.add_body(b).unwrap();
    }

    // Run for 1000 steps
    let mut initial_hash = 0;
    for step in 0..1000 {
        world.step(1.0/60.0).unwrap();
        if step == 0 { initial_hash = world.state_hash(); }
    }
    
    assert_ne!(initial_hash, world.state_hash());
    
    // Verify stability
    for h in world.body_handles() {
        let b = world.body(h).unwrap();
        assert!(b.position.is_finite());
        assert!(b.velocity.length() < 1.0); // Should have settled or be slowly falling
    }
}

#[test]
fn test_multithreaded_determinism() {
    // This test would compare SingleThread vs ThreadPool if we had a toggle in World.
    // For now we verify that the current world (which might use MT if featured) is bitwise deterministic.
    let mut w1 = World2::default();
    let mut w2 = World2::default();
    
    let b = BodyBuilder2::dynamic()
        .position(Vec2 { x: 1.0, y: 2.0 })
        .velocity(Vec2 { x: 3.0, y: 4.0 });
    
    w1.add_body(b.clone()).unwrap();
    w2.add_body(b).unwrap();
    
    for _ in 0..100 {
        w1.step(0.016).unwrap();
        w2.step(0.016).unwrap();
    }
    
    assert_eq!(w1.state_hash(), w2.state_hash());
}
