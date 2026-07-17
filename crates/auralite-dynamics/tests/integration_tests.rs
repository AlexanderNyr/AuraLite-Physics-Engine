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
            shape: ColliderShape2::Box(
                auralite_geometry::Box2::new(Vec2 { x: 100.0, y: 0.5 }).unwrap(),
            ),
            offset: Vec2::ZERO,
            material: Material::default(),
            filter: auralite_collision::CollisionFilter::default(),
        });
    world.add_body(ground).unwrap();

    // Add stack
    for i in 0..10 {
        let b = BodyBuilder2::dynamic()
            .position(Vec2 {
                x: 0.0,
                y: 1.0 + i as Real * 1.1,
            })
            .linear_damping(0.05)
            .angular_damping(0.05)
            .add_collider(Collider2 {
                shape: ColliderShape2::Box(
                    auralite_geometry::Box2::new(Vec2 { x: 0.5, y: 0.5 }).unwrap(),
                ),
                offset: Vec2::ZERO,
                material: Material::default(),
                filter: auralite_collision::CollisionFilter::default(),
            });
        world.add_body(b).unwrap();
    }

    // Run for 1000 steps
    let mut initial_hash = 0;
    for step in 0..1000 {
        world.step(1.0 / 60.0).unwrap();
        if step == 0 {
            initial_hash = world.state_hash();
        }
    }

    assert_ne!(initial_hash, world.state_hash());

    // Verify stability
    for h in world.body_handles() {
        let b = world.body(h).unwrap();
        assert!(b.position.is_finite());
        assert!(
            b.velocity.length() < 1.0,
            "vel len: {}",
            b.velocity.length()
        ); // Should have settled or be slowly falling
    }
}

#[test]
fn test_multithreaded_determinism() {
    let build_world = || -> World2 {
        let mut w = World2::default();
        w.add_body(
            BodyBuilder2::static_body()
                .position(Vec2 { x: 0.0, y: -5.0 })
                .add_collider(Collider2 {
                    shape: ColliderShape2::Box(
                        auralite_geometry::Box2::new(Vec2 { x: 50.0, y: 1.0 }).unwrap(),
                    ),
                    offset: Vec2::ZERO,
                    material: Material::default(),
                    filter: auralite_collision::CollisionFilter::default(),
                }),
        )
        .unwrap();
        // Add 25 dynamic bodies overlapping/interacting (`>16 pairs to trigger multithreaded chunk splitting`)
        for i in 0..5 {
            for j in 0..5 {
                w.add_body(
                    BodyBuilder2::dynamic()
                        .position(Vec2 {
                            x: (i as Real - 2.0) * 1.5,
                            y: 2.0 + j as Real * 1.5,
                        })
                        .velocity(Vec2 {
                            x: (j as Real - 2.0) * 0.5,
                            y: -1.0,
                        })
                        .add_collider(Collider2 {
                            shape: ColliderShape2::Circle(
                                auralite_geometry::Circle2::new(0.5).unwrap(),
                            ),
                            offset: Vec2::ZERO,
                            material: Material::default(),
                            filter: auralite_collision::CollisionFilter::default(),
                        }),
                )
                .unwrap();
            }
        }
        w
    };

    let mut w_st = build_world();
    let mut w_mt = build_world();
    let mut st_sched = auralite_core::SingleThreadScheduler;
    let mut mt_sched = auralite_core::ThreadPoolScheduler;

    for _ in 0..100 {
        w_st.step_with_scheduler(0.016, &mut st_sched).unwrap();
        w_mt.step_with_scheduler(0.016, &mut mt_sched).unwrap();
    }

    assert_eq!(
        w_st.state_hash(),
        w_mt.state_hash(),
        "Tier-A ST=MT bitwise determinism must hold across multi-chunk parallel execution vs single-threaded execution"
    );
}
