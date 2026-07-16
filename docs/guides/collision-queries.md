# Collision Query Guide

AuraLite provides several ways to query the world for collision information.

## Ray Casting
Queries the world for the first intersection with a ray.

```rust
let ray = Ray2::new(origin, direction).unwrap();
if let Some((body_h, t, normal)) = world.ray_cast(ray, 100.0) {
    println!("Hit body {:?} at distance {}", body_h, t);
}
```

## Shape Casting
Sweeps a shape along a translation vector and returns potential candidates.

```rust
let shape_aabb = Aabb2::new(min, max).unwrap();
let candidates = world.dynamic_tree.shape_cast(shape_aabb, translation);
```

## AABB Query
Returns all objects whose bounds overlap an AABB.

```rust
let query_aabb = Aabb2::new(min, max).unwrap();
let overlaps = world.dynamic_tree.query(query_aabb);
```
