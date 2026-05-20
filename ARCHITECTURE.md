# Just Physics Engine Architecture

## Overview

`just_physics_engine` is a dual-backend physics package for Flutter:

- Native platforms (Android/iOS/macOS/Windows/Linux): Box2D v3.0 through Dart FFI.
- Web: pure-Dart 2D fallback engine.

The public package entry point is `lib/just_physics_engine.dart`, which exports:

- `physics_2d` API (always available)
- `physics_3d` stubs
- platform-conditional Box2D exports (`box2d_public_native.dart` on IO, `box2d_public_web.dart` on web)

## Design Goals

- Keep gameplay-facing API stable across backends.
- Preserve deterministic fixed-step simulation behavior.
- Minimize per-frame allocations to reduce GC pressure.
- Support graceful fallback when native Box2D is unavailable.

## Layered Structure

## 1) Public API Layer

- `PhysicsEngine()` selects the best runtime backend by default.
- `PhysicsEngineFactory.create()` remains available for explicit factory-style construction.
- Consumers use shared types like `PhysicsBody`, `CollisionShape`, and `Ray`.
- Higher-level game code should not need backend-specific branches for normal use.

## 2) Pure-Dart 2D Layer (`lib/src/physics_2d`)

Core files:

- `physics_engine.dart`: simulation loop, broad-phase invocation, impulse resolution, debug rendering.
- `physics_body.dart`: rigid body state carrier and force/impulse helpers.
- `collision_shapes.dart`: SAT/circle manifold generation.
- `spatial_grid.dart`: broad-phase uniform grid with pooled buckets.
- `collision_manifold.dart`: narrow-phase result object.
- `ray_2d.dart`: ray utility.

Key runtime flow per update:

1. Integrate active and awake bodies (semi-implicit Euler).
2. Sync broad-phase grid and build potential pairs.
3. Compute manifolds and resolve collisions via impulses + friction + positional correction.
4. Store frame diagnostics in `stats`.

Allocation strategy highlights:

- Reusable scratch vectors in hot paths.
- Set-backed deduplication of broad-phase pairs.
- Cell bucket pooling in `SpatialGrid`.

## 3) Native Box2D Layer (`lib/src/box2d`)

Core pieces:

- `Box2DPhysicsEngine`: backend adapter that mirrors `PhysicsEngine` API.
- `Box2DWorld`: native world handle + fixed-timestep accumulator.
- `Box2DBody`: native body wrapper with previous/current transform snapshots.
- `PhysicsGameLoop`: lightweight step coordinator that exposes interpolation alpha.
- `TransformInterpolator`: sub-frame interpolation helper.

Native synchronization pattern:

1. Before stepping, capture previous body transforms.
2. Advance fixed-step Box2D world (0..N steps per frame).
3. Bulk extract transforms in one FFI call to pre-allocated native buffers.
4. Write transformed state back into shared `PhysicsBody` objects.

This write-through keeps ECS/gameplay integrations unchanged.

## 4) Platform Selection and Fallback

- `PhysicsEngineFactory` uses conditional imports.
- On native init failure (missing shared lib/submodule issues), `Box2DPhysicsEngine` logs and falls back to pure-Dart update behavior.
- On web, `box2d_public_web.dart` provides compatible stubs where native FFI is not possible.

## Data Model

Primary state object:

- `PhysicsBody` is the shared carrier for gameplay-visible position, velocity, angle, material properties, sleep state, and shape.

Collision model:

- Broad-phase: `SpatialGrid` returns candidate `BodyPair`s.
- Narrow-phase: shape-specific manifold generation returns `CollisionManifold`.
- Response: impulse + friction + positional correction based on inverse mass.

## Time Stepping

Pure-Dart backend:

- Integrates directly using provided `deltaTime`.

Box2D backend:

- Uses fixed timestep `1/60` with accumulator and sub-steps.
- Exposes interpolation alpha (`accumulator / fixedDt`) for render smoothing.

## Memory and Resource Ownership

- Box2D world and bodies are disposable and guarded with `NativeFinalizer` as a safety net.
- Explicit `dispose()` remains required for predictable resource release.
- Native transform buffers are manually allocated/freed and resized with growth strategy.

## 3D Status

- `PhysicsEngine3D` currently exists as a stub API.
- No production 3D simulation pipeline is implemented yet.
