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

1. Integrate active and awake bodies (semi-implicit Euler). `drag` damps the
   velocity and `angularDamping` the spin, each separately; both factors
   (`1 - damping * dt`) are clamped at 0 so heavy damping stops a body rather
   than reversing it. A body with `canSleep == false` never starts its sleep
   timer.
2. Sync broad-phase grid and build potential pairs. Pairs of two static
   bodies are skipped before any narrow-phase work, as Box2D does, so a level
   made of hundreds of static pieces (tile-map collision) costs nothing
   between its own pieces.
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

- `src/native/box2d_wrapper.{h,cpp}`: the C ABI (`b2w_*`) over Box2D that
  `ffi/box2d_bindings.dart` binds. Only this package calls it, so its
  signatures change with the package (see the CHANGELOG's "Native ABI"
  entries).

Native synchronization pattern:

1. Before stepping, capture previous body transforms and push Dart-side
   writes: linear velocity for awake dynamic bodies, and angular velocity
   only when it differs from what Box2D last reported (`Box2DBody.angularVelocity`),
   so an unchanged value never overwrites Box2D's own spin.
2. Advance fixed-step Box2D world (0..N steps per frame).
3. Bulk extract transforms in one FFI call to pre-allocated native buffers:
   7 floats per body — x, y, angle, vx, vy, awake, angular velocity.
4. Write transformed state back into shared `PhysicsBody` objects.

This write-through keeps ECS/gameplay integrations unchanged.

Per-body flags in the wrapper:

- The wrapper keeps a sensor flag per body handle, read when shape fixtures
  are created. Handles are reused (a new world, or a freed slot), so
  `addBody` writes the flag for every body — solid ones too — before adding
  shapes, and `b2w_destroyBody` erases it. Otherwise a solid body could
  inherit `true` from a destroyed sensor.
- Bullet, sleep (`b2w_setBodySleepEnabled`), damping and the other body
  settings are set at `addBody` and again by the matching `setBody*` engine
  methods at runtime.

Shapes: `CircleShape.center` is passed to Box2D as the circle's local centre,
so it rotates with the body. The pure-Dart backend does not rotate shapes;
it offsets the circle by `center` unrotated.

## 4) Platform Selection and Fallback

- `PhysicsEngineFactory` uses conditional imports.
- On native init failure (missing shared lib/submodule issues), `Box2DPhysicsEngine` logs and falls back to pure-Dart update behavior.
- On web, `box2d_public_web.dart` provides compatible stubs where native FFI is not possible.

## Data Model

Primary state object:

- `PhysicsBody` is the shared carrier for gameplay-visible position, velocity, angle, angular velocity, material properties (friction, restitution, `drag`, `angularDamping`), sleep state (`isAwake`, `canSleep`), and shape.
- Fields set before `addBody` reach both backends. To change one afterwards, call the engine's `setBody*` method; writing the field directly only reaches the pure-Dart backend. The exceptions are the per-step motion fields — `velocity`, `angularVelocity`, and the forces left by `applyForce`/`applyTorque` (`acceleration`, `torque`) — which the native backend pushes for every awake dynamic body before each step.

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
