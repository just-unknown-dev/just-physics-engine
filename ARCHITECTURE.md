# Just Physics Engine Architecture

## Overview

`just_physics_engine` is a dual-backend physics package for Flutter:

- Native platforms (Android/iOS/macOS/Windows/Linux): Box2D v3.2 through Dart FFI.
- Web: pure-Dart 2D fallback engine.

The public package entry point is `lib/just_physics_engine.dart`, which exports:

- `physics_2d` API (always available)
- `physics_3d`: the 3D interface and its data, and `JoltPhysicsEngine` (conditional: a stub on the web)
- platform-conditional Box2D exports: `box2d_public_native.dart` by default — what the analyzer and the API docs see — and `box2d_public_web.dart`, the same API doing nothing, where `dart.library.js_interop` is (JS and Wasm)

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
- `shape_distance_2d.dart`: Box2D's GJK distance and shape cast, ported.
- `mover_2d.dart`: the character mover — its shapes, filter and planes, Box2D's plane solver, `CharacterMover2D`, and the pure-Dart answers to `collideMover` and `castMover`.

Key runtime flow per update:

1. Integrate active and awake bodies (semi-implicit Euler). `drag` damps the velocity and `angularDamping` the spin, each separately; both factors (`1 - damping * dt`) are clamped at 0 so heavy damping stops a body rather than reversing it. A body with `canSleep == false` never starts its sleep timer.
2. Sync broad-phase grid and build potential pairs. Pairs of two static bodies are skipped before any narrow-phase work, as Box2D does, so a level made of hundreds of static pieces (tile-map collision) costs nothing between its own pieces. So are pairs with no dynamic body (kinematic and static, two kinematic) unless one is a sensor: Box2D makes no contact between them either.
3. Compute manifolds and resolve collisions via impulses + friction + positional correction. Materials combine as Box2D's do: friction as the geometric mean, restitution as the less bouncy of the two (the native world installs the same rule), and no bounce below `restitutionThreshold`.
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

- Box2D itself and the C layer over it (`b2w_*`) live in the **just_box_2d** package, which builds them with its own hook and generates the bindings. The Box2D classes here import `package:just_box_2d/ffi.dart` as `box2d`. This package compiles no C.
- just_box_2d's bindings are top-level `@Native` functions resolved through its code asset `package:just_box_2d/just_box_2d`, so the library is found by asset id on every platform rather than by file name. `Box2D.ensureAvailable()` (in `Box2DPhysicsEngine.initialize`) resolves the finalizers' addresses and the newer functions up front, so a missing or older library falls back to pure Dart at `initialize()` instead of failing mid-game.

Native synchronization pattern:

1. Before stepping, capture previous body transforms and push Dart-side writes: linear velocity for awake dynamic bodies, and angular velocity only when it differs from what Box2D last reported (`Box2DBody.angularVelocity`), so an unchanged value never overwrites Box2D's own spin.
2. Advance fixed-step Box2D world (0..N steps per frame).
3. Bulk extract transforms in one FFI call to pre-allocated native buffers: 7 floats per body — x, y, angle, vx, vy, awake, angular velocity.
4. Write transformed state back into shared `PhysicsBody` objects.

This write-through keeps ECS/gameplay integrations unchanged.

Per-body flags in the wrapper:

- The wrapper keeps a sensor flag per body handle, read when shape fixtures are created. Handles are reused (a new world, or a freed slot), so `addBody` writes the flag for every body — solid ones too — before adding shapes, and `b2w_destroyBody` erases it. Otherwise a solid body could inherit `true` from a destroyed sensor.
- Bullet, sleep (`b2w_setBodySleepEnabled`), damping and the other body settings are set at `addBody` and again by the matching `setBody*` engine methods at runtime.

Shapes: `CircleShape.center` is passed to Box2D as the circle's local centre, so it rotates with the body. The pure-Dart backend does not rotate shapes; it offsets the circle by `center` unrotated.

Character movers: `b2w_moverCollide` and `b2w_moverCast` in `box2d_wrapper.cpp` walk the broad phase (`b2World_OverlapAABB`) and run Box2D's own `b2ShapeDistance` and `b2ShapeCast` per shape, with the same rules as the Dart side: sensors and the mover's own body passed over, a one-way shape solid from one side only (its flags are the shape user data the pre-solve callback reads), a chain segment one-sided. Box2D's `b2World_CollideMover`/`CastMover` take only a capsule and cannot pass over one-way shapes or one body, so they are not used.

## 4) Platform Selection and Fallback

- `PhysicsEngineFactory` uses conditional imports.
- When native init fails (just_box_2d's library not in the build: the app turned it off), `Box2DPhysicsEngine` logs and falls back to pure-Dart update behavior.
- A game that never needs Box2D (a 3-D game with no 2-D physics) can ask the engine for the pure-Dart engine directly: `EngineConfig(createPhysics: PhysicsEngine.pureDart)` in just_game_engine.
- On web, `box2d_public_web.dart` has the native classes' whole API, doing nothing, so code written against them compiles there too. Keep the two in step.

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

- Uses a fixed `1/60` s timestep with an accumulator, at most 5 sub-steps a frame (`PhysicsEngine.maxSubSteps`), and exposes the remainder as `alpha`. `PhysicsEngine.pureDart(fixedTimestep: false)` integrates the frame's `deltaTime` directly instead.

Box2D backend:

- Uses fixed timestep `1/60` with accumulator and sub-steps.
- Exposes interpolation alpha (`accumulator / fixedDt`) for render smoothing.

## Memory and Resource Ownership

- Box2D world and bodies are disposable and guarded with `NativeFinalizer` as a safety net. A finalizer can run on any thread, so it only queues the destroy; the next call from Dart performs it.
- Native handles (just_box_2d's) carry their world's generation as well as Box2D's id, and every `b2w_*` entry point checks them before Box2D sees them (Box2D checks nothing in a release build): a handle of a body, chain, joint or world that has gone does nothing, even after a new world took its slot.
- Explicit `dispose()` remains required for predictable resource release.
- Native transform buffers are manually allocated/freed and resized with growth strategy.

## 3D

- `PhysicsEngine3D` makes `PhysicsWorld3D`s; the world API is int handles plus batched reads (`readMovedBodies`, 14 floats a body) and one event drain per step, so a frame costs a few FFI calls, not one per body.
- `JoltPhysicsEngine` (`src/physics_3d/jolt/`) is the implementation, on just_jolt's bindings: `JoltWorld` (bodies, joints, characters, queries) and `JoltShapes` (shared, reference-counted native shapes; height fields turned to Jolt's +Y up). Worlds ask just_jolt for min restitution, as the 2D engines mix it.
- A `JoltWorld` acts only on the bodies, joints and characters it made itself: a handle from another world, or a character's stand-in body passed to `destroyBody` or `createJoint`, is passed over — Jolt would otherwise destroy that body twice. Shapes are checked (`Shape3D.isValid`) before Jolt sees them.
- Jolt's native library is compiled only when the app asks (`hooks: user_defines: just_jolt: build: true`). The 3D code is reachable only through `PhysicsEngineFactory.create3D` and `JoltPhysicsEngine`, so a 2D app tree-shakes all of it and, without the define, builds no Jolt.
- `create3D` throws `Physics3DUnsupportedError` on the web, on 32-bit Android ABIs and when Jolt was not built.
