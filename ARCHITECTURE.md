# Just Physics Engine Architecture

## Overview

`just_physics_engine` is a dual-backend physics package for Flutter:

- Native platforms (Android/iOS/macOS/Windows/Linux): Box2D v3.2 through Dart FFI, for 2D; Jolt Physics, for 3D.
- Web: the pure-Dart 2D engine.

The public libraries:

- `lib/just_physics_engine.dart`: 2D and 3D, and `PhysicsEngineFactory`;
- `lib/physics_3d.dart`: 3D on its own (with the factory, whose `create2D` gives a type only the main library exports);
- `lib/jolt.dart`: Jolt's own classes, native only (they import `dart:ffi`);
- `lib/testing.dart`: `FakePhysics3DEngine`, a plain-Dart 3D engine for tests.

## Design Goals

- One gameplay-facing API on both 2D backends: every `Physics2DEngine`, `Physics2DBody` and `Joint2D` member means the same on each, so game code needs no backend branch.
- Preserve deterministic fixed-step simulation behavior.
- Minimize per-frame allocations to reduce GC pressure.
- Support graceful fallback when native Box2D is unavailable.

## Layout and layers

```
lib/src/
  physics_engine_factory.dart   PhysicsEngineFactory: above both dimensions
  physics_2d/                   the 2D engine (pure Dart), its data, and Box2D
    physics_2d.dart             barrel (Box2D's files conditionally: native | _stub)
    physics_2d_engine.dart      library: Physics2DEngine, with its parts —
                                physics_2d_body, shape_2d, collision_manifold,
                                spatial_grid, physics_2d_contacts, joint_2d,
                                mover_2d, physics_2d_queries, physics_2d_debug_draw
    ray_2d.dart, shape_2d_distance.dart
    box2d/                      the native 2D backend (mirrors physics_3d/jolt/)
      box2d_physics_engine.dart / _stub.dart   Box2DPhysicsEngine
      box2d_joint.dart / _stub.dart            Box2DJoint
      box2d_world.dart, box2d_body.dart        @internal handles
      box2d_shapes.dart, box2d_queries.dart,   @internal: fixtures, queries,
      box2d_events.dart, box2d_poses.dart,     events, pose read-back,
      box2d_debug_draw.dart                    debug drawing
      create_physics_2d_engine.dart / _stub.dart  @internal: the platform's engine
  physics_3d/                   the 3D interface, its data, and Jolt
    physics_3d.dart             barrel (Jolt's engine conditionally)
    physics_3d_engine.dart, physics_3d_results.dart, physics_3d_settings.dart,
    shape_3d.dart
    jolt/                       JoltPhysicsEngine (+ _stub), JoltWorld, JoltShapes
  testing/                      fakes over the 3D interface
    testing.dart                barrel
    fake_physics_3d_engine.dart library, with parts fake_physics_3d_world,
                                fake_body_3d and fake_3d_geometry
```

Layers, bottom to top:

1. `physics_2d`, `physics_3d`: the two dimensions. Neither imports the other.
2. `testing`: imports `physics_3d` only.
3. `src/physics_engine_factory.dart`, the only file directly in `lib/src`: it imports both dimensions' backends (conditionally, native or web), and only the public libraries import it.

Each top-level folder has one barrel, named after it, that exports the whole folder including its subfolder; a library that holds parts is not a barrel. Within a dimension the folder loop is tolerated: `physics_2d.dart` exports `box2d/`, whose files import `physics_2d_engine.dart`, and `physics_3d` does the same with `jolt/`. Every file is exported, or its declarations are `@internal`.

`test/library_layout_test.dart` checks all of it, and two things more: the web never reaches `dart:ffi` (following the `dart.library.js_interop` branch of every conditional directive from the web's libraries), and each `_stub` file declares what its native file adds to the public API. Tests mirror `lib/src` (`test/physics_2d/…`, `test/physics_3d/…`, `test/testing/…`); `test/physics_2d/backends.dart` runs a group on both 2D backends.

## Pure-Dart 2D (`lib/src/physics_2d`)

Core files:

- `physics_2d_engine.dart`: simulation loop, broad-phase invocation, impulse resolution, the body, joint and query API both backends share.
- `physics_2d_body.dart`: rigid body state carrier, forces, and its shapes as distance proxies (`proxies`).
- `shape_2d.dart`: the shapes, and SAT/circle manifold generation (`manifoldWith`, `boundsAt`).
- `spatial_grid.dart`: broad-phase uniform grid with pooled buckets (internal).
- `physics_2d_contacts.dart`: the narrow phase's deepest overlap, one-way platforms, and the impulse that resolves a contact.
- `joint_2d.dart`: `Joint2D` and the pure-Dart joints.
- `collision_manifold.dart`: narrow-phase result object.
- `physics_2d_queries.dart`: the ray tests and the query result types.
- `physics_2d_debug_draw.dart`: `renderDebug`'s drawing.
- `ray_2d.dart`: ray utility.
- `shape_2d_distance.dart`: Box2D's GJK distance and shape cast, ported.
- `mover_2d.dart`: the character mover — its shapes, filter and planes, Box2D's plane solver, `CharacterMover2D`, and the pure-Dart answers to `collideMover` and `castMover`.

Key runtime flow per update:

1. A delta that is not a positive, finite number runs no step. Otherwise integrate active and awake bodies (semi-implicit Euler). `drag` damps the velocity and `angularDamping` the spin, each separately; both factors (`1 - damping * dt`) are clamped at 0 so heavy damping stops a body rather than reversing it. A body with `canSleep == false` never starts its sleep timer.
2. Sync the broad-phase grid and build potential pairs. Pairs that cannot touch are left out before anything is allocated for them: two static bodies, as Box2D does, so a level made of hundreds of static pieces (tile-map collision) costs nothing between its own pieces; and pairs with no dynamic body (kinematic and static, two kinematic) unless one is a sensor. Pairs a joint holds with `collideConnected` off are skipped too, sensors excepted.
3. Compute manifolds and resolve collisions via impulses + friction + positional correction. Materials combine as Box2D's do: friction as the geometric mean, restitution as the less bouncy of the two (the native world installs the same rule), and no bounce below `restitutionThreshold`.
4. Apply the joints, then store frame diagnostics in `stats`.

Allocation strategy highlights:

- Reusable scratch vectors in hot paths.
- Cell buckets pooled, each body's cell range updated in place, and pairs deduplicated by an int key of two small per-grid ids (no allocation for a pair seen twice).
- Contact and sensor sets swapped each step, not copied; shapes walked in place.
- `bodies` and `joints` are live views, not copies; queries walk the bodies' bounds without copying the list.

Inactive bodies (`isActive` false, or `setBodyActive`) are out of everything: not integrated, not in the grid (so their contacts and sensor overlaps end), skipped by queries and movers, and their joints do nothing.

Joints (`Joint2D`) are configured through `setLimits`, `setMotor` and `setSpring` on both backends. The pure-Dart joints take springs in hertz and a damping ratio and apply them as Box2D's soft step does for the engine's fixed step, so a stiff spring holds instead of blowing up. A distance joint holds an anchor on each body (turning with it); the engine is translational, so joints move bodies rather than turn them.

## Native Box2D (`lib/src/physics_2d/box2d`)

Core pieces:

- `Box2DPhysicsEngine`: the backend; it overrides `Physics2DEngine`'s API to run it natively.
- `Box2DJoint`: a native joint, configured through `Joint2D`'s setters by its kind.
- `Box2DWorld` (internal): native world handle + fixed-timestep accumulator.
- `Box2DBody` (internal): native body wrapper with previous/current transform snapshots and what native last reported.
- `box2d_shapes.dart`, `box2d_queries.dart`, `box2d_events.dart`, `box2d_poses.dart`, `box2d_debug_draw.dart` (internal): the fixtures for each `Shape2D`; the ray, AABB and mover queries with their native buffers; each step's events; the one-call pose read-back; debug drawing.

- Box2D itself and the C layer over it (`b2w_*`) live in the **just_box_2d** package, which builds them with its own hook and generates the bindings. The Box2D classes here import `package:just_box_2d/ffi.dart` as `box2d`. This package compiles no C.
- just_box_2d's bindings are top-level `@Native` functions resolved through its code asset `package:just_box_2d/just_box_2d`, so the library is found by asset id on every platform rather than by file name. `Box2D.ensureAvailable()` (in `Box2DPhysicsEngine.initialize`) resolves the finalizers' addresses and the newer functions up front, so a missing or older library falls back to pure Dart at `initialize()` instead of failing mid-game. `initialize()` moves bodies added before it into the native world.

Native synchronization pattern:

1. Before stepping, capture previous body transforms and push what the game wrote since the last step, only where it differs from what Box2D last reported: type, gravity scale, mass, locked rotation and whether the body is in the simulation (`b2w_setBodyEnabled`), then for active bodies whether it is awake, its velocity and spin, and (before a step) its forces.
2. Advance fixed-step Box2D world (0..N steps per frame).
3. Bulk extract transforms in one FFI call to pre-allocated native buffers: 7 floats per body — x, y, angle, vx, vy, awake, angular velocity. A body out of the simulation is not written back: it keeps its velocity for when it is back, as on the pure-Dart engine.
4. Write transformed state back into shared `Physics2DBody` objects.

This write-through keeps ECS/gameplay integrations unchanged.

Per-body flags in the wrapper:

- The wrapper keeps a sensor flag per body handle, read when shape fixtures are created. Handles are reused (a new world, or a freed slot), so `addBody` writes the flag for every body — solid ones too — before adding shapes, and `b2w_destroyBody` erases it. Otherwise a solid body could inherit `true` from a destroyed sensor.
- Bullet, sleep (`b2w_setBodySleepEnabled`), damping and the other body settings are set at `addBody` and again by the matching `setBody*` engine methods at runtime.

Shapes: `Circle2DShape.center` is passed to Box2D as the circle's local centre, so it rotates with the body. The pure-Dart backend does not rotate shapes in its collisions; it offsets the circle by `center` unrotated (its queries and the mover do turn shapes, as Box2D does). A segment, and each segment of a chain, with a thickness goes in as a capsule (Box2D's lines have none); a chain with none goes in as two Box2D chains, one each way round, as a Box2D chain is solid on one side. Every shape has density 1: Box2D gives static and kinematic bodies no mass whatever it is, and `b2w_setBodyMass` sets a dynamic body's.

Joints: anchors are world points as the bodies are when the joint is made, and each joint's frames keep the angle between the bodies (just_box_2d 0.2.0), so a weld or slider between bodies turned apart holds them as they are, and a hinge's limits count from the angle it was made at, as the pure-Dart joints do. `collideConnected` goes to Box2D at creation.

Character movers: `b2w_moverCollide` and `b2w_moverCast` in `box2d_wrapper.cpp` walk the broad phase (`b2World_OverlapAABB`) and run Box2D's own `b2ShapeDistance` and `b2ShapeCast` per shape, with the same rules as the Dart side: sensors and the mover's own body passed over, a one-way shape solid from one side only (its flags are the shape user data the pre-solve callback reads), a chain segment one-sided (a chain with no thickness goes into Box2D both ways round, so it stops a mover from either side, as on the Dart side). Box2D's `b2World_CollideMover`/`CastMover` take only a capsule and cannot pass over one-way shapes or one body, so they are not used. `circleCast`, `queryCircle` and `queryPoint` take their candidates from Box2D's tree and test them exactly in Dart.

## Platform Selection and Fallback

- `PhysicsEngineFactory.create2D()` is the one way to get the platform's engine: it imports `create_physics_2d_engine.dart` or its stub conditionally. `Physics2DEngine` itself imports nothing of Box2D.
- When native init fails (just_box_2d's library not in the build: the app turned it off), `Box2DPhysicsEngine` logs and falls back to pure-Dart update behavior; `isNative` says which runs.
- A game that never needs Box2D (a 3-D game with no 2-D physics) can ask the engine for the pure-Dart engine directly: `EngineConfig(createPhysics: Physics2DEngine.pureDart)` in just_game_engine.
- On web, each native file of `box2d/` has a `_stub` twin with its public API, running pure Dart, so code written against the native classes compiles there too; the layout test keeps them in step.

## Data Model

Primary state object:

- `Physics2DBody` is the shared carrier for gameplay-visible position, velocity, angle, angular velocity, material properties (friction, restitution, `drag`, `angularDamping`), sleep state (`isAwake`, `canSleep`), activity (`isActive`) and shape.
- Fields set before `addBody` reach both backends. To change one afterwards, call the engine's `setBody*` method. The native backend also picks up direct writes of `velocity`, `angularVelocity`, the forces left by `applyForce`/`applyTorque`, `isAwake`, `isActive`, `fixedRotation`, the body type, gravity scale and mass at the next update.

Collision model:

- Broad-phase: the grid returns candidate pairs (internal).
- Narrow-phase: shape-specific manifold generation returns `CollisionManifold`.
- Response: impulse + friction + positional correction based on inverse mass.

## Time Stepping

Pure-Dart backend:

- Uses a fixed `1/60` s timestep with an accumulator, at most 5 sub-steps a frame (`Physics2DEngine.maxSubSteps`), and exposes the remainder as `alpha`. `Physics2DEngine.pureDart(fixedTimestep: false)` integrates the frame's `deltaTime` directly instead. Move events come after a frame that stepped.

Box2D backend:

- Uses fixed timestep `1/60` with accumulator and sub-steps.
- Reads each fixed step's contact, sensor and move events right after it (`Box2DWorld.step`'s `onStep`) and keeps a frame's until the next `update`: Box2D holds the last step's only.
- Hands `applyForce`'s accumulated forces to Box2D as impulses (acceleration × step) before a step, as the pure-Dart engine integrates them, and runs Dart joints (the mouse joint, and joints made before `initialize`) before every native step (`Box2DWorld.step`'s `beforeStep`).
- Exposes interpolation alpha (`accumulator / fixedDt`) for render smoothing.

## Memory and Resource Ownership

- Box2D world and bodies are disposable and guarded with `NativeFinalizer` as a safety net. A finalizer can run on any thread, so it only queues the destroy; the next call from Dart performs it.
- Native handles (just_box_2d's) carry their world's generation as well as Box2D's id, and every `b2w_*` entry point checks them before Box2D sees them (Box2D checks nothing in a release build): a handle of a body, chain, joint or world that has gone does nothing, even after a new world took its slot.
- Explicit `dispose()` remains required for predictable resource release. The impact callback's listener does not keep the isolate alive.
- Native transform, event and query buffers are allocated once, grown when needed and freed by `dispose()`.

## 3D

- `Physics3DEngine` makes `Physics3DWorld`s; the world API is int handles plus batched reads (`readMovedBodies`, 14 floats a body; `readEvents`, 7 floats an event; `readCharacterContacts`, 10 floats a contact), so a step costs a few FFI calls and no object per body or event. `drainEvents` and `characterContacts` give objects, for tests and code that runs rarely.
- A world's collision grid (`layerMasks`) can change while it runs (`setLayerMasks`): just_jolt rebuilds its pair and broad-phase tables and wakes every body that can move, so pairs at rest are judged again.
- `JoltPhysicsEngine` (`src/physics_3d/jolt/`) is the implementation, on just_jolt's bindings: `JoltWorld` (bodies, joints, characters, queries) and `JoltShapes` (shared, reference-counted native shapes; height fields turned to Jolt's +Y up). Worlds ask just_jolt for min restitution, as the 2D engines mix it.
- A `JoltWorld` acts only on the bodies, joints and characters it made itself: a handle from another world, or a character's stand-in body passed to `destroyBody` or `createJoint`, is passed over — Jolt would otherwise destroy that body twice. Shapes are checked (`Shape3D.isValid`) before Jolt sees them.
- Jolt's native library is compiled only when the app asks (`hooks: user_defines: just_jolt: build: true`). The 3D code is reachable only through `PhysicsEngineFactory.create3D` and `JoltPhysicsEngine`, so a 2D app tree-shakes all of it and, without the define, builds no Jolt.
- `create3D` throws `Physics3DUnsupportedError` on the web, on 32-bit Android ABIs and when Jolt was not built.
- `FakePhysics3DEngine` (`testing/`) answers the same interface in plain Dart with boxes. Its handles are numbered per world and tagged with the world's place among its engine's, so the same calls on a new engine give the same handles, and one world's handle names nothing in another.
