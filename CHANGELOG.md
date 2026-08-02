## 1.2.2 - 2026-08-02

Correctness and WASM-compatibility patch release.

### Fixed

- `PhysicsEngineFactory` (`physics_engine_factory.dart`) still used the old `dart.library.html` conditional import that 1.2.1 removed everywhere else. On WASM web builds, where `dart.library.html` is not defined, this resolved to the native Box2D FFI backend instead of the pure-Dart fallback, pulling `dart:ffi` into the web/wasm compile graph and breaking the build. All conditional imports now consistently key off `dart.library.io`.
- Native contact-begin events (`b2w_getContactBeginEvent`) always reported a `(0, 0)` contact normal instead of the real collision normal. Now reads it from `b2Contact_GetData`.
- `Box2DJoint.reactionForce` always returned `Offset.zero` even though the underlying native query (`b2w_getJointReactionForce`) was fully implemented and bound; it just wasn't being called.
- `Box2DPhysicsEngine.dispose()` destroyed native bodies but never called `destroy()` on native joints before tearing down the world, leaving any retained `Box2DJoint` in a stale, not-`_destroyed` state.
- `RevoluteJoint` (pure-Dart) accepted `limitEnabled`/`lowerLimit`/`upperLimit` but never enforced them — angle limits are now applied during `applyConstraint`.
- `ChainShape` was not recognized as a valid collision partner by `CircleShape`, `PolygonShape`, `CapsuleShape`, and `RoundedPolygonShape` — collisions against chain terrain were silently missed depending on which body was added to the engine first. All four now delegate correctly.
- A plain `PolygonShape` colliding against a `RoundedPolygonShape` ignored the other shape's `cornerRadius` when the plain polygon was the "self" side of the check.

#### Native Box2D backend correctness

Found while verifying the fixes above with the full test suite — the native backend was silently producing wrong physics on every device, not just a test artifact:

- **Every fast-moving body was speed-capped far too low.** `b2w_createWorld` never called `b2SetLengthUnitsPerMeter`, so Box2D's tuning constants (`maximumLinearSpeed`, sleep threshold, etc.) used their meters-scale defaults against this package's centimetre-scale convention (`gravityY = 981`). Box2D's "faster than the speed of sound" 400 m/s safety cap resolved to 400 *centimetres*/s — 4 m/s — silently clamping any reasonably fast falling or thrown body. Now set once to 100 (1 m = 100 of this package's units), matching the pure-Dart engine's own tuning (e.g. `PhysicsBody.sleepVelocityThreshold`'s default of 5.0 now lines up with Box2D's resulting 5 cm/s default sleep threshold — not a coincidence).
- **`PhysicsBody.useGravity = false` had no effect** on the native backend — nothing set the native body's `gravityScale`. Added `b2w_setBodyGravityScale`, called from `addBody()`.
- **`PhysicsBody.isAwake` was never synchronized with native Box2D**, in either direction: a body created with `isAwake: false` actually started awake natively, and nothing ever wrote Box2D's real sleep state back to the Dart field, so it stayed stuck at whatever the caller last set. Added `b2w_setBodyAwake` (pushed once at creation) and extended `b2w_bulkExtractTransforms`'s previously-unused padding float to report awake state every step.
- **`PhysicsBody.applyForce()`/`applyTorque()` had no effect** on the native backend — they only wrote to the Dart-only `acceleration`/`torque` fields, which nothing read on this backend. `Box2DPhysicsEngine.update()` now pushes them through `b2w_applyForce`/`b2w_applyTorque` each frame and resets them afterward, matching the pure-Dart engine's per-frame consumption.
- **A dynamic body's actual simulated mass never matched `PhysicsBody.mass`.** Box2D derives mass from shape area × density, and this wrapper always passed a fixed `density = 1.0`, so e.g. a `CircleShape(5.0)` massed ~78.5 regardless of what `PhysicsBody.mass` said — silently distorting every force, impulse, and collision response. Added `b2w_setBodyMass`, which overrides Box2D's shape-derived mass with `PhysicsBody.mass` (scaling rotational inertia proportionally to keep angular response consistent), called after shape fixtures are attached.
- **The test suite hung/crashed partway through** on higher-core-count machines. Each `Box2DWorld` spun up its own `ThreadPool` sized to `hardware_concurrency - 1` (23 threads on a 24-core machine); creating and disposing dozens of worlds in one process (as the test suite does) exhausted OS thread/handle resources. Capped the auto-detected default to 4 workers — callers needing more can still pass an explicit `numThreads`.
- Two tests were themselves written assuming pure-Dart semantics (a single large `update(1.0)` call, or 20 steps of 0.016s) and don't hold against the Box2D backend's fixed-timestep accumulator (which caps at 5 steps per call to prevent spiral-of-death) or its internal fixed 0.5s time-to-sleep constant. Adjusted both to step in a fixed-timestep-compatible way.
- **The native Box2D backend never actually worked on Windows.** `box2d_wrapper.h`/`.cpp` declared all `b2w_*` functions with no export annotation; unlike ELF/Mach-O, Windows DLLs export nothing by default, so `link.exe` silently produced a DLL with an empty export table. Every `dart:ffi` `DynamicLibrary.lookup()` call failed with "procedure not found" even though the `.dll` itself loaded fine. Added a `B2W_EXPORT` macro (`__declspec(dllexport)` on Windows, `__attribute__((visibility("default")))` elsewhere) to every native entry point.
- **Restitution mixing disagreed between backends.** Box2D 3.0's default restitution mixing is `max(a, b)`; the pure-Dart engine mixes with `min(a, b)` (the documented behavior — see the Bounciness demo's "Resolution uses: min(a.restitution, b.restitution)"). A perfectly-restitutive floor (`restitution = 1.0`, relied on so "the ball governs bounce" under min-mixing) instead forced every contact to `max(1.0, ball) == 1.0` on the native backend, making every ball bounce identically regardless of its own restitution. Installed a matching min-mixing `restitutionCallback` on native world creation.
- **Joint motors didn't wake sleeping bodies on the native backend.** `b2*Joint_EnableMotor`/`SetMotor*` only write into the joint's solver state — they never wake the jointed bodies. A body that had fallen asleep (e.g. a car settled on its suspension) had its island skipped by the solver entirely, so `SetMotorSpeed`/`EnableMotor` silently produced no motion until something else disturbed it. Revolute, prismatic, and wheel joint motors now explicitly wake both jointed bodies when enabled.
- **`PhysicsBody.velocity` writes never reached the native simulation.** Nothing bridged Dart → native for velocity, so any ECS-driven velocity change (player input, knockback, etc. — e.g. `PhysicsSystem`'s per-frame push from `VelocityComponent`) was silently discarded on the Box2D backend, and the very next transform sync overwrote it back to whatever native already had. `Box2DPhysicsEngine.update()` now pushes each dynamic body's current `velocity` into the native body before stepping. Likewise, a body's initial `velocity` set at construction time is now carried over to the native body when added — previously it silently started at rest.

### Added

- Pure-Dart `WheelJoint` and unified `addWheelJoint()` on `PhysicsEngine`/`Box2DPhysicsEngine`. Previously `createWheelJoint` only existed on the native Box2D backend with no Dart fallback, unlike every other joint type — contradicting the 1.2.0 changelog's claim of full pure-Dart joint parity.
- Solid-contact begin/end event polling (`pollContactBeginEvents`/`pollContactEndEvents`) on the pure-Dart `PhysicsEngine`. Previously only the Box2D backend exposed contact-begin polling, with no pure-Dart equivalent and no contact-end event at all on either backend.
- One-way / pass-through platform support via `PhysicsBody.isOneWay` on the pure-Dart engine — contacts against an `isOneWay` body are skipped while the dynamic side is moving upward, so a body can pass through from below and only lands when moving downward onto it. Pure-Dart only for now; the Box2D FFI backend has no first-class one-way-platform primitive (would need a native PreSolve contact filter) and currently ignores this field.

### Changed

- The native Box2D submodule now points to `just-unknown-dev/just-box-2d` (a maintained fork) instead of upstream `erincatto/box2d`, at `src/native/third_party/just_box_2d`.

## 1.2.1 - 2026-06-11

Metadata and compatibility patch release focused on clearer pub.dev platform signaling and safer WebAssembly target behavior.

### Added

- Explicit pub.dev platform declarations in package metadata for Android, iOS, Linux, macOS, Web, and Windows.
- Pub.dev topic tags to improve discoverability, including `wasm`.

### Changed

- Updated conditional backend import routing so only `dart.library.io` targets resolve to the native Box2D FFI path.
- Non-IO targets (including WASM and Web) now consistently resolve to the pure-Dart/stub Box2D-compatible surface.
- README compatibility section now reflects the current package/version constraints and platform/backend support matrix.

### Notes

- This release does not introduce physics behavior changes; it improves package metadata accuracy and cross-target compatibility guarantees.

## 1.2.0 - 2026-05-27

Feature release focused on richer 2D authoring/query APIs, compound bodies, and broader Box2D parity for joints and sensors.

### Added

- New 2D collision shapes: `CapsuleShape`, `SegmentShape`, `ChainShape`, and `RoundedPolygonShape`.
- Compound-body support through `PhysicsBody.additionalShapes`, `PhysicsBody.isCompound`, and aggregate bounds handling for broad-phase collision.
- World query APIs: `castRay()`, `castRayAll()`, `castCircle()`, `queryAABB()`, `queryCircle()`, and `queryPoint()`.
- New query result types: `RayBodyHit` and `ShapeCastResult`.
- Joint constraint support for the pure-Dart engine: `DistanceJoint`, `MouseJoint`, `WeldJoint`, `RevoluteJoint`, `PrismaticJoint`, and `WheelJoint` via the unified engine API.
- Native Box2D joint wrappers and configuration helpers through `Box2DJoint` plus creation APIs for revolute, prismatic, distance, weld, and wheel joints.
- Sensor begin/end polling across both backends, plus native Box2D sensor event bridging.
- Expanded collision filtering and body flags with `categoryBits`, `maskBits`, `groupIndex`, and `isBullet` on `PhysicsBody`.
- Comprehensive test coverage for joints, collision primitives, engine lifecycle, and query helpers.

### Changed

- Spatial grid broad-phase now uses compound bounds so multi-shape bodies are culled correctly.
- Native Box2D fixture creation now applies sensor state during shape creation and supports rounded polygon, capsule, segment, and chain fixtures.
- Web-safe Box2D public stubs were expanded to stay import-compatible with the new joint API surface.
- `PhysicsEngine3D` is now explicitly marked experimental.

### Notes

- This release expands the 2D gameplay API significantly while keeping the package centered on a shared pure-Dart and Box2D-backed interface.
- On the Box2D backend, mouse-joint behavior still falls back to the Dart-side spring constraint because Box2D v3 no longer exposes a native mouse joint.

## 1.1.0 - 2026-05-20

Performance-focused update that streamlines 2D collision detection and simplifies the physics body force API.

### Added

- Architecture documentation explaining ECS integration, system lifecycle, and collision resolution pipeline.
- Comprehensive API documentation for all public types and methods.
- Example files demonstrating basic physics setup, rigid-body manipulation, and collision handling.

### Changed

- Optimized SAT collision detection to avoid temporary heap allocations during overlap checks.
- Reworked internal polygon and circle overlap helpers to use inlined double math instead of allocating intermediate vectors and lists.
- Simplified `PhysicsBody.applyForce()` by removing the unused `z` parameter for a cleaner 2D-only API.

### Notes

- This release is a behavior-preserving performance refactor for the 2D physics path.

## 1.0.0 - 2026-05-15

Stable release introducing native Box2D backend support with platform-aware engine selection.

### Added

- Native Box2D v3.0 backend via Dart FFI for Android, iOS, Windows, macOS, and Linux.
- Platform-aware backend selection through `PhysicsEngineFactory.create()`.
- New Box2D API surface: `Box2DPhysicsEngine`, `Box2DWorld`, `Box2DBody`, `PhysicsGameLoop`, and `TransformInterpolator`.
- Native collision/contact bridge utilities including begin-contact polling and impact callback registration.
- Bulk transform extraction from native memory to reduce per-frame overhead.
- Native assets/tooling integration (`hook/build.dart`, `ffigen.yaml`, generated bindings, and C/C++ wrappers).
- Graceful fallback to pure-Dart simulation when native binaries are unavailable.
- Web-safe Box2D public stubs so the package API remains import-compatible on Flutter Web.

### Changed

- Library exports now conditionally expose native or web Box2D APIs.
- `PhysicsEngine` now includes `setGravity(double gx, double gy)` for runtime gravity updates across backends.
- Collision shape math was updated to use `Vector2` helpers directly and remove internal `Offset` extension utilities.

### Notes

- On native platforms, `PhysicsEngineFactory.create()` returns the Box2D backend; on web, it returns the pure-Dart backend.
- Box2D native sources are provided via a git submodule under `src/native/third_party/box2d`.

## 0.1.0 - 2026-05-11

Initial release of just_physics_engine.

### Added

- 2D rigid-body simulation with PhysicsEngine and PhysicsBody.
- Core physical properties including gravity, drag, restitution, friction, torque, and sleeping.
- Collision shapes: CircleShape, RectangleShape, and PolygonShape.
- Broad-phase collision culling via SpatialGrid.
- Collision resolution with impulse response, friction, and positional correction.
- Runtime simulation stats via engine.stats.
- Debug rendering support via engine.renderDebug.
- 2D ray utility helpers with Ray and Ray.fromPoints.
- Initial 3D API scaffolding stubs: PhysicsEngine3D

### Notes

- This release targets Dart SDK ^3.11.0 and Flutter >=1.17.0.
- 2D simulation is production-ready; 3D APIs are currently stubs/scaffolding.

