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

