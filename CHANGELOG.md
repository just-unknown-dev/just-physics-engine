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

