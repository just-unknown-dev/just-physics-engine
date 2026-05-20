/// just_physics_engine
///
/// Standalone physics engine for Flutter with dual backends:
///   - Native (Android/iOS/Windows/macOS/Linux): Box2D v3.0 via Dart FFI
///   - Web: pure-Dart fallback (SAT collision, spatial grid, impulse resolution)
///
/// Core types (both backends):
/// - [PhysicsEngineFactory]  — create the best backend for the current platform
/// - [PhysicsEngine]         — pure-Dart engine (also returned on web)
/// - [Box2DPhysicsEngine]    — Box2D v3.0 FFI engine (native only)
/// - [PhysicsBody]           — shared rigid body state carrier
/// - [CollisionShape], [CircleShape], [PolygonShape], [RectangleShape]
/// - [CollisionManifold], [SpatialGrid], [BodyPair]
/// - [RigidBody], [CollisionDetector], [ForceManager]
/// - [Ray]                   — 2D ray descriptor
///
/// Box2D-specific types (native only):
/// - [Box2DWorld]            — native world + fixed-step accumulator
/// - [Box2DBody]             — native body handle with prev/current state
/// - [PhysicsGameLoop]       — fixed-timestep coordinator
/// - [TransformInterpolator] — sub-frame render interpolation (α-blending)
///
/// 3D (stub, not yet implemented):
/// - [PhysicsEngine3D]
library;

// ── Pure-Dart 2D backend (web fallback, unchanged) ─────────────────────────
export 'src/physics_2d/physics_2d.dart';

// ── 3D stubs ───────────────────────────────────────────────────────────────
export 'src/physics_3d/physics_3d.dart';

// ── Box2D FFI backend (native platforms) ───────────────────────────────────
export 'src/box2d/box2d_public_web.dart'
    if (dart.library.io) 'src/box2d/box2d_public_native.dart';
