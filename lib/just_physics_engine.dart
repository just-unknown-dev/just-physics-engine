/// just_physics_engine
///
/// Standalone physics engine for Flutter with dual backends:
///   - Native (Android/iOS/Windows/macOS/Linux): Box2D v3.2 via Dart FFI, from just_box_2d
///   - Web: pure-Dart fallback (SAT collision, spatial grid, impulse resolution)
///
/// Core types (both backends):
/// - [PhysicsEngineFactory]  — create the best backend for the current platform
/// - [PhysicsEngine]         — pure-Dart engine (also returned on web)
/// - [Box2DPhysicsEngine]    — Box2D v3.2 FFI engine (native; a stub on the web)
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
/// 3D (also on its own in `package:just_physics_engine/physics_3d.dart`):
/// - [PhysicsEngine3D]       — makes worlds; `PhysicsEngineFactory.create3D`
///                             gives the platform's
/// - [JoltPhysicsEngine]     — Jolt Physics through just_jolt (native only;
///                             the app's pubspec opts in to building it)
/// - [PhysicsWorld3D]        — one simulation, with int body handles
/// - [Physics3DUnsupportedError] — where 3D cannot run
/// - shapes, settings, poses, hits and events (`…3D`)
library;

// ── Pure-Dart 2D backend (web fallback, unchanged) ─────────────────────────
export 'src/physics_2d/physics_2d.dart';

// ── 3D: the interface and Jolt ─────────────────────────────────────────────
export 'src/physics_3d/physics_3d.dart';

// ── Box2D FFI backend (native platforms) ───────────────────────────────────
// Native by default, so the analyzer and the API docs see the Box2D classes;
// the web gets same-shaped stubs.
export 'src/box2d/box2d_public_native.dart'
    if (dart.library.js_interop) 'src/box2d/box2d_public_web.dart';
