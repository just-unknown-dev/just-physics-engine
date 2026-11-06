/// just_physics_engine
///
/// Standalone physics engine for Flutter with dual 2-D backends:
///   - Native (Android/iOS/Windows/macOS/Linux): Box2D v3.2 via Dart FFI, from just_box_2d
///   - Web: pure-Dart fallback (SAT collision, spatial grid, impulse resolution)
///
/// 2-D (both backends, one API):
/// - [PhysicsEngineFactory]  — `create2D` gives the platform's engine
/// - [Physics2DEngine]       — the engine: bodies, joints, queries, events
/// - [Box2DPhysicsEngine]    — Box2D v3.2 through FFI (native; pure Dart on the web)
/// - [Physics2DBody]         — a body: shared state the game reads and writes
/// - [Shape2D] and its eight shapes ([Circle2DShape], [Rectangle2DShape],
///   [Polygon2DShape], [Capsule2DShape], …), [CollisionManifold]
/// - [Joint2D] and its kinds ([DistanceJoint2D], [RevoluteJoint2D], …;
///   [Box2DJoint] natively), configured through [Joint2D]'s setters
/// - [Ray2D], [Ray2DHit], [CircleCast2DHit], [Shape2DDistance]
/// - [CharacterMover2D] and its queries
///
/// 3D (also on its own in `package:just_physics_engine/physics_3d.dart`):
/// - [Physics3DEngine]       — makes worlds; `PhysicsEngineFactory.create3D`
///                             gives the platform's
/// - [JoltPhysicsEngine]     — Jolt Physics through just_jolt (native only;
///                             the app's pubspec opts in to building it)
/// - [Physics3DWorld]        — one simulation, with int body handles
/// - [Physics3DUnsupportedError] — where 3D cannot run
/// - shapes, settings, poses, hits and events (`Box3DShape`,
///   `Body3DSettings`, `Body3DPoses`, `Query3DHit`, `Physics3DEvent`, …),
///   with the math types they use ([Vector3], [Quaternion])
library;

export 'package:just_dart/just_dart.dart' show Quaternion, Vector3;

export 'src/physics_2d/physics_2d.dart';
export 'src/physics_3d/physics_3d.dart';
export 'src/physics_engine_factory.dart';
