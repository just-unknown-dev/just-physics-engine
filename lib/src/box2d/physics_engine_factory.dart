import '../physics_2d/physics_engine.dart';
import '../physics_3d/jolt/jolt_physics_engine_stub.dart'
    if (dart.library.ffi) '../physics_3d/jolt/jolt_physics_engine.dart';
import '../physics_3d/physics_engine_3d.dart';
import '_box2d_engine_stub.dart'
    if (dart.library.io) '_box2d_engine_native.dart';

/// Platform-conditional factory for the physics engine backend.
///
/// On native platforms (Android, iOS, Windows, macOS, Linux):
///   returns [Box2DPhysicsEngine] — Box2D v3.0 via Dart FFI.
///
/// On Flutter Web:
///   returns [PhysicsEngine] — pure-Dart fallback (no FFI on web).
///
/// Usage (the only required change in just_game_engine):
/// ```dart
/// // Before:  physics = PhysicsEngine();
/// // After:
/// physics = PhysicsEngineFactory.create();
/// ```
abstract final class PhysicsEngineFactory {
  /// Create the best available physics engine for the current platform.
  static PhysicsEngine create() => createPhysicsEngine();

  /// The platform's 3-D physics engine: Jolt Physics, through just_jolt, its
  /// job system started with [workerThreads] threads (below 0: cores minus
  /// one; the first engine started decides).
  ///
  /// Throws a [Physics3DUnsupportedError] where it cannot run: the web, a
  /// 32-bit Android ABI, or an app whose pubspec does not ask for Jolt to be
  /// built (`hooks: user_defines: just_jolt: build: true`). Nothing 3-D is in
  /// an app that never calls it.
  static PhysicsEngine3D create3D({int workerThreads = -1}) {
    final engine = JoltPhysicsEngine(workerThreads: workerThreads);
    final reason = engine.unavailableReason;
    if (reason != null) throw Physics3DUnsupportedError(reason);
    return engine;
  }

  /// Why [create3D] would throw here, or null when 3-D physics can run.
  static String? get unavailable3DReason => JoltPhysicsEngine().unavailableReason;

  /// Whether [create3D] can make an engine here.
  static bool get supports3D => unavailable3DReason == null;
}
