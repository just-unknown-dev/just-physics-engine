import '../physics_2d/physics_engine.dart';
// Native by default (what the analyzer and the API docs see); the web's stubs
// where there is no dart:ffi.
import '../physics_3d/jolt/jolt_physics_engine.dart'
    if (dart.library.js_interop) '../physics_3d/jolt/jolt_physics_engine_stub.dart';
import '../physics_3d/physics_engine_3d.dart';
import '_box2d_engine_native.dart'
    if (dart.library.js_interop) '_box2d_engine_stub.dart';

/// The platform's physics engines.
///
/// [create] gives the 2-D engine: [Box2DPhysicsEngine] (Box2D v3.2 through
/// just_box_2d) on native platforms, which falls back to the pure-Dart
/// engine where the library is not built, and the pure-Dart [PhysicsEngine]
/// on the web. [create3D] gives the 3-D engine, Jolt Physics, where it can
/// run; [supports3D] and [unavailable3DReason] ask first.
///
/// ```dart
/// final physics = PhysicsEngineFactory.create()..initialize();
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
  ///
  /// Finding out starts Jolt where it can run, with the default job system
  /// (cores minus one threads): call [create3D] first when the thread count
  /// matters.
  static String? get unavailable3DReason =>
      JoltPhysicsEngine().unavailableReason;

  /// Whether [create3D] can make an engine here (see [unavailable3DReason]).
  static bool get supports3D => unavailable3DReason == null;
}
