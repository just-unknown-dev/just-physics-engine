// Above both dimensions: the one file that imports each. Native by default
// (what the analyzer and the API docs see); the web's stubs where there is
// no dart:ffi.
import 'physics_2d/box2d/create_physics_2d_engine.dart'
    if (dart.library.js_interop) 'physics_2d/box2d/create_physics_2d_engine_stub.dart';
import 'physics_2d/physics_2d_engine.dart';
import 'physics_3d/jolt/jolt_physics_engine.dart'
    if (dart.library.js_interop) 'physics_3d/jolt/jolt_physics_engine_stub.dart';
import 'physics_3d/physics_3d_engine.dart';

/// The platform's physics engines.
///
/// [create2D] gives the 2-D engine: `Box2DPhysicsEngine` (Box2D v3.2 through
/// just_box_2d) on native platforms, which falls back to the pure-Dart
/// engine where the library is not built, and the pure-Dart [Physics2DEngine]
/// on the web. [create3D] gives the 3-D engine, Jolt Physics, where it can
/// run; [supports3D] and [unavailable3DReason] ask first.
///
/// `package:just_physics_engine/physics_3d.dart` exports this class too, for
/// its 3-D members; [create2D]'s [Physics2DEngine] is in
/// `package:just_physics_engine/just_physics_engine.dart`.
///
/// ```dart
/// final physics = PhysicsEngineFactory.create2D()..initialize();
/// ```
abstract final class PhysicsEngineFactory {
  /// The platform's 2-D physics engine; call [Physics2DEngine.initialize]
  /// before adding bodies.
  static Physics2DEngine create2D() => createPhysics2DEngine();

  /// The platform's 3-D physics engine: Jolt Physics, through just_jolt, its
  /// job system started with [workerThreads] threads (below 0: cores minus
  /// one; the first engine started decides).
  ///
  /// Throws a [Physics3DUnsupportedError] where it cannot run: the web, a
  /// 32-bit Android ABI, or an app whose pubspec does not ask for Jolt to be
  /// built (`hooks: user_defines: just_jolt: build: true`). Nothing 3-D is in
  /// an app that never calls it.
  static Physics3DEngine create3D({int workerThreads = -1}) {
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
