import '_box2d_engine_native.dart'
    if (dart.library.html) '_box2d_engine_stub.dart';

/// Platform-conditional factory for the physics engine backend.
///
/// On native platforms (Android, iOS, Windows, macOS, Linux):
///   returns [Box2DPhysicsEngine] — Box2D v3.0 via Dart FFI.
///
/// On Flutter Web:
///   returns [PhysicsEngine] — pure-Dart fallback (no FFI on web).
///
/// Both return types expose the same duck-typed API:
///   initialize(), update(dt), addBody(body), removeBody(body),
///   renderDebug(canvas, size), stats, dispose()
///
/// Usage (the only required change in just_game_engine):
/// ```dart
/// // Before:  physics = PhysicsEngine();
/// // After:
/// physics = PhysicsEngineFactory.create();
/// ```
abstract final class PhysicsEngineFactory {
  /// Create the best available physics engine for the current platform.
  static dynamic create() => createPhysicsEngine();
}
