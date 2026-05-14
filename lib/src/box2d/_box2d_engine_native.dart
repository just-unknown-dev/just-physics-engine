import 'box2d_physics_engine.dart';

/// Returns a [Box2DPhysicsEngine] on native platforms (Android, iOS,
/// Windows, macOS, Linux).
///
/// Imported conditionally by physics_engine_factory.dart:
///   import '_box2d_engine_native.dart'
///       if (dart.library.html) '_box2d_engine_stub.dart';
dynamic createPhysicsEngine() => Box2DPhysicsEngine();
