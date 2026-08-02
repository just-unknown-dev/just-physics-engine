import '../physics_2d/physics_engine.dart';
import 'box2d_physics_engine.dart';

/// Returns a [Box2DPhysicsEngine] on native platforms (Android, iOS,
/// Windows, macOS, Linux).
///
/// Imported conditionally by physics_engine_factory.dart:
///   import '_box2d_engine_stub.dart'
///       if (dart.library.io) '_box2d_engine_native.dart';
PhysicsEngine createPhysicsEngine() => Box2DPhysicsEngine();
