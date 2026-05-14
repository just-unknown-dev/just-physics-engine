import '../physics_2d/physics_engine.dart';

/// Returns the pure-Dart [PhysicsEngine] on Flutter Web.
///
/// FFI is not available on web. This stub is selected by the conditional
/// import in physics_engine_factory.dart when dart.library.html is present.
dynamic createPhysicsEngine() => PhysicsEngine();
