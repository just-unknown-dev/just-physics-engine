import '../physics_2d/physics_engine.dart';

/// Returns the pure-Dart [PhysicsEngine] on Flutter Web (including Wasm).
///
/// FFI is not available on web. This stub is selected by the conditional
/// import in physics_engine_factory.dart wherever dart.library.js_interop is
/// (the JS and Wasm web compile targets).
PhysicsEngine createPhysicsEngine() => PhysicsEngine.pureDart();
