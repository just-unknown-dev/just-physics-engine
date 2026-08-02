import '../physics_2d/physics_engine.dart';

/// Returns the pure-Dart [PhysicsEngine] on Flutter Web (including Wasm).
///
/// FFI is not available on web. This stub is selected by the conditional
/// import in physics_engine_factory.dart whenever dart.library.io is absent
/// (JS and Wasm web compile targets both lack dart:io).
PhysicsEngine createPhysicsEngine() => PhysicsEngine.pureDart();
