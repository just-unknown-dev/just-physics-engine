import '../physics_2d/physics_2d_engine.dart';

/// Returns the pure-Dart [Physics2DEngine] on Flutter Web (including Wasm).
///
/// FFI is not available on web. This stub is selected by the conditional
/// import in physics_engine_factory.dart wherever dart.library.js_interop is
/// (the JS and Wasm web compile targets).
Physics2DEngine createPhysics2DEngine() => Physics2DEngine.pureDart();
