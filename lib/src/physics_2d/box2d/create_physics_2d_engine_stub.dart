import 'package:meta/meta.dart' show internal;

import '../physics_2d_engine.dart';

/// The web's 2-D engine (JavaScript and WebAssembly, where there is no
/// `dart:ffi`): the pure-Dart [Physics2DEngine]. `PhysicsEngineFactory`
/// picks this file over `create_physics_2d_engine.dart` where
/// `dart.library.js_interop` is.
@internal
Physics2DEngine createPhysics2DEngine() => Physics2DEngine.pureDart();
