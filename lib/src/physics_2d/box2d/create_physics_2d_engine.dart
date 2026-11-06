import 'package:meta/meta.dart' show internal;

import '../physics_2d_engine.dart';
import 'box2d_physics_engine.dart';

/// The native platforms' 2-D engine: Box2D. `PhysicsEngineFactory.create2D`
/// calls it; the web's twin is in `create_physics_2d_engine_stub.dart`.
@internal
Physics2DEngine createPhysics2DEngine() => Box2DPhysicsEngine();
