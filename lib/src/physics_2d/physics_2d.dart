/// 2-D physics: the engine — pure Dart everywhere, Box2D on native
/// platforms — with its bodies, shapes, joints, queries and character
/// mover.
library;

// Native by default, so the analyzer and the API docs see the Box2D
// classes; the web gets same-shaped stubs.
export 'box2d/box2d_joint.dart'
    if (dart.library.js_interop) 'box2d/box2d_joint_stub.dart';
export 'box2d/box2d_physics_engine.dart'
    if (dart.library.js_interop) 'box2d/box2d_physics_engine_stub.dart';
export 'physics_2d_engine.dart' hide BodyPair, SpatialGrid;
export 'ray_2d.dart';
export 'shape_2d_distance.dart';
