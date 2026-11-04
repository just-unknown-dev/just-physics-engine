/// just_physics_engine's 3-D side on its own: [Physics3DEngine] (made by
/// `PhysicsEngineFactory.create3D`, Jolt Physics through just_jolt), the
/// [Physics3DWorld] it makes, and the plain data both speak — shapes,
/// settings, poses, hits and events — with the math types they use.
///
/// Units are metres, seconds and radians; +Y is down, so gravity is
/// (0, +9.81, 0).
library;

export 'package:just_dart/just_dart.dart' show Quaternion, Vector3;

export 'src/box2d/physics_engine_factory.dart' show PhysicsEngineFactory;
export 'src/physics_3d/physics_3d.dart';
