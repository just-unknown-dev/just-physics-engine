/// The Jolt engine's own classes, for native platforms only (they import
/// `dart:ffi`): [JoltWorld] with its native handle, and [JoltShapes], the
/// shared, reference-counted native shapes. `JoltPhysicsEngine` itself is in
/// `package:just_physics_engine/physics_3d.dart`, which is all most code
/// needs.
library;

export 'src/physics_3d/jolt/jolt_shapes.dart';
export 'src/physics_3d/jolt/jolt_world.dart';
