/// 3-D physics: the [PhysicsEngine3D] interface, its plain data, and Jolt.
library;

export 'jolt/jolt_physics_engine_stub.dart'
    if (dart.library.ffi) 'jolt/jolt_physics_engine.dart';
export 'physics_engine_3d.dart';
export 'results_3d.dart';
export 'settings_3d.dart';
export 'shapes_3d.dart';
