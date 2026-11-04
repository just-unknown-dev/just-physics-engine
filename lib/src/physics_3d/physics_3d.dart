/// 3-D physics: the [Physics3DEngine] interface, its plain data, and Jolt.
library;

export 'jolt/jolt_physics_engine.dart'
    if (dart.library.js_interop) 'jolt/jolt_physics_engine_stub.dart';
export 'physics_3d_engine.dart';
export 'physics_3d_results.dart';
export 'physics_3d_settings.dart';
export 'shape_3d.dart';
