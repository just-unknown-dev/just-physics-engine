import 'dart:ffi';

import 'box2d_bindings.dart';

DynamicLibrary? _lib;
Box2DBindings? _box2dInstance;
Pointer<NativeFunction<Void Function(Pointer<Void>)>>? _finWorldPtr;
Pointer<NativeFunction<Void Function(Pointer<Void>)>>? _finBodyPtr;

/// Opens 'box2d_flutter' and initialises all FFI singletons.
///
/// Safe to call multiple times — subsequent calls are no-ops.
/// Must be called before accessing [box2d], [box2dFinalizerWorld], or
/// [box2dFinalizerBody]. Placing this call inside a try/catch (as
/// [Box2DPhysicsEngine.initialize] does) allows graceful fallback when the
/// native library has not been compiled yet.
void loadBox2DLibrary() {
  if (_lib != null) return;
  final lib = DynamicLibrary.open('box2d_flutter');
  final bindings = Box2DBindings(lib);
  final finWorld =
      lib.lookup<NativeFunction<Void Function(Pointer<Void>)>>('b2w_finalizer_world');
  final finBody =
      lib.lookup<NativeFunction<Void Function(Pointer<Void>)>>('b2w_finalizer_body');
  // Assign atomically — all-or-nothing so callers never see partial state.
  _box2dInstance = bindings;
  _finWorldPtr = finWorld;
  _finBodyPtr = finBody;
  _lib = lib; // set last: non-null signals fully loaded
}

/// Singleton FFI binding instance — call [loadBox2DLibrary] first.
Box2DBindings get box2d {
  assert(_box2dInstance != null, 'Call loadBox2DLibrary() before using Box2D bindings.');
  return _box2dInstance!;
}

/// Raw pointer to `b2w_finalizer_world` for use with [NativeFinalizer].
Pointer<NativeFunction<Void Function(Pointer<Void>)>> get box2dFinalizerWorld {
  assert(_finWorldPtr != null, 'Call loadBox2DLibrary() before using Box2D bindings.');
  return _finWorldPtr!;
}

/// Raw pointer to `b2w_finalizer_body` for use with [NativeFinalizer].
Pointer<NativeFunction<Void Function(Pointer<Void>)>> get box2dFinalizerBody {
  assert(_finBodyPtr != null, 'Call loadBox2DLibrary() before using Box2D bindings.');
  return _finBodyPtr!;
}
