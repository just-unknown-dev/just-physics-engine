import 'dart:ffi';

import 'box2d_bindings.dart';

DynamicLibrary? _lib;
Box2DBindings? _box2dInstance;
Pointer<NativeFunction<Void Function(Pointer<Void>)>>? _finWorldPtr;
Pointer<NativeFunction<Void Function(Pointer<Void>)>>? _finBodyPtr;

/// Symbols that must exist for the native backend to be considered usable.
///
/// Deliberately only the ones added after the bindings were first generated —
/// the original surface is covered by the finalizer lookups above, and listing
/// all ~70 would make this a maintenance chore with no extra safety.
const List<String> _requiredSymbols = <String>[
  'b2w_createBody',
  'b2w_setBodyTransform',
  'b2w_setBodyType',
  'b2w_getBodyType',
  'b2w_setBodyOneWay',
  'b2w_castRayClosest',
  'b2w_castRayAll',
  'b2w_queryAABB',
  'b2w_setBodyFilter64',
  'b2w_setBodyFriction',
  'b2w_setBodyRestitution',
  'b2w_setBodyLinearDamping',
];

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

  // Eagerly resolve the symbols added for cross-platform platformer parity.
  //
  // Box2DBindings resolves every function through a `late final` lookup, so a
  // missing symbol normally throws the first time that *specific* function is
  // called. With a stale box2d_flutter.dll that means the library opens fine,
  // _nativeReady is true, stats['backend'] proudly reports 'box2d_v3', and the
  // app dies with "Failed to lookup symbol 'b2w_setBodyTransform'" the first
  // time a player respawns — mid-game, with no fallback.
  //
  // Touching them here turns that into a clean throw inside
  // Box2DPhysicsEngine.initialize()'s try/catch, which falls back to the
  // pure-Dart engine and reports an honest 'dart_fallback'. Cheap insurance
  // against the .dll and the bindings drifting apart.
  for (final symbol in _requiredSymbols) {
    lib.lookup<NativeFunction<Void Function()>>(symbol);
  }

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
