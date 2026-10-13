import 'dart:ffi';

import 'box2d_bindings.dart' as box2d;

/// Whether [loadBox2DLibrary] has found a usable native library.
bool _loaded = false;

Pointer<NativeFunction<Void Function(Pointer<Void>)>>? _finWorldPtr;
Pointer<NativeFunction<Void Function(Pointer<Void>)>>? _finBodyPtr;

/// Symbols that must exist for the native backend to be considered usable.
///
/// Deliberately only the ones added after the bindings were first generated —
/// the original surface is covered by the finalizer lookups below, and listing
/// all ~70 would make this a maintenance chore with no extra safety. Each is
/// in `symbol-address` in ffigen.yaml, so it has an `addresses` getter.
final List<Pointer<NativeType> Function()> _requiredSymbols = [
  () => box2d.addresses.b2w_createBody,
  () => box2d.addresses.b2w_setBodyTransform,
  () => box2d.addresses.b2w_setBodyType,
  () => box2d.addresses.b2w_getBodyType,
  () => box2d.addresses.b2w_setBodyOneWay,
  () => box2d.addresses.b2w_castRayClosest,
  () => box2d.addresses.b2w_castRayAll,
  () => box2d.addresses.b2w_queryAABB,
  () => box2d.addresses.b2w_setBodyFilter64,
  () => box2d.addresses.b2w_setBodyFriction,
  () => box2d.addresses.b2w_setBodyRestitution,
  () => box2d.addresses.b2w_setBodyLinearDamping,
];

/// Checks that the native Box2D library is there and initialises the FFI
/// singletons; throws when it is not.
///
/// The bindings (`box2d_bindings.dart`) are `@Native` functions that resolve
/// through the code asset `package:just_physics_engine/box2d_flutter`, which
/// the build hook bundles — so the library is found by asset id on every
/// platform, Android's `libbox2d_flutter.so` included, instead of by a file
/// name. Resolving an address is the availability probe: it throws when the
/// hook bundled no library (the Box2D sources missing, or the web) or when
/// the library lacks a symbol.
///
/// Safe to call multiple times — subsequent calls are no-ops.
/// Must be called before accessing [box2dFinalizerWorld] or
/// [box2dFinalizerBody]. Placing this call inside a try/catch (as
/// [Box2DPhysicsEngine.initialize] does) allows graceful fallback when the
/// native library has not been compiled yet.
void loadBox2DLibrary() {
  if (_loaded) return;
  final finWorld = box2d.addresses.b2w_finalizer_world;
  final finBody = box2d.addresses.b2w_finalizer_body;

  // Eagerly resolve the symbols added for cross-platform platformer parity.
  //
  // An @Native function is resolved the first time it is called, so a
  // missing symbol normally throws the first time that *specific* function is
  // called. With a stale box2d_flutter library that means the library loads
  // fine, _nativeReady is true, stats['backend'] proudly reports 'box2d_v3',
  // and the app dies with "Couldn't resolve native function
  // 'b2w_setBodyTransform'" the first time a player respawns — mid-game,
  // with no fallback.
  //
  // Touching them here turns that into a clean throw inside
  // Box2DPhysicsEngine.initialize()'s try/catch, which falls back to the
  // pure-Dart engine and reports an honest 'dart_fallback'. Cheap insurance
  // against the library and the bindings drifting apart.
  for (final symbol in _requiredSymbols) {
    symbol();
  }

  // Assign atomically — all-or-nothing so callers never see partial state.
  _finWorldPtr = finWorld;
  _finBodyPtr = finBody;
  _loaded = true; // set last: true signals fully loaded
}

/// Whether the native Box2D library is bundled and usable: [loadBox2DLibrary]
/// without the throw.
bool get isBox2DAvailable {
  try {
    loadBox2DLibrary();
    return true;
  } on Object {
    return false;
  }
}

/// Raw pointer to `b2w_finalizer_world` for use with [NativeFinalizer].
Pointer<NativeFunction<Void Function(Pointer<Void>)>> get box2dFinalizerWorld {
  assert(
    _finWorldPtr != null,
    'Call loadBox2DLibrary() before using Box2D bindings.',
  );
  return _finWorldPtr!;
}

/// Raw pointer to `b2w_finalizer_body` for use with [NativeFinalizer].
Pointer<NativeFunction<Void Function(Pointer<Void>)>> get box2dFinalizerBody {
  assert(
    _finBodyPtr != null,
    'Call loadBox2DLibrary() before using Box2D bindings.',
  );
  return _finBodyPtr!;
}
