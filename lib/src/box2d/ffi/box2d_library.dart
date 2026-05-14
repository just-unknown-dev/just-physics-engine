import 'dart:ffi';

import 'box2d_bindings.dart';

final DynamicLibrary _nativeLib = DynamicLibrary.open('box2d_flutter');

/// Singleton FFI binding instance — opened once and reused for the process lifetime.
///
/// Method names match the generated bindings: snake_case `b2w_*`.
/// This file must never be imported on web; use the conditional import in
/// physics_engine_factory.dart to guard the FFI path.
final Box2DBindings box2d = Box2DBindings(_nativeLib);

/// Raw pointer to `b2w_finalizer_world` for use with [NativeFinalizer].
///
/// The function has signature `void(void*)` — compatible with [NativeFinalizer].
/// It receives the packed int64 world handle reinterpreted as `Pointer<Void>`
/// (set via `Pointer.fromAddress(handle)`) and forwards to `b2w_destroyWorld`.
final Pointer<NativeFunction<Void Function(Pointer<Void>)>> box2dFinalizerWorld =
    _nativeLib.lookup<NativeFunction<Void Function(Pointer<Void>)>>(
        'b2w_finalizer_world');

/// Raw pointer to `b2w_finalizer_body` for use with [NativeFinalizer].
final Pointer<NativeFunction<Void Function(Pointer<Void>)>> box2dFinalizerBody =
    _nativeLib.lookup<NativeFunction<Void Function(Pointer<Void>)>>(
        'b2w_finalizer_body');
