import 'dart:ffi';

import 'ffi/box2d_library.dart';

// Finalizable is a marker interface required by NativeFinalizer.attach.
class Box2DWorld implements Finalizable {
  final int _handle;
  bool _disposed = false;

  static const double _fixedDt = 1.0 / 60.0;
  double _accumulator = 0.0;

  /// Number of Box2D sub-steps per fixed tick (recommended: 4).
  final int subSteps;

  /// Render interpolation factor α ∈ [0, 1).
  ///
  /// Computed as `accumulator / fixedDt` after the last [step] call.
  /// Use as: `renderPos = current * alpha + previous * (1.0 - alpha)`.
  double get alpha {
    _throwIfDisposed();
    return _accumulator / _fixedDt;
  }

  // NativeFinalizer safety net: calls `b2w_finalizer_world(token)` if Dart GC
  // collects this object before dispose() is called.
  // externalSize = 50 MB tells the GC the native footprint so it schedules
  // collection appropriately. Prefer explicit dispose() in all normal paths.
  static final _finalizer = NativeFinalizer(box2dFinalizerWorld);

  Box2DWorld._({required int handle, this.subSteps = 4}) : _handle = handle {
    _finalizer.attach(
      this,
      // Reinterpret the packed int64 handle as a Pointer<Void> token.
      // The finalizer C function casts it back: (int64_t)(intptr_t)token.
      Pointer<Void>.fromAddress(_handle),
      detach: this,
      externalSize: 50 * 1024 * 1024,
    );
  }

  /// Create a Box2D world.
  ///
  /// Default gravity `(0, 981)` = 9.81 m/s² at 1 unit = 1 cm.
  /// [numThreads] ≤ 0 → `hardware_concurrency − 1`.
  factory Box2DWorld({
    double gravityX = 0.0,
    double gravityY = 981.0,
    int numThreads = 0,
    int subSteps = 4,
  }) {
    final h = box2d.b2w_createWorld(gravityX, gravityY, numThreads);
    if (h == 0) throw StateError('b2w_createWorld failed — check Box2D build.');
    return Box2DWorld._(handle: h, subSteps: subSteps);
  }

  int get handle {
    _throwIfDisposed();
    return _handle;
  }

  /// Advance the simulation using a fixed-timestep accumulator.
  ///
  /// Caps the accumulator at 5 fixed steps to prevent the spiral-of-death
  /// after long pauses (e.g. debugger breakpoints).
  /// Returns the number of fixed steps fired this call.
  int step(double dt) {
    _throwIfDisposed();
    _accumulator += dt;
    const maxAccum = _fixedDt * 5.0;
    if (_accumulator > maxAccum) _accumulator = maxAccum;

    int steps = 0;
    while (_accumulator >= _fixedDt) {
      box2d.b2w_step(_handle, _fixedDt, subSteps);
      _accumulator -= _fixedDt;
      steps++;
    }
    return steps;
  }

  /// Override world gravity at runtime.
  void setGravity(double gx, double gy) {
    _throwIfDisposed();
    box2d.b2w_setGravity(_handle, gx, gy);
  }

  /// Release the native world and detach the finalizer.
  void dispose() {
    if (_disposed) return;
    _disposed = true;
    _finalizer.detach(this);
    box2d.b2w_destroyWorld(_handle);
  }

  void _throwIfDisposed() {
    if (_disposed) throw StateError('Box2DWorld has been disposed.');
  }
}
