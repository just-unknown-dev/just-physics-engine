import 'dart:ffi';

import 'package:just_box_2d/ffi.dart' as box2d;
import 'package:meta/meta.dart' show internal;

/// A native Box2D body handle with what `Box2DPhysicsEngine` last read of
/// it. Internal to this package.
///
/// It implements [Finalizable], the marker `NativeFinalizer.attach` needs.
@internal
class Box2DBody implements Finalizable {
  final int _handle;
  bool _destroyed = false;

  // Previous physics-tick state — captured before each step for interpolation.
  double prevX = 0.0;
  double prevY = 0.0;
  double prevAngle = 0.0;

  // Current physics-tick state — written by Box2DPhysicsEngine after each step.
  double currentX = 0.0;
  double currentY = 0.0;
  double currentAngle = 0.0;
  double velocityX = 0.0;
  double velocityY = 0.0;

  /// The spin native reported after the last step, so a different value on
  /// the Dart body is known to be a write to push. The same holds for
  /// [velocityX] and [velocityY].
  double angularVelocity = 0.0;

  /// Whether native reported the body awake after the last step (or was
  /// last told so), so a different value on the Dart body is a write.
  bool awake = true;

  /// Whether the body is in the simulation natively (see
  /// `b2w_setBodyEnabled`).
  bool enabled = true;

  // ~1 KB per body (b2BodySim + b2BodyState in Box2D 3.0 internals).
  static final _finalizer = NativeFinalizer(
    box2d.addresses.b2w_finalizer_body.cast(),
  );

  Box2DBody._({required int handle}) : _handle = handle {
    if (handle == 0) {
      throw StateError('Box2D made no body: its world has gone.');
    }
    _finalizer.attach(
      this,
      Pointer<Void>.fromAddress(handle),
      detach: this,
      externalSize: 1024,
    );
  }

  /// Create a body of any [Body2DType].
  ///
  /// [bodyTypeIndex] is `Body2DType.index`, which matches Box2D's `b2BodyType`
  /// ordinals exactly (static=0, kinematic=1, dynamic=2), so no translation
  /// table is needed.
  factory Box2DBody.ofType({
    required int worldHandle,
    required int bodyTypeIndex,
    required double posX,
    required double posY,
    double angle = 0.0,
  }) {
    final h = box2d.b2w_createBody(
      worldHandle,
      bodyTypeIndex,
      posX,
      posY,
      angle,
    );
    return Box2DBody._(handle: h)
      ..currentX = posX
      ..currentY = posY
      ..currentAngle = angle
      ..prevX = posX
      ..prevY = posY
      ..prevAngle = angle;
  }

  /// The native body's handle.
  int get handle => _handle;

  /// Teleport the native body, keeping the interpolation state consistent.
  ///
  /// Writes *both* the current and previous snapshots. Setting only `current`
  /// would leave `prev` at the old location, and anything interpolating
  /// between them would draw the body streaking across the level over the
  /// following frame.
  void setTransform(double x, double y, double angle, {bool wake = true}) {
    _throwIfDestroyed();
    box2d.b2w_setBodyTransform(_handle, x, y, angle, wake ? 1 : 0);
    currentX = x;
    currentY = y;
    currentAngle = angle;
    prevX = x;
    prevY = y;
    prevAngle = angle;
  }

  /// This body's current [Body2DType] index, as Box2D reports it.
  int get typeIndex {
    _throwIfDestroyed();
    return box2d.b2w_getBodyType(_handle);
  }

  /// Change the body type.
  ///
  /// Box2D recomputes mass properties and rebuilds contacts, and it **resets
  /// any mass override** — callers must re-apply the configured mass after
  /// a real change. [Box2DPhysicsEngine.setBodyType] handles both.
  void setType(int bodyTypeIndex) {
    _throwIfDestroyed();
    box2d.b2w_setBodyType(_handle, bodyTypeIndex);
  }

  /// Snapshot current → previous before each physics step.
  /// Must be called once per frame, before [Box2DWorld.step].
  void capturePrevious() {
    prevX = currentX;
    prevY = currentY;
    prevAngle = currentAngle;
  }

  /// Destroy the native body and detach the finalizer.
  void destroy() {
    if (_destroyed) return;
    _destroyed = true;
    _finalizer.detach(this);
    box2d.b2w_destroyBody(_handle);
  }

  /// Gives the body an impulse at its centre of mass (it wakes).
  void applyLinearImpulse(double ix, double iy) {
    _throwIfDestroyed();
    box2d.b2w_applyLinearImpulse(_handle, ix, iy);
  }

  /// Sets the body's velocity, and remembers it as what native holds.
  void setLinearVelocity(double vx, double vy) {
    _throwIfDestroyed();
    box2d.b2w_setLinearVelocity(_handle, vx, vy);
    velocityX = vx;
    velocityY = vy;
  }

  void _throwIfDestroyed() {
    if (_destroyed) throw StateError('Box2DBody has already been destroyed.');
  }
}
