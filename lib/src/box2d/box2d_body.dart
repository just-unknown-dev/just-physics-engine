import 'dart:ffi';

import 'ffi/box2d_library.dart';

// Finalizable is a marker interface required by NativeFinalizer.attach.
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

  // ~1 KB per body (b2BodySim + b2BodyState in Box2D 3.0 internals).
  static final _finalizer = NativeFinalizer(box2dFinalizerBody);

  Box2DBody._({required int handle}) : _handle = handle {
    _finalizer.attach(
      this,
      Pointer<Void>.fromAddress(handle),
      detach: this,
      externalSize: 1024,
    );
  }

  /// Create a dynamic (gravity-affected, collidable) body.
  factory Box2DBody.dynamic({
    required int worldHandle,
    required double posX,
    required double posY,
    double angle = 0.0,
  }) {
    final h = box2d.b2w_createDynamicBody(worldHandle, posX, posY, angle);
    return Box2DBody._(handle: h)
      ..currentX = posX
      ..currentY = posY
      ..currentAngle = angle
      ..prevX = posX
      ..prevY = posY
      ..prevAngle = angle;
  }

  /// Create a static (immovable) body for terrain and walls.
  factory Box2DBody.static({
    required int worldHandle,
    required double posX,
    required double posY,
    double angle = 0.0,
  }) {
    final h = box2d.b2w_createStaticBody(worldHandle, posX, posY, angle);
    return Box2DBody._(handle: h)
      ..currentX = posX
      ..currentY = posY
      ..currentAngle = angle
      ..prevX = posX
      ..prevY = posY
      ..prevAngle = angle;
  }

  /// Create a body of any [BodyType].
  ///
  /// [bodyTypeIndex] is `BodyType.index`, which matches Box2D's `b2BodyType`
  /// ordinals exactly (static=0, kinematic=1, dynamic=2), so no translation
  /// table is needed. The `.dynamic`/`.static` factories above remain for
  /// existing callers.
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

  int get handle => _handle;

  /// Teleport the native body, keeping the interpolation state consistent.
  ///
  /// Writes *both* the current and previous snapshots. Setting only `current`
  /// would leave `prev` at the old location, and TransformInterpolator would
  /// render the body streaking across the level over the following frame —
  /// every respawn would draw a line from the death point to the checkpoint.
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

  /// This body's current [BodyType] index, as Box2D reports it.
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

  void applyForce(double fx, double fy) {
    _throwIfDestroyed();
    box2d.b2w_applyForce(_handle, fx, fy);
  }

  void applyLinearImpulse(double ix, double iy) {
    _throwIfDestroyed();
    box2d.b2w_applyLinearImpulse(_handle, ix, iy);
  }

  void applyTorque(double t) {
    _throwIfDestroyed();
    box2d.b2w_applyTorque(_handle, t);
  }

  void setLinearVelocity(double vx, double vy) {
    _throwIfDestroyed();
    box2d.b2w_setLinearVelocity(_handle, vx, vy);
  }

  void _throwIfDestroyed() {
    if (_destroyed) throw StateError('Box2DBody has already been destroyed.');
  }
}
