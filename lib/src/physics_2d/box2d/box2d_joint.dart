/// The native Box2D joint.
library;

import 'dart:ffi';

import 'package:ffi/ffi.dart';
import 'package:flutter/painting.dart' show Offset;
import 'package:just_box_2d/ffi.dart' as box2d;
import 'package:meta/meta.dart' show internal;

import '../physics_2d_engine.dart';

/// A joint Box2D solves natively: what `Box2DPhysicsEngine`'s `add*Joint`
/// methods give for the bodies it holds.
///
/// Configure it through [Joint2D]'s setters, which it passes to Box2D by
/// [kind]: the same calls, in the same units, as for the pure-Dart joints.
/// Removing it from its engine ([Physics2DEngine.removeJoint]) destroys it,
/// and so does removing either body; after that every call does nothing.
class Box2DJoint extends Joint2D {
  Box2DJoint._(
    super.kind,
    this._handle,
    this.bodyA,
    this.bodyB, {
    required super.collideConnected,
  });

  int _handle;
  bool _destroyed = false;

  /// The first body.
  final Physics2DBody bodyA;

  /// The second body.
  final Physics2DBody bodyB;

  @override
  List<Physics2DBody> get bodies => [bodyA, bodyB];

  /// Whether the native joint is gone.
  bool get isDestroyed => _destroyed;

  /// Destroys the native joint (its engine does, when it is removed).
  /// Destroying it again, or after its world, does nothing.
  @internal
  void destroy() {
    if (_destroyed) return;
    _destroyed = true;
    box2d.b2w_destroyJoint(_handle);
    _handle = 0;
  }

  /// Box2D handles constraint solving — this is intentionally a no-op.
  @override
  void applyConstraint(double dt) {}

  @override
  void setLimits(double lower, double upper, {bool enable = true}) {
    if (_destroyed) return;
    final on = enable ? 1 : 0;
    switch (kind) {
      case Joint2DKind.revolute:
        box2d.b2w_setRevoluteLimits(_handle, lower, upper, on);
      case Joint2DKind.prismatic:
        box2d.b2w_setPrismaticLimits(_handle, lower, upper, on);
      case Joint2DKind.distance:
        box2d.b2w_setDistanceLimits(_handle, lower, upper);
      case Joint2DKind.mouse || Joint2DKind.weld || Joint2DKind.wheel:
        break;
    }
  }

  @override
  void setMotor(double speed, double maxForceOrTorque, {bool enable = true}) {
    if (_destroyed) return;
    final on = enable ? 1 : 0;
    switch (kind) {
      case Joint2DKind.revolute:
        box2d.b2w_setRevoluteMotor(_handle, speed, maxForceOrTorque, on);
      case Joint2DKind.prismatic:
        box2d.b2w_setPrismaticMotor(_handle, speed, maxForceOrTorque, on);
      case Joint2DKind.wheel:
        box2d.b2w_setWheelMotor(_handle, speed, maxForceOrTorque, on);
      case Joint2DKind.mouse || Joint2DKind.weld || Joint2DKind.distance:
        break;
    }
  }

  @override
  void setSpring(double hertz, double dampingRatio) {
    if (_destroyed) return;
    switch (kind) {
      case Joint2DKind.distance:
        box2d.b2w_setDistanceSpring(_handle, hertz, dampingRatio);
      case Joint2DKind.wheel:
        box2d.b2w_setWheelSpring(_handle, hertz, dampingRatio);
      case Joint2DKind.mouse ||
          Joint2DKind.weld ||
          Joint2DKind.revolute ||
          Joint2DKind.prismatic:
        break;
    }
  }

  @override
  Offset get reactionForce {
    if (_destroyed) return Offset.zero;
    // An infrequent query: a small allocation for the two out-parameters.
    final out = calloc<Float>(2);
    try {
      box2d.b2w_getJointReactionForce(_handle, out, out + 1);
      return Offset(out[0], out[1]);
    } finally {
      calloc.free(out);
    }
  }

  @override
  double get reactionTorque {
    if (_destroyed) return 0.0;
    return box2d.b2w_getJointReactionTorque(_handle);
  }

  // ── Making them ───────────────────────────────────────────────────────────
  //
  // From raw handles, for Box2DPhysicsEngine. Each returns null when Box2D
  // made no joint (a dead handle, or the same body twice).

  static Box2DJoint? _wrap(
    Joint2DKind kind,
    int handle,
    Physics2DBody a,
    Physics2DBody b,
    bool collideConnected,
  ) => handle == 0
      ? null
      : Box2DJoint._(kind, handle, a, b, collideConnected: collideConnected);

  /// A native hinge about [anchor].
  @internal
  static Box2DJoint? revolute(
    int world,
    (Physics2DBody, int) a,
    (Physics2DBody, int) b,
    Offset anchor, {
    required bool collideConnected,
  }) => _wrap(
    Joint2DKind.revolute,
    box2d.b2w_createRevoluteJoint(
      world,
      a.$2,
      b.$2,
      anchor.dx,
      anchor.dy,
      collideConnected ? 1 : 0,
    ),
    a.$1,
    b.$1,
    collideConnected,
  );

  /// A native slider along [axis] through [anchor].
  @internal
  static Box2DJoint? prismatic(
    int world,
    (Physics2DBody, int) a,
    (Physics2DBody, int) b,
    Offset anchor,
    Offset axis, {
    required bool collideConnected,
  }) => _wrap(
    Joint2DKind.prismatic,
    box2d.b2w_createPrismaticJoint(
      world,
      a.$2,
      b.$2,
      anchor.dx,
      anchor.dy,
      axis.dx,
      axis.dy,
      collideConnected ? 1 : 0,
    ),
    a.$1,
    b.$1,
    collideConnected,
  );

  /// A native distance joint from [anchorA] to [anchorB], [minLength] to
  /// [maxLength] long.
  @internal
  static Box2DJoint? distance(
    int world,
    (Physics2DBody, int) a,
    (Physics2DBody, int) b,
    Offset anchorA,
    Offset anchorB,
    double minLength,
    double maxLength, {
    required bool collideConnected,
  }) => _wrap(
    Joint2DKind.distance,
    box2d.b2w_createDistanceJoint(
      world,
      a.$2,
      b.$2,
      anchorA.dx,
      anchorA.dy,
      anchorB.dx,
      anchorB.dy,
      minLength,
      maxLength,
      collideConnected ? 1 : 0,
    ),
    a.$1,
    b.$1,
    collideConnected,
  );

  /// A native weld at [anchor].
  @internal
  static Box2DJoint? weld(
    int world,
    (Physics2DBody, int) a,
    (Physics2DBody, int) b,
    Offset anchor, {
    required bool collideConnected,
  }) => _wrap(
    Joint2DKind.weld,
    box2d.b2w_createWeldJoint(
      world,
      a.$2,
      b.$2,
      anchor.dx,
      anchor.dy,
      collideConnected ? 1 : 0,
    ),
    a.$1,
    b.$1,
    collideConnected,
  );

  /// A native wheel with its axle at [anchor], riding [axis].
  @internal
  static Box2DJoint? wheel(
    int world,
    (Physics2DBody, int) a,
    (Physics2DBody, int) b,
    Offset anchor,
    Offset axis, {
    required bool collideConnected,
  }) => _wrap(
    Joint2DKind.wheel,
    box2d.b2w_createWheelJoint(
      world,
      a.$2,
      b.$2,
      anchor.dx,
      anchor.dy,
      axis.dx,
      axis.dy,
      collideConnected ? 1 : 0,
    ),
    a.$1,
    b.$1,
    collideConnected,
  );
}
