/// Box2D native joint handle wrapper.
library;

import 'dart:ffi';

import 'package:ffi/ffi.dart';
import 'package:flutter/material.dart' show Offset;

import '../physics_2d/physics_engine.dart';
import 'ffi/box2d_library.dart' show box2d;

/// Wraps a native Box2D joint and implements [JointConstraint] as a no-op
/// (Box2D solves the constraint internally each step).
///
/// Obtain instances via [Box2DPhysicsEngine.createRevoluteJoint] etc.
/// Call [destroy] to release the native joint; after [destroy] all other
/// method calls are silently ignored.
class Box2DJoint extends JointConstraint {
  int _handle;
  bool _destroyed = false;

  Box2DJoint._(super.type, this._handle);

  bool get isDestroyed => _destroyed;

  /// Destroy the native joint. Must be called before the world is disposed.
  void destroy() {
    if (_destroyed) return;
    _destroyed = true;
    box2d.b2w_destroyJoint(_handle);
    _handle = 0;
  }

  /// Box2D handles constraint solving — this is intentionally a no-op.
  @override
  void applyConstraint(double dt) {}

  // ── Revolute joint configuration ─────────────────────────────────────────

  void setRevoluteLimits(double lower, double upper, {bool enable = true}) {
    if (_destroyed) return;
    box2d.b2w_setRevoluteLimits(_handle, lower, upper, enable ? 1 : 0);
  }

  void setRevoluteMotor(
    double speed,
    double maxTorque, {
    bool enable = true,
  }) {
    if (_destroyed) return;
    box2d.b2w_setRevoluteMotor(_handle, speed, maxTorque, enable ? 1 : 0);
  }

  // ── Prismatic joint configuration ─────────────────────────────────────────

  void setPrismaticLimits(double lower, double upper, {bool enable = true}) {
    if (_destroyed) return;
    box2d.b2w_setPrismaticLimits(_handle, lower, upper, enable ? 1 : 0);
  }

  void setPrismaticMotor(
    double speed,
    double maxForce, {
    bool enable = true,
  }) {
    if (_destroyed) return;
    box2d.b2w_setPrismaticMotor(_handle, speed, maxForce, enable ? 1 : 0);
  }

  // ── Distance joint configuration ──────────────────────────────────────────

  void setDistanceLimits(double minLen, double maxLen) {
    if (_destroyed) return;
    box2d.b2w_setDistanceLimits(_handle, minLen, maxLen);
  }

  void setDistanceSpring(double stiffness, double damping) {
    if (_destroyed) return;
    box2d.b2w_setDistanceSpring(_handle, stiffness, damping);
  }

  // ── Mouse joint configuration ─────────────────────────────────────────────

  void setTarget(double x, double y) {
    if (_destroyed) return;
    box2d.b2w_setMouseJointTarget(_handle, x, y);
  }

  // ── Wheel joint configuration ─────────────────────────────────────────────

  void setWheelSpring(double stiffness, double damping) {
    if (_destroyed) return;
    box2d.b2w_setWheelSpring(_handle, stiffness, damping);
  }

  void setWheelMotor(double speed, double maxTorque, {bool enable = true}) {
    if (_destroyed) return;
    box2d.b2w_setWheelMotor(_handle, speed, maxTorque, enable ? 1 : 0);
  }

  // ── Reaction queries ──────────────────────────────────────────────────────

  Offset get reactionForce {
    if (_destroyed) return Offset.zero;
    // Dart doesn't support stack-allocated out-params; use a small allocation.
    // This is an infrequent query call — allocation cost is acceptable.
    final outFx = calloc<Float>();
    final outFy = calloc<Float>();
    try {
      box2d.b2w_getJointReactionForce(_handle, outFx, outFy);
      return Offset(outFx.value, outFy.value);
    } finally {
      calloc
        ..free(outFx)
        ..free(outFy);
    }
  }

  double get reactionTorque {
    if (_destroyed) return 0.0;
    return box2d.b2w_getJointReactionTorque(_handle);
  }

  // ── Internal factory ──────────────────────────────────────────────────────

  static Box2DJoint _wrap(JointType type, int handle) =>
      Box2DJoint._(type, handle);
}

/// Mixin providing joint creation methods for [Box2DPhysicsEngine].
///
/// All methods require the native world and body handles to be valid.
extension Box2DJointFactory on Object {
  static Box2DJoint createRevolute(
    int worldHandle,
    int bodyAHandle,
    int bodyBHandle,
    Offset worldAnchor,
  ) {
    final h = box2d.b2w_createRevoluteJoint(
      worldHandle,
      bodyAHandle,
      bodyBHandle,
      worldAnchor.dx,
      worldAnchor.dy,
    );
    return Box2DJoint._wrap(JointType.revolute, h);
  }

  static Box2DJoint createPrismatic(
    int worldHandle,
    int bodyAHandle,
    int bodyBHandle,
    Offset worldAnchor,
    Offset axis,
  ) {
    final h = box2d.b2w_createPrismaticJoint(
      worldHandle,
      bodyAHandle,
      bodyBHandle,
      worldAnchor.dx,
      worldAnchor.dy,
      axis.dx,
      axis.dy,
    );
    return Box2DJoint._wrap(JointType.prismatic, h);
  }

  static Box2DJoint createDistance(
    int worldHandle,
    int bodyAHandle,
    int bodyBHandle,
    double minLen,
    double maxLen,
  ) {
    final h = box2d.b2w_createDistanceJoint(
      worldHandle,
      bodyAHandle,
      bodyBHandle,
      minLen,
      maxLen,
    );
    return Box2DJoint._wrap(JointType.distance, h);
  }

  static Box2DJoint createMouse(
    int worldHandle,
    int bodyBHandle,
    Offset target,
  ) {
    final h = box2d.b2w_createMouseJoint(
      worldHandle,
      bodyBHandle,
      target.dx,
      target.dy,
    );
    return Box2DJoint._wrap(JointType.mouse, h);
  }

  static Box2DJoint createWeld(
    int worldHandle,
    int bodyAHandle,
    int bodyBHandle,
    Offset worldAnchor,
  ) {
    final h = box2d.b2w_createWeldJoint(
      worldHandle,
      bodyAHandle,
      bodyBHandle,
      worldAnchor.dx,
      worldAnchor.dy,
    );
    return Box2DJoint._wrap(JointType.weld, h);
  }

  static Box2DJoint createWheel(
    int worldHandle,
    int bodyAHandle,
    int bodyBHandle,
    Offset worldAnchor,
    Offset axis,
  ) {
    final h = box2d.b2w_createWheelJoint(
      worldHandle,
      bodyAHandle,
      bodyBHandle,
      worldAnchor.dx,
      worldAnchor.dy,
      axis.dx,
      axis.dy,
    );
    return Box2DJoint._wrap(JointType.wheel, h);
  }
}
