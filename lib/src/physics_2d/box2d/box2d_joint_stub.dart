/// The web's `Box2DJoint`: the native file's public API, for code that
/// names the class. The web has no native joints, so none is ever made.
library;

import 'package:meta/meta.dart' show internal;

import '../physics_2d_engine.dart';

/// A joint Box2D solves natively. On the web there is none: the engine
/// makes pure-Dart joints.
class Box2DJoint extends Joint2D {
  // Never called: the web makes no native joints.
  // ignore: unused_element
  Box2DJoint._(super.kind, this.bodyA, this.bodyB)
    : super(collideConnected: false);

  /// The first body.
  final Physics2DBody bodyA;

  /// The second body.
  final Physics2DBody bodyB;

  @override
  List<Physics2DBody> get bodies => [bodyA, bodyB];

  /// Whether the native joint is gone: always, on the web.
  bool get isDestroyed => true;

  /// Destroys the native joint: nothing to do on the web.
  @internal
  void destroy() {}

  @override
  void applyConstraint(double dt) {}
}
