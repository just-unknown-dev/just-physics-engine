/// The web's `Box2DPhysicsEngine` (no `dart:ffi`): the native file's
/// public API, running the pure-Dart engine. Keep the two in step: code that
/// compiles against the native class must compile here
/// (`test/library_layout_test.dart` checks the names).
library;

import '../physics_2d_engine.dart';

/// The 2-D engine on native Box2D. On the web there is no Box2D: this runs
/// the pure-Dart engine, and [isNative] is false.
class Box2DPhysicsEngine extends Physics2DEngine {
  /// A Box2D engine; on the web, the pure-Dart engine with this gravity.
  Box2DPhysicsEngine({
    double gravityX = 0.0,
    double gravityY = 981.0,
    this.subSteps = 4,
    this.numThreads = 0,
  }) : super.pureDart() {
    gravity.x = gravityX;
    gravity.y = gravityY;
  }

  /// Box2D sub-steps per step; not used on the web.
  final int subSteps;

  /// Native worker threads; not used on the web.
  final int numThreads;

  @override
  Map<String, dynamic> get stats => {
    ...super.stats,
    'backend': 'dart_fallback_web_stub',
  };

  /// Box2D's hit events: never on the web.
  void registerImpactCallback(
    void Function(int bodyA, int bodyB, double speed) callback, {
    double speedThreshold = 100.0,
  }) {}

  /// A native handle's body: there are none on the web.
  Physics2DBody? bodyFromHandle(int handle) => null;
}
