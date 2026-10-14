// The web's side of the Box2D classes (no dart:ffi): the same public API as
// the native files, doing nothing — the pure-Dart engine runs instead — or,
// where a native world is needed, throwing UnsupportedError. Keep the two in
// step: code that compiles against the native classes must compile here.

import 'dart:ui' show Offset;

import '../physics_2d/physics_engine.dart';

export 'physics_engine_factory.dart';

class Box2DWorld {
  Box2DWorld({
    double gravityX = 0.0,
    double gravityY = 981.0,
    int numThreads = 0,
    this.subSteps = 4,
  });

  final int subSteps;

  double get alpha => 0.0;

  int get handle => 0;

  int step(double dt, {void Function()? beforeStep, void Function()? onStep}) =>
      0;

  void setGravity(double gx, double gy) {}

  void dispose() {}
}

class Box2DBody {
  Box2DBody.dynamic({
    required int worldHandle,
    required double posX,
    required double posY,
    double angle = 0.0,
  }) : currentX = posX,
       currentY = posY,
       currentAngle = angle,
       prevX = posX,
       prevY = posY,
       prevAngle = angle;

  Box2DBody.static({
    required int worldHandle,
    required double posX,
    required double posY,
    double angle = 0.0,
  }) : currentX = posX,
       currentY = posY,
       currentAngle = angle,
       prevX = posX,
       prevY = posY,
       prevAngle = angle;

  Box2DBody.ofType({
    required int worldHandle,
    required int bodyTypeIndex,
    required double posX,
    required double posY,
    double angle = 0.0,
  }) : currentX = posX,
       currentY = posY,
       currentAngle = angle,
       prevX = posX,
       prevY = posY,
       prevAngle = angle;

  double prevX;
  double prevY;
  double prevAngle;
  double currentX;
  double currentY;
  double currentAngle;
  double velocityX = 0.0;
  double velocityY = 0.0;
  double angularVelocity = 0.0;

  int get handle => 0;

  void setTransform(double x, double y, double angle, {bool wake = true}) {
    currentX = prevX = x;
    currentY = prevY = y;
    currentAngle = prevAngle = angle;
  }

  int get typeIndex => 0;

  void setType(int bodyTypeIndex) {}

  void capturePrevious() {
    prevX = currentX;
    prevY = currentY;
    prevAngle = currentAngle;
  }

  void destroy() {}

  void applyForce(double fx, double fy) {}

  void applyLinearImpulse(double ix, double iy) {}

  void applyTorque(double t) {}

  void setLinearVelocity(double vx, double vy) {
    velocityX = vx;
    velocityY = vy;
  }
}

class Box2DJoint extends JointConstraint {
  Box2DJoint._(super.type);

  bool get isDestroyed => true;
  void destroy() {}
  @override
  void applyConstraint(double dt) {}
  void setRevoluteLimits(double lower, double upper, {bool enable = true}) {}
  void setRevoluteMotor(double speed, double maxTorque, {bool enable = true}) {}
  void setPrismaticLimits(double lower, double upper, {bool enable = true}) {}
  void setPrismaticMotor(double speed, double maxForce, {bool enable = true}) {}
  void setDistanceLimits(double minLen, double maxLen) {}
  void setDistanceSpring(double hertz, double dampingRatio) {}
  void setWheelSpring(double hertz, double dampingRatio) {}
  void setWheelMotor(double speed, double maxTorque, {bool enable = true}) {}
  Offset get reactionForce => Offset.zero;
  double get reactionTorque => 0.0;
}

extension Box2DJointFactory on Object {
  static Box2DJoint createRevolute(
    int worldHandle,
    int bodyAHandle,
    int bodyBHandle,
    Offset worldAnchor,
  ) => throw UnsupportedError(_noJoints);

  static Box2DJoint createPrismatic(
    int worldHandle,
    int bodyAHandle,
    int bodyBHandle,
    Offset worldAnchor,
    Offset axis,
  ) => throw UnsupportedError(_noJoints);

  static Box2DJoint createDistance(
    int worldHandle,
    int bodyAHandle,
    int bodyBHandle,
    double minLen,
    double maxLen,
  ) => throw UnsupportedError(_noJoints);

  static Box2DJoint createWeld(
    int worldHandle,
    int bodyAHandle,
    int bodyBHandle,
    Offset worldAnchor,
  ) => throw UnsupportedError(_noJoints);

  static Box2DJoint createWheel(
    int worldHandle,
    int bodyAHandle,
    int bodyBHandle,
    Offset worldAnchor,
    Offset axis,
  ) => throw UnsupportedError(_noJoints);

  static const _noJoints =
      'Box2D joints need a native world; the web runs the pure-Dart engine.';
}

class Box2DPhysicsEngine extends PhysicsEngine {
  Box2DPhysicsEngine({
    double gravityX = 0.0,
    double gravityY = 981.0,
    this.subSteps = 4,
    this.numThreads = 0,
  }) : super.pureDart() {
    gravity.x = gravityX;
    gravity.y = gravityY;
  }

  final int subSteps;
  final int numThreads;

  @override
  Map<String, dynamic> get stats => {
    ...super.stats,
    'backend': 'dart_fallback_web_stub',
  };

  Box2DJoint? createRevoluteJoint(
    PhysicsBody bodyA,
    PhysicsBody bodyB,
    Offset worldAnchor,
  ) => null;

  Box2DJoint? createPrismaticJoint(
    PhysicsBody bodyA,
    PhysicsBody bodyB,
    Offset worldAnchor,
    Offset axis,
  ) => null;

  Box2DJoint? createDistanceJoint(
    PhysicsBody bodyA,
    PhysicsBody bodyB, {
    required double minLength,
    required double maxLength,
  }) => null;

  Box2DJoint? createWeldJoint(
    PhysicsBody bodyA,
    PhysicsBody bodyB,
    Offset worldAnchor,
  ) => null;

  Box2DJoint? createWheelJoint(
    PhysicsBody bodyA,
    PhysicsBody bodyB,
    Offset worldAnchor,
    Offset axis,
  ) => null;

  void destroyJoint(Box2DJoint joint) {}

  void registerImpactCallback(
    void Function(int bodyA, int bodyB, double speed) callback, {
    double speedThreshold = 100.0,
  }) {}

  PhysicsBody? physicsBodyFromHandle(int handle) => null;
}

class PhysicsGameLoop {
  PhysicsGameLoop(this.world);

  final Box2DWorld world;

  double get alpha => world.alpha;

  int advance(
    double dt, {
    void Function()? beforeStep,
    void Function()? onStep,
  }) => world.step(dt, beforeStep: beforeStep, onStep: onStep);
}

abstract final class TransformInterpolator {
  static void interpolate({
    required Map<PhysicsBody, Box2DBody> bodyMap,
    required double alpha,
    required void Function(PhysicsBody body, double x, double y, double angle)
    write,
  }) {
    final oneMinusAlpha = 1.0 - alpha;
    for (final entry in bodyMap.entries) {
      final body = entry.value;
      write(
        entry.key,
        body.currentX * alpha + body.prevX * oneMinusAlpha,
        body.currentY * alpha + body.prevY * oneMinusAlpha,
        body.currentAngle * alpha + body.prevAngle * oneMinusAlpha,
      );
    }
  }
}
