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

  int step(double dt) => 0;

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

  double prevX;
  double prevY;
  double prevAngle;
  double currentX;
  double currentY;
  double currentAngle;
  double velocityX = 0.0;
  double velocityY = 0.0;

  int get handle => 0;

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
}

class PhysicsGameLoop {
  PhysicsGameLoop(this.world);

  final Box2DWorld world;

  double get alpha => world.alpha;

  int advance(double dt) => world.step(dt);
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
