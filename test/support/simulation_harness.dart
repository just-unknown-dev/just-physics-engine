import 'dart:math' as math;

import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

const double fixedStep60 = 1.0 / 60.0;

PhysicsEngine buildDeterministicScenario({
  required PhysicsEngine engine,
  required int seed,
  required int dynamicBodyCount,
}) {
  final random = math.Random(seed);

  engine.initialize();

  // World bounds.
  engine.addBody(
    PhysicsBody(
      position: Vector2(0, 900),
      shape: RectangleShape(4000, 80),
      mass: 0,
      friction: 0.8,
      restitution: 0.1,
    ),
  );

  engine.addBody(
    PhysicsBody(
      position: Vector2(-2000, 300),
      shape: RectangleShape(80, 2000),
      mass: 0,
      friction: 0.8,
      restitution: 0.1,
    ),
  );

  engine.addBody(
    PhysicsBody(
      position: Vector2(2000, 300),
      shape: RectangleShape(80, 2000),
      mass: 0,
      friction: 0.8,
      restitution: 0.1,
    ),
  );

  for (var i = 0; i < dynamicBodyCount; i++) {
    final col = i % 24;
    final row = i ~/ 24;

    final body = PhysicsBody(
      position: Vector2(
        -1000 + col * 84 + (random.nextDouble() - 0.5) * 6,
        -2000 + row * 72 + (random.nextDouble() - 0.5) * 6,
      ),
      shape: CircleShape(12 + (i % 3).toDouble()),
      velocity: Vector2(
        (random.nextDouble() - 0.5) * 20,
        (random.nextDouble() - 0.5) * 20,
      ),
      mass: 1.0 + (i % 4) * 0.25,
      friction: 0.2 + (i % 4) * 0.05,
      restitution: 0.1 + (i % 4) * 0.05,
      drag: 0.01,
      sleepVelocityThreshold: 0.25,
      sleepTimeThreshold: 0.4,
      isBullet: i % 25 == 0,
    );

    engine.addBody(body);
  }

  return engine;
}

void stepEngine(
  PhysicsEngine engine, {
  required int steps,
  double dt = fixedStep60,
}) {
  for (var i = 0; i < steps; i++) {
    engine.update(dt);
  }
}

int snapshotHash(PhysicsEngine engine) {
  var hash = 0xcbf29ce484222325;
  for (final body in engine.bodies) {
    final px = (body.position.x * 1000).round();
    final py = (body.position.y * 1000).round();
    final vx = (body.velocity.x * 1000).round();
    final vy = (body.velocity.y * 1000).round();

    hash ^= px;
    hash *= 0x100000001b3;
    hash ^= py;
    hash *= 0x100000001b3;
    hash ^= vx;
    hash *= 0x100000001b3;
    hash ^= vy;
    hash *= 0x100000001b3;
    hash &= 0x7fffffffffffffff;
  }
  return hash;
}

List<PhysicsBody> dynamicBodies(PhysicsEngine engine) =>
    engine.bodies.where((b) => b.mass > 0).toList(growable: false);
