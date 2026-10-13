// How two bodies' materials combine at a contact, the same on both
// backends: the less bouncy of the two (the native backend installs this
// rule in place of Box2D's), the geometric mean of the grips (Box2D's), and
// no bounce below Box2D's speed threshold. So a ball bounces the same on the
// web as on a phone.

import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  test('the mixing rules', () {
    expect(PhysicsEngine.mixRestitution(0, 0.8), 0);
    expect(PhysicsEngine.mixRestitution(0.6, 0.3), 0.3);
    expect(PhysicsEngine.mixFriction(0.2, 0.5), closeTo(math.sqrt(0.1), 1e-12));
    expect(PhysicsEngine.mixFriction(0, 0.9), 0, reason: 'ice stays ice');
  });

  for (final (name, make) in [
    ('pure Dart', () => PhysicsEngine.pureDart()..initialize()),
    ('native Box2D', () => PhysicsEngine()..initialize()),
  ]) {
    group(name, () {
      late PhysicsEngine engine;
      setUp(() => engine = make());
      tearDown(() => engine.dispose());

      bool skipped() {
        if (name == 'native Box2D' && engine.stats['backend'] != 'box2d_v3') {
          markTestSkipped('native Box2D backend unavailable');
          return true;
        }
        return false;
      }

      PhysicsBody floor(double restitution) => PhysicsBody(
        position: Vector2(0, 100),
        shape: RectangleShape(400, 20),
        mass: 0,
        restitution: restitution,
      );

      /// The fastest the ball rises (most negative y velocity) in 1.5 s,
      /// dropped from 200 units above the floor.
      double rise(double floorRestitution) {
        final ball = PhysicsBody(
          position: Vector2(0, -100),
          shape: CircleShape(10),
          restitution: 0.8,
          drag: 0,
        );
        engine
          ..addBody(floor(floorRestitution))
          ..addBody(ball);
        var rising = 0.0;
        for (var i = 0; i < 90; i++) {
          engine.update(1 / 60);
          rising = math.min(rising, ball.velocity.y);
        }
        return rising;
      }

      test('a bounce needs both bodies bouncy', () {
        if (skipped()) return;
        // Up is -Y. It hits at about 600 units/s.
        expect(rise(0), greaterThan(-10), reason: 'a dead floor: no bounce');
      });

      test('two bouncy bodies bounce', () {
        if (skipped()) return;
        expect(rise(1), lessThan(-300));
      });

      test('a slow touch settles rather than bouncing', () {
        if (skipped()) return;
        final ball = PhysicsBody(
          position: Vector2(0, 75),
          shape: CircleShape(10),
          restitution: 1,
          useGravity: false,
          drag: 0,
        )..velocity.setValues(0, 50);
        engine
          ..addBody(floor(1))
          ..addBody(ball);
        for (var i = 0; i < 60; i++) {
          engine.update(1 / 60);
        }
        expect(
          ball.velocity.y,
          greaterThan(-10),
          reason: '50 units/s is under the threshold: no bounce back',
        );
      });
    });
  }
}
