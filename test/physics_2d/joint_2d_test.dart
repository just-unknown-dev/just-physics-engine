// The pure-Dart joints on their own, then every joint on both backends.

import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

import 'backends.dart';

void main() {
  // ── Joint constraints ─────────────────────────────────────────────────────

  Physics2DEngine dartEngine() => Physics2DEngine.pureDart()..initialize();

  Physics2DBody makeBody(double x, double y, {double mass = 1.0}) =>
      Physics2DBody(
        position: Vector2(x, y),
        shape: Circle2DShape(10),
        mass: mass,
        useGravity: false,
        drag: 0.0,
        sleepVelocityThreshold: 0.0,
      );

  group('DistanceJoint2D', () {
    test('rigid distance is maintained', () {
      final engine = dartEngine();
      final a = makeBody(0, 0);
      final b = makeBody(50, 0);
      engine.addBody(a);
      engine.addBody(b);
      engine.addDistanceJoint(a, b, length: 50);

      for (int i = 0; i < 60; i++) {
        engine.update(0.016);
      }

      final dx = b.position.x - a.position.x;
      final dy = b.position.y - a.position.y;
      final dist = math.sqrt(dx * dx + dy * dy);
      expect(dist, closeTo(50, 5));
      engine.dispose();
    });

    test('spring pulls bodies together when stretched', () {
      final engine = dartEngine();
      final a = makeBody(0, 0)..mass = 0; // static
      final b = makeBody(200, 0);
      engine.addBody(a);
      engine.addBody(b);
      engine.addDistanceJoint(
        a,
        b,
        length: 50,
        springHertz: 2,
        dampingRatio: 0.5,
      );

      final startX = b.position.x;
      for (int i = 0; i < 30; i++) {
        engine.update(0.016);
      }

      expect(b.position.x, lessThan(startX));
      engine.dispose();
    });
  });

  group('WeldJoint2D', () {
    test('bodies remain at fixed relative offset', () {
      final engine = dartEngine();
      final a = makeBody(0, 0);
      final b = makeBody(30, 0);
      engine.addBody(a);
      engine.addBody(b);
      engine.addWeldJoint(a, b);

      // Push a rightward; b should follow.
      a.velocity.x = 100;
      for (int i = 0; i < 30; i++) {
        engine.update(0.016);
      }

      final dx = b.position.x - a.position.x;
      final dy = b.position.y - a.position.y;
      expect(dx, closeTo(30, 10));
      expect(dy, closeTo(0, 5));
      engine.dispose();
    });
  });

  group('RevoluteJoint2D', () {
    test('anchor points on both bodies converge', () {
      final engine = dartEngine();
      final a = makeBody(0, 0)..mass = 0; // static pivot
      final b = makeBody(40, 0);
      engine.addBody(a);
      engine.addBody(b);
      engine.addRevoluteJoint(a, b, const Offset(0, 0));

      for (int i = 0; i < 60; i++) {
        engine.update(0.016);
      }

      // World anchor on b should be close to (0, 0).
      expect(b.position.x, closeTo(40, 20));
      engine.dispose();
    });

    test('motor applies angular acceleration', () {
      final engine = dartEngine();
      // inertia=0 on the static anchor so motor torque is not absorbed by it.
      final a = makeBody(0, 0)
        ..mass = 0
        ..inertia = 0;
      final b = makeBody(40, 0);
      engine.addBody(a);
      engine.addBody(b);
      final j =
          engine.addRevoluteJoint(a, b, const Offset(0, 0)) as RevoluteJoint2D;
      // maxMotorTorque * inverseInertia * dt must be < motorSpeed to avoid
      // bang-bang oscillation (50 * 1.0 * 0.016 = 0.8 rad/s per step).
      j.setMotor(5.0, 50);

      for (int i = 0; i < 20; i++) {
        engine.update(0.016);
      }
      expect(b.angularVelocity, greaterThan(0));
      engine.dispose();
    });
  });

  group('MouseJoint2D', () {
    test('body is pulled toward target', () {
      final engine = dartEngine();
      final b = makeBody(0, 0);
      engine.addBody(b);
      final j = engine.addMouseJoint(b, Vector2(200, 0)) as MouseJoint2D;
      j.setTarget(200, 0);

      for (int i = 0; i < 20; i++) {
        engine.update(0.016);
      }
      expect(b.position.x, greaterThan(0));
      engine.dispose();
    });
  });

  group('PrismaticJoint2D', () {
    test('off-axis displacement goes back to where it started', () {
      final engine = dartEngine();
      final a = makeBody(0, 0)..mass = 0;
      final b = makeBody(0, 50); // offset perpendicular to x-axis
      engine.addBody(a);
      engine.addBody(b);
      // Slides along x from where it is; held at y = 50 across it.
      engine.addPrismaticJoint(a, b, const Offset(1, 0));
      b.position.y = 70; // knocked off its line
      b.velocity.y = 100;

      for (int i = 0; i < 60; i++) {
        engine.update(0.016);
      }
      expect(b.position.y - a.position.y, closeTo(50, 0.5));
      engine.dispose();
    });

    test('motor drives body along axis', () {
      final engine = dartEngine();
      final a = makeBody(0, 0)..mass = 0;
      final b = makeBody(0, 0);
      engine.addBody(a);
      engine.addBody(b);
      final j =
          engine.addPrismaticJoint(a, b, const Offset(1, 0))
              as PrismaticJoint2D;
      j.setMotor(100, 500);

      for (int i = 0; i < 20; i++) {
        engine.update(0.016);
      }
      expect(b.velocity.x, greaterThan(0));
      engine.dispose();
    });
  });

  // ── On both backends: the same calls, the same units ──────────────────────

  Physics2DBody fixed(double x, double y, {Shape2D? shape, double angle = 0}) =>
      Physics2DBody(
        position: Vector2(x, y),
        shape: shape ?? Circle2DShape(2),
        mass: 0,
        angle: angle,
      );

  Physics2DBody ball(
    double x,
    double y, {
    double radius = 5,
    bool gravity = true,
  }) => Physics2DBody(
    position: Vector2(x, y),
    shape: Circle2DShape(radius),
    restitution: 0,
    drag: 0,
    useGravity: gravity,
  );

  onBothBackends((engine) {
    test('a distance joint holds its anchors, not the centres', () {
      final e = engine();
      if (noBox2D(e)) return markTestSkipped('no native Box2D');
      final beam = fixed(0, 0, shape: Rectangle2DShape(100, 10));
      final bob = ball(50, 40);
      e
        ..addBody(beam)
        ..addBody(bob)
        // From the beam's right end to the top of the ball.
        ..addDistanceJoint(
          beam,
          bob,
          anchorA: const Offset(50, 0),
          anchorB: const Offset(50, 35),
        );
      runFrames(e, 120);
      expect(bob.position.x, closeTo(50, 2), reason: 'under the end');
      expect(bob.position.y, closeTo(40, 2));
    });

    for (final collide in [false, true]) {
      test('collideConnected: $collide', () {
        final e = engine();
        if (noBox2D(e)) return markTestSkipped('no native Box2D');
        final ground = fixed(0, 20, shape: Rectangle2DShape(4000, 40));
        final bob = ball(0, -5); // resting on the ground
        e
          ..addBody(ground)
          ..addBody(bob)
          // A loose rope from a point on the ground well to the side.
          ..addDistanceJoint(
            ground,
            bob,
            anchorA: const Offset(200, 20),
            minLength: 0,
            maxLength: 400,
            collideConnected: collide,
          );
        runFrames(e, 60);
        if (collide) {
          expect(bob.position.y, closeTo(-5, 1), reason: 'on the ground');
        } else {
          expect(bob.position.y, greaterThan(150), reason: 'through it');
        }
      });
    }

    test('a spring swings at its frequency in hertz', () {
      final e = engine();
      if (noBox2D(e)) return markTestSkipped('no native Box2D');
      final anchor = fixed(0, 0);
      final bob = ball(0, 100, gravity: false);
      e
        ..addBody(anchor)
        ..addBody(bob)
        ..addDistanceJoint(
          anchor,
          bob,
          length: 50,
          springHertz: 2,
          dampingRatio: 0.05,
        );
      // Frames at which the stretch turns from long to short.
      final crossings = <int>[];
      var last = 50.0;
      for (var i = 0; i < 120 && crossings.length < 3; i++) {
        e.update(1 / 60);
        final stretch = (bob.position - anchor.position).length - 50;
        if (last > 0 && stretch <= 0) crossings.add(i);
        last = stretch;
      }
      expect(crossings, hasLength(greaterThanOrEqualTo(2)));
      // 2 Hz: a period of half a second, 30 frames.
      expect(crossings[1] - crossings[0], closeTo(30, 3));
    });

    test("a wheel's suspension is a spring in hertz", () {
      final e = engine();
      if (noBox2D(e)) return markTestSkipped('no native Box2D');
      final chassis = fixed(0, 0, shape: Rectangle2DShape(100, 20));
      final wheel = ball(0, 50, radius: 10);
      e
        ..addBody(chassis)
        ..addBody(wheel);
      e.addWheelJoint(chassis, wheel, const Offset(0, 1)).setSpring(4, 1);
      runFrames(e, 120);
      // A 4 Hz spring holds the wheel's weight a few units below where it
      // started (g / ω² ≈ 1.6 for an ideal one; the solvers' soft steps sag
      // a little more). In the old pure-Dart units, 4 was a pull of 4 per
      // unit: it would hang 245 units down.
      final y = wheel.position.y;
      expect(y, inInclusiveRange(50.5, 55));
      runFrames(e, 30);
      expect(wheel.position.y, closeTo(y, 0.2), reason: 'settled');
    });

    test('a weld keeps bodies turned apart turned apart', () {
      final e = engine();
      if (noBox2D(e)) return markTestSkipped('no native Box2D');
      final wall = fixed(0, 0, shape: Rectangle2DShape(10, 10), angle: 0.5);
      final box = Physics2DBody(
        position: Vector2(30, 0),
        shape: Rectangle2DShape(10, 10),
        restitution: 0,
      );
      e
        ..addBody(wall)
        ..addBody(box)
        ..addWeldJoint(wall, box);
      runFrames(e, 60);
      expect(box.angle, closeTo(0, 0.05));
      expect(box.position.x, closeTo(30, 1));
      expect(box.position.y, closeTo(0, 1));
    });

    test("a hinge's limits count from the angle it was made at", () {
      final e = engine();
      if (noBox2D(e)) return markTestSkipped('no native Box2D');
      final post = fixed(0, 0);
      final door = Physics2DBody(
        position: Vector2(0, 0),
        shape: Rectangle2DShape(10, 60),
        angle: 1,
        useGravity: false,
        drag: 0,
      );
      e
        ..addBody(post)
        ..addBody(door)
        ..addRevoluteJoint(post, door, Offset.zero).setLimits(-0.1, 0.1);
      door.angularVelocity = 5;
      runFrames(e, 60);
      expect(door.angle, closeTo(1.1, 0.05));
    });

    test("a slider's limits and motor, through Joint2D", () {
      final e = engine();
      if (noBox2D(e)) return markTestSkipped('no native Box2D');
      final rail = fixed(0, 0);
      final car = ball(100, 0, gravity: false);
      e
        ..addBody(rail)
        ..addBody(car);
      e.addPrismaticJoint(rail, car, const Offset(1, 0))
        ..setLimits(-20, 20)
        ..setMotor(300, 1000);
      runFrames(e, 60);
      expect(car.position.x, closeTo(120, 1), reason: 'stopped at its limit');
      expect(car.position.y, closeTo(0, 0.5), reason: 'on its rail');
    });

    test('removing a joint frees its bodies', () {
      final e = engine();
      if (noBox2D(e)) return markTestSkipped('no native Box2D');
      final anchor = fixed(0, 0);
      final bob = ball(0, 50);
      e
        ..addBody(anchor)
        ..addBody(bob);
      final joint = e.addDistanceJoint(anchor, bob, length: 50);
      runFrames(e, 30);
      expect(bob.position.y, closeTo(50, 1.5));
      e.removeJoint(joint);
      expect(e.joints, isEmpty);
      runFrames(e, 30);
      expect(bob.position.y, greaterThan(150), reason: 'it falls');
    });
  });
}
