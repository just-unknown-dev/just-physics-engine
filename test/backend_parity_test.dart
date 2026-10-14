// What both 2D backends must do alike: events once per thing that happened
// at any frame rate, bodies changed at runtime, queries past a fixed
// number of hits, shapes away from their body's origin, lines with a
// thickness, and joints that hold. Every case runs on both backends; the
// native one is skipped where the Box2D library is not built.

import 'dart:math' as math;
import 'dart:ui' show Rect;

import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

PhysicsBody _static(Offset at, CollisionShape shape, {bool sensor = false}) =>
    PhysicsBody(
      position: Vector2(at.dx, at.dy),
      shape: shape,
      mass: 0,
      isSensor: sensor,
    );

PhysicsBody _ball(Offset at, {double radius = 5, bool gravity = true}) =>
    PhysicsBody(
      position: Vector2(at.dx, at.dy),
      shape: CircleShape(radius),
      restitution: 0,
      drag: 0,
      useGravity: gravity,
    );

/// A floor 40 m wide, its top at y = 0 (+Y is down).
PhysicsBody _floor() => _static(const Offset(0, 20), RectangleShape(4000, 40));

void _run(PhysicsEngine engine, int frames, {double dt = 1 / 60}) {
  for (var i = 0; i < frames; i++) {
    engine.update(dt);
  }
}

double _distance(PhysicsBody a, PhysicsBody b) =>
    (b.position - a.position).length;

void main() {
  final backends = <String, PhysicsEngine Function()>{
    'pure Dart': PhysicsEngine.pureDart,
    'Box2D': Box2DPhysicsEngine.new,
  };

  for (final MapEntry(key: name, value: make) in backends.entries) {
    group(name, () {
      late PhysicsEngine engine;
      setUp(() => engine = make()..initialize());
      tearDown(() => engine.dispose());

      bool unavailable() =>
          name == 'Box2D' && engine.stats['backend'] != 'box2d_v3';

      group('events', () {
        for (final (label, dt) in [
          ('two steps a frame', 2 / 60),
          ('one step every other frame', 1 / 120),
        ]) {
          test('a landing begins one contact, $label', () {
            if (unavailable()) return markTestSkipped('no native Box2D');
            final floor = _floor();
            final ball = _ball(const Offset(0, -100));
            engine
              ..addBody(floor)
              ..addBody(ball);
            var begins = 0;
            for (var i = 0; i < (2 / dt).round(); i++) {
              engine
                ..update(dt)
                ..pollContactBeginEvents((a, b, nx, ny) {
                  if ({a, b}.containsAll([floor, ball])) begins++;
                });
            }
            expect(ball.position.y, closeTo(-5, 1), reason: 'landed');
            expect(begins, 1);
          });

          test('a sensor sees a ball pass once, sensor first, $label', () {
            if (unavailable()) return markTestSkipped('no native Box2D');
            final sensor = _static(
              const Offset(0, -100),
              RectangleShape(100, 20),
              sensor: true,
            );
            final ball = _ball(const Offset(0, -200));
            engine
              ..addBody(_floor())
              ..addBody(ball)
              ..addBody(sensor);
            var begins = 0, ends = 0;
            for (var i = 0; i < (2 / dt).round(); i++) {
              engine
                ..update(dt)
                ..pollSensorBeginEvents((s, v) {
                  expect(s, same(sensor));
                  expect(v, same(ball));
                  begins++;
                })
                ..pollSensorEndEvents((s, v) {
                  expect(s, same(sensor));
                  ends++;
                });
            }
            expect([begins, ends], [1, 1]);
          });
        }
      });

      group('bodies changed at runtime', () {
        test('a static body turned dynamic falls, and turns', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final body = PhysicsBody(
            position: Vector2(0, -100),
            shape: RectangleShape(20, 20),
            bodyType: BodyType.static,
            useGravity: false,
            drag: 0,
          );
          engine.addBody(body);
          _run(engine, 10);
          expect(body.position.y, -100);

          body.useGravity = true;
          engine.setBodyType(body, BodyType.dynamic);
          _run(engine, 20);
          expect(body.position.y, greaterThan(-90));

          body.torque = 1e7;
          _run(engine, 1);
          expect(body.angularVelocity, isNot(0));
        });

        test('gravity switched off on the body itself takes effect', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final ball = _ball(const Offset(0, -1000));
          engine.addBody(ball);
          _run(engine, 10);
          ball.useGravity = false;
          _run(engine, 1);
          final vy = ball.velocity.y;
          _run(engine, 20);
          expect(ball.velocity.y, closeTo(vy, 1));
        });

        test('a kinematic body at rest moves when it is told to', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final platform = PhysicsBody(
            position: Vector2(0, 0),
            shape: RectangleShape(100, 10),
            bodyType: BodyType.kinematic,
          );
          engine.addBody(platform);
          _run(engine, 120); // long enough for Box2D to put it to sleep
          platform.velocity.x = 100;
          _run(engine, 30);
          expect(platform.position.x, greaterThan(30));
        });
      });

      group('queries', () {
        test('castRayAll reports every body, nearest first, past 32', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          for (var i = 0; i < 40; i++) {
            engine.addBody(
              _static(Offset(i * 20.0, 0), RectangleShape(10, 10)),
            );
          }
          final far = _ball(const Offset(900, 0), gravity: false);
          engine.addBody(far);
          _run(engine, 1);
          final hits = engine.castRayAll(
            Ray(
              origin: const Offset(-50, 0),
              direction: const Offset(1, 0),
              maxDistance: 1000,
            ),
          );
          expect(hits, hasLength(41));
          expect(hits.last.body, same(far));
          for (var i = 1; i < hits.length; i++) {
            expect(hits[i].distance, greaterThan(hits[i - 1].distance));
          }
        });

        test("a ray hits a compound body's nearest shape", () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final body = PhysicsBody(
            position: Vector2(100, 0),
            shape: RectangleShape(20, 20),
            additionalShapes: [CircleShape(10, center: const Offset(-60, 0))],
            mass: 0,
          );
          engine.addBody(body);
          _run(engine, 1);
          final ray = Ray(
            origin: Offset.zero,
            direction: const Offset(1, 0),
            maxDistance: 500,
          );
          expect(engine.castRay(ray)!.distance, closeTo(30, 0.5));
          expect(engine.castRayAll(ray).single.distance, closeTo(30, 0.5));
        });

        test('a ray passes the shape it starts in, excluding or not', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final inside = _static(Offset.zero, RectangleShape(40, 40));
          final beyond = _static(const Offset(100, 0), RectangleShape(20, 20));
          final other = _ball(const Offset(0, 500), gravity: false);
          engine
            ..addBody(inside)
            ..addBody(beyond)
            ..addBody(other);
          _run(engine, 1);
          final ray = Ray(
            origin: Offset.zero,
            direction: const Offset(1, 0),
            maxDistance: 500,
          );
          expect(engine.castRay(ray)!.body, same(beyond));
          expect(engine.castRay(ray, exclude: {other})!.body, same(beyond));
        });

        test('queryAABB finds every body, past 32', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          for (var i = 0; i < 50; i++) {
            engine.addBody(
              _static(Offset(i * 20.0, 0), RectangleShape(10, 10)),
            );
          }
          _run(engine, 1);
          expect(
            engine.queryAABB(const Rect.fromLTRB(-50, -50, 1100, 50)),
            hasLength(50),
          );
        });

        test('castCircle stops where the circle touches', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final box = _static(const Offset(60, 0), RectangleShape(20, 20));
          engine.addBody(box);
          _run(engine, 1);
          final head = engine.castCircle(Offset.zero, 10, const Offset(100, 0));
          expect(head!.body, same(box));
          expect(head.toi, closeTo(0.4, 0.01));
          expect(head.normal.dx, closeTo(-1, 1e-3));
          // Grazing the top: the circle spans y 5..25, the box -10..10.
          expect(
            engine.castCircle(const Offset(0, 15), 10, const Offset(100, 0)),
            isNotNull,
          );
        });
      });

      group('shapes', () {
        /// Where a body of [shape] dropped at [x] comes to rest.
        double rest(CollisionShape shape, {double x = 0}) {
          final body = PhysicsBody(
            position: Vector2(x, -60),
            shape: shape,
            restitution: 0,
            fixedRotation: true,
          );
          engine.addBody(body);
          _run(engine, 120);
          return body.position.y;
        }

        test('a chain drawn right to left holds from above', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          engine.addBody(
            _static(
              Offset.zero,
              ChainShape(const [
                Offset(200, 0),
                Offset(0, 0),
                Offset(-200, 0),
              ], thickness: 0),
            ),
          );
          expect(rest(CircleShape(5)), closeTo(-5, 0.6));
        });

        test("a chain's thickness holds a ball off it", () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          engine.addBody(
            _static(
              Offset.zero,
              ChainShape(const [
                Offset(-200, 0),
                Offset(0, 0),
                Offset(200, 0),
              ], thickness: 4),
            ),
          );
          expect(rest(CircleShape(5)), closeTo(-9, 0.6));
        });

        test("a segment's thickness holds a ball off it", () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          engine.addBody(
            _static(
              Offset.zero,
              SegmentShape(
                const Offset(-200, 0),
                const Offset(200, 0),
                thickness: 3,
              ),
            ),
          );
          expect(rest(CircleShape(5)), closeTo(-8, 0.6));
        });

        test('a capsule lands on a segment added after it', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final capsule = PhysicsBody(
            position: Vector2(0, -40),
            shape: CapsuleShape.vertical(height: 40, radius: 10),
            restitution: 0,
            fixedRotation: true,
          );
          engine
            ..addBody(capsule)
            ..addBody(
              _static(
                Offset.zero,
                SegmentShape(const Offset(-200, 0), const Offset(200, 0)),
              ),
            );
          _run(engine, 120);
          expect(capsule.position.y, closeTo(-22, 0.6));
        });

        test("a polygon away from its body's origin holds a crate", () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          // The body is at the origin; its polygon spans y 40..60.
          engine.addBody(
            _static(
              Offset.zero,
              PolygonShape(const [
                Offset(-100, 40),
                Offset(100, 40),
                Offset(100, 60),
                Offset(-100, 60),
              ]),
            ),
          );
          final crate = PhysicsBody(
            position: Vector2(0, -50),
            shape: RectangleShape(20, 20),
            restitution: 0,
            fixedRotation: true,
          );
          engine.addBody(crate);
          _run(engine, 180);
          expect(crate.position.y, closeTo(30, 1));
        });

        test("a rounded box holds a capsule on its rounding", () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          // Core 200 x 20, rounded by 10: its top at y = -20.
          engine.addBody(
            _static(
              Offset.zero,
              RoundedPolygonShape.rect(
                width: 200,
                height: 20,
                cornerRadius: 10,
              ),
            ),
          );
          final y = rest(CapsuleShape.vertical(height: 40, radius: 10));
          expect(y, closeTo(-40, 1));
        });
      });

      group('joints', () {
        /// A static anchor at the origin, and a ball [length] to its right.
        (PhysicsBody, PhysicsBody) pendulum({double length = 100}) {
          final anchor = _static(Offset.zero, CircleShape(2));
          final bob = _ball(Offset(length, 0));
          engine
            ..addBody(anchor)
            ..addBody(bob);
          return (anchor, bob);
        }

        test('a hinged bob swings down and keeps its length', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final (anchor, bob) = pendulum();
          engine.addRevoluteJoint(anchor, bob, Offset.zero);
          var lowest = double.negativeInfinity;
          for (var i = 0; i < 60; i++) {
            engine.update(1 / 60);
            expect(_distance(anchor, bob), closeTo(100, 2), reason: '$i');
            lowest = math.max(lowest, bob.position.y);
          }
          expect(lowest, greaterThan(95), reason: 'it swung to the bottom');
        });

        test('a rope holds its length for ten seconds', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final (anchor, bob) = pendulum();
          engine.addDistanceJoint(anchor, bob, length: 100);
          _run(engine, 600);
          expect(_distance(anchor, bob), closeTo(100, 1.5));
        });

        test('removeJoint lets the bob fall', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final (anchor, bob) = pendulum();
          final joint = engine.addRevoluteJoint(anchor, bob, Offset.zero);
          _run(engine, 10);
          engine.removeJoint(joint);
          _run(engine, 60);
          expect(_distance(anchor, bob), greaterThan(150));
          expect(engine.joints, isEmpty);
        });

        test('removeBody takes its joints with it', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final (anchor, bob) = pendulum();
          engine.addDistanceJoint(anchor, bob, length: 100);
          _run(engine, 5);
          engine.removeBody(bob);
          expect(engine.joints, isEmpty);
          _run(engine, 5);
        });

        test('a slider holds its line', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final a = _static(Offset.zero, CircleShape(2));
          final b = _ball(const Offset(0, 50));
          engine
            ..addBody(a)
            ..addBody(b)
            ..addPrismaticJoint(a, b, const Offset(1, 0));
          b.velocity.x = 200;
          _run(engine, 30);
          expect(b.position.y, closeTo(50, 1), reason: 'gravity held off');
          expect(b.position.x, greaterThan(50));
        });

        test('a mouse joint drags a body toward its target', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final body = _ball(Offset.zero, gravity: false);
          engine
            ..addBody(body)
            ..addMouseJoint(body, Vector2(200, 0));
          _run(engine, 30);
          expect(body.position.x, greaterThan(20));
          // A spring about the target: it swings, but never further out.
          for (var i = 0; i < 180; i++) {
            engine.update(1 / 60);
            expect(body.position.x, inInclusiveRange(-20, 420), reason: '$i');
          }
        });

        test('a wheel turns about its own centre', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final chassis = _static(Offset.zero, RectangleShape(100, 20));
          final wheel = _ball(const Offset(0, 50), radius: 10, gravity: false);
          engine
            ..addBody(chassis)
            ..addBody(wheel);
          final joint = engine.addWheelJoint(
            chassis,
            wheel,
            const Offset(0, 1),
          );
          switch (joint) {
            case Box2DJoint():
              joint.setWheelMotor(5, 1e9);
            case WheelJoint():
              joint.setWheelMotor(5, 1e9);
          }
          for (var i = 0; i < 60; i++) {
            engine.update(1 / 60);
            expect(wheel.position.x, closeTo(0, 1), reason: '$i');
            expect(wheel.position.y, closeTo(50, 1), reason: '$i');
          }
          expect(wheel.angularVelocity, greaterThan(1));
        });

        test('a joint does not move a kinematic body', () {
          if (unavailable()) return markTestSkipped('no native Box2D');
          final platform = PhysicsBody(
            position: Vector2(0, 0),
            shape: RectangleShape(100, 10),
            bodyType: BodyType.kinematic,
          );
          final lamp = _ball(const Offset(0, 100));
          engine
            ..addBody(platform)
            ..addBody(lamp)
            ..addDistanceJoint(platform, lamp, length: 100);
          _run(engine, 120);
          expect(platform.position.y, closeTo(0, 1e-6));
          expect(_distance(platform, lamp), closeTo(100, 1.5));
        });
      });
    });
  }

  group('pure Dart only', () {
    late PhysicsEngine engine;
    setUp(() => engine = PhysicsEngine.pureDart()..initialize());
    tearDown(() => engine.dispose());

    test('a removed body ends no contact, and leaves no joint', () {
      final floor = _floor();
      final ball = _ball(const Offset(0, -10));
      final other = _ball(const Offset(50, -10));
      engine
        ..addBody(floor)
        ..addBody(ball)
        ..addBody(other)
        ..addDistanceJoint(ball, other, length: 50);
      _run(engine, 30);
      engine.removeBody(ball);
      engine.update(1 / 60);
      engine.pollContactEndEvents((a, b) {
        expect([a, b], isNot(contains(ball)));
      });
      expect(engine.joints, isEmpty);
    });

    test('shapes of no size do not poison the world', () {
      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(0, 0),
        useGravity: false,
      );
      final b = PhysicsBody(
        position: Vector2(50, 0),
        shape: RectangleShape(0, 0),
        useGravity: false,
      );
      engine
        ..addBody(a)
        ..addBody(b);
      _run(engine, 10);
      expect(a.position.x.isFinite && b.position.x.isFinite, isTrue);
    });

    test('a slider stops at its limits, from where it started', () {
      final a = _static(Offset.zero, CircleShape(2));
      final b = _ball(const Offset(100, 0), gravity: false);
      engine
        ..addBody(a)
        ..addBody(b);
      final joint =
          engine.addPrismaticJoint(a, b, const Offset(3, 0)) as PrismaticJoint
            ..limitEnabled = true
            ..lowerLimit = -20
            ..upperLimit = 20;
      b.velocity.x = 300;
      _run(engine, 60);
      expect(joint.translation, closeTo(20, 1));
      expect(b.position.x, closeTo(120, 1));
      expect(b.position.y, closeTo(0, 1e-6));
    });

    test('a distance spring settles', () {
      final anchor = _static(Offset.zero, CircleShape(2));
      final bob = _ball(const Offset(0, 100), gravity: false);
      engine
        ..addBody(anchor)
        ..addBody(bob)
        ..addDistanceJoint(anchor, bob, length: 50, stiffness: 100, damping: 4);
      double swing(int frames) {
        var most = 0.0;
        for (var i = 0; i < frames; i++) {
          engine.update(1 / 60);
          most = math.max(most, (_distance(anchor, bob) - 50).abs());
        }
        return most;
      }

      final first = swing(60);
      swing(60);
      expect(swing(60), lessThan(first / 4));
    });
  });

  group('Box2D only', () {
    test('a second initialize keeps the world and its bodies', () {
      final engine = Box2DPhysicsEngine()..initialize();
      addTearDown(engine.dispose);
      if (engine.stats['backend'] != 'box2d_v3') {
        return markTestSkipped('no native Box2D');
      }
      final ball = _ball(const Offset(0, -100));
      engine
        ..addBody(_floor())
        ..addBody(ball)
        ..initialize();
      expect(engine.bodyCount, 2);
      _run(engine, 120);
      expect(ball.position.y, closeTo(-5, 1));
    });
  });
}
