// The 2-D engine: lifecycle, gravity and integration, sleep and
// collision resolution, on the platform's engine.

import 'dart:ui' show Rect;

import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

import 'backends.dart';

void main() {
  // ── Physics2DEngine lifecycle ──────────────────────────────────────────────

  group('Physics2DEngine lifecycle', () {
    test('initialize and dispose without error', () {
      final engine = PhysicsEngineFactory.create2D();
      engine.initialize();
      engine.dispose();
    });

    test('addBody and removeBody', () {
      final engine = PhysicsEngineFactory.create2D()..initialize();
      final body = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(10.0),
      );
      engine.addBody(body);
      expect(engine.bodies.length, 1);
      engine.removeBody(body);
      expect(engine.bodies.length, 0);
      engine.dispose();
    });

    test('addBody is idempotent — duplicate add is ignored', () {
      final engine = PhysicsEngineFactory.create2D()..initialize();
      final body = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(5.0),
      );
      engine.addBody(body);
      engine.addBody(body);
      expect(engine.bodies.length, 1);
      engine.dispose();
    });

    test('removeBody on unregistered body does not throw', () {
      final engine = PhysicsEngineFactory.create2D()..initialize();
      final body = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(5.0),
      );
      expect(() => engine.removeBody(body), returnsNormally);
      engine.dispose();
    });

    test('stats map contains expected keys after update', () {
      final engine = PhysicsEngineFactory.create2D()..initialize();
      engine.update(0.016);
      final s = engine.stats;
      // These keys are guaranteed by both the pure-Dart and Box2D backends.
      expect(s.containsKey('bodyCount'), isTrue);
      expect(s.containsKey('awakeBodies'), isTrue);
      expect(s.containsKey('potentialPairs'), isTrue);
      expect(s.containsKey('resolvedCollisions'), isTrue);
      expect(s.containsKey('lastStepMs'), isTrue);
      expect(s.containsKey('backend'), isTrue);
      engine.dispose();
    });
  });

  // ── Physics2DEngine gravity & integration ───────────────────────────────────

  group('Physics2DEngine gravity & integration', () {
    test('default gravity is 981 units/s²', () {
      final engine = PhysicsEngineFactory.create2D();
      expect(engine.gravity.y, closeTo(981.0, 1e-6));
    });

    test('setGravity updates gravity vector', () {
      final engine = PhysicsEngineFactory.create2D();
      engine.setGravity(0, 500);
      expect(engine.gravity.x, closeTo(0, 1e-6));
      expect(engine.gravity.y, closeTo(500, 1e-6));
    });

    test('body falls under gravity over one second', () {
      final engine = PhysicsEngineFactory.create2D()..initialize();
      final body = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(5.0),
        drag: 0.0,
        sleepVelocityThreshold: 0.0,
      );
      engine.addBody(body);

      // Simulate 1 second in small steps.
      const steps = 60;
      const dt = 1.0 / steps;
      for (int i = 0; i < steps; i++) {
        engine.update(dt);
      }

      // After ~1 s of free fall: y ≈ ½ × 981 × 1² ≈ 490.5 units.
      // Allow ±5% tolerance for integration error.
      expect(body.position.y, greaterThan(450));
      expect(body.position.y, lessThan(520));
      engine.dispose();
    });

    test('static body (mass=0) does not move under gravity', () {
      final engine = PhysicsEngineFactory.create2D()..initialize();
      final body = Physics2DBody(
        position: Vector2(100, 200),
        shape: Circle2DShape(10.0),
        mass: 0.0,
      );
      engine.addBody(body);
      engine.update(1.0);
      expect(body.position.x, closeTo(100, 1e-6));
      expect(body.position.y, closeTo(200, 1e-6));
      engine.dispose();
    });

    test('useGravity=false body is unaffected by world gravity', () {
      final engine = PhysicsEngineFactory.create2D()..initialize();
      final body = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(5.0),
        useGravity: false,
        drag: 0.0,
      );
      engine.addBody(body);
      engine.update(1.0);
      expect(body.position.y, closeTo(0, 1e-4));
      engine.dispose();
    });

    test('applied force accelerates body', () {
      final engine = PhysicsEngineFactory.create2D()..initialize();
      final body = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(5.0),
        useGravity: false,
        drag: 0.0,
        sleepVelocityThreshold: 0.0,
      );
      engine.addBody(body);
      // Apply the force every step and simulate 1 second in small steps —
      // a single large update(1.0) call only fires a handful of fixed steps
      // on the Box2D backend due to its spiral-of-death accumulator cap (see
      // "falls under gravity" above), and Box2D clears applied forces after
      // each internal step, so the force must be re-applied every frame just
      // like a sustained force (thrust, wind) would be in a real game loop.
      const steps = 60;
      const dt = 1.0 / steps;
      for (int i = 0; i < steps; i++) {
        body.applyForce(Vector2(100, 0));
        engine.update(dt);
      }
      // v = F/m × dt × steps = 100/1 × 1 = 100; x ≈ 100 (semi-implicit euler)
      expect(body.velocity.x, greaterThan(50));
      expect(body.position.x, greaterThan(50));
      engine.dispose();
    });
  });

  // ── Sleep system ──────────────────────────────────────────────────────────

  group('Sleep system', () {
    test('body below sleep threshold eventually sleeps', () {
      final engine = PhysicsEngineFactory.create2D()..initialize();
      final body = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(5.0),
        useGravity: false,
        drag: 0.0,
        sleepVelocityThreshold: 100.0,
        sleepTimeThreshold: 0.1,
      );
      engine.addBody(body);
      // Simulate until sleep triggers. sleepTimeThreshold is a pure-Dart-only
      // concept — Box2D's own time-to-sleep is a fixed 0.5s internal
      // constant (B2_TIME_TO_SLEEP) not exposed through its public API, so
      // on the native backend this must simulate past 0.5s regardless of
      // sleepTimeThreshold above.
      for (int i = 0; i < 45; i++) {
        engine.update(0.016);
      }
      expect(body.isAwake, isFalse);
      engine.dispose();
    });

    test('collision wakes a sleeping body', () {
      final engine = PhysicsEngineFactory.create2D()..initialize();
      final sleeper = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(10.0),
        useGravity: false,
        drag: 0.0,
        sleepVelocityThreshold: 100.0,
        sleepTimeThreshold: 0.01,
        isAwake: false,
      );
      final mover = Physics2DBody(
        position: Vector2(30, 0),
        shape: Circle2DShape(10.0),
        useGravity: false,
        drag: 0.0,
        velocity: Vector2(-500, 0),
        sleepVelocityThreshold: 0.0,
      );
      engine.addBody(sleeper);
      engine.addBody(mover);

      // Step until mover reaches sleeper.
      for (int i = 0; i < 30; i++) {
        engine.update(0.016);
        if (sleeper.isAwake) break;
      }
      expect(sleeper.isAwake, isTrue);
      engine.dispose();
    });
  });

  // ── Collision resolution (impulse) ────────────────────────────────────────

  group('Collision resolution', () {
    test('two circles bouncing apart separate after resolution', () {
      final engine = PhysicsEngineFactory.create2D()..initialize();
      engine.setGravity(0, 0);
      final a = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(10),
        drag: 0.0,
        velocity: Vector2(100, 0),
        useGravity: false,
        sleepVelocityThreshold: 0.0,
      );
      final b = Physics2DBody(
        position: Vector2(18, 0),
        shape: Circle2DShape(10),
        drag: 0.0,
        useGravity: false,
        sleepVelocityThreshold: 0.0,
      );
      engine.addBody(a);
      engine.addBody(b);

      for (int i = 0; i < 10; i++) {
        engine.update(0.016);
      }

      // After collision, b should have gained positive x-velocity.
      expect(b.velocity.x, greaterThan(0));
      engine.dispose();
    });
  });

  // ── On both backends ──────────────────────────────────────────────────────

  /// A floor 40 m wide, its top at y = 0 (+Y is down).
  Physics2DBody floor() => Physics2DBody(
    position: Vector2(0, 20),
    shape: Rectangle2DShape(4000, 40),
    mass: 0,
    restitution: 0,
  );

  Physics2DBody ball(double x, double y, {bool gravity = true}) =>
      Physics2DBody(
        position: Vector2(x, y),
        shape: Circle2DShape(5),
        restitution: 0,
        drag: 0,
        useGravity: gravity,
      );

  onBothBackends((engine) {
    group('an inactive body', () {
      for (final (how, switchOff)
          in <(String, void Function(Physics2DEngine, Physics2DBody))>[
            ('setBodyActive', (e, b) => e.setBodyActive(b, false)),
            ('a write to isActive', (e, b) => b.isActive = false),
          ]) {
        test('is out of the simulation until it is back ($how)', () {
          final e = engine();
          if (noBox2D(e)) return markTestSkipped('no native Box2D');
          final crate = ball(0, -100);
          final probe = Physics2DBody(
            position: Vector2(0, -100),
            shape: Circle2DShape(20),
            mass: 0,
            isSensor: true,
          );
          e
            ..addBody(floor())
            ..addBody(crate);
          runFrames(e, 1);
          switchOff(e, crate);
          e.addBody(probe);
          final y = crate.position.y;
          var sensed = 0;
          for (var i = 0; i < 30; i++) {
            e.update(1 / 60);
            e.pollSensorBeginEvents((_, _) => sensed++);
          }
          expect(crate.position.y, y, reason: 'it does not fall');
          expect(sensed, 0, reason: 'no sensor sees it');
          final ray = Ray2D(
            origin: const Offset(0, -300),
            direction: const Offset(0, 1),
            maxDistance: 600,
          );
          expect(e.raycast(ray)?.body, isNot(crate), reason: 'no ray hits it');
          expect(e.raycastAll(ray).map((h) => h.body), isNot(contains(crate)));
          expect(
            e.queryAABB(Rect.fromCircle(center: Offset(0, y), radius: 10)),
            isNot(contains(crate)),
          );
          expect(e.queryPoint(Offset(0, y)), isNot(contains(crate)));
          final mover = Mover2DShape.capsule(radius: 4, height: 12);
          expect(
            e.collideMover(mover, Offset(0, y)).map((p) => p.body),
            isNot(contains(crate)),
            reason: 'no mover touches it',
          );

          e.setBodyActive(crate, true);
          runFrames(e, 60);
          expect(crate.position.y, closeTo(-5, 1), reason: 'back: it lands');
        });
      }

      test('ends its contacts', () {
        final e = engine();
        if (noBox2D(e)) return markTestSkipped('no native Box2D');
        final ground = floor();
        final crate = ball(0, -5);
        e
          ..addBody(ground)
          ..addBody(crate);
        runFrames(e, 30);
        e.setBodyActive(crate, false);
        var ended = false;
        for (var i = 0; i < 3 && !ended; i++) {
          e.update(1 / 60);
          e.pollContactEndEvents((a, b) {
            ended |= {a, b}.containsAll([ground, crate]);
          });
        }
        expect(ended, isTrue);
      });

      test('keeps its velocity for when it is back', () {
        final e = engine();
        if (noBox2D(e)) return markTestSkipped('no native Box2D');
        final crate = ball(0, 0, gravity: false)..velocity.setValues(120, 0);
        e.addBody(crate);
        runFrames(e, 1);
        e.setBodyActive(crate, false);
        runFrames(e, 10);
        expect(crate.velocity.x, closeTo(120, 1e-3));
        final x = crate.position.x;
        e.setBodyActive(crate, true);
        runFrames(e, 30);
        expect(crate.position.x, closeTo(x + 60, 3), reason: 'on its way');
      });
    });

    group('a sleeping body', () {
      Physics2DBody sleeper() => Physics2DBody(
        position: Vector2(0, -100),
        shape: Circle2DShape(5),
        restitution: 0,
        drag: 0,
        isAwake: false,
      );

      for (final (how, wake)
          in <(String, void Function(Physics2DEngine, Physics2DBody))>[
            ('wakeBody', (e, b) => e.wakeBody(b)),
            ('a write to isAwake', (e, b) => b.isAwake = true),
            ('setGravity', (e, b) => e.setGravity(0, 500)),
            ('setBodyGravityScale', (e, b) => e.setBodyGravityScale(b, 2)),
          ]) {
        test('wakes through $how', () {
          final e = engine();
          if (noBox2D(e)) return markTestSkipped('no native Box2D');
          final body = sleeper();
          e.addBody(body);
          runFrames(e, 20);
          expect(body.position.y, -100, reason: 'asleep, it hangs');
          wake(e, body);
          runFrames(e, 20);
          expect(body.isAwake, isTrue);
          expect(body.position.y, greaterThan(-90), reason: 'it falls');
        });
      }

      test('is woken by a force, which does not pile up', () {
        final e = engine();
        if (noBox2D(e)) return markTestSkipped('no native Box2D');
        final body = sleeper()..useGravity = false;
        e.addBody(body);
        runFrames(e, 5);
        // 100 units/s² for half a second: 50 units/s.
        for (var i = 0; i < 30; i++) {
          body.applyForce(Vector2(100, 0));
          e.update(1 / 60);
        }
        expect(body.velocity.x, closeTo(50, 5));
      });
    });

    test('setBodyFixedRotation locks and frees a live body', () {
      final e = engine();
      if (noBox2D(e)) return markTestSkipped('no native Box2D');
      final wheel = ball(0, 0, gravity: false)..angularVelocity = 3;
      e.addBody(wheel);
      runFrames(e, 10);
      expect(wheel.angle, greaterThan(0.3), reason: 'it turns');
      e.setBodyFixedRotation(wheel, true);
      runFrames(e, 1);
      final angle = wheel.angle;
      wheel.angularVelocity = 3;
      runFrames(e, 10);
      expect(wheel.angle, closeTo(angle, 1e-3), reason: 'locked');
      e.setBodyFixedRotation(wheel, false);
      wheel.angularVelocity = 3;
      runFrames(e, 10);
      expect(wheel.angle, greaterThan(angle + 0.3), reason: 'free again');
    });

    test(
      'a delta that is not a positive number runs no step and harms none',
      () {
        final e = engine();
        if (noBox2D(e)) return markTestSkipped('no native Box2D');
        final body = ball(0, 0);
        e.addBody(body);
        for (final dt in [double.nan, double.infinity, -1.0, 0.0]) {
          e.update(dt);
          expect(body.position.y, 0, reason: '$dt');
          expect(e.alpha.isFinite, isTrue, reason: '$dt');
        }
        runFrames(e, 60);
        expect(body.position.y, greaterThan(400), reason: 'it still falls');
      },
    );
  });

  group('Box2D', () {
    test('bodies added before initialize go into the native world', () {
      final e = Box2DPhysicsEngine();
      addTearDown(e.dispose);
      final ground = floor();
      final crate = ball(0, -100);
      final buddy = ball(30, -100);
      e
        ..addBody(ground)
        ..addBody(crate)
        ..addBody(buddy)
        ..addDistanceJoint(crate, buddy, length: 30)
        ..initialize();
      if (!e.isNative) return markTestSkipped('no native Box2D');
      expect(e.bodies, [ground, crate, buddy]);
      expect(e.joints, hasLength(1), reason: 'the joint came too');
      runFrames(e, 120);
      expect(crate.position.y, closeTo(-5, 1), reason: 'it landed natively');
      expect(
        (buddy.position - crate.position).length,
        closeTo(30, 1.5),
        reason: 'held',
      );
    });

    test('says whether it runs natively', () {
      final e = Box2DPhysicsEngine();
      addTearDown(e.dispose);
      expect(e.isNative, isFalse, reason: 'not initialised: pure Dart');
      e.initialize();
      expect(e.isNative, e.stats['backend'] == 'box2d_v3');
      expect(Physics2DEngine.pureDart().isNative, isFalse);
    });
  });
}
