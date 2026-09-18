// Behaviour a 2D platformer depends on, and that used to differ between the
// pure-Dart (web) backend and the Box2D FFI (native) backend.
//
// The pure-Dart assertions here are the oracle the native overrides are held
// to. The final group replays the same expectations against the native
// backend, skipping when it is unavailable rather than failing — CI without
// the Box2D submodule should report "skipped", not "broken".

import 'package:flutter/material.dart' show Offset, Rect;
import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  group('Fixed timestep', () {
    /// Free-fall distance after [seconds] of simulation, stepped at [hz].
    double fallDistance({required double hz, required double seconds}) {
      final engine = PhysicsEngine.pureDart()..initialize();
      final body = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(1),
        drag: 0,
      );
      engine.addBody(body);
      final dt = 1.0 / hz;
      for (var i = 0; i < (seconds * hz).round(); i++) {
        engine.update(dt);
      }
      final y = body.position.y;
      engine.dispose();
      return y;
    }

    test('fall distance is independent of frame rate', () {
      final at60 = fallDistance(hz: 60, seconds: 1);
      final at144 = fallDistance(hz: 144, seconds: 1);

      // Near the analytic 0.5*g*t^2 = 490.5 units and — the actual point —
      // near each other. Before the accumulator these diverged purely from
      // refresh rate, so the same jump cleared different gaps on different
      // monitors.
      //
      // The residual is bounded by one sub-step of fall (~16.4 units at t=1s):
      // floating-point accumulation means one rate can bank a leftover the
      // other consumed. That leftover is exactly what `alpha` is for, and the
      // renderer interpolates it away.
      const oneSubStepOfFall = 17.0;
      expect(at60, closeTo(490.5, 20));
      expect(at144, closeTo(at60, oneSubStepOfFall));
    });

    test('alpha exposes the unconsumed remainder', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      engine.update(PhysicsEngine.fixedDeltaTime / 2);
      expect(engine.alpha, closeTo(0.5, 1e-6));
      engine.dispose();
    });

    test('opting out reports no interpolation remainder', () {
      final engine = PhysicsEngine.pureDart(fixedTimestep: false)..initialize();
      engine.update(1 / 60);
      expect(engine.alpha, 0.0);
      engine.dispose();
    });

    test('a long stall is clamped instead of spiralling', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      final body = PhysicsBody(position: Vector2(0, 0), shape: CircleShape(1));
      engine.addBody(body);

      // Ten simulated seconds arriving as one frame must run at most
      // maxSubSteps, not 600 of them.
      engine.update(10);

      expect(body.position.y, lessThan(50));
      engine.dispose();
    });
  });

  group('gravityScale', () {
    double fallWith({double scale = 1.0, bool useGravity = true}) {
      final engine = PhysicsEngine.pureDart()..initialize();
      final body = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(1),
        drag: 0,
        gravityScale: scale,
        useGravity: useGravity,
      );
      engine.addBody(body);
      for (var i = 0; i < 60; i++) {
        engine.update(1 / 60);
      }
      final y = body.position.y;
      engine.dispose();
      return y;
    }

    test('halving the scale roughly halves the fall', () {
      expect(fallWith(scale: 0.5), closeTo(fallWith() / 2, 5));
    });

    test('zero scale is weightless', () {
      expect(fallWith(scale: 0), closeTo(0, 1e-9));
    });

    test('useGravity=false overrides a non-zero scale', () {
      expect(fallWith(scale: 2, useGravity: false), closeTo(0, 1e-9));
    });

    test('effectiveGravityScale folds both inputs', () {
      final body = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(1),
        gravityScale: 2,
      );
      expect(body.effectiveGravityScale, 2.0);
      body.useGravity = false;
      expect(body.effectiveGravityScale, 0.0);
    });
  });

  group('Kinematic bodies', () {
    test('move by velocity and ignore gravity', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      final platform = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(100, 20),
        bodyType: BodyType.kinematic,
      )..velocity.setValues(60, 0);
      engine.addBody(platform);

      for (var i = 0; i < 60; i++) {
        engine.update(1 / 60);
      }

      expect(platform.position.x, closeTo(60, 1));
      expect(platform.position.y, closeTo(0, 1e-9), reason: 'no gravity');
      engine.dispose();
    });

    test('cannot be pushed by a heavy dynamic body resting on them', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      final platform = PhysicsBody(
        position: Vector2(0, 100),
        shape: RectangleShape(200, 20),
        bodyType: BodyType.kinematic,
      );
      final crate = PhysicsBody(
        position: Vector2(0, 80),
        shape: RectangleShape(20, 20),
        mass: 50,
      );
      engine
        ..addBody(platform)
        ..addBody(crate);

      for (var i = 0; i < 60; i++) {
        engine.update(1 / 60);
      }

      expect(platform.position.x, closeTo(0, 1e-9));
      expect(platform.position.y, closeTo(100, 1e-9));
      engine.dispose();
    });

    test('never fall asleep while creeping below the sleep threshold', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      final platform = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(100, 20),
        bodyType: BodyType.kinematic,
      )..velocity.setValues(1, 0); // well under sleepVelocityThreshold of 5
      engine.addBody(platform);

      for (var i = 0; i < 180; i++) {
        engine.update(1 / 60);
      }

      expect(platform.isAwake, isTrue);
      expect(platform.position.x, closeTo(3, 0.2));
      engine.dispose();
    });

    test('mass <= 0 still wins over an explicit dynamic type', () {
      final body = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(1),
        mass: 0,
      );
      expect(body.effectiveBodyType, BodyType.static);
      expect(body.solverInverseMass, 0.0);
    });
  });

  group('One-way platforms', () {
    /// Moves a body toward a one-way platform and reports whether any contact
    /// was resolved.
    bool resolves({
      required double startY,
      required double vy,
      OneWayDirection direction = OneWayDirection.fromAbove,
    }) {
      final engine = PhysicsEngine.pureDart()..initialize();
      final platform = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(200, 10),
        mass: 0,
        isOneWay: true,
        oneWayDirection: direction,
      );
      final mover = PhysicsBody(
        position: Vector2(0, startY),
        shape: RectangleShape(20, 20),
        useGravity: false,
        drag: 0,
      )..velocity.setValues(0, vy);
      engine
        ..addBody(platform)
        ..addBody(mover);

      for (var i = 0; i < 40; i++) {
        engine.update(1 / 60);
        if ((engine.stats['resolvedCollisions'] as int) > 0) {
          engine.dispose();
          return true;
        }
      }
      engine.dispose();
      return false;
    }

    test('lands when approached from above', () {
      expect(resolves(startY: -40, vy: 200), isTrue);
    });

    test('passes through when approached from below', () {
      expect(resolves(startY: 40, vy: -200), isFalse);
    });

    test('fromBelow inverts the solid side', () {
      expect(
        resolves(startY: 40, vy: -200, direction: OneWayDirection.fromBelow),
        isTrue,
      );
    });

    test('two one-way bodies collide normally', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(40, 40),
        mass: 0,
        isOneWay: true,
      );
      final b = PhysicsBody(
        position: Vector2(0, 30),
        shape: RectangleShape(40, 40),
        isOneWay: true,
        useGravity: false,
        drag: 0,
      )..velocity.setValues(0, -100);
      engine
        ..addBody(a)
        ..addBody(b);

      var resolved = false;
      for (var i = 0; i < 20 && !resolved; i++) {
        engine.update(1 / 60);
        resolved = (engine.stats['resolvedCollisions'] as int) > 0;
      }

      expect(resolved, isTrue);
      engine.dispose();
    });
  });

  group('Runtime body mutation', () {
    test('setBodyTransform repositions and wakes', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      final body = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(10),
        useGravity: false,
      );
      engine.addBody(body);
      body.isAwake = false;

      engine.setBodyTransform(body, 500, -250, angle: 1.5);

      expect(body.position.x, 500);
      expect(body.position.y, -250);
      expect(body.angle, 1.5);
      expect(body.isAwake, isTrue);
      engine.dispose();
    });

    test('setBodyTransform survives the next step', () {
      // The regression this feature exists for: on the native backend a
      // position write used to be discarded by the sync-back after the step,
      // so respawns and checkpoints silently did nothing on desktop/mobile
      // while working fine on web.
      final engine = PhysicsEngine.pureDart()..initialize();
      final body = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(10),
        useGravity: false,
        drag: 0,
      );
      engine.addBody(body);

      engine.setBodyTransform(body, 1000, 500);
      engine.update(1 / 60);

      expect(body.position.x, closeTo(1000, 1));
      expect(body.position.y, closeTo(500, 1));
      engine.dispose();
    });

    test('setBodyFilter leaves omitted fields alone', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      final body = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(1),
        categoryBits: 0x0004,
        maskBits: 0x00FF,
        groupIndex: -2,
      );
      engine.addBody(body);

      engine.setBodyFilter(body, maskBits: 0x0001);

      expect(body.categoryBits, 0x0004);
      expect(body.maskBits, 0x0001);
      expect(body.groupIndex, -2);
      engine.dispose();
    });

    test('a filter change takes effect on the next step', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      final ground = PhysicsBody(
        position: Vector2(0, 100),
        shape: RectangleShape(200, 20),
        mass: 0,
      );
      final faller = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(20, 20),
        drag: 0,
      );
      engine
        ..addBody(ground)
        ..addBody(faller);

      engine.setBodyFilter(faller, maskBits: 0x0000);
      for (var i = 0; i < 60; i++) {
        engine.update(1 / 60);
      }

      expect(
        faller.position.y,
        greaterThan(150),
        reason: 'mask cleared, so the ground should not stop it',
      );
      engine.dispose();
    });

    test('applyLinearImpulse preserves existing momentum', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      final body = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(1),
        mass: 2,
        useGravity: false,
      )..velocity.setValues(100, 0);
      engine.addBody(body);

      engine.applyLinearImpulse(body, 0, -20);

      expect(body.velocity.x, 100, reason: 'existing motion is kept');
      expect(body.velocity.y, closeTo(-10, 1e-9), reason: 'impulse / mass');
      engine.dispose();
    });

    test('applyLinearImpulse does nothing to a static body', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      final body = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(1),
        mass: 0,
      );
      engine.addBody(body);

      engine.applyLinearImpulse(body, 500, 500);

      expect(body.velocity.x, 0.0);
      expect(body.velocity.y, 0.0);
      engine.dispose();
    });

    test('friction, restitution and damping are settable at runtime', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      final body = PhysicsBody(position: Vector2(0, 0), shape: CircleShape(1));
      engine.addBody(body);

      engine
        ..setBodyFriction(body, 0.01)
        ..setBodyRestitution(body, 0.9)
        ..setBodyDamping(body, 0.5);

      expect(body.friction, 0.01);
      expect(body.restitution, 0.9);
      expect(body.drag, 0.5);
      engine.dispose();
    });

    test('setBodyType and setBodyGravityScale write through', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      final body = PhysicsBody(position: Vector2(0, 0), shape: CircleShape(1));
      engine.addBody(body);

      engine
        ..setBodyType(body, BodyType.kinematic)
        ..setBodyGravityScale(body, 0.25);

      expect(body.isKinematic, isTrue);
      expect(body.gravityScale, 0.25);
      engine.dispose();
    });

    test('setBodyOneWay sets both flag and direction', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      final body = PhysicsBody(position: Vector2(0, 0), shape: CircleShape(1));
      engine.addBody(body);

      engine.setBodyOneWay(body, true, direction: OneWayDirection.fromLeft);

      expect(body.isOneWay, isTrue);
      expect(body.oneWayDirection, OneWayDirection.fromLeft);
      engine.dispose();
    });
  });

  group('Ray casting', () {
    /// A world with two horizontal slabs stacked below the origin.
    (PhysicsEngine, PhysicsBody near, PhysicsBody far) stackedSlabs() {
      final engine = PhysicsEngine.pureDart()..initialize();
      final near = PhysicsBody(
        position: Vector2(0, 100),
        shape: RectangleShape(200, 20),
        mass: 0,
      );
      final far = PhysicsBody(
        position: Vector2(0, 300),
        shape: RectangleShape(200, 20),
        mass: 0,
      );
      engine
        ..addBody(near)
        ..addBody(far);
      return (engine, near, far);
    }

    test('returns the nearer of two stacked bodies', () {
      final (engine, near, _) = stackedSlabs();

      final hit = engine.castRay(
        Ray(origin: Offset.zero, direction: const Offset(0, 1)),
      );

      expect(hit, isNotNull);
      expect(identical(hit!.body, near), isTrue);
      engine.dispose();
    });

    test('castRayAll reports both, nearest first', () {
      final (engine, _, _) = stackedSlabs();

      final hits = engine.castRayAll(
        Ray(origin: Offset.zero, direction: const Offset(0, 1)),
      );

      expect(hits.length, 2);
      expect(hits.first.distance, lessThan(hits.last.distance));
      engine.dispose();
    });

    test('exclude skips a body', () {
      final (engine, near, far) = stackedSlabs();

      final hit = engine.castRay(
        Ray(origin: Offset.zero, direction: const Offset(0, 1)),
        exclude: {near},
      );

      expect(hit, isNotNull);
      expect(identical(hit!.body, far), isTrue);
      engine.dispose();
    });

    test('a miss returns null', () {
      final (engine, _, _) = stackedSlabs();

      // Straight along +X, where nothing sits.
      final hit = engine.castRay(
        Ray(origin: Offset.zero, direction: const Offset(1, 0)),
      );

      expect(hit, isNull);
      engine.dispose();
    });
  });

  group('Box2D native backend', () {
    late PhysicsEngine engine;

    setUp(() => engine = PhysicsEngine()..initialize());
    tearDown(() => engine.dispose());

    // Gate on the reported backend rather than on Platform.isWindows: a
    // machine without the Box2D submodule built falls back to pure Dart and
    // should skip these, not fail them.
    bool nativeLive() => engine.stats['backend'] == 'box2d_v3';

    test('reports which backend is live', () {
      expect(
        engine.stats['backend'],
        anyOf('box2d_v3', 'dart_fallback', 'dart_fallback_web_stub'),
      );
    });

    test('setBodyTransform reaches the native simulation', () {
      if (!nativeLive()) {
        markTestSkipped('native Box2D backend unavailable');
        return;
      }
      final body = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(10),
        useGravity: false,
        drag: 0,
      );
      engine.addBody(body);

      engine
        ..setBodyTransform(body, 1000, 500)
        ..update(1 / 60);

      expect(body.position.x, closeTo(1000, 2));
      expect(body.position.y, closeTo(500, 2));
    });

    test('one-way platforms pass through from below', () {
      if (!nativeLive()) {
        markTestSkipped('native Box2D backend unavailable');
        return;
      }
      final platform = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(200, 10),
        mass: 0,
        isOneWay: true,
      );
      final mover = PhysicsBody(
        position: Vector2(0, 60),
        shape: RectangleShape(20, 20),
        useGravity: false,
        drag: 0,
        fixedRotation: true,
      )..velocity.setValues(0, -300);
      engine
        ..addBody(platform)
        ..addBody(mover);

      for (var i = 0; i < 30; i++) {
        engine.update(1 / 60);
      }

      expect(
        mover.position.y,
        lessThan(-20),
        reason: 'should have passed through the platform',
      );
    });

    test('raycast uses the native broad phase and agrees with pure Dart', () {
      if (!nativeLive()) {
        markTestSkipped('native Box2D backend unavailable');
        return;
      }
      final near = PhysicsBody(
        position: Vector2(0, 100),
        shape: RectangleShape(200, 20),
        mass: 0,
      );
      final far = PhysicsBody(
        position: Vector2(0, 300),
        shape: RectangleShape(200, 20),
        mass: 0,
      );
      engine
        ..addBody(near)
        ..addBody(far)
        // Native bodies only enter the broad-phase tree once stepped.
        ..update(1 / 60);

      final hit = engine.castRay(
        Ray(origin: Offset.zero, direction: const Offset(0, 1)),
      );
      expect(hit, isNotNull);
      expect(identical(hit!.body, near), isTrue);
      // Top face of the near slab: centre 100, half-height 10.
      expect(hit.point.dy, closeTo(90, 2));

      final all = engine.castRayAll(
        Ray(origin: Offset.zero, direction: const Offset(0, 1)),
      );
      expect(all.length, 2);
      expect(all.first.distance, lessThan(all.last.distance));

      final excluded = engine.castRay(
        Ray(origin: Offset.zero, direction: const Offset(0, 1)),
        exclude: {near},
      );
      expect(identical(excluded!.body, far), isTrue);
    });

    test('queryAABB reports overlapping bodies', () {
      if (!nativeLive()) {
        markTestSkipped('native Box2D backend unavailable');
        return;
      }
      final inside = PhysicsBody(
        position: Vector2(0, 100),
        shape: RectangleShape(40, 40),
        mass: 0,
      );
      final outside = PhysicsBody(
        position: Vector2(5000, 5000),
        shape: RectangleShape(40, 40),
        mass: 0,
      );
      engine
        ..addBody(inside)
        ..addBody(outside)
        ..update(1 / 60);

      final found = engine.queryAABB(const Rect.fromLTRB(-100, 0, 100, 200));

      expect(found, contains(inside));
      expect(found, isNot(contains(outside)));
    });

    test('kinematic platforms are immovable', () {
      if (!nativeLive()) {
        markTestSkipped('native Box2D backend unavailable');
        return;
      }
      final platform = PhysicsBody(
        position: Vector2(0, 200),
        shape: RectangleShape(400, 20),
        bodyType: BodyType.kinematic,
      );
      final crate = PhysicsBody(
        position: Vector2(0, 150),
        shape: RectangleShape(20, 20),
        mass: 50,
        fixedRotation: true,
      );
      engine
        ..addBody(platform)
        ..addBody(crate);

      for (var i = 0; i < 60; i++) {
        engine.update(1 / 60);
      }

      expect(platform.position.y, closeTo(200, 1));
    });
  });

  group('Contact events', () {
    /// Drops a body onto a static slab and counts the begin events.
    ///
    /// This is the mechanism `PhysicsBodyComponent.isGrounded` is built from,
    /// and therefore the mechanism every jump in a platformer depends on.
    int countLandingContacts(PhysicsEngine engine) {
      final ground = PhysicsBody(
        position: Vector2(0, 300),
        shape: RectangleShape(400, 40),
        mass: 0,
      );
      final faller = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(32, 48),
        fixedRotation: true,
        drag: 0,
      );
      engine
        ..addBody(ground)
        ..addBody(faller);

      var contacts = 0;
      for (var i = 0; i < 120; i++) {
        engine
          ..update(1 / 60)
          ..pollContactBeginEvents((a, b, nx, ny) => contacts++);
      }
      return contacts;
    }

    test('a landing raises a contact begin event on the pure-Dart backend', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      expect(countLandingContacts(engine), greaterThan(0));
      engine.dispose();
    });

    test('a landing raises a contact begin event on the native backend', () {
      // Regression: b2DefaultShapeDef zero-initialises, so enableContactEvents
      // defaults to false. The wrapper never set it, which meant Box2D emitted
      // no touch events at all — contacts resolved correctly, so bodies landed
      // and stacked as expected, but pollContactBeginEvents stayed permanently
      // silent. isGrounded was therefore always false on desktop and mobile
      // and always correct on web: a platformer where the player could not
      // jump, on every platform except the one most people test on.
      final engine = PhysicsEngine()..initialize();
      if (engine.stats['backend'] != 'box2d_v3') {
        markTestSkipped('native Box2D backend unavailable');
        engine.dispose();
        return;
      }
      expect(countLandingContacts(engine), greaterThan(0));
      engine.dispose();
    });

    test('a sensor overlap raises a sensor event', () {
      final engine = PhysicsEngine()..initialize();
      final trigger = PhysicsBody(
        position: Vector2(0, 300),
        shape: RectangleShape(200, 200),
        mass: 0,
        isSensor: true,
      );
      final visitor = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(32, 32),
        fixedRotation: true,
        drag: 0,
      );
      engine
        ..addBody(trigger)
        ..addBody(visitor);

      var enters = 0;
      for (var i = 0; i < 120; i++) {
        engine
          ..update(1 / 60)
          ..pollSensorBeginEvents((sensor, other) => enters++);
      }

      expect(enters, greaterThan(0));
      engine.dispose();
    });
  });
}
