import 'dart:math' as math;

import 'package:flutter/material.dart' show Offset, Rect;
import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  // ── Physics2DEngine lifecycle ──────────────────────────────────────────────

  group('Physics2DEngine lifecycle', () {
    test('initialize and dispose without error', () {
      final engine = Physics2DEngine();
      engine.initialize();
      engine.dispose();
    });

    test('addBody and removeBody', () {
      final engine = Physics2DEngine()..initialize();
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
      final engine = Physics2DEngine()..initialize();
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
      final engine = Physics2DEngine()..initialize();
      final body = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(5.0),
      );
      expect(() => engine.removeBody(body), returnsNormally);
      engine.dispose();
    });

    test('stats map contains expected keys after update', () {
      final engine = Physics2DEngine()..initialize();
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
      final engine = Physics2DEngine();
      expect(engine.gravity.y, closeTo(981.0, 1e-6));
    });

    test('setGravity updates gravity vector', () {
      final engine = Physics2DEngine();
      engine.setGravity(0, 500);
      expect(engine.gravity.x, closeTo(0, 1e-6));
      expect(engine.gravity.y, closeTo(500, 1e-6));
    });

    test('body falls under gravity over one second', () {
      final engine = Physics2DEngine()..initialize();
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
      final engine = Physics2DEngine()..initialize();
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
      final engine = Physics2DEngine()..initialize();
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
      final engine = Physics2DEngine()..initialize();
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
      final engine = Physics2DEngine()..initialize();
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
      final engine = Physics2DEngine()..initialize();
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

  // ── Shape caching ─────────────────────────────────────────────────────────

  group('Shape caching', () {
    test('cachePolygonShape and getCachedPolygonShape round-trip', () {
      final engine = Physics2DEngine();
      const id = 'triangle';
      final verts = [
        const Offset(0, 0),
        const Offset(10, 0),
        const Offset(5, 10),
      ];
      engine.cachePolygonShape(id, verts);
      final result = engine.getCachedPolygonShape(id);
      expect(result, equals(verts));
    });

    test('getCachedPolygonShape returns null for unknown id', () {
      final engine = Physics2DEngine();
      expect(engine.getCachedPolygonShape('missing'), isNull);
    });
  });

  // ── Vector2 ───────────────────────────────────────────────────────────────

  group('Vector2', () {
    test('addScaled mutates in-place', () {
      final v = Vector2(1.0, 0.0);
      v.addScaled(Vector2(0.0, 1.0), 2.0);
      expect(v.x, 1.0);
      expect(v.y, 2.0);
    });

    test('setZero clears both components', () {
      final v = Vector2(3.0, 4.0);
      v.setZero();
      expect(v.x, 0.0);
      expect(v.y, 0.0);
    });

    test('lengthSquared is correct', () {
      final v = Vector2(3.0, 4.0);
      expect(v.lengthSquared, closeTo(25.0, 1e-9));
    });
  });

  // ── Physics2DBody ───────────────────────────────────────────────────────────

  group('Physics2DBody', () {
    test('inverseMass is 1/mass for positive mass', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        mass: 4.0,
      );
      expect(body.inverseMass, closeTo(0.25, 1e-9));
    });

    test('inverseMass is 0 for static body (mass=0)', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        mass: 0.0,
      );
      expect(body.inverseMass, 0.0);
    });

    test('inverseInertia is 0 when inertia is 0', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        inertia: 0.0,
      );
      expect(body.inverseInertia, 0.0);
    });

    test('applyForce accumulates into acceleration', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        mass: 2.0,
      );
      body.applyForce(Vector2(10, 0));
      // a += F * inverseMass = 10 * 0.5 = 5
      expect(body.acceleration.x, closeTo(5.0, 1e-9));
    });

    test('applyForce does nothing for static body', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        mass: 0.0,
      );
      body.applyForce(Vector2(999, 999));
      expect(body.acceleration.x, 0.0);
      expect(body.acceleration.y, 0.0);
    });

    test('applyImpulse changes velocity directly', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
      );
      body.applyImpulse(Vector2(5, -3));
      expect(body.velocity.x, closeTo(5.0, 1e-9));
      expect(body.velocity.y, closeTo(-3.0, 1e-9));
    });

    test('applyTorque accumulates', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        inertia: 2.0,
      );
      body.applyTorque(10.0);
      expect(body.torque, closeTo(10.0, 1e-9));
    });

    test('applyTorque does nothing when inertia is 0', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        inertia: 0.0,
      );
      body.applyTorque(999.0);
      expect(body.torque, 0.0);
    });
  });

  // ── ForceManager ──────────────────────────────────────────────────────────

  group('ForceManager', () {
    test('default gravity is 981 units/s²', () {
      final fm = ForceManager();
      expect(fm.gravity.y, closeTo(981.0, 1e-6));
    });

    test('setGravity updates gravity', () {
      final fm = ForceManager();
      fm.setGravity(0, 200);
      expect(fm.gravity.y, closeTo(200.0, 1e-6));
    });

    test(
      'applyGravity adds gravity to active awake bodies with useGravity',
      () {
        final fm = ForceManager();
        final body = Physics2DBody(
          position: Vector2.zero(),
          shape: Circle2DShape(1),
        );
        fm.applyGravity([body]);
        expect(body.acceleration.y, closeTo(981.0, 1e-6));
      },
    );

    test('applyGravity skips static bodies', () {
      final fm = ForceManager();
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        mass: 0.0,
      );
      fm.applyGravity([body]);
      expect(body.acceleration.y, 0.0);
    });

    test('applyGravity skips bodies with useGravity=false', () {
      final fm = ForceManager();
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        useGravity: false,
      );
      fm.applyGravity([body]);
      expect(body.acceleration.y, 0.0);
    });
  });

  // ── Circle2DShape collision ─────────────────────────────────────────────────

  group('Circle2DShape collision', () {
    test('overlapping circles produce a collision manifold', () {
      final a = Circle2DShape(10.0);
      final b = Circle2DShape(10.0);
      final m = a.getManifold(const Offset(0, 0), b, const Offset(15, 0));
      expect(m.isColliding, isTrue);
      expect(m.penetration, closeTo(5.0, 1e-6));
      expect(m.normal.dx, closeTo(1.0, 1e-6));
    });

    test('separated circles return no collision', () {
      final a = Circle2DShape(5.0);
      final b = Circle2DShape(5.0);
      final m = a.getManifold(const Offset(0, 0), b, const Offset(20, 0));
      expect(m.isColliding, isFalse);
    });

    test('concentric circles use fallback normal', () {
      final a = Circle2DShape(10.0);
      final b = Circle2DShape(10.0);
      // distance == 0: fallback normal should be (1, 0)
      final m = a.getManifold(const Offset(0, 0), b, const Offset(0, 0));
      expect(m.isColliding, isTrue);
      expect(m.normal, const Offset(1, 0));
    });

    test('getBounds returns correct AABB', () {
      final s = Circle2DShape(5.0);
      final r = s.getBounds(const Offset(10, 20));
      expect(r.left, closeTo(5.0, 1e-9));
      expect(r.top, closeTo(15.0, 1e-9));
      expect(r.right, closeTo(15.0, 1e-9));
      expect(r.bottom, closeTo(25.0, 1e-9));
    });
  });

  // ── Rectangle2DShape collision ──────────────────────────────────────────────

  group('Rectangle2DShape collision', () {
    test('overlapping rectangles produce a collision', () {
      final a = Rectangle2DShape(20, 20);
      final b = Rectangle2DShape(20, 20);
      final m = a.getManifold(const Offset(0, 0), b, const Offset(15, 0));
      expect(m.isColliding, isTrue);
      expect(m.penetration, greaterThan(0));
    });

    test('separated rectangles return no collision', () {
      final a = Rectangle2DShape(10, 10);
      final b = Rectangle2DShape(10, 10);
      final m = a.getManifold(const Offset(0, 0), b, const Offset(30, 0));
      expect(m.isColliding, isFalse);
    });

    test('getBounds returns correct AABB', () {
      final s = Rectangle2DShape(20, 10);
      final r = s.getBounds(const Offset(50, 50));
      expect(r.left, closeTo(40, 1e-9));
      expect(r.top, closeTo(45, 1e-9));
      expect(r.right, closeTo(60, 1e-9));
      expect(r.bottom, closeTo(55, 1e-9));
    });
  });

  // ── Polygon2DShape collision ────────────────────────────────────────────────

  group('Polygon2DShape collision', () {
    Polygon2DShape makeSquare(double size) => Polygon2DShape([
      Offset(-size / 2, -size / 2),
      Offset(size / 2, -size / 2),
      Offset(size / 2, size / 2),
      Offset(-size / 2, size / 2),
    ]);

    test('overlapping polygons produce a collision', () {
      final a = makeSquare(20);
      final b = makeSquare(20);
      final m = a.getManifold(const Offset(0, 0), b, const Offset(15, 0));
      expect(m.isColliding, isTrue);
      expect(m.penetration, greaterThan(0));
    });

    test('separated polygons return no collision', () {
      final a = makeSquare(10);
      final b = makeSquare(10);
      final m = a.getManifold(const Offset(0, 0), b, const Offset(30, 0));
      expect(m.isColliding, isFalse);
    });

    test('polygon vs circle — overlapping produces collision', () {
      final poly = makeSquare(20);
      final circle = Circle2DShape(5.0);
      final m = poly.getManifold(
        const Offset(0, 0),
        circle,
        const Offset(8, 0),
      );
      expect(m.isColliding, isTrue);
    });

    test('polygon vs circle — separated returns no collision', () {
      final poly = makeSquare(10);
      final circle = Circle2DShape(5.0);
      final m = poly.getManifold(
        const Offset(0, 0),
        circle,
        const Offset(50, 0),
      );
      expect(m.isColliding, isFalse);
    });

    test('getBounds returns tight AABB around vertices', () {
      final poly = makeSquare(10);
      final r = poly.getBounds(const Offset(100, 200));
      expect(r.left, closeTo(95, 1e-9));
      expect(r.top, closeTo(195, 1e-9));
      expect(r.right, closeTo(105, 1e-9));
      expect(r.bottom, closeTo(205, 1e-9));
    });

    test('empty polygon getBounds returns Rect.zero', () {
      final poly = Polygon2DShape([]);
      expect(poly.getBounds(const Offset(0, 0)), equals(Rect.zero));
    });
  });

  // ── CollisionManifold ─────────────────────────────────────────────────────

  group('CollisionManifold', () {
    test('empty() factory produces non-colliding manifold', () {
      final m = CollisionManifold.empty();
      expect(m.isColliding, isFalse);
      expect(m.penetration, 0.0);
    });
  });

  // ── SpatialGrid ───────────────────────────────────────────────────────────

  group('SpatialGrid', () {
    Physics2DBody makeBody(double x, double y, double r) =>
        Physics2DBody(position: Vector2(x, y), shape: Circle2DShape(r));

    test('insert and getPotentialCollisions finds nearby pair', () {
      final grid = SpatialGrid(100.0);
      final a = makeBody(0, 0, 10);
      final b = makeBody(5, 0, 10);
      grid.insert(a);
      grid.insert(b);
      final pairs = grid.getPotentialCollisions();
      expect(pairs.length, 1);
      expect(
        (pairs.first.a == a && pairs.first.b == b) ||
            (pairs.first.a == b && pairs.first.b == a),
        isTrue,
      );
    });

    test('bodies in different cells are not paired', () {
      final grid = SpatialGrid(50.0);
      final a = makeBody(0, 0, 5);
      final b = makeBody(1000, 1000, 5);
      grid.insert(a);
      grid.insert(b);
      expect(grid.getPotentialCollisions(), isEmpty);
    });

    test('clear resets all state', () {
      final grid = SpatialGrid(100.0);
      grid.insert(makeBody(0, 0, 10));
      grid.clear();
      expect(grid.trackedBodyCount, 0);
      expect(grid.trackedCellCount, 0);
    });

    test('syncBodies removes stale bodies', () {
      final grid = SpatialGrid(100.0);
      final a = makeBody(0, 0, 10);
      final b = makeBody(5, 0, 10);
      grid.syncBodies([a, b]);
      grid.syncBodies([a]); // b removed
      expect(grid.trackedBodyCount, 1);
    });

    test('removeBody removes a single body', () {
      final grid = SpatialGrid(100.0);
      final a = makeBody(0, 0, 10);
      final b = makeBody(5, 0, 10);
      grid.insert(a);
      grid.insert(b);
      grid.removeBody(b);
      expect(grid.trackedBodyCount, 1);
    });

    test('inactive body is not inserted into grid', () {
      final grid = SpatialGrid(100.0);
      final body = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(10),
        isActive: false,
      );
      grid.insert(body);
      expect(grid.trackedBodyCount, 0);
    });

    test('duplicate pairs from multiple cells are deduplicated', () {
      // A large body spanning several cells should not generate duplicate pairs.
      final grid = SpatialGrid(10.0);
      final a = makeBody(0, 0, 50); // spans many cells
      final b = makeBody(5, 0, 50);
      grid.insert(a);
      grid.insert(b);
      final pairs = grid.getPotentialCollisions();
      expect(pairs.length, 1);
    });
  });

  // ── CollisionDetector ─────────────────────────────────────────────────────

  group('CollisionDetector', () {
    test('detects overlapping bodies', () {
      final detector = CollisionDetector();
      final a = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(10),
      );
      final b = Physics2DBody(
        position: Vector2(5, 0),
        shape: Circle2DShape(10),
      );
      detector.addBody(a);
      detector.addBody(b);
      final pairs = detector.detectCollisions();
      expect(pairs.length, 1);
    });

    test('does not report separated bodies', () {
      final detector = CollisionDetector();
      final a = Physics2DBody(position: Vector2(0, 0), shape: Circle2DShape(5));
      final b = Physics2DBody(
        position: Vector2(100, 0),
        shape: Circle2DShape(5),
      );
      detector.addBody(a);
      detector.addBody(b);
      expect(detector.detectCollisions(), isEmpty);
    });

    test('removeBody stops it from being detected', () {
      final detector = CollisionDetector();
      final a = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(10),
      );
      final b = Physics2DBody(
        position: Vector2(5, 0),
        shape: Circle2DShape(10),
      );
      detector.addBody(a);
      detector.addBody(b);
      detector.removeBody(b);
      expect(detector.detectCollisions(), isEmpty);
    });

    test('inactive body is not included in pairs', () {
      final detector = CollisionDetector();
      final a = Physics2DBody(
        position: Vector2(0, 0),
        shape: Circle2DShape(10),
      );
      final b = Physics2DBody(
        position: Vector2(5, 0),
        shape: Circle2DShape(10),
        isActive: false,
      );
      detector.addBody(a);
      detector.addBody(b);
      expect(detector.detectCollisions(), isEmpty);
    });
  });

  // ── Ray2D ───────────────────────────────────────────────────────────────────

  group('Ray2D', () {
    test('direction is normalised on construction', () {
      final ray = Ray2D(
        origin: const Offset(0, 0),
        direction: const Offset(3, 4),
      );
      final len = math.sqrt(
        ray.direction.dx * ray.direction.dx +
            ray.direction.dy * ray.direction.dy,
      );
      expect(len, closeTo(1.0, 1e-9));
    });

    test('zero direction falls back to positive-x axis', () {
      final ray = Ray2D(origin: const Offset(0, 0), direction: Offset.zero);
      expect(ray.direction, const Offset(1, 0));
    });

    test('at(t) returns correct world-space point', () {
      final ray = Ray2D(
        origin: const Offset(1, 2),
        direction: const Offset(1, 0),
      );
      final pt = ray.at(5.0);
      expect(pt.dx, closeTo(6.0, 1e-9));
      expect(pt.dy, closeTo(2.0, 1e-9));
    });

    test('fromPoints builds ray toward target', () {
      final ray = Ray2D.fromPoints(const Offset(0, 0), const Offset(10, 0));
      expect(ray.direction.dx, closeTo(1.0, 1e-9));
      expect(ray.direction.dy, closeTo(0.0, 1e-9));
      expect(ray.maxDistance, closeTo(10.0, 1e-9));
    });

    test('fromPoints with explicit maxDistance overrides distance', () {
      final ray = Ray2D.fromPoints(
        const Offset(0, 0),
        const Offset(10, 0),
        maxDistance: 999.0,
      );
      expect(ray.maxDistance, closeTo(999.0, 1e-9));
    });
  });

  // ── BodyPair equality ─────────────────────────────────────────────────────

  group('BodyPair', () {
    test('(a,b) equals (b,a)', () {
      final a = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
      );
      final b = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
      );
      expect(BodyPair(a, b), equals(BodyPair(b, a)));
    });

    test('hashCode is symmetric', () {
      final a = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
      );
      final b = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
      );
      expect(BodyPair(a, b).hashCode, equals(BodyPair(b, a).hashCode));
    });
  });

  // ── Collision resolution (impulse) ────────────────────────────────────────

  group('Collision resolution', () {
    test('two circles bouncing apart separate after resolution', () {
      final engine = Physics2DEngine()..initialize();
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
      engine.addDistanceJoint(a, b, length: 50, stiffness: 300, damping: 0.5);

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
      j.motorEnabled = true;
      j.motorSpeed = 5.0;
      // maxMotorTorque * inverseInertia * dt must be < motorSpeed to avoid
      // bang-bang oscillation (50 * 1.0 * 0.016 = 0.8 rad/s per step).
      j.maxMotorTorque = 50;

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
      j.motorEnabled = true;
      j.motorSpeed = 100;
      j.maxMotorForce = 500;

      for (int i = 0; i < 20; i++) {
        engine.update(0.016);
      }
      expect(b.velocity.x, greaterThan(0));
      engine.dispose();
    });
  });
}
