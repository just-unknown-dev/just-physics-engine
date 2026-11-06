// The pure-Dart engine's broad phase, which is internal: imported from
// its source.

import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/src/physics_2d/physics_2d_engine.dart';

void main() {
  // ── SpatialGrid ───────────────────────────────────────────────────────────

  group('SpatialGrid', () {
    Physics2DBody makeBody(double x, double y, double r) =>
        Physics2DBody(position: Vector2(x, y), shape: Circle2DShape(r));

    test('insert and potentialPairs finds nearby pair', () {
      final grid = SpatialGrid(100.0);
      final a = makeBody(0, 0, 10);
      final b = makeBody(5, 0, 10);
      grid.insert(a);
      grid.insert(b);
      final pairs = grid.potentialPairs();
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
      expect(grid.potentialPairs(), isEmpty);
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
      final pairs = grid.potentialPairs();
      expect(pairs.length, 1);
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

  group('pairs that cannot touch', () {
    Physics2DBody body(double x, Body2DType type, {bool sensor = false}) =>
        Physics2DBody(
          position: Vector2(x, 0),
          shape: Circle2DShape(10),
          bodyType: type,
          isSensor: sensor,
        );

    test('are not made', () {
      final grid = SpatialGrid(100.0);
      grid
        ..insert(body(0, Body2DType.static))
        ..insert(body(5, Body2DType.static))
        ..insert(body(10, Body2DType.kinematic));
      expect(grid.potentialPairs(), isEmpty);
    });

    test(
      'a sensor still sees a kinematic body, and a dynamic body anything',
      () {
        final grid = SpatialGrid(100.0);
        grid
          ..insert(body(0, Body2DType.static, sensor: true))
          ..insert(body(5, Body2DType.kinematic));
        expect(grid.potentialPairs(), hasLength(1));
        grid.insert(body(8, Body2DType.dynamic));
        expect(grid.potentialPairs(), hasLength(3));
      },
    );

    test('ids are reused, and pairs stay unique past 2^15 bodies', () {
      final grid = SpatialGrid(1000.0);
      final many = [
        for (var i = 0; i < 40000; i++) body(-1e6 - i * 50, Body2DType.static),
      ];
      many.forEach(grid.insert);
      final a = body(0, Body2DType.dynamic), b = body(5, Body2DType.dynamic);
      grid
        ..insert(a)
        ..insert(b);
      expect(grid.potentialPairs(), hasLength(1));
      for (final m in many) {
        grid.removeBody(m);
      }
      final c = body(8, Body2DType.dynamic);
      grid.insert(c);
      expect(grid.potentialPairs(), hasLength(3));
    });
  });
}
