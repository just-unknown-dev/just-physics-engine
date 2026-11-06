import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
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
}
