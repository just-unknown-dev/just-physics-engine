import 'package:flutter/material.dart' show Offset, Rect;
import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  // ── Circle2DShape collision ─────────────────────────────────────────────────

  group('Circle2DShape collision', () {
    test('overlapping circles produce a collision manifold', () {
      final a = Circle2DShape(10.0);
      final b = Circle2DShape(10.0);
      final m = a.manifoldWith(const Offset(0, 0), b, const Offset(15, 0));
      expect(m.isColliding, isTrue);
      expect(m.penetration, closeTo(5.0, 1e-6));
      expect(m.normal.dx, closeTo(1.0, 1e-6));
    });

    test('separated circles return no collision', () {
      final a = Circle2DShape(5.0);
      final b = Circle2DShape(5.0);
      final m = a.manifoldWith(const Offset(0, 0), b, const Offset(20, 0));
      expect(m.isColliding, isFalse);
    });

    test('concentric circles use fallback normal', () {
      final a = Circle2DShape(10.0);
      final b = Circle2DShape(10.0);
      // distance == 0: fallback normal should be (1, 0)
      final m = a.manifoldWith(const Offset(0, 0), b, const Offset(0, 0));
      expect(m.isColliding, isTrue);
      expect(m.normal, const Offset(1, 0));
    });

    test('boundsAt returns correct AABB', () {
      final s = Circle2DShape(5.0);
      final r = s.boundsAt(const Offset(10, 20));
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
      final m = a.manifoldWith(const Offset(0, 0), b, const Offset(15, 0));
      expect(m.isColliding, isTrue);
      expect(m.penetration, greaterThan(0));
    });

    test('separated rectangles return no collision', () {
      final a = Rectangle2DShape(10, 10);
      final b = Rectangle2DShape(10, 10);
      final m = a.manifoldWith(const Offset(0, 0), b, const Offset(30, 0));
      expect(m.isColliding, isFalse);
    });

    test('boundsAt returns correct AABB', () {
      final s = Rectangle2DShape(20, 10);
      final r = s.boundsAt(const Offset(50, 50));
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
      final m = a.manifoldWith(const Offset(0, 0), b, const Offset(15, 0));
      expect(m.isColliding, isTrue);
      expect(m.penetration, greaterThan(0));
    });

    test('separated polygons return no collision', () {
      final a = makeSquare(10);
      final b = makeSquare(10);
      final m = a.manifoldWith(const Offset(0, 0), b, const Offset(30, 0));
      expect(m.isColliding, isFalse);
    });

    test('polygon vs circle — overlapping produces collision', () {
      final poly = makeSquare(20);
      final circle = Circle2DShape(5.0);
      final m = poly.manifoldWith(
        const Offset(0, 0),
        circle,
        const Offset(8, 0),
      );
      expect(m.isColliding, isTrue);
    });

    test('polygon vs circle — separated returns no collision', () {
      final poly = makeSquare(10);
      final circle = Circle2DShape(5.0);
      final m = poly.manifoldWith(
        const Offset(0, 0),
        circle,
        const Offset(50, 0),
      );
      expect(m.isColliding, isFalse);
    });

    test('boundsAt returns tight AABB around vertices', () {
      final poly = makeSquare(10);
      final r = poly.boundsAt(const Offset(100, 200));
      expect(r.left, closeTo(95, 1e-9));
      expect(r.top, closeTo(195, 1e-9));
      expect(r.right, closeTo(105, 1e-9));
      expect(r.bottom, closeTo(205, 1e-9));
    });

    test('empty polygon boundsAt returns Rect.zero', () {
      final poly = Polygon2DShape([]);
      expect(poly.boundsAt(const Offset(0, 0)), equals(Rect.zero));
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
}
