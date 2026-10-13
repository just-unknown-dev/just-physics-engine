// Box2D's distance and shape cast for convex point clouds with a radius, in
// Dart: the pure-Dart (web) half of the character mover. A port of
// b2ShapeDistance (GJK) and b2ShapeCast (conservative advancement) from
// Box2D's distance.c (MIT, Erin Catto), so both backends answer the same.
//
// Everything is in one frame (the caller's, usually relative to the mover)
// and in world units.

import 'dart:math' as math;
import 'dart:ui' show Offset;

/// A convex shape as GJK sees it: its points' convex hull, grown by
/// [radius]. A circle is one point, a capsule or segment two, a box four.
final class ShapeProxy2D {
  /// A proxy of [points] (at least one) grown by [radius].
  const ShapeProxy2D(this.points, this.radius);

  /// The core's points.
  final List<Offset> points;

  /// How far the shape reaches past its core.
  final double radius;

  /// The same proxy moved by [delta].
  ShapeProxy2D shifted(Offset delta) =>
      ShapeProxy2D([for (final p in points) p + delta], radius);

  /// The point furthest along ([dx], [dy]).
  int _support(double dx, double dy) {
    var best = 0;
    var bestValue = points[0].dx * dx + points[0].dy * dy;
    for (var i = 1; i < points.length; i++) {
      final value = points[i].dx * dx + points[i].dy * dy;
      if (value > bestValue) {
        best = i;
        bestValue = value;
      }
    }
    return best;
  }
}

/// The closest points between two proxies' cores.
final class ShapeDistance2D {
  const ShapeDistance2D._(this.pointA, this.pointB, this.normal, this.distance);

  /// The closest point on A's core.
  final Offset pointA;

  /// The closest point on B's core.
  final Offset pointB;

  /// From A towards B; zero when the cores overlap.
  final Offset normal;

  /// Between the cores (radii not taken off); 0 when they overlap.
  final double distance;

  /// Whether the cores overlap, leaving no usable [normal].
  bool get overlaps => distance == 0;

  static const _epsilonSquared = 1.4210854715202004e-14; // FLT_EPSILON²

  /// The distance between [a]'s and [b]'s cores (Box2D's b2ShapeDistance,
  /// without radii).
  static ShapeDistance2D between(ShapeProxy2D a, ShapeProxy2D b) {
    final s = _Simplex();
    final v1 = s.v1
      ..indexA = 0
      ..indexB = 0;
    v1.setPoints(a.points[0], b.points[0]);
    v1.a = 1;
    s.count = 1;

    var nx = 0.0, ny = 0.0;
    final saveA = [0, 0, 0], saveB = [0, 0, 0];
    var iteration = 0;
    while (iteration < 20) {
      final saveCount = s.count;
      for (var i = 0; i < saveCount; i++) {
        saveA[i] = s[i].indexA;
        saveB[i] = s[i].indexB;
      }

      double dx, dy;
      switch (s.count) {
        case 1:
          dx = -s.v1.wx;
          dy = -s.v1.wy;
        case 2:
          (dx, dy) = s.solve2();
        default:
          (dx, dy) = s.solve3();
      }

      // Three points: the origin is inside, so the cores overlap.
      if (s.count == 3) return _overlap(s);
      // A search direction too small to trust: overlapping, near enough.
      if (dx * dx + dy * dy < _epsilonSquared) return _overlap(s);

      nx = dx;
      ny = dy;
      final vertex = s[s.count];
      vertex
        ..indexA = a._support(dx, dy)
        ..indexB = b._support(-dx, -dy)
        ..setPoints(a.points[vertex.indexA], b.points[vertex.indexB]);
      iteration++;

      var duplicate = false;
      for (var i = 0; i < saveCount; i++) {
        if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i]) {
          duplicate = true;
          break;
        }
      }
      if (duplicate) break;
      s.count++;
    }

    final length = math.sqrt(nx * nx + ny * ny);
    final normal = length == 0 ? Offset.zero : Offset(nx / length, ny / length);
    final (pa, pb) = s.witnessPoints();
    return ShapeDistance2D._(pa, pb, normal, (pa - pb).distance);
  }

  static ShapeDistance2D _overlap(_Simplex s) {
    final (pa, pb) = s.witnessPoints();
    return ShapeDistance2D._(pa, pb, Offset.zero, 0);
  }
}

/// Where a shape moving along a translation first touches another.
final class ShapeCastHit2D {
  const ShapeCastHit2D._(this.fraction, this.point, this.normal);

  /// How much of the translation it moves before touching; 0 when it
  /// started out touching (an initial overlap).
  final double fraction;

  /// Where they touch, on the still shape.
  final Offset point;

  /// The still shape's surface normal there, towards the moving one; zero
  /// for an initial overlap.
  final Offset normal;

  /// Whether they touched from the start.
  bool get initialOverlap => fraction == 0 && normal == Offset.zero;
}

/// Box2D's b2ShapeCast: [moving] swept along [translation] against the
/// still [still], by conservative advancement. Null for a miss.
///
/// [linearSlop] is Box2D's B2_LINEAR_SLOP in world units (0.005 m, so 0.5
/// at 100 units per metre). With [canEncroach], a shape already touching
/// may move a little closer instead of reporting an initial overlap.
ShapeCastHit2D? castShape2D(
  ShapeProxy2D still,
  ShapeProxy2D moving,
  Offset translation, {
  required double linearSlop,
  double maxFraction = 1,
  bool canEncroach = false,
}) {
  final totalRadius = still.radius + moving.radius;
  var target = math.max(linearSlop, totalRadius - linearSlop);
  final tolerance = 0.25 * linearSlop;

  var fraction = 0.0;
  var current = moving;
  for (var iteration = 0; iteration < 20; iteration++) {
    final d = ShapeDistance2D.between(still, current);
    if (d.distance < target + tolerance) {
      if (iteration == 0) {
        if (canEncroach && d.distance > 2 * linearSlop) {
          target = d.distance - linearSlop;
        } else {
          // Initial overlap: a common point between the two surfaces.
          final c1 = d.pointA + d.normal * still.radius;
          final c2 = d.pointB - d.normal * moving.radius;
          return ShapeCastHit2D._(0, (c1 + c2) / 2, Offset.zero);
        }
      } else {
        return ShapeCastHit2D._(
          fraction,
          d.pointA + d.normal * still.radius,
          d.normal,
        );
      }
    }
    if (d.overlaps) return null;

    // Approaching each other?
    final denominator = translation.dx * d.normal.dx + translation.dy * d.normal.dy;
    if (denominator >= 0) return null;

    fraction += (target - d.distance) / denominator;
    if (fraction >= maxFraction) return null;
    current = moving.shifted(translation * fraction);
  }
  return null;
}

final class _Vertex {
  double wAx = 0, wAy = 0, wBx = 0, wBy = 0, wx = 0, wy = 0, a = 0;
  int indexA = 0, indexB = 0;

  void setPoints(Offset pa, Offset pb) {
    wAx = pa.dx;
    wAy = pa.dy;
    wBx = pb.dx;
    wBy = pb.dy;
    wx = wAx - wBx;
    wy = wAy - wBy;
  }

  void copy(_Vertex o) {
    wAx = o.wAx;
    wAy = o.wAy;
    wBx = o.wBx;
    wBy = o.wBy;
    wx = o.wx;
    wy = o.wy;
    a = o.a;
    indexA = o.indexA;
    indexB = o.indexB;
  }
}

final class _Simplex {
  final v1 = _Vertex(), v2 = _Vertex(), v3 = _Vertex();
  int count = 0;

  _Vertex operator [](int i) => i == 0 ? v1 : (i == 1 ? v2 : v3);

  static double _cross(double ax, double ay, double bx, double by) =>
      ax * by - ay * bx;

  /// b2CrossSV(s, v) = (-s * v.y, s * v.x).
  static (double, double) _crossSV(double s, double vx, double vy) =>
      (-s * vy, s * vx);

  (double, double) solve2() {
    final w1x = v1.wx, w1y = v1.wy, w2x = v2.wx, w2y = v2.wy;
    final ex = w2x - w1x, ey = w2y - w1y;
    final d12n2 = -(w1x * ex + w1y * ey);
    if (d12n2 <= 0) {
      v1.a = 1;
      count = 1;
      return (-w1x, -w1y);
    }
    final d12n1 = w2x * ex + w2y * ey;
    if (d12n1 <= 0) {
      v2.a = 1;
      count = 1;
      v1.copy(v2);
      return (-w2x, -w2y);
    }
    final inv = 1 / (d12n1 + d12n2);
    v1.a = d12n1 * inv;
    v2.a = d12n2 * inv;
    count = 2;
    return _crossSV(_cross(w1x + w2x, w1y + w2y, ex, ey), ex, ey);
  }

  (double, double) solve3() {
    final w1x = v1.wx, w1y = v1.wy;
    final w2x = v2.wx, w2y = v2.wy;
    final w3x = v3.wx, w3y = v3.wy;

    final e12x = w2x - w1x, e12y = w2y - w1y;
    final d12n1 = w2x * e12x + w2y * e12y;
    final d12n2 = -(w1x * e12x + w1y * e12y);

    final e13x = w3x - w1x, e13y = w3y - w1y;
    final d13n1 = w3x * e13x + w3y * e13y;
    final d13n2 = -(w1x * e13x + w1y * e13y);

    final e23x = w3x - w2x, e23y = w3y - w2y;
    final d23n1 = w3x * e23x + w3y * e23y;
    final d23n2 = -(w2x * e23x + w2y * e23y);

    final n123 = _cross(e12x, e12y, e13x, e13y);
    final d123n1 = n123 * _cross(w2x, w2y, w3x, w3y);
    final d123n2 = n123 * _cross(w3x, w3y, w1x, w1y);
    final d123n3 = n123 * _cross(w1x, w1y, w2x, w2y);

    if (d12n2 <= 0 && d13n2 <= 0) {
      v1.a = 1;
      count = 1;
      return (-w1x, -w1y);
    }
    if (d12n1 > 0 && d12n2 > 0 && d123n3 <= 0) {
      final inv = 1 / (d12n1 + d12n2);
      v1.a = d12n1 * inv;
      v2.a = d12n2 * inv;
      count = 2;
      return _crossSV(_cross(w1x + w2x, w1y + w2y, e12x, e12y), e12x, e12y);
    }
    if (d13n1 > 0 && d13n2 > 0 && d123n2 <= 0) {
      final inv = 1 / (d13n1 + d13n2);
      v1.a = d13n1 * inv;
      v3.a = d13n2 * inv;
      count = 2;
      v2.copy(v3);
      return _crossSV(_cross(w1x + w3x, w1y + w3y, e13x, e13y), e13x, e13y);
    }
    if (d12n1 <= 0 && d23n2 <= 0) {
      v2.a = 1;
      count = 1;
      v1.copy(v2);
      return (-w2x, -w2y);
    }
    if (d13n1 <= 0 && d23n1 <= 0) {
      v3.a = 1;
      count = 1;
      v1.copy(v3);
      return (-w3x, -w3y);
    }
    if (d23n1 > 0 && d23n2 > 0 && d123n1 <= 0) {
      final inv = 1 / (d23n1 + d23n2);
      v2.a = d23n1 * inv;
      v3.a = d23n2 * inv;
      count = 2;
      v1.copy(v3);
      return _crossSV(_cross(w2x + w3x, w2y + w3y, e23x, e23y), e23x, e23y);
    }
    final inv = 1 / (d123n1 + d123n2 + d123n3);
    v1.a = d123n1 * inv;
    v2.a = d123n2 * inv;
    v3.a = d123n3 * inv;
    count = 3;
    return (0, 0);
  }

  (Offset, Offset) witnessPoints() {
    switch (count) {
      case 1:
        return (Offset(v1.wAx, v1.wAy), Offset(v1.wBx, v1.wBy));
      case 2:
        return (
          Offset(v1.a * v1.wAx + v2.a * v2.wAx, v1.a * v1.wAy + v2.a * v2.wAy),
          Offset(v1.a * v1.wBx + v2.a * v2.wBx, v1.a * v1.wBy + v2.a * v2.wBy),
        );
      default:
        final a = Offset(
          v1.a * v1.wAx + v2.a * v2.wAx + v3.a * v3.wAx,
          v1.a * v1.wAy + v2.a * v2.wAy + v3.a * v3.wAy,
        );
        return (a, a);
    }
  }
}
