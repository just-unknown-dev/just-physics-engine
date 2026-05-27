part of 'physics_engine.dart';

/// Base class for collision shapes.
abstract class CollisionShape {
  /// Check collision and return manifold.
  CollisionManifold getManifold(Offset posA, CollisionShape other, Offset posB);

  /// Get the axis-aligned bounding box for this shape.
  Rect getBounds(Offset position);
}

/// A circular collision shape.
class CircleShape extends CollisionShape {
  final double radius;

  CircleShape(this.radius);

  @override
  CollisionManifold getManifold(
    Offset posA,
    CollisionShape other,
    Offset posB,
  ) {
    if (other is CircleShape) {
      final delta = posB - posA;
      final distance = delta.distance;
      final totalRadius = radius + other.radius;

      if (distance < totalRadius) {
        final penetration = totalRadius - distance;
        final normal = distance > 0 ? delta / distance : const Offset(1, 0);
        return CollisionManifold(
          isColliding: true,
          normal: normal,
          penetration: penetration,
        );
      }
    } else if (other is PolygonShape ||
        other is CapsuleShape ||
        other is SegmentShape) {
      // Delegate to the other shape and flip the normal so it points A→B.
      final m = other.getManifold(posB, this, posA);
      if (!m.isColliding) return CollisionManifold.empty();
      return CollisionManifold(
        isColliding: true,
        normal: -m.normal,
        penetration: m.penetration,
      );
    }
    return CollisionManifold.empty();
  }

  @override
  Rect getBounds(Offset position) {
    return Rect.fromCircle(center: position, radius: radius);
  }
}

/// A convex polygonal collision shape using SAT (Separating Axis Theorem).
class PolygonShape extends CollisionShape {
  /// Vertices defined relative to the center of the body.
  List<Offset> vertices;

  PolygonShape(this.vertices);

  @override
  CollisionManifold getManifold(
    Offset posA,
    CollisionShape other,
    Offset posB,
  ) {
    if (other is PolygonShape) {
      return _satPolygonVsPolygon(posA, this, posB, other);
    } else if (other is CircleShape) {
      // Invert the result so normal points A→B.
      final manifold = _satCircleVsPolygon(posB, other, posA, this);
      return CollisionManifold(
        isColliding: manifold.isColliding,
        normal: -manifold.normal,
        penetration: manifold.penetration,
      );
    } else if (other is CapsuleShape) {
      return _satPolygonVsCapsule(posA, this, posB, other);
    } else if (other is SegmentShape) {
      // Treat segment as a thin capsule.
      final cap = CapsuleShape(
        center1: other.point1,
        center2: other.point2,
        radius: other.thickness,
      );
      return _satPolygonVsCapsule(posA, this, posB, cap);
    }
    return CollisionManifold.empty();
  }

  @override
  Rect getBounds(Offset position) {
    if (vertices.isEmpty) return Rect.zero;
    double minX = double.infinity;
    double minY = double.infinity;
    double maxX = double.negativeInfinity;
    double maxY = double.negativeInfinity;

    for (final v in vertices) {
      final px = position.dx + v.dx;
      final py = position.dy + v.dy;
      if (px < minX) minX = px;
      if (py < minY) minY = py;
      if (px > maxX) maxX = px;
      if (py > maxY) maxY = py;
    }
    return Rect.fromLTRB(minX, minY, maxX, maxY);
  }

  CollisionManifold _satPolygonVsPolygon(
    Offset posA,
    PolygonShape polyA,
    Offset posB,
    PolygonShape polyB,
  ) {
    double minPenetration = double.infinity;
    double bestNx = 0, bestNy = 0;

    // Test axes from polyA
    for (int i = 0; i < polyA.vertices.length; i++) {
      final j = (i + 1) % polyA.vertices.length;
      final ex = polyA.vertices[j].dx - polyA.vertices[i].dx;
      final ey = polyA.vertices[j].dy - polyA.vertices[i].dy;
      // Perpendicular (left-hand normal): (-ey, ex)
      final len = math.sqrt(ex * ex + ey * ey);
      if (len == 0) continue;
      final nx = -ey / len;
      final ny = ex / len;

      final overlap = _overlapOnAxis(polyA, posA, polyB, posB, nx, ny);
      if (overlap == null) return CollisionManifold.empty();
      if (overlap < minPenetration) {
        minPenetration = overlap;
        bestNx = nx;
        bestNy = ny;
      }
    }

    // Test axes from polyB
    for (int i = 0; i < polyB.vertices.length; i++) {
      final j = (i + 1) % polyB.vertices.length;
      final ex = polyB.vertices[j].dx - polyB.vertices[i].dx;
      final ey = polyB.vertices[j].dy - polyB.vertices[i].dy;
      final len = math.sqrt(ex * ex + ey * ey);
      if (len == 0) continue;
      final nx = -ey / len;
      final ny = ex / len;

      final overlap = _overlapOnAxis(polyA, posA, polyB, posB, nx, ny);
      if (overlap == null) return CollisionManifold.empty();
      if (overlap < minPenetration) {
        minPenetration = overlap;
        bestNx = nx;
        bestNy = ny;
      }
    }

    // Ensure normal points from A to B
    final centerDx = posB.dx - posA.dx;
    final centerDy = posB.dy - posA.dy;
    if (bestNx * centerDx + bestNy * centerDy < 0) {
      bestNx = -bestNx;
      bestNy = -bestNy;
    }

    return CollisionManifold(
      isColliding: true,
      normal: Offset(bestNx, bestNy),
      penetration: minPenetration,
    );
  }

  CollisionManifold _satCircleVsPolygon(
    Offset center,
    CircleShape circle,
    Offset polyPos,
    PolygonShape poly,
  ) {
    if (poly.vertices.isEmpty) return CollisionManifold.empty();
    double minPenetration = double.infinity;
    double bestNx = 0, bestNy = 0;

    // Find the polygon vertex closest to the circle center (world space).
    double closestX = poly.vertices[0].dx + polyPos.dx;
    double closestY = poly.vertices[0].dy + polyPos.dy;
    double minDistSq = (closestX - center.dx) * (closestX - center.dx) +
        (closestY - center.dy) * (closestY - center.dy);
    for (int i = 1; i < poly.vertices.length; i++) {
      final vx = poly.vertices[i].dx + polyPos.dx;
      final vy = poly.vertices[i].dy + polyPos.dy;
      final dSq = (vx - center.dx) * (vx - center.dx) +
          (vy - center.dy) * (vy - center.dy);
      if (dSq < minDistSq) {
        minDistSq = dSq;
        closestX = vx;
        closestY = vy;
      }
    }

    // Axis from closest vertex to circle center.
    if (minDistSq > 0) {
      final axLen = math.sqrt(minDistSq);
      final nx = (center.dx - closestX) / axLen;
      final ny = (center.dy - closestY) / axLen;
      final overlap =
          _overlapOnAxisCircle(poly, polyPos, center, circle.radius, nx, ny);
      if (overlap == null) return CollisionManifold.empty();
      minPenetration = overlap;
      bestNx = nx;
      bestNy = ny;
    }

    // Test polygon edge axes.
    for (int i = 0; i < poly.vertices.length; i++) {
      final j = (i + 1) % poly.vertices.length;
      final ex = poly.vertices[j].dx - poly.vertices[i].dx;
      final ey = poly.vertices[j].dy - poly.vertices[i].dy;
      final len = math.sqrt(ex * ex + ey * ey);
      if (len == 0) continue;
      final nx = -ey / len;
      final ny = ex / len;

      final overlap =
          _overlapOnAxisCircle(poly, polyPos, center, circle.radius, nx, ny);
      if (overlap == null) return CollisionManifold.empty();

      if (overlap < minPenetration) {
        minPenetration = overlap;
        bestNx = nx;
        bestNy = ny;
      }
    }

    final centerDx = polyPos.dx - center.dx;
    final centerDy = polyPos.dy - center.dy;
    if (bestNx * centerDx + bestNy * centerDy < 0) {
      bestNx = -bestNx;
      bestNy = -bestNy;
    }

    return CollisionManifold(
      isColliding: true,
      normal: Offset(bestNx, bestNy),
      penetration: minPenetration,
    );
  }

  /// SAT test between this polygon and a capsule.
  ///
  /// Tests polygon edge normals (with capsule radius expansion) and the axis
  /// from each polygon vertex to the closest point on the capsule axis.
  CollisionManifold _satPolygonVsCapsule(
    Offset posA,
    PolygonShape poly,
    Offset posB,
    CapsuleShape cap,
  ) {
    final p1 = Offset(posB.dx + cap.center1.dx, posB.dy + cap.center1.dy);
    final p2 = Offset(posB.dx + cap.center2.dx, posB.dy + cap.center2.dy);

    // Pre-compute world-space positions once; avoids repeated offset addition
    // inside each per-axis projection loop, reducing constant factor.
    final worldVerts = List<Offset>.generate(
      poly.vertices.length,
      (i) => Offset(poly.vertices[i].dx + posA.dx, poly.vertices[i].dy + posA.dy),
    );

    double minPen = double.infinity;
    double bestNx = 0, bestNy = 0;

    // Test polygon edge normals (capsule projection expanded by radius).
    for (int i = 0; i < worldVerts.length; i++) {
      final j = (i + 1) % worldVerts.length;
      final ex = worldVerts[j].dx - worldVerts[i].dx;
      final ey = worldVerts[j].dy - worldVerts[i].dy;
      final len = math.sqrt(ex * ex + ey * ey);
      if (len < 1e-8) continue;
      final nx = -ey / len;
      final ny = ex / len;

      double minA = double.infinity, maxA = double.negativeInfinity;
      for (final w in worldVerts) {
        final p = w.dx * nx + w.dy * ny;
        if (p < minA) minA = p;
        if (p > maxA) maxA = p;
      }

      final proj1 = p1.dx * nx + p1.dy * ny;
      final proj2 = p2.dx * nx + p2.dy * ny;
      final minB = math.min(proj1, proj2) - cap.radius;
      final maxB = math.max(proj1, proj2) + cap.radius;

      if (minA > maxB || minB > maxA) return CollisionManifold.empty();
      final overlap = math.min(maxA - minB, maxB - minA);
      if (overlap < minPen) {
        minPen = overlap;
        bestNx = nx;
        bestNy = ny;
      }
    }

    // Test axis from each polygon vertex to the closest point on capsule axis.
    for (final w in worldVerts) {
      final closest = CapsuleShape.closestPointOnSegment(w, p1, p2);
      final dx = w.dx - closest.dx;
      final dy = w.dy - closest.dy;
      final len = math.sqrt(dx * dx + dy * dy);
      if (len < 1e-8) continue;
      final nx = dx / len;
      final ny = dy / len;

      double minA = double.infinity, maxA = double.negativeInfinity;
      for (final w2 in worldVerts) {
        final p = w2.dx * nx + w2.dy * ny;
        if (p < minA) minA = p;
        if (p > maxA) maxA = p;
      }

      final proj1 = p1.dx * nx + p1.dy * ny;
      final proj2 = p2.dx * nx + p2.dy * ny;
      final minB = math.min(proj1, proj2) - cap.radius;
      final maxB = math.max(proj1, proj2) + cap.radius;

      if (minA > maxB || minB > maxA) return CollisionManifold.empty();
      final overlap = math.min(maxA - minB, maxB - minA);
      if (overlap < minPen) {
        minPen = overlap;
        bestNx = nx;
        bestNy = ny;
      }
    }

    // Ensure normal points from polygon (A) to capsule (B).
    final cx = posB.dx - posA.dx;
    final cy = posB.dy - posA.dy;
    if (bestNx * cx + bestNy * cy < 0) {
      bestNx = -bestNx;
      bestNy = -bestNy;
    }

    return CollisionManifold(
      isColliding: true,
      normal: Offset(bestNx, bestNy),
      penetration: minPen,
    );
  }

  /// Projects both polygons onto the axis (nx, ny) and returns the overlap,
  /// or null if the projections are separated (no collision on this axis).
  /// All arithmetic is inline — zero heap allocation.
  double? _overlapOnAxis(
    PolygonShape polyA,
    Offset posA,
    PolygonShape polyB,
    Offset posB,
    double nx,
    double ny,
  ) {
    double minA = double.infinity, maxA = double.negativeInfinity;
    for (final v in polyA.vertices) {
      final p = (v.dx + posA.dx) * nx + (v.dy + posA.dy) * ny;
      if (p < minA) minA = p;
      if (p > maxA) maxA = p;
    }

    double minB = double.infinity, maxB = double.negativeInfinity;
    for (final v in polyB.vertices) {
      final p = (v.dx + posB.dx) * nx + (v.dy + posB.dy) * ny;
      if (p < minB) minB = p;
      if (p > maxB) maxB = p;
    }

    if (minA > maxB || minB > maxA) return null;
    return math.min(maxA - minB, maxB - minA);
  }

  /// Projects the polygon and a circle onto the axis (nx, ny) and returns
  /// the overlap, or null if separated. Zero heap allocation.
  double? _overlapOnAxisCircle(
    PolygonShape poly,
    Offset polyPos,
    Offset circleCenter,
    double radius,
    double nx,
    double ny,
  ) {
    double minP = double.infinity, maxP = double.negativeInfinity;
    for (final v in poly.vertices) {
      final p = (v.dx + polyPos.dx) * nx + (v.dy + polyPos.dy) * ny;
      if (p < minP) minP = p;
      if (p > maxP) maxP = p;
    }

    final centerProj = circleCenter.dx * nx + circleCenter.dy * ny;
    final minC = centerProj - radius;
    final maxC = centerProj + radius;

    if (minP > maxC || minC > maxP) return null;
    return math.min(maxP - minC, maxC - minP);
  }
}

/// A rectangular collision shape (simplified Polygon).
class RectangleShape extends PolygonShape {
  final double width;
  final double height;

  RectangleShape(this.width, this.height)
    : super([
        Offset(-width / 2, -height / 2),
        Offset(width / 2, -height / 2),
        Offset(width / 2, height / 2),
        Offset(-width / 2, height / 2),
      ]);
}

// ── New shapes ────────────────────────────────────────────────────────────────

/// A capsule collision shape — the Minkowski sum of a line segment and a circle.
///
/// Defined by two center offsets [center1] and [center2] relative to the
/// body's position, and a [radius]. Suitable for characters, pills, bullets.
class CapsuleShape extends CollisionShape {
  final Offset center1;
  final Offset center2;
  final double radius;

  CapsuleShape({
    required this.center1,
    required this.center2,
    required this.radius,
  });

  /// Convenience constructor for a vertical capsule of given [height] and [radius].
  factory CapsuleShape.vertical({
    required double height,
    required double radius,
  }) {
    final half = height / 2 - radius;
    return CapsuleShape(
      center1: Offset(0, -half),
      center2: Offset(0, half),
      radius: radius,
    );
  }

  @override
  Rect getBounds(Offset position) {
    final wx1 = position.dx + center1.dx;
    final wy1 = position.dy + center1.dy;
    final wx2 = position.dx + center2.dx;
    final wy2 = position.dy + center2.dy;
    return Rect.fromLTRB(
      math.min(wx1, wx2) - radius,
      math.min(wy1, wy2) - radius,
      math.max(wx1, wx2) + radius,
      math.max(wy1, wy2) + radius,
    );
  }

  @override
  CollisionManifold getManifold(
    Offset posA,
    CollisionShape other,
    Offset posB,
  ) {
    if (other is CircleShape) {
      return _capsuleVsCircle(posA, other, posB);
    } else if (other is CapsuleShape) {
      return _capsuleVsCapsule(posA, other, posB);
    } else if (other is PolygonShape) {
      // Delegate to the polygon side and flip the normal.
      final m = other._satPolygonVsCapsule(posB, other, posA, this);
      if (!m.isColliding) return CollisionManifold.empty();
      return CollisionManifold(
        isColliding: true,
        normal: -m.normal,
        penetration: m.penetration,
      );
    }
    return CollisionManifold.empty();
  }

  CollisionManifold _capsuleVsCircle(
    Offset posA,
    CircleShape circle,
    Offset posB,
  ) {
    final wa1 = Offset(posA.dx + center1.dx, posA.dy + center1.dy);
    final wa2 = Offset(posA.dx + center2.dx, posA.dy + center2.dy);

    final closest = closestPointOnSegment(posB, wa1, wa2);
    final dx = posB.dx - closest.dx;
    final dy = posB.dy - closest.dy;
    final distSq = dx * dx + dy * dy;
    final sumR = radius + circle.radius;

    if (distSq >= sumR * sumR) return CollisionManifold.empty();

    final dist = math.sqrt(distSq);
    final nx = dist > 1e-8 ? dx / dist : 0.0;
    final ny = dist > 1e-8 ? dy / dist : 1.0;

    return CollisionManifold(
      isColliding: true,
      normal: Offset(nx, ny), // points from capsule axis toward circle (A→B)
      penetration: sumR - dist,
    );
  }

  CollisionManifold _capsuleVsCapsule(
    Offset posA,
    CapsuleShape other,
    Offset posB,
  ) {
    final wa1 = Offset(posA.dx + center1.dx, posA.dy + center1.dy);
    final wa2 = Offset(posA.dx + center2.dx, posA.dy + center2.dy);
    final wb1 = Offset(posB.dx + other.center1.dx, posB.dy + other.center1.dy);
    final wb2 = Offset(posB.dx + other.center2.dx, posB.dy + other.center2.dy);

    final (ptA, ptB) = _closestPointsOnSegments(wa1, wa2, wb1, wb2);

    final dx = ptB.dx - ptA.dx;
    final dy = ptB.dy - ptA.dy;
    final distSq = dx * dx + dy * dy;
    final sumR = radius + other.radius;

    if (distSq >= sumR * sumR) return CollisionManifold.empty();

    final dist = math.sqrt(distSq);
    final nx = dist > 1e-8 ? dx / dist : 0.0;
    final ny = dist > 1e-8 ? dy / dist : 1.0;

    return CollisionManifold(
      isColliding: true,
      normal: Offset(nx, ny),
      penetration: sumR - dist,
    );
  }

  /// Returns the closest point on segment [a]→[b] to point [p].
  static Offset closestPointOnSegment(Offset p, Offset a, Offset b) {
    final abx = b.dx - a.dx;
    final aby = b.dy - a.dy;
    final lenSq = abx * abx + aby * aby;
    if (lenSq < 1e-12) return a;
    final t = ((p.dx - a.dx) * abx + (p.dy - a.dy) * aby) / lenSq;
    final tc = t.clamp(0.0, 1.0);
    return Offset(a.dx + tc * abx, a.dy + tc * aby);
  }

  /// Returns the closest points on segments [a1]→[a2] and [b1]→[b2].
  ///
  /// Based on the GDC shortest-distance-between-two-segments algorithm.
  static (Offset, Offset) _closestPointsOnSegments(
    Offset a1,
    Offset a2,
    Offset b1,
    Offset b2,
  ) {
    final dax = a2.dx - a1.dx;
    final day = a2.dy - a1.dy;
    final dbx = b2.dx - b1.dx;
    final dby = b2.dy - b1.dy;
    final dx = a1.dx - b1.dx;
    final dy = a1.dy - b1.dy;

    final a = dax * dax + day * day; // |da|²
    final e = dbx * dbx + dby * dby; // |db|²
    final f = dbx * dx + dby * dy;

    double s, t;

    if (a < 1e-10 && e < 1e-10) {
      return (a1, b1);
    }
    if (a < 1e-10) {
      s = 0.0;
      t = (f / e).clamp(0.0, 1.0);
    } else {
      final c = dax * dx + day * dy;
      if (e < 1e-10) {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
      } else {
        final b = dax * dbx + day * dby;
        final denom = a * e - b * b;
        if (denom.abs() > 1e-10) {
          s = ((b * f - c * e) / denom).clamp(0.0, 1.0);
        } else {
          s = 0.0;
        }
        t = (b * s + f) / e;
        if (t < 0.0) {
          t = 0.0;
          s = (-c / a).clamp(0.0, 1.0);
        } else if (t > 1.0) {
          t = 1.0;
          s = ((b - c) / a).clamp(0.0, 1.0);
        }
      }
    }

    return (
      Offset(a1.dx + s * dax, a1.dy + s * day),
      Offset(b1.dx + t * dbx, b1.dy + t * dby),
    );
  }
}

/// A line-segment collision shape for static terrain and one-way platforms.
///
/// The [thickness] gives the segment a small radius so that fast-moving bodies
/// don't tunnel through it. For static geometry only — dynamic segment bodies
/// are not physically meaningful.
class SegmentShape extends CollisionShape {
  final Offset point1;
  final Offset point2;
  final double thickness;

  SegmentShape(this.point1, this.point2, {this.thickness = 2.0});

  @override
  Rect getBounds(Offset position) {
    final wx1 = position.dx + point1.dx;
    final wy1 = position.dy + point1.dy;
    final wx2 = position.dx + point2.dx;
    final wy2 = position.dy + point2.dy;
    return Rect.fromLTRB(
      math.min(wx1, wx2) - thickness,
      math.min(wy1, wy2) - thickness,
      math.max(wx1, wx2) + thickness,
      math.max(wy1, wy2) + thickness,
    );
  }

  @override
  CollisionManifold getManifold(
    Offset posA,
    CollisionShape other,
    Offset posB,
  ) {
    // Treat the segment as a zero-mass thin capsule for all collision types.
    final asCapsule = CapsuleShape(
      center1: point1,
      center2: point2,
      radius: thickness,
    );
    return asCapsule.getManifold(posA, other, posB);
  }
}

/// A chain of connected line segments for complex terrain and loop-the-loops.
///
/// When [loop] is true, the last vertex connects back to the first.
/// Each segment uses [thickness] as a radius (like [SegmentShape]).
class ChainShape extends CollisionShape {
  final List<Offset> vertices;
  final bool loop;
  final double thickness;

  ChainShape(this.vertices, {this.loop = false, this.thickness = 2.0});

  @override
  Rect getBounds(Offset position) {
    if (vertices.isEmpty) return Rect.zero;
    double minX = double.infinity, minY = double.infinity;
    double maxX = double.negativeInfinity, maxY = double.negativeInfinity;
    for (final v in vertices) {
      final wx = position.dx + v.dx;
      final wy = position.dy + v.dy;
      if (wx - thickness < minX) minX = wx - thickness;
      if (wy - thickness < minY) minY = wy - thickness;
      if (wx + thickness > maxX) maxX = wx + thickness;
      if (wy + thickness > maxY) maxY = wy + thickness;
    }
    return Rect.fromLTRB(minX, minY, maxX, maxY);
  }

  @override
  CollisionManifold getManifold(
    Offset posA,
    CollisionShape other,
    Offset posB,
  ) {
    if (vertices.length < 2) return CollisionManifold.empty();

    // Return the manifold from the deepest penetrating segment.
    CollisionManifold? best;
    final n = vertices.length;
    final segCount = loop ? n : n - 1;

    for (int i = 0; i < segCount; i++) {
      final p1 = vertices[i];
      final p2 = vertices[(i + 1) % n];
      final seg = SegmentShape(p1, p2, thickness: thickness);
      final m = seg.getManifold(posA, other, posB);
      if (m.isColliding) {
        if (best == null || m.penetration > best.penetration) {
          best = m;
        }
      }
    }

    return best ?? CollisionManifold.empty();
  }
}

/// A convex polygon with rounded corners.
///
/// Behaves like [PolygonShape] but the shape is inflated outward by
/// [cornerRadius], producing smooth rounded edges. Matches Box2D's
/// `b2MakePolygon(&hull, radius)` semantics.
class RoundedPolygonShape extends PolygonShape {
  final double cornerRadius;

  RoundedPolygonShape(super.vertices, this.cornerRadius);

  /// Convenience constructor for a rounded rectangle.
  factory RoundedPolygonShape.rect({
    required double width,
    required double height,
    required double cornerRadius,
  }) {
    return RoundedPolygonShape(
      [
        Offset(-width / 2, -height / 2),
        Offset(width / 2, -height / 2),
        Offset(width / 2, height / 2),
        Offset(-width / 2, height / 2),
      ],
      cornerRadius,
    );
  }

  @override
  Rect getBounds(Offset position) {
    return super.getBounds(position).inflate(cornerRadius);
  }

  @override
  CollisionManifold getManifold(
    Offset posA,
    CollisionShape other,
    Offset posB,
  ) {
    if (other is CircleShape) {
      return _roundedPolyVsCircle(posA, other, posB);
    } else if (other is PolygonShape) {
      return _roundedPolyVsPolygon(posA, other, posB);
    } else if (other is CapsuleShape) {
      return _satPolygonVsCapsule(posA, this, posB, other);
    } else if (other is SegmentShape) {
      final cap = CapsuleShape(
        center1: other.point1,
        center2: other.point2,
        radius: other.thickness,
      );
      return _satPolygonVsCapsule(posA, this, posB, cap);
    }
    return CollisionManifold.empty();
  }

  CollisionManifold _roundedPolyVsCircle(
    Offset posA,
    CircleShape circle,
    Offset posB,
  ) {
    // Inflate the SAT test by treating the circle radius as (circle.radius + cornerRadius).
    final inflatedRadius = circle.radius + cornerRadius;
    final m = _satCircleVsPolygon(posB, CircleShape(inflatedRadius), posA, this);
    if (!m.isColliding) return CollisionManifold.empty();
    // The normal from _satCircleVsPolygon points circle→polygon; flip for A→B.
    return CollisionManifold(
      isColliding: true,
      normal: -m.normal,
      penetration: m.penetration,
    );
  }

  CollisionManifold _roundedPolyVsPolygon(
    Offset posA,
    PolygonShape other,
    Offset posB,
  ) {
    // SAT where the rounded polygon's axis projections are inflated by cornerRadius.
    final r = cornerRadius;
    double minPen = double.infinity;
    double bestNx = 0, bestNy = 0;
    bool separated = false;

    // Pre-compute world-space positions once to avoid repeated addition per axis.
    final worldA = List<Offset>.generate(
      vertices.length,
      (i) => Offset(vertices[i].dx + posA.dx, vertices[i].dy + posA.dy),
    );
    final worldB = List<Offset>.generate(
      other.vertices.length,
      (i) => Offset(other.vertices[i].dx + posB.dx, other.vertices[i].dy + posB.dy),
    );

    void testAxis(double nx, double ny) {
      if (separated) return;
      // Inflate rounded polygon projection by r on both sides.
      double minA = double.infinity, maxA = double.negativeInfinity;
      for (final w in worldA) {
        final p = w.dx * nx + w.dy * ny;
        if (p < minA) minA = p;
        if (p > maxA) maxA = p;
      }
      minA -= r;
      maxA += r;

      double minB = double.infinity, maxB = double.negativeInfinity;
      for (final w in worldB) {
        final p = w.dx * nx + w.dy * ny;
        if (p < minB) minB = p;
        if (p > maxB) maxB = p;
      }

      if (minA > maxB || minB > maxA) {
        separated = true;
        return;
      }
      final overlap = math.min(maxA - minB, maxB - minA);
      if (overlap < minPen) {
        minPen = overlap;
        bestNx = nx;
        bestNy = ny;
      }
    }

    for (int i = 0; i < worldA.length; i++) {
      final j = (i + 1) % worldA.length;
      final ex = worldA[j].dx - worldA[i].dx;
      final ey = worldA[j].dy - worldA[i].dy;
      final len = math.sqrt(ex * ex + ey * ey);
      if (len < 1e-8) continue;
      testAxis(-ey / len, ex / len);
    }

    for (int i = 0; i < worldB.length; i++) {
      final j = (i + 1) % worldB.length;
      final ex = worldB[j].dx - worldB[i].dx;
      final ey = worldB[j].dy - worldB[i].dy;
      final len = math.sqrt(ex * ex + ey * ey);
      if (len < 1e-8) continue;
      testAxis(-ey / len, ex / len);
    }

    if (separated) return CollisionManifold.empty();

    final cx = posB.dx - posA.dx;
    final cy = posB.dy - posA.dy;
    if (bestNx * cx + bestNy * cy < 0) {
      bestNx = -bestNx;
      bestNy = -bestNy;
    }

    return CollisionManifold(
      isColliding: true,
      normal: Offset(bestNx, bestNy),
      penetration: minPen,
    );
  }
}
