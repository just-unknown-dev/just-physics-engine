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
      // Invert the result so normal points A->B
      final manifold = _satCircleVsPolygon(posB, other, posA, this);
      return CollisionManifold(
        isColliding: manifold.isColliding,
        normal: -manifold.normal,
        penetration: manifold.penetration,
      );
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
      final overlap = _overlapOnAxisCircle(poly, polyPos, center, circle.radius, nx, ny);
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

      final overlap = _overlapOnAxisCircle(poly, polyPos, center, circle.radius, nx, ny);
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
