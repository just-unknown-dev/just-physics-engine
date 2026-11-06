part of 'physics_2d_engine.dart';

// The pure-Dart engine's ray tests: a ray against each shape of a body,
// turned with it, as Box2D turns shapes for its own ray casts.

/// [ray]'s nearest hit on any of [body]'s shapes, turned with it. A shape
/// the ray starts inside is passed over, as on the native backend.
Ray2DHit? _rayHitBody(Ray2D ray, Physics2DBody body) {
  final pos = body.position.toOffset();
  Ray2DHit? best;
  final extra = body.additionalShapes;
  for (var i = -1; i < extra.length; i++) {
    final shape = _turnedShape(i < 0 ? body.shape : extra[i], body.angle);
    // AABB quick-reject using the slab method.
    if (!_rayIntersectsAABB(ray, shape.boundsAt(pos))) continue;
    final hit = _rayHitShape(ray, pos, shape, body);
    if (hit != null && (best == null || hit.distance < best.distance)) {
      best = hit;
    }
  }
  return best;
}

Ray2DHit? _rayHitShape(
  Ray2D ray,
  Offset pos,
  Shape2D shape,
  Physics2DBody body,
) {
  if (shape is Capsule2DShape) {
    return _rayHitCapsule(ray, pos, shape, body);
  } else if (shape is Chain2DShape) {
    return _rayHitChain(ray, pos, shape, body);
  } else if (shape is Segment2DShape) {
    return _rayHitSegmentShape(ray, pos, shape, body);
  } else if (shape is Circle2DShape) {
    return _rayHitCircle(ray, shape.centerAt(pos), shape.radius, body);
  } else if (shape is RoundedPolygon2DShape) {
    return _rayHitRoundedPolygon(ray, pos, shape, body);
  } else if (shape is Polygon2DShape) {
    return _rayHitPolygonVerts(ray, shape.vertices, pos, body);
  }
  return null;
}

/// [shape] turned by [angle] about its body's position.
Shape2D _turnedShape(Shape2D shape, double angle) {
  if (angle == 0) return shape;
  final c = math.cos(angle), s = math.sin(angle);
  Offset turn(Offset p) => Offset(c * p.dx - s * p.dy, s * p.dx + c * p.dy);
  return switch (shape) {
    Circle2DShape() when shape.center == Offset.zero => shape,
    Circle2DShape() => Circle2DShape(shape.radius, center: turn(shape.center)),
    Capsule2DShape() => Capsule2DShape(
      center1: turn(shape.center1),
      center2: turn(shape.center2),
      radius: shape.radius,
    ),
    RoundedPolygon2DShape() => RoundedPolygon2DShape([
      for (final v in shape.vertices) turn(v),
    ], shape.cornerRadius),
    Polygon2DShape() => Polygon2DShape([
      for (final v in shape.vertices) turn(v),
    ]),
    Segment2DShape() => Segment2DShape(
      turn(shape.point1),
      turn(shape.point2),
      thickness: shape.thickness,
    ),
    Chain2DShape() => Chain2DShape(
      [for (final v in shape.vertices) turn(v)],
      loop: shape.loop,
      thickness: shape.thickness,
    ),
    _ => shape,
  };
}

/// Whether [p] is inside the convex polygon [verts] placed at [pos].
bool _containsConvex(List<Offset> verts, Offset pos, Offset p) {
  if (verts.length < 3) return false;
  var sign = 0;
  for (var i = 0; i < verts.length; i++) {
    final a = verts[i] + pos, b = verts[(i + 1) % verts.length] + pos;
    final cross = (b.dx - a.dx) * (p.dy - a.dy) - (b.dy - a.dy) * (p.dx - a.dx);
    if (cross == 0) continue;
    final s = cross > 0 ? 1 : -1;
    if (sign == 0) {
      sign = s;
    } else if (s != sign) {
      return false;
    }
  }
  return sign != 0;
}

/// [ray] against a polygon grown by its corner radius: each side moved out
/// by the radius, and a circle at each corner.
Ray2DHit? _rayHitRoundedPolygon(
  Ray2D ray,
  Offset pos,
  RoundedPolygon2DShape shape,
  Physics2DBody body,
) {
  final r = shape.cornerRadius;
  final verts = [for (final v in shape.vertices) v + pos];
  if (r <= 0 || verts.length < 2) {
    return _rayHitPolygonVerts(ray, shape.vertices, pos, body);
  }
  // Starting inside: passed over.
  if (_containsConvex(shape.vertices, pos, ray.origin)) return null;
  final n = verts.length;
  for (var i = 0; i < n; i++) {
    final c = Capsule2DShape.closestPointOnSegment(
      ray.origin,
      verts[i],
      verts[(i + 1) % n],
    );
    if ((ray.origin - c).distanceSquared < r * r) return null;
  }

  var centroid = Offset.zero;
  for (final v in verts) {
    centroid += v;
  }
  centroid /= n.toDouble();

  Ray2DHit? best;
  for (var i = 0; i < n; i++) {
    final a = verts[i], b = verts[(i + 1) % n];
    final e = b - a;
    final len = e.distance;
    if (len < 1e-8) continue;
    var normal = Offset(-e.dy / len, e.dx / len);
    if (normal.dx * (a.dx - centroid.dx) + normal.dy * (a.dy - centroid.dy) <
        0) {
      normal = -normal;
    }
    // Only a side the ray comes at from outside.
    final facing = normal.dx * ray.direction.dx + normal.dy * ray.direction.dy;
    if (facing >= 0) continue;
    final a2 = a + normal * r; // the side, moved out
    final denom = ray.direction.dx * e.dy - ray.direction.dy * e.dx;
    if (denom.abs() < 1e-10) continue;
    final dpx = a2.dx - ray.origin.dx, dpy = a2.dy - ray.origin.dy;
    final t = (dpx * e.dy - dpy * e.dx) / denom;
    final s = (dpx * ray.direction.dy - dpy * ray.direction.dx) / denom;
    if (t < 0 || t > ray.maxDistance || s < 0 || s > 1) continue;
    if (best != null && t >= best.distance) continue;
    best = Ray2DHit(body: body, point: ray.at(t), normal: normal, distance: t);
  }
  for (final v in verts) {
    final h = _rayHitCircle(ray, v, r, body);
    if (h != null && (best == null || h.distance < best.distance)) best = h;
  }
  return best;
}

bool _rayIntersectsAABB(Ray2D ray, Rect rect) {
  double tMin = 0, tMax = ray.maxDistance;

  if (ray.direction.dx.abs() < 1e-10) {
    if (ray.origin.dx < rect.left || ray.origin.dx > rect.right) return false;
  } else {
    final inv = 1.0 / ray.direction.dx;
    var t1 = (rect.left - ray.origin.dx) * inv;
    var t2 = (rect.right - ray.origin.dx) * inv;
    if (t1 > t2) {
      final tmp = t1;
      t1 = t2;
      t2 = tmp;
    }
    tMin = math.max(tMin, t1);
    tMax = math.min(tMax, t2);
    if (tMin > tMax) return false;
  }

  if (ray.direction.dy.abs() < 1e-10) {
    if (ray.origin.dy < rect.top || ray.origin.dy > rect.bottom) return false;
  } else {
    final inv = 1.0 / ray.direction.dy;
    var t1 = (rect.top - ray.origin.dy) * inv;
    var t2 = (rect.bottom - ray.origin.dy) * inv;
    if (t1 > t2) {
      final tmp = t1;
      t1 = t2;
      t2 = tmp;
    }
    tMin = math.max(tMin, t1);
    tMax = math.min(tMax, t2);
    if (tMin > tMax) return false;
  }

  return true;
}

Ray2DHit? _rayHitCircle(
  Ray2D ray,
  Offset center,
  double radius,
  Physics2DBody body,
) {
  final ocx = ray.origin.dx - center.dx;
  final ocy = ray.origin.dy - center.dy;
  final b = ocx * ray.direction.dx + ocy * ray.direction.dy;
  final c = ocx * ocx + ocy * ocy - radius * radius;
  // Starting inside: passed over.
  if (c < 0) return null;
  final disc = b * b - c;
  if (disc < 0) return null;
  final sqrtD = math.sqrt(disc);
  double t = -b - sqrtD;
  if (t < 0) t = -b + sqrtD;
  if (t < 0 || t > ray.maxDistance) return null;
  final point = ray.at(t);
  final dx = point.dx - center.dx;
  final dy = point.dy - center.dy;
  final len = math.sqrt(dx * dx + dy * dy);
  return Ray2DHit(
    body: body,
    point: point,
    normal: len > 1e-8 ? Offset(dx / len, dy / len) : const Offset(0, -1),
    distance: t,
  );
}

Ray2DHit? _rayHitPolygonVerts(
  Ray2D ray,
  List<Offset> verts,
  Offset pos,
  Physics2DBody body,
) {
  // Starting inside: passed over.
  if (_containsConvex(verts, pos, ray.origin)) return null;
  double closestT = ray.maxDistance;
  Offset? closestNormal;

  for (int i = 0; i < verts.length; i++) {
    final j = (i + 1) % verts.length;
    final ax = verts[i].dx + pos.dx;
    final ay = verts[i].dy + pos.dy;
    final bx = verts[j].dx + pos.dx;
    final by = verts[j].dy + pos.dy;

    final edgeDx = bx - ax;
    final edgeDy = by - ay;
    final denom = ray.direction.dx * edgeDy - ray.direction.dy * edgeDx;
    if (denom.abs() < 1e-10) continue;

    final dpx = ax - ray.origin.dx;
    final dpy = ay - ray.origin.dy;
    final t = (dpx * edgeDy - dpy * edgeDx) / denom;
    final s = (dpx * ray.direction.dy - dpy * ray.direction.dx) / denom;

    if (t < 0 || t > closestT || s < 0 || s > 1) continue;

    final len = math.sqrt(edgeDx * edgeDx + edgeDy * edgeDy);
    if (len < 1e-8) continue;
    var nx = -edgeDy / len;
    var ny = edgeDx / len;
    // Ensure normal faces toward ray origin (opposite to ray direction).
    if (nx * ray.direction.dx + ny * ray.direction.dy > 0) {
      nx = -nx;
      ny = -ny;
    }
    closestT = t;
    closestNormal = Offset(nx, ny);
  }

  if (closestNormal == null) return null;
  return Ray2DHit(
    body: body,
    point: ray.at(closestT),
    normal: closestNormal,
    distance: closestT,
  );
}

Ray2DHit? _rayHitCapsule(
  Ray2D ray,
  Offset pos,
  Capsule2DShape cap,
  Physics2DBody body,
) {
  final p1 = Offset(pos.dx + cap.center1.dx, pos.dy + cap.center1.dy);
  final p2 = Offset(pos.dx + cap.center2.dx, pos.dy + cap.center2.dy);
  final r = cap.radius;

  // Starting inside: passed over.
  final inner = Capsule2DShape.closestPointOnSegment(ray.origin, p1, p2);
  if ((ray.origin - inner).distanceSquared < r * r) return null;

  Ray2DHit? best;

  // Endpoint circles (caps).
  for (final center in [p1, p2]) {
    final h = _rayHitCircle(ray, center, r, body);
    if (h != null && (best == null || h.distance < best.distance)) best = h;
  }

  // Lateral surface of the capsule body.
  final ex = p2.dx - p1.dx;
  final ey = p2.dy - p1.dy;
  final segLen = math.sqrt(ex * ex + ey * ey);
  if (segLen < 1e-8) return best;

  final axX = ex / segLen;
  final axY = ey / segLen;
  // Perpendicular to axis (outward normals of the lateral surface).
  final perX = -axY;
  final perY = axX;

  final ox = ray.origin.dx - p1.dx;
  final oy = ray.origin.dy - p1.dy;
  final oPer = ox * perX + oy * perY;
  final dPer = ray.direction.dx * perX + ray.direction.dy * perY;

  if (dPer.abs() > 1e-10) {
    for (final sign in [1.0, -1.0]) {
      final t = (sign * r - oPer) / dPer;
      if (t < 0 || t > ray.maxDistance) continue;
      if (best != null && t >= best.distance) continue;
      // Check that the hit point is within the segment span.
      final hx = ox + t * ray.direction.dx;
      final hy = oy + t * ray.direction.dy;
      final sAlong = hx * axX + hy * axY;
      if (sAlong < 0 || sAlong > segLen) continue;
      var nx = sign * perX;
      var ny = sign * perY;
      if (nx * ray.direction.dx + ny * ray.direction.dy > 0) {
        nx = -nx;
        ny = -ny;
      }
      best = Ray2DHit(
        body: body,
        point: ray.at(t),
        normal: Offset(nx, ny),
        distance: t,
      );
    }
  }

  return best;
}

Ray2DHit? _rayHitSegmentShape(
  Ray2D ray,
  Offset pos,
  Segment2DShape seg,
  Physics2DBody body,
) {
  return _rayHitCapsule(
    ray,
    pos,
    Capsule2DShape(
      center1: seg.point1,
      center2: seg.point2,
      radius: seg.thickness,
    ),
    body,
  );
}

Ray2DHit? _rayHitChain(
  Ray2D ray,
  Offset pos,
  Chain2DShape chain,
  Physics2DBody body,
) {
  if (chain.vertices.length < 2) return null;
  Ray2DHit? best;
  final n = chain.vertices.length;
  final segCount = chain.loop ? n : n - 1;
  for (int i = 0; i < segCount; i++) {
    final seg = Segment2DShape(
      chain.vertices[i],
      chain.vertices[(i + 1) % n],
      thickness: chain.thickness,
    );
    final h = _rayHitSegmentShape(ray, pos, seg, body);
    if (h != null && (best == null || h.distance < best.distance)) best = h;
  }
  return best;
}

/// The result of a circle shape cast against a [Physics2DBody].
class CircleCast2DHit {
  /// The body that was hit.
  final Physics2DBody body;

  /// World-space contact point.
  final Offset point;

  /// Surface normal at the contact point.
  final Offset normal;

  /// Time of impact (0–1, where 1 means the full motion was completed).
  final double toi;

  /// Distance from the sweep origin to the contact point.
  final double distance;

  /// A hit on [body] at [point], facing [normal], [toi] of the way along
  /// the sweep and [distance] from its start.
  const CircleCast2DHit({
    required this.body,
    required this.point,
    required this.normal,
    required this.toi,
    required this.distance,
  });
}

/// The result of a ray cast against a [Physics2DBody].
class Ray2DHit {
  /// The body that was intersected.
  final Physics2DBody body;

  /// World-space hit point.
  final Offset point;

  /// Surface normal at the hit point, pointing toward the ray origin.
  final Offset normal;

  /// Distance from the ray origin to [point] (world units).
  final double distance;

  /// A hit on [body] at [point], facing [normal], [distance] along the ray.
  const Ray2DHit({
    required this.body,
    required this.point,
    required this.normal,
    required this.distance,
  });
}
