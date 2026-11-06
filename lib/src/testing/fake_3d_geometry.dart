part of 'fake_physics_3d_engine.dart';

/// [shape]'s bounds in its own space.
(Vector3, Vector3) _boundsOf(Shape3D shape) {
  switch (shape) {
    case Box3DShape(:final hx, :final hy, :final hz):
      return (Vector3(-hx, -hy, -hz), Vector3(hx, hy, hz));
    case Sphere3DShape(:final radius):
      return (
        Vector3(-radius, -radius, -radius),
        Vector3(radius, radius, radius),
      );
    case Capsule3DShape(:final halfHeight, :final radius):
      final h = halfHeight + radius;
      return (Vector3(-radius, -h, -radius), Vector3(radius, h, radius));
    case Cylinder3DShape(:final halfHeight, :final radius):
      return (
        Vector3(-radius, -halfHeight, -radius),
        Vector3(radius, halfHeight, radius),
      );
    case ConvexHull3DShape(:final points):
      return _pointBounds(points);
    case Mesh3DShape(:final positions):
      return _pointBounds(positions);
    case HeightField3DShape(
      :final heights,
      :final sampleCount,
      :final cellSize,
    ):
      var low = double.infinity, high = -double.infinity;
      for (final h in heights) {
        low = math.min(low, h);
        high = math.max(high, h);
      }
      final side = (sampleCount - 1) * cellSize;
      // Heights go up, towards −Y.
      return (Vector3(0, -high, 0), Vector3(side, -low, side));
    case Transformed3DShape(
      :final shape,
      :final x,
      :final y,
      :final z,
      :final qx,
      :final qy,
      :final qz,
      :final qw,
    ):
      final (lo, hi) = _boundsOf(shape);
      final (min, max) = _turned(lo, hi, Quaternion(qx, qy, qz, qw));
      final offset = Vector3(x, y, z);
      return (min..add(offset), max..add(offset));
    case Compound3DShape(:final parts):
      // The box round every part.
      final (min, max) = _boundsOf(parts.first);
      for (final part in parts.skip(1)) {
        final (lo, hi) = _boundsOf(part);
        min.setValues(
          math.min(min.x, lo.x),
          math.min(min.y, lo.y),
          math.min(min.z, lo.z),
        );
        max.setValues(
          math.max(max.x, hi.x),
          math.max(max.y, hi.y),
          math.max(max.z, hi.z),
        );
      }
      return (min, max);
  }
}

/// [q] made unit length; the identity when it has no length.
Quaternion _unitOf(Quaternion q) {
  final length = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return length.isFinite && length > 1e-12
      ? Quaternion(q.x / length, q.y / length, q.z / length, q.w / length)
      : Quaternion.identity();
}

/// The box round [min]–[max] turned by [q] about the origin.
(Vector3, Vector3) _turned(Vector3 min, Vector3 max, Quaternion q) {
  final length = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!(length > 1e-12) || (q.w.abs() - length).abs() < 1e-12) {
    return (Vector3.copy(min), Vector3.copy(max));
  }
  final x = q.x / length, y = q.y / length, z = q.z / length;
  final w = q.w / length;
  // The rotation matrix's rows.
  final m = [
    1 - 2 * (y * y + z * z),
    2 * (x * y - w * z),
    2 * (x * z + w * y),
    2 * (x * y + w * z),
    1 - 2 * (x * x + z * z),
    2 * (y * z - w * x),
    2 * (x * z - w * y),
    2 * (y * z + w * x),
    1 - 2 * (x * x + y * y),
  ];
  final centre = [
    (min.x + max.x) / 2,
    (min.y + max.y) / 2,
    (min.z + max.z) / 2,
  ];
  final half = [(max.x - min.x) / 2, (max.y - min.y) / 2, (max.z - min.z) / 2];
  final c = List.filled(3, 0.0), e = List.filled(3, 0.0);
  for (var i = 0; i < 3; i++) {
    for (var j = 0; j < 3; j++) {
      c[i] += m[i * 3 + j] * centre[j];
      e[i] += m[i * 3 + j].abs() * half[j];
    }
  }
  return (
    Vector3(c[0] - e[0], c[1] - e[1], c[2] - e[2]),
    Vector3(c[0] + e[0], c[1] + e[1], c[2] + e[2]),
  );
}

(Vector3, Vector3) _pointBounds(Float32List p) {
  if (p.length < 3) return (Vector3.zero(), Vector3.zero());
  final min = Vector3(p[0], p[1], p[2]), max = Vector3(p[0], p[1], p[2]);
  for (var i = 3; i + 2 < p.length; i += 3) {
    min.setValues(
      math.min(min.x, p[i]),
      math.min(min.y, p[i + 1]),
      math.min(min.z, p[i + 2]),
    );
    max.setValues(
      math.max(max.x, p[i]),
      math.max(max.y, p[i + 1]),
      math.max(max.z, p[i + 2]),
    );
  }
  return (min, max);
}
