import 'dart:typed_data';

/// The shape of a body, a query or a character, as plain data a backend
/// turns into its own.
///
/// Units are metres. Shapes compare by value, so a backend can share one
/// native shape between every body that asks for an equal one. The big
/// ones ([ConvexHull3DShape], [Mesh3DShape], [HeightField3DShape]) compare
/// their data by identity: reuse the same lists to share them, and leave a
/// list (or a [Compound3DShape]'s parts) unchanged once a shape holds it.
sealed class Shape3D {
  const Shape3D();

  /// Whether this shape is well formed: every size finite and above zero, a
  /// hull of at least 4 points, a mesh of at least one triangle whose
  /// indices are all in its vertex list, a height field of at least 4 × 4
  /// samples, a compound of at least one part, every number finite, a
  /// rotation of some length. A body of a shape that is not is never made
  /// (`createBody` returns 0) and a query with one finds nothing, on every
  /// backend. A backend can still refuse a well-formed shape it cannot build
  /// — Jolt a hull whose points are all on one plane, or a mesh of only
  /// degenerate triangles — so check what `createBody` returns.
  bool get isValid => switch (this) {
    Box3DShape(:final hx, :final hy, :final hz) =>
      _positive(hx) && _positive(hy) && _positive(hz),
    Sphere3DShape(:final radius) => _positive(radius),
    Capsule3DShape(:final halfHeight, :final radius) =>
      _positive(halfHeight) && _positive(radius),
    Cylinder3DShape(:final halfHeight, :final radius) =>
      _positive(halfHeight) && _positive(radius),
    ConvexHull3DShape(:final points) =>
      points.length >= 12 && _allFinite(points),
    Mesh3DShape(:final positions, :final indices) => _validMesh(
      positions,
      indices,
    ),
    HeightField3DShape(:final heights, :final sampleCount, :final cellSize) =>
      sampleCount >= 4 &&
          heights.length >= sampleCount * sampleCount &&
          _positive(cellSize) &&
          _allFinite(heights),
    final Transformed3DShape t => _validTransformed(t),
    Compound3DShape(:final parts) =>
      parts.isNotEmpty && parts.every((part) => part.isValid),
  };

  static bool _positive(double v) => v.isFinite && v > 0;

  static bool _allFinite(Float32List values) {
    for (final v in values) {
      if (!v.isFinite) return false;
    }
    return true;
  }

  static bool _validMesh(Float32List positions, Uint32List indices) {
    final vertices = positions.length ~/ 3;
    if (vertices < 3 || indices.length < 3) return false;
    if (!_allFinite(positions)) return false;
    for (final i in indices) {
      if (i >= vertices) return false;
    }
    return true;
  }

  static bool _validTransformed(Transformed3DShape t) {
    final q = t.qx * t.qx + t.qy * t.qy + t.qz * t.qz + t.qw * t.qw;
    return t.shape.isValid &&
        t.x.isFinite &&
        t.y.isFinite &&
        t.z.isFinite &&
        q.isFinite &&
        q > 1e-12;
  }
}

/// A box of half extents [hx], [hy], [hz] about its centre.
final class Box3DShape extends Shape3D {
  /// A box twice [hx] by twice [hy] by twice [hz].
  const Box3DShape(this.hx, this.hy, this.hz);

  /// Half its size along X, Y and Z.
  final double hx, hy, hz;

  @override
  bool operator ==(Object other) =>
      other is Box3DShape && other.hx == hx && other.hy == hy && other.hz == hz;

  @override
  int get hashCode => Object.hash(Box3DShape, hx, hy, hz);

  @override
  String toString() => 'Box3DShape($hx, $hy, $hz)';
}

/// A sphere of [radius] about its centre.
final class Sphere3DShape extends Shape3D {
  /// A sphere of [radius].
  const Sphere3DShape(this.radius);

  /// Its radius.
  final double radius;

  @override
  bool operator ==(Object other) =>
      other is Sphere3DShape && other.radius == radius;

  @override
  int get hashCode => Object.hash(Sphere3DShape, radius);

  @override
  String toString() => 'Sphere3DShape($radius)';
}

/// A capsule along Y about its centre: a cylinder [halfHeight] each way
/// with a hemisphere of [radius] on each end.
final class Capsule3DShape extends Shape3D {
  /// A capsule whose straight part is twice [halfHeight] long.
  const Capsule3DShape(this.halfHeight, this.radius);

  /// Half the length of its straight part (not counting the caps).
  final double halfHeight;

  /// The radius of the cylinder and of the caps.
  final double radius;

  @override
  bool operator ==(Object other) =>
      other is Capsule3DShape &&
      other.halfHeight == halfHeight &&
      other.radius == radius;

  @override
  int get hashCode => Object.hash(Capsule3DShape, halfHeight, radius);

  @override
  String toString() => 'Capsule3DShape($halfHeight, $radius)';
}

/// A cylinder along Y about its centre.
final class Cylinder3DShape extends Shape3D {
  /// A cylinder twice [halfHeight] tall.
  const Cylinder3DShape(this.halfHeight, this.radius);

  /// Half its height.
  final double halfHeight;

  /// Its radius.
  final double radius;

  @override
  bool operator ==(Object other) =>
      other is Cylinder3DShape &&
      other.halfHeight == halfHeight &&
      other.radius == radius;

  @override
  int get hashCode => Object.hash(Cylinder3DShape, halfHeight, radius);

  @override
  String toString() => 'Cylinder3DShape($halfHeight, $radius)';
}

/// The convex hull of [points] (x, y, z each).
final class ConvexHull3DShape extends Shape3D {
  /// The smallest convex shape holding every point.
  const ConvexHull3DShape(this.points);

  /// Three floats per point.
  final Float32List points;

  @override
  bool operator ==(Object other) =>
      other is ConvexHull3DShape && identical(other.points, points);

  @override
  int get hashCode => Object.hash(ConvexHull3DShape, identityHashCode(points));
}

/// A triangle mesh: three indices per triangle into [positions]. Static
/// bodies only: a body that moves needs a convex shape, or a
/// [Compound3DShape] of them.
final class Mesh3DShape extends Shape3D {
  /// A mesh of [indices] over [positions] (x, y, z each).
  const Mesh3DShape(this.positions, this.indices);

  /// Three floats per vertex.
  final Float32List positions;

  /// Three indices per triangle.
  final Uint32List indices;

  /// How many triangles it has.
  int get triangleCount => indices.length ~/ 3;

  @override
  bool operator ==(Object other) =>
      other is Mesh3DShape &&
      identical(other.positions, positions) &&
      identical(other.indices, indices);

  @override
  int get hashCode => Object.hash(
    Mesh3DShape,
    identityHashCode(positions),
    identityHashCode(indices),
  );
}

/// Terrain: a square grid of heights. Static bodies only.
///
/// [heights] holds [sampleCount] rows of [sampleCount] samples; sample
/// `heights[row * sampleCount + column]` is how far **up** (towards −Y, the
/// engine's up) the ground is at x = column × [cellSize], z = row ×
/// [cellSize] in the body's space. The grid starts at the body's origin and
/// runs towards +X and +Z. A sample of [hole] or more is a hole: the cells
/// round it are not there, and things fall through.
final class HeightField3DShape extends Shape3D {
  /// A terrain of [sampleCount] × [sampleCount] [heights], [cellSize]
  /// metres apart.
  const HeightField3DShape(this.heights, this.sampleCount, this.cellSize);

  /// A height that marks a hole.
  static const double hole = 3.0e38;

  /// Row-major heights, [sampleCount] squared of them.
  final Float32List heights;

  /// Samples along each side; at least 4.
  final int sampleCount;

  /// Distance between neighbouring samples.
  final double cellSize;

  @override
  bool operator ==(Object other) =>
      other is HeightField3DShape &&
      identical(other.heights, heights) &&
      other.sampleCount == sampleCount &&
      other.cellSize == cellSize;

  @override
  int get hashCode => Object.hash(
    HeightField3DShape,
    identityHashCode(heights),
    sampleCount,
    cellSize,
  );
}

/// [shape] moved by ([x], [y], [z]) and turned by the unit quaternion
/// ([qx], [qy], [qz], [qw]) in its body's space: a collider's offset.
final class Transformed3DShape extends Shape3D {
  /// [shape] at an offset and a rotation.
  const Transformed3DShape(
    this.shape, {
    this.x = 0,
    this.y = 0,
    this.z = 0,
    this.qx = 0,
    this.qy = 0,
    this.qz = 0,
    this.qw = 1,
  });

  /// The shape being placed.
  final Shape3D shape;

  /// The offset.
  final double x, y, z;

  /// The rotation, a unit quaternion.
  final double qx, qy, qz, qw;

  /// Whether it neither moves nor turns [shape].
  bool get isIdentity =>
      x == 0 && y == 0 && z == 0 && qx == 0 && qy == 0 && qz == 0 && qw == 1;

  @override
  bool operator ==(Object other) =>
      other is Transformed3DShape &&
      other.shape == shape &&
      other.x == x &&
      other.y == y &&
      other.z == z &&
      other.qx == qx &&
      other.qy == qy &&
      other.qz == qz &&
      other.qw == qw;

  @override
  int get hashCode =>
      Object.hash(Transformed3DShape, shape, x, y, z, qx, qy, qz, qw);

  @override
  String toString() => 'Transformed3DShape($shape at ($x, $y, $z))';
}

/// Several shapes as one: a staircase of boxes, an arch of hulls. Each part
/// is placed in its body's space — a [Transformed3DShape] for one that is
/// not at the body's origin. Any body type may have one, a dynamic body
/// included, so a concave thing made of convex parts can tumble.
final class Compound3DShape extends Shape3D {
  /// [parts], at least one.
  const Compound3DShape(this.parts);

  /// The shapes it is made of.
  final List<Shape3D> parts;

  @override
  bool operator ==(Object other) {
    if (other is! Compound3DShape || other.parts.length != parts.length) {
      return false;
    }
    for (var i = 0; i < parts.length; i++) {
      if (other.parts[i] != parts[i]) return false;
    }
    return true;
  }

  @override
  int get hashCode => Object.hash(Compound3DShape, Object.hashAll(parts));

  @override
  String toString() => 'Compound3DShape(${parts.length} parts)';
}
