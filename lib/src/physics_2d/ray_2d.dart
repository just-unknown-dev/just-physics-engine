/// Ray — a 2-D ray descriptor with origin, normalised direction, and max distance.
library;

import 'package:flutter/material.dart' show Offset;

/// A ray in 2-D world space with an origin, normalised direction, and maximum
/// travel distance.
class Ray {
  /// Starting point in world space.
  final Offset origin;

  /// Normalised direction vector (always length 1).
  final Offset direction;

  /// Maximum travel distance (world units). The ray ignores anything farther.
  final double maxDistance;

  /// Creates a ray from an [origin] toward [direction].
  ///
  /// [direction] is normalised automatically; passing [Offset.zero] falls back
  /// to the positive-x axis so no division-by-zero can occur.
  Ray({
    required this.origin,
    required Offset direction,
    this.maxDistance = 2000.0,
  }) : direction = _normalise(direction);

  /// Convenience constructor — builds a ray from [from] aimed at [to].
  ///
  /// If [maxDistance] is omitted the distance between the two points is used.
  factory Ray.fromPoints(Offset from, Offset to, {double? maxDistance}) {
    final delta = to - from;
    return Ray(
      origin: from,
      direction: delta,
      maxDistance: maxDistance ?? delta.distance,
    );
  }

  /// Returns the world-space point at parameter [t] (world units along the ray).
  Offset at(double t) => origin + direction * t;

  static Offset _normalise(Offset v) {
    final len = v.distance;
    return len > 1e-9 ? v / len : const Offset(1, 0);
  }
}
