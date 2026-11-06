part of 'physics_2d_engine.dart';

/// Contains information about a collision between two bodies.
class CollisionManifold {
  /// Whether a collision occurred
  final bool isColliding;

  /// The normal vector pointing from body A to body B
  final Offset normal;

  /// The depth of the penetration along the normal
  final double penetration;

  /// A result: [isColliding], and the [normal] and [penetration] to push
  /// the two apart along when they are.
  CollisionManifold({
    required this.isColliding,
    this.normal = Offset.zero,
    this.penetration = 0.0,
  });

  /// No collision.
  factory CollisionManifold.empty() {
    return CollisionManifold(isColliding: false);
  }
}
