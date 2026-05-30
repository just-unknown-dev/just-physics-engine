part of 'physics_engine.dart';

/// Single contact point entry inside a collision manifold.
class CollisionContactPoint {
  final Offset point;
  final int? featureId;

  const CollisionContactPoint({required this.point, this.featureId});
}

/// Contains information about a collision between two bodies.
class CollisionManifold {
  /// Whether a collision occurred
  final bool isColliding;

  /// The normal vector pointing from body A to body B
  final Offset normal;

  /// The depth of the penetration along the normal
  final double penetration;

  /// Approximate world-space contact point for manifold continuity.
  final Offset? contactPoint;

  /// Optional persistent feature identifier for contact matching.
  final int? contactFeatureId;

  /// Optional manifold contact points for multi-point continuity.
  final List<CollisionContactPoint> contactPoints;

  CollisionManifold({
    required this.isColliding,
    this.normal = Offset.zero,
    this.penetration = 0.0,
    this.contactPoint,
    this.contactFeatureId,
    List<CollisionContactPoint>? contactPoints,
  }) : contactPoints =
           contactPoints ??
           (contactPoint == null
               ? const <CollisionContactPoint>[]
               : <CollisionContactPoint>[
                   CollisionContactPoint(
                     point: contactPoint,
                     featureId: contactFeatureId,
                   ),
                 ]);

  factory CollisionManifold.empty() {
    return CollisionManifold(isColliding: false);
  }
}
