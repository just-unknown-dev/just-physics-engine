part of 'physics_2d_engine.dart';

// The pure-Dart engine's narrow phase and response: the deepest overlap of
// two bodies' shapes, one-way platforms, and the impulse that resolves a
// contact.

/// The deepest overlap between any shape of [a] and any of [b], or null.
CollisionManifold? _deepestManifold(Physics2DBody a, Physics2DBody b) {
  final posA = a.position.toOffset();
  final posB = b.position.toOffset();
  final extraA = a.additionalShapes, extraB = b.additionalShapes;
  CollisionManifold? best;
  for (var i = -1; i < extraA.length; i++) {
    final shapeA = i < 0 ? a.shape : extraA[i];
    for (var j = -1; j < extraB.length; j++) {
      final m = shapeA.manifoldWith(posA, j < 0 ? b.shape : extraB[j], posB);
      if (m.isColliding && (best == null || m.penetration > best.penetration)) {
        best = m;
      }
    }
  }
  return best;
}

/// Cosine threshold for a one-way contact, ~60 degrees of tolerance.
///
/// Tighter values drop a player who lands on the very edge of a platform;
/// looser values let a player rising steeply from below catch on it. Kept
/// identical to the native pre-solve filter so both backends agree.
const double _oneWaySolidCos = 0.5;

/// Whether a contact involving exactly one one-way body should be resolved.
///
/// [normal] points from [a] to [b]; it is flipped as needed so the test
/// always reads "which way does the mover lie relative to the platform".
/// Screen-space convention: +Y is down, so "above" is negative Y.
bool _oneWayContactIsSolid(Physics2DBody a, Physics2DBody b, Offset normal) {
  final platformIsA = a.isOneWay;
  final platform = platformIsA ? a : b;
  final sign = platformIsA ? 1.0 : -1.0;
  final nx = sign * normal.dx;
  final ny = sign * normal.dy;

  return switch (platform.oneWayDirection) {
    OneWayDirection.fromAbove => ny < -_oneWaySolidCos,
    OneWayDirection.fromBelow => ny > _oneWaySolidCos,
    OneWayDirection.fromLeft => nx < -_oneWaySolidCos,
    OneWayDirection.fromRight => nx > _oneWaySolidCos,
  };
}

/// Pushes [a] and [b] apart along [manifold] and takes away the speed
/// they close at: positional correction, then an impulse with restitution,
/// then friction.
void _resolveCollision(
  Physics2DBody a,
  Physics2DBody b,
  CollisionManifold manifold,
) {
  final normal = manifold.normal;
  final penetration = manifold.penetration;

  if (penetration <= 0) return;

  // Only wake bodies for significant collisions
  if (penetration > 0.05 ||
      a.velocity.lengthSquared > 1.0 ||
      b.velocity.lengthSquared > 1.0) {
    a.isAwake = true;
    a.sleepTimer = 0.0;
    b.isAwake = true;
    b.sleepTimer = 0.0;
  }

  // ── Positional correction (mass-proportional) ─────────────────────────
  // solverInverseMass, not inverseMass: a kinematic body has a real mass but
  // must be immovable, so it contributes zero here while still pushing the
  // dynamic side out. When both sides are immovable the sum is zero and
  // there is nothing to resolve.
  final invMassA = a.solverInverseMass;
  final invMassB = b.solverInverseMass;
  final inverseMassSum = invMassA + invMassB;

  if (inverseMassSum == 0) return; // both immovable

  const correctionPercent = 0.8;
  const slop = 0.05;
  final correctionMag =
      math.max(penetration - slop, 0.0) / inverseMassSum * correctionPercent;
  a.position.x -= normal.dx * correctionMag * invMassA;
  a.position.y -= normal.dy * correctionMag * invMassA;
  b.position.x += normal.dx * correctionMag * invMassB;
  b.position.y += normal.dy * correctionMag * invMassB;

  // ── Impulse resolution ────────────────────────────────────────────────
  final rvx = b.velocity.x - a.velocity.x;
  final rvy = b.velocity.y - a.velocity.y;
  final velAlongNormal = rvx * normal.dx + rvy * normal.dy;

  if (velAlongNormal > 0) return; // separating

  final restitution = -velAlongNormal < Physics2DEngine.restitutionThreshold
      ? 0.0
      : Physics2DEngine.mixRestitution(a.restitution, b.restitution);
  final j = -(1.0 + restitution) * velAlongNormal / inverseMassSum;

  final jnx = normal.dx * j;
  final jny = normal.dy * j;
  a.velocity.x -= jnx * invMassA;
  a.velocity.y -= jny * invMassA;
  b.velocity.x += jnx * invMassB;
  b.velocity.y += jny * invMassB;

  // ── Friction (Tangent Impulse) ──────────────────────────────────────────
  final rvx2 = b.velocity.x - a.velocity.x;
  final rvy2 = b.velocity.y - a.velocity.y;
  final rvDotN = rvx2 * normal.dx + rvy2 * normal.dy;
  var tx = rvx2 - normal.dx * rvDotN;
  var ty = rvy2 - normal.dy * rvDotN;

  final tangentLen = math.sqrt(tx * tx + ty * ty);
  if (tangentLen > 0.0001) {
    final invLen = 1.0 / tangentLen;
    tx *= invLen;
    ty *= invLen;

    final jt = -(rvx2 * tx + rvy2 * ty) / inverseMassSum;
    final mu = Physics2DEngine.mixFriction(a.friction, b.friction);

    double fScalar = jt;
    if (fScalar.abs() > j * mu) {
      fScalar = (fScalar > 0 ? 1.0 : -1.0) * j * mu;
    }

    a.velocity.x -= tx * fScalar * invMassA;
    a.velocity.y -= ty * fScalar * invMassA;
    b.velocity.x += tx * fScalar * invMassB;
    b.velocity.y += ty * fScalar * invMassB;
  }
}
