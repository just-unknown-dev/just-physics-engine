// How bouncy a contact is: the less bouncy of the two bodies, as in 2-D
// (just_physics_engine), so a bouncy ball on a dead floor stays put and the
// same scene bounces alike in either dimension.

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/physics_3d.dart';

final _backend = JoltPhysicsEngine();

/// The highest a ball dropped 3 m onto a floor rises again after it first
/// lands, in metres above where it lands (up is −Y).
double _rebound(double floorRestitution) {
  final w = _backend.createWorld(PhysicsWorldSettings3D());
  addTearDown(w.dispose);
  w.createBody(
    const BoxShape3D(10, 0.5, 10),
    BodySettings3D(
      position: Vector3(0, 0.5, 0),
      type: BodyType3D.static,
      restitution: floorRestitution,
    ),
  );
  final ball = w.createBody(
    const SphereShape3D(0.5),
    BodySettings3D(position: Vector3(0, -3.5, 0), restitution: 0.8),
  );
  final poses = BodyPoses3D();
  final p = Vector3.zero();
  double? landed;
  var highest = double.infinity;
  for (var i = 0; i < 180; i++) {
    w.step(1 / 60);
    w.readBodies([ball], poses);
    poses.positionInto(0, p);
    if (landed == null) {
      if (p.y > -0.6) landed = p.y;
    } else if (p.y < highest) {
      highest = p.y;
    }
  }
  expect(landed, isNotNull, reason: 'it reached the floor');
  return landed! - highest;
}

void main() {
  test('a bouncy ball on a dead floor does not bounce', () {
    expect(_rebound(0), lessThan(0.05));
  });

  test('a bouncy ball on a bouncy floor does', () {
    expect(_rebound(1), greaterThan(1));
  });
}
