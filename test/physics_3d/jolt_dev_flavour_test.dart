// The dev flavour: Jolt's debug renderer as a line buffer. Skipped unless
// the library was built with it — add to this package's pubspec to run:
//
//   hooks:
//     user_defines:
//       just_jolt:
//         dev: true

import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:just_jolt/ffi.dart';
import 'package:just_physics_engine/physics_3d.dart';

void main() {
  final backend = JoltPhysicsEngine();

  test('debug lines draw every body as a wireframe', () {
    expect(backend.unavailableReason, isNull);
    if (!jj_has_debug_renderer()) {
      markTestSkipped('built without the dev flavour');
      return;
    }
    final w = backend.createWorld(PhysicsWorldSettings3D());
    addTearDown(w.dispose);
    w.createBody(
      const BoxShape3D(1, 1, 1),
      BodySettings3D(position: Vector3(3, 0, 0)),
    );
    final a = w.createBody(
      const SphereShape3D(0.5),
      BodySettings3D(position: Vector3(-3, 0, 0)),
    );
    w.createJoint(
      a,
      0,
      JointSettings3D(kind: JointKind3D.point, pointA: Vector3(-3, -1, 0)),
    );
    w.step(1 / 60);

    final lines = w.debugLines()!;
    expect(lines.length % JJ_DEBUG_LINE_FLOATS, 0);
    final count = lines.length ~/ JJ_DEBUG_LINE_FLOATS;
    expect(count, greaterThan(12), reason: 'at least a box\'s edges');
    var maxX = -double.infinity, minX = double.infinity;
    for (var i = 0; i < count; i++) {
      final o = i * JJ_DEBUG_LINE_FLOATS;
      for (final x in [lines[o], lines[o + 3]]) {
        if (x > maxX) maxX = x;
        if (x < minX) minX = x;
      }
    }
    expect(maxX, closeTo(4, 0.1), reason: 'the box reaches x = 4');
    expect(minX, closeTo(-3.5, 0.1), reason: 'the sphere reaches x = -3.5');
    final colour = ByteData.view(lines.buffer).getUint32(6 * 4, Endian.host);
    expect(colour >> 24, greaterThan(250), reason: 'opaque');

    final withJoints = w.debugLines(joints: true)!;
    expect(withJoints.length, greaterThanOrEqualTo(lines.length));
  });
}
