// The same calls give the same world: 600 steps of a busy scene run twice
// in one process end bit for bit the same, on Jolt's worker threads too.
// (Matching across platforms needs Jolt's cross-platform determinism build,
// which just_jolt does not turn on.)

import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/physics_3d.dart';

const _dt = 1 / 60;

/// Runs the scene and returns every body's pose and the character's
/// position, as raw floats.
Float32List _run(JoltPhysicsEngine backend) {
  final w = backend.createWorld(
    PhysicsWorldSettings3D(reportContactStay: true),
  );
  try {
    w.createBody(
      const BoxShape3D(20, 0.5, 20),
      BodySettings3D(position: Vector3(0, 0.5, 0), type: BodyType3D.static),
    );
    final bodies = <int>[];
    // A pyramid of boxes and a rain of spheres and capsules.
    for (var layer = 0; layer < 6; layer++) {
      for (var i = 0; i < 6 - layer; i++) {
        bodies.add(
          w.createBody(
            const BoxShape3D(0.5, 0.5, 0.5),
            BodySettings3D(
              position: Vector3(
                i * 1.02 + layer * 0.51 - 3,
                -0.5 - layer * 1.0,
                0,
              ),
            ),
          ),
        );
      }
    }
    for (var i = 0; i < 40; i++) {
      final shape = i.isEven
          ? const SphereShape3D(0.3)
          : const CapsuleShape3D(0.3, 0.2);
      bodies.add(
        w.createBody(
          shape,
          BodySettings3D(
            position: Vector3(
              (i % 8) * 0.7 - 2.5,
              -8 - (i ~/ 8) * 0.9,
              (i % 3) * 0.4 - 0.4,
            ),
            angularVelocity: Vector3(0.1 * i, 0, 0),
          ),
        ),
      );
    }
    final character = w.createCharacter(
      CharacterSettings3D(),
      Vector3(-8, 0, 0),
    );
    final state = CharacterState3D();
    final velocity = Vector3.zero();
    for (var step = 0; step < 600; step++) {
      velocity.setValues(2, step < 300 ? 0 : 1, 0);
      velocity.addScaled(w.gravity, _dt);
      w.moveCharacter(
        character,
        _dt,
        velocity,
        state,
        stepDown: Vector3(0, 0.5, 0),
        stepUp: Vector3(0, -0.4, 0),
      );
      if (step == 100) w.addImpulse(bodies.first, Vector3(0, -500, 300));
      w.step(_dt);
      w.drainEvents();
    }
    final poses = BodyPoses3D();
    w.readBodies(bodies, poses);
    return Float32List.fromList([
      ...poses.data,
      state.position.x,
      state.position.y,
      state.position.z,
    ]);
  } finally {
    w.dispose();
  }
}

void main() {
  test('600 steps twice give the same result', () {
    final backend = JoltPhysicsEngine();
    final first = _run(backend);
    final second = _run(backend);
    expect(first.length, (21 + 40) * BodyPoses3D.stride + 3);
    // Something happened: the pyramid fell, the rain landed.
    expect(first.any((v) => v != 0), isTrue);
    expect(
      ByteData.view(second.buffer).buffer.asUint8List(),
      ByteData.view(first.buffer).buffer.asUint8List(),
    );
  });
}
