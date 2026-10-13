// 500 dynamic bodies piling up: how long a step and the reads after it take.
// Prints the numbers; fails only when something is badly wrong.

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/physics_3d.dart';

const _dt = 1 / 60;

void main() {
  test('500 bodies: step, batch read and event drain', () {
    final w = JoltPhysicsEngine().createWorld(PhysicsWorldSettings3D());
    addTearDown(w.dispose);
    w.createBody(
      const BoxShape3D(30, 0.5, 30),
      BodySettings3D(position: Vector3(0, 0.5, 0), type: BodyType3D.static),
    );
    for (var i = 0; i < 500; i++) {
      final x = (i % 10) * 1.5 - 7,
          z = ((i ~/ 10) % 10) * 1.5 - 7,
          y = -1 - (i ~/ 100) * 1.5;
      final shape = i.isEven
          ? const BoxShape3D(0.5, 0.5, 0.5)
          : const SphereShape3D(0.5);
      w.createBody(shape, BodySettings3D(position: Vector3(x, y, z)));
    }
    final poses = BodyPoses3D();
    // Warm up.
    for (var i = 0; i < 30; i++) {
      w.step(_dt);
      w.readMovedBodies(poses);
      w.drainEvents();
    }
    const steps = 240;
    final step = Stopwatch(), read = Stopwatch(), drain = Stopwatch();
    var moved = 0, events = 0;
    for (var i = 0; i < steps; i++) {
      step.start();
      w.step(_dt);
      step.stop();
      read.start();
      w.readMovedBodies(poses);
      read.stop();
      moved += poses.count;
      drain.start();
      events += w.drainEvents().length;
      drain.stop();
    }
    double ms(Stopwatch s) => s.elapsedMicroseconds / 1000 / steps;
    // ignore: avoid_print
    print(
      'just_jolt perf, 500 bodies: step ${ms(step).toStringAsFixed(3)} ms, '
      'readMovedBodies ${ms(read).toStringAsFixed(3)} ms '
      '(${(moved / steps).toStringAsFixed(0)} moving), '
      'drainEvents ${ms(drain).toStringAsFixed(3)} ms '
      '(${(events / steps).toStringAsFixed(1)} per step)',
    );
    expect(ms(step), lessThan(16), reason: 'a 60 Hz budget');
    expect(ms(read), lessThan(2));
  });
}
