// What sleeping bodies hear in 3D: the same on Jolt and on the fake engine
// that other packages' tests stand on.

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/physics_3d.dart';
import 'package:just_physics_engine/testing.dart';

void main() {
  for (final (name, make) in <(String, PhysicsEngine3D Function())>[
    ('jolt', PhysicsEngineFactory.create3D),
    ('fake', FakePhysicsEngine3D.new),
  ]) {
    group(name, () {
      late PhysicsEngine3D engine;
      setUp(() => engine = make());

      /// A world with a floor, its top at y = 0, and a crate asleep on it.
      (PhysicsWorld3D, int) crateAsleep({bool stays = false}) {
        final w = engine.createWorld(
          PhysicsWorldSettings3D(reportContactStay: stays),
        );
        addTearDown(w.dispose);
        w.createBody(
          const BoxShape3D(10, 0.5, 10),
          BodySettings3D(position: Vector3(0, 0.5, 0), type: BodyType3D.static),
        );
        final crate = w.createBody(
          const BoxShape3D(0.5, 0.5, 0.5),
          BodySettings3D(position: Vector3(0, -0.6, 0)),
        );
        for (var i = 0; i < 600; i++) {
          w.step(1 / 60);
        }
        expect(w.isAwake(crate), isFalse, reason: 'asleep to start with');
        return (w, crate);
      }

      test('a moving kinematic sensor wakes nothing it passes', () {
        final (w, crate) = crateAsleep();
        final sensor = w.createBody(
          const BoxShape3D(1, 1, 1),
          BodySettings3D(
            position: Vector3(-4, -0.6, 0),
            type: BodyType3D.kinematic,
            isSensor: true,
          ),
        );
        w.setLinearVelocity(sensor, Vector3(4, 0, 0));
        for (var i = 0; i < 120; i++) {
          w.step(1 / 60);
        }
        expect(w.isAwake(crate), isFalse);
      });

      test('a pair asleep together reports no contact stay', () {
        final (w, _) = crateAsleep(stays: true);
        w.drainEvents();
        for (var i = 0; i < 30; i++) {
          w.step(1 / 60);
        }
        expect(
          w.drainEvents().where(
            (e) => e.type == PhysicsEventType3D.contactStay,
          ),
          isEmpty,
        );
      });
    });
  }
}
