// What sleeping bodies hear in 3D: the same on Jolt and on the fake engine
// that other packages' tests stand on.

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/physics_3d.dart';
import 'package:just_physics_engine/testing.dart';

void main() {
  for (final (name, make) in <(String, Physics3DEngine Function())>[
    ('jolt', PhysicsEngineFactory.create3D),
    ('fake', FakePhysics3DEngine.new),
  ]) {
    group(name, () {
      late Physics3DEngine engine;
      setUp(() => engine = make());

      /// A world with a floor, its top at y = 0, and a crate asleep on it.
      (Physics3DWorld, int) crateAsleep({bool stays = false}) {
        final w = engine.createWorld(
          Physics3DWorldSettings(reportContactStay: stays),
        );
        addTearDown(w.dispose);
        w.createBody(
          const Box3DShape(10, 0.5, 10),
          Body3DSettings(position: Vector3(0, 0.5, 0), type: Body3DType.static),
        );
        final crate = w.createBody(
          const Box3DShape(0.5, 0.5, 0.5),
          Body3DSettings(position: Vector3(0, -0.6, 0)),
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
          const Box3DShape(1, 1, 1),
          Body3DSettings(
            position: Vector3(-4, -0.6, 0),
            type: Body3DType.kinematic,
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
            (e) => e.type == Physics3DEventType.contactStay,
          ),
          isEmpty,
        );
      });
    });
  }
}
