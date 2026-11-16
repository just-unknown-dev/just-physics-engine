// Ragdolls, the same on Jolt and on the fake engine other packages' tests
// stand on: three boxes in a row fall together; a pose set is read back;
// a push moves a part; a drive leads them to a pose; a part is not
// destroyed alone, and goes with its ragdoll; a ragdoll goes with its
// world. On Jolt the parts also stay joined.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/physics_3d.dart';
import 'package:just_physics_engine/testing.dart';

const _dt = 1 / 60;

void main() {
  for (final (name, make) in <(String, Physics3DEngine Function())>[
    ('jolt', PhysicsEngineFactory.create3D),
    ('fake', FakePhysics3DEngine.new),
  ]) {
    group(name, () {
      late Physics3DEngine engine;
      setUp(() => engine = make());

      Physics3DWorld world() {
        final w = engine.createWorld(Physics3DWorldSettings());
        addTearDown(w.dispose);
        return w;
      }

      /// Three 0.2 m boxes along +X, 0.25 m apart.
      List<Ragdoll3DPart> parts() => [
        for (var i = 0; i < 3; i++)
          Ragdoll3DPart(
            shape: const Box3DShape(0.1, 0.1, 0.1),
            parent: i - 1,
            position: Vector3(i * 0.25, 0, 0),
            pivot: Vector3(i * 0.25 - 0.125, 0, 0),
            twistAxis: Vector3(1, 0, 0),
            planeAxis: Vector3(0, 1, 0),
          ),
      ];

      Float32List pose(Physics3DWorld w, int ragdoll) {
        final out = Float32List(21);
        expect(w.readRagdollPose(ragdoll, out), 3);
        return out;
      }

      test('three parts fall together; a push moves one', () {
        final w = world();
        final ragdoll = w.createRagdoll(parts(), const Ragdoll3DSettings());
        expect(ragdoll, isNot(0));
        expect(w.isRagdoll(ragdoll), isTrue);
        expect(w.ragdollPartCount(ragdoll), 3);
        w.addRagdollImpulse(ragdoll, 1, Vector3(0, 0, 30));
        for (var i = 0; i < 60; i++) {
          w.step(_dt);
        }
        final p = pose(w, ragdoll);
        expect(p[1], greaterThan(4), reason: 'fell, +Y down');
        expect(p[7 + 2], greaterThan(0.1), reason: 'pushed along +Z');
        if (name == 'jolt') {
          double gap(int a, int b) => math.sqrt(
            math.pow(p[a * 7] - p[b * 7], 2) +
                math.pow(p[a * 7 + 1] - p[b * 7 + 1], 2) +
                math.pow(p[a * 7 + 2] - p[b * 7 + 2], 2),
          );
          expect(gap(0, 1), closeTo(0.25, 0.03), reason: 'joined');
          expect(gap(1, 2), closeTo(0.25, 0.03), reason: 'joined');
        }
      });

      test('a pose set is read back; a drive leads it to one', () {
        final w = world();
        final ragdoll = w.createRagdoll(parts(), const Ragdoll3DSettings());
        final t = Float32List(21);
        for (var i = 0; i < 3; i++) {
          t[i * 7] = 10 + i * 0.25;
          t[i * 7 + 1] = -2;
          t[i * 7 + 6] = 1;
        }
        w.setRagdollPose(ragdoll, t);
        expect(pose(w, ragdoll)[0], closeTo(10, 1e-5));
        expect(pose(w, ragdoll)[2 * 7 + 1], closeTo(-2, 1e-5));
        for (var i = 0; i < 3; i++) {
          t[i * 7] += 1;
        }
        w.driveRagdoll(ragdoll, t, _dt);
        w.step(_dt);
        expect(pose(w, ragdoll)[0], closeTo(11, 0.05));
      });

      test('a part goes only with its ragdoll; a ragdoll with its world', () {
        final w = world();
        final ragdoll = w.createRagdoll(parts(), const Ragdoll3DSettings());
        final body = w.ragdollBody(ragdoll, 0);
        expect(w.isBody(body), isTrue);
        expect(w.ragdollBody(ragdoll, 3), 0);
        w.destroyBody(body);
        expect(w.isBody(body), isTrue);
        w.destroyRagdoll(ragdoll);
        expect(w.isRagdoll(ragdoll), isFalse);
        expect(w.isBody(body), isFalse);
        expect(w.ragdollPartCount(ragdoll), 0);
        w.destroyRagdoll(ragdoll);
        final again = w.createRagdoll(parts(), const Ragdoll3DSettings());
        w.dispose();
        expect(w.isRagdoll(again), isFalse);
      });

      test('parts must follow their parents', () {
        final w = world();
        final wrong = [
          Ragdoll3DPart(shape: const Box3DShape(0.1, 0.1, 0.1), parent: 1),
          Ragdoll3DPart(shape: const Box3DShape(0.1, 0.1, 0.1)),
        ];
        expect(w.createRagdoll(wrong, const Ragdoll3DSettings()), 0);
      });
    });
  }
}
