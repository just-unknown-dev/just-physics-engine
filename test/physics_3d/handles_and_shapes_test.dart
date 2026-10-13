// What a 3D backend does with handles it did not make, dead worlds and shapes
// it cannot make: the same on Jolt and on the fake engine that other packages'
// tests stand on. Several of these used to crash Jolt natively.

import 'dart:math' as math;
import 'dart:typed_data';

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

      PhysicsWorld3D world() {
        final w = engine.createWorld(PhysicsWorldSettings3D());
        addTearDown(w.dispose);
        return w;
      }

      int box(
        PhysicsWorld3D w, {
        BodyType3D type = BodyType3D.dynamic,
        double y = 0,
      }) => w.createBody(
        const BoxShape3D(0.5, 0.5, 0.5),
        BodySettings3D(position: Vector3(0, y, 0), type: type),
      );

      test("a character's body is neither destroyed nor joined alone", () {
        final w = world();
        final c = w.createCharacter(CharacterSettings3D(), Vector3.zero());
        final inner = w.characterBody(c);
        expect(inner, isNot(0));
        w.destroyBody(inner);
        expect(w.isBody(inner), isTrue);
        final crate = box(w, y: -5);
        final joint = JointSettings3D(kind: JointKind3D.fixed);
        expect(w.createJoint(inner, crate, joint), 0);
        expect(w.createJoint(crate, inner, joint), 0);
        expect(w.createJoint(crate, 0, joint), isNot(0));
        w.destroyCharacter(c);
        expect(w.isCharacter(c), isFalse);
        expect(w.isBody(inner), isFalse);
        w.step(1 / 60);
      });

      test('handles of another world are passed over', () {
        final a = world(), b = world();
        final body = box(b);
        final c = b.createCharacter(CharacterSettings3D(), Vector3(3, 0, 0));
        a.destroyBody(body);
        a.destroyCharacter(c);
        expect(b.isBody(body), isTrue);
        expect(b.isCharacter(c), isTrue);
        expect(a.isCharacter(c), isFalse);
        expect(a.characterBody(c), 0);
      });

      test('a dead character reads as zeros, in the air', () {
        final w = world();
        final c = w.createCharacter(CharacterSettings3D(), Vector3(1, -2, 3));
        final out = CharacterState3D();
        w.characterState(c, out);
        expect(out.position.x, closeTo(1, 1e-5));
        w.destroyCharacter(c);
        w.moveCharacter(
          c,
          1 / 60,
          Vector3(1, 0, 0),
          out,
          stepDown: Vector3.zero(),
          stepUp: Vector3.zero(),
        );
        expect(out.position.length, 0);
        expect(out.velocity.length, 0);
        expect(out.groundState, GroundState3D.inAir);
        expect(out.groundBody, 0);
        expect(out.contactCount, 0);
      });

      test('shapes that cannot be made make no body', () {
        final w = world();
        final positions = Float32List.fromList([0, 0, 0, 1, 0, 0, 0, 0, 1]);
        final mesh = MeshShape3D(positions, Uint32List.fromList([0, 1, 2]));
        BodySettings3D of(BodyType3D type) => BodySettings3D(type: type);
        expect(w.createBody(mesh, of(BodyType3D.static)), isNot(0));
        expect(w.createBody(mesh, of(BodyType3D.kinematic)), 0);
        expect(w.createBody(mesh, of(BodyType3D.dynamic)), 0);
        final pastTheEnd = MeshShape3D(
          positions,
          Uint32List.fromList([0, 1, 9]),
        );
        expect(pastTheEnd.isValid, isFalse);
        expect(w.createBody(pastTheEnd, of(BodyType3D.static)), 0);
        for (final shape in <Shape3D>[
          const BoxShape3D(double.nan, 1, 1),
          const BoxShape3D(0, 1, 1),
          const SphereShape3D(-1),
          const CompoundShape3D([]),
          ConvexHullShape3D(Float32List(9)),
          const TransformedShape3D(SphereShape3D(1), qw: 0),
        ]) {
          expect(shape.isValid, isFalse, reason: '$shape');
          expect(
            w.createBody(shape, of(BodyType3D.dynamic)),
            0,
            reason: '$shape',
          );
          expect(
            w.overlap(shape, Vector3.zero(), Quaternion.identity()),
            isEmpty,
          );
        }
      });

      test('a layer outside 0–15 is passed over', () {
        final w = world();
        final b = box(w);
        w.setLayer(b, 16);
        w.setLayer(b, -1);
        w.step(1 / 60);
        expect(w.raycast(Vector3(-5, 0, 0), Vector3(10, 0, 0)), isNotEmpty);
      });

      test('a turned box is hit where it reaches', () {
        final w = world();
        // 2 m along X, turned a quarter about Z: upright, reaching y ±1.
        final b = w.createBody(
          const BoxShape3D(1, 0.1, 0.1),
          BodySettings3D(
            type: BodyType3D.static,
            rotation: Quaternion(
              0,
              0,
              math.sin(math.pi / 4),
              math.cos(math.pi / 4),
            ),
          ),
        );
        final hits = w.raycast(Vector3(-5, -0.8, 0), Vector3(10, 0, 0));
        expect(hits.map((h) => h.body), [b]);
      });

      test('a rotation of no length is read as none', () {
        final w = world();
        final b = box(w, type: BodyType3D.static);
        w.setBodyPose(b, Vector3.zero(), Quaternion(0, 0, 0, 0));
        expect(
          w.raycast(Vector3(-5, 0, 0), Vector3(10, 0, 0)).map((h) => h.body),
          [b],
        );
      });

      test('bodies at rest together sleep together', () {
        final w = world();
        w.createBody(
          const BoxShape3D(10, 0.5, 10),
          BodySettings3D(position: Vector3(0, 0.5, 0), type: BodyType3D.static),
        );
        final low = box(w, y: -0.6), high = box(w, y: -1.7);
        for (var i = 0; i < 600; i++) {
          w.step(1 / 60);
        }
        expect(w.isAwake(low), isFalse);
        expect(w.isAwake(high), isFalse);
      });

      test('a full world still makes a character, without its body', () {
        final w = engine.createWorld(PhysicsWorldSettings3D(maxBodies: 1));
        addTearDown(w.dispose);
        expect(box(w), isNot(0));
        final c = w.createCharacter(CharacterSettings3D(), Vector3(3, 0, 0));
        expect(c, isNot(0));
        expect(w.characterBody(c), 0);
      });

      test('six-DOF lists matter to six-DOF joints only', () {
        final w = world();
        final a = box(w), b = box(w, y: -3);
        expect(
          w.createJoint(
            a,
            b,
            JointSettings3D(kind: JointKind3D.hinge, sixDofMin: const []),
          ),
          isNot(0),
        );
        expect(
          w.createJoint(
            a,
            b,
            JointSettings3D(kind: JointKind3D.sixDof, sixDofMin: const []),
          ),
          0,
        );
      });

      test("a character's body is moved by its character only", () {
        final w = world();
        final c = w.createCharacter(CharacterSettings3D(), Vector3.zero());
        final inner = w.characterBody(c);
        w.setLinearVelocity(inner, Vector3(5, 0, 0));
        w.wake(inner);
        expect(w.isAwake(inner), isFalse);
        w.step(1 / 60);
        final poses = BodyPoses3D();
        w.readBodies([inner], poses);
        expect(poses.data[7], 0, reason: 'no velocity of its own');
      });

      test('a rotation of no length reads back as none', () {
        final w = world();
        final b = box(w, type: BodyType3D.static);
        w.setBodyPose(b, Vector3.zero(), Quaternion(0, 0, 0, 0));
        final poses = BodyPoses3D();
        w.readBodies([b], poses);
        expect(
          [poses.data[3], poses.data[4], poses.data[5], poses.data[6]],
          [0, 0, 0, 1],
        );
      });

      test('poses read before dispose stay readable after it', () {
        final w = engine.createWorld(PhysicsWorldSettings3D());
        final b = box(w);
        w.step(1 / 60);
        final poses = BodyPoses3D();
        w.readMovedBodies(poses);
        expect(poses.count, 1);
        expect(poses.bodies.first, b);
        final y = poses.data[1];
        w.dispose();
        expect(poses.data[1], y);
      });

      test('destroyAll leaves the worlds it ended disposed', () {
        final w = engine.createWorld(PhysicsWorldSettings3D());
        final b = box(w);
        engine.destroyAll();
        expect(w.isDisposed, isTrue);
        w.setLinearVelocity(b, Vector3(1, 0, 0));
        w.step(1 / 60);
        expect(w.isBody(b), isFalse);
        w.dispose();
      });
    });
  }
}
