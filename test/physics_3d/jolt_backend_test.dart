// JoltPhysicsEngine through the PhysicsEngine3D interface: bodies, batch reads,
// queries, events, layers, shapes and joints in a +Y-down world in metres.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:just_jolt/ffi.dart';
import 'package:just_physics_engine/jolt.dart';
import 'package:just_physics_engine/physics_3d.dart';

const _dt = 1 / 60;
final _backend = JoltPhysicsEngine();

PhysicsWorld3D _world({
  List<int>? masks,
  bool stay = false,
  bool activation = false,
  Vector3? gravity,
}) {
  final world = _backend.createWorld(
    PhysicsWorldSettings3D(
      layerMasks: masks,
      reportContactStay: stay,
      reportActivation: activation,
      gravity: gravity,
    ),
  );
  addTearDown(world.dispose);
  return world;
}

/// A 20 m floor whose top is at y = 0.
int _floor(PhysicsWorld3D w, {int layer = 0}) => w.createBody(
  const BoxShape3D(10, 0.5, 10),
  BodySettings3D(
    position: Vector3(0, 0.5, 0),
    type: BodyType3D.static,
    layer: layer,
  ),
);

int _box(
  PhysicsWorld3D w,
  Vector3 at, {
  double half = 0.5,
  BodyType3D type = BodyType3D.dynamic,
  int layer = 0,
  bool sensor = false,
}) => w.createBody(
  BoxShape3D(half, half, half),
  BodySettings3D(position: at, type: type, layer: layer, isSensor: sensor),
);

Vector3 _position(PhysicsWorld3D w, int body) {
  final poses = BodyPoses3D();
  w.readBodies([body], poses);
  final p = Vector3.zero();
  poses.positionInto(0, p);
  return p;
}

void _run(PhysicsWorld3D w, int steps) {
  for (var i = 0; i < steps; i++) {
    w.step(_dt);
  }
}

void main() {
  test('the engine is available here and names Jolt', () {
    expect(_backend.unavailableReason, isNull);
    expect(_backend.name, 'jolt');
    expect(_backend.joltVersion, '5.6.0');
  });

  group('bodies', () {
    test('a box falls under +Y gravity and rests on the floor', () {
      final w = _world();
      _floor(w);
      final box = _box(w, Vector3(0, -3, 0));
      expect(w.bodyCount, 2);
      _run(w, 180);
      final p = _position(w, box);
      // Jolt lets resting contacts sink up to 2 cm (its penetration slop).
      expect(p.y, closeTo(-0.5, 0.03));
      expect(p.x, closeTo(0, 0.01));
    });

    test(
      'moved bodies are read in one batch, and those that fell asleep once more',
      () {
        final w = _world();
        _floor(w);
        final boxes = [
          for (var i = 0; i < 5; i++) _box(w, Vector3(i * 2.0, -1, 0)),
        ];
        final poses = BodyPoses3D();
        w.step(_dt);
        w.readMovedBodies(poses);
        expect(poses.count, 5, reason: 'the static floor is left out');
        expect(poses.bodies.toSet(), boxes.toSet());
        expect(poses.isAwake(0), isTrue);

        var sawAsleep = false;
        for (var i = 0; i < 600 && !sawAsleep; i++) {
          w.step(_dt);
          w.readMovedBodies(poses);
          for (var k = 0; k < poses.count; k++) {
            if (!poses.isAwake(k)) sawAsleep = true;
          }
        }
        expect(
          sawAsleep,
          isTrue,
          reason: 'a body that falls asleep is reported asleep once',
        );
        for (var i = 0; i < 5; i++) {
          w.step(_dt);
        }
        w.readMovedBodies(poses);
        expect(poses.count, 0, reason: 'nothing moves at rest');
        expect(w.isAwake(boxes.first), isFalse);
      },
    );

    test(
      'velocities, impulses and forces move a body; static bodies ignore them',
      () {
        final w = _world(gravity: Vector3.zero());
        final box = _box(w, Vector3.zero());
        w.setLinearVelocity(box, Vector3(2, 0, 0));
        _run(w, 60);
        expect(_position(w, box).x, closeTo(2, 0.1));
        w.setLinearVelocity(box, Vector3.zero());
        w.addImpulse(box, Vector3(0, 0, 1000)); // A 1 m box is 1000 kg.
        _run(w, 60);
        expect(_position(w, box).z, closeTo(1, 0.1));

        final wall = _box(w, Vector3(10, 0, 0), type: BodyType3D.static);
        w.setLinearVelocity(wall, Vector3(5, 0, 0));
        w.addForce(wall, Vector3(1e6, 0, 0));
        w.moveKinematic(wall, Vector3(20, 0, 0), Quaternion.identity(), _dt);
        _run(w, 10);
        expect(_position(w, wall).x, 10);
      },
    );

    test(
      'a kinematic body moves to its target and pushes what is in the way',
      () {
        final w = _world();
        _floor(w);
        final pusher = _box(
          w,
          Vector3(-2, -0.5, 0),
          type: BodyType3D.kinematic,
        );
        final crate = _box(w, Vector3(0, -0.5, 0));
        for (var i = 1; i <= 60; i++) {
          w.moveKinematic(
            pusher,
            Vector3(-2 + i * 2 / 60, -0.5, 0),
            Quaternion.identity(),
            _dt,
          );
          w.step(_dt);
        }
        expect(_position(w, pusher).x, closeTo(0, 1e-3));
        expect(_position(w, crate).x, greaterThan(0.9), reason: 'pushed ahead');
      },
    );

    test('a teleport, gravity factor and a dead handle', () {
      final w = _world();
      final box = _box(w, Vector3.zero());
      w.setGravityFactor(box, 0);
      _run(w, 30);
      expect(_position(w, box).y, closeTo(0, 1e-6));
      w.setBodyPose(box, Vector3(4, 5, 6), Quaternion.identity());
      expect(_position(w, box), Vector3(4, 5, 6));
      w.destroyBody(box);
      expect(w.isBody(box), isFalse);
      // Every call accepts a dead handle.
      w
        ..setLinearVelocity(box, Vector3(1, 1, 1))
        ..addForce(box, Vector3(1, 1, 1))
        ..setBodyPose(box, Vector3.zero(), Quaternion.identity())
        ..destroyBody(box);
      expect(_position(w, box), Vector3.zero());
    });

    test(
      'a mesh and a height field make static ground; a dynamic mesh is refused',
      () {
        final w = _world();
        // Two triangles: a 10 m square at y = 0, facing up (−Y).
        final mesh = MeshShape3D(
          Float32List.fromList([-5, 0, -5, 5, 0, -5, 5, 0, 5, -5, 0, 5]),
          Uint32List.fromList([0, 1, 2, 0, 2, 3]),
        );
        expect(
          w.createBody(mesh, BodySettings3D(type: BodyType3D.static)),
          isNot(0),
        );
        final ball = w.createBody(
          const SphereShape3D(0.5),
          BodySettings3D(position: Vector3(0, -2, 0)),
        );
        _run(w, 120);
        expect(
          _position(w, ball).y,
          closeTo(-0.5, 0.05),
          reason: 'resting on the mesh',
        );

        final field = HeightFieldShape3D(
          Float32List(16)..fillRange(0, 16, 1),
          4,
          2,
        );
        final ground = w.createBody(
          field,
          BodySettings3D(position: Vector3(20, 0, 0), type: BodyType3D.static),
        );
        expect(ground, isNot(0));
        final rock = w.createBody(
          const SphereShape3D(0.5),
          BodySettings3D(position: Vector3(23, -4, 3)),
        );
        _run(w, 120);
        expect(
          _position(w, rock).y,
          closeTo(-1.5, 0.05),
          reason: 'heights rise towards −Y',
        );

        expect(
          w.createBody(field, BodySettings3D()), // dynamic, the default
          0,
        );

        // Holes: four samples in the middle of an 8 × 8 field, 2 m apart.
        final holed = Float32List(64)..fillRange(0, 64, 1);
        for (final (row, column) in [(3, 3), (3, 4), (4, 3), (4, 4)]) {
          holed[row * 8 + column] = HeightFieldShape3D.hole;
        }
        w.createBody(
          HeightFieldShape3D(holed, 8, 2),
          BodySettings3D(position: Vector3(-40, 0, 0), type: BodyType3D.static),
        );
        final through = w.createBody(
          const SphereShape3D(0.3),
          BodySettings3D(position: Vector3(-33, -4, 7)),
        );
        final beside = w.createBody(
          const SphereShape3D(0.3),
          BodySettings3D(position: Vector3(-38, -4, 1)),
        );
        _run(w, 120);
        expect(_position(w, through).y, greaterThan(2), reason: 'fell through the hole');
        expect(_position(w, beside).y, closeTo(-1.3, 0.05), reason: 'held where there is ground');

        // A terrain's size: 129 samples a side, not a multiple of Jolt's
        // blocks.
        final big = Float32List(129 * 129)..fillRange(0, 129 * 129, 2);
        expect(
          w.createBody(
            HeightFieldShape3D(big, 129, 0.5),
            BodySettings3D(position: Vector3(0, 0, 40), type: BodyType3D.static),
          ),
          isNot(0),
        );
        final pebble = w.createBody(
          const SphereShape3D(0.3),
          BodySettings3D(position: Vector3(50, -6, 90)),
        );
        _run(w, 120);
        expect(_position(w, pebble).y, closeTo(-2.3, 0.05), reason: 'on the far side of it too');
      },
    );

    test('a convex hull and an offset shape', () {
      final w = _world();
      _floor(w);
      final cube = Float32List.fromList([
        for (final x in [-0.5, 0.5])
          for (final y in [-0.5, 0.5])
            for (final z in [-0.5, 0.5]) ...[x, y, z],
      ]);
      final hull = w.createBody(
        ConvexHullShape3D(cube),
        BodySettings3D(position: Vector3(0, -2, 0)),
      );
      final shifted = w.createBody(
        const TransformedShape3D(BoxShape3D(0.5, 0.5, 0.5), y: -1),
        BodySettings3D(position: Vector3(4, -3, 0), lockRotation: true),
      );
      _run(w, 180);
      expect(_position(w, hull).y, closeTo(-0.5, 0.05));
      expect(
        _position(w, shifted).y,
        closeTo(0.5, 0.05),
        reason: 'the box sits 1 m above its body',
      );
    });

    test('equal shapes are shared and let go with their last body', () {
      final w = _world() as JoltWorld;
      final boxes = [
        for (var i = 0; i < 50; i++) _box(w, Vector3(i * 2.0, 0, 0)),
      ];
      expect(w.shapes.length, 1);
      for (final b in boxes.skip(1)) {
        w.destroyBody(b);
      }
      expect(w.shapes.length, 1);
      w.destroyBody(boxes.first);
      expect(w.shapes.length, 0);
    });
  });

  group('layers', () {
    test('layers the grid keeps apart pass through each other', () {
      // Layer 1 collides with nothing but itself.
      final w = _world(masks: [0xFFFF & ~2, 2]);
      _floor(w);
      final ghost = _box(w, Vector3(0, -2, 0), layer: 1);
      final solid = _box(w, Vector3(3, -2, 0));
      _run(w, 120);
      expect(
        _position(w, ghost).y,
        greaterThan(3),
        reason: 'fell through the floor',
      );
      expect(_position(w, solid).y, closeTo(-0.5, 0.05));
    });

    test('a body can move to another layer', () {
      final w = _world(masks: [0xFFFF & ~2, 2]);
      _floor(w);
      final box = _box(w, Vector3(0, -2, 0), layer: 1);
      w.setLayer(box, 0);
      _run(w, 120);
      expect(_position(w, box).y, closeTo(-0.5, 0.05));
    });
  });

  group('events', () {
    test('one contact begin when a box lands and one end when it leaves', () {
      final w = _world();
      final floor = _floor(w);
      final box = _box(w, Vector3(0, -2, 0));
      final events = <PhysicsEvent3D>[];
      for (var i = 0; i < 90; i++) {
        w.step(_dt);
        events.addAll(w.drainEvents());
      }
      final begins = events
          .where((e) => e.type == PhysicsEventType3D.contactBegin)
          .toList();
      expect(begins, hasLength(1));
      expect({begins.single.bodyA, begins.single.bodyB}, {box, floor});
      expect(begins.single.normal.y.abs(), closeTo(1, 0.01));

      w.setLinearVelocity(box, Vector3(0, -6, 0));
      for (var i = 0; i < 20; i++) {
        w.step(_dt);
        events.addAll(w.drainEvents());
      }
      expect(
        events.where((e) => e.type == PhysicsEventType3D.contactEnd),
        hasLength(1),
      );
    });

    test(
      'a body falling asleep on the floor still touches it; leaving or going ends it',
      () {
        final w = _world();
        final floor = _floor(w);
        final box = _box(w, Vector3(0, -1, 0));
        final types = <PhysicsEventType3D>[];
        for (var i = 0; i < 600; i++) {
          w.step(_dt);
          types.addAll(w.drainEvents().map((e) => e.type));
        }
        expect(w.isAwake(box), isFalse, reason: 'asleep by now');
        expect(types, [
          PhysicsEventType3D.contactBegin,
        ], reason: 'no end for falling asleep');

        // Woken where it lies: still touching, nothing new.
        w.wake(box);
        _run(w, 3);
        expect(w.drainEvents(), isEmpty);
        // Thrown up: it ends.
        w.setLinearVelocity(box, Vector3(0, -6, 0));
        _run(w, 10);
        expect(w.drainEvents().map((e) => e.type), [
          PhysicsEventType3D.contactEnd,
        ]);

        // A sleeping body destroyed ends its contacts too.
        final other = _box(w, Vector3(3, -1, 0));
        for (var i = 0; i < 600 && w.isAwake(other); i++) {
          w.step(_dt);
        }
        w.drainEvents();
        w.destroyBody(other);
        w.step(_dt);
        final ended = w
            .drainEvents()
            .where((e) => e.type == PhysicsEventType3D.contactEnd)
            .toList();
        expect(ended, hasLength(1));
        expect({ended.single.bodyA, ended.single.bodyB}, {floor, other});
      },
    );

    test('a sensor reports a body entering and leaving, and stops nothing', () {
      final w = _world();
      final zone = _box(
        w,
        Vector3(0, -3, 0),
        half: 1,
        type: BodyType3D.static,
        sensor: true,
      );
      final ball = w.createBody(
        const SphereShape3D(0.25),
        BodySettings3D(position: Vector3(0, -6, 0)),
      );
      final events = <PhysicsEvent3D>[];
      for (var i = 0; i < 90; i++) {
        w.step(_dt);
        events.addAll(w.drainEvents());
      }
      final types = events.map((e) => e.type).toList();
      expect(
        types,
        containsAllInOrder([
          PhysicsEventType3D.sensorBegin,
          PhysicsEventType3D.sensorEnd,
        ]),
      );
      expect({events.first.bodyA, events.first.bodyB}, {zone, ball});
      expect(_position(w, ball).y, greaterThan(0), reason: 'fell through it');
    });

    test('stays and activation are only reported when asked for', () {
      final quiet = _world();
      _floor(quiet);
      _box(quiet, Vector3(0, -0.6, 0));
      final loud = _world(stay: true, activation: true);
      _floor(loud);
      _box(loud, Vector3(0, -0.6, 0));
      final quietTypes = <PhysicsEventType3D>{},
          loudTypes = <PhysicsEventType3D>{};
      for (var i = 0; i < 600; i++) {
        quiet.step(_dt);
        loud.step(_dt);
        quietTypes.addAll(quiet.drainEvents().map((e) => e.type));
        loudTypes.addAll(loud.drainEvents().map((e) => e.type));
      }
      expect(quietTypes, {PhysicsEventType3D.contactBegin});
      expect(
        loudTypes,
        containsAll([
          PhysicsEventType3D.contactBegin,
          PhysicsEventType3D.contactStay,
          PhysicsEventType3D.bodyActivated,
          PhysicsEventType3D.bodyDeactivated,
        ]),
      );
    });

    test('a thousand events in one step are all drained', () {
      final w = _world(gravity: Vector3.zero(), stay: true);
      // 40 boxes overlapping a sensor plate, and each other in pairs.
      _box(w, Vector3.zero(), half: 30, type: BodyType3D.static, sensor: true);
      for (var i = 0; i < 1100; i++) {
        w.createBody(
          const SphereShape3D(0.1),
          BodySettings3D(
            position: Vector3((i % 40) * 1.0 - 20, 0, (i ~/ 40) * 1.0 - 14),
          ),
        );
      }
      w.step(_dt);
      final events = w.drainEvents();
      expect(
        events.where((e) => e.type == PhysicsEventType3D.sensorBegin),
        hasLength(1100),
      );
    });
  });

  group('queries', () {
    test(
      'rays: closest, all, by layer, ignoring a body, sensors only when asked',
      () {
        final w = _world();
        final floor = _floor(w, layer: 1);
        final crate = _box(w, Vector3.zero(), type: BodyType3D.static);
        final zone = _box(
          w,
          Vector3(0, -3, 0),
          type: BodyType3D.static,
          sensor: true,
        );
        final origin = Vector3(0, -10, 0), down = Vector3(0, 20, 0);

        final closest = w.raycast(origin, down);
        expect(closest.single.body, crate);
        expect(closest.single.point.y, closeTo(-0.5, 1e-4));
        expect(
          closest.single.normal.y,
          closeTo(-1, 1e-4),
          reason: 'facing up, towards the ray',
        );
        expect(closest.single.fraction, closeTo(9.5 / 20, 1e-4));

        final all = w.raycast(origin, down, allHits: true);
        expect(all.map((h) => h.body), [crate, floor]);

        expect(
          w
              .raycast(origin, down, filter: const QueryFilter3D(layerMask: 2))
              .single
              .body,
          floor,
        );
        expect(
          w
              .raycast(origin, down, filter: QueryFilter3D(ignoreBody: crate))
              .single
              .body,
          floor,
        );
        expect(
          w
              .raycast(
                origin,
                down,
                filter: const QueryFilter3D(includeSensors: true),
              )
              .single
              .body,
          zone,
        );
        expect(w.raycast(origin, Vector3(0, -20, 0)), isEmpty);
      },
    );

    test(
      'a sphere sweep stops where the sphere touches; an overlap finds what it touches',
      () {
        final w = _world();
        final floor = _floor(w);
        final crate = _box(w, Vector3(0, -0.5, 0), type: BodyType3D.static);
        final hit = w.shapeCast(
          const SphereShape3D(0.5),
          Vector3(0, -10, 0),
          Quaternion.identity(),
          Vector3(0, 20, 0),
        );
        expect(hit.single.body, crate);
        expect(
          hit.single.fraction * 20,
          closeTo(8.5, 0.01),
          reason: 'the sphere stops 0.5 m above the crate',
        );
        expect(hit.single.normal.y, closeTo(-1, 0.01));

        final touching = w.overlap(
          const BoxShape3D(2, 0.1, 2),
          Vector3(0, -0.05, 0),
          Quaternion.identity(),
        );
        expect(touching.map((h) => h.body).toSet(), {floor, crate});
        expect(
          w.overlap(
            const SphereShape3D(0.2),
            Vector3(5, -5, 5),
            Quaternion.identity(),
          ),
          isEmpty,
        );
      },
    );
  });

  group('joints', () {
    test(
      'a hinge turns about its axis within its limits; its angle is read back',
      () {
        final w = _world(gravity: Vector3.zero());
        final door = w.createBody(
          const BoxShape3D(0.5, 1, 0.05),
          BodySettings3D(position: Vector3(0.5, 0, 0)),
        );
        final hinge = w.createJoint(
          door,
          0,
          JointSettings3D(
            kind: JointKind3D.hinge,
            pointA: Vector3.zero(),
            axis: Vector3(0, 1, 0),
            normal: Vector3(1, 0, 0),
            hasLimits: true,
            limitMin: -0.5,
            limitMax: 0.5,
          ),
        );
        expect(hinge, isNot(0));
        w.setAngularVelocity(door, Vector3(0, 3, 0));
        _run(w, 60);
        expect(w.jointValue(hinge).abs(), lessThanOrEqualTo(0.52));
        expect(
          w.jointValue(hinge).abs(),
          greaterThan(0.4),
          reason: 'pushed against a limit',
        );
        final p = _position(w, door);
        expect(
          math.sqrt(p.x * p.x + p.z * p.z),
          closeTo(0.5, 0.02),
          reason: 'still on the hinge',
        );
      },
    );

    test(
      'a velocity motor turns a hinge, a position motor drives a slider',
      () {
        final w = _world(gravity: Vector3.zero());
        final wheel = w.createBody(
          const CylinderShape3D(0.1, 0.5),
          BodySettings3D(position: Vector3.zero()),
        );
        final axle = w.createJoint(
          wheel,
          0,
          JointSettings3D(
            kind: JointKind3D.hinge,
            axis: Vector3(0, 1, 0),
            normal: Vector3(1, 0, 0),
            motorState: MotorState3D.velocity,
            motorTarget: 1,
          ),
        );
        final cart = _box(w, Vector3(0, 0, 5));
        final rail = w.createJoint(
          cart,
          0,
          JointSettings3D(
            kind: JointKind3D.slider,
            pointA: Vector3(0, 0, 5),
            axis: Vector3(1, 0, 0),
            normal: Vector3(0, 1, 0),
          ),
        );
        w.setJointMotor(rail, MotorState3D.position, 2);
        _run(w, 120);
        expect(
          w.jointValue(axle).abs(),
          closeTo(2, 0.2),
          reason: '1 rad/s for 2 s',
        );
        expect(w.jointValue(rail), closeTo(2, 0.05));
        expect(_position(w, cart).x, closeTo(2, 0.05));
        expect(
          _position(w, cart).z,
          closeTo(5, 1e-3),
          reason: 'only along its axis',
        );
      },
    );

    test(
      'a distance joint is a pendulum; fixed and point joints hold; a six-DOF joint frees one axis',
      () {
        final w = _world();
        final bob = w.createBody(
          const SphereShape3D(0.2),
          BodySettings3D(position: Vector3(2, -5, 0)),
        );
        w.createJoint(
          bob,
          0,
          JointSettings3D(
            kind: JointKind3D.distance,
            pointA: Vector3(2, -5, 0),
            pointB: Vector3(0, -5, 0),
            hasLimits: true,
            limitMin: -1,
            limitMax: -1,
          ),
        );
        final a = _box(w, Vector3(10, -5, 0)), b = _box(w, Vector3(11, -5, 0));
        w.createJoint(
          a,
          b,
          JointSettings3D(
            kind: JointKind3D.fixed,
            pointA: Vector3(10.5, -5, 0),
          ),
        );
        w.createJoint(
          a,
          0,
          JointSettings3D(kind: JointKind3D.point, pointA: Vector3(10, -6, 0)),
        );
        final slider = _box(w, Vector3(20, -5, 0));
        w.createJoint(
          slider,
          0,
          JointSettings3D(
            kind: JointKind3D.sixDof,
            pointA: Vector3(20, -5, 0),
            axis: Vector3(1, 0, 0),
            normal: Vector3(0, 1, 0),
            sixDofModes: [
              SixDofAxisMode3D.free,
              ...List.filled(5, SixDofAxisMode3D.fixed),
            ],
          ),
        );
        w.setLinearVelocity(slider, Vector3(1, 1, 1));
        _run(w, 120);
        final p = _position(w, bob);
        expect(p.distanceTo(Vector3(0, -5, 0)), closeTo(2, 0.02));
        expect(p.y, greaterThan(-4.5), reason: 'swung down');
        expect(_position(w, b).distanceTo(_position(w, a)), closeTo(1, 0.02));
        final s = _position(w, slider);
        expect(
          s.y,
          closeTo(-5, 0.01),
          reason: 'Y is fixed; gravity cannot move it',
        );
        expect(s.z, closeTo(0, 0.01));
        expect(s.x, greaterThan(21));
      },
    );

    test('a joint goes when a body it holds goes', () {
      final w = _world();
      final a = _box(w, Vector3.zero()), b = _box(w, Vector3(2, 0, 0));
      final j = w.createJoint(
        a,
        b,
        JointSettings3D(kind: JointKind3D.fixed, pointA: Vector3(1, 0, 0)),
      );
      expect(w.isJoint(j), isTrue);
      w.destroyBody(b);
      expect(w.isJoint(j), isFalse);
      expect(
        w.createJoint(a, b, JointSettings3D(kind: JointKind3D.fixed)),
        0,
        reason: 'a dead body',
      );
    });
  });

  test('a disposed world answers everything with nothing', () {
    final w = _backend.createWorld(PhysicsWorldSettings3D());
    final box = _box(w, Vector3.zero());
    w.dispose();
    w.dispose();
    expect(w.isDisposed, isTrue);
    expect(w.isBody(box), isFalse);
    expect(w.bodyCount, 0);
    w.step(_dt);
    expect(w.drainEvents(), isEmpty);
    expect(w.raycast(Vector3.zero(), Vector3(0, 1, 0)), isEmpty);
    expect(w.createBody(const SphereShape3D(1), BodySettings3D()), 0);
  });

  test('debug lines need the dev flavour', () {
    expect(_world().debugLines() == null, !jj_has_debug_renderer());
  });
}
