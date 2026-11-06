// Events, queries, the collision grid and the packed reads, through the
// Physics3DWorld interface, on Jolt and on the fake engine that other
// packages' tests stand on: both must answer alike.

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/physics_3d.dart';
import 'package:just_physics_engine/testing.dart';

const _dt = 1 / 60;
late Physics3DEngine _backend;

Physics3DWorld _world({
  List<int>? masks,
  bool stay = false,
  bool activation = false,
  Vector3? gravity,
}) {
  final world = _backend.createWorld(
    Physics3DWorldSettings(
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
int _floor(Physics3DWorld w, {int layer = 0}) => w.createBody(
  const Box3DShape(10, 0.5, 10),
  Body3DSettings(
    position: Vector3(0, 0.5, 0),
    type: Body3DType.static,
    layer: layer,
  ),
);

int _box(
  Physics3DWorld w,
  Vector3 at, {
  double half = 0.5,
  Body3DType type = Body3DType.dynamic,
  int layer = 0,
  bool sensor = false,
}) => w.createBody(
  Box3DShape(half, half, half),
  Body3DSettings(position: at, type: type, layer: layer, isSensor: sensor),
);

Vector3 _position(Physics3DWorld w, int body) {
  final poses = Body3DPoses();
  w.readBodies([body], poses);
  final p = Vector3.zero();
  poses.positionInto(0, p);
  return p;
}

void _run(Physics3DWorld w, int steps) {
  for (var i = 0; i < steps; i++) {
    w.step(_dt);
  }
}

void main() {
  for (final (name, make) in <(String, Physics3DEngine Function())>[
    ('jolt', PhysicsEngineFactory.create3D),
    ('fake', FakePhysics3DEngine.new),
  ]) {
    group(name, () {
      setUp(() => _backend = make());

      group('events', () {
        test(
          'one contact begin when a box lands and one end when it leaves',
          () {
            final w = _world();
            final floor = _floor(w);
            final box = _box(w, Vector3(0, -2, 0));
            final events = <Physics3DEvent>[];
            for (var i = 0; i < 90; i++) {
              w.step(_dt);
              events.addAll(w.drainEvents());
            }
            final begins = events
                .where((e) => e.type == Physics3DEventType.contactBegin)
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
              events.where((e) => e.type == Physics3DEventType.contactEnd),
              hasLength(1),
            );
          },
        );

        test(
          'a body falling asleep on the floor still touches it; leaving or going ends it',
          () {
            final w = _world();
            final floor = _floor(w);
            final box = _box(w, Vector3(0, -1, 0));
            final types = <Physics3DEventType>[];
            for (var i = 0; i < 600; i++) {
              w.step(_dt);
              types.addAll(w.drainEvents().map((e) => e.type));
            }
            expect(w.isAwake(box), isFalse, reason: 'asleep by now');
            expect(types, [
              Physics3DEventType.contactBegin,
            ], reason: 'no end for falling asleep');

            // Woken where it lies: still touching, nothing new.
            w.wake(box);
            _run(w, 3);
            expect(w.drainEvents(), isEmpty);
            // Thrown up: it ends.
            w.setLinearVelocity(box, Vector3(0, -6, 0));
            _run(w, 10);
            expect(w.drainEvents().map((e) => e.type), [
              Physics3DEventType.contactEnd,
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
                .where((e) => e.type == Physics3DEventType.contactEnd)
                .toList();
            expect(ended, hasLength(1));
            expect({ended.single.bodyA, ended.single.bodyB}, {floor, other});
          },
        );

        test(
          'a sensor reports a body entering and leaving, and stops nothing',
          () {
            final w = _world();
            final zone = _box(
              w,
              Vector3(0, -3, 0),
              half: 1,
              type: Body3DType.static,
              sensor: true,
            );
            final ball = w.createBody(
              const Sphere3DShape(0.25),
              Body3DSettings(position: Vector3(0, -6, 0)),
            );
            final events = <Physics3DEvent>[];
            for (var i = 0; i < 90; i++) {
              w.step(_dt);
              events.addAll(w.drainEvents());
            }
            final types = events.map((e) => e.type).toList();
            expect(
              types,
              containsAllInOrder([
                Physics3DEventType.sensorBegin,
                Physics3DEventType.sensorEnd,
              ]),
            );
            expect({events.first.bodyA, events.first.bodyB}, {zone, ball});
            expect(
              _position(w, ball).y,
              greaterThan(0),
              reason: 'fell through it',
            );
          },
        );

        test('stays and activation are only reported when asked for', () {
          final quiet = _world();
          _floor(quiet);
          _box(quiet, Vector3(0, -0.6, 0));
          final loud = _world(stay: true, activation: true);
          _floor(loud);
          _box(loud, Vector3(0, -0.6, 0));
          final quietTypes = <Physics3DEventType>{},
              loudTypes = <Physics3DEventType>{};
          for (var i = 0; i < 600; i++) {
            quiet.step(_dt);
            loud.step(_dt);
            quietTypes.addAll(quiet.drainEvents().map((e) => e.type));
            loudTypes.addAll(loud.drainEvents().map((e) => e.type));
          }
          expect(quietTypes, {Physics3DEventType.contactBegin});
          expect(
            loudTypes,
            containsAll([
              Physics3DEventType.contactBegin,
              Physics3DEventType.contactStay,
              Physics3DEventType.bodyActivated,
              Physics3DEventType.bodyDeactivated,
            ]),
          );
        });

        test('a thousand events in one step are all drained', () {
          final w = _world(gravity: Vector3.zero(), stay: true);
          // 40 boxes overlapping a sensor plate, and each other in pairs.
          _box(
            w,
            Vector3.zero(),
            half: 30,
            type: Body3DType.static,
            sensor: true,
          );
          for (var i = 0; i < 1100; i++) {
            w.createBody(
              const Sphere3DShape(0.1),
              Body3DSettings(
                position: Vector3((i % 40) * 1.0 - 20, 0, (i ~/ 40) * 1.0 - 14),
              ),
            );
          }
          w.step(_dt);
          final events = w.drainEvents();
          expect(
            events.where((e) => e.type == Physics3DEventType.sensorBegin),
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
            final crate = _box(w, Vector3.zero(), type: Body3DType.static);
            final zone = _box(
              w,
              Vector3(0, -3, 0),
              type: Body3DType.static,
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
                  .raycast(
                    origin,
                    down,
                    filter: const Query3DFilter(layerMask: 2),
                  )
                  .single
                  .body,
              floor,
            );
            expect(
              w
                  .raycast(
                    origin,
                    down,
                    filter: Query3DFilter(ignoreBody: crate),
                  )
                  .single
                  .body,
              floor,
            );
            expect(
              w
                  .raycast(
                    origin,
                    down,
                    filter: const Query3DFilter(includeSensors: true),
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
            final crate = _box(w, Vector3(0, -0.5, 0), type: Body3DType.static);
            final hit = w.shapeCast(
              const Sphere3DShape(0.5),
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
              const Box3DShape(2, 0.1, 2),
              Vector3(0, -0.05, 0),
              Quaternion.identity(),
            );
            expect(touching.map((h) => h.body).toSet(), {floor, crate});
            expect(
              w.overlap(
                const Sphere3DShape(0.2),
                Vector3(5, -5, 5),
                Quaternion.identity(),
              ),
              isEmpty,
            );
          },
        );
      });

      group('layers', () {
        test('a new grid takes effect on bodies at rest, and back', () {
          final w = _world();
          final floor = _floor(w, layer: 1);
          final box = _box(w, Vector3(0, -1, 0), layer: 2);
          expect(w.layerMasks, everyElement(0xFFFF));
          _run(w, 120);
          expect(_position(w, box).y, closeTo(-0.5, 0.05), reason: 'on it');
          w.drainEvents();

          // Layers 1 and 2 no longer collide: the box falls through, asleep
          // or not, and its contact ends.
          final apart = [for (var i = 0; i < 16; i++) 0xFFFF];
          apart[1] &= ~(1 << 2);
          apart[2] &= ~(1 << 1);
          w.setLayerMasks(apart);
          expect(w.layerMasks[1], 0xFFFF & ~(1 << 2));
          expect(w.layerMasks[2], 0xFFFF & ~(1 << 1));
          _run(w, 60);
          expect(_position(w, box).y, greaterThan(2), reason: 'through it');
          final ended = w
              .drainEvents()
              .where((e) => e.type == Physics3DEventType.contactEnd)
              .toList();
          expect(ended, hasLength(1));
          expect({ended.single.bodyA, ended.single.bodyB}, {floor, box});

          // Back: another box lands again.
          w.setLayerMasks(const []);
          expect(w.layerMasks, everyElement(0xFFFF), reason: 'missing rows');
          final second = _box(w, Vector3(3, -1, 0), layer: 2);
          _run(w, 120);
          expect(_position(w, second).y, closeTo(-0.5, 0.05));
        });

        test(
          'the same grid again changes nothing, bits past 16 are dropped',
          () {
            final w = _world(masks: [0x1FFFF, 0xFFFE]);
            expect(w.layerMasks[0], 0xFFFF);
            expect(w.layerMasks[1], 0xFFFE);
            _floor(w);
            final box = _box(w, Vector3(0, -1, 0));
            _run(w, 600);
            expect(w.isAwake(box), isFalse, reason: 'asleep on the floor');
            w.setLayerMasks([0xFFFF, 0xFFFE]);
            expect(
              w.isAwake(box),
              isFalse,
              reason: 'nothing changed, nothing woke',
            );
          },
        );
      });

      group('packed reads', () {
        test('readEvents gives what drainEvents would, without objects', () {
          final w = _world();
          final floor = _floor(w);
          final box = _box(w, Vector3(0, -2, 0));
          final read = Physics3DEvents();
          var begins = 0;
          for (var i = 0; i < 90; i++) {
            w
              ..step(_dt)
              ..readEvents(read);
            for (var k = 0; k < read.count; k++) {
              if (read.typeOf(k) != Physics3DEventType.contactBegin) continue;
              begins++;
              expect({read.bodyAOf(k), read.bodyBOf(k)}, {floor, box});
              final normal = Vector3.zero();
              read.normalInto(k, normal);
              expect(normal.y.abs(), closeTo(1, 0.01));
              expect(read.eventAt(k).type, Physics3DEventType.contactBegin);
            }
          }
          expect(begins, 1);
          w.readEvents(read);
          expect(read.count, 0, reason: 'read once');
          w.dispose();
          w.readEvents(read);
          expect(read.count, 0);
        });

        test('readCharacterContacts gives what characterContacts would', () {
          final w = _world();
          final floor = _floor(w);
          final c = w.createCharacter(
            Character3DSettings(),
            Vector3(0, -0.1, 0),
          );
          final state = Character3DState();
          for (var i = 0; i < 10; i++) {
            w.moveCharacter(
              c,
              _dt,
              Vector3(0, 1, 0),
              state,
              stepDown: Vector3(0, 0.5, 0),
              stepUp: Vector3(0, -0.4, 0),
            );
          }
          final read = Character3DContacts();
          w.readCharacterContacts(c, read);
          final objects = w.characterContacts(c);
          expect(read.count, objects.length);
          expect(read.count, greaterThan(0));
          final k = [
            for (var i = 0; i < read.count; i++) read.bodyOf(i),
          ].indexOf(floor);
          expect(k, isNot(-1), reason: 'it stands on the floor');
          final normal = Vector3.zero();
          read.normalInto(k, normal);
          expect(normal.y, closeTo(-1, 0.05), reason: 'up, towards it');
          expect(read.isSensor(k), isFalse);
          w.readCharacterContacts(0, read);
          expect(read.count, 0, reason: 'no character');
        });
      });
    });
  }
}
