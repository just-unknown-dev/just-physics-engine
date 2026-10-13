// The character mover: Box2D's GJK and shape cast in Dart, its plane
// solver, and a character walking, landing, climbing and passing through
// one-way platforms — on both backends, which must agree.

import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

const _dt = 1 / 60;
const _gravity = 2000.0;

/// A character driven the way a platformer drives one.
class _Walker {
  _Walker(this.engine, this.shape, this.position);

  final PhysicsEngine engine;
  final MoverShape shape;
  late final mover = CharacterMover2D(engine);
  Offset position;
  Offset velocity = Offset.zero;
  bool grounded = false;
  GroundState2D state = GroundState2D.inAir;
  PhysicsBody? ground;

  void step({
    double moveX = 0,
    double? jump,
    double stepHeight = 0,
    double stick = 0,
    MoverFilter filter = const MoverFilter(),
    double push = 0,
  }) {
    var vy = velocity.dy;
    if (grounded && vy > 0) vy = 0;
    if (jump != null) vy = -jump;
    vy += _gravity * _dt;
    final r = mover.move(
      shape: shape,
      position: position,
      velocity: Offset(moveX, vy),
      dt: _dt,
      stepHeight: stepHeight,
      stickToFloor: stick,
      wasGrounded: grounded,
      filter: filter,
      mass: 70,
      pushStrength: push,
    );
    position = r.position;
    velocity = r.velocity;
    grounded = r.isGrounded;
    state = r.groundState;
    ground = r.groundBody;
    engine.update(_dt);
  }

  void run(int steps, {double moveX = 0, double stepHeight = 0, double stick = 0, double push = 0}) {
    for (var i = 0; i < steps; i++) {
      step(moveX: moveX, stepHeight: stepHeight, stick: stick, push: push);
    }
  }
}

PhysicsBody _static(Offset at, CollisionShape shape, {bool oneWay = false, bool sensor = false}) => PhysicsBody(
  position: Vector2(at.dx, at.dy),
  shape: shape,
  mass: 0,
  bodyType: BodyType.static,
  isOneWay: oneWay,
  isSensor: sensor,
);

void main() {
  group('GJK and shape cast', () {
    test('the distance between two cores, and its normal from A to B', () {
      final d = ShapeDistance2D.between(
        const ShapeProxy2D([Offset.zero], 1),
        const ShapeProxy2D([Offset(5, 0), Offset(5, 4)], 1),
      );
      expect(d.distance, closeTo(5, 1e-9));
      expect(d.normal.dx, closeTo(1, 1e-9));
      expect(d.normal.dy, closeTo(0, 1e-9));
      expect(d.pointB.dy, closeTo(0, 1e-9));
    });

    test('a point inside a box overlaps it', () {
      final d = ShapeDistance2D.between(
        const ShapeProxy2D([Offset(-2, -2), Offset(2, -2), Offset(2, 2), Offset(-2, 2)], 0),
        const ShapeProxy2D([Offset(0.5, 0.25)], 0),
      );
      expect(d.overlaps, isTrue);
    });

    test('a circle swept at a wall stops a slop short of it', () {
      final hit = castShape2D(
        const ShapeProxy2D([Offset(10, -5), Offset(10, 5)], 0),
        const ShapeProxy2D([Offset.zero], 2),
        const Offset(20, 0),
        linearSlop: moverLinearSlop,
      )!;
      // Box2D lets the surfaces meet a slop deep: the circle's centre stops
      // at 10 - (2 - 0.5), a fraction 8.5 / 20 of the way.
      expect(hit.fraction, closeTo(8.5 / 20, 1e-6));
      expect(hit.normal.dx, closeTo(-1, 1e-9));
    });

    test('moving away is a miss', () {
      expect(
        castShape2D(
          const ShapeProxy2D([Offset(10, 0)], 1),
          const ShapeProxy2D([Offset.zero], 1),
          const Offset(-5, 0),
          linearSlop: moverLinearSlop,
        ),
        isNull,
      );
    });
  });

  test('the plane solver keeps out of a floor and clips the velocity', () {
    final floor = MoverPlane(normal: const Offset(0, -1), offset: 0, point: Offset.zero);
    final r = MoverSolver.solvePlanes(const Offset(5, 10), [floor]);
    expect(r.translation.dx, closeTo(5, 1e-9));
    // Box2D's solver leaves the mover a slop into the surface, so it keeps
    // touching what it stands on.
    expect(r.translation.dy, closeTo(moverLinearSlop, 1e-9));
    final v = MoverSolver.clipVector(const Offset(300, 400), [floor]);
    expect(v, const Offset(300, 0));
  });

  test('both backends walk the same course alike', () {
    final native = PhysicsEngine()..initialize();
    addTearDown(native.dispose);
    if (native.stats['backend'] != 'box2d_v3') {
      markTestSkipped('native Box2D backend unavailable');
      return;
    }
    final dart = PhysicsEngine.pureDart()..initialize();
    addTearDown(dart.dispose);
    List<Offset> walk(PhysicsEngine engine) {
      engine
        ..addBody(_static(const Offset(0, 120), RectangleShape(4000, 40)))
        ..addBody(_static(const Offset(300, 94), RectangleShape(400, 12)))
        ..addBody(_static(
          const Offset(500, 88),
          PolygonShape([const Offset(0, 0), const Offset(150, 0), const Offset(150, -60)]),
        ))
        ..addBody(_static(const Offset(250, 20), RectangleShape(120, 10), oneWay: true));
      final w = _Walker(engine, MoverShape.box(width: 24, height: 48), const Offset(0, 40));
      final trail = <Offset>[];
      for (var i = 0; i < 240; i++) {
        w.step(moveX: 220, jump: i == 60 ? 900 : null, stepHeight: 16, stick: 8);
        if (i % 20 == 0) trail.add(w.position);
      }
      return trail;
    }

    final a = walk(dart), b = walk(native);
    for (var i = 0; i < a.length; i++) {
      expect((a[i] - b[i]).distance, lessThan(1.5), reason: 'sample $i: ${a[i]} vs ${b[i]}');
    }
  });

  for (final (name, make) in [
    ('pure Dart', () => PhysicsEngine.pureDart()..initialize()),
    ('native Box2D', () => PhysicsEngine()..initialize()),
  ]) {
    group(name, () {
      late PhysicsEngine engine;
      setUp(() => engine = make());
      tearDown(() => engine.dispose());

      bool skipped() {
        if (name == 'native Box2D' && engine.stats['backend'] != 'box2d_v3') {
          markTestSkipped('native Box2D backend unavailable');
          return true;
        }
        return false;
      }

      /// A long floor whose top is at y = 100.
      PhysicsBody floor() {
        final f = _static(const Offset(0, 120), RectangleShape(4000, 40));
        engine.addBody(f);
        return f;
      }

      final capsule = MoverShape.capsule(radius: 12, height: 48);

      test('a capsule falls, lands and stands', () {
        if (skipped()) return;
        final f = floor();
        final w = _Walker(engine, capsule, const Offset(0, 0))..run(90);
        expect(w.grounded, isTrue);
        expect(w.ground, same(f));
        expect(w.position.dy, closeTo(76, 1.5), reason: 'feet on the floor');
        expect(w.velocity.dy, closeTo(0, 1));
        final rest = w.position;
        w.run(60);
        expect((w.position - rest).distance, lessThan(0.5), reason: 'it stays put');
      });

      test('it runs along the floor at the speed asked', () {
        if (skipped()) return;
        floor();
        final w = _Walker(engine, capsule, const Offset(0, 70))..run(30);
        final start = w.position.dx;
        w.run(60, moveX: 300);
        expect(w.position.dx - start, closeTo(300, 6));
        expect(w.grounded, isTrue);
      });

      test('a wall stops it', () {
        if (skipped()) return;
        floor();
        engine.addBody(_static(const Offset(200, 0), RectangleShape(40, 200)));
        final w = _Walker(engine, capsule, const Offset(0, 70))..run(120, moveX: 400);
        expect(w.position.dx, closeTo(180 - 12, 1.5), reason: 'against the wall');
        expect(w.velocity.dx, closeTo(0, 1));
      });

      test('it walks up a gentle slope and slides on a steep one', () {
        if (skipped()) return;
        floor();
        // A 30° ramp rising to the right from x = 100.
        final rise = 200 * math.tan(math.pi / 6);
        engine.addBody(_static(
          const Offset(100, 100),
          PolygonShape([const Offset(0, 0), const Offset(200, 0), Offset(200, -rise)]),
        ));
        final w = _Walker(engine, capsule, const Offset(0, 70))..run(20);
        w.run(60, moveX: 200, stick: 8);
        expect(w.position.dy, lessThan(40), reason: 'it climbed');
        expect(w.grounded, isTrue);

        // A 70° wall-like slope is not ground.
        final e2 = PhysicsEngine.pureDart()..initialize();
        addTearDown(e2.dispose);
        e2.addBody(_static(const Offset(0, 120), RectangleShape(4000, 40)));
        final steep = 100 * math.tan(70 * math.pi / 180);
        e2.addBody(_static(
          const Offset(100, 100),
          PolygonShape([const Offset(0, 0), const Offset(100, 0), Offset(100, -steep)]),
        ));
        final w2 = _Walker(e2, capsule, const Offset(0, 70))..run(20);
        w2.run(60, moveX: 200);
        expect(w2.position.dy, greaterThan(40), reason: 'too steep to climb');
      });

      test('dropped on a slope too steep to stand on, it slides down', () {
        if (skipped()) return;
        floor();
        final steep = 100 * math.tan(70 * math.pi / 180);
        engine.addBody(_static(
          const Offset(100, 100),
          PolygonShape([const Offset(0, 0), const Offset(100, 0), Offset(100, -steep)]),
        ));
        final w = _Walker(engine, capsule, const Offset(150, -200))..run(120);
        expect(w.position.dx, lessThan(100), reason: 'slid off to the foot');
        expect(w.grounded, isTrue);
      });

      test('a step is climbed up to stepHeight and no higher', () {
        if (skipped()) return;
        floor();
        // A 12-unit step from x = 100.
        engine.addBody(_static(const Offset(300, 94), RectangleShape(400, 12)));
        final blocked = _Walker(engine, MoverShape.box(width: 24, height: 48), const Offset(0, 70))..run(20);
        blocked.run(60, moveX: 200);
        expect(blocked.position.dx, lessThan(100), reason: 'no step height: blocked');

        final climber = _Walker(engine, MoverShape.box(width: 24, height: 48), const Offset(0, 70))..run(20);
        climber.run(60, moveX: 200, stepHeight: 16, stick: 8);
        expect(climber.position.dx, greaterThan(150), reason: 'stepped up');
        expect(climber.position.dy, closeTo(64, 2), reason: 'standing on the step');
        expect(climber.grounded, isTrue);
      });

      test('a ceiling stops a jump', () {
        if (skipped()) return;
        floor();
        engine.addBody(_static(const Offset(0, 0), RectangleShape(400, 20)));
        final w = _Walker(engine, capsule, const Offset(0, 70))..run(20);
        w.step(jump: 900);
        for (var i = 0; i < 5; i++) {
          w.step();
        }
        expect(w.position.dy, greaterThan(10 + 24 - 1), reason: 'under the ceiling');
        expect(w.velocity.dy, greaterThanOrEqualTo(0), reason: 'no speed up into it');
      });

      test('it jumps up through a one-way platform and lands on it', () {
        if (skipped()) return;
        floor();
        engine.addBody(_static(const Offset(0, 20), RectangleShape(300, 10), oneWay: true));
        final w = _Walker(engine, capsule, const Offset(0, 70))..run(20);
        w.step(jump: 1000);
        w.run(90);
        expect(w.grounded, isTrue);
        expect(w.position.dy, closeTo(15 - 24, 1.5), reason: 'standing on the platform');
      });

      test('sensors and its own body are passed over', () {
        if (skipped()) return;
        floor();
        engine.addBody(_static(const Offset(0, 60), RectangleShape(100, 20), sensor: true));
        final own = _static(const Offset(0, -100), CapsuleShape(center1: const Offset(0, -12), center2: const Offset(0, 12), radius: 12));
        engine.addBody(own);
        final w = _Walker(engine, capsule, const Offset(0, -100));
        for (var i = 0; i < 90; i++) {
          w.step(filter: MoverFilter(ignore: own));
        }
        expect(w.position.dy, closeTo(76, 1.5), reason: 'fell through both to the floor');
      });

      test('it keeps to the floor running down a slope', () {
        if (skipped()) return;
        floor();
        final rise = 200 * math.tan(math.pi / 9); // 20°, falling to the right
        engine.addBody(_static(
          const Offset(-200, 100),
          PolygonShape([const Offset(0, 0), Offset(0, -rise), const Offset(200, 0)]),
        ));
        final w = _Walker(engine, capsule, Offset(-190, 100 - rise - 26))..run(20, stick: 8);
        expect(w.grounded, isTrue);
        var airborne = 0;
        for (var i = 0; i < 40; i++) {
          w.step(moveX: 250, stick: 8);
          if (!w.grounded) airborne++;
        }
        expect(airborne, lessThanOrEqualTo(2), reason: 'not skipping down it');
      });

      test('it pushes a crate', () {
        if (skipped()) return;
        floor();
        final crate = PhysicsBody(position: Vector2(80, 84), shape: RectangleShape(32, 32), drag: 0, fixedRotation: true);
        engine.addBody(crate);
        final w = _Walker(engine, capsule, const Offset(0, 70))..run(20, push: 2000);
        w.run(90, moveX: 150, push: 2000);
        expect(crate.position.x, greaterThan(100), reason: 'the crate moved');
      });

      test('a box stands on a ledge corner a capsule slips off', () {
        if (skipped()) return;
        // A ledge ending at x = 0, top at y = 100.
        engine.addBody(_static(const Offset(-200, 120), RectangleShape(400, 40)));
        // Centre 8 units past the edge: a 24-wide box still has 4 on it.
        final box = _Walker(engine, MoverShape.box(width: 24, height: 48, cornerRadius: 2), const Offset(8, 70))..run(60);
        expect(box.grounded, isTrue);
        expect(box.position.dx, closeTo(8, 0.5));
        final cap = _Walker(engine, capsule, const Offset(8, 70))..run(60);
        expect(cap.position.dy, greaterThan(100), reason: 'rolled off and fell');
      });
    });
  }
}
