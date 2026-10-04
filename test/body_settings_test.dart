// A circle off its body's centre, spin damping and sleep, on both backends:
// the platform's own (Box2D where it loads) and the pure-Dart one the web
// runs.

import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  final backends = <(String, PhysicsEngine Function())>[
    ('platform backend', PhysicsEngine.new),
    ('pure-Dart backend', PhysicsEngine.pureDart),
  ];

  for (final (name, make) in backends) {
    group(name, () {
      late PhysicsEngine engine;
      setUp(() => engine = make()..initialize());
      tearDown(() => engine.dispose());

      // A floor whose top is at y = 0.
      void floor() => engine.addBody(
        PhysicsBody(
          position: Vector2(0, 16),
          shape: RectangleShape(400, 32),
          mass: 0,
          bodyType: BodyType.static,
          restitution: 0,
        ),
      );

      void run(double seconds) {
        for (var t = 0.0; t < seconds; t += 1 / 60) {
          engine.update(1 / 60);
        }
      }

      test('an off-centre circle rests on its centre, not the body\'s', () {
        floor();
        final ball = PhysicsBody(
          position: Vector2(0, -100),
          shape: CircleShape(8, center: const Offset(0, 20)),
          restitution: 0,
          drag: 0,
          fixedRotation: true,
        );
        engine.addBody(ball);
        run(2);
        // The circle's bottom (body + 20 + 8) is on the floor's top.
        expect(ball.position.y, closeTo(-28, 1.5));
      });

      test('angular damping slows the spin, and drag leaves it alone', () {
        PhysicsBody spinner({double drag = 0, double angularDamping = 0}) =>
            PhysicsBody(
              position: Vector2(0, -500),
              shape: CircleShape(10),
              useGravity: false,
              drag: drag,
              angularDamping: angularDamping,
              angularVelocity: 10,
            );
        final damped = spinner(angularDamping: 2);
        final dragged = spinner(drag: 5);
        engine
          ..addBody(damped)
          ..addBody(dragged);
        run(1);
        expect(damped.angularVelocity, inExclusiveRange(0.5, 3));
        expect(dragged.angularVelocity, closeTo(10, 0.05));
      });

      test('a body that cannot sleep stays awake at rest', () {
        // Weightless and still: the pure-Dart backend counts gravity as
        // something acting on a body, so only these ever sleep there.
        PhysicsBody box({required bool canSleep}) => PhysicsBody(
          position: Vector2(canSleep ? -100 : 100, -10),
          shape: RectangleShape(20, 20),
          useGravity: false,
          canSleep: canSleep,
        );
        final sleepy = box(canSleep: true);
        final restless = box(canSleep: false);
        engine
          ..addBody(sleepy)
          ..addBody(restless);
        run(4);
        expect(sleepy.isAwake, isFalse);
        expect(restless.isAwake, isTrue);

        // Forbidding sleep later wakes it.
        engine.setBodyCanSleep(sleepy, false);
        run(2);
        expect(sleepy.isAwake, isTrue);
        expect(sleepy.canSleep, isFalse);
      });

      test('continuous collision can be switched at runtime', () {
        final b = PhysicsBody(position: Vector2.zero(), shape: CircleShape(4));
        engine.addBody(b);
        engine.setBodyBullet(b, true);
        expect(b.isBullet, isTrue);
        engine.setBodyBullet(b, false);
        expect(b.isBullet, isFalse);
      });
    });
  }
}
