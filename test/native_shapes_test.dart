// Shapes the native backend used to make differently from the pure-Dart one:
// open chains (Box2D treats an open chain's first and last points as ghosts,
// and needs four points at least) and polygons with more than Box2D's eight
// vertices. Every case runs on both backends; the native one is skipped where
// the Box2D library is not built.

import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  final backends = <String, Physics2DEngine Function()>{
    'pure Dart': Physics2DEngine.pureDart,
    'Box2D': Box2DPhysicsEngine.new,
  };

  for (final MapEntry(key: name, value: make) in backends.entries) {
    group(name, () {
      late Physics2DEngine engine;
      setUp(() => engine = make()..initialize());
      tearDown(() => engine.dispose());

      bool unavailable() =>
          name == 'Box2D' && engine.stats['backend'] != 'box2d_v3';

      /// Where a ball dropped at [x] onto [ground] comes to rest (+Y is down;
      /// the ground's line is at y = 0).
      double restY(Shape2D ground, double x) {
        engine.addBody(
          Physics2DBody(position: Vector2(0, 0), shape: ground, mass: 0),
        );
        final ball = Physics2DBody(
          position: Vector2(x, -40),
          shape: Circle2DShape(5),
          restitution: 0,
        );
        engine.addBody(ball);
        for (var i = 0; i < 120; i++) {
          engine.update(1 / 60);
        }
        return ball.position.y;
      }

      test('a ball lands on the end segments of an open chain', () {
        if (unavailable()) return markTestSkipped('native Box2D unavailable');
        final chain = Chain2DShape(const [
          Offset(-150, 0),
          Offset(-50, 0),
          Offset(50, 0),
          Offset(150, 0),
        ]);
        expect(restY(chain, -100), closeTo(-5, 2));
      });

      test('a ball lands on the last segment of an open chain', () {
        if (unavailable()) return markTestSkipped('native Box2D unavailable');
        final chain = Chain2DShape(const [
          Offset(-150, 0),
          Offset(-50, 0),
          Offset(50, 0),
          Offset(150, 0),
        ]);
        expect(restY(chain, 100), closeTo(-5, 2));
      });

      test('a two-point chain is solid', () {
        if (unavailable()) return markTestSkipped('native Box2D unavailable');
        final chain = Chain2DShape(const [Offset(-100, 0), Offset(100, 0)]);
        expect(restY(chain, 0), closeTo(-5, 2));
      });

      test('a polygon of 12 vertices keeps every one', () {
        if (unavailable()) return markTestSkipped('native Box2D unavailable');
        const r = 50.0;
        final dodecagon = Polygon2DShape([
          for (var k = 0; k < 12; k++)
            Offset(
              r * math.cos(k * math.pi / 6),
              r * math.sin(k * math.pi / 6),
            ),
        ]);
        engine.addBody(
          Physics2DBody(position: Vector2(0, 0), shape: dodecagon, mass: 0),
        );
        engine.update(1 / 60);
        // Along 285°: past the eighth vertex, so outside the hull of the
        // first eight and inside the dodecagon, whose edge is r·cos 15° out.
        const angle = 285 * math.pi / 180;
        final out = Offset(math.cos(angle), math.sin(angle));
        final hit = engine.raycast(Ray2D.fromPoints(out * 200, Offset.zero));
        expect(hit, isNotNull);
        expect(hit!.point.distance, closeTo(r * math.cos(math.pi / 12), 1.5));
      });
    });
  }
}
