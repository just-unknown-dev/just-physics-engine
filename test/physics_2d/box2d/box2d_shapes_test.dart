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

      bool unavailable() => name == 'Box2D' && !engine.isNative;

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

      // Box2D welds a polygon's points closer than 2 units (four linear
      // slops) and makes no shape from fewer than three, so a polygon
      // thinner than that, or a box rounded by half its side, once had no
      // shape on the native backend: rays and bodies went through it.

      /// Where a ray straight down x = [x] first meets a static body of
      /// [shape] at the origin; null when it meets nothing.
      double? hitY(Shape2D shape, {double x = 0}) {
        engine.addBody(
          Physics2DBody(position: Vector2(0, 0), shape: shape, mass: 0),
        );
        engine.update(1 / 60);
        final hit = engine.raycast(
          Ray2D.fromPoints(Offset(x, -100), Offset(x, 100)),
        );
        return hit?.point.dy;
      }

      /// A 150 × 14 box rounded by [r], as the game engine builds one: its
      /// corners' centres on a box 2·[r] smaller.
      RoundedPolygon2DShape plank(double r) => RoundedPolygon2DShape.rect(
        width: 150 - 2 * r,
        height: 14 - 2 * r,
        cornerRadius: r,
      );

      test('a box rounded by just under half its height is solid', () {
        if (unavailable()) return markTestSkipped('native Box2D unavailable');
        // The game engine keeps a corner radius 0.001 under half the box.
        final shape = plank(7 - 1e-3);
        expect(hitY(shape), closeTo(-7, 0.1));
        expect(
          engine
              .raycast(
                Ray2D.fromPoints(const Offset(60, -100), const Offset(60, 100)),
              )
              ?.point
              .dy,
          closeTo(-7, 0.1),
        );
      });

      test('a box rounded by half its height is solid', () {
        if (unavailable()) return markTestSkipped('native Box2D unavailable');
        expect(hitY(plank(7)), closeTo(-7, 0.1));
      });

      test('a ball lands on a box rounded by half its height', () {
        if (unavailable()) return markTestSkipped('native Box2D unavailable');
        expect(restY(plank(7 - 1e-3), 0), closeTo(-12, 2));
      });

      test('a box rounded by under half its height is unchanged', () {
        if (unavailable()) return markTestSkipped('native Box2D unavailable');
        expect(hitY(plank(6)), closeTo(-7, 0.1));
      });

      test('a rounded box keeps its length', () {
        if (unavailable()) return markTestSkipped('native Box2D unavailable');
        final shape = plank(7 - 1e-3);
        engine.addBody(
          Physics2DBody(position: Vector2(0, 0), shape: shape, mass: 0),
        );
        engine.update(1 / 60);
        final hit = engine.raycast(
          Ray2D.fromPoints(const Offset(-200, 0), const Offset(200, 0)),
        );
        expect(hit?.point.dx, closeTo(-75, 0.1));
      });

      test('a rounded polygon smaller than 2 units is a circle', () {
        if (unavailable()) return markTestSkipped('native Box2D unavailable');
        final shape = RoundedPolygon2DShape(const [
          Offset(-0.5, 0.5),
          Offset(0.5, 0.5),
          Offset(0, -0.5),
        ], 10);
        // Grown by 10 round its top corner, 0.5 up.
        expect(hitY(shape), closeTo(-10.5, 0.6));
      });

      test('a thin rounded triangle is solid', () {
        if (unavailable()) return markTestSkipped('native Box2D unavailable');
        final shape = RoundedPolygon2DShape(const [
          Offset(-60, 0),
          Offset(60, 0),
          Offset(0, -1),
        ], 4);
        expect(hitY(shape, x: 30), closeTo(-4.5, 0.6));
      });

      test('a polygon thinner than 2 units is solid', () {
        if (unavailable()) return markTestSkipped('native Box2D unavailable');
        final shape = Polygon2DShape(const [
          Offset(-60, -0.5),
          Offset(60, -0.5),
          Offset(60, 0.5),
          Offset(-60, 0.5),
        ]);
        expect(hitY(shape, x: 30), closeTo(-0.5, 0.1));
      });

      test('a capsule as wide as it is long is a circle', () {
        if (unavailable()) return markTestSkipped('native Box2D unavailable');
        final shape = Capsule2DShape(
          center1: Offset.zero,
          center2: Offset.zero,
          radius: 10,
        );
        expect(hitY(shape), closeTo(-10, 0.1));
      });
    });
  }
}
