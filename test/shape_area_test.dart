// How much ground each shape covers: what a body whose mass is left at 0 is
// weighed by in just_game_engine.

import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  test('a circle is pi r squared, wherever its centre is', () {
    expect(CircleShape(10).area, closeTo(math.pi * 100, 1e-9));
    expect(
      CircleShape(10, center: const Offset(5, -3)).area,
      closeTo(math.pi * 100, 1e-9),
    );
  });

  test('a rectangle and a polygon, either way round', () {
    expect(RectangleShape(32, 48).area, closeTo(1536, 1e-9));
    const triangle = [Offset(0, 0), Offset(10, 0), Offset(0, 10)];
    expect(PolygonShape(triangle).area, closeTo(50, 1e-9));
    expect(
      PolygonShape(triangle.reversed.toList()).area,
      closeTo(50, 1e-9),
      reason: 'clockwise or not, an area is positive',
    );
  });

  test('a capsule is its middle box and one whole circle', () {
    final capsule = CapsuleShape.vertical(height: 48, radius: 16);
    // 16 between the centres, 32 wide, plus a circle of radius 16.
    expect(capsule.area, closeTo(16 * 32 + math.pi * 256, 1e-9));
  });

  test('a rounded rectangle is the box grown by its corner radius', () {
    final rounded = RoundedPolygonShape.rect(
      width: 20,
      height: 10,
      cornerRadius: 2,
    );
    // The full 24 x 14 box, less the four square corners the caps round off.
    expect(rounded.area, closeTo(24 * 14 - (4 - math.pi) * 4, 1e-9));
  });

  test('a line covers nothing', () {
    expect(SegmentShape(Offset.zero, const Offset(100, 0)).area, 0);
    expect(
      ChainShape(const [Offset(0, 0), Offset(50, 0), Offset(50, 50)]).area,
      0,
    );
  });
}
