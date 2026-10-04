// Static bodies never test against one another.
//
// A level's collision can be hundreds of static pieces side by side — a tile
// map's merged rows, say. They cannot move, so they can neither push each
// other nor start or stop touching; Box2D never pairs them, and the pure-Dart
// backend must not either, or every step pays for every neighbouring pair
// and reports contacts between pieces of the same floor.

import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  PhysicsBody staticBox(double x, double y, {bool sensor = false}) =>
      PhysicsBody(
        position: Vector2(x, y),
        shape: RectangleShape(32, 32),
        bodyType: BodyType.static,
        useGravity: false,
        isSensor: sensor,
      );

  test('overlapping static bodies raise no contacts and no sensor events', () {
    final engine = PhysicsEngine.pureDart()..initialize();
    // A row of pieces that overlap their neighbours, plus a static sensor
    // lying across all of them.
    for (var i = 0; i < 100; i++) {
      engine.addBody(staticBox(i * 30.0, 0));
    }
    engine.addBody(staticBox(1500, 0, sensor: true));

    var contacts = 0;
    var sensors = 0;
    for (var step = 0; step < 5; step++) {
      engine.update(1 / 60);
      engine.pollContactBeginEvents((_, _, _, _) => contacts++);
      engine.pollSensorBeginEvents((_, _) => sensors++);
    }

    expect(contacts, 0);
    expect(sensors, 0);
    engine.dispose();
  });

  test('a moving body still lands on a static one', () {
    final engine = PhysicsEngine.pureDart()..initialize();
    engine.addBody(staticBox(0, 0));
    engine.addBody(staticBox(30, 0));
    final ball = PhysicsBody(
      position: Vector2(0, -40),
      shape: CircleShape(8),
      restitution: 0,
    );
    engine.addBody(ball);

    var landed = false;
    for (var step = 0; step < 60 && !landed; step++) {
      engine.update(1 / 60);
      engine.pollContactBeginEvents((a, b, _, _) {
        if (identical(a, ball) || identical(b, ball)) landed = true;
      });
    }

    expect(landed, isTrue);
    expect(ball.position.y, lessThan(0));
    engine.dispose();
  });
}
