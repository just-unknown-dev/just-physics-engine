import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  group('PhysicsEngine', () {
    test('initialize and dispose without error', () {
      final engine = PhysicsEngine();
      engine.initialize();
      engine.dispose();
    });

    test('addBody and removeBody', () {
      final engine = PhysicsEngine();
      engine.initialize();
      final body = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(10.0),
      );
      engine.addBody(body);
      expect(engine.bodies.length, 1);
      engine.removeBody(body);
      expect(engine.bodies.length, 0);
      engine.dispose();
    });
  });

  group('Vector2', () {
    test('addScaled mutates in-place', () {
      final v = Vector2(1.0, 0.0);
      final other = Vector2(0.0, 1.0);
      v.addScaled(other, 2.0);
      expect(v.x, 1.0);
      expect(v.y, 2.0);
    });
  });

  group('PhysicsEngine3D', () {
    test('stub lifecycle', () {
      final engine = PhysicsEngine3D();
      engine.initialize();
      engine.update(0.016);
      engine.dispose();
    });
  });
}
