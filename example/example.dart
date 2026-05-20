import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  final engine = PhysicsEngine()..initialize();

  final floor = PhysicsBody(
    position: Vector2(320, 520),
    shape: RectangleShape(640, 40),
    mass: 0.0, // Static body.
    restitution: 0.2,
    friction: 0.8,
  );

  final ball = PhysicsBody(
    position: Vector2(320, 120),
    shape: CircleShape(20),
    mass: 1.0,
    restitution: 0.55,
    friction: 0.35,
    drag: 0.02,
  );

  engine.addBody(floor);
  engine.addBody(ball);

  // Simulate 2 seconds at 60 Hz.
  const dt = 1.0 / 60.0;
  const steps = 120;
  for (var i = 0; i < steps; i++) {
    engine.update(dt);
  }

  final stats = engine.stats;
  print(
    'Ball position: (${ball.position.x.toStringAsFixed(2)}, '
    '${ball.position.y.toStringAsFixed(2)})',
  );
  print(
    'Ball velocity: (${ball.velocity.x.toStringAsFixed(2)}, '
    '${ball.velocity.y.toStringAsFixed(2)})',
  );
  print('Backend: ${stats['backend'] ?? 'physics_2d'}');
  print('Bodies: ${stats['bodyCount']}');
  print(
    'Resolved collisions: ${stats['resolvedCollisions'] ?? stats['contactCount'] ?? 0}',
  );

  engine.dispose();
}
