// PhysicsEngineFactory.create3D: Jolt where it can run (this package's own
// pubspec asks for it to be built), and a Physics3DUnsupportedError naming
// the reason where it cannot.

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/physics_3d.dart';
import 'package:just_physics_engine/testing.dart';

void main() {
  test('create3D gives Jolt here', () {
    expect(PhysicsEngineFactory.supports3D, isTrue);
    expect(PhysicsEngineFactory.unavailable3DReason, isNull);
    final engine = PhysicsEngineFactory.create3D();
    expect(engine, isA<JoltPhysicsEngine>());
    expect(engine.name, 'jolt');
    final world = engine.createWorld(Physics3DWorldSettings());
    addTearDown(world.dispose);
    expect(world.bodyCount, 0);
  });

  test('an engine that cannot run throws the typed error with its reason', () {
    final engine = FakePhysics3DEngine(unavailableReason: 'not here');
    expect(
      () => engine.createWorld(Physics3DWorldSettings()),
      throwsA(
        isA<Physics3DUnsupportedError>().having(
          (e) => e.message,
          'message',
          contains('not here'),
        ),
      ),
    );
  });

  test('the typed error is an UnsupportedError', () {
    expect(Physics3DUnsupportedError('x'), isA<UnsupportedError>());
  });
}
