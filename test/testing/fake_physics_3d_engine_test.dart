// The fake 3-D engine's own promises: handles that are the same for the
// same calls on a new engine whatever ran before, that never name another
// world's body, and a world that stands in for an engine that cannot run.
// What it shares with Jolt is in ../physics_3d/.

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/physics_3d.dart';
import 'package:just_physics_engine/testing.dart';

List<int> _someHandles(Physics3DEngine engine) {
  final w = engine.createWorld(Physics3DWorldSettings());
  final handles = [
    w.createBody(const Box3DShape(1, 1, 1), Body3DSettings()),
    w.createCharacter(Character3DSettings(), Vector3.zero()),
    w.createBody(const Sphere3DShape(1), Body3DSettings()),
  ];
  w.dispose();
  return handles;
}

void main() {
  test('the same calls on a new engine give the same handles', () {
    final first = _someHandles(FakePhysics3DEngine());
    // Whatever else ran in between.
    _someHandles(FakePhysics3DEngine());
    expect(_someHandles(FakePhysics3DEngine()), first);
    expect(first, everyElement(isNot(0)));
  });

  test("one world's handle names nothing in another", () {
    final engine = FakePhysics3DEngine();
    final a = engine.createWorld(Physics3DWorldSettings());
    final b = engine.createWorld(Physics3DWorldSettings());
    addTearDown(a.dispose);
    addTearDown(b.dispose);
    final inA = a.createBody(const Box3DShape(1, 1, 1), Body3DSettings());
    final inB = b.createBody(const Box3DShape(1, 1, 1), Body3DSettings());
    expect(inA, isNot(inB));
    expect(b.isBody(inA), isFalse);
    b.destroyBody(inA);
    expect(a.isBody(inA), isTrue);
  });

  test('an engine that cannot run says why, and makes no world', () {
    final engine = FakePhysics3DEngine(unavailableReason: 'no Jolt here');
    expect(engine.unavailableReason, 'no Jolt here');
    expect(
      () => engine.createWorld(Physics3DWorldSettings()),
      throwsA(isA<Physics3DUnsupportedError>()),
    );
  });

  test('it keeps its worlds, and destroyAll disposes them', () {
    final engine = FakePhysics3DEngine();
    final w = engine.createWorld(Physics3DWorldSettings());
    expect(engine.worlds, [w]);
    engine.destroyAll();
    expect(w.isDisposed, isTrue);
    expect(engine.worlds, isEmpty);
    expect(engine.destroyAllCalls, 1);
  });
}
