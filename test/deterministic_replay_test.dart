import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

import 'support/simulation_harness.dart';

void main() {
  group('deterministic replay - pure dart', () {
    test('same seed yields identical snapshot hash', () {
      final runA = buildDeterministicScenario(
        engine: PhysicsEngine.pureDart(),
        seed: 4242,
        dynamicBodyCount: 256,
      );
      stepEngine(runA, steps: 240);
      final hashA = snapshotHash(runA);
      runA.dispose();

      final runB = buildDeterministicScenario(
        engine: PhysicsEngine.pureDart(),
        seed: 4242,
        dynamicBodyCount: 256,
      );
      stepEngine(runB, steps: 240);
      final hashB = snapshotHash(runB);
      runB.dispose();

      expect(hashA, equals(hashB));
    });

    test('different seeds yield different snapshot hash', () {
      final runA = buildDeterministicScenario(
        engine: PhysicsEngine.pureDart(),
        seed: 4242,
        dynamicBodyCount: 256,
      );
      stepEngine(runA, steps: 180);
      final hashA = snapshotHash(runA);
      runA.dispose();

      final runB = buildDeterministicScenario(
        engine: PhysicsEngine.pureDart(),
        seed: 4243,
        dynamicBodyCount: 256,
      );
      stepEngine(runB, steps: 180);
      final hashB = snapshotHash(runB);
      runB.dispose();

      expect(hashA, isNot(equals(hashB)));
    });
  });
}
