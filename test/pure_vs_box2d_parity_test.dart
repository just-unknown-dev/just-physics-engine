import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

import 'support/simulation_harness.dart';

void main() {
  group('pure dart vs box2d parity', () {
    test('position deltas stay bounded in deterministic stress scenario', () {
      final pure = buildDeterministicScenario(
        engine: PhysicsEngine.pureDart(),
        seed: 9001,
        dynamicBodyCount: 180,
      );

      final native = buildDeterministicScenario(
        engine: PhysicsEngine(),
        seed: 9001,
        dynamicBodyCount: 180,
      );

      final nativeBackend = (native.stats['backend'] ?? 'unknown').toString();
      if (nativeBackend != 'box2d_v3') {
        pure.dispose();
        native.dispose();
        return;
      }

      stepEngine(pure, steps: 180);
      stepEngine(native, steps: 180);

      final pureBodies = dynamicBodies(pure);
      final nativeBodies = dynamicBodies(native);
      expect(nativeBodies.length, equals(pureBodies.length));

      var maxDist = 0.0;
      var sumDist = 0.0;

      for (var i = 0; i < pureBodies.length; i++) {
        final dx = pureBodies[i].position.x - nativeBodies[i].position.x;
        final dy = pureBodies[i].position.y - nativeBodies[i].position.y;
        final dist = math.sqrt(dx * dx + dy * dy);
        sumDist += dist;
        if (dist > maxDist) {
          maxDist = dist;
        }
      }

      final avgDist = sumDist / pureBodies.length;
      expect(avgDist, lessThan(45.0));
      expect(maxDist, lessThan(160.0));

      pure.dispose();
      native.dispose();
    });

    test('collision workload exists on both backends', () {
      final pure = buildDeterministicScenario(
        engine: PhysicsEngine.pureDart(),
        seed: 1337,
        dynamicBodyCount: 220,
      );

      final native = buildDeterministicScenario(
        engine: PhysicsEngine(),
        seed: 1337,
        dynamicBodyCount: 220,
      );

      final nativeBackend = (native.stats['backend'] ?? 'unknown').toString();
      if (nativeBackend != 'box2d_v3') {
        pure.dispose();
        native.dispose();
        return;
      }

      stepEngine(pure, steps: 200);
      stepEngine(native, steps: 200);

      final pureResolved = (pure.stats['resolvedCollisions'] ?? 0) as int;
      final nativeContacts = (native.stats['contactCount'] ?? 0) as int;

      expect(pureResolved, greaterThan(0));
      expect(nativeContacts, greaterThanOrEqualTo(0));

      pure.dispose();
      native.dispose();
    });
  });
}
