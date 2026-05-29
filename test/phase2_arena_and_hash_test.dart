import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  group('phase2 arena + deterministic hash', () {
    test('arena diagnostics are exposed in stats', () {
      final engine = PhysicsEngine.pureDart()..initialize();

      for (var i = 0; i < 12; i++) {
        engine.addBody(
          PhysicsBody(
            position: Vector2(i * 20.0, i * -12.0),
            shape: CircleShape(8),
          ),
        );
      }

      engine.update(1.0 / 60.0);
      final stats = engine.stats;

      expect(stats['experimentalArena'], isTrue);
      expect((stats['arenaTrackedBodies'] as int), equals(12));

      engine.dispose();
    });

    test('deterministic hash is stable for identical scenarios', () {
      int runHash() {
        final engine = PhysicsEngine.pureDart(deterministicHashEnabled: true)
          ..initialize();

        for (var i = 0; i < 40; i++) {
          engine.addBody(
            PhysicsBody(
              position: Vector2(i * 9.0, -i * 14.0),
              shape: CircleShape(7 + (i % 2)),
              velocity: Vector2(i * 0.7, -i * 0.3),
              mass: 1.0 + (i % 3) * 0.2,
              drag: 0.01,
              restitution: 0.2,
            ),
          );
        }

        for (var i = 0; i < 120; i++) {
          engine.update(1.0 / 60.0);
        }

        final hash = engine.stats['deterministicHash'] as int;
        engine.dispose();
        return hash;
      }

      final a = runHash();
      final b = runHash();
      expect(a, equals(b));
    });

    test('deterministic hash cadence follows configured step interval', () {
      final engine = PhysicsEngine.pureDart(
        deterministicHashEnabled: true,
        deterministicHashIntervalSteps: 5,
      )..initialize();

      engine.addBody(
        PhysicsBody(
          position: Vector2(0, 0),
          shape: CircleShape(10),
          velocity: Vector2(3, -2),
        ),
      );

      for (var i = 0; i < 4; i++) {
        engine.update(1.0 / 60.0);
      }

      expect(engine.stats['simulationStep'], equals(4));
      expect(engine.stats['deterministicHashLastStep'], equals(0));

      engine.update(1.0 / 60.0);

      expect(engine.stats['simulationStep'], equals(5));
      expect(engine.stats['deterministicHashLastStep'], equals(5));
      expect(engine.stats['deterministicHash'], isNot(equals(0)));

      engine.dispose();
    });

    test('broadphase pipeline reports isolate worker or fallback mode', () {
      final engine = PhysicsEngine.pureDart(
        experimentalIsolateBroadphaseEnabled: true,
        experimentalIsolateBroadphaseMinBodies: 2,
      )..initialize();

      for (var i = 0; i < 64; i++) {
        engine.addBody(
          PhysicsBody(
            position: Vector2(i * 14.0, 0),
            shape: CircleShape(10),
            velocity: Vector2(i.isEven ? 3 : -3, 0),
          ),
        );
      }

      for (var i = 0; i < 20; i++) {
        engine.update(1.0 / 60.0);
      }

      expect(engine.stats['experimentalIsolateBroadphase'], isTrue);
      expect(engine.stats['broadphaseWorkerInFlight'], isA<bool>());

      final pipeline = engine.stats['broadphasePipeline'] as String;
      expect(
        [
          'isolate_worker',
          'isolate_fallback_main_thread',
          'main_thread_fallback',
          'isolate_waiting_dispatch',
        ].contains(pipeline),
        isTrue,
      );

      engine.dispose();
    });

    test('isolate broadphase algorithm matches main-thread pair keys', () {
      final engine = PhysicsEngine.pureDart()..initialize();

      for (var i = 0; i < 120; i++) {
        final isStatic = i % 17 == 0;
        engine.addBody(
          PhysicsBody(
            position: Vector2((i % 20) * 22.0, (i ~/ 20) * 26.0),
            shape: CircleShape(9 + (i % 3)),
            velocity: Vector2(i.isEven ? 2.0 : -2.0, i % 3 == 0 ? 1.0 : -1.0),
            mass: isStatic ? 0.0 : 1.0,
            checkCollision: i % 29 != 0,
          ),
        );
      }

      final mainKeys = engine.debugBroadphasePairKeys(
        useIsolateAlgorithm: false,
      );
      final isolateAlgoKeys = engine.debugBroadphasePairKeys(
        useIsolateAlgorithm: true,
      );

      expect(isolateAlgoKeys, equals(mainKeys));

      engine.dispose();
    });

    test('isolate payload is compacted to enabled collider set', () {
      final engine = PhysicsEngine.pureDart(
        experimentalIsolateBroadphaseEnabled: true,
        experimentalIsolateBroadphaseMinBodies: 2,
      )..initialize();

      for (var i = 0; i < 50; i++) {
        engine.addBody(
          PhysicsBody(
            position: Vector2(i * 10.0, 0),
            shape: CircleShape(8),
            mass: i % 10 == 0 ? 0.0 : 1.0,
            checkCollision: i % 7 != 0,
          ),
        );
      }

      for (var i = 0; i < 4; i++) {
        engine.update(1.0 / 60.0);
      }

      final serializedBodies =
          engine.stats['broadphaseSerializedBodies'] as int;
      expect(serializedBodies, lessThan(engine.bodies.length));
      expect(engine.stats['broadphaseSerializedBytes'], greaterThan(0));

      engine.dispose();
    });

    test('isolate broadphase respects minimum body threshold', () {
      final engine = PhysicsEngine.pureDart(
        experimentalIsolateBroadphaseEnabled: true,
        experimentalIsolateBroadphaseMinBodies: 100,
      )..initialize();

      for (var i = 0; i < 20; i++) {
        engine.addBody(
          PhysicsBody(position: Vector2(i * 12.0, 0), shape: CircleShape(8)),
        );
      }

      engine.update(1.0 / 60.0);

      expect(engine.stats['broadphaseEligibleBodies'], equals(20));
      expect(
        engine.stats['broadphasePipeline'],
        equals('isolate_disabled_small_world'),
      );
      expect(engine.stats['broadphaseSerializedBodies'], equals(0));

      engine.dispose();
    });

    test('isolate broadphase dispatch cadence gates scheduling', () {
      final engine = PhysicsEngine.pureDart(
        experimentalIsolateBroadphaseEnabled: true,
        experimentalIsolateBroadphaseMinBodies: 2,
        experimentalIsolateBroadphaseDispatchEverySteps: 4,
      )..initialize();

      for (var i = 0; i < 32; i++) {
        engine.addBody(
          PhysicsBody(
            position: Vector2(i * 12.0, 0),
            shape: CircleShape(8),
            velocity: Vector2(i.isEven ? 2 : -2, 0),
          ),
        );
      }

      engine.update(1.0 / 60.0); // step 1 (no dispatch)
      expect(engine.stats['simulationStep'], equals(1));
      expect(
        engine.stats['broadphasePipeline'],
        equals('isolate_waiting_dispatch'),
      );

      engine.update(1.0 / 60.0); // step 2 (no dispatch)
      expect(
        engine.stats['broadphasePipeline'],
        equals('isolate_waiting_dispatch'),
      );

      engine.update(1.0 / 60.0); // step 3 (no dispatch)
      expect(
        engine.stats['broadphasePipeline'],
        equals('isolate_waiting_dispatch'),
      );

      engine.update(1.0 / 60.0); // step 4 (dispatch)
      final pipeline = engine.stats['broadphasePipeline'] as String;
      expect(
        [
          'isolate_fallback_main_thread',
          'isolate_worker',
          'main_thread_fallback',
        ].contains(pipeline),
        isTrue,
      );

      engine.dispose();
    });

    test('adaptive mode disables isolate under very high frame budget', () {
      final engine = PhysicsEngine.pureDart(
        experimentalIsolateBroadphaseAdaptiveEnabled: true,
        experimentalIsolateBroadphaseMinBodies: 2,
        experimentalIsolateBroadphaseAdaptiveCheckIntervalSteps: 1,
        experimentalIsolateBroadphaseTargetStepMs: 10000,
      )..initialize();

      for (var i = 0; i < 64; i++) {
        engine.addBody(
          PhysicsBody(position: Vector2(i * 12.0, 0), shape: CircleShape(8)),
        );
      }

      engine.update(1.0 / 60.0);

      expect(engine.stats['adaptiveIsolateActive'], isFalse);
      expect(
        engine.stats['broadphasePipeline'],
        equals('adaptive_disabled_main_thread'),
      );

      engine.dispose();
    });

    test('adaptive mode enables isolate under tiny frame budget', () {
      final engine = PhysicsEngine.pureDart(
        experimentalIsolateBroadphaseAdaptiveEnabled: true,
        experimentalIsolateBroadphaseMinBodies: 2,
        experimentalIsolateBroadphaseAdaptiveCheckIntervalSteps: 1,
        experimentalIsolateBroadphaseTargetStepMs: 0.0001,
      )..initialize();

      for (var i = 0; i < 64; i++) {
        engine.addBody(
          PhysicsBody(
            position: Vector2(i * 12.0, 0),
            shape: CircleShape(8),
            velocity: Vector2(i.isEven ? 1 : -1, 0),
          ),
        );
      }

      for (var i = 0; i < 8; i++) {
        engine.update(1.0 / 60.0);
      }

      expect(engine.stats['adaptiveIsolateActive'], isTrue);
      final pipeline = engine.stats['broadphasePipeline'] as String;
      expect(
        [
          'isolate_worker',
          'isolate_fallback_main_thread',
          'main_thread_fallback',
          'isolate_waiting_dispatch',
        ].contains(pipeline),
        isTrue,
      );

      engine.dispose();
    });

    test('adaptive min hold steps and transition diagnostics are reported', () {
      final engine = PhysicsEngine.pureDart(
        experimentalIsolateBroadphaseAdaptiveEnabled: true,
        experimentalIsolateBroadphaseMinBodies: 2,
        experimentalIsolateBroadphaseAdaptiveCheckIntervalSteps: 1,
        experimentalIsolateBroadphaseAdaptiveMinHoldSteps: 1000,
        experimentalIsolateBroadphaseTargetStepMs: 0.0001,
      )..initialize();

      for (var i = 0; i < 32; i++) {
        engine.addBody(
          PhysicsBody(
            position: Vector2(i * 12.0, 0),
            shape: CircleShape(8),
            velocity: Vector2(i.isEven ? 1 : -1, 0),
          ),
        );
      }

      engine.update(1.0 / 60.0);

      expect(
        engine.stats['experimentalIsolateBroadphaseAdaptiveMinHoldSteps'],
        equals(1000),
      );
      expect(engine.stats['adaptiveIsolateActive'], isTrue);
      expect(engine.stats['adaptiveTransitionCount'], equals(1));
      expect(engine.stats['adaptiveLastTransitionStep'], equals(1));

      for (var i = 0; i < 10; i++) {
        engine.update(1.0 / 60.0);
      }

      expect(engine.stats['adaptiveTransitionCount'], equals(1));

      engine.dispose();
    });

    test('adaptive policy transitions enable then disable with hold lock', () {
      final engine = PhysicsEngine.pureDart(
        experimentalIsolateBroadphaseAdaptiveEnabled: true,
        experimentalIsolateBroadphaseMinBodies: 2,
        experimentalIsolateBroadphaseAdaptiveCheckIntervalSteps: 1,
        experimentalIsolateBroadphaseAdaptiveMinHoldSteps: 5,
        experimentalIsolateBroadphaseAdaptiveMargin: 0,
        experimentalIsolateBroadphaseTargetStepMs: 1.0,
        experimentalAdaptiveStepMsOverrideForTesting: (step, measuredMs) {
          if (step <= 2) return 4.0; // force enable
          return -100.0; // force below-target quickly despite EMA inertia
        },
      )..initialize();

      for (var i = 0; i < 32; i++) {
        engine.addBody(
          PhysicsBody(
            position: Vector2(i * 12.0, 0),
            shape: CircleShape(8),
            velocity: Vector2(i.isEven ? 1 : -1, 0),
          ),
        );
      }

      engine.update(1.0 / 60.0); // step 1: enable
      expect(engine.stats['adaptiveIsolateActive'], isTrue);
      expect(engine.stats['adaptiveTransitionCount'], equals(1));
      expect(engine.stats['adaptiveLastTransitionStep'], equals(1));

      engine.update(1.0 / 60.0); // step 2: still high, still enabled
      expect(engine.stats['adaptiveIsolateActive'], isTrue);
      expect(engine.stats['adaptiveTransitionCount'], equals(1));

      engine.update(1.0 / 60.0); // step 3: low, hold-locked
      expect(engine.stats['adaptiveIsolateActive'], isTrue);
      expect(engine.stats['adaptiveDecisionReason'], equals('hold_locked'));
      expect(engine.stats['adaptiveTransitionCount'], equals(1));

      engine.update(1.0 / 60.0); // step 4: low, still hold-locked
      expect(engine.stats['adaptiveIsolateActive'], isTrue);
      expect(engine.stats['adaptiveDecisionReason'], equals('hold_locked'));
      expect(engine.stats['adaptiveTransitionCount'], equals(1));

      engine.update(1.0 / 60.0); // step 5: low, still hold-locked
      expect(engine.stats['adaptiveIsolateActive'], isTrue);
      expect(engine.stats['adaptiveDecisionReason'], equals('hold_locked'));
      expect(engine.stats['adaptiveTransitionCount'], equals(1));

      engine.update(1.0 / 60.0); // step 6: low, hold satisfied -> disable
      expect(engine.stats['adaptiveIsolateActive'], isFalse);
      expect(engine.stats['adaptiveDecisionReason'], equals('below_target'));
      expect(engine.stats['adaptiveTransitionCount'], equals(2));
      expect(engine.stats['adaptiveLastTransitionStep'], equals(6));

      engine.dispose();
    });
  });
}
