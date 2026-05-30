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

    test(
      'joint-connected sleeping body wakes when partner is externally woken',
      () {
        final engine = PhysicsEngine.pureDart()..initialize();
        engine.setGravity(0, 0);

        final bodyA = PhysicsBody(
          position: Vector2(0, 0),
          shape: CircleShape(10),
          velocity: Vector2.zero(),
          useGravity: false,
          sleepVelocityThreshold: 0.5,
          sleepTimeThreshold: 0.03,
        );
        final bodyB = PhysicsBody(
          position: Vector2(40, 0),
          shape: CircleShape(10),
          velocity: Vector2.zero(),
          useGravity: false,
          sleepVelocityThreshold: 0.5,
          sleepTimeThreshold: 0.03,
        );

        engine.addBody(bodyA);
        engine.addBody(bodyB);
        engine.addDistanceJoint(bodyA, bodyB, length: 40);

        for (var i = 0; i < 6; i++) {
          engine.update(1.0 / 60.0);
        }
        expect(bodyA.isAwake, isFalse);
        expect(bodyB.isAwake, isFalse);

        bodyA.applyImpulse(Vector2(1, 0));
        engine.update(1.0 / 60.0);

        expect(bodyA.isAwake, isTrue);
        expect(bodyB.isAwake, isTrue);
        expect(engine.stats['jointWakeActivations'], greaterThan(0));

        engine.dispose();
      },
    );
    test('warm-start supports independent normal and tangent decay', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactWarmStartNormalImpulseDecay: 0.0,
        experimentalContactWarmStartTangentImpulseDecay: 1.0,
      )..initialize();

      engine.addBody(
        PhysicsBody(
          position: Vector2(0, 120),
          shape: RectangleShape(1000, 40),
          mass: 0,
          useGravity: false,
        ),
      );
      engine.addBody(
        PhysicsBody(
          position: Vector2(0, 95),
          shape: CircleShape(18),
          velocity: Vector2(30, 0),
          restitution: 0,
          friction: 0.9,
          drag: 0.0,
        ),
      );

      for (var i = 0; i < 12; i++) {
        engine.update(1.0 / 60.0);
      }

      expect(engine.stats['warmStartContacts'], greaterThan(0));
      expect(engine.stats['warmStartCacheSize'], greaterThan(0));
      expect(
        engine.stats['warmStartMaxCachedNormalImpulse'] as double,
        equals(0.0),
      );
      expect(
        engine.stats['warmStartMaxCachedTangentImpulse'] as double,
        greaterThan(0.0),
      );

      engine.dispose();
    });

    test('warm-start split decay defaults to shared decay', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactWarmStartImpulseDecay: 0.42,
      )..initialize();

      expect(
        engine.stats['experimentalContactWarmStartImpulseDecay'],
        equals(0.42),
      );
      expect(
        engine.stats['experimentalContactWarmStartNormalImpulseDecay'],
        equals(0.42),
      );
      expect(
        engine.stats['experimentalContactWarmStartTangentImpulseDecay'],
        equals(0.42),
      );

      engine.dispose();
    });

    test('contact velocity iterations apply per manifold point', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactVelocityIterations: 3,
      )..initialize();
      engine.setGravity(0, 0);

      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2.zero(),
        useGravity: false,
        drag: 0.0,
      );
      final b = PhysicsBody(
        position: Vector2(30, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2.zero(),
        useGravity: false,
        drag: 0.0,
      );

      engine.addBody(a);
      engine.addBody(b);
      engine.update(1.0 / 60.0);

      expect(engine.stats['experimentalContactVelocityIterations'], equals(3));
      final manifoldPoints = engine.stats['resolvedManifoldPoints'] as int;
      final velocityIterations =
          engine.stats['resolvedVelocityIterations'] as int;
      expect(manifoldPoints, greaterThan(0));
      expect(velocityIterations, equals(manifoldPoints * 3));

      engine.dispose();
    });

    test(
      'contact velocity iterations resolve tangent impulse under friction',
      () {
        final engine = PhysicsEngine.pureDart(
          experimentalContactVelocityIterations: 3,
        )..initialize();
        engine.setGravity(0, 0);

        final a = PhysicsBody(
          position: Vector2(0, 0),
          shape: RectangleShape(40, 40),
          velocity: Vector2(4, 3),
          friction: 0.9,
          useGravity: false,
          drag: 0.0,
        );
        final b = PhysicsBody(
          position: Vector2(30, 0),
          shape: RectangleShape(40, 40),
          velocity: Vector2(-4, -3),
          friction: 0.9,
          useGravity: false,
          drag: 0.0,
        );

        engine.addBody(a);
        engine.addBody(b);
        engine.update(1.0 / 60.0);

        expect(engine.stats['resolvedManifoldPoints'], greaterThanOrEqualTo(2));
        final tangentImpulse =
            (engine.stats['resolvedTangentImpulse'] as double).abs();
        expect(tangentImpulse, greaterThan(0.0));

        engine.dispose();
      },
    );

    test('two-point normal block solve executes for edge contacts', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactTwoPointBlockNormalSolveEnabled: true,
        experimentalContactVelocityIterations: 2,
      )..initialize();
      engine.setGravity(0, 0);

      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(5, 0),
        useGravity: false,
        drag: 0.0,
      );
      final b = PhysicsBody(
        position: Vector2(30, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(-5, 0),
        useGravity: false,
        drag: 0.0,
      );

      engine.addBody(a);
      engine.addBody(b);
      engine.update(1.0 / 60.0);

      expect(
        engine.stats['experimentalContactTwoPointBlockNormalSolveEnabled'],
        isTrue,
      );
      final manifoldPoints = engine.stats['resolvedManifoldPoints'] as int;
      final velocityIterations =
          engine.stats['resolvedVelocityIterations'] as int;
      expect(manifoldPoints, greaterThanOrEqualTo(2));
      expect(velocityIterations, equals(manifoldPoints * 2));
      expect(engine.stats['resolvedBlockSolves'], greaterThan(0));

      engine.dispose();
    });

    test('two-point normal block solve can be warm-state gated', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactTwoPointBlockNormalSolveEnabled: true,
        experimentalContactTwoPointBlockNormalSolveMinWarmStates: 2,
        experimentalContactWarmStartManifoldSlots: 2,
        experimentalContactVelocityIterations: 2,
      )..initialize();
      engine.setGravity(0, 0);

      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(5, 0),
        useGravity: false,
        drag: 0.0,
      );
      final b = PhysicsBody(
        position: Vector2(30, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(-5, 0),
        useGravity: false,
        drag: 0.0,
      );

      engine.addBody(a);
      engine.addBody(b);

      engine.update(1.0 / 60.0);
      expect(engine.stats['resolvedBlockSolves'], equals(0));
      expect(engine.stats['warmStartMatchedStates'], equals(0));

      engine.update(1.0 / 60.0);
      expect(engine.stats['warmStartMatchedStates'], greaterThanOrEqualTo(2));
      expect(engine.stats['resolvedBlockSolves'], greaterThan(0));

      engine.dispose();
    });

    test('two-point normal block solve supports pair hysteresis hold', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactTwoPointBlockNormalSolveEnabled: true,
        experimentalContactTwoPointBlockNormalSolveMinWarmStates: 2,
        experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates: 0,
        experimentalContactWarmStartManifoldSlots: 2,
        experimentalContactVelocityIterations: 2,
        experimentalContactWarmStartFeatureIdOverrideForTesting:
            (step, featureId) {
              if (featureId == null) return null;
              return step >= 3 ? featureId + 100000 : featureId;
            },
      )..initialize();
      engine.setGravity(0, 0);

      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(5, 0),
        useGravity: false,
        drag: 0.0,
      );
      final b = PhysicsBody(
        position: Vector2(30, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(-5, 0),
        useGravity: false,
        drag: 0.0,
      );

      engine.addBody(a);
      engine.addBody(b);

      engine.update(1.0 / 60.0);
      expect(engine.stats['resolvedBlockSolves'], equals(0));

      a.velocity.setValues(5, 0);
      b.velocity.setValues(-5, 0);
      engine.update(1.0 / 60.0);
      expect(engine.stats['warmStartMatchedStates'], greaterThanOrEqualTo(2));
      expect(engine.stats['resolvedBlockSolves'], greaterThan(0));

      a.velocity.setValues(5, 0);
      b.velocity.setValues(-5, 0);
      engine.update(1.0 / 60.0);
      expect(engine.stats['warmStartMatchedStates'], equals(0));
      expect(engine.stats['resolvedBlockSolves'], greaterThan(0));

      engine.dispose();
    });

    test(
      'two-point normal block solve drops when hysteresis threshold is crossed',
      () {
        final engine = PhysicsEngine.pureDart(
          experimentalContactTwoPointBlockNormalSolveEnabled: true,
          experimentalContactTwoPointBlockNormalSolveMinWarmStates: 2,
          experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates: 1,
          experimentalContactWarmStartManifoldSlots: 2,
          experimentalContactVelocityIterations: 2,
          experimentalContactWarmStartFeatureIdOverrideForTesting:
              (step, featureId) {
                if (featureId == null) return null;
                return step >= 3 ? featureId + 100000 : featureId;
              },
        )..initialize();
        engine.setGravity(0, 0);

        final a = PhysicsBody(
          position: Vector2(0, 0),
          shape: RectangleShape(40, 40),
          velocity: Vector2(5, 0),
          useGravity: false,
          drag: 0.0,
        );
        final b = PhysicsBody(
          position: Vector2(30, 0),
          shape: RectangleShape(40, 40),
          velocity: Vector2(-5, 0),
          useGravity: false,
          drag: 0.0,
        );

        engine.addBody(a);
        engine.addBody(b);

        engine.update(1.0 / 60.0);
        expect(engine.stats['resolvedBlockSolves'], equals(0));

        a.velocity.setValues(5, 0);
        b.velocity.setValues(-5, 0);
        engine.update(1.0 / 60.0);
        expect(engine.stats['warmStartMatchedStates'], greaterThanOrEqualTo(2));
        expect(engine.stats['resolvedBlockSolves'], greaterThan(0));

        a.velocity.setValues(5, 0);
        b.velocity.setValues(-5, 0);
        engine.update(1.0 / 60.0);
        expect(engine.stats['warmStartMatchedStates'], equals(0));
        expect(engine.stats['resolvedBlockSolves'], equals(0));

        engine.dispose();
      },
    );

    test(
      'two-point block friction solve executes for edge sliding contacts',
      () {
        final engine = PhysicsEngine.pureDart(
          experimentalContactTwoPointBlockNormalSolveEnabled: true,
          experimentalContactTwoPointBlockFrictionSolveEnabled: true,
          experimentalContactVelocityIterations: 2,
        )..initialize();
        engine.setGravity(0, 0);

        final a = PhysicsBody(
          position: Vector2(0, 0),
          shape: RectangleShape(40, 40),
          velocity: Vector2(5, 3),
          friction: 0.9,
          useGravity: false,
          drag: 0.0,
        );
        final b = PhysicsBody(
          position: Vector2(30, 0),
          shape: RectangleShape(40, 40),
          velocity: Vector2(-5, -3),
          friction: 0.9,
          useGravity: false,
          drag: 0.0,
        );

        engine.addBody(a);
        engine.addBody(b);
        engine.update(1.0 / 60.0);

        expect(
          engine.stats['experimentalContactTwoPointBlockFrictionSolveEnabled'],
          isTrue,
        );
        expect(engine.stats['resolvedManifoldPoints'], greaterThanOrEqualTo(2));
        expect(engine.stats['resolvedBlockFrictionSolves'], greaterThan(0));
        expect(
          (engine.stats['resolvedTangentImpulse'] as double).abs(),
          greaterThan(0.0),
        );

        engine.dispose();
      },
    );

    test('two-point block friction solve can be warm-state gated', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactTwoPointBlockNormalSolveEnabled: true,
        experimentalContactTwoPointBlockFrictionSolveEnabled: true,
        experimentalContactTwoPointBlockFrictionSolveMinWarmStates: 2,
        experimentalContactWarmStartManifoldSlots: 2,
        experimentalContactVelocityIterations: 2,
      )..initialize();
      engine.setGravity(0, 0);

      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(5, 3),
        friction: 0.9,
        useGravity: false,
        drag: 0.0,
      );
      final b = PhysicsBody(
        position: Vector2(30, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(-5, -3),
        friction: 0.9,
        useGravity: false,
        drag: 0.0,
      );

      engine.addBody(a);
      engine.addBody(b);

      engine.update(1.0 / 60.0);
      expect(engine.stats['resolvedBlockFrictionSolves'], equals(0));
      expect(engine.stats['warmStartMatchedStates'], equals(0));

      engine.update(1.0 / 60.0);
      expect(engine.stats['warmStartMatchedStates'], greaterThanOrEqualTo(2));
      expect(engine.stats['resolvedBlockFrictionSolves'], greaterThan(0));

      engine.dispose();
    });

    test('two-point block friction solve supports pair hysteresis hold', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactTwoPointBlockNormalSolveEnabled: true,
        experimentalContactTwoPointBlockFrictionSolveEnabled: true,
        experimentalContactTwoPointBlockFrictionSolveMinWarmStates: 2,
        experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates: 0,
        experimentalContactWarmStartManifoldSlots: 2,
        experimentalContactVelocityIterations: 2,
        experimentalContactWarmStartFeatureIdOverrideForTesting:
            (step, featureId) {
              if (featureId == null) return null;
              return step >= 3 ? featureId + 100000 : featureId;
            },
      )..initialize();
      engine.setGravity(0, 0);

      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(5, 3),
        friction: 0.9,
        useGravity: false,
        drag: 0.0,
      );
      final b = PhysicsBody(
        position: Vector2(30, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(-5, -3),
        friction: 0.9,
        useGravity: false,
        drag: 0.0,
      );

      engine.addBody(a);
      engine.addBody(b);

      engine.update(1.0 / 60.0);
      expect(engine.stats['resolvedBlockFrictionSolves'], equals(0));

      a.velocity.setValues(5, 3);
      b.velocity.setValues(-5, -3);
      engine.update(1.0 / 60.0);
      expect(engine.stats['warmStartMatchedStates'], greaterThanOrEqualTo(2));
      expect(engine.stats['resolvedBlockFrictionSolves'], greaterThan(0));

      a.velocity.setValues(5, 3);
      b.velocity.setValues(-5, -3);
      engine.update(1.0 / 60.0);
      expect(engine.stats['warmStartMatchedStates'], equals(0));
      expect(engine.stats['resolvedBlockFrictionSolves'], greaterThan(0));

      engine.dispose();
    });

    test(
      'two-point block friction solve drops when hysteresis threshold is crossed',
      () {
        final engine = PhysicsEngine.pureDart(
          experimentalContactTwoPointBlockNormalSolveEnabled: true,
          experimentalContactTwoPointBlockFrictionSolveEnabled: true,
          experimentalContactTwoPointBlockFrictionSolveMinWarmStates: 2,
          experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates:
              1,
          experimentalContactWarmStartManifoldSlots: 2,
          experimentalContactVelocityIterations: 2,
          experimentalContactWarmStartFeatureIdOverrideForTesting:
              (step, featureId) {
                if (featureId == null) return null;
                return step >= 3 ? featureId + 100000 : featureId;
              },
        )..initialize();
        engine.setGravity(0, 0);

        final a = PhysicsBody(
          position: Vector2(0, 0),
          shape: RectangleShape(40, 40),
          velocity: Vector2(5, 3),
          friction: 0.9,
          useGravity: false,
          drag: 0.0,
        );
        final b = PhysicsBody(
          position: Vector2(30, 0),
          shape: RectangleShape(40, 40),
          velocity: Vector2(-5, -3),
          friction: 0.9,
          useGravity: false,
          drag: 0.0,
        );

        engine.addBody(a);
        engine.addBody(b);

        engine.update(1.0 / 60.0);
        expect(engine.stats['resolvedBlockFrictionSolves'], equals(0));

        a.velocity.setValues(5, 3);
        b.velocity.setValues(-5, -3);
        engine.update(1.0 / 60.0);
        expect(engine.stats['warmStartMatchedStates'], greaterThanOrEqualTo(2));
        expect(engine.stats['resolvedBlockFrictionSolves'], greaterThan(0));

        a.velocity.setValues(5, 3);
        b.velocity.setValues(-5, -3);
        engine.update(1.0 / 60.0);
        expect(engine.stats['warmStartMatchedStates'], equals(0));
        expect(engine.stats['resolvedBlockFrictionSolves'], equals(0));

        engine.dispose();
      },
    );

    test(
      'block solve hysteresis transition diagnostics follow scripted run',
      () {
        final engine = PhysicsEngine.pureDart(
          experimentalContactTwoPointBlockNormalSolveEnabled: true,
          experimentalContactTwoPointBlockFrictionSolveEnabled: true,
          experimentalContactTwoPointBlockNormalSolveMinWarmStates: 2,
          experimentalContactTwoPointBlockFrictionSolveMinWarmStates: 2,
          experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates: 1,
          experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates:
              1,
          experimentalContactBlockHysteresisTransitionRateWindowSteps: 2,
          experimentalContactWarmStartManifoldSlots: 2,
          experimentalContactVelocityIterations: 2,
          experimentalContactWarmStartFeatureIdOverrideForTesting:
              (step, featureId) {
                if (featureId == null) return null;
                return step >= 3 ? featureId + 100000 : featureId;
              },
        )..initialize();
        engine.setGravity(0, 0);

        final a = PhysicsBody(
          position: Vector2(0, 0),
          shape: RectangleShape(40, 40),
          velocity: Vector2(5, 3),
          friction: 0.9,
          useGravity: false,
          drag: 0.0,
        );
        final b = PhysicsBody(
          position: Vector2(30, 0),
          shape: RectangleShape(40, 40),
          velocity: Vector2(-5, -3),
          friction: 0.9,
          useGravity: false,
          drag: 0.0,
        );

        engine.addBody(a);
        engine.addBody(b);

        // Step 1: no matched warm states, no activation.
        engine.update(1.0 / 60.0);
        expect(engine.stats['blockNormalHysteresisActivations'], equals(0));
        expect(engine.stats['blockFrictionHysteresisActivations'], equals(0));
        expect(engine.stats['blockNormalHysteresisDeactivations'], equals(0));
        expect(engine.stats['blockFrictionHysteresisDeactivations'], equals(0));
        expect(
          engine.stats['blockNormalHysteresisActivatedByThreshold'],
          equals(0),
        );
        expect(
          engine.stats['blockFrictionHysteresisActivatedByThreshold'],
          equals(0),
        );
        expect(
          engine.stats['blockNormalHysteresisDeactivatedBelowDisable'],
          equals(0),
        );
        expect(
          engine.stats['blockFrictionHysteresisDeactivatedBelowDisable'],
          equals(0),
        );
        expect(
          engine.stats['totalBlockNormalHysteresisActivations'],
          equals(0),
        );
        expect(
          engine.stats['totalBlockFrictionHysteresisActivations'],
          equals(0),
        );
        expect(
          engine.stats['totalBlockNormalHysteresisDeactivations'],
          equals(0),
        );
        expect(
          engine.stats['totalBlockFrictionHysteresisDeactivations'],
          equals(0),
        );
        expect(
          engine.stats['rollingBlockNormalHysteresisActivationRate'] as double,
          closeTo(0.0, 1e-9),
        );
        expect(
          engine.stats['rollingBlockFrictionHysteresisActivationRate']
              as double,
          closeTo(0.0, 1e-9),
        );

        // Step 2: matched warm states activate both block paths.
        a.velocity.setValues(5, 3);
        b.velocity.setValues(-5, -3);
        engine.update(1.0 / 60.0);
        expect(engine.stats['blockNormalHysteresisActivations'], equals(1));
        expect(engine.stats['blockFrictionHysteresisActivations'], equals(1));
        expect(engine.stats['blockNormalHysteresisDeactivations'], equals(0));
        expect(engine.stats['blockFrictionHysteresisDeactivations'], equals(0));
        expect(
          engine.stats['blockNormalHysteresisActivatedByThreshold'],
          equals(1),
        );
        expect(
          engine.stats['blockFrictionHysteresisActivatedByThreshold'],
          equals(1),
        );
        expect(
          engine.stats['totalBlockNormalHysteresisActivations'],
          equals(1),
        );
        expect(
          engine.stats['totalBlockFrictionHysteresisActivations'],
          equals(1),
        );
        expect(
          engine.stats['totalBlockNormalHysteresisDeactivations'],
          equals(0),
        );
        expect(
          engine.stats['totalBlockFrictionHysteresisDeactivations'],
          equals(0),
        );
        expect(
          engine.stats['rollingBlockNormalHysteresisActivationRate'] as double,
          closeTo(0.5, 1e-9),
        );
        expect(
          engine.stats['rollingBlockFrictionHysteresisActivationRate']
              as double,
          closeTo(0.5, 1e-9),
        );
        expect(
          engine.stats['rollingBlockNormalHysteresisDeactivationRate']
              as double,
          closeTo(0.0, 1e-9),
        );
        expect(
          engine.stats['rollingBlockFrictionHysteresisDeactivationRate']
              as double,
          closeTo(0.0, 1e-9),
        );
        expect(engine.stats['resolvedBlockSolves'], greaterThan(0));
        expect(engine.stats['resolvedBlockFrictionSolves'], greaterThan(0));

        // Step 3: forced feature mismatch removes matched states and drops both.
        a.velocity.setValues(5, 3);
        b.velocity.setValues(-5, -3);
        engine.update(1.0 / 60.0);
        expect(engine.stats['warmStartMatchedStates'], equals(0));
        expect(engine.stats['blockNormalHysteresisActivations'], equals(0));
        expect(engine.stats['blockFrictionHysteresisActivations'], equals(0));
        expect(engine.stats['blockNormalHysteresisDeactivations'], equals(1));
        expect(engine.stats['blockFrictionHysteresisDeactivations'], equals(1));
        expect(
          engine.stats['blockNormalHysteresisDeactivatedBelowDisable'],
          equals(1),
        );
        expect(
          engine.stats['blockFrictionHysteresisDeactivatedBelowDisable'],
          equals(1),
        );
        expect(
          engine.stats['blockNormalHysteresisDeactivatedNonTwoPoint'],
          equals(0),
        );
        expect(
          engine.stats['blockFrictionHysteresisDeactivatedNonTwoPoint'],
          equals(0),
        );
        expect(
          engine.stats['totalBlockNormalHysteresisActivations'],
          equals(1),
        );
        expect(
          engine.stats['totalBlockFrictionHysteresisActivations'],
          equals(1),
        );
        expect(
          engine.stats['totalBlockNormalHysteresisDeactivations'],
          equals(1),
        );
        expect(
          engine.stats['totalBlockFrictionHysteresisDeactivations'],
          equals(1),
        );
        expect(
          engine.stats['rollingBlockNormalHysteresisActivationRate'] as double,
          closeTo(0.5, 1e-9),
        );
        expect(
          engine.stats['rollingBlockFrictionHysteresisActivationRate']
              as double,
          closeTo(0.5, 1e-9),
        );
        expect(
          engine.stats['rollingBlockNormalHysteresisDeactivationRate']
              as double,
          closeTo(0.5, 1e-9),
        );
        expect(
          engine.stats['rollingBlockFrictionHysteresisDeactivationRate']
              as double,
          closeTo(0.5, 1e-9),
        );
        expect(engine.stats['resolvedBlockSolves'], equals(0));
        expect(engine.stats['resolvedBlockFrictionSolves'], equals(0));

        engine.dispose();
      },
    );

    test('block solve hysteresis reports non-two-point reset reason', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactTwoPointBlockNormalSolveEnabled: true,
        experimentalContactTwoPointBlockFrictionSolveEnabled: true,
        experimentalContactTwoPointBlockNormalSolveMinWarmStates: 2,
        experimentalContactTwoPointBlockFrictionSolveMinWarmStates: 2,
        experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates: 0,
        experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates: 0,
        experimentalContactWarmStartManifoldSlots: 2,
        experimentalContactVelocityIterations: 2,
      )..initialize();
      engine.setGravity(0, 0);

      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(5, 3),
        friction: 0.9,
        useGravity: false,
        drag: 0.0,
      );
      final b = PhysicsBody(
        position: Vector2(30, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(-5, -3),
        friction: 0.9,
        useGravity: false,
        drag: 0.0,
      );

      engine.addBody(a);
      engine.addBody(b);

      engine.update(1.0 / 60.0);
      a.velocity.setValues(5, 3);
      b.velocity.setValues(-5, -3);
      engine.update(1.0 / 60.0);
      expect(engine.stats['resolvedBlockSolves'], greaterThan(0));
      expect(engine.stats['resolvedBlockFrictionSolves'], greaterThan(0));

      // Force non-two-point manifold while keeping contact by switching shape.
      b.shape = CircleShape(20);
      a.velocity.setValues(5, 3);
      b.velocity.setValues(-5, -3);
      engine.update(1.0 / 60.0);

      expect(
        engine.stats['blockNormalHysteresisDeactivatedNonTwoPoint'],
        equals(1),
      );
      expect(
        engine.stats['blockNormalHysteresisDeactivatedNonTwoPointContact'],
        equals(1),
      );
      expect(
        engine.stats['blockNormalHysteresisDeactivatedPairDropped'],
        equals(0),
      );
      expect(
        engine.stats['blockFrictionHysteresisDeactivatedNonTwoPoint'],
        equals(1),
      );
      expect(
        engine.stats['blockFrictionHysteresisDeactivatedNonTwoPointContact'],
        equals(1),
      );
      expect(
        engine.stats['blockFrictionHysteresisDeactivatedPairDropped'],
        equals(0),
      );

      engine.dispose();
    });

    test('block solve hysteresis reports pair-dropped reset reason', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactTwoPointBlockNormalSolveEnabled: true,
        experimentalContactTwoPointBlockFrictionSolveEnabled: true,
        experimentalContactTwoPointBlockNormalSolveMinWarmStates: 2,
        experimentalContactTwoPointBlockFrictionSolveMinWarmStates: 2,
        experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates: 0,
        experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates: 0,
        experimentalContactWarmStartManifoldSlots: 2,
        experimentalContactVelocityIterations: 2,
      )..initialize();
      engine.setGravity(0, 0);

      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(5, 3),
        friction: 0.9,
        useGravity: false,
        drag: 0.0,
      );
      final b = PhysicsBody(
        position: Vector2(30, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2(-5, -3),
        friction: 0.9,
        useGravity: false,
        drag: 0.0,
      );

      engine.addBody(a);
      engine.addBody(b);

      engine.update(1.0 / 60.0);
      a.velocity.setValues(5, 3);
      b.velocity.setValues(-5, -3);
      engine.update(1.0 / 60.0);
      expect(engine.stats['resolvedBlockSolves'], greaterThan(0));
      expect(engine.stats['resolvedBlockFrictionSolves'], greaterThan(0));

      // Force pair removal after activation so stale-pair cleanup deactivates.
      engine.removeBody(b);
      engine.update(1.0 / 60.0);

      expect(
        engine.stats['blockNormalHysteresisDeactivatedNonTwoPoint'],
        equals(1),
      );
      expect(
        engine.stats['blockNormalHysteresisDeactivatedNonTwoPointContact'],
        equals(0),
      );
      expect(
        engine.stats['blockNormalHysteresisDeactivatedPairDropped'],
        equals(1),
      );
      expect(
        engine.stats['blockFrictionHysteresisDeactivatedNonTwoPoint'],
        equals(1),
      );
      expect(
        engine.stats['blockFrictionHysteresisDeactivatedNonTwoPointContact'],
        equals(0),
      );
      expect(
        engine.stats['blockFrictionHysteresisDeactivatedPairDropped'],
        equals(1),
      );

      engine.dispose();
    });

    test('joint wake propagation traverses a connected chain', () {
      final engine = PhysicsEngine.pureDart()..initialize();
      engine.setGravity(0, 0);

      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(8),
        velocity: Vector2.zero(),
        useGravity: false,
        sleepVelocityThreshold: 0.5,
        sleepTimeThreshold: 0.03,
      );
      final b = PhysicsBody(
        position: Vector2(32, 0),
        shape: CircleShape(8),
        velocity: Vector2.zero(),
        useGravity: false,
        sleepVelocityThreshold: 0.5,
        sleepTimeThreshold: 0.03,
      );
      final c = PhysicsBody(
        position: Vector2(64, 0),
        shape: CircleShape(8),
        velocity: Vector2.zero(),
        useGravity: false,
        sleepVelocityThreshold: 0.5,
        sleepTimeThreshold: 0.03,
      );

      engine.addBody(a);
      engine.addBody(b);
      engine.addBody(c);
      engine.addDistanceJoint(a, b, length: 32);
      engine.addDistanceJoint(b, c, length: 32);

      for (var i = 0; i < 6; i++) {
        engine.update(1.0 / 60.0);
      }
      expect(a.isAwake, isFalse);
      expect(b.isAwake, isFalse);
      expect(c.isAwake, isFalse);

      a.applyForce(Vector2(100, 0));
      engine.update(1.0 / 60.0);

      expect(a.isAwake, isTrue);
      expect(b.isAwake, isTrue);
      expect(c.isAwake, isTrue);
      expect(engine.stats['jointWakeActivations'], greaterThanOrEqualTo(2));

      engine.dispose();
    });

    test('warm-start cache records hits for persistent contacts', () {
      final engine = PhysicsEngine.pureDart()..initialize();

      engine.addBody(
        PhysicsBody(
          position: Vector2(0, 120),
          shape: RectangleShape(1000, 40),
          mass: 0,
          useGravity: false,
        ),
      );
      engine.addBody(
        PhysicsBody(
          position: Vector2(0, 95),
          shape: CircleShape(18),
          velocity: Vector2.zero(),
          restitution: 0,
          friction: 0.7,
          drag: 0.0,
        ),
      );

      for (var i = 0; i < 8; i++) {
        engine.update(1.0 / 60.0);
      }

      expect(engine.stats['experimentalContactWarmStart'], isTrue);
      expect(engine.stats['warmStartContacts'], greaterThan(0));
      expect(engine.stats['warmStartHits'], greaterThan(0));
      expect(engine.stats['warmStartCacheSize'], greaterThan(0));

      engine.dispose();
    });

    test('warm-start can be disabled and keeps cache empty', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactWarmStartEnabled: false,
      )..initialize();

      engine.addBody(
        PhysicsBody(
          position: Vector2(0, 120),
          shape: RectangleShape(1000, 40),
          mass: 0,
          useGravity: false,
        ),
      );
      engine.addBody(
        PhysicsBody(
          position: Vector2(0, 95),
          shape: CircleShape(18),
          velocity: Vector2.zero(),
          restitution: 0,
          friction: 0.7,
          drag: 0.0,
        ),
      );

      for (var i = 0; i < 8; i++) {
        engine.update(1.0 / 60.0);
      }

      expect(engine.stats['experimentalContactWarmStart'], isFalse);
      expect(engine.stats['warmStartContacts'], greaterThan(0));
      expect(engine.stats['warmStartHits'], equals(0));
      expect(engine.stats['warmStartCacheSize'], equals(0));

      engine.dispose();
    });

    test('warm-start impulse cap constrains cached impulse peaks', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactWarmStartMaxImpulse: 0.05,
      )..initialize();

      engine.addBody(
        PhysicsBody(
          position: Vector2(0, 120),
          shape: RectangleShape(1000, 40),
          mass: 0,
          useGravity: false,
        ),
      );
      engine.addBody(
        PhysicsBody(
          position: Vector2(0, 95),
          shape: CircleShape(18),
          velocity: Vector2.zero(),
          restitution: 0,
          friction: 0.7,
          drag: 0.0,
        ),
      );

      for (var i = 0; i < 12; i++) {
        engine.update(1.0 / 60.0);
      }

      expect(engine.stats['warmStartContacts'], greaterThan(0));
      expect(engine.stats['warmStartHits'], greaterThan(0));
      expect(engine.stats['warmStartCacheSize'], greaterThan(0));
      expect(
        engine.stats['warmStartMaxCachedNormalImpulse'] as double,
        lessThanOrEqualTo(0.0500001),
      );
      expect(
        engine.stats['warmStartMaxCachedTangentImpulse'] as double,
        lessThanOrEqualTo(0.0500001),
      );

      engine.dispose();
    });

    test('warm-start deterministic stats are stable across identical runs', () {
      Map<String, num> runSnapshot() {
        final engine = PhysicsEngine.pureDart(
          deterministicHashEnabled: true,
          experimentalContactWarmStartImpulseDecay: 0.9,
          experimentalContactWarmStartMaxImpulse: 2.0,
        )..initialize();

        engine.addBody(
          PhysicsBody(
            position: Vector2(0, 220),
            shape: RectangleShape(1400, 80),
            mass: 0,
            useGravity: false,
          ),
        );

        for (var i = 0; i < 24; i++) {
          engine.addBody(
            PhysicsBody(
              position: Vector2(-260 + i * 22.0, 110 - (i % 4) * 10.0),
              shape: CircleShape(9 + (i % 3)),
              velocity: Vector2(i.isEven ? 12.0 : -12.0, i % 5 == 0 ? 8.0 : 0),
              restitution: 0.1,
              friction: 0.5,
              drag: 0.0,
            ),
          );
        }

        for (var i = 0; i < 40; i++) {
          engine.update(1.0 / 60.0);
        }

        final snapshot = <String, num>{
          'deterministicHash': engine.stats['deterministicHash'] as int,
          'contactVelocityIterations':
              engine.stats['experimentalContactVelocityIterations'] as int,
          'twoPointBlockSolveEnabled':
              engine.stats['experimentalContactTwoPointBlockNormalSolveEnabled']
                  as bool
              ? 1
              : 0,
          'twoPointBlockFrictionSolveEnabled':
              engine.stats['experimentalContactTwoPointBlockFrictionSolveEnabled']
                  as bool
              ? 1
              : 0,
          'twoPointBlockNormalSolveMinWarmStates':
              engine.stats['experimentalContactTwoPointBlockNormalSolveMinWarmStates']
                  as int,
          'twoPointBlockFrictionSolveMinWarmStates':
              engine.stats['experimentalContactTwoPointBlockFrictionSolveMinWarmStates']
                  as int,
          'twoPointBlockNormalSolveDisableBelowWarmStates':
              engine.stats['experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates']
                  as int,
          'twoPointBlockFrictionSolveDisableBelowWarmStates':
              engine.stats['experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates']
                  as int,
          'blockHysteresisTransitionRateWindowSteps':
              engine.stats['experimentalContactBlockHysteresisTransitionRateWindowSteps']
                  as int,
          'warmStartImpulseDecay':
              ((engine.stats['experimentalContactWarmStartImpulseDecay']
                          as double) *
                      1000000)
                  .round(),
          'warmStartNormalImpulseDecay':
              ((engine.stats['experimentalContactWarmStartNormalImpulseDecay']
                          as double) *
                      1000000)
                  .round(),
          'warmStartTangentImpulseDecay':
              ((engine.stats['experimentalContactWarmStartTangentImpulseDecay']
                          as double) *
                      1000000)
                  .round(),
          'warmStartContacts': engine.stats['warmStartContacts'] as int,
          'warmStartHits': engine.stats['warmStartHits'] as int,
          'resolvedVelocityIterations':
              engine.stats['resolvedVelocityIterations'] as int,
          'resolvedBlockSolves': engine.stats['resolvedBlockSolves'] as int,
          'resolvedBlockFrictionSolves':
              engine.stats['resolvedBlockFrictionSolves'] as int,
          'blockNormalHysteresisActiveContacts':
              engine.stats['blockNormalHysteresisActiveContacts'] as int,
          'blockFrictionHysteresisActiveContacts':
              engine.stats['blockFrictionHysteresisActiveContacts'] as int,
          'blockNormalHysteresisActivations':
              engine.stats['blockNormalHysteresisActivations'] as int,
          'blockNormalHysteresisDeactivations':
              engine.stats['blockNormalHysteresisDeactivations'] as int,
          'blockNormalHysteresisActivatedByThreshold':
              engine.stats['blockNormalHysteresisActivatedByThreshold'] as int,
          'blockNormalHysteresisDeactivatedBelowDisable':
              engine.stats['blockNormalHysteresisDeactivatedBelowDisable']
                  as int,
          'blockNormalHysteresisDeactivatedNonTwoPoint':
              engine.stats['blockNormalHysteresisDeactivatedNonTwoPoint']
                  as int,
          'blockNormalHysteresisDeactivatedNonTwoPointContact':
              engine.stats['blockNormalHysteresisDeactivatedNonTwoPointContact']
                  as int,
          'blockNormalHysteresisDeactivatedPairDropped':
              engine.stats['blockNormalHysteresisDeactivatedPairDropped']
                  as int,
          'blockFrictionHysteresisActivations':
              engine.stats['blockFrictionHysteresisActivations'] as int,
          'blockFrictionHysteresisDeactivations':
              engine.stats['blockFrictionHysteresisDeactivations'] as int,
          'blockFrictionHysteresisActivatedByThreshold':
              engine.stats['blockFrictionHysteresisActivatedByThreshold']
                  as int,
          'blockFrictionHysteresisDeactivatedBelowDisable':
              engine.stats['blockFrictionHysteresisDeactivatedBelowDisable']
                  as int,
          'blockFrictionHysteresisDeactivatedNonTwoPoint':
              engine.stats['blockFrictionHysteresisDeactivatedNonTwoPoint']
                  as int,
          'blockFrictionHysteresisDeactivatedNonTwoPointContact':
              engine.stats['blockFrictionHysteresisDeactivatedNonTwoPointContact']
                  as int,
          'blockFrictionHysteresisDeactivatedPairDropped':
              engine.stats['blockFrictionHysteresisDeactivatedPairDropped']
                  as int,
          'totalBlockNormalHysteresisActivations':
              engine.stats['totalBlockNormalHysteresisActivations'] as int,
          'totalBlockNormalHysteresisDeactivations':
              engine.stats['totalBlockNormalHysteresisDeactivations'] as int,
          'totalBlockFrictionHysteresisActivations':
              engine.stats['totalBlockFrictionHysteresisActivations'] as int,
          'totalBlockFrictionHysteresisDeactivations':
              engine.stats['totalBlockFrictionHysteresisDeactivations'] as int,
          'rollingBlockNormalHysteresisActivationRate':
              ((engine.stats['rollingBlockNormalHysteresisActivationRate']
                          as double) *
                      1000000)
                  .round(),
          'rollingBlockNormalHysteresisDeactivationRate':
              ((engine.stats['rollingBlockNormalHysteresisDeactivationRate']
                          as double) *
                      1000000)
                  .round(),
          'rollingBlockFrictionHysteresisActivationRate':
              ((engine.stats['rollingBlockFrictionHysteresisActivationRate']
                          as double) *
                      1000000)
                  .round(),
          'rollingBlockFrictionHysteresisDeactivationRate':
              ((engine.stats['rollingBlockFrictionHysteresisDeactivationRate']
                          as double) *
                      1000000)
                  .round(),
          'warmStartCacheSize': engine.stats['warmStartCacheSize'] as int,
          'warmStartMaxCachedNormalImpulse':
              ((engine.stats['warmStartMaxCachedNormalImpulse'] as double) *
                      1000000)
                  .round(),
          'warmStartMaxCachedTangentImpulse':
              ((engine.stats['warmStartMaxCachedTangentImpulse'] as double) *
                      1000000)
                  .round(),
        };
        engine.dispose();
        return snapshot;
      }

      final a = runSnapshot();
      final b = runSnapshot();

      expect(a, equals(b));
      expect(a['warmStartHits'], greaterThan(0));
      expect(a['warmStartCacheSize'], greaterThan(0));
    });

    test('warm-start rejects stale cache entries by age gate', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactWarmStartMaxAgeSteps: 1,
      )..initialize();

      final floor = PhysicsBody(
        position: Vector2(0, 120),
        shape: RectangleShape(1000, 40),
        mass: 0,
        useGravity: false,
      );
      final ball = PhysicsBody(
        position: Vector2(0, 95),
        shape: CircleShape(18),
        velocity: Vector2.zero(),
        restitution: 0,
        friction: 0.7,
        drag: 0.0,
      );

      engine.addBody(floor);
      engine.addBody(ball);

      for (var i = 0; i < 6; i++) {
        engine.update(1.0 / 60.0);
      }
      expect(engine.stats['warmStartHits'], greaterThan(0));

      ball.checkCollision = false;
      engine.update(1.0 / 60.0); // cached entry ages by one step

      ball.checkCollision = true;
      engine.update(1.0 / 60.0); // re-contact should reject stale cache

      expect(engine.stats['warmStartContacts'], greaterThan(0));
      expect(engine.stats['warmStartHits'], equals(0));
      expect(engine.stats['warmStartRejectedByAge'], greaterThan(0));

      engine.dispose();
    });

    test('warm-start rejects contact when normal alignment flips', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactWarmStartMaxAgeSteps: 8,
        experimentalContactWarmStartNormalAlignmentThreshold: 0.95,
        experimentalContactWarmStartAlignmentOverrideForTesting:
            (step, alignment) => step >= 2 ? -1.0 : alignment,
      )..initialize();
      engine.setGravity(0, 0);

      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(20),
        velocity: Vector2.zero(),
        useGravity: false,
        drag: 0.0,
      );
      final b = PhysicsBody(
        position: Vector2(25, 0),
        shape: CircleShape(20),
        velocity: Vector2.zero(),
        useGravity: false,
        drag: 0.0,
      );

      engine.addBody(a);
      engine.addBody(b);
      engine.update(1.0 / 60.0); // seeds cache with +X-ish normal

      b.position.x = 0;
      b.position.y = 25;
      engine.update(1.0 / 60.0);

      expect(engine.stats['warmStartContacts'], greaterThan(0));
      expect(engine.stats['warmStartHits'], equals(0));
      expect(engine.stats['warmStartRejectedByNormal'], greaterThan(0));

      engine.dispose();
    });

    test('warm-start rejects contact when anchor distance diverges', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactWarmStartMaxAgeSteps: 8,
        experimentalContactWarmStartAnchorDistanceThreshold: 1.0,
      )..initialize();
      engine.setGravity(0, 0);

      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(20),
        velocity: Vector2.zero(),
        useGravity: false,
        drag: 0.0,
      );
      final b = PhysicsBody(
        position: Vector2(25, 0),
        shape: CircleShape(20),
        velocity: Vector2.zero(),
        useGravity: false,
        drag: 0.0,
      );

      engine.addBody(a);
      engine.addBody(b);
      engine.update(1.0 / 60.0); // seed cache
      expect(engine.stats['warmStartHits'], equals(0));

      b.position.x = 5;
      b.position.y =
          25; // still colliding but contact anchors shift significantly
      engine.update(1.0 / 60.0);

      expect(engine.stats['warmStartContacts'], greaterThan(0));
      expect(engine.stats['warmStartHits'], equals(0));
      expect(engine.stats['warmStartRejectedByAnchor'], greaterThan(0));

      engine.dispose();
    });

    test('warm-start rejects contact when manifold feature id diverges', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactWarmStartMaxAgeSteps: 8,
        experimentalContactWarmStartFeatureIdOverrideForTesting:
            (step, featureId) => step >= 2 ? 999 : featureId,
      )..initialize();
      engine.setGravity(0, 0);

      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: CircleShape(20),
        velocity: Vector2.zero(),
        useGravity: false,
        drag: 0.0,
      );
      final b = PhysicsBody(
        position: Vector2(25, 0),
        shape: CircleShape(20),
        velocity: Vector2.zero(),
        useGravity: false,
        drag: 0.0,
      );

      engine.addBody(a);
      engine.addBody(b);
      engine.update(1.0 / 60.0); // seed cache with original feature id
      expect(engine.stats['warmStartHits'], equals(0));

      engine.update(
        1.0 / 60.0,
      ); // should reject cache due to forced feature mismatch

      expect(engine.stats['warmStartContacts'], greaterThan(0));
      expect(engine.stats['warmStartHits'], equals(0));
      expect(engine.stats['warmStartRejectedByFeature'], greaterThan(0));

      engine.dispose();
    });

    test('warm-start caches multiple manifold points for box edge contact', () {
      final engine = PhysicsEngine.pureDart(
        experimentalContactWarmStartMaxAgeSteps: 8,
        experimentalContactWarmStartManifoldSlots: 2,
        experimentalContactWarmStartAnchorDistanceThreshold: 8.0,
      )..initialize();
      engine.setGravity(0, 0);

      final a = PhysicsBody(
        position: Vector2(0, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2.zero(),
        useGravity: false,
        drag: 0.0,
      );
      final b = PhysicsBody(
        position: Vector2(30, 0),
        shape: RectangleShape(40, 40),
        velocity: Vector2.zero(),
        useGravity: false,
        drag: 0.0,
      );

      engine.addBody(a);
      engine.addBody(b);
      engine.update(1.0 / 60.0);

      expect(engine.stats['warmStartFeatureHistoryReused'], equals(0));
      expect(engine.stats['warmStartMatchedStates'], equals(0));
      expect(engine.stats['resolvedManifoldPoints'], greaterThanOrEqualTo(2));
      expect(engine.stats['warmStartContacts'], greaterThan(0));
      expect(engine.stats['warmStartStateCount'], greaterThanOrEqualTo(2));
      expect(engine.stats['warmStartStateCount'], lessThanOrEqualTo(2));

      // Persistent contact should reuse feature-scoped cached impulse history.
      engine.update(1.0 / 60.0);
      expect(engine.stats['warmStartMatchedStates'], greaterThanOrEqualTo(2));
      expect(engine.stats['resolvedManifoldPoints'], greaterThanOrEqualTo(2));
      expect(engine.stats['warmStartResolvedFeatureSeeded'], equals(0));
      expect(engine.stats['warmStartFeatureHistoryReused'], greaterThan(0));

      engine.dispose();
    });

    test('off-center contact produces angular collision response', () {
      final engine = PhysicsEngine.pureDart()..initialize();

      final floor = PhysicsBody(
        position: Vector2(0, 140),
        shape: RectangleShape(500, 40),
        mass: 0,
        useGravity: false,
      );
      final box = PhysicsBody(
        position: Vector2(24, 90),
        shape: RectangleShape(40, 24),
        velocity: Vector2(18, 0),
        restitution: 0.0,
        friction: 0.8,
        drag: 0.0,
      );

      engine.addBody(floor);
      engine.addBody(box);

      for (var i = 0; i < 20; i++) {
        engine.update(1.0 / 60.0);
      }

      expect(engine.stats['resolvedManifoldPoints'], greaterThan(0));
      expect(box.angularVelocity.abs(), greaterThan(1e-4));

      engine.dispose();
    });

    test(
      'warm-start replays angular preload on persistent off-center contact',
      () {
        final engine = PhysicsEngine.pureDart(
          experimentalContactWarmStartManifoldSlots: 2,
        )..initialize();
        engine.setGravity(0, 0);

        final floor = PhysicsBody(
          position: Vector2(0, 140),
          shape: RectangleShape(500, 40),
          mass: 0,
          useGravity: false,
        );
        final box = PhysicsBody(
          position: Vector2(24, 109),
          shape: RectangleShape(40, 24),
          velocity: Vector2(10, 8),
          restitution: 0.0,
          friction: 0.8,
          drag: 0.0,
        );

        engine.addBody(floor);
        engine.addBody(box);

        engine.update(1.0 / 60.0);
        expect(engine.stats['warmStartHits'], equals(0));
        expect(engine.stats['warmStartAngularPreloadCount'], equals(0));

        engine.update(1.0 / 60.0);
        expect(engine.stats['warmStartHits'], greaterThan(0));
        expect(engine.stats['warmStartAngularPreloadCount'], greaterThan(0));

        engine.dispose();
      },
    );

    test('polygon manifold clipping emits stable paired feature ids', () {
      final a = RectangleShape(40, 40);
      final b = RectangleShape(40, 40);
      final manifold = a.getManifold(
        const Offset(0, 0),
        b,
        const Offset(30, 0),
      );

      expect(manifold.isColliding, isTrue);
      expect(manifold.contactPoints.length, greaterThanOrEqualTo(1));
      expect(manifold.contactPoints.length, lessThanOrEqualTo(2));
      expect(manifold.contactPoints.first.featureId, isNotNull);

      if (manifold.contactPoints.length == 2) {
        expect(
          manifold.contactPoints[0].featureId,
          isNot(equals(manifold.contactPoints[1].featureId)),
        );
      }
    });

    test('polygon-capsule manifold exposes stable point features', () {
      final poly = RectangleShape(40, 40);
      final capsule = CapsuleShape.vertical(height: 40, radius: 6);
      final manifold = poly.getManifold(
        const Offset(0, 0),
        capsule,
        const Offset(24, 0),
      );

      expect(manifold.isColliding, isTrue);
      expect(manifold.contactPoints.length, greaterThanOrEqualTo(1));
      expect(manifold.contactPoints.length, lessThanOrEqualTo(2));
      expect(manifold.contactPoints.first.featureId, isNotNull);
      if (manifold.contactPoints.length == 2) {
        expect(
          manifold.contactPoints[0].featureId,
          isNot(equals(manifold.contactPoints[1].featureId)),
        );
      }
    });

    test('rounded polygon manifold exposes stable clipped features', () {
      final rounded = RoundedPolygonShape.rect(
        width: 40,
        height: 40,
        cornerRadius: 4,
      );
      final other = RectangleShape(40, 40);
      final manifold = rounded.getManifold(
        const Offset(0, 0),
        other,
        const Offset(30, 0),
      );

      expect(manifold.isColliding, isTrue);
      expect(manifold.contactPoints.length, greaterThanOrEqualTo(1));
      expect(manifold.contactPoints.length, lessThanOrEqualTo(2));
      expect(manifold.contactPoints.first.featureId, isNotNull);
    });

    test('warm-start manifold slots retain alternating contact anchors', () {
      ({int finalHits, int seededStateCount}) runAndGetFinalHits({
        required int manifoldSlots,
      }) {
        final engine = PhysicsEngine.pureDart(
          experimentalContactWarmStartMaxAgeSteps: 20,
          experimentalContactWarmStartAnchorDistanceThreshold: 8.0,
          experimentalContactWarmStartManifoldSlots: manifoldSlots,
        )..initialize();
        engine.setGravity(0, 0);

        final a = PhysicsBody(
          position: Vector2(0, 0),
          shape: CircleShape(20),
          velocity: Vector2.zero(),
          useGravity: false,
          drag: 0.0,
        );
        final b = PhysicsBody(
          position: Vector2(25, 0),
          shape: CircleShape(20),
          velocity: Vector2.zero(),
          useGravity: false,
          drag: 0.0,
        );

        engine.addBody(a);
        engine.addBody(b);

        // Seed anchor A.
        a.position.x = 0;
        a.position.y = 0;
        a.velocity.setZero();
        b.position.x = 39;
        b.position.y = 0;
        b.velocity.setZero();
        engine.update(1.0 / 60.0);

        // Seed anchor B.
        a.position.x = 0;
        a.position.y = 0;
        a.velocity.setZero();
        b.position.x = 0;
        b.position.y = 39;
        b.velocity.setZero();
        engine.update(1.0 / 60.0);
        final seededStateCount = engine.stats['warmStartStateCount'] as int;

        // Return to anchor A and inspect reuse hit on this step.
        a.position.x = 0;
        a.position.y = 0;
        a.velocity.setZero();
        b.position.x = 39;
        b.position.y = 0;
        b.velocity.setZero();
        engine.update(1.0 / 60.0);

        final hits = engine.stats['warmStartHits'] as int;

        engine.dispose();
        return (finalHits: hits, seededStateCount: seededStateCount);
      }

      final oneSlot = runAndGetFinalHits(manifoldSlots: 1);
      final twoSlots = runAndGetFinalHits(manifoldSlots: 2);

      expect(oneSlot.seededStateCount, lessThanOrEqualTo(1));
      expect(twoSlots.seededStateCount, lessThanOrEqualTo(2));
      expect(
        twoSlots.seededStateCount,
        greaterThanOrEqualTo(oneSlot.seededStateCount),
      );
      expect(twoSlots.finalHits, greaterThanOrEqualTo(oneSlot.finalHits));
    });
  });
}
