// ignore_for_file: avoid_print, avoid_redundant_argument_values

import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_memory/just_memory.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

const double _dt60 = 1.0 / 60.0;

void main() {
  test('phase1 deterministic performance baseline', () {
    final config = _BenchmarkConfig.fromEnvironment();

    print('=== just_physics_engine phase1 benchmark ===');
    print('targets: desktop=10000 @ 60fps, android=2000 @ 60fps');
    print('seed=${config.seed} steps=${config.steps} dt=${config.dt}');
    print(
      'isolateTuning: minBodies=${config.isolateMinBodies} dispatchEvery=${config.isolateDispatchEverySteps}',
    );
    print(
      'adaptiveTuning: enabled=${config.adaptiveEnabled} targetMs=${config.adaptiveTargetStepMs} checkEvery=${config.adaptiveCheckEverySteps} margin=${config.adaptiveMargin} minHold=${config.adaptiveMinHoldSteps}',
    );
    print('--------------------------------------------');

    final pureResult = _runPhysicsScenario(
      engineFactory: () => PhysicsEngine.pureDart(),
      bodyCount: config.bodyCount,
      seed: config.seed,
      steps: config.steps,
      dt: config.dt,
      label: 'pure_dart',
    );

    _printScenario(pureResult);

    final bestResult = _runPhysicsScenario(
      engineFactory: () => PhysicsEngine(),
      bodyCount: config.bodyCount,
      seed: config.seed,
      steps: config.steps,
      dt: config.dt,
      label: 'best_backend',
    );

    _printScenario(bestResult);

    final isolateResult = _runPhysicsScenario(
      engineFactory: () => PhysicsEngine.pureDart(
        experimentalIsolateBroadphaseEnabled: true,
        experimentalIsolateBroadphaseMinBodies: config.isolateMinBodies,
        experimentalIsolateBroadphaseDispatchEverySteps:
            config.isolateDispatchEverySteps,
      ),
      bodyCount: config.bodyCount,
      seed: config.seed,
      steps: config.steps,
      dt: config.dt,
      label: 'pure_dart_isolate_tuned',
    );

    _printScenario(isolateResult);

    final adaptiveResult = _runPhysicsScenario(
      engineFactory: () => PhysicsEngine.pureDart(
        experimentalIsolateBroadphaseAdaptiveEnabled: config.adaptiveEnabled,
        experimentalIsolateBroadphaseEnabled: true,
        experimentalIsolateBroadphaseMinBodies: config.isolateMinBodies,
        experimentalIsolateBroadphaseDispatchEverySteps:
            config.isolateDispatchEverySteps,
        experimentalIsolateBroadphaseTargetStepMs: config.adaptiveTargetStepMs,
        experimentalIsolateBroadphaseAdaptiveCheckIntervalSteps:
            config.adaptiveCheckEverySteps,
        experimentalIsolateBroadphaseAdaptiveMargin: config.adaptiveMargin,
        experimentalIsolateBroadphaseAdaptiveMinHoldSteps:
            config.adaptiveMinHoldSteps,
      ),
      bodyCount: config.bodyCount,
      seed: config.seed,
      steps: config.steps,
      dt: config.dt,
      label: 'pure_dart_isolate_adaptive',
    );

    _printScenario(adaptiveResult);

    final arenaResult = _runArenaMicroBenchmark(
      bodyCount: config.bodyCount,
      seed: config.seed,
      steps: config.steps,
      dt: config.dt,
    );
    _printArena(arenaResult);

    print('--------------------------------------------');
    print('phase1 baseline complete');

    expect(pureResult.bodyCount, equals(config.bodyCount));
    expect(pureResult.msPerStep, greaterThan(0));
    expect(arenaResult.elapsedMs, greaterThan(0));
  });
}

_BenchmarkResult _runPhysicsScenario({
  required PhysicsEngine Function() engineFactory,
  required int bodyCount,
  required int seed,
  required int steps,
  required double dt,
  required String label,
}) {
  final engine = engineFactory();
  engine.initialize();

  _populateDeterministicWorld(engine: engine, bodyCount: bodyCount, seed: seed);

  final warmupSteps = math.min(steps ~/ 10, 60);
  for (var i = 0; i < warmupSteps; i++) {
    engine.update(dt);
  }

  final watch = Stopwatch()..start();
  for (var i = 0; i < steps; i++) {
    engine.update(dt);
  }
  watch.stop();

  final elapsedMs = watch.elapsedMicroseconds / 1000.0;
  final msPerStep = elapsedMs / steps;
  final fps = msPerStep <= 0 ? 0.0 : 1000.0 / msPerStep;
  final hash = _snapshotHash(engine.bodies);
  final backend = (engine.stats['backend'] ?? 'unknown').toString();

  engine.dispose();

  return _BenchmarkResult(
    label: label,
    backend: backend,
    bodyCount: bodyCount,
    steps: steps,
    elapsedMs: elapsedMs,
    msPerStep: msPerStep,
    fps: fps,
    deterministicHash: hash,
  );
}

void _populateDeterministicWorld({
  required PhysicsEngine engine,
  required int bodyCount,
  required int seed,
}) {
  final random = math.Random(seed);

  // Floor and walls keep bodies constrained so collisions stay meaningful.
  engine.addBody(
    PhysicsBody(
      position: Vector2(0, 1200),
      shape: RectangleShape(6000, 100),
      mass: 0,
      friction: 0.9,
      restitution: 0.1,
    ),
  );
  engine.addBody(
    PhysicsBody(
      position: Vector2(-3000, 400),
      shape: RectangleShape(100, 2000),
      mass: 0,
      friction: 0.8,
      restitution: 0.1,
    ),
  );
  engine.addBody(
    PhysicsBody(
      position: Vector2(3000, 400),
      shape: RectangleShape(100, 2000),
      mass: 0,
      friction: 0.8,
      restitution: 0.1,
    ),
  );

  for (var i = 0; i < bodyCount; i++) {
    final col = i % 100;
    final row = i ~/ 100;

    final xJitter = (random.nextDouble() - 0.5) * 4.0;
    final yJitter = (random.nextDouble() - 0.5) * 4.0;

    final body = PhysicsBody(
      position: Vector2(-2400 + col * 48 + xJitter, -3000 + row * 52 + yJitter),
      shape: CircleShape(14 + (i % 3).toDouble()),
      velocity: Vector2(
        (random.nextDouble() - 0.5) * 50.0,
        (random.nextDouble() - 0.5) * 20.0,
      ),
      mass: 1.0 + (i % 4) * 0.25,
      friction: 0.2 + (i % 5) * 0.05,
      restitution: 0.05 + (i % 4) * 0.05,
      drag: 0.005,
      sleepVelocityThreshold: 0.25,
      sleepTimeThreshold: 0.5,
      isBullet: i % 40 == 0,
    );

    engine.addBody(body);
  }
}

int _snapshotHash(List<PhysicsBody> bodies) {
  // FNV-1a 64-bit style hash for deterministic replay checks.
  var hash = 0xcbf29ce484222325;
  for (final b in bodies) {
    final px = (b.position.x * 1000).round();
    final py = (b.position.y * 1000).round();
    final vx = (b.velocity.x * 1000).round();
    final vy = (b.velocity.y * 1000).round();

    hash ^= px;
    hash *= 0x100000001b3;
    hash ^= py;
    hash *= 0x100000001b3;
    hash ^= vx;
    hash *= 0x100000001b3;
    hash ^= vy;
    hash *= 0x100000001b3;
    hash &= 0x7fffffffffffffff;
  }
  return hash;
}

void _printScenario(_BenchmarkResult result) {
  print('[${result.label}] backend=${result.backend}');
  print('  bodies=${result.bodyCount} steps=${result.steps}');
  print('  totalMs=${result.elapsedMs.toStringAsFixed(2)}');
  print(
    '  msPerStep=${result.msPerStep.toStringAsFixed(4)} fps=${result.fps.toStringAsFixed(2)}',
  );
  print('  deterministicHash=${result.deterministicHash}');
}

_ArenaBenchmarkResult _runArenaMicroBenchmark({
  required int bodyCount,
  required int seed,
  required int steps,
  required double dt,
}) {
  final random = math.Random(seed);
  final arena = MemoryArena(capacity: bodyCount, componentsPerEntity: 8);
  final slots = List<int>.generate(bodyCount, (_) => arena.allocate());

  for (final slot in slots) {
    arena.setPosition(
      slot,
      random.nextDouble() * 4000 - 2000,
      random.nextDouble() * 4000 - 2000,
    );
    arena.setVelocity(
      slot,
      random.nextDouble() * 80 - 40,
      random.nextDouble() * 80 - 40,
    );
  }

  final watch = Stopwatch()..start();
  for (var i = 0; i < steps; i++) {
    arena.applyVelocityAll(dt);
  }
  watch.stop();

  var checksum = 0.0;
  for (final slot in slots) {
    checksum += arena.getX(slot) * 0.7 + arena.getY(slot) * 0.3;
  }

  return _ArenaBenchmarkResult(
    bodyCount: bodyCount,
    steps: steps,
    elapsedMs: watch.elapsedMicroseconds / 1000.0,
    checksum: checksum,
  );
}

void _printArena(_ArenaBenchmarkResult result) {
  final msPerStep = result.elapsedMs / result.steps;
  final updatesPerSec = msPerStep <= 0
      ? 0.0
      : (result.bodyCount / (msPerStep / 1000.0));

  print('[just_memory_arena]');
  print('  bodies=${result.bodyCount} steps=${result.steps}');
  print(
    '  totalMs=${result.elapsedMs.toStringAsFixed(2)} msPerStep=${msPerStep.toStringAsFixed(4)}',
  );
  print('  effectiveUpdatesPerSecond=${updatesPerSec.toStringAsFixed(0)}');
  print('  checksum=${result.checksum.toStringAsFixed(3)}');
}

class _BenchmarkConfig {
  _BenchmarkConfig({
    required this.bodyCount,
    required this.steps,
    required this.dt,
    required this.seed,
    required this.isolateMinBodies,
    required this.isolateDispatchEverySteps,
    required this.adaptiveEnabled,
    required this.adaptiveTargetStepMs,
    required this.adaptiveCheckEverySteps,
    required this.adaptiveMargin,
    required this.adaptiveMinHoldSteps,
  });

  final int bodyCount;
  final int steps;
  final double dt;
  final int seed;
  final int isolateMinBodies;
  final int isolateDispatchEverySteps;
  final bool adaptiveEnabled;
  final double adaptiveTargetStepMs;
  final int adaptiveCheckEverySteps;
  final double adaptiveMargin;
  final int adaptiveMinHoldSteps;

  factory _BenchmarkConfig.fromEnvironment() {
    const bodyCount = int.fromEnvironment('JPE_BODIES', defaultValue: 10000);
    const steps = int.fromEnvironment('JPE_STEPS', defaultValue: 600);
    const seed = int.fromEnvironment('JPE_SEED', defaultValue: 1337);
    const dtString = String.fromEnvironment('JPE_DT', defaultValue: '');
    const isolateMinBodies = int.fromEnvironment(
      'JPE_ISOLATE_MIN_BODIES',
      defaultValue: 256,
    );
    const isolateDispatchEverySteps = int.fromEnvironment(
      'JPE_ISOLATE_DISPATCH_EVERY',
      defaultValue: 1,
    );
    const adaptiveEnabled = bool.fromEnvironment(
      'JPE_ADAPTIVE_ENABLED',
      defaultValue: true,
    );
    const adaptiveTargetStepMsString = String.fromEnvironment(
      'JPE_ADAPTIVE_TARGET_MS',
      defaultValue: '16.67',
    );
    const adaptiveCheckEverySteps = int.fromEnvironment(
      'JPE_ADAPTIVE_CHECK_EVERY',
      defaultValue: 30,
    );
    const adaptiveMarginString = String.fromEnvironment(
      'JPE_ADAPTIVE_MARGIN',
      defaultValue: '0.15',
    );
    const adaptiveMinHoldSteps = int.fromEnvironment(
      'JPE_ADAPTIVE_MIN_HOLD_STEPS',
      defaultValue: 30,
    );
    final dt = dtString.isEmpty ? _dt60 : (double.tryParse(dtString) ?? _dt60);
    final adaptiveTargetStepMs =
        double.tryParse(adaptiveTargetStepMsString) ?? 16.67;
    final adaptiveMargin = double.tryParse(adaptiveMarginString) ?? 0.15;

    return _BenchmarkConfig(
      bodyCount: bodyCount,
      steps: steps,
      dt: dt,
      seed: seed,
      isolateMinBodies: isolateMinBodies,
      isolateDispatchEverySteps: isolateDispatchEverySteps,
      adaptiveEnabled: adaptiveEnabled,
      adaptiveTargetStepMs: adaptiveTargetStepMs,
      adaptiveCheckEverySteps: adaptiveCheckEverySteps,
      adaptiveMargin: adaptiveMargin,
      adaptiveMinHoldSteps: adaptiveMinHoldSteps,
    );
  }
}

class _BenchmarkResult {
  _BenchmarkResult({
    required this.label,
    required this.backend,
    required this.bodyCount,
    required this.steps,
    required this.elapsedMs,
    required this.msPerStep,
    required this.fps,
    required this.deterministicHash,
  });

  final String label;
  final String backend;
  final int bodyCount;
  final int steps;
  final double elapsedMs;
  final double msPerStep;
  final double fps;
  final int deterministicHash;
}

class _ArenaBenchmarkResult {
  _ArenaBenchmarkResult({
    required this.bodyCount,
    required this.steps,
    required this.elapsedMs,
    required this.checksum,
  });

  final int bodyCount;
  final int steps;
  final double elapsedMs;
  final double checksum;
}
