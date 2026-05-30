/// Physics Engine — 2D
///
/// Simulates realistic movement, gravity, collision detection, and object interactions.
library;

import 'dart:isolate';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_memory/just_memory.dart';
import 'ray_2d.dart';
import '../box2d/_box2d_engine_native.dart'
    if (dart.library.html) '../box2d/_box2d_engine_stub.dart';

part 'collision_manifold.dart';
part 'collision_shapes.dart';
part 'physics_body.dart';
part 'spatial_grid.dart';
part 'joint_2d.dart';

/// Main physics engine class
class PhysicsEngine {
  /// Create the best backend for the current platform.
  ///
  /// Native platforms resolve to the Box2D backend.
  /// Web resolves to the pure-Dart backend.
  factory PhysicsEngine() => createPhysicsEngine();

  /// Explicit pure-Dart backend constructor.
  ///
  /// Use this when you intentionally want the Dart implementation regardless
  /// of platform selection.
  PhysicsEngine.pureDart({
    this.experimentalArenaEnabled = true,
    this.experimentalContactWarmStartEnabled = true,
    int experimentalContactVelocityIterations = 1,
    this.experimentalContactTwoPointBlockNormalSolveEnabled = false,
    this.experimentalContactTwoPointBlockFrictionSolveEnabled = false,
    int experimentalContactTwoPointBlockNormalSolveMinWarmStates = 0,
    int experimentalContactTwoPointBlockFrictionSolveMinWarmStates = 0,
    int? experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates,
    int? experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates,
    int experimentalContactBlockHysteresisTransitionRateWindowSteps = 0,
    double experimentalContactWarmStartImpulseDecay = 1.0,
    double? experimentalContactWarmStartNormalImpulseDecay,
    double? experimentalContactWarmStartTangentImpulseDecay,
    double experimentalContactWarmStartMaxImpulse = double.infinity,
    int experimentalContactWarmStartMaxAgeSteps = 2,
    double experimentalContactWarmStartNormalAlignmentThreshold = 0.0,
    double experimentalContactWarmStartAnchorDistanceThreshold =
        double.infinity,
    int experimentalContactWarmStartManifoldSlots = 1,
    this.experimentalContactWarmStartFeatureIdMatchingEnabled = true,
    this.experimentalContactWarmStartAlignmentOverrideForTesting,
    this.experimentalContactWarmStartFeatureIdOverrideForTesting,
    this.deterministicHashEnabled = false,
    int deterministicHashIntervalSteps = 1,
    this.experimentalIsolateBroadphaseEnabled = false,
    this.experimentalIsolateBroadphaseAdaptiveEnabled = false,
    int experimentalIsolateBroadphaseMinBodies = 256,
    int experimentalIsolateBroadphaseDispatchEverySteps = 1,
    double experimentalIsolateBroadphaseTargetStepMs = 16.67,
    int experimentalIsolateBroadphaseAdaptiveCheckIntervalSteps = 30,
    double experimentalIsolateBroadphaseAdaptiveMargin = 0.15,
    int experimentalIsolateBroadphaseAdaptiveMinHoldSteps = 30,
    this.experimentalAdaptiveStepMsOverrideForTesting,
  }) : deterministicHashIntervalSteps = deterministicHashIntervalSteps < 1
           ? 1
           : deterministicHashIntervalSteps,
       experimentalContactWarmStartImpulseDecay =
           experimentalContactWarmStartImpulseDecay.clamp(0.0, 1.0).toDouble(),
       experimentalContactWarmStartNormalImpulseDecay =
           (experimentalContactWarmStartNormalImpulseDecay ??
                   experimentalContactWarmStartImpulseDecay)
               .clamp(0.0, 1.0)
               .toDouble(),
       experimentalContactWarmStartTangentImpulseDecay =
           (experimentalContactWarmStartTangentImpulseDecay ??
                   experimentalContactWarmStartImpulseDecay)
               .clamp(0.0, 1.0)
               .toDouble(),
       experimentalContactVelocityIterations =
           experimentalContactVelocityIterations < 1
           ? 1
           : (experimentalContactVelocityIterations > 8
                 ? 8
                 : experimentalContactVelocityIterations),
       experimentalContactTwoPointBlockNormalSolveMinWarmStates =
           experimentalContactTwoPointBlockNormalSolveMinWarmStates < 0
           ? 0
           : (experimentalContactTwoPointBlockNormalSolveMinWarmStates > 4
                 ? 4
                 : experimentalContactTwoPointBlockNormalSolveMinWarmStates),
       experimentalContactTwoPointBlockFrictionSolveMinWarmStates =
           experimentalContactTwoPointBlockFrictionSolveMinWarmStates < 0
           ? 0
           : (experimentalContactTwoPointBlockFrictionSolveMinWarmStates > 4
                 ? 4
                 : experimentalContactTwoPointBlockFrictionSolveMinWarmStates),
       experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates =
           ((experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates ??
                   experimentalContactTwoPointBlockNormalSolveMinWarmStates) <
               0)
           ? 0
           : ((experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates ??
                         experimentalContactTwoPointBlockNormalSolveMinWarmStates) >
                     4
                 ? 4
                 : (experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates ??
                       experimentalContactTwoPointBlockNormalSolveMinWarmStates)),
       experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates =
           ((experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates ??
                   experimentalContactTwoPointBlockFrictionSolveMinWarmStates) <
               0)
           ? 0
           : ((experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates ??
                         experimentalContactTwoPointBlockFrictionSolveMinWarmStates) >
                     4
                 ? 4
                 : (experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates ??
                       experimentalContactTwoPointBlockFrictionSolveMinWarmStates)),
       experimentalContactBlockHysteresisTransitionRateWindowSteps =
           experimentalContactBlockHysteresisTransitionRateWindowSteps < 0
           ? 0
           : (experimentalContactBlockHysteresisTransitionRateWindowSteps > 600
                 ? 600
                 : experimentalContactBlockHysteresisTransitionRateWindowSteps),
       experimentalContactWarmStartMaxImpulse =
           experimentalContactWarmStartMaxImpulse.isNaN ||
               experimentalContactWarmStartMaxImpulse < 0
           ? 0.0
           : experimentalContactWarmStartMaxImpulse,
       experimentalContactWarmStartMaxAgeSteps =
           experimentalContactWarmStartMaxAgeSteps < 1
           ? 1
           : experimentalContactWarmStartMaxAgeSteps,
       experimentalContactWarmStartNormalAlignmentThreshold =
           experimentalContactWarmStartNormalAlignmentThreshold
               .clamp(0.0, 1.0)
               .toDouble(),
       experimentalContactWarmStartAnchorDistanceThreshold =
           experimentalContactWarmStartAnchorDistanceThreshold.isNaN ||
               experimentalContactWarmStartAnchorDistanceThreshold < 0
           ? 0.0
           : experimentalContactWarmStartAnchorDistanceThreshold,
       experimentalContactWarmStartManifoldSlots =
           experimentalContactWarmStartManifoldSlots < 1
           ? 1
           : (experimentalContactWarmStartManifoldSlots > 4
                 ? 4
                 : experimentalContactWarmStartManifoldSlots),
       experimentalIsolateBroadphaseMinBodies =
           experimentalIsolateBroadphaseMinBodies < 2
           ? 2
           : experimentalIsolateBroadphaseMinBodies,
       experimentalIsolateBroadphaseDispatchEverySteps =
           experimentalIsolateBroadphaseDispatchEverySteps < 1
           ? 1
           : experimentalIsolateBroadphaseDispatchEverySteps,
       experimentalIsolateBroadphaseTargetStepMs =
           experimentalIsolateBroadphaseTargetStepMs <= 0
           ? 16.67
           : experimentalIsolateBroadphaseTargetStepMs,
       experimentalIsolateBroadphaseAdaptiveCheckIntervalSteps =
           experimentalIsolateBroadphaseAdaptiveCheckIntervalSteps < 1
           ? 1
           : experimentalIsolateBroadphaseAdaptiveCheckIntervalSteps,
       experimentalIsolateBroadphaseAdaptiveMargin =
           experimentalIsolateBroadphaseAdaptiveMargin < 0
           ? 0.0
           : experimentalIsolateBroadphaseAdaptiveMargin,
       experimentalIsolateBroadphaseAdaptiveMinHoldSteps =
           experimentalIsolateBroadphaseAdaptiveMinHoldSteps < 1
           ? 1
           : experimentalIsolateBroadphaseAdaptiveMinHoldSteps;

  /// Experimental just_memory-backed state mirroring for hot-loop integration.
  ///
  /// This is Phase 2 scaffolding: positions/velocities/angles for dynamic
  /// bodies are mirrored into [MemoryArena], integrated there, and written
  /// back to [PhysicsBody] each step.
  final bool experimentalArenaEnabled;

  /// Enables deterministic snapshot hashing in [stats] after each update.
  ///
  /// Disabled by default because hashing all bodies each frame has a cost.
  final bool deterministicHashEnabled;

  /// Enables contact impulse persistence (warm-start) for repeated contacts.
  final bool experimentalContactWarmStartEnabled;

  /// Number of sequential velocity-solve iterations applied per contact point.
  final int experimentalContactVelocityIterations;

  /// Enables an additional coupled 2-point normal solve for edge contacts.
  final bool experimentalContactTwoPointBlockNormalSolveEnabled;

  /// Enables an additional coupled 2-point friction solve for edge contacts.
  final bool experimentalContactTwoPointBlockFrictionSolveEnabled;

  /// Minimum matched warm-start states required before running block normal solve.
  final int experimentalContactTwoPointBlockNormalSolveMinWarmStates;

  /// Minimum matched warm-start states required before running block friction solve.
  final int experimentalContactTwoPointBlockFrictionSolveMinWarmStates;

  /// Matched warm-start count below which an active block normal solve disables.
  final int experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates;

  /// Matched warm-start count below which an active block friction solve disables.
  final int experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates;

  /// Rolling window size (in steps) for hysteresis transition rates.
  ///
  /// A value of 0 disables rolling rate metrics.
  final int experimentalContactBlockHysteresisTransitionRateWindowSteps;

  /// Multiplier applied to cached impulses when storing warm-start state.
  ///
  /// 1.0 keeps full impulse, lower values gradually decay cached impulses.
  final double experimentalContactWarmStartImpulseDecay;

  /// Optional normal-impulse-specific warm-start decay multiplier.
  ///
  /// Defaults to [experimentalContactWarmStartImpulseDecay] when not provided.
  final double experimentalContactWarmStartNormalImpulseDecay;

  /// Optional tangent-impulse-specific warm-start decay multiplier.
  ///
  /// Defaults to [experimentalContactWarmStartImpulseDecay] when not provided.
  final double experimentalContactWarmStartTangentImpulseDecay;

  /// Absolute cap applied to cached normal/tangent impulses.
  final double experimentalContactWarmStartMaxImpulse;

  /// Maximum age (in simulation steps) to keep a warm-start contact cached.
  final int experimentalContactWarmStartMaxAgeSteps;

  /// Minimum normal dot-product required to reuse cached contact impulses.
  final double experimentalContactWarmStartNormalAlignmentThreshold;

  /// Maximum allowed anchor mismatch distance for warm-start reuse.
  final double experimentalContactWarmStartAnchorDistanceThreshold;

  /// Number of cached warm-start states retained per body pair.
  final int experimentalContactWarmStartManifoldSlots;

  /// Enables feature-id matching when selecting cached warm-start states.
  final bool experimentalContactWarmStartFeatureIdMatchingEnabled;

  /// Optional test hook to override computed warm-start normal alignment.
  @visibleForTesting
  final double Function(int simulationStep, double computedAlignment)?
  experimentalContactWarmStartAlignmentOverrideForTesting;

  /// Optional test hook to override computed warm-start manifold feature id.
  @visibleForTesting
  final int? Function(int simulationStep, int? computedFeatureId)?
  experimentalContactWarmStartFeatureIdOverrideForTesting;

  /// Hash cadence in simulation steps when [deterministicHashEnabled] is true.
  ///
  /// Example: 1 = every step, 5 = every fifth step.
  final int deterministicHashIntervalSteps;

  /// Experimental broad-phase pipeline selector.
  ///
  /// Currently runs synchronously while exposing the API/stats shape needed
  /// for later isolate-based offload.
  final bool experimentalIsolateBroadphaseEnabled;

  /// Enables adaptive isolate broadphase selection based on measured step time.
  final bool experimentalIsolateBroadphaseAdaptiveEnabled;

  /// Minimum collider count before isolate broadphase is eligible.
  final int experimentalIsolateBroadphaseMinBodies;

  /// Dispatch cadence for isolate broadphase jobs (in simulation steps).
  final int experimentalIsolateBroadphaseDispatchEverySteps;

  /// Adaptive isolate target step time in milliseconds.
  final double experimentalIsolateBroadphaseTargetStepMs;

  /// Adaptive decision cadence in simulation steps.
  final int experimentalIsolateBroadphaseAdaptiveCheckIntervalSteps;

  /// Adaptive hysteresis margin around target step time.
  final double experimentalIsolateBroadphaseAdaptiveMargin;

  /// Minimum simulation steps to hold current adaptive isolate state.
  final int experimentalIsolateBroadphaseAdaptiveMinHoldSteps;

  /// Optional test hook to override measured step time before adaptive EMA.
  ///
  /// When null, adaptive policy uses real measured step time.
  @visibleForTesting
  final double Function(int simulationStep, double measuredStepMs)?
  experimentalAdaptiveStepMsOverrideForTesting;

  /// All physics bodies — list preserves insertion order for deterministic iteration.
  final List<PhysicsBody> _bodies = [];

  /// Set mirror of [_bodies] for O(1) duplicate-check in [addBody].
  final Set<PhysicsBody> _bodySet = {};

  /// Arena slots for dynamic bodies when [experimentalArenaEnabled] is true.
  final Map<PhysicsBody, int> _arenaSlots = {};
  MemoryArena? _stateArena;
  bool _arenaDirty = true;

  final _IsolateBroadphaseWorker _broadphaseWorker = _IsolateBroadphaseWorker();

  /// Global gravity vector. Default: 981 units/s² (9.81 m/s² at 1 unit = 1 cm),
  /// matching the Box2D backend so both simulate identically.
  final Vector2 gravity = Vector2(0, 981.0);

  /// Whether debug rendering is enabled
  bool debugRender = false;

  /// Sub-frame interpolation alpha in [0, 1) for render-smooth positioning.
  ///
  /// Always 0.0 on the pure-Dart backend (no fixed-step accumulator).
  /// [Box2DPhysicsEngine] overrides this with the native accumulator remainder.
  double get alpha => 0.0;

  // ── Sensor state ──────────────────────────────────────────────────────────
  // Tracks which sensor pairs are currently overlapping (uses identity keys).
  final Set<BodyPair> _activeSensorPairs = {};
  final List<BodyPair> _sensorBeginBuffer = [];
  final List<BodyPair> _sensorEndBuffer = [];

  /// Initialize the physics engine
  void initialize() {
    debugPrint('Physics Engine initialized');
  }

  /// Set world gravity, updating both the Dart gravity vector and any native
  /// simulation state. Override in backend-specific subclasses.
  void setGravity(double gx, double gy) {
    gravity.setValues(gx, gy);
  }

  // ── Scratch vectors for the update loop (avoids per-frame allocation) ──
  final Vector2 _accel = Vector2.zero();

  int _lastPotentialPairCount = 0;
  int _lastResolvedCollisionCount = 0;
  int _lastResolvedManifoldPointCount = 0;
  int _lastResolvedVelocityIterationCount = 0;
  int _lastResolvedBlockSolveCount = 0;
  int _lastResolvedBlockFrictionSolveCount = 0;
  int _lastBlockNormalHysteresisActiveContacts = 0;
  int _lastBlockFrictionHysteresisActiveContacts = 0;
  int _lastBlockNormalHysteresisActivationCount = 0;
  int _lastBlockNormalHysteresisDeactivationCount = 0;
  int _lastBlockFrictionHysteresisActivationCount = 0;
  int _lastBlockFrictionHysteresisDeactivationCount = 0;
  int _lastBlockNormalHysteresisActivatedByThresholdCount = 0;
  int _lastBlockNormalHysteresisDeactivatedBelowDisableCount = 0;
  int _lastBlockNormalHysteresisDeactivatedNonTwoPointCount = 0;
  int _lastBlockNormalHysteresisDeactivatedNonTwoPointContactCount = 0;
  int _lastBlockNormalHysteresisDeactivatedPairDroppedCount = 0;
  int _lastBlockFrictionHysteresisActivatedByThresholdCount = 0;
  int _lastBlockFrictionHysteresisDeactivatedBelowDisableCount = 0;
  int _lastBlockFrictionHysteresisDeactivatedNonTwoPointCount = 0;
  int _lastBlockFrictionHysteresisDeactivatedNonTwoPointContactCount = 0;
  int _lastBlockFrictionHysteresisDeactivatedPairDroppedCount = 0;
  int _totalBlockNormalHysteresisActivationCount = 0;
  int _totalBlockNormalHysteresisDeactivationCount = 0;
  int _totalBlockFrictionHysteresisActivationCount = 0;
  int _totalBlockFrictionHysteresisDeactivationCount = 0;
  final List<({int nAct, int nDeact, int fAct, int fDeact})>
  _blockHysteresisTransitionWindow =
      <({int nAct, int nDeact, int fAct, int fDeact})>[];
  int _rollingBlockNormalHysteresisActivationCount = 0;
  int _rollingBlockNormalHysteresisDeactivationCount = 0;
  int _rollingBlockFrictionHysteresisActivationCount = 0;
  int _rollingBlockFrictionHysteresisDeactivationCount = 0;
  int _lastAwakeBodyCount = 0;
  int _lastBroadphaseDirtyBodyCount = 0;
  int _lastTrackedCellCount = 0;
  double _lastStepMs = 0.0;
  int _lastDeterministicHash = 0;
  int _lastDeterministicHashStep = 0;
  int _simulationStep = 0;
  String _lastBroadphasePipeline = 'main_thread';
  int _lastBroadphaseWorkerPairCount = 0;
  bool _lastBroadphaseWorkerInFlight = false;
  int _lastBroadphaseSerializedBodies = 0;
  int _lastBroadphaseSerializedBytes = 0;
  int _lastBroadphaseEligibleBodies = 0;
  int _lastJointWakeActivations = 0;
  int _lastWarmStartContactCount = 0;
  int _lastWarmStartHitCount = 0;
  int _lastWarmStartRejectedByAgeCount = 0;
  int _lastWarmStartRejectedByNormalCount = 0;
  int _lastWarmStartRejectedByAnchorCount = 0;
  int _lastWarmStartRejectedByFeatureCount = 0;
  int _lastWarmStartMatchedStateCount = 0;
  int _lastWarmStartFeatureHistoryReuseCount = 0;
  int _lastWarmStartResolvedFeatureSeedCount = 0;
  int _lastWarmStartAngularPreloadCount = 0;
  double _lastWarmStartMaxCachedNormalImpulse = 0.0;
  double _lastWarmStartMaxCachedTangentImpulse = 0.0;

  final Map<BodyPair, List<_ContactWarmStartState>> _contactWarmStartCache = {};
  final Map<BodyPair, _BlockSolveHysteresisState> _blockSolveHysteresisByPair =
      {};

  bool _adaptiveIsolateActive = false;
  double _adaptiveAvgStepMs = 0.0;
  String _adaptiveDecisionReason = 'not_evaluated';
  int _adaptiveLastTransitionStep = 0;
  int _adaptiveTransitionCount = 0;

  // Persistent Stopwatch instance — reused every frame to avoid heap allocation.
  final Stopwatch _stepStopwatch = Stopwatch();

  /// Update physics simulation
  void update(double deltaTime) {
    _stepStopwatch
      ..reset()
      ..start();
    _simulationStep++;
    _rebuildArenaIfNeeded();
    _lastJointWakeActivations = 0;
    _propagateJointWakefulness();
    var awakeBodyCount = 0;
    _lastResolvedCollisionCount = 0;

    // Update all bodies
    for (final body in _bodies) {
      if (body.isActive) {
        if (body.isAwake) {
          awakeBodyCount++;
          // Calculate total acceleration for this frame (in-place)
          _accel.setFrom(body.acceleration);
          if (body.useGravity) {
            _accel.add(gravity);
          }

          // Check for sleeping
          if (body.velocity.lengthSquared <
                  body.sleepVelocityThreshold * body.sleepVelocityThreshold &&
              _accel.lengthSquared < 0.1) {
            body.sleepTimer += deltaTime;
            if (body.sleepTimer >= body.sleepTimeThreshold) {
              body.isAwake = false;
              body.velocity.setZero();
              body.acceleration.setZero();
            }
          } else {
            body.sleepTimer = 0.0;
          }

          if (body.isAwake) {
            if (body.mass <= 0) continue; // static — never integrate

            final slot = _arenaSlots[body];
            if (slot != null && _stateArena != null) {
              final arena = _stateArena!;
              var velocityX = arena.getVelocityX(slot);
              var velocityY = arena.getVelocityY(slot);
              var angularVelocity = arena.getValue(
                slot,
                MemoryArena.offsetExtra,
              );

              // Semi-implicit Euler using arena-backed scalar state.
              velocityX += _accel.x * deltaTime;
              velocityY += _accel.y * deltaTime;
              angularVelocity +=
                  (body.torque * body.inverseInertia) * deltaTime;

              final dragFactor = 1.0 - body.drag * deltaTime;
              velocityX *= dragFactor;
              velocityY *= dragFactor;
              angularVelocity *= dragFactor;

              final nextX = arena.getX(slot) + velocityX * deltaTime;
              final nextY = arena.getY(slot) + velocityY * deltaTime;
              final nextAngle =
                  arena.getRotation(slot) + angularVelocity * deltaTime;

              arena.setPosition(slot, nextX, nextY);
              arena.setVelocity(slot, velocityX, velocityY);
              arena.setRotation(slot, nextAngle);
              arena.setValue(slot, MemoryArena.offsetExtra, angularVelocity);

              // Write-through keeps external gameplay/ECS observers unchanged.
              body.position.x = nextX;
              body.position.y = nextY;
              body.velocity.x = velocityX;
              body.velocity.y = velocityY;
              body.angle = nextAngle;
              body.angularVelocity = angularVelocity;

              body.acceleration.setZero();
              body.torque = 0.0;
              continue;
            }

            // Semi-Implicit Euler Integration — all in-place Vec2 ops
            // 1. Update velocity: v += accel * dt
            body.velocity.addScaled(_accel, deltaTime);
            body.angularVelocity +=
                (body.torque * body.inverseInertia) * deltaTime;

            // Apply drag (simple linear drag)
            final dragFactor = 1.0 - body.drag * deltaTime;
            body.velocity.scale(dragFactor);
            body.angularVelocity *= dragFactor;

            // 2. Update position: x += v * dt
            body.position.addScaled(body.velocity, deltaTime);
            body.angle += body.angularVelocity * deltaTime;

            // Reset acceleration for the next frame
            body.acceleration.setZero();
            body.torque = 0.0;
          }
        }
      }
    }

    // Simple collision detection
    _detectCollisions();

    // Keep connected islands awake when contacts wake one member.
    _propagateJointWakefulness();

    // Apply joint constraints after collision resolution.
    for (final joint in _joints) {
      joint.applyConstraint(deltaTime);
    }

    // Body move events (Dart fallback: track awake transitions).
    _moveEventBuffer.clear();
    for (final body in _bodies) {
      if (body.mass <= 0) continue; // static bodies don't move
      final wasAwake = _prevAwake[body] ?? true;
      final fellAsleep = wasAwake && !body.isAwake;
      if (body.isAwake || fellAsleep) {
        _moveEventBuffer.add((body: body, fellAsleep: fellAsleep));
      }
      _prevAwake[body] = body.isAwake;
    }

    _recordBlockHysteresisTransitionWindowSample();

    _stepStopwatch.stop();
    _lastAwakeBodyCount = awakeBodyCount;
    _lastStepMs = _stepStopwatch.elapsedMicroseconds / 1000.0;
    _updateAdaptiveBroadphaseDecision();
    if (deterministicHashEnabled &&
        _simulationStep % deterministicHashIntervalSteps == 0) {
      _lastDeterministicHash = _computeDeterministicHash();
      _lastDeterministicHashStep = _simulationStep;
    }
  }

  void _wakeBody(PhysicsBody body) {
    if (body.mass <= 0 || !body.isActive || body.isAwake) return;
    body.isAwake = true;
    body.sleepTimer = 0.0;
    _lastJointWakeActivations++;
  }

  void _propagateWakePair(PhysicsBody a, PhysicsBody b) {
    if (!a.isActive || !b.isActive) return;
    if (a.mass <= 0 || b.mass <= 0) return;
    if (a.isAwake && !b.isAwake) {
      _wakeBody(b);
    } else if (b.isAwake && !a.isAwake) {
      _wakeBody(a);
    }
  }

  void _propagateJointWakefulness() {
    if (_joints.isEmpty) return;

    var changed = true;
    var passBudget = _joints.length;

    while (changed && passBudget-- > 0) {
      changed = false;
      for (final joint in _joints) {
        final prevActivations = _lastJointWakeActivations;

        switch (joint) {
          case DistanceJoint():
            _propagateWakePair(joint.bodyA, joint.bodyB);
          case WeldJoint():
            _propagateWakePair(joint.bodyA, joint.bodyB);
          case PrismaticJoint():
            _propagateWakePair(joint.bodyA, joint.bodyB);
          case RevoluteJoint():
            _propagateWakePair(joint.bodyA, joint.bodyB);
          case MouseJoint():
            // Mouse joints already keep their single body awake internally.
            continue;
          default:
            continue;
        }

        if (_lastJointWakeActivations != prevActivations) {
          changed = true;
        }
      }
    }
  }

  /// Add a physics body
  void addBody(PhysicsBody body) {
    if (_bodySet.add(body)) {
      _bodies.add(body);
      _arenaDirty = true;
    }
  }

  /// Remove a physics body
  void removeBody(PhysicsBody body) {
    if (_bodySet.remove(body)) {
      _bodies.remove(body);
      _grid.removeBody(body);
      _arenaSlots.remove(body);
      _arenaDirty = true;
    }
  }

  void _rebuildArenaIfNeeded() {
    if (!experimentalArenaEnabled) return;
    if (!_arenaDirty && _stateArena != null) return;

    _arenaSlots.clear();

    final dynamicBodies = <PhysicsBody>[];
    for (final body in _bodies) {
      if (body.mass > 0) {
        dynamicBodies.add(body);
      }
    }

    if (dynamicBodies.isEmpty) {
      _stateArena = null;
      _arenaDirty = false;
      return;
    }

    final arena = MemoryArena(capacity: dynamicBodies.length);
    for (final body in dynamicBodies) {
      final slot = arena.allocate();
      if (slot < 0) continue;
      _arenaSlots[body] = slot;
      arena.setPosition(slot, body.position.x, body.position.y);
      arena.setVelocity(slot, body.velocity.x, body.velocity.y);
      arena.setRotation(slot, body.angle);
      arena.setValue(slot, MemoryArena.offsetExtra, body.angularVelocity);
    }

    _stateArena = arena;
    _arenaDirty = false;
  }

  int _computeDeterministicHash() {
    var hash = 0xcbf29ce484222325;
    for (final body in _bodies) {
      final px = (body.position.x * 1000).round();
      final py = (body.position.y * 1000).round();
      final vx = (body.velocity.x * 1000).round();
      final vy = (body.velocity.y * 1000).round();

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

  /// Broad-phase grid
  final SpatialGrid _grid = SpatialGrid(100.0);

  /// Returns true if two bodies should interact (collision filter + group index).
  bool _shouldBodiesCollide(PhysicsBody a, PhysicsBody b) {
    if (a.groupIndex != 0 && a.groupIndex == b.groupIndex) {
      return a.groupIndex > 0; // same positive group → always; negative → never
    }
    return (a.categoryBits & b.maskBits) != 0 &&
        (b.categoryBits & a.maskBits) != 0;
  }

  /// Detect collisions
  void _detectCollisions() {
    final potentialPairs = _getPotentialCollisions();
    _lastPotentialPairCount = potentialPairs.length;
    _lastWarmStartContactCount = 0;
    _lastWarmStartHitCount = 0;
    _lastWarmStartRejectedByAgeCount = 0;
    _lastWarmStartRejectedByNormalCount = 0;
    _lastWarmStartRejectedByAnchorCount = 0;
    _lastWarmStartRejectedByFeatureCount = 0;
    _lastWarmStartMatchedStateCount = 0;
    _lastWarmStartFeatureHistoryReuseCount = 0;
    _lastWarmStartResolvedFeatureSeedCount = 0;
    _lastWarmStartAngularPreloadCount = 0;
    _lastWarmStartMaxCachedNormalImpulse = 0.0;
    _lastWarmStartMaxCachedTangentImpulse = 0.0;
    _lastResolvedManifoldPointCount = 0;
    _lastResolvedVelocityIterationCount = 0;
    _lastResolvedBlockSolveCount = 0;
    _lastResolvedBlockFrictionSolveCount = 0;
    _lastBlockNormalHysteresisActiveContacts = 0;
    _lastBlockFrictionHysteresisActiveContacts = 0;
    _lastBlockNormalHysteresisActivationCount = 0;
    _lastBlockNormalHysteresisDeactivationCount = 0;
    _lastBlockFrictionHysteresisActivationCount = 0;
    _lastBlockFrictionHysteresisDeactivationCount = 0;
    _lastBlockNormalHysteresisActivatedByThresholdCount = 0;
    _lastBlockNormalHysteresisDeactivatedBelowDisableCount = 0;
    _lastBlockNormalHysteresisDeactivatedNonTwoPointCount = 0;
    _lastBlockNormalHysteresisDeactivatedNonTwoPointContactCount = 0;
    _lastBlockNormalHysteresisDeactivatedPairDroppedCount = 0;
    _lastBlockFrictionHysteresisActivatedByThresholdCount = 0;
    _lastBlockFrictionHysteresisDeactivatedBelowDisableCount = 0;
    _lastBlockFrictionHysteresisDeactivatedNonTwoPointCount = 0;
    _lastBlockFrictionHysteresisDeactivatedNonTwoPointContactCount = 0;
    _lastBlockFrictionHysteresisDeactivatedPairDroppedCount = 0;

    // Reset sensor buffers for this step.
    _sensorBeginBuffer.clear();
    _sensorEndBuffer.clear();
    final previousSensorPairs = Set<BodyPair>.from(_activeSensorPairs);
    final currentSensorPairs = <BodyPair>{};
    final currentCollisionPairs = <BodyPair>{};

    for (final pair in potentialPairs) {
      final bodyA = pair.a;
      final bodyB = pair.b;

      if (!_shouldBodiesCollide(bodyA, bodyB)) continue;

      final posA = bodyA.position.toOffset();
      final posB = bodyB.position.toOffset();

      // Collect all shapes for each body (primary + additional).
      final shapesA = [bodyA.shape, ...bodyA.additionalShapes];
      final shapesB = [bodyB.shape, ...bodyB.additionalShapes];

      // Find the deepest colliding manifold across all shape combinations.
      CollisionManifold? best;
      for (final sA in shapesA) {
        for (final sB in shapesB) {
          final m = sA.getManifold(posA, sB, posB);
          if (m.isColliding) {
            if (best == null || m.penetration > best.penetration) best = m;
          }
        }
      }

      if (best == null) continue;

      final eitherSensor = bodyA.isSensor || bodyB.isSensor;
      if (eitherSensor) {
        currentSensorPairs.add(pair);
        if (!previousSensorPairs.contains(pair)) {
          _sensorBeginBuffer.add(pair);
        }
      } else {
        _lastWarmStartContactCount++;
        final contactPoints = best.contactPoints.isNotEmpty
            ? best.contactPoints
            : (best.contactPoint == null
                  ? const <CollisionContactPoint>[]
                  : <CollisionContactPoint>[
                      CollisionContactPoint(
                        point: best.contactPoint!,
                        featureId: best.contactFeatureId,
                      ),
                    ]);
        final currentFeatureIds = contactPoints
            .map((cp) => cp.featureId)
            .whereType<int>()
            .toSet()
            .toList(growable: false);
        final warmStates = _selectWarmStartStates(
          pair,
          best.normal,
          bodyA,
          bodyB,
          contactPoints: contactPoints,
          currentFeatureIds: currentFeatureIds,
        );
        _lastWarmStartMatchedStateCount += warmStates.length;
        if (warmStates.isNotEmpty) _lastWarmStartHitCount++;
        currentCollisionPairs.add(pair);

        final solverPointCountForGate = contactPoints.isNotEmpty
            ? contactPoints.length
            : (best.contactPoint == null ? 0 : 1);
        final isTwoPointContact = solverPointCountForGate == 2;
        final blockSolveState = _blockSolveHysteresisByPair.putIfAbsent(
          pair,
          () => _BlockSolveHysteresisState(),
        );
        blockSolveState.lastSeenStep = _simulationStep;

        var blockNormalSolveAllowed = false;
        var blockFrictionSolveAllowed = false;
        final wasBlockNormalSolveActive = blockSolveState.normalActive;
        final wasBlockFrictionSolveActive = blockSolveState.frictionActive;
        if (isTwoPointContact) {
          blockNormalSolveAllowed = _updateBlockSolveHysteresisActive(
            currentlyActive: blockSolveState.normalActive,
            matchedWarmStateCount: warmStates.length,
            enableAtWarmStates:
                experimentalContactTwoPointBlockNormalSolveMinWarmStates,
            disableBelowWarmStates:
                experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates,
          );
          blockFrictionSolveAllowed = _updateBlockSolveHysteresisActive(
            currentlyActive: blockSolveState.frictionActive,
            matchedWarmStateCount: warmStates.length,
            enableAtWarmStates:
                experimentalContactTwoPointBlockFrictionSolveMinWarmStates,
            disableBelowWarmStates:
                experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates,
          );
        }

        if (!wasBlockNormalSolveActive && blockNormalSolveAllowed) {
          _lastBlockNormalHysteresisActivationCount++;
          _lastBlockNormalHysteresisActivatedByThresholdCount++;
        } else if (wasBlockNormalSolveActive && !blockNormalSolveAllowed) {
          _lastBlockNormalHysteresisDeactivationCount++;
          if (isTwoPointContact) {
            _lastBlockNormalHysteresisDeactivatedBelowDisableCount++;
          } else {
            _lastBlockNormalHysteresisDeactivatedNonTwoPointCount++;
            _lastBlockNormalHysteresisDeactivatedNonTwoPointContactCount++;
          }
        }
        if (!wasBlockFrictionSolveActive && blockFrictionSolveAllowed) {
          _lastBlockFrictionHysteresisActivationCount++;
          _lastBlockFrictionHysteresisActivatedByThresholdCount++;
        } else if (wasBlockFrictionSolveActive && !blockFrictionSolveAllowed) {
          _lastBlockFrictionHysteresisDeactivationCount++;
          if (isTwoPointContact) {
            _lastBlockFrictionHysteresisDeactivatedBelowDisableCount++;
          } else {
            _lastBlockFrictionHysteresisDeactivatedNonTwoPointCount++;
            _lastBlockFrictionHysteresisDeactivatedNonTwoPointContactCount++;
          }
        }

        blockSolveState.normalActive = blockNormalSolveAllowed;
        blockSolveState.frictionActive = blockFrictionSolveAllowed;
        if (blockNormalSolveAllowed) {
          _lastBlockNormalHysteresisActiveContacts++;
        }
        if (blockFrictionSolveAllowed) {
          _lastBlockFrictionHysteresisActiveContacts++;
        }

        _lastResolvedCollisionCount++;
        _resolveCollision(
          bodyA,
          bodyB,
          best,
          warmStates: warmStates,
          blockNormalSolveAllowed: blockNormalSolveAllowed,
          blockFrictionSolveAllowed: blockFrictionSolveAllowed,
        );

        if (experimentalContactWarmStartEnabled) {
          final normalLen = math.sqrt(
            best.normal.dx * best.normal.dx + best.normal.dy * best.normal.dy,
          );
          final normalX = normalLen > 1e-9 ? best.normal.dx / normalLen : 0.0;
          final normalY = normalLen > 1e-9 ? best.normal.dy / normalLen : 0.0;
          final cacheContacts = contactPoints.isNotEmpty
              ? contactPoints
              : <CollisionContactPoint>[
                  CollisionContactPoint(
                    point: Offset(
                      (bodyA.position.x + bodyB.position.x) * 0.5,
                      (bodyA.position.y + bodyB.position.y) * 0.5,
                    ),
                    featureId: best.contactFeatureId,
                  ),
                ];
          for (final cp in cacheContacts) {
            final contactX = cp.point.dx;
            final contactY = cp.point.dy;
            final featureId =
                experimentalContactWarmStartFeatureIdOverrideForTesting?.call(
                  _simulationStep,
                  cp.featureId,
                ) ??
                cp.featureId;

            final priorFeatureState = _findWarmStartStateByFeature(
              pair,
              featureId,
            );
            final resolvedFeatureImpulse =
                _lastResolvedImpulsesByFeature[featureId];
            final sourceNormalImpulse =
                priorFeatureState?.normalImpulse ??
                resolvedFeatureImpulse?.normalImpulse ??
                _lastResolvedNormalImpulse;
            final sourceTangentImpulse =
                priorFeatureState?.tangentImpulse ??
                resolvedFeatureImpulse?.tangentImpulse ??
                _lastResolvedTangentImpulse;
            if (priorFeatureState != null) {
              _lastWarmStartFeatureHistoryReuseCount++;
            } else if (resolvedFeatureImpulse != null) {
              _lastWarmStartResolvedFeatureSeedCount++;
            }

            final featureCachedNormal = _clampWarmStartNormalImpulse(
              sourceNormalImpulse *
                  experimentalContactWarmStartNormalImpulseDecay,
            );
            final featureCachedTangent = _clampWarmStartTangentImpulse(
              sourceTangentImpulse *
                  experimentalContactWarmStartTangentImpulseDecay,
            );

            final cached = _ContactWarmStartState(
              normalImpulse: featureCachedNormal,
              tangentImpulse: featureCachedTangent,
              normalX: normalX,
              normalY: normalY,
              localAnchorAX: contactX - bodyA.position.x,
              localAnchorAY: contactY - bodyA.position.y,
              localAnchorBX: contactX - bodyB.position.x,
              localAnchorBY: contactY - bodyB.position.y,
              contactFeatureId: featureId,
              lastSeenStep: _simulationStep,
            );
            _upsertWarmStartState(pair, cached);
            _recordWarmStartCachePeak(cached);
          }
        }
      }
    }

    if (experimentalContactWarmStartEnabled) {
      _contactWarmStartCache.removeWhere((pair, cachedList) {
        cachedList.removeWhere(
          (cached) =>
              (_simulationStep - cached.lastSeenStep) >
              experimentalContactWarmStartMaxAgeSteps,
        );
        return cachedList.isEmpty;
      });
      for (final cachedList in _contactWarmStartCache.values) {
        for (final cached in cachedList) {
          _recordWarmStartCachePeak(cached);
        }
      }
    } else {
      _contactWarmStartCache.clear();
    }

    final stalePairs = <BodyPair>[];
    _blockSolveHysteresisByPair.forEach((pair, state) {
      if (!currentCollisionPairs.contains(pair)) {
        if (state.normalActive) {
          _lastBlockNormalHysteresisDeactivationCount++;
          _lastBlockNormalHysteresisDeactivatedNonTwoPointCount++;
          _lastBlockNormalHysteresisDeactivatedPairDroppedCount++;
        }
        if (state.frictionActive) {
          _lastBlockFrictionHysteresisDeactivationCount++;
          _lastBlockFrictionHysteresisDeactivatedNonTwoPointCount++;
          _lastBlockFrictionHysteresisDeactivatedPairDroppedCount++;
        }
        stalePairs.add(pair);
      }
    });
    for (final pair in stalePairs) {
      _blockSolveHysteresisByPair.remove(pair);
    }

    // Any previously active sensor pair no longer overlapping → end event.
    for (final old in previousSensorPairs) {
      if (!currentSensorPairs.contains(old)) {
        _sensorEndBuffer.add(old);
      }
    }
    _activeSensorPairs
      ..clear()
      ..addAll(currentSensorPairs);
  }

  List<BodyPair> _getPotentialCollisions() {
    final isolateConfigured =
        experimentalIsolateBroadphaseEnabled ||
        experimentalIsolateBroadphaseAdaptiveEnabled;

    if (!isolateConfigured) {
      _grid.syncBodies(_bodies);
      _lastBroadphaseDirtyBodyCount = _grid.dirtyBodyCount;
      _lastTrackedCellCount = _grid.trackedCellCount;
      _lastBroadphasePipeline = 'main_thread';
      _lastBroadphaseWorkerPairCount = 0;
      _lastBroadphaseWorkerInFlight = false;
      _lastBroadphaseSerializedBodies = 0;
      _lastBroadphaseSerializedBytes = 0;
      _lastBroadphaseEligibleBodies = 0;
      _adaptiveDecisionReason = 'disabled';
      return _grid.getPotentialCollisions();
    }

    final eligibleBodies = _countEligibleBroadphaseBodies();
    _lastBroadphaseEligibleBodies = eligibleBodies;
    final aboveMinBodies =
        eligibleBodies >= experimentalIsolateBroadphaseMinBodies;
    final dispatchStep =
        _simulationStep % experimentalIsolateBroadphaseDispatchEverySteps == 0;

    if (!aboveMinBodies) {
      _grid.syncBodies(_bodies);
      _lastBroadphasePipeline = 'isolate_disabled_small_world';
      _lastBroadphaseDirtyBodyCount = _grid.dirtyBodyCount;
      _lastTrackedCellCount = _grid.trackedCellCount;
      _lastBroadphaseWorkerPairCount = 0;
      _lastBroadphaseWorkerInFlight = false;
      _lastBroadphaseSerializedBodies = 0;
      _lastBroadphaseSerializedBytes = 0;
      _adaptiveDecisionReason = 'below_min_bodies';
      return _grid.getPotentialCollisions();
    }

    if (_isolateBlockedByAdaptivePolicy()) {
      _grid.syncBodies(_bodies);
      _lastBroadphasePipeline = 'adaptive_disabled_main_thread';
      _lastBroadphaseDirtyBodyCount = _grid.dirtyBodyCount;
      _lastTrackedCellCount = _grid.trackedCellCount;
      _lastBroadphaseWorkerPairCount = 0;
      _lastBroadphaseWorkerInFlight = false;
      _lastBroadphaseSerializedBodies = 0;
      _lastBroadphaseSerializedBytes = 0;
      return _grid.getPotentialCollisions();
    }

    final usedWorkerPairs = _broadphaseWorker.consumePairs(_bodies);
    final scheduled = dispatchStep
        ? _broadphaseWorker.scheduleIfIdle(_bodies, _grid.cellSize)
        : false;

    _lastBroadphaseWorkerInFlight = _broadphaseWorker.isInFlight;
    _lastBroadphaseSerializedBodies = _broadphaseWorker.lastSerializedBodies;
    _lastBroadphaseSerializedBytes = _broadphaseWorker.lastSerializedBytes;
    _lastBroadphaseDirtyBodyCount = -1;
    _lastTrackedCellCount = -1;

    if (usedWorkerPairs != null) {
      _lastBroadphasePipeline = 'isolate_worker';
      _lastBroadphaseWorkerPairCount = usedWorkerPairs.length;
      return usedWorkerPairs;
    }

    // Before the first isolate result arrives, keep simulation functional.
    _grid.syncBodies(_bodies);
    if (!dispatchStep && !_broadphaseWorker.isInFlight) {
      _lastBroadphasePipeline = 'isolate_waiting_dispatch';
    } else {
      _lastBroadphasePipeline = scheduled
          ? 'isolate_fallback_main_thread'
          : 'main_thread_fallback';
    }
    _lastBroadphaseDirtyBodyCount = _grid.dirtyBodyCount;
    _lastTrackedCellCount = _grid.trackedCellCount;
    _lastBroadphaseWorkerPairCount = 0;
    return _grid.getPotentialCollisions();
  }

  bool _isolateBlockedByAdaptivePolicy() {
    if (!experimentalIsolateBroadphaseAdaptiveEnabled) return false;
    return !_adaptiveIsolateActive;
  }

  void _updateAdaptiveBroadphaseDecision() {
    if (!experimentalIsolateBroadphaseAdaptiveEnabled) return;

    final observedStepMs =
        experimentalAdaptiveStepMsOverrideForTesting?.call(
          _simulationStep,
          _lastStepMs,
        ) ??
        _lastStepMs;

    if (_adaptiveAvgStepMs == 0.0) {
      _adaptiveAvgStepMs = observedStepMs;
    } else {
      // Exponential moving average for stable decisions.
      _adaptiveAvgStepMs = _adaptiveAvgStepMs * 0.85 + observedStepMs * 0.15;
    }

    if (_simulationStep %
            experimentalIsolateBroadphaseAdaptiveCheckIntervalSteps !=
        0) {
      return;
    }

    final hi =
        experimentalIsolateBroadphaseTargetStepMs *
        (1.0 + experimentalIsolateBroadphaseAdaptiveMargin);
    final lo =
        experimentalIsolateBroadphaseTargetStepMs *
        (1.0 - experimentalIsolateBroadphaseAdaptiveMargin);

    final allowFirstTransition = _adaptiveTransitionCount == 0;
    final heldLongEnough =
        (_simulationStep - _adaptiveLastTransitionStep) >=
        experimentalIsolateBroadphaseAdaptiveMinHoldSteps;
    final canTransition = allowFirstTransition || heldLongEnough;

    if (_adaptiveAvgStepMs > hi) {
      if (!_adaptiveIsolateActive && canTransition) {
        _adaptiveIsolateActive = true;
        _adaptiveLastTransitionStep = _simulationStep;
        _adaptiveTransitionCount++;
        _adaptiveDecisionReason = 'above_target';
      } else if (!_adaptiveIsolateActive && !canTransition) {
        _adaptiveDecisionReason = 'hold_locked';
      } else {
        _adaptiveDecisionReason = 'above_target';
      }
    } else if (_adaptiveAvgStepMs < lo) {
      if (_adaptiveIsolateActive && canTransition) {
        _adaptiveIsolateActive = false;
        _adaptiveLastTransitionStep = _simulationStep;
        _adaptiveTransitionCount++;
        _adaptiveDecisionReason = 'below_target';
      } else if (_adaptiveIsolateActive && !canTransition) {
        _adaptiveDecisionReason = 'hold_locked';
      } else {
        _adaptiveDecisionReason = 'below_target';
      }
    } else {
      _adaptiveDecisionReason = 'within_band';
    }
  }

  int _countEligibleBroadphaseBodies() {
    var count = 0;
    for (final body in _bodies) {
      if (body.isActive && body.checkCollision) count++;
    }
    return count;
  }

  /// Iterate sensor-enter events from the last step.
  ///
  /// [fn] is called for each (bodyA, bodyB) pair that started overlapping.
  void pollSensorBeginEvents(void Function(PhysicsBody a, PhysicsBody b) fn) {
    for (final pair in _sensorBeginBuffer) {
      fn(pair.a, pair.b);
    }
  }

  /// Iterate sensor-exit events from the last step.
  ///
  /// [fn] is called for each (bodyA, bodyB) pair that stopped overlapping.
  void pollSensorEndEvents(void Function(PhysicsBody a, PhysicsBody b) fn) {
    for (final pair in _sensorEndBuffer) {
      fn(pair.a, pair.b);
    }
  }

  /// Resolve collision
  double _lastResolvedNormalImpulse = 0.0;
  double _lastResolvedTangentImpulse = 0.0;
  Map<int?, _ResolvedContactImpulse> _lastResolvedImpulsesByFeature =
      const <int?, _ResolvedContactImpulse>{};

  double _clampWarmStartNormalImpulse(double impulse) {
    final maxImpulse = experimentalContactWarmStartMaxImpulse;
    if (maxImpulse.isInfinite) return impulse < 0 ? 0.0 : impulse;
    return impulse.clamp(0.0, maxImpulse).toDouble();
  }

  double _clampWarmStartTangentImpulse(double impulse) {
    final maxImpulse = experimentalContactWarmStartMaxImpulse;
    if (maxImpulse.isInfinite) return impulse;
    return impulse.clamp(-maxImpulse, maxImpulse).toDouble();
  }

  _ContactWarmStartState? _selectWarmStartState(
    BodyPair pair,
    Offset currentNormal,
    PhysicsBody bodyA,
    PhysicsBody bodyB, {
    required List<int> currentFeatureIds,
  }) {
    if (!experimentalContactWarmStartEnabled) return null;
    final cachedList = _contactWarmStartCache[pair];
    if (cachedList == null || cachedList.isEmpty) return null;

    final currentLen = math.sqrt(
      currentNormal.dx * currentNormal.dx + currentNormal.dy * currentNormal.dy,
    );
    if (currentLen <= 1e-9) {
      _lastWarmStartRejectedByNormalCount++;
      return null;
    }
    final currentNx = currentNormal.dx / currentLen;
    final currentNy = currentNormal.dy / currentLen;
    final effectiveFeatureIds = currentFeatureIds
        .map(
          (featureId) =>
              experimentalContactWarmStartFeatureIdOverrideForTesting?.call(
                _simulationStep,
                featureId,
              ) ??
              featureId,
        )
        .whereType<int>()
        .toSet();

    var sawAgeReject = false;
    var sawNormalReject = false;
    var sawAnchorReject = false;
    var sawFeatureReject = false;
    _ContactWarmStartState? bestCandidate;
    var bestAnchorDistance = double.infinity;

    for (final cached in cachedList) {
      final age = _simulationStep - cached.lastSeenStep;
      if (age > experimentalContactWarmStartMaxAgeSteps) {
        sawAgeReject = true;
        continue;
      }

      final rawAlignment =
          cached.normalX * currentNx + cached.normalY * currentNy;
      final alignment =
          experimentalContactWarmStartAlignmentOverrideForTesting?.call(
            _simulationStep,
            rawAlignment,
          ) ??
          rawAlignment;
      if (alignment < experimentalContactWarmStartNormalAlignmentThreshold) {
        sawNormalReject = true;
        continue;
      }

      if (experimentalContactWarmStartFeatureIdMatchingEnabled &&
          effectiveFeatureIds.isNotEmpty &&
          cached.contactFeatureId != null &&
          !effectiveFeatureIds.contains(cached.contactFeatureId)) {
        sawFeatureReject = true;
        continue;
      }

      final anchorDistance = _computeWarmStartAnchorDistance(
        cached,
        bodyA,
        bodyB,
      );
      if (anchorDistance >
          experimentalContactWarmStartAnchorDistanceThreshold) {
        sawAnchorReject = true;
        continue;
      }

      if (anchorDistance < bestAnchorDistance) {
        bestAnchorDistance = anchorDistance;
        bestCandidate = cached;
      }
    }

    if (bestCandidate != null) return bestCandidate;

    if (sawFeatureReject) {
      _lastWarmStartRejectedByFeatureCount++;
    } else if (sawAnchorReject) {
      _lastWarmStartRejectedByAnchorCount++;
    } else if (sawNormalReject) {
      _lastWarmStartRejectedByNormalCount++;
    } else if (sawAgeReject) {
      _lastWarmStartRejectedByAgeCount++;
    }
    return null;
  }

  List<_ContactWarmStartState> _selectWarmStartStates(
    BodyPair pair,
    Offset currentNormal,
    PhysicsBody bodyA,
    PhysicsBody bodyB, {
    required List<CollisionContactPoint> contactPoints,
    required List<int> currentFeatureIds,
  }) {
    if (!experimentalContactWarmStartEnabled) {
      return const <_ContactWarmStartState>[];
    }

    final states = <_ContactWarmStartState>[];
    final seen = <_ContactWarmStartState>{};

    if (contactPoints.isNotEmpty) {
      for (final cp in contactPoints) {
        final effectiveFeatureId =
            experimentalContactWarmStartFeatureIdOverrideForTesting?.call(
              _simulationStep,
              cp.featureId,
            ) ??
            cp.featureId;
        final featureScope = effectiveFeatureId == null
            ? currentFeatureIds
            : <int>[effectiveFeatureId];
        final state = _selectWarmStartState(
          pair,
          currentNormal,
          bodyA,
          bodyB,
          currentFeatureIds: featureScope,
        );
        if (state == null || seen.contains(state)) continue;
        seen.add(state);
        states.add(state);
      }
    }

    if (states.isEmpty) {
      final fallback = _selectWarmStartState(
        pair,
        currentNormal,
        bodyA,
        bodyB,
        currentFeatureIds: currentFeatureIds,
      );
      if (fallback != null) {
        states.add(fallback);
      }
    }

    return states;
  }

  void _upsertWarmStartState(BodyPair pair, _ContactWarmStartState state) {
    final cachedList = _contactWarmStartCache.putIfAbsent(
      pair,
      () => <_ContactWarmStartState>[],
    );

    final mergeThreshold =
        experimentalContactWarmStartAnchorDistanceThreshold.isInfinite
        ? 4.0
        : math.max(1.0, experimentalContactWarmStartAnchorDistanceThreshold);

    var mergeIndex = -1;
    var bestDelta = double.infinity;
    for (var i = 0; i < cachedList.length; i++) {
      if (experimentalContactWarmStartFeatureIdMatchingEnabled &&
          cachedList[i].contactFeatureId != null &&
          state.contactFeatureId != null &&
          cachedList[i].contactFeatureId != state.contactFeatureId) {
        continue;
      }

      final alignment = _computeCachedStateNormalAlignment(
        cachedList[i],
        state,
      );
      if (alignment < 0.95) continue;

      final delta = _computeCachedStateAnchorDelta(cachedList[i], state);
      if (delta < bestDelta) {
        bestDelta = delta;
        mergeIndex = i;
      }
    }

    if (mergeIndex >= 0 && bestDelta <= mergeThreshold) {
      cachedList[mergeIndex] = state;
    } else {
      cachedList.add(state);
      if (cachedList.length > experimentalContactWarmStartManifoldSlots) {
        var oldestIndex = 0;
        var oldestStep = cachedList[0].lastSeenStep;
        for (var i = 1; i < cachedList.length; i++) {
          if (cachedList[i].lastSeenStep < oldestStep) {
            oldestStep = cachedList[i].lastSeenStep;
            oldestIndex = i;
          }
        }
        cachedList.removeAt(oldestIndex);
      }
    }
  }

  _ContactWarmStartState? _findWarmStartStateByFeature(
    BodyPair pair,
    int? featureId,
  ) {
    if (featureId == null) return null;
    final cachedList = _contactWarmStartCache[pair];
    if (cachedList == null || cachedList.isEmpty) return null;

    for (final cached in cachedList) {
      if (cached.contactFeatureId == featureId) {
        return cached;
      }
    }
    return null;
  }

  double _computeCachedStateAnchorDelta(
    _ContactWarmStartState a,
    _ContactWarmStartState b,
  ) {
    final forwardAx = a.localAnchorAX - b.localAnchorAX;
    final forwardAy = a.localAnchorAY - b.localAnchorAY;
    final forwardBx = a.localAnchorBX - b.localAnchorBX;
    final forwardBy = a.localAnchorBY - b.localAnchorBY;
    final forward = math.sqrt(
      forwardAx * forwardAx +
          forwardAy * forwardAy +
          forwardBx * forwardBx +
          forwardBy * forwardBy,
    );

    final reverseAx = a.localAnchorAX - b.localAnchorBX;
    final reverseAy = a.localAnchorAY - b.localAnchorBY;
    final reverseBx = a.localAnchorBX - b.localAnchorAX;
    final reverseBy = a.localAnchorBY - b.localAnchorAY;
    final reverse = math.sqrt(
      reverseAx * reverseAx +
          reverseAy * reverseAy +
          reverseBx * reverseBx +
          reverseBy * reverseBy,
    );

    return math.min(forward, reverse);
  }

  double _computeCachedStateNormalAlignment(
    _ContactWarmStartState a,
    _ContactWarmStartState b,
  ) {
    final direct = a.normalX * b.normalX + a.normalY * b.normalY;
    return direct.abs();
  }

  double _computeWarmStartAnchorDistance(
    _ContactWarmStartState cached,
    PhysicsBody bodyA,
    PhysicsBody bodyB,
  ) {
    final aForwardX = bodyA.position.x + cached.localAnchorAX;
    final aForwardY = bodyA.position.y + cached.localAnchorAY;
    final bForwardX = bodyB.position.x + cached.localAnchorBX;
    final bForwardY = bodyB.position.y + cached.localAnchorBY;
    final forwardDx = aForwardX - bForwardX;
    final forwardDy = aForwardY - bForwardY;
    final forwardDist = math.sqrt(
      forwardDx * forwardDx + forwardDy * forwardDy,
    );

    final aReverseX = bodyA.position.x + cached.localAnchorBX;
    final aReverseY = bodyA.position.y + cached.localAnchorBY;
    final bReverseX = bodyB.position.x + cached.localAnchorAX;
    final bReverseY = bodyB.position.y + cached.localAnchorAY;
    final reverseDx = aReverseX - bReverseX;
    final reverseDy = aReverseY - bReverseY;
    final reverseDist = math.sqrt(
      reverseDx * reverseDx + reverseDy * reverseDy,
    );

    return math.min(forwardDist, reverseDist);
  }

  void _recordWarmStartCachePeak(_ContactWarmStartState state) {
    if (state.normalImpulse > _lastWarmStartMaxCachedNormalImpulse) {
      _lastWarmStartMaxCachedNormalImpulse = state.normalImpulse;
    }
    final tangentAbs = state.tangentImpulse.abs();
    if (tangentAbs > _lastWarmStartMaxCachedTangentImpulse) {
      _lastWarmStartMaxCachedTangentImpulse = tangentAbs;
    }
  }

  bool _updateBlockSolveHysteresisActive({
    required bool currentlyActive,
    required int matchedWarmStateCount,
    required int enableAtWarmStates,
    required int disableBelowWarmStates,
  }) {
    if (currentlyActive) {
      return matchedWarmStateCount >= disableBelowWarmStates;
    }
    return matchedWarmStateCount >= enableAtWarmStates;
  }

  void _recordBlockHysteresisTransitionWindowSample() {
    _totalBlockNormalHysteresisActivationCount +=
        _lastBlockNormalHysteresisActivationCount;
    _totalBlockNormalHysteresisDeactivationCount +=
        _lastBlockNormalHysteresisDeactivationCount;
    _totalBlockFrictionHysteresisActivationCount +=
        _lastBlockFrictionHysteresisActivationCount;
    _totalBlockFrictionHysteresisDeactivationCount +=
        _lastBlockFrictionHysteresisDeactivationCount;

    if (experimentalContactBlockHysteresisTransitionRateWindowSteps <= 0) {
      _blockHysteresisTransitionWindow.clear();
      _rollingBlockNormalHysteresisActivationCount = 0;
      _rollingBlockNormalHysteresisDeactivationCount = 0;
      _rollingBlockFrictionHysteresisActivationCount = 0;
      _rollingBlockFrictionHysteresisDeactivationCount = 0;
      return;
    }

    _blockHysteresisTransitionWindow.add((
      nAct: _lastBlockNormalHysteresisActivationCount,
      nDeact: _lastBlockNormalHysteresisDeactivationCount,
      fAct: _lastBlockFrictionHysteresisActivationCount,
      fDeact: _lastBlockFrictionHysteresisDeactivationCount,
    ));

    _rollingBlockNormalHysteresisActivationCount +=
        _lastBlockNormalHysteresisActivationCount;
    _rollingBlockNormalHysteresisDeactivationCount +=
        _lastBlockNormalHysteresisDeactivationCount;
    _rollingBlockFrictionHysteresisActivationCount +=
        _lastBlockFrictionHysteresisActivationCount;
    _rollingBlockFrictionHysteresisDeactivationCount +=
        _lastBlockFrictionHysteresisDeactivationCount;

    while (_blockHysteresisTransitionWindow.length >
        experimentalContactBlockHysteresisTransitionRateWindowSteps) {
      final evicted = _blockHysteresisTransitionWindow.removeAt(0);
      _rollingBlockNormalHysteresisActivationCount -= evicted.nAct;
      _rollingBlockNormalHysteresisDeactivationCount -= evicted.nDeact;
      _rollingBlockFrictionHysteresisActivationCount -= evicted.fAct;
      _rollingBlockFrictionHysteresisDeactivationCount -= evicted.fDeact;
    }
  }

  void _resolveCollision(
    PhysicsBody a,
    PhysicsBody b,
    CollisionManifold manifold, {
    List<_ContactWarmStartState> warmStates = const <_ContactWarmStartState>[],
    bool blockNormalSolveAllowed = false,
    bool blockFrictionSolveAllowed = false,
  }) {
    final normal = manifold.normal;
    final penetration = manifold.penetration;
    _lastResolvedNormalImpulse = 0.0;
    _lastResolvedTangentImpulse = 0.0;
    _lastResolvedImpulsesByFeature = const <int?, _ResolvedContactImpulse>{};

    if (penetration <= 0) return;

    // Only wake bodies for significant collisions
    if (penetration > 0.05 ||
        a.velocity.lengthSquared > 1.0 ||
        b.velocity.lengthSquared > 1.0) {
      a.isAwake = true;
      a.sleepTimer = 0.0;
      b.isAwake = true;
      b.sleepTimer = 0.0;
    }

    // ── Positional correction (mass-proportional) ─────────────────────────
    final inverseMassSum = a.inverseMass + b.inverseMass;

    if (inverseMassSum == 0) return; // both immovable

    if (warmStates.isNotEmpty) {
      final warmNx = normal.dx;
      final warmNy = normal.dy;
      final warmTx = -warmNy;
      final warmTy = warmNx;

      final warmScale = 1.0 / warmStates.length;
      for (final warmState in warmStates) {
        final warmNormalImpulse =
            _clampWarmStartNormalImpulse(warmState.normalImpulse) * warmScale;
        final warmTangentImpulse =
            _clampWarmStartTangentImpulse(warmState.tangentImpulse) * warmScale;

        final pnX = warmNx * warmNormalImpulse;
        final pnY = warmNy * warmNormalImpulse;
        final ptX = warmTx * warmTangentImpulse;
        final ptY = warmTy * warmTangentImpulse;
        final warmX = pnX + ptX;
        final warmY = pnY + ptY;

        final forwardAX = warmState.localAnchorAX;
        final forwardAY = warmState.localAnchorAY;
        final forwardBX = warmState.localAnchorBX;
        final forwardBY = warmState.localAnchorBY;
        final forwardDx =
            (a.position.x + forwardAX) - (b.position.x + forwardBX);
        final forwardDy =
            (a.position.y + forwardAY) - (b.position.y + forwardBY);
        final forwardDistSq = forwardDx * forwardDx + forwardDy * forwardDy;

        final reverseAX = warmState.localAnchorBX;
        final reverseAY = warmState.localAnchorBY;
        final reverseBX = warmState.localAnchorAX;
        final reverseBY = warmState.localAnchorAY;
        final reverseDx =
            (a.position.x + reverseAX) - (b.position.x + reverseBX);
        final reverseDy =
            (a.position.y + reverseAY) - (b.position.y + reverseBY);
        final reverseDistSq = reverseDx * reverseDx + reverseDy * reverseDy;

        final useReverse = reverseDistSq < forwardDistSq;
        final raX = useReverse ? reverseAX : forwardAX;
        final raY = useReverse ? reverseAY : forwardAY;
        final rbX = useReverse ? reverseBX : forwardBX;
        final rbY = useReverse ? reverseBY : forwardBY;

        a.velocity.x -= warmX * a.inverseMass;
        a.velocity.y -= warmY * a.inverseMass;
        b.velocity.x += warmX * b.inverseMass;
        b.velocity.y += warmY * b.inverseMass;

        final torqueA = (raX * warmY - raY * warmX) * a.inverseInertia;
        final torqueB = (rbX * warmY - rbY * warmX) * b.inverseInertia;
        a.angularVelocity -= torqueA;
        b.angularVelocity += torqueB;

        if (torqueA.abs() > 1e-9 || torqueB.abs() > 1e-9) {
          _lastWarmStartAngularPreloadCount++;
        }
      }
    }

    const correctionPercent = 0.8;
    const slop = 0.05;
    final correctionMag =
        math.max(penetration - slop, 0.0) / inverseMassSum * correctionPercent;
    a.position.x -= normal.dx * correctionMag * a.inverseMass;
    a.position.y -= normal.dy * correctionMag * a.inverseMass;
    b.position.x += normal.dx * correctionMag * b.inverseMass;
    b.position.y += normal.dy * correctionMag * b.inverseMass;

    final solveContacts = manifold.contactPoints.isNotEmpty
        ? manifold.contactPoints
        : <CollisionContactPoint>[
            CollisionContactPoint(
              point:
                  manifold.contactPoint ??
                  Offset(
                    (a.position.x + b.position.x) * 0.5,
                    (a.position.y + b.position.y) * 0.5,
                  ),
              featureId: manifold.contactFeatureId,
            ),
          ];
    final solverPointCount = solveContacts.length;
    _lastResolvedManifoldPointCount += solverPointCount;

    // ── Impulse resolution ────────────────────────────────────────────────
    final restitution = math.min(a.restitution, b.restitution);
    final mu = (a.friction + b.friction) / 2.0;
    var accumulatedNormalImpulse = 0.0;
    var accumulatedTangentImpulse = 0.0;
    final resolvedImpulsesByFeature = <int?, _ResolvedContactImpulse>{};
    final accumulatedNormalLambdas = List<double>.filled(solverPointCount, 0.0);
    final accumulatedTangentLambdas = List<double>.filled(
      solverPointCount,
      0.0,
    );
    final velocityIterations = experimentalContactVelocityIterations;
    final blockNormalSolveEnabledForContact =
        blockNormalSolveAllowed && solveContacts.length == 2;
    final blockFrictionSolveEnabledForContact =
        blockFrictionSolveAllowed && solveContacts.length == 2;

    for (var iteration = 0; iteration < velocityIterations; iteration++) {
      final effectiveRestitution = iteration == 0 ? restitution : 0.0;
      for (var pointIndex = 0; pointIndex < solverPointCount; pointIndex++) {
        final contact = solveContacts[pointIndex];
        _lastResolvedVelocityIterationCount++;
        final raX = contact.point.dx - a.position.x;
        final raY = contact.point.dy - a.position.y;
        final rbX = contact.point.dx - b.position.x;
        final rbY = contact.point.dy - b.position.y;

        final vaX = a.velocity.x + (-a.angularVelocity * raY);
        final vaY = a.velocity.y + (a.angularVelocity * raX);
        final vbX = b.velocity.x + (-b.angularVelocity * rbY);
        final vbY = b.velocity.y + (b.angularVelocity * rbX);

        final rvx = vbX - vaX;
        final rvy = vbY - vaY;
        final velAlongNormal = rvx * normal.dx + rvy * normal.dy;

        if (velAlongNormal > 0) continue; // separating

        final raCrossN = raX * normal.dy - raY * normal.dx;
        final rbCrossN = rbX * normal.dy - rbY * normal.dx;
        final normalMass =
            inverseMassSum +
            (raCrossN * raCrossN) * a.inverseInertia +
            (rbCrossN * rbCrossN) * b.inverseInertia;
        if (normalMass <= 1e-9) continue;

        final rawNormalDelta =
            -(1.0 + effectiveRestitution) *
            velAlongNormal /
            (normalMass * solverPointCount);
        final previousNormalLambda = accumulatedNormalLambdas[pointIndex];
        final nextNormalLambda = math.max(
          0.0,
          previousNormalLambda + rawNormalDelta,
        );
        final j = nextNormalLambda - previousNormalLambda;
        if (j <= 1e-9) continue;
        accumulatedNormalLambdas[pointIndex] = nextNormalLambda;

        final jnx = normal.dx * j;
        final jny = normal.dy * j;
        a.velocity.x -= jnx * a.inverseMass;
        a.velocity.y -= jny * a.inverseMass;
        b.velocity.x += jnx * b.inverseMass;
        b.velocity.y += jny * b.inverseMass;
        a.angularVelocity -= (raX * jny - raY * jnx) * a.inverseInertia;
        b.angularVelocity += (rbX * jny - rbY * jnx) * b.inverseInertia;
        accumulatedNormalImpulse += j;

        final previous = resolvedImpulsesByFeature[contact.featureId];
        if (previous == null) {
          resolvedImpulsesByFeature[contact.featureId] =
              _ResolvedContactImpulse(normalImpulse: j, tangentImpulse: 0.0);
        } else {
          resolvedImpulsesByFeature[contact.featureId] =
              _ResolvedContactImpulse(
                normalImpulse: previous.normalImpulse + j,
                tangentImpulse: previous.tangentImpulse,
              );
        }

        // For 2-point edge contacts, use the optional coupled friction block
        // pass instead of sequential per-point friction.
        if (blockFrictionSolveEnabledForContact) {
          continue;
        }

        // ── Friction (Tangent Impulse) ───────────────────────────────────
        final vaX2 = a.velocity.x + (-a.angularVelocity * raY);
        final vaY2 = a.velocity.y + (a.angularVelocity * raX);
        final vbX2 = b.velocity.x + (-b.angularVelocity * rbY);
        final vbY2 = b.velocity.y + (b.angularVelocity * rbX);
        final rvx2 = vbX2 - vaX2;
        final rvy2 = vbY2 - vaY2;
        final rvDotN = rvx2 * normal.dx + rvy2 * normal.dy;
        var tx = rvx2 - normal.dx * rvDotN;
        var ty = rvy2 - normal.dy * rvDotN;

        final tangentLen = math.sqrt(tx * tx + ty * ty);
        if (tangentLen <= 0.0001) continue;

        final invLen = 1.0 / tangentLen;
        tx *= invLen;
        ty *= invLen;

        final raCrossT = raX * ty - raY * tx;
        final rbCrossT = rbX * ty - rbY * tx;
        final tangentMass =
            inverseMassSum +
            (raCrossT * raCrossT) * a.inverseInertia +
            (rbCrossT * rbCrossT) * b.inverseInertia;
        if (tangentMass <= 1e-9) continue;

        final jt = -(rvx2 * tx + rvy2 * ty) / (tangentMass * solverPointCount);
        final previousTangentLambda = accumulatedTangentLambdas[pointIndex];
        final maxFriction = accumulatedNormalLambdas[pointIndex] * mu;
        final nextTangentLambda = (previousTangentLambda + jt).clamp(
          -maxFriction,
          maxFriction,
        );
        final fScalar = nextTangentLambda - previousTangentLambda;
        if (fScalar.abs() <= 1e-9) continue;
        accumulatedTangentLambdas[pointIndex] = nextTangentLambda;

        a.velocity.x -= tx * fScalar * a.inverseMass;
        a.velocity.y -= ty * fScalar * a.inverseMass;
        b.velocity.x += tx * fScalar * b.inverseMass;
        b.velocity.y += ty * fScalar * b.inverseMass;
        final fX = tx * fScalar;
        final fY = ty * fScalar;
        a.angularVelocity -= (raX * fY - raY * fX) * a.inverseInertia;
        b.angularVelocity += (rbX * fY - rbY * fX) * b.inverseInertia;
        accumulatedTangentImpulse += fScalar;

        final withNormal = resolvedImpulsesByFeature[contact.featureId];
        if (withNormal != null) {
          resolvedImpulsesByFeature[contact.featureId] =
              _ResolvedContactImpulse(
                normalImpulse: withNormal.normalImpulse,
                tangentImpulse: withNormal.tangentImpulse + fScalar,
              );
        }
      }

      if (blockNormalSolveEnabledForContact) {
        final c1 = solveContacts[0];
        final c2 = solveContacts[1];

        final ra1X = c1.point.dx - a.position.x;
        final ra1Y = c1.point.dy - a.position.y;
        final rb1X = c1.point.dx - b.position.x;
        final rb1Y = c1.point.dy - b.position.y;
        final ra2X = c2.point.dx - a.position.x;
        final ra2Y = c2.point.dy - a.position.y;
        final rb2X = c2.point.dx - b.position.x;
        final rb2Y = c2.point.dy - b.position.y;

        final va1X = a.velocity.x + (-a.angularVelocity * ra1Y);
        final va1Y = a.velocity.y + (a.angularVelocity * ra1X);
        final vb1X = b.velocity.x + (-b.angularVelocity * rb1Y);
        final vb1Y = b.velocity.y + (b.angularVelocity * rb1X);
        final va2X = a.velocity.x + (-a.angularVelocity * ra2Y);
        final va2Y = a.velocity.y + (a.angularVelocity * ra2X);
        final vb2X = b.velocity.x + (-b.angularVelocity * rb2Y);
        final vb2Y = b.velocity.y + (b.angularVelocity * rb2X);

        final rv1X = vb1X - va1X;
        final rv1Y = vb1Y - va1Y;
        final rv2X = vb2X - va2X;
        final rv2Y = vb2Y - va2Y;
        final vn1 = rv1X * normal.dx + rv1Y * normal.dy;
        final vn2 = rv2X * normal.dx + rv2Y * normal.dy;

        final ra1CrossN = ra1X * normal.dy - ra1Y * normal.dx;
        final rb1CrossN = rb1X * normal.dy - rb1Y * normal.dx;
        final ra2CrossN = ra2X * normal.dy - ra2Y * normal.dx;
        final rb2CrossN = rb2X * normal.dy - rb2Y * normal.dx;

        final k11 =
            inverseMassSum +
            (ra1CrossN * ra1CrossN) * a.inverseInertia +
            (rb1CrossN * rb1CrossN) * b.inverseInertia;
        final k22 =
            inverseMassSum +
            (ra2CrossN * ra2CrossN) * a.inverseInertia +
            (rb2CrossN * rb2CrossN) * b.inverseInertia;
        final k12 =
            inverseMassSum +
            (ra1CrossN * ra2CrossN) * a.inverseInertia +
            (rb1CrossN * rb2CrossN) * b.inverseInertia;

        final det = k11 * k22 - k12 * k12;
        if (det > 1e-9) {
          final rhs1 = -(1.0 + effectiveRestitution) * vn1 / solverPointCount;
          final rhs2 = -(1.0 + effectiveRestitution) * vn2 / solverPointCount;

          final blockDelta1 = (k22 * rhs1 - k12 * rhs2) / det;
          final blockDelta2 = (k11 * rhs2 - k12 * rhs1) / det;
          final previousLambda1 = accumulatedNormalLambdas[0];
          final previousLambda2 = accumulatedNormalLambdas[1];
          final nextLambda1 = math.max(0.0, previousLambda1 + blockDelta1);
          final nextLambda2 = math.max(0.0, previousLambda2 + blockDelta2);
          final lambda1 = nextLambda1 - previousLambda1;
          final lambda2 = nextLambda2 - previousLambda2;

          if (lambda1 > 1e-9 || lambda2 > 1e-9) {
            _lastResolvedBlockSolveCount++;
            accumulatedNormalLambdas[0] = nextLambda1;
            accumulatedNormalLambdas[1] = nextLambda2;

            final j1x = normal.dx * lambda1;
            final j1y = normal.dy * lambda1;
            a.velocity.x -= j1x * a.inverseMass;
            a.velocity.y -= j1y * a.inverseMass;
            b.velocity.x += j1x * b.inverseMass;
            b.velocity.y += j1y * b.inverseMass;
            a.angularVelocity -= (ra1X * j1y - ra1Y * j1x) * a.inverseInertia;
            b.angularVelocity += (rb1X * j1y - rb1Y * j1x) * b.inverseInertia;

            final j2x = normal.dx * lambda2;
            final j2y = normal.dy * lambda2;
            a.velocity.x -= j2x * a.inverseMass;
            a.velocity.y -= j2y * a.inverseMass;
            b.velocity.x += j2x * b.inverseMass;
            b.velocity.y += j2y * b.inverseMass;
            a.angularVelocity -= (ra2X * j2y - ra2Y * j2x) * a.inverseInertia;
            b.angularVelocity += (rb2X * j2y - rb2Y * j2x) * b.inverseInertia;

            accumulatedNormalImpulse += lambda1 + lambda2;

            final prev1 = resolvedImpulsesByFeature[c1.featureId];
            if (prev1 == null) {
              resolvedImpulsesByFeature[c1.featureId] = _ResolvedContactImpulse(
                normalImpulse: lambda1,
                tangentImpulse: 0.0,
              );
            } else {
              resolvedImpulsesByFeature[c1.featureId] = _ResolvedContactImpulse(
                normalImpulse: prev1.normalImpulse + lambda1,
                tangentImpulse: prev1.tangentImpulse,
              );
            }

            final prev2 = resolvedImpulsesByFeature[c2.featureId];
            if (prev2 == null) {
              resolvedImpulsesByFeature[c2.featureId] = _ResolvedContactImpulse(
                normalImpulse: lambda2,
                tangentImpulse: 0.0,
              );
            } else {
              resolvedImpulsesByFeature[c2.featureId] = _ResolvedContactImpulse(
                normalImpulse: prev2.normalImpulse + lambda2,
                tangentImpulse: prev2.tangentImpulse,
              );
            }
          }
        }
      }

      if (blockFrictionSolveEnabledForContact) {
        final c1 = solveContacts[0];
        final c2 = solveContacts[1];

        final ra1X = c1.point.dx - a.position.x;
        final ra1Y = c1.point.dy - a.position.y;
        final rb1X = c1.point.dx - b.position.x;
        final rb1Y = c1.point.dy - b.position.y;
        final ra2X = c2.point.dx - a.position.x;
        final ra2Y = c2.point.dy - a.position.y;
        final rb2X = c2.point.dx - b.position.x;
        final rb2Y = c2.point.dy - b.position.y;

        final va1X = a.velocity.x + (-a.angularVelocity * ra1Y);
        final va1Y = a.velocity.y + (a.angularVelocity * ra1X);
        final vb1X = b.velocity.x + (-b.angularVelocity * rb1Y);
        final vb1Y = b.velocity.y + (b.angularVelocity * rb1X);
        final va2X = a.velocity.x + (-a.angularVelocity * ra2Y);
        final va2Y = a.velocity.y + (a.angularVelocity * ra2X);
        final vb2X = b.velocity.x + (-b.angularVelocity * rb2Y);
        final vb2Y = b.velocity.y + (b.angularVelocity * rb2X);

        final rv1X = vb1X - va1X;
        final rv1Y = vb1Y - va1Y;
        final rv2X = vb2X - va2X;
        final rv2Y = vb2Y - va2Y;

        final vn1 = rv1X * normal.dx + rv1Y * normal.dy;
        final vn2 = rv2X * normal.dx + rv2Y * normal.dy;
        var tx = (rv1X - normal.dx * vn1) + (rv2X - normal.dx * vn2);
        var ty = (rv1Y - normal.dy * vn1) + (rv2Y - normal.dy * vn2);

        final tangentLenSq = tx * tx + ty * ty;
        if (tangentLenSq <= 1e-9) {
          tx = -normal.dy;
          ty = normal.dx;
        } else {
          final invLen = 1.0 / math.sqrt(tangentLenSq);
          tx *= invLen;
          ty *= invLen;
        }

        final vt1 = rv1X * tx + rv1Y * ty;
        final vt2 = rv2X * tx + rv2Y * ty;

        final ra1CrossT = ra1X * ty - ra1Y * tx;
        final rb1CrossT = rb1X * ty - rb1Y * tx;
        final ra2CrossT = ra2X * ty - ra2Y * tx;
        final rb2CrossT = rb2X * ty - rb2Y * tx;

        final k11 =
            inverseMassSum +
            (ra1CrossT * ra1CrossT) * a.inverseInertia +
            (rb1CrossT * rb1CrossT) * b.inverseInertia;
        final k22 =
            inverseMassSum +
            (ra2CrossT * ra2CrossT) * a.inverseInertia +
            (rb2CrossT * rb2CrossT) * b.inverseInertia;
        final k12 =
            inverseMassSum +
            (ra1CrossT * ra2CrossT) * a.inverseInertia +
            (rb1CrossT * rb2CrossT) * b.inverseInertia;

        final sharedNormalLambda = accumulatedNormalImpulse / solverPointCount;
        final maxFriction1 =
            math.max(accumulatedNormalLambdas[0], sharedNormalLambda) * mu;
        final maxFriction2 =
            math.max(accumulatedNormalLambdas[1], sharedNormalLambda) * mu;

        final det = k11 * k22 - k12 * k12;
        if (det > 1e-9) {
          final rhs1 = -vt1 / solverPointCount;
          final rhs2 = -vt2 / solverPointCount;

          final blockDelta1 = (k22 * rhs1 - k12 * rhs2) / det;
          final blockDelta2 = (k11 * rhs2 - k12 * rhs1) / det;
          final previousLambda1 = accumulatedTangentLambdas[0];
          final previousLambda2 = accumulatedTangentLambdas[1];
          final nextLambda1 = (previousLambda1 + blockDelta1)
              .clamp(-maxFriction1, maxFriction1)
              .toDouble();
          final nextLambda2 = (previousLambda2 + blockDelta2)
              .clamp(-maxFriction2, maxFriction2)
              .toDouble();
          final lambda1 = nextLambda1 - previousLambda1;
          final lambda2 = nextLambda2 - previousLambda2;

          if (lambda1.abs() > 1e-9 || lambda2.abs() > 1e-9) {
            _lastResolvedBlockFrictionSolveCount++;
            accumulatedTangentLambdas[0] = nextLambda1;
            accumulatedTangentLambdas[1] = nextLambda2;

            final j1x = tx * lambda1;
            final j1y = ty * lambda1;
            a.velocity.x -= j1x * a.inverseMass;
            a.velocity.y -= j1y * a.inverseMass;
            b.velocity.x += j1x * b.inverseMass;
            b.velocity.y += j1y * b.inverseMass;
            a.angularVelocity -= (ra1X * j1y - ra1Y * j1x) * a.inverseInertia;
            b.angularVelocity += (rb1X * j1y - rb1Y * j1x) * b.inverseInertia;

            final j2x = tx * lambda2;
            final j2y = ty * lambda2;
            a.velocity.x -= j2x * a.inverseMass;
            a.velocity.y -= j2y * a.inverseMass;
            b.velocity.x += j2x * b.inverseMass;
            b.velocity.y += j2y * b.inverseMass;
            a.angularVelocity -= (ra2X * j2y - ra2Y * j2x) * a.inverseInertia;
            b.angularVelocity += (rb2X * j2y - rb2Y * j2x) * b.inverseInertia;

            accumulatedTangentImpulse += lambda1 + lambda2;

            final prev1 = resolvedImpulsesByFeature[c1.featureId];
            if (prev1 == null) {
              resolvedImpulsesByFeature[c1.featureId] = _ResolvedContactImpulse(
                normalImpulse: 0.0,
                tangentImpulse: lambda1,
              );
            } else {
              resolvedImpulsesByFeature[c1.featureId] = _ResolvedContactImpulse(
                normalImpulse: prev1.normalImpulse,
                tangentImpulse: prev1.tangentImpulse + lambda1,
              );
            }

            final prev2 = resolvedImpulsesByFeature[c2.featureId];
            if (prev2 == null) {
              resolvedImpulsesByFeature[c2.featureId] = _ResolvedContactImpulse(
                normalImpulse: 0.0,
                tangentImpulse: lambda2,
              );
            } else {
              resolvedImpulsesByFeature[c2.featureId] = _ResolvedContactImpulse(
                normalImpulse: prev2.normalImpulse,
                tangentImpulse: prev2.tangentImpulse + lambda2,
              );
            }
          }
        } else {
          var appliedAny = false;
          if (k11 > 1e-9) {
            final previousLambda1 = accumulatedTangentLambdas[0];
            final delta1 = -vt1 / (k11 * solverPointCount);
            final nextLambda1 = (previousLambda1 + delta1)
                .clamp(-maxFriction1, maxFriction1)
                .toDouble();
            final lambda1 = nextLambda1 - previousLambda1;
            if (lambda1.abs() > 1e-9) {
              appliedAny = true;
              accumulatedTangentLambdas[0] = nextLambda1;

              final j1x = tx * lambda1;
              final j1y = ty * lambda1;
              a.velocity.x -= j1x * a.inverseMass;
              a.velocity.y -= j1y * a.inverseMass;
              b.velocity.x += j1x * b.inverseMass;
              b.velocity.y += j1y * b.inverseMass;
              a.angularVelocity -= (ra1X * j1y - ra1Y * j1x) * a.inverseInertia;
              b.angularVelocity += (rb1X * j1y - rb1Y * j1x) * b.inverseInertia;
              accumulatedTangentImpulse += lambda1;

              final prev1 = resolvedImpulsesByFeature[c1.featureId];
              if (prev1 == null) {
                resolvedImpulsesByFeature[c1.featureId] =
                    _ResolvedContactImpulse(
                      normalImpulse: 0.0,
                      tangentImpulse: lambda1,
                    );
              } else {
                resolvedImpulsesByFeature[c1.featureId] =
                    _ResolvedContactImpulse(
                      normalImpulse: prev1.normalImpulse,
                      tangentImpulse: prev1.tangentImpulse + lambda1,
                    );
              }
            }
          }

          if (k22 > 1e-9) {
            final previousLambda2 = accumulatedTangentLambdas[1];
            final delta2 = -vt2 / (k22 * solverPointCount);
            final nextLambda2 = (previousLambda2 + delta2)
                .clamp(-maxFriction2, maxFriction2)
                .toDouble();
            final lambda2 = nextLambda2 - previousLambda2;
            if (lambda2.abs() > 1e-9) {
              appliedAny = true;
              accumulatedTangentLambdas[1] = nextLambda2;

              final j2x = tx * lambda2;
              final j2y = ty * lambda2;
              a.velocity.x -= j2x * a.inverseMass;
              a.velocity.y -= j2y * a.inverseMass;
              b.velocity.x += j2x * b.inverseMass;
              b.velocity.y += j2y * b.inverseMass;
              a.angularVelocity -= (ra2X * j2y - ra2Y * j2x) * a.inverseInertia;
              b.angularVelocity += (rb2X * j2y - rb2Y * j2x) * b.inverseInertia;
              accumulatedTangentImpulse += lambda2;

              final prev2 = resolvedImpulsesByFeature[c2.featureId];
              if (prev2 == null) {
                resolvedImpulsesByFeature[c2.featureId] =
                    _ResolvedContactImpulse(
                      normalImpulse: 0.0,
                      tangentImpulse: lambda2,
                    );
              } else {
                resolvedImpulsesByFeature[c2.featureId] =
                    _ResolvedContactImpulse(
                      normalImpulse: prev2.normalImpulse,
                      tangentImpulse: prev2.tangentImpulse + lambda2,
                    );
              }
            }
          }

          if (appliedAny) {
            _lastResolvedBlockFrictionSolveCount++;
          }
        }
      }
    }

    _lastResolvedImpulsesByFeature = resolvedImpulsesByFeature;
    _lastResolvedNormalImpulse = accumulatedNormalImpulse / solverPointCount;
    _lastResolvedTangentImpulse = accumulatedTangentImpulse / solverPointCount;
  }

  // ── Cached debug paints ────────────────────────────────────────────────
  static final Paint _debugActivePaint = Paint()
    ..color = Colors.green
    ..style = PaintingStyle.stroke
    ..strokeWidth = 2.0;
  static final Paint _debugInactivePaint = Paint()
    ..color = Colors.red
    ..style = PaintingStyle.stroke
    ..strokeWidth = 2.0;
  static final Paint _debugVelocityPaint = Paint()
    ..color = Colors.blue
    ..strokeWidth = 2.0;
  static final Paint _debugCenterPaint = Paint()..color = Colors.red;

  static final Paint _debugExtraShapePaint = Paint()
    ..color = const Color(0xFF00CCFF)
    ..strokeWidth = 1.0;

  /// Render debug visualization
  void renderDebug(Canvas canvas, Size size) {
    if (!debugRender) return;

    for (final body in _bodies) {
      final paint = body.isActive ? _debugActivePaint : _debugInactivePaint;
      _renderBodyShape(canvas, body, body.shape, paint);
      for (final extra in body.additionalShapes) {
        _renderBodyShape(canvas, body, extra, _debugExtraShapePaint);
      }

      if (body.velocity.lengthSquared > 0) {
        canvas.drawLine(
          body.position.toOffset(),
          Offset(
            body.position.x + body.velocity.x * 0.1,
            body.position.y + body.velocity.y * 0.1,
          ),
          _debugVelocityPaint,
        );
      }

      canvas.drawCircle(body.position.toOffset(), 3, _debugCenterPaint);
    }
  }

  void _renderBodyShape(
    Canvas canvas,
    PhysicsBody body,
    CollisionShape shape,
    Paint paint,
  ) {
    if (shape is CircleShape) {
      canvas.drawCircle(body.position.toOffset(), shape.radius, paint);
    } else if (shape is CapsuleShape) {
      final wa1 = Offset(
        body.position.x + shape.center1.dx,
        body.position.y + shape.center1.dy,
      );
      final wa2 = Offset(
        body.position.x + shape.center2.dx,
        body.position.y + shape.center2.dy,
      );
      canvas.drawCircle(wa1, shape.radius, paint);
      canvas.drawCircle(wa2, shape.radius, paint);
      canvas.drawLine(wa1, wa2, paint);
    } else if (shape is ChainShape) {
      for (int i = 0; i < shape.vertices.length - 1; i++) {
        canvas.drawLine(
          Offset(
            body.position.x + shape.vertices[i].dx,
            body.position.y + shape.vertices[i].dy,
          ),
          Offset(
            body.position.x + shape.vertices[i + 1].dx,
            body.position.y + shape.vertices[i + 1].dy,
          ),
          paint,
        );
      }
      if (shape.loop && shape.vertices.length > 1) {
        canvas.drawLine(
          Offset(
            body.position.x + shape.vertices.last.dx,
            body.position.y + shape.vertices.last.dy,
          ),
          Offset(
            body.position.x + shape.vertices.first.dx,
            body.position.y + shape.vertices.first.dy,
          ),
          paint,
        );
      }
    } else if (shape is SegmentShape) {
      canvas.drawLine(
        Offset(
          body.position.x + shape.point1.dx,
          body.position.y + shape.point1.dy,
        ),
        Offset(
          body.position.x + shape.point2.dx,
          body.position.y + shape.point2.dy,
        ),
        paint,
      );
    } else if (shape is PolygonShape) {
      final path = Path();
      if (shape.vertices.isNotEmpty) {
        path.moveTo(
          body.position.x + shape.vertices[0].dx,
          body.position.y + shape.vertices[0].dy,
        );
        for (int i = 1; i < shape.vertices.length; i++) {
          path.lineTo(
            body.position.x + shape.vertices[i].dx,
            body.position.y + shape.vertices[i].dy,
          );
        }
        path.close();
      }
      canvas.drawPath(path, paint);
    }
  }

  /// Clean up physics resources
  void dispose() {
    _bodies.clear();
    _bodySet.clear();
    _arenaSlots.clear();
    _stateArena = null;
    _arenaDirty = true;
    _broadphaseWorker.dispose();
    _simulationStep = 0;
    _lastDeterministicHash = 0;
    _lastDeterministicHashStep = 0;
    _lastBroadphasePipeline = 'main_thread';
    _lastBroadphaseWorkerPairCount = 0;
    _lastBroadphaseWorkerInFlight = false;
    _lastBroadphaseSerializedBodies = 0;
    _lastBroadphaseSerializedBytes = 0;
    _lastBroadphaseEligibleBodies = 0;
    _lastJointWakeActivations = 0;
    _lastWarmStartContactCount = 0;
    _lastWarmStartHitCount = 0;
    _lastWarmStartRejectedByAgeCount = 0;
    _lastWarmStartRejectedByNormalCount = 0;
    _lastWarmStartRejectedByAnchorCount = 0;
    _lastWarmStartRejectedByFeatureCount = 0;
    _lastWarmStartMatchedStateCount = 0;
    _lastWarmStartFeatureHistoryReuseCount = 0;
    _lastWarmStartResolvedFeatureSeedCount = 0;
    _lastWarmStartAngularPreloadCount = 0;
    _lastResolvedManifoldPointCount = 0;
    _lastResolvedVelocityIterationCount = 0;
    _lastResolvedBlockSolveCount = 0;
    _lastResolvedBlockFrictionSolveCount = 0;
    _lastBlockNormalHysteresisActiveContacts = 0;
    _lastBlockFrictionHysteresisActiveContacts = 0;
    _lastBlockNormalHysteresisActivationCount = 0;
    _lastBlockNormalHysteresisDeactivationCount = 0;
    _lastBlockFrictionHysteresisActivationCount = 0;
    _lastBlockFrictionHysteresisDeactivationCount = 0;
    _lastBlockNormalHysteresisActivatedByThresholdCount = 0;
    _lastBlockNormalHysteresisDeactivatedBelowDisableCount = 0;
    _lastBlockNormalHysteresisDeactivatedNonTwoPointCount = 0;
    _lastBlockNormalHysteresisDeactivatedNonTwoPointContactCount = 0;
    _lastBlockNormalHysteresisDeactivatedPairDroppedCount = 0;
    _lastBlockFrictionHysteresisActivatedByThresholdCount = 0;
    _lastBlockFrictionHysteresisDeactivatedBelowDisableCount = 0;
    _lastBlockFrictionHysteresisDeactivatedNonTwoPointCount = 0;
    _lastBlockFrictionHysteresisDeactivatedNonTwoPointContactCount = 0;
    _lastBlockFrictionHysteresisDeactivatedPairDroppedCount = 0;
    _totalBlockNormalHysteresisActivationCount = 0;
    _totalBlockNormalHysteresisDeactivationCount = 0;
    _totalBlockFrictionHysteresisActivationCount = 0;
    _totalBlockFrictionHysteresisDeactivationCount = 0;
    _blockHysteresisTransitionWindow.clear();
    _rollingBlockNormalHysteresisActivationCount = 0;
    _rollingBlockNormalHysteresisDeactivationCount = 0;
    _rollingBlockFrictionHysteresisActivationCount = 0;
    _rollingBlockFrictionHysteresisDeactivationCount = 0;
    _lastWarmStartMaxCachedNormalImpulse = 0.0;
    _lastWarmStartMaxCachedTangentImpulse = 0.0;
    _lastResolvedImpulsesByFeature = const <int?, _ResolvedContactImpulse>{};
    _contactWarmStartCache.clear();
    _blockSolveHysteresisByPair.clear();
    _adaptiveIsolateActive = false;
    _adaptiveAvgStepMs = 0.0;
    _adaptiveDecisionReason = 'not_evaluated';
    _adaptiveLastTransitionStep = 0;
    _adaptiveTransitionCount = 0;
    _grid.clear();
    _activeSensorPairs.clear();
    _sensorBeginBuffer.clear();
    _sensorEndBuffer.clear();
    _joints.clear();
    _prevAwake.clear();
    _moveEventBuffer.clear();
    debugPrint('Physics Engine disposed');
  }

  /// Get all bodies
  List<PhysicsBody> get bodies => List.unmodifiable(_bodies);

  // ── Joint management ─────────────────────────────────────────────────────

  final List<JointConstraint> _joints = [];

  /// Add a joint constraint to the simulation.
  void addJoint(JointConstraint joint) {
    _joints.add(joint);
  }

  /// Remove a joint constraint.
  void removeJoint(JointConstraint joint) {
    _joints.remove(joint);
  }

  /// All active joints (read-only view).
  List<JointConstraint> get joints => List.unmodifiable(_joints);

  // ── Unified joint factory (works on both backends) ────────────────────────
  //
  // Dart fallback: adds the Dart constraint directly.
  // Box2DPhysicsEngine overrides to create native joints instead.

  JointConstraint addRevoluteJoint(
    PhysicsBody a,
    PhysicsBody b,
    Offset worldAnchor,
  ) {
    final j = RevoluteJoint(bodyA: a, bodyB: b, worldAnchor: worldAnchor);
    addJoint(j);
    return j;
  }

  JointConstraint addDistanceJoint(
    PhysicsBody a,
    PhysicsBody b, {
    double? length,
    double? minLength,
    double? maxLength,
    double stiffness = 0.0,
    double damping = 0.3,
  }) {
    final l = length ?? ((b.position - a.position).length);
    final j = DistanceJoint(
      bodyA: a,
      bodyB: b,
      length: l,
      minLength: minLength,
      maxLength: maxLength,
      stiffness: stiffness,
      damping: damping,
    );
    addJoint(j);
    return j;
  }

  JointConstraint addWeldJoint(PhysicsBody a, PhysicsBody b) {
    final j = WeldJoint(bodyA: a, bodyB: b);
    addJoint(j);
    return j;
  }

  JointConstraint addMouseJoint(PhysicsBody b, Vector2 target) {
    final j = MouseJoint(body: b, target: target);
    addJoint(j);
    return j;
  }

  JointConstraint addPrismaticJoint(PhysicsBody a, PhysicsBody b, Offset axis) {
    final j = PrismaticJoint(bodyA: a, bodyB: b, axis: axis);
    addJoint(j);
    return j;
  }

  // ── Body movement events ──────────────────────────────────────────────────

  // Per-body previous awake state for Dart-fallback sleep notifications.
  final Map<PhysicsBody, bool> _prevAwake = {};

  final List<({PhysicsBody body, bool fellAsleep})> _moveEventBuffer = [];

  /// Iterate body-move events from the last step.
  ///
  /// Fires for every dynamic body that moved. [fellAsleep] is true if the body
  /// transitioned to sleep this step.
  void pollBodyMoveEvents(
    void Function(PhysicsBody body, {required bool fellAsleep}) fn,
  ) {
    for (final ev in _moveEventBuffer) {
      fn(ev.body, fellAsleep: ev.fellAsleep);
    }
  }

  // ── Ray casting ───────────────────────────────────────────────────────────

  /// Cast a ray and return the closest hit among all physics bodies.
  ///
  /// [exclude] is an optional set of bodies to skip (e.g. the casting body).
  RayBodyHit? castRay(Ray ray, {Set<PhysicsBody>? exclude}) {
    RayBodyHit? best;
    for (final body in bodies) {
      if (!body.isActive) continue;
      if (exclude != null && exclude.contains(body)) continue;
      final hit = _rayHitBody(ray, body);
      if (hit != null && (best == null || hit.distance < best.distance)) {
        best = hit;
      }
    }
    return best;
  }

  /// Cast a ray and return all hits sorted nearest-first.
  List<RayBodyHit> castRayAll(Ray ray, {Set<PhysicsBody>? exclude}) {
    final hits = <RayBodyHit>[];
    for (final body in bodies) {
      if (!body.isActive) continue;
      if (exclude != null && exclude.contains(body)) continue;
      final hit = _rayHitBody(ray, body);
      if (hit != null) hits.add(hit);
    }
    hits.sort((a, b) => a.distance.compareTo(b.distance));
    return hits;
  }

  // ── Shape → ray intersection helpers ─────────────────────────────────────

  static RayBodyHit? _rayHitBody(Ray ray, PhysicsBody body) {
    final pos = body.position.toOffset();
    final shape = body.shape;

    // AABB quick-reject using the slab method.
    if (!_rayIntersectsAABB(ray, shape.getBounds(pos))) return null;

    if (shape is CapsuleShape) {
      return _rayHitCapsule(ray, pos, shape, body);
    } else if (shape is ChainShape) {
      return _rayHitChain(ray, pos, shape, body);
    } else if (shape is SegmentShape) {
      return _rayHitSegmentShape(ray, pos, shape, body);
    } else if (shape is CircleShape) {
      return _rayHitCircle(ray, pos, shape.radius, body);
    } else if (shape is PolygonShape) {
      return _rayHitPolygonVerts(ray, shape.vertices, pos, body);
    }
    return null;
  }

  static bool _rayIntersectsAABB(Ray ray, Rect rect) {
    double tMin = 0, tMax = ray.maxDistance;

    if (ray.direction.dx.abs() < 1e-10) {
      if (ray.origin.dx < rect.left || ray.origin.dx > rect.right) return false;
    } else {
      final inv = 1.0 / ray.direction.dx;
      var t1 = (rect.left - ray.origin.dx) * inv;
      var t2 = (rect.right - ray.origin.dx) * inv;
      if (t1 > t2) {
        final tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      tMin = math.max(tMin, t1);
      tMax = math.min(tMax, t2);
      if (tMin > tMax) return false;
    }

    if (ray.direction.dy.abs() < 1e-10) {
      if (ray.origin.dy < rect.top || ray.origin.dy > rect.bottom) return false;
    } else {
      final inv = 1.0 / ray.direction.dy;
      var t1 = (rect.top - ray.origin.dy) * inv;
      var t2 = (rect.bottom - ray.origin.dy) * inv;
      if (t1 > t2) {
        final tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      tMin = math.max(tMin, t1);
      tMax = math.min(tMax, t2);
      if (tMin > tMax) return false;
    }

    return true;
  }

  static RayBodyHit? _rayHitCircle(
    Ray ray,
    Offset center,
    double radius,
    PhysicsBody body,
  ) {
    final ocx = ray.origin.dx - center.dx;
    final ocy = ray.origin.dy - center.dy;
    final b = ocx * ray.direction.dx + ocy * ray.direction.dy;
    final c = ocx * ocx + ocy * ocy - radius * radius;
    final disc = b * b - c;
    if (disc < 0) return null;
    final sqrtD = math.sqrt(disc);
    double t = -b - sqrtD;
    if (t < 0) t = -b + sqrtD;
    if (t < 0 || t > ray.maxDistance) return null;
    final point = ray.at(t);
    final dx = point.dx - center.dx;
    final dy = point.dy - center.dy;
    final len = math.sqrt(dx * dx + dy * dy);
    return RayBodyHit(
      body: body,
      point: point,
      normal: len > 1e-8 ? Offset(dx / len, dy / len) : const Offset(0, -1),
      distance: t,
    );
  }

  static RayBodyHit? _rayHitPolygonVerts(
    Ray ray,
    List<Offset> verts,
    Offset pos,
    PhysicsBody body,
  ) {
    double closestT = ray.maxDistance;
    Offset? closestNormal;

    for (int i = 0; i < verts.length; i++) {
      final j = (i + 1) % verts.length;
      final ax = verts[i].dx + pos.dx;
      final ay = verts[i].dy + pos.dy;
      final bx = verts[j].dx + pos.dx;
      final by = verts[j].dy + pos.dy;

      final edgeDx = bx - ax;
      final edgeDy = by - ay;
      final denom = ray.direction.dx * edgeDy - ray.direction.dy * edgeDx;
      if (denom.abs() < 1e-10) continue;

      final dpx = ax - ray.origin.dx;
      final dpy = ay - ray.origin.dy;
      final t = (dpx * edgeDy - dpy * edgeDx) / denom;
      final s = (dpx * ray.direction.dy - dpy * ray.direction.dx) / denom;

      if (t < 0 || t > closestT || s < 0 || s > 1) continue;

      final len = math.sqrt(edgeDx * edgeDx + edgeDy * edgeDy);
      if (len < 1e-8) continue;
      var nx = -edgeDy / len;
      var ny = edgeDx / len;
      // Ensure normal faces toward ray origin (opposite to ray direction).
      if (nx * ray.direction.dx + ny * ray.direction.dy > 0) {
        nx = -nx;
        ny = -ny;
      }
      closestT = t;
      closestNormal = Offset(nx, ny);
    }

    if (closestNormal == null) return null;
    return RayBodyHit(
      body: body,
      point: ray.at(closestT),
      normal: closestNormal,
      distance: closestT,
    );
  }

  static RayBodyHit? _rayHitCapsule(
    Ray ray,
    Offset pos,
    CapsuleShape cap,
    PhysicsBody body,
  ) {
    final p1 = Offset(pos.dx + cap.center1.dx, pos.dy + cap.center1.dy);
    final p2 = Offset(pos.dx + cap.center2.dx, pos.dy + cap.center2.dy);
    final r = cap.radius;

    RayBodyHit? best;

    // Endpoint circles (caps).
    for (final center in [p1, p2]) {
      final h = _rayHitCircle(ray, center, r, body);
      if (h != null && (best == null || h.distance < best.distance)) best = h;
    }

    // Lateral surface of the capsule body.
    final ex = p2.dx - p1.dx;
    final ey = p2.dy - p1.dy;
    final segLen = math.sqrt(ex * ex + ey * ey);
    if (segLen < 1e-8) return best;

    final axX = ex / segLen;
    final axY = ey / segLen;
    // Perpendicular to axis (outward normals of the lateral surface).
    final perX = -axY;
    final perY = axX;

    final ox = ray.origin.dx - p1.dx;
    final oy = ray.origin.dy - p1.dy;
    final oPer = ox * perX + oy * perY;
    final dPer = ray.direction.dx * perX + ray.direction.dy * perY;

    if (dPer.abs() > 1e-10) {
      for (final sign in [1.0, -1.0]) {
        final t = (sign * r - oPer) / dPer;
        if (t < 0 || t > ray.maxDistance) continue;
        if (best != null && t >= best.distance) continue;
        // Check that the hit point is within the segment span.
        final hx = ox + t * ray.direction.dx;
        final hy = oy + t * ray.direction.dy;
        final sAlong = hx * axX + hy * axY;
        if (sAlong < 0 || sAlong > segLen) continue;
        var nx = sign * perX;
        var ny = sign * perY;
        if (nx * ray.direction.dx + ny * ray.direction.dy > 0) {
          nx = -nx;
          ny = -ny;
        }
        best = RayBodyHit(
          body: body,
          point: ray.at(t),
          normal: Offset(nx, ny),
          distance: t,
        );
      }
    }

    return best;
  }

  static RayBodyHit? _rayHitSegmentShape(
    Ray ray,
    Offset pos,
    SegmentShape seg,
    PhysicsBody body,
  ) {
    return _rayHitCapsule(
      ray,
      pos,
      CapsuleShape(
        center1: seg.point1,
        center2: seg.point2,
        radius: seg.thickness,
      ),
      body,
    );
  }

  static RayBodyHit? _rayHitChain(
    Ray ray,
    Offset pos,
    ChainShape chain,
    PhysicsBody body,
  ) {
    if (chain.vertices.length < 2) return null;
    RayBodyHit? best;
    final n = chain.vertices.length;
    final segCount = chain.loop ? n : n - 1;
    for (int i = 0; i < segCount; i++) {
      final seg = SegmentShape(
        chain.vertices[i],
        chain.vertices[(i + 1) % n],
        thickness: chain.thickness,
      );
      final h = _rayHitSegmentShape(ray, pos, seg, body);
      if (h != null && (best == null || h.distance < best.distance)) best = h;
    }
    return best;
  }

  /// Lightweight physics diagnostics from the last simulation step.
  Map<String, dynamic> get stats => {
    'warmStartPairCount': _contactWarmStartCache.length,
    'warmStartStateCount': _contactWarmStartCache.values.fold<int>(
      0,
      (sum, list) => sum + list.length,
    ),
    'bodyCount': _bodies.length,
    'awakeBodies': _lastAwakeBodyCount,
    'potentialPairs': _lastPotentialPairCount,
    'resolvedCollisions': _lastResolvedCollisionCount,
    'broadphaseDirtyBodies': _lastBroadphaseDirtyBodyCount,
    'trackedCells': _lastTrackedCellCount,
    'lastStepMs': _lastStepMs,
    'experimentalArena': experimentalArenaEnabled,
    'experimentalContactWarmStart': experimentalContactWarmStartEnabled,
    'experimentalContactVelocityIterations':
        experimentalContactVelocityIterations,
    'experimentalContactTwoPointBlockNormalSolveEnabled':
        experimentalContactTwoPointBlockNormalSolveEnabled,
    'experimentalContactTwoPointBlockFrictionSolveEnabled':
        experimentalContactTwoPointBlockFrictionSolveEnabled,
    'experimentalContactTwoPointBlockNormalSolveMinWarmStates':
        experimentalContactTwoPointBlockNormalSolveMinWarmStates,
    'experimentalContactTwoPointBlockFrictionSolveMinWarmStates':
        experimentalContactTwoPointBlockFrictionSolveMinWarmStates,
    'experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates':
        experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates,
    'experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates':
        experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates,
    'experimentalContactBlockHysteresisTransitionRateWindowSteps':
        experimentalContactBlockHysteresisTransitionRateWindowSteps,
    'experimentalContactWarmStartImpulseDecay':
        experimentalContactWarmStartImpulseDecay,
    'experimentalContactWarmStartNormalImpulseDecay':
        experimentalContactWarmStartNormalImpulseDecay,
    'experimentalContactWarmStartTangentImpulseDecay':
        experimentalContactWarmStartTangentImpulseDecay,
    'experimentalContactWarmStartMaxImpulse':
        experimentalContactWarmStartMaxImpulse,
    'experimentalContactWarmStartMaxAgeSteps':
        experimentalContactWarmStartMaxAgeSteps,
    'experimentalContactWarmStartNormalAlignmentThreshold':
        experimentalContactWarmStartNormalAlignmentThreshold,
    'experimentalContactWarmStartAnchorDistanceThreshold':
        experimentalContactWarmStartAnchorDistanceThreshold,
    'experimentalContactWarmStartManifoldSlots':
        experimentalContactWarmStartManifoldSlots,
    'experimentalContactWarmStartFeatureIdMatchingEnabled':
        experimentalContactWarmStartFeatureIdMatchingEnabled,
    'arenaTrackedBodies': _arenaSlots.length,
    'experimentalIsolateBroadphase': experimentalIsolateBroadphaseEnabled,
    'experimentalIsolateBroadphaseAdaptiveEnabled':
        experimentalIsolateBroadphaseAdaptiveEnabled,
    'experimentalIsolateBroadphaseMinBodies':
        experimentalIsolateBroadphaseMinBodies,
    'experimentalIsolateBroadphaseDispatchEverySteps':
        experimentalIsolateBroadphaseDispatchEverySteps,
    'experimentalIsolateBroadphaseTargetStepMs':
        experimentalIsolateBroadphaseTargetStepMs,
    'experimentalIsolateBroadphaseAdaptiveCheckIntervalSteps':
        experimentalIsolateBroadphaseAdaptiveCheckIntervalSteps,
    'experimentalIsolateBroadphaseAdaptiveMargin':
        experimentalIsolateBroadphaseAdaptiveMargin,
    'experimentalIsolateBroadphaseAdaptiveMinHoldSteps':
        experimentalIsolateBroadphaseAdaptiveMinHoldSteps,
    'adaptiveIsolateActive': _adaptiveIsolateActive,
    'adaptiveAvgStepMs': _adaptiveAvgStepMs,
    'adaptiveDecisionReason': _adaptiveDecisionReason,
    'adaptiveLastTransitionStep': _adaptiveLastTransitionStep,
    'adaptiveTransitionCount': _adaptiveTransitionCount,
    'broadphasePipeline': _lastBroadphasePipeline,
    'broadphaseWorkerPairCount': _lastBroadphaseWorkerPairCount,
    'broadphaseWorkerInFlight': _lastBroadphaseWorkerInFlight,
    'broadphaseEligibleBodies': _lastBroadphaseEligibleBodies,
    'broadphaseSerializedBodies': _lastBroadphaseSerializedBodies,
    'broadphaseSerializedBytes': _lastBroadphaseSerializedBytes,
    'jointWakeActivations': _lastJointWakeActivations,
    'warmStartContacts': _lastWarmStartContactCount,
    'warmStartHits': _lastWarmStartHitCount,
    'resolvedManifoldPoints': _lastResolvedManifoldPointCount,
    'resolvedVelocityIterations': _lastResolvedVelocityIterationCount,
    'resolvedNormalImpulse': _lastResolvedNormalImpulse,
    'resolvedTangentImpulse': _lastResolvedTangentImpulse,
    'resolvedBlockSolves': _lastResolvedBlockSolveCount,
    'resolvedBlockFrictionSolves': _lastResolvedBlockFrictionSolveCount,
    'blockNormalHysteresisActiveContacts':
        _lastBlockNormalHysteresisActiveContacts,
    'blockFrictionHysteresisActiveContacts':
        _lastBlockFrictionHysteresisActiveContacts,
    'blockNormalHysteresisActivations':
        _lastBlockNormalHysteresisActivationCount,
    'blockNormalHysteresisDeactivations':
        _lastBlockNormalHysteresisDeactivationCount,
    'blockNormalHysteresisActivatedByThreshold':
        _lastBlockNormalHysteresisActivatedByThresholdCount,
    'blockNormalHysteresisDeactivatedBelowDisable':
        _lastBlockNormalHysteresisDeactivatedBelowDisableCount,
    'blockNormalHysteresisDeactivatedNonTwoPoint':
        _lastBlockNormalHysteresisDeactivatedNonTwoPointCount,
    'blockNormalHysteresisDeactivatedNonTwoPointContact':
        _lastBlockNormalHysteresisDeactivatedNonTwoPointContactCount,
    'blockNormalHysteresisDeactivatedPairDropped':
        _lastBlockNormalHysteresisDeactivatedPairDroppedCount,
    'blockFrictionHysteresisActivations':
        _lastBlockFrictionHysteresisActivationCount,
    'blockFrictionHysteresisDeactivations':
        _lastBlockFrictionHysteresisDeactivationCount,
    'blockFrictionHysteresisActivatedByThreshold':
        _lastBlockFrictionHysteresisActivatedByThresholdCount,
    'blockFrictionHysteresisDeactivatedBelowDisable':
        _lastBlockFrictionHysteresisDeactivatedBelowDisableCount,
    'blockFrictionHysteresisDeactivatedNonTwoPoint':
        _lastBlockFrictionHysteresisDeactivatedNonTwoPointCount,
    'blockFrictionHysteresisDeactivatedNonTwoPointContact':
        _lastBlockFrictionHysteresisDeactivatedNonTwoPointContactCount,
    'blockFrictionHysteresisDeactivatedPairDropped':
        _lastBlockFrictionHysteresisDeactivatedPairDroppedCount,
    'totalBlockNormalHysteresisActivations':
        _totalBlockNormalHysteresisActivationCount,
    'totalBlockNormalHysteresisDeactivations':
        _totalBlockNormalHysteresisDeactivationCount,
    'totalBlockFrictionHysteresisActivations':
        _totalBlockFrictionHysteresisActivationCount,
    'totalBlockFrictionHysteresisDeactivations':
        _totalBlockFrictionHysteresisDeactivationCount,
    'rollingBlockNormalHysteresisActivationRate':
        experimentalContactBlockHysteresisTransitionRateWindowSteps <= 0
        ? 0.0
        : _rollingBlockNormalHysteresisActivationCount /
              experimentalContactBlockHysteresisTransitionRateWindowSteps,
    'rollingBlockNormalHysteresisDeactivationRate':
        experimentalContactBlockHysteresisTransitionRateWindowSteps <= 0
        ? 0.0
        : _rollingBlockNormalHysteresisDeactivationCount /
              experimentalContactBlockHysteresisTransitionRateWindowSteps,
    'rollingBlockFrictionHysteresisActivationRate':
        experimentalContactBlockHysteresisTransitionRateWindowSteps <= 0
        ? 0.0
        : _rollingBlockFrictionHysteresisActivationCount /
              experimentalContactBlockHysteresisTransitionRateWindowSteps,
    'rollingBlockFrictionHysteresisDeactivationRate':
        experimentalContactBlockHysteresisTransitionRateWindowSteps <= 0
        ? 0.0
        : _rollingBlockFrictionHysteresisDeactivationCount /
              experimentalContactBlockHysteresisTransitionRateWindowSteps,
    'warmStartRejectedByAge': _lastWarmStartRejectedByAgeCount,
    'warmStartRejectedByNormal': _lastWarmStartRejectedByNormalCount,
    'warmStartRejectedByAnchor': _lastWarmStartRejectedByAnchorCount,
    'warmStartRejectedByFeature': _lastWarmStartRejectedByFeatureCount,
    'warmStartMatchedStates': _lastWarmStartMatchedStateCount,
    'warmStartFeatureHistoryReused': _lastWarmStartFeatureHistoryReuseCount,
    'warmStartResolvedFeatureSeeded': _lastWarmStartResolvedFeatureSeedCount,
    'warmStartAngularPreloadCount': _lastWarmStartAngularPreloadCount,
    'warmStartCacheSize': _contactWarmStartCache.values.fold<int>(
      0,
      (sum, list) => sum + list.length,
    ),
    'warmStartMaxCachedNormalImpulse': _lastWarmStartMaxCachedNormalImpulse,
    'warmStartMaxCachedTangentImpulse': _lastWarmStartMaxCachedTangentImpulse,
    'deterministicHashEnabled': deterministicHashEnabled,
    'deterministicHashIntervalSteps': deterministicHashIntervalSteps,
    'deterministicHash': _lastDeterministicHash,
    'deterministicHashLastStep': _lastDeterministicHashStep,
    'simulationStep': _simulationStep,
  };

  /// Deterministic broad-phase pair-key snapshot for algorithm parity checks.
  ///
  /// Keys are derived from body insertion indices (not identity hash codes),
  /// so they are stable across equivalent runs.
  @visibleForTesting
  Set<int> debugBroadphasePairKeys({required bool useIsolateAlgorithm}) {
    if (!useIsolateAlgorithm) {
      _grid.syncBodies(_bodies);
      final pairs = _grid.getPotentialCollisions();
      final base = _bodies.length + 1;
      final keys = <int>{};
      for (final pair in pairs) {
        final ai = _bodies.indexOf(pair.a);
        final bi = _bodies.indexOf(pair.b);
        if (ai < 0 || bi < 0) continue;
        var a = ai;
        var b = bi;
        if (a > b) {
          final tmp = a;
          a = b;
          b = tmp;
        }
        keys.add(a * base + b);
      }
      return keys;
    }

    final payload = _serializeBodiesForWorker(_bodies);
    if (payload.enabledBodyCount < 2) return <int>{};

    final bounds = payload.boundsData.materialize().asFloat32List();
    final packedPairs = _computeBroadphasePairsFromBounds(
      bounds,
      payload.enabledBodyCount,
      _grid.cellSize,
    );

    final base = _bodies.length + 1;
    final keys = <int>{};
    for (var i = 0; i + 1 < packedPairs.length; i += 2) {
      final compactA = packedPairs[i];
      final compactB = packedPairs[i + 1];
      if (compactA >= payload.originalBodyIndices.length ||
          compactB >= payload.originalBodyIndices.length) {
        continue;
      }

      var a = payload.originalBodyIndices[compactA];
      var b = payload.originalBodyIndices[compactB];
      if (a > b) {
        final tmp = a;
        a = b;
        b = tmp;
      }
      keys.add(a * base + b);
    }

    return keys;
  }

  // ── Shape casts ──────────────────────────────────────────────────────────

  /// Sweep a circle of [radius] from [origin] by [motion] and return the first
  /// body hit, or null if nothing is in the way.
  ///
  /// This is an approximate Minkowski-sum sweep: the body's shape is expanded by
  /// [radius] and then a ray is cast along [motion] against the expanded shapes.
  ShapeCastResult? castCircle(
    Offset origin,
    double radius,
    Offset motion, {
    Set<PhysicsBody>? exclude,
  }) {
    if (motion == Offset.zero) return null;
    final motionDist = motion.distance;
    final ray = Ray(origin: origin, direction: motion, maxDistance: motionDist);

    ShapeCastResult? best;
    for (final body in bodies) {
      if (!body.isActive) continue;
      if (exclude != null && exclude.contains(body)) continue;

      final pos = body.position.toOffset();
      final shape = body.shape;

      RayBodyHit? hit;
      if (shape is CircleShape) {
        hit = _rayHitCircle(ray, pos, shape.radius + radius, body);
      } else if (shape is PolygonShape) {
        // Expand polygon outward by radius (approximate: use SAT-inflated bounds)
        hit = _rayHitPolygonVerts(ray, shape.vertices, pos, body);
        // Re-test with an AABB inflated by radius for better rejection
        final inflated = shape.getBounds(pos).inflate(radius);
        if (!_rayIntersectsAABB(ray, inflated)) hit = null;
      } else if (shape is CapsuleShape) {
        final inflated = CapsuleShape(
          center1: shape.center1,
          center2: shape.center2,
          radius: shape.radius + radius,
        );
        hit = _rayHitCapsule(ray, pos, inflated, body);
      } else {
        hit = _rayHitBody(ray, body);
      }

      if (hit != null && (best == null || hit.distance < best.distance)) {
        best = ShapeCastResult(
          body: hit.body,
          point: hit.point,
          normal: hit.normal,
          toi: motionDist > 0 ? hit.distance / motionDist : 0.0,
          distance: hit.distance,
        );
      }
    }
    return best;
  }

  // ── Overlap queries ───────────────────────────────────────────────────────

  /// Return all active bodies whose bounding box overlaps [rect].
  List<PhysicsBody> queryAABB(Rect rect) {
    final result = <PhysicsBody>[];
    for (final body in bodies) {
      if (!body.isActive) continue;
      if (body.shape.getBounds(body.position.toOffset()).overlaps(rect)) {
        result.add(body);
      }
    }
    return result;
  }

  /// Return all active bodies overlapping a circle at [center] with [radius].
  List<PhysicsBody> queryCircle(Offset center, double radius) {
    final aabb = Rect.fromCircle(center: center, radius: radius);
    final result = <PhysicsBody>[];
    for (final body in bodies) {
      if (!body.isActive) continue;
      final pos = body.position.toOffset();
      if (!body.shape.getBounds(pos).overlaps(aabb)) continue;
      // Precise circle check using collision manifold against a dummy circle.
      final shape = body.shape;
      final tempCircle = CircleShape(radius);
      final manifold = shape.getManifold(pos, tempCircle, center);
      if (manifold.isColliding) result.add(body);
    }
    return result;
  }

  /// Return all active bodies containing [point].
  List<PhysicsBody> queryPoint(Offset point) =>
      queryCircle(point, 0.5); // 0.5-unit epsilon circle

  // ── Shape caching (in-memory) ─────────────────────────────────────────────

  final Map<String, List<Offset>> _shapeCache = {};

  void cachePolygonShape(String cacheId, List<Offset> vertices) {
    _shapeCache[cacheId] = vertices;
  }

  List<Offset>? getCachedPolygonShape(String cacheId) {
    return _shapeCache[cacheId];
  }
}

/// The result of a circle shape cast against a [PhysicsBody].
class ShapeCastResult {
  /// The body that was hit.
  final PhysicsBody body;

  /// World-space contact point.
  final Offset point;

  /// Surface normal at the contact point.
  final Offset normal;

  /// Time of impact (0–1, where 1 means the full motion was completed).
  final double toi;

  /// Distance from the sweep origin to the contact point.
  final double distance;

  const ShapeCastResult({
    required this.body,
    required this.point,
    required this.normal,
    required this.toi,
    required this.distance,
  });
}

/// The result of a ray cast against a [PhysicsBody].
class RayBodyHit {
  /// The body that was intersected.
  final PhysicsBody body;

  /// World-space hit point.
  final Offset point;

  /// Surface normal at the hit point, pointing toward the ray origin.
  final Offset normal;

  /// Distance from the ray origin to [point] (world units).
  final double distance;

  const RayBodyHit({
    required this.body,
    required this.point,
    required this.normal,
    required this.distance,
  });
}

class _IsolateBroadphaseWorker {
  Future<TransferableTypedData>? _inFlight;
  Uint32List? _readyPairIndices;
  List<int> _originalBodyIndices = const [];
  int _lastSerializedBodies = 0;
  int _lastSerializedBytes = 0;

  bool get isInFlight => _inFlight != null;
  int get lastSerializedBodies => _lastSerializedBodies;
  int get lastSerializedBytes => _lastSerializedBytes;

  bool scheduleIfIdle(List<PhysicsBody> bodies, double cellSize) {
    if (_inFlight != null) return false;

    final payload = _serializeBodiesForWorker(bodies);
    _originalBodyIndices = payload.originalBodyIndices;
    _lastSerializedBodies = payload.enabledBodyCount;
    _lastSerializedBytes = payload.serializedBytes;
    if (payload.enabledBodyCount < 2) {
      _readyPairIndices = Uint32List(0);
      return false;
    }

    try {
      _inFlight = Isolate.run(
        () => _computeBroadphasePairsInWorker(
          payload.boundsData,
          payload.enabledBodyCount,
          cellSize,
        ),
      );

      _inFlight!
          .then((result) {
            _readyPairIndices = result.materialize().asUint32List();
          })
          .whenComplete(() {
            _inFlight = null;
          });
      return true;
    } catch (_) {
      _inFlight = null;
      return false;
    }
  }

  List<BodyPair>? consumePairs(List<PhysicsBody> bodies) {
    final packed = _readyPairIndices;
    if (packed == null) return null;
    _readyPairIndices = null;

    final pairs = <BodyPair>[];
    for (var i = 0; i + 1 < packed.length; i += 2) {
      final compactA = packed[i];
      final compactB = packed[i + 1];
      if (compactA >= _originalBodyIndices.length ||
          compactB >= _originalBodyIndices.length) {
        continue;
      }

      final aIndex = _originalBodyIndices[compactA];
      final bIndex = _originalBodyIndices[compactB];
      if (aIndex >= bodies.length || bIndex >= bodies.length) continue;

      final a = bodies[aIndex];
      final b = bodies[bIndex];
      if (!a.isActive ||
          !b.isActive ||
          !a.checkCollision ||
          !b.checkCollision) {
        continue;
      }
      pairs.add(BodyPair(a, b));
    }
    return pairs;
  }

  void dispose() {
    _inFlight = null;
    _readyPairIndices = null;
    _originalBodyIndices = const [];
    _lastSerializedBodies = 0;
    _lastSerializedBytes = 0;
  }
}

class _BroadphaseWorkerPayload {
  final TransferableTypedData boundsData;
  final int enabledBodyCount;
  final List<int> originalBodyIndices;
  final int serializedBytes;

  const _BroadphaseWorkerPayload({
    required this.boundsData,
    required this.enabledBodyCount,
    required this.originalBodyIndices,
    required this.serializedBytes,
  });
}

_BroadphaseWorkerPayload _serializeBodiesForWorker(List<PhysicsBody> bodies) {
  final originalIndices = <int>[];
  for (var i = 0; i < bodies.length; i++) {
    final body = bodies[i];
    if (!body.isActive || !body.checkCollision) continue;
    originalIndices.add(i);
  }

  // Compact transfer: only enabled bodies, 4 floats each (l,t,r,b).
  final data = Float32List(originalIndices.length * 4);

  for (
    var compactIndex = 0;
    compactIndex < originalIndices.length;
    compactIndex++
  ) {
    final body = bodies[originalIndices[compactIndex]];
    final base = compactIndex * 4;

    final bounds = body.getCompoundBounds(body.position.toOffset());
    data[base] = bounds.left;
    data[base + 1] = bounds.top;
    data[base + 2] = bounds.right;
    data[base + 3] = bounds.bottom;
  }

  final bytes = data.length * Float32List.bytesPerElement;

  return _BroadphaseWorkerPayload(
    boundsData: TransferableTypedData.fromList([data.buffer.asUint8List()]),
    enabledBodyCount: originalIndices.length,
    originalBodyIndices: originalIndices,
    serializedBytes: bytes,
  );
}

TransferableTypedData _computeBroadphasePairsInWorker(
  TransferableTypedData packedBounds,
  int enabledBodyCount,
  double cellSize,
) {
  final bounds = packedBounds.materialize().asFloat32List();

  final packedPairs = _computeBroadphasePairsFromBounds(
    bounds,
    enabledBodyCount,
    cellSize,
  );

  return TransferableTypedData.fromList([packedPairs.buffer.asUint8List()]);
}

Uint32List _computeBroadphasePairsFromBounds(
  Float32List bounds,
  int enabledBodyCount,
  double cellSize,
) {
  if (enabledBodyCount < 2) return Uint32List(0);

  final buckets = <int, List<int>>{};

  int hashCell(int x, int y) => (x * 73856093) ^ (y * 83492791);

  for (var i = 0; i < enabledBodyCount; i++) {
    final base = i * 4;

    final minX = (bounds[base] / cellSize).floor();
    final minY = (bounds[base + 1] / cellSize).floor();
    final maxX = (bounds[base + 2] / cellSize).floor();
    final maxY = (bounds[base + 3] / cellSize).floor();

    for (var x = minX; x <= maxX; x++) {
      for (var y = minY; y <= maxY; y++) {
        final cellHash = hashCell(x, y);
        (buckets[cellHash] ??= <int>[]).add(i);
      }
    }
  }

  final seen = <int>{};
  final pairList = <int>[];
  for (final bucket in buckets.values) {
    if (bucket.length < 2) continue;

    for (var i = 0; i < bucket.length; i++) {
      for (var j = i + 1; j < bucket.length; j++) {
        var a = bucket[i];
        var b = bucket[j];
        if (a == b) continue;
        if (a > b) {
          final tmp = a;
          a = b;
          b = tmp;
        }

        final key = a * enabledBodyCount + b;
        if (seen.add(key)) {
          pairList
            ..add(a)
            ..add(b);
        }
      }
    }
  }

  final packedPairs = Uint32List.fromList(pairList);
  return packedPairs;
}
