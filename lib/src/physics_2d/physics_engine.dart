/// Physics Engine — 2D
///
/// Simulates realistic movement, gravity, collision detection, and object interactions.
library;

import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:just_dart/just_dart.dart';
import 'ray_2d.dart';
import '../box2d/_box2d_engine_stub.dart'
    if (dart.library.io) '../box2d/_box2d_engine_native.dart';

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
  ///
  /// [fixedTimestep] runs the simulation in fixed 1/60 s increments with a
  /// leftover accumulator exposed as [alpha], mirroring what the Box2D
  /// backend has always done. Without it the integrator consumes the raw
  /// frame delta, so results depend on refresh rate — the same jump reaches a
  /// different height at 60 Hz and 144 Hz. Pass `false` only to restore the
  /// pre-1.3 behaviour.
  PhysicsEngine.pureDart({this.fixedTimestep = true});

  /// Whether [update] sub-steps at a fixed 1/60 s. See [PhysicsEngine.pureDart].
  final bool fixedTimestep;

  /// The fixed simulation increment used when [fixedTimestep] is set.
  /// Matches `Box2DWorld._fixedDt` so both backends integrate identically.
  static const double fixedDeltaTime = 1.0 / 60.0;

  /// Upper bound on accumulated time carried into one [update], expressed in
  /// sub-steps. Without it a long stall (a breakpoint, a backgrounded tab)
  /// hands the next frame a huge delta, which runs hundreds of sub-steps,
  /// which stalls again — the spiral of death. Matches the Box2D backend.
  static const int maxSubSteps = 5;

  double _accumulator = 0.0;

  /// All physics bodies — list preserves insertion order for deterministic iteration.
  final List<PhysicsBody> _bodies = [];

  /// Set mirror of [_bodies] for O(1) duplicate-check in [addBody].
  final Set<PhysicsBody> _bodySet = {};

  /// Global gravity vector. Default: 981 units/s² (9.81 m/s² at 1 unit = 1 cm),
  /// matching the Box2D backend so both simulate identically.
  final Vector2 gravity = Vector2(0, 981.0);

  /// Whether debug rendering is enabled
  bool debugRender = false;

  /// Sub-frame interpolation alpha in [0, 1) for render-smooth positioning.
  ///
  /// The fraction of a fixed step left unconsumed by the last [update]. Zero
  /// when [fixedTimestep] is off, since there is then no leftover to
  /// interpolate across. [Box2DPhysicsEngine] overrides this with the native
  /// accumulator remainder.
  double get alpha =>
      fixedTimestep ? (_accumulator / fixedDeltaTime).clamp(0.0, 1.0) : 0.0;

  // ── Sensor state ──────────────────────────────────────────────────────────
  // Tracks which sensor pairs are currently overlapping (uses identity keys).
  final Set<BodyPair> _activeSensorPairs = {};
  final List<BodyPair> _sensorBeginBuffer = [];
  final List<BodyPair> _sensorEndBuffer = [];

  // ── Solid-contact state ───────────────────────────────────────────────────
  // Tracks which non-sensor pairs are currently resolved-colliding, mirroring
  // the sensor tracking above so callers get a uniform begin/end poll API
  // regardless of whether a pair is a sensor or a solid contact.
  final Set<BodyPair> _activeContactPairs = {};
  final List<(BodyPair pair, double nx, double ny)> _contactBeginBuffer = [];
  final List<BodyPair> _contactEndBuffer = [];

  // ── One-way pass-through latch ────────────────────────────────────────────
  // Pairs currently mid-pass-through. Once a one-way contact is allowed to
  // pass, it keeps passing until the two bodies separate completely.
  //
  // Without this, a body rising through a platform is caught on the way *out*:
  // the contact normal is derived from the overlap, so the moment the body's
  // centre crosses the platform the normal flips and the contact reads as a
  // legitimate landing-from-above. The latch is what makes "jump up through a
  // platform" work rather than snagging halfway.
  final Set<BodyPair> _oneWayPassThroughPairs = {};

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
  int _lastAwakeBodyCount = 0;
  int _lastBroadphaseDirtyBodyCount = 0;
  int _lastTrackedCellCount = 0;
  double _lastStepMs = 0.0;

  // Persistent Stopwatch instance — reused every frame to avoid heap allocation.
  final Stopwatch _stepStopwatch = Stopwatch();

  /// Advance the simulation by [deltaTime] seconds.
  ///
  /// When [fixedTimestep] is set this drains an accumulator in fixed
  /// [fixedDeltaTime] increments, so a frame may run zero, one or several
  /// sub-steps. Event buffers are cleared *here* rather than per sub-step so
  /// that events raised by an early sub-step survive until the caller polls
  /// them — the poll API is once-per-frame, not once-per-step.
  void update(double deltaTime) {
    _stepStopwatch
      ..reset()
      ..start();

    _lastResolvedCollisionCount = 0;
    _lastAwakeBodyCount = 0;

    // Cleared once per frame, not once per sub-step: _detectCollisions()
    // appends to these, and clearing inside it would discard everything the
    // first sub-step found the moment a second one ran.
    _sensorBeginBuffer.clear();
    _sensorEndBuffer.clear();
    _contactBeginBuffer.clear();
    _contactEndBuffer.clear();

    if (fixedTimestep) {
      _accumulator += deltaTime;
      final maxAccumulated = fixedDeltaTime * maxSubSteps;
      if (_accumulator > maxAccumulated) _accumulator = maxAccumulated;
      while (_accumulator >= fixedDeltaTime) {
        _stepFixed(fixedDeltaTime);
        _accumulator -= fixedDeltaTime;
      }
    } else {
      _accumulator = 0.0;
      _stepFixed(deltaTime);
    }

    // Body move events (Dart fallback: track awake transitions).
    _moveEventBuffer.clear();
    for (final body in _bodies) {
      if (body.isStatic) continue; // static bodies don't move
      final wasAwake = _prevAwake[body] ?? true;
      final fellAsleep = wasAwake && !body.isAwake;
      if (body.isAwake || fellAsleep) {
        _moveEventBuffer.add((body: body, fellAsleep: fellAsleep));
      }
      _prevAwake[body] = body.isAwake;
    }

    _stepStopwatch.stop();
    _lastStepMs = _stepStopwatch.elapsedMicroseconds / 1000.0;
  }

  /// One simulation sub-step: integrate, collide, then satisfy joints.
  ///
  /// [deltaTime] is [fixedDeltaTime] under [fixedTimestep], otherwise the raw
  /// frame delta. Frame-scoped bookkeeping (buffer clearing, move events,
  /// timing) belongs in [update], not here — this runs more than once a frame.
  void _stepFixed(double deltaTime) {
    var awakeBodyCount = 0;

    for (final body in _bodies) {
      if (!body.isActive || !body.isAwake) continue;
      awakeBodyCount++;

      switch (body.effectiveBodyType) {
        case BodyType.static:
          // Never integrates and is never moved by the solver.
          continue;

        case BodyType.kinematic:
          // Driven purely by whatever wrote `velocity` — no gravity, no
          // accumulated force, no drag. Deliberately exempt from the sleep
          // heuristic below: a moving platform creeping along under the
          // velocity threshold would otherwise be put to sleep and stop dead,
          // and nothing would ever wake it because no force acts on it.
          body.position.addScaled(body.velocity, deltaTime);
          if (!body.fixedRotation) {
            body.angle += body.angularVelocity * deltaTime;
          }
          body.acceleration.setZero();
          body.torque = 0.0;
          continue;

        case BodyType.dynamic:
          break;
      }

      // Total acceleration for this sub-step (in-place).
      _accel.setFrom(body.acceleration);
      // effectiveGravityScale folds in both the useGravity switch and the
      // per-body multiplier that variable jump height is built on.
      _accel.addScaled(gravity, body.effectiveGravityScale);

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

      if (!body.isAwake) continue;

      // Semi-Implicit Euler Integration — all in-place Vec2 ops
      // 1. Update velocity: v += accel * dt
      body.velocity.addScaled(_accel, deltaTime);
      if (!body.fixedRotation) {
        body.angularVelocity += (body.torque * body.inverseInertia) * deltaTime;
      }

      // Apply drag (simple linear drag)
      final dragFactor = 1.0 - body.drag * deltaTime;
      body.velocity.scale(dragFactor);
      body.angularVelocity *= dragFactor;

      // 2. Update position: x += v * dt
      body.position.addScaled(body.velocity, deltaTime);
      if (!body.fixedRotation) {
        body.angle += body.angularVelocity * deltaTime;
      }

      // Reset acceleration for the next sub-step
      body.acceleration.setZero();
      body.torque = 0.0;
    }

    // Simple collision detection
    _detectCollisions();

    // Apply joint constraints after collision resolution.
    for (final joint in _joints) {
      joint.applyConstraint(deltaTime);
    }

    if (awakeBodyCount > _lastAwakeBodyCount) {
      _lastAwakeBodyCount = awakeBodyCount;
    }
  }

  /// Add a physics body
  void addBody(PhysicsBody body) {
    if (_bodySet.add(body)) {
      _bodies.add(body);
    }
  }

  /// Remove a physics body
  void removeBody(PhysicsBody body) {
    if (_bodySet.remove(body)) {
      _bodies.remove(body);
      _grid.removeBody(body);
    }
  }

  /// Broad-phase grid
  final SpatialGrid _grid = SpatialGrid(100.0);

  // ── Runtime body mutation ─────────────────────────────────────────────────
  //
  // Everything below is overridden by Box2DPhysicsEngine to also push the
  // change into the native simulation. The implementations here are the real
  // ones for the pure-Dart backend (which is also the web backend), so a
  // subclass override that forgets to call super must reproduce the state
  // write itself.

  /// Teleport [body] to ([x], [y]), optionally also setting [angle].
  ///
  /// This is a discontinuous move: it breaks contacts and discards any solver
  /// warm-start for the body, so it is meant for respawns, checkpoints, level
  /// loads and warps — not for per-frame movement. Drive continuous motion
  /// with `velocity` instead.
  ///
  /// The body is woken, since a teleported sleeping body would otherwise sit
  /// inert at its new location until something else disturbed it.
  void setBodyTransform(
    PhysicsBody body,
    double x,
    double y, {
    double? angle,
  }) {
    body.position.setValues(x, y);
    if (angle != null) body.angle = angle;
    body.isAwake = true;
    body.sleepTimer = 0.0;
    // Force the broad-phase to re-bin the body from scratch: its cached cell
    // range is for the old position and a large jump would otherwise leave it
    // registered in cells it no longer occupies until the next sync.
    _grid.removeBody(body);
  }

  /// Change how [body] participates in the simulation.
  void setBodyType(PhysicsBody body, BodyType type) {
    body.bodyType = type;
  }

  /// Set the per-body gravity multiplier. See [PhysicsBody.gravityScale].
  void setBodyGravityScale(PhysicsBody body, double scale) {
    body.gravityScale = scale;
  }

  /// Change [body]'s collision filter at runtime.
  ///
  /// Omitted arguments keep their current value, so flipping a single layer
  /// does not require restating the others.
  void setBodyFilter(
    PhysicsBody body, {
    int? categoryBits,
    int? maskBits,
    int? groupIndex,
  }) {
    if (categoryBits != null) body.categoryBits = categoryBits;
    if (maskBits != null) body.maskBits = maskBits;
    if (groupIndex != null) body.groupIndex = groupIndex;
  }

  /// Turn [body] into a one-way / pass-through platform, solid only when
  /// approached from [direction].
  void setBodyOneWay(
    PhysicsBody body,
    bool enabled, {
    OneWayDirection direction = OneWayDirection.fromAbove,
  }) {
    body.isOneWay = enabled;
    body.oneWayDirection = direction;
  }

  /// Apply an instantaneous change in momentum to [body].
  ///
  /// Unlike writing `velocity` directly this preserves existing motion, which
  /// is what knockback, bounce pads and explosions want.
  void applyLinearImpulse(PhysicsBody body, double ix, double iy) {
    if (!body.isDynamic) return;
    body.velocity.x += ix * body.inverseMass;
    body.velocity.y += iy * body.inverseMass;
    body.isAwake = true;
    body.sleepTimer = 0.0;
  }

  /// Set [body]'s linear damping (velocity decay per second).
  void setBodyDamping(PhysicsBody body, double damping) {
    body.drag = damping;
  }

  /// Set [body]'s surface friction. Swap this to make a platform icy or sticky.
  void setBodyFriction(PhysicsBody body, double friction) {
    body.friction = friction;
  }

  /// Set [body]'s bounciness.
  void setBodyRestitution(PhysicsBody body, double restitution) {
    body.restitution = restitution;
  }

  /// Toggle sensor mode on [body].
  ///
  /// Honoured fully here, but **not** on the Box2D backend: Box2D forbids
  /// converting a shape between sensor and solid at runtime because it breaks
  /// the begin/end sensor event contract. [Box2DPhysicsEngine] warns and
  /// leaves the native shape alone, so code that must work on every platform
  /// should create the body as a sensor and toggle `isActive` or the collision
  /// filter instead.
  void setBodySensor(PhysicsBody body, bool isSensor) {
    body.isSensor = isSensor;
  }

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
    _grid.syncBodies(_bodies);
    _lastBroadphaseDirtyBodyCount = _grid.dirtyBodyCount;
    _lastTrackedCellCount = _grid.trackedCellCount;

    final potentialPairs = _grid.getPotentialCollisions();
    _lastPotentialPairCount = potentialPairs.length;

    // NOTE: the four event buffers are cleared once per frame by update(),
    // not here — this method runs once per sub-step, and clearing per sub-step
    // would silently drop every event raised before the final one.
    final previousSensorPairs = Set<BodyPair>.from(_activeSensorPairs);
    final currentSensorPairs = <BodyPair>{};

    final previousContactPairs = Set<BodyPair>.from(_activeContactPairs);
    final currentContactPairs = <BodyPair>{};

    // Latched one-way pairs seen overlapping this step; anything latched but
    // absent has separated and is released below.
    final currentOneWayPairs = <BodyPair>{};

    for (final pair in potentialPairs) {
      final bodyA = pair.a;
      final bodyB = pair.b;

      // Two static bodies never move, so they can neither push each other
      // nor begin or end touching: skip them, as Box2D does. A level built
      // from hundreds of static pieces (a tile map's collision) would
      // otherwise test every neighbouring pair every step.
      if (bodyA.isStatic && bodyB.isStatic) continue;
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
        continue;
      }

      // One-way / pass-through platforms.
      //
      // Decided from the contact normal rather than the mover's velocity so
      // this agrees with the native Box2D pre-solve filter, which is only
      // handed (point, normal) and cannot read velocity safely from a worker
      // thread. A body that jumps up through a platform and stalls at the apex
      // still passes through, which the old velocity test got wrong.
      //
      // Two one-way bodies meeting each other resolve normally — neither has a
      // claim to pass through the other.
      if (bodyA.isOneWay != bodyB.isOneWay) {
        if (_oneWayPassThroughPairs.contains(pair) ||
            !_oneWayContactIsSolid(bodyA, bodyB, best.normal)) {
          // Latch it so the rest of the traversal keeps passing through even
          // once the normal flips.
          _oneWayPassThroughPairs.add(pair);
          currentOneWayPairs.add(pair);
          continue;
        }
      }

      currentContactPairs.add(pair);
      if (!previousContactPairs.contains(pair)) {
        _contactBeginBuffer.add((pair, best.normal.dx, best.normal.dy));
      }

      _lastResolvedCollisionCount++;
      _resolveCollision(bodyA, bodyB, best);
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

    // Any previously active solid contact no longer overlapping → end event.
    for (final old in previousContactPairs) {
      if (!currentContactPairs.contains(old)) {
        _contactEndBuffer.add(old);
      }
    }
    // Release any latch whose bodies are no longer overlapping at all, so the
    // next approach is judged fresh.
    _oneWayPassThroughPairs.removeWhere((p) => !currentOneWayPairs.contains(p));

    _activeContactPairs
      ..clear()
      ..addAll(currentContactPairs);
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

  /// Iterate solid-contact-begin events from the last step.
  ///
  /// [fn] receives the two colliding [PhysicsBody] objects and the contact
  /// normal (nx, ny), matching [Box2DPhysicsEngine]'s native contact-begin
  /// event shape so callers use one signature regardless of backend.
  void pollContactBeginEvents(
    void Function(PhysicsBody a, PhysicsBody b, double nx, double ny) fn,
  ) {
    for (final (pair, nx, ny) in _contactBeginBuffer) {
      fn(pair.a, pair.b, nx, ny);
    }
  }

  /// Iterate solid-contact-end events from the last step.
  ///
  /// [fn] is called for each non-sensor (bodyA, bodyB) pair that stopped
  /// overlapping. [Box2DPhysicsEngine] overrides this with the native
  /// contact-end event stream.
  void pollContactEndEvents(void Function(PhysicsBody a, PhysicsBody b) fn) {
    for (final pair in _contactEndBuffer) {
      fn(pair.a, pair.b);
    }
  }

  /// Resolve collision
  /// Cosine threshold for a one-way contact, ~60 degrees of tolerance.
  ///
  /// Tighter values drop a player who lands on the very edge of a platform;
  /// looser values let a player rising steeply from below catch on it. Kept
  /// identical to the native pre-solve filter so both backends agree.
  static const double _oneWaySolidCos = 0.5;

  /// Whether a contact involving exactly one one-way body should be resolved.
  ///
  /// [normal] points from [a] to [b]; it is flipped as needed so the test
  /// always reads "which way does the mover lie relative to the platform".
  /// Screen-space convention: +Y is down, so "above" is negative Y.
  bool _oneWayContactIsSolid(PhysicsBody a, PhysicsBody b, Offset normal) {
    final platformIsA = a.isOneWay;
    final platform = platformIsA ? a : b;
    final sign = platformIsA ? 1.0 : -1.0;
    final nx = sign * normal.dx;
    final ny = sign * normal.dy;

    return switch (platform.oneWayDirection) {
      OneWayDirection.fromAbove => ny < -_oneWaySolidCos,
      OneWayDirection.fromBelow => ny > _oneWaySolidCos,
      OneWayDirection.fromLeft => nx < -_oneWaySolidCos,
      OneWayDirection.fromRight => nx > _oneWaySolidCos,
    };
  }

  void _resolveCollision(
    PhysicsBody a,
    PhysicsBody b,
    CollisionManifold manifold,
  ) {
    final normal = manifold.normal;
    final penetration = manifold.penetration;

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
    // solverInverseMass, not inverseMass: a kinematic body has a real mass but
    // must be immovable, so it contributes zero here while still pushing the
    // dynamic side out. When both sides are immovable the sum is zero and
    // there is nothing to resolve.
    final invMassA = a.solverInverseMass;
    final invMassB = b.solverInverseMass;
    final inverseMassSum = invMassA + invMassB;

    if (inverseMassSum == 0) return; // both immovable

    const correctionPercent = 0.8;
    const slop = 0.05;
    final correctionMag =
        math.max(penetration - slop, 0.0) / inverseMassSum * correctionPercent;
    a.position.x -= normal.dx * correctionMag * invMassA;
    a.position.y -= normal.dy * correctionMag * invMassA;
    b.position.x += normal.dx * correctionMag * invMassB;
    b.position.y += normal.dy * correctionMag * invMassB;

    // ── Impulse resolution ────────────────────────────────────────────────
    final rvx = b.velocity.x - a.velocity.x;
    final rvy = b.velocity.y - a.velocity.y;
    final velAlongNormal = rvx * normal.dx + rvy * normal.dy;

    if (velAlongNormal > 0) return; // separating

    final restitution = math.min(a.restitution, b.restitution);
    final j = -(1.0 + restitution) * velAlongNormal / inverseMassSum;

    final jnx = normal.dx * j;
    final jny = normal.dy * j;
    a.velocity.x -= jnx * invMassA;
    a.velocity.y -= jny * invMassA;
    b.velocity.x += jnx * invMassB;
    b.velocity.y += jny * invMassB;

    // ── Friction (Tangent Impulse) ──────────────────────────────────────────
    final rvx2 = b.velocity.x - a.velocity.x;
    final rvy2 = b.velocity.y - a.velocity.y;
    final rvDotN = rvx2 * normal.dx + rvy2 * normal.dy;
    var tx = rvx2 - normal.dx * rvDotN;
    var ty = rvy2 - normal.dy * rvDotN;

    final tangentLen = math.sqrt(tx * tx + ty * ty);
    if (tangentLen > 0.0001) {
      final invLen = 1.0 / tangentLen;
      tx *= invLen;
      ty *= invLen;

      final jt = -(rvx2 * tx + rvy2 * ty) / inverseMassSum;
      final mu = (a.friction + b.friction) / 2.0;

      double fScalar = jt;
      if (fScalar.abs() > j * mu) {
        fScalar = (fScalar > 0 ? 1.0 : -1.0) * j * mu;
      }

      a.velocity.x -= tx * fScalar * invMassA;
      a.velocity.y -= ty * fScalar * invMassA;
      b.velocity.x += tx * fScalar * invMassB;
      b.velocity.y += ty * fScalar * invMassB;
    }
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

  JointConstraint addWheelJoint(PhysicsBody a, PhysicsBody b, Offset axis) {
    final j = WheelJoint(bodyA: a, bodyB: b, axis: axis);
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
    'bodyCount': _bodies.length,
    'awakeBodies': _lastAwakeBodyCount,
    'potentialPairs': _lastPotentialPairCount,
    'resolvedCollisions': _lastResolvedCollisionCount,
    'broadphaseDirtyBodies': _lastBroadphaseDirtyBodyCount,
    'trackedCells': _lastTrackedCellCount,
    'lastStepMs': _lastStepMs,
  };

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
