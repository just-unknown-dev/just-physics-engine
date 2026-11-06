/// The 2-D physics engine: pure Dart, and the base of the native Box2D
/// backend.
library;

import 'dart:collection' show UnmodifiableListView;
import 'dart:math' as math;

import 'package:flutter/foundation.dart' show kDebugMode;
import 'package:flutter/material.dart';
import 'package:just_dart/just_dart.dart';
import 'package:meta/meta.dart' show internal, protected;

import 'ray_2d.dart';
import 'shape_2d_distance.dart';

part 'collision_manifold.dart';
part 'joint_2d.dart';
part 'mover_2d.dart';
part 'physics_2d_body.dart';
part 'physics_2d_contacts.dart';
part 'physics_2d_debug_draw.dart';
part 'physics_2d_queries.dart';
part 'shape_2d.dart';
part 'spatial_grid.dart';

/// The 2-D physics engine.
///
/// This class is the pure-Dart engine, which runs everywhere (the web's
/// engine), and the base of `Box2DPhysicsEngine`, which runs Box2D v3.2 on
/// native platforms. Code written against this class runs on both: every
/// body, joint and query method below means the same on each.
///
/// `PhysicsEngineFactory.create2D()` gives the platform's engine; make this
/// one anywhere with [Physics2DEngine.pureDart]. Call [initialize] before
/// adding bodies, then [update] once a frame.
///
/// Units: lengths in units (100 to a metre, so one unit is a centimetre),
/// seconds, radians; +y is down, so gravity is (0, 981).
class Physics2DEngine {
  /// The pure-Dart engine.
  ///
  /// [fixedTimestep] runs the simulation in fixed 1/60 s increments with a
  /// leftover accumulator exposed as [alpha], as the Box2D backend does.
  /// Without it the integrator consumes the raw frame delta, so results
  /// depend on refresh rate — the same jump reaches a different height at
  /// 60 Hz and 144 Hz.
  Physics2DEngine.pureDart({this.fixedTimestep = true});

  /// Whether [update] sub-steps at a fixed 1/60 s. See [Physics2DEngine.pureDart].
  final bool fixedTimestep;

  /// The fixed simulation increment used when [fixedTimestep] is set.
  /// Matches the Box2D backend's step so both backends integrate identically.
  static const double fixedDeltaTime = 1.0 / 60.0;

  /// Upper bound on accumulated time carried into one [update], expressed in
  /// sub-steps. Without it a long stall (a breakpoint, a backgrounded tab)
  /// hands the next frame a huge delta, which runs hundreds of sub-steps,
  /// which stalls again — the spiral of death. Matches the Box2D backend.
  static const int maxSubSteps = 5;

  double _accumulator = 0.0;

  /// All physics bodies — list preserves insertion order for deterministic iteration.
  final List<Physics2DBody> _bodies = [];

  /// Set mirror of [_bodies] for O(1) duplicate-check in [addBody].
  final Set<Physics2DBody> _bodySet = {};

  /// Global gravity vector. Default: 981 units/s² (9.81 m/s² at 1 unit = 1 cm),
  /// matching the Box2D backend so both simulate identically. Change it
  /// with [setGravity].
  final Vector2 gravity = Vector2(0, 981.0);

  /// Whether [renderDebug] draws anything.
  bool debugRender = false;

  /// Whether this engine runs a native simulation: false here, true for
  /// `Box2DPhysicsEngine` once its native world is up. A Box2D engine whose
  /// library is not in the build, or that was not [initialize]d, runs this
  /// pure-Dart engine instead and says false.
  bool get isNative => false;

  /// Sub-frame interpolation alpha in [0, 1) for render-smooth positioning.
  ///
  /// The fraction of a fixed step left unconsumed by the last [update]. Zero
  /// when [fixedTimestep] is off, since there is then no leftover to
  /// interpolate across. The Box2D backend gives its native accumulator's.
  double get alpha =>
      fixedTimestep ? (_accumulator / fixedDeltaTime).clamp(0.0, 1.0) : 0.0;

  // ── Sensor state ──────────────────────────────────────────────────────────
  // Which sensor pairs overlap, after the last step and during this one:
  // swapped every step rather than copied.
  Set<BodyPair> _activeSensorPairs = {};
  Set<BodyPair> _nextSensorPairs = {};
  final List<BodyPair> _sensorBeginBuffer = [];
  final List<BodyPair> _sensorEndBuffer = [];

  // ── Solid-contact state ───────────────────────────────────────────────────
  // Which non-sensor pairs touch, mirroring the sensor tracking above so
  // callers get a uniform begin/end poll API regardless of whether a pair
  // is a sensor or a solid contact.
  Set<BodyPair> _activeContactPairs = {};
  Set<BodyPair> _nextContactPairs = {};
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
  final Set<BodyPair> _seenOneWayPairs = {};

  // ── Jointed pairs ─────────────────────────────────────────────────────────
  // Pairs of bodies a joint with collideConnected off holds, and how many
  // such joints: they pass through each other, as on Box2D.
  final Map<BodyPair, int> _noCollide = {};

  /// Initialize the physics engine. The pure-Dart engine needs nothing; the
  /// Box2D backend makes its native world here.
  void initialize() {
    if (kDebugMode) debugPrint('Physics Engine initialized');
  }

  /// Sets world gravity, and wakes every sleeping dynamic body so it feels
  /// the change (on both backends: neither wakes a body for it by itself).
  void setGravity(double gx, double gy) {
    gravity.setValues(gx, gy);
    for (final body in bodies) {
      if (body.isDynamic && body.isActive && !body.isAwake) wakeBody(body);
    }
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
  ///
  /// A [deltaTime] that is not a positive, finite number (NaN from a time
  /// scale of 0/0, a negative one) runs no step: it would otherwise poison
  /// the accumulator, and no step would ever run again.
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
    _moveEventBuffer.clear();

    var steps = 0;
    if (deltaTime > 0 && deltaTime.isFinite) {
      if (fixedTimestep) {
        _accumulator += deltaTime;
        final maxAccumulated = fixedDeltaTime * maxSubSteps;
        if (_accumulator > maxAccumulated) _accumulator = maxAccumulated;
        while (_accumulator >= fixedDeltaTime) {
          _stepFixed(fixedDeltaTime);
          _accumulator -= fixedDeltaTime;
          steps++;
        }
      } else {
        _accumulator = 0.0;
        _stepFixed(deltaTime);
        steps++;
      }
    }

    // Body move events, after a frame that stepped (as the native backend
    // reports them per step): every awake moving body, and those that fell
    // asleep.
    if (steps > 0) {
      for (final body in _bodies) {
        if (body.isStatic || !body.isActive) continue;
        final wasAwake = _prevAwake[body] ?? true;
        final fellAsleep = wasAwake && !body.isAwake;
        if (body.isAwake || fellAsleep) {
          _moveEventBuffer.add((body: body, fellAsleep: fellAsleep));
        }
        _prevAwake[body] = body.isAwake;
      }
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
        case Body2DType.static:
          // Never integrates and is never moved by the solver.
          continue;

        case Body2DType.kinematic:
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

        case Body2DType.dynamic:
          break;
      }

      // Total acceleration for this sub-step (in-place).
      _accel.setFrom(body.acceleration);
      // effectiveGravityScale folds in both the useGravity switch and the
      // per-body multiplier that variable jump height is built on.
      _accel.addScaled(gravity, body.effectiveGravityScale);

      // Check for sleeping
      if (body.canSleep &&
          body.velocity.lengthSquared <
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

      // Damping: drag slows the velocity and angular damping the spin, each
      // on its own, as on the native backend.
      final dragFactor = 1.0 - body.drag * deltaTime;
      body.velocity.scale(dragFactor < 0 ? 0 : dragFactor);
      if (body.angularDamping != 0.0) {
        final spinFactor = 1.0 - body.angularDamping * deltaTime;
        body.angularVelocity *= spinFactor < 0 ? 0 : spinFactor;
      }

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

  /// Adds [body] to the simulation. Adding it twice does nothing.
  void addBody(Physics2DBody body) {
    if (_bodySet.add(body)) {
      _bodies.add(body);
    }
  }

  /// Remove a physics body, and the joints that hold it. The contacts and
  /// overlaps it was in end without an event, as on the native backend.
  void removeBody(Physics2DBody body) {
    if (_bodySet.remove(body)) {
      _bodies.remove(body);
      _grid.removeBody(body);
      bool holds(BodyPair p) => identical(p.a, body) || identical(p.b, body);
      _activeContactPairs.removeWhere(holds);
      _activeSensorPairs.removeWhere(holds);
      _oneWayPassThroughPairs.removeWhere(holds);
      _prevAwake.remove(body);
      for (final joint in [
        for (final j in _joints)
          if (j.bodies.contains(body)) j,
      ]) {
        _forgetJoint(joint);
      }
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
    Physics2DBody body,
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

  /// Takes [body] out of the simulation ([active] false) or puts it back
  /// (true): see [Physics2DBody.isActive]. A body put back is woken. The
  /// native backend takes it out of Box2D's world at once (a plain write to
  /// the field reaches it at the next [update]).
  void setBodyActive(Physics2DBody body, bool active) {
    if (body.isActive == active) return;
    body.isActive = active;
    if (active) {
      body.isAwake = true;
      body.sleepTimer = 0.0;
    }
  }

  /// Wakes [body] if it sleeps, on both backends: a sleeping body is not
  /// simulated until something touches it, and writing a velocity of zero,
  /// or changing gravity, does not wake it by itself.
  void wakeBody(Physics2DBody body) {
    body.isAwake = true;
    body.sleepTimer = 0.0;
  }

  /// Change how [body] participates in the simulation.
  void setBodyType(Physics2DBody body, Body2DType type) {
    body.bodyType = type;
  }

  /// Set the per-body gravity multiplier (see [Physics2DBody.gravityScale]),
  /// waking the body when it changes.
  void setBodyGravityScale(Physics2DBody body, double scale) {
    if (body.gravityScale == scale) return;
    body.gravityScale = scale;
    if (body.isDynamic) wakeBody(body);
  }

  /// Locks [body]'s rotation ([fixed] true: nothing turns it, not torques
  /// nor contacts) or frees it, on a live body on both backends.
  void setBodyFixedRotation(Physics2DBody body, bool fixed) {
    body.fixedRotation = fixed;
    if (fixed) body.angularVelocity = 0.0;
  }

  /// Change [body]'s collision filter at runtime.
  ///
  /// Omitted arguments keep their current value, so flipping a single layer
  /// does not require restating the others.
  void setBodyFilter(
    Physics2DBody body, {
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
    Physics2DBody body,
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
  void applyLinearImpulse(Physics2DBody body, double ix, double iy) {
    if (!body.isDynamic) return;
    body.velocity.x += ix * body.inverseMass;
    body.velocity.y += iy * body.inverseMass;
    body.isAwake = true;
    body.sleepTimer = 0.0;
  }

  /// Set [body]'s linear damping (velocity decay per second).
  void setBodyDamping(Physics2DBody body, double damping) {
    body.drag = damping;
  }

  /// Set [body]'s angular damping (spin decay per second).
  void setBodyAngularDamping(Physics2DBody body, double damping) {
    body.angularDamping = damping;
  }

  /// Allow or forbid [body] to fall asleep. Forbidding it wakes the body.
  void setBodyCanSleep(Physics2DBody body, bool canSleep) {
    body.canSleep = canSleep;
    if (!canSleep) {
      body.isAwake = true;
      body.sleepTimer = 0.0;
    }
  }

  /// Turn continuous collision ([Physics2DBody.isBullet]) on or off for
  /// [body]. Only the native backend has it; here it is just the flag.
  void setBodyBullet(Physics2DBody body, bool isBullet) {
    body.isBullet = isBullet;
  }

  /// Set [body]'s surface friction. Swap this to make a platform icy or sticky.
  void setBodyFriction(Physics2DBody body, double friction) {
    body.friction = friction;
  }

  /// Set [body]'s bounciness.
  void setBodyRestitution(Physics2DBody body, double restitution) {
    body.restitution = restitution;
  }

  /// Toggle sensor mode on [body].
  ///
  /// Honoured fully here, but **not** on the Box2D backend: Box2D forbids
  /// converting a shape between sensor and solid at runtime because it breaks
  /// the begin/end sensor event contract. The Box2D backend warns and leaves
  /// the native shape alone, so code that must work on every platform should
  /// create the body as a sensor and switch it with [setBodyActive] or the
  /// collision filter instead.
  void setBodySensor(Physics2DBody body, bool isSensor) {
    body.isSensor = isSensor;
  }

  /// Returns true if two bodies should interact (collision filter + group index).
  bool _shouldBodiesCollide(Physics2DBody a, Physics2DBody b) {
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

    // Pairs that cannot touch (two static bodies; two bodies neither of
    // which is dynamic nor a sensor) are left out by the grid, as Box2D
    // makes no contact for them.
    final potentialPairs = _grid.potentialPairs();
    _lastPotentialPairCount = potentialPairs.length;

    // NOTE: the four event buffers are cleared once per frame by update(),
    // not here — this method runs once per sub-step, and clearing per sub-step
    // would silently drop every event raised before the final one.
    _nextSensorPairs.clear();
    _nextContactPairs.clear();
    // Latched one-way pairs seen overlapping this step; anything latched but
    // absent has separated and is released below.
    _seenOneWayPairs.clear();

    for (final pair in potentialPairs) {
      final bodyA = pair.a;
      final bodyB = pair.b;
      if (!_shouldBodiesCollide(bodyA, bodyB)) continue;
      final eitherSensor = bodyA.isSensor || bodyB.isSensor;
      // Held by a joint with collideConnected off: they pass through each
      // other, as on Box2D. Sensors still see them.
      if (!eitherSensor &&
          _noCollide.isNotEmpty &&
          _noCollide.containsKey(pair)) {
        continue;
      }

      final best = _deepestManifold(bodyA, bodyB);
      if (best == null) continue;

      if (eitherSensor) {
        _nextSensorPairs.add(pair);
        if (!_activeSensorPairs.contains(pair)) {
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
          _seenOneWayPairs.add(pair);
          continue;
        }
      }

      _nextContactPairs.add(pair);
      if (!_activeContactPairs.contains(pair)) {
        _contactBeginBuffer.add((pair, best.normal.dx, best.normal.dy));
      }

      _lastResolvedCollisionCount++;
      _resolveCollision(bodyA, bodyB, best);
    }

    // Any previously active sensor pair no longer overlapping → end event.
    for (final old in _activeSensorPairs) {
      if (!_nextSensorPairs.contains(old)) {
        _sensorEndBuffer.add(old);
      }
    }
    final sensors = _activeSensorPairs;
    _activeSensorPairs = _nextSensorPairs;
    _nextSensorPairs = sensors;

    // Any previously active solid contact no longer overlapping → end event.
    for (final old in _activeContactPairs) {
      if (!_nextContactPairs.contains(old)) {
        _contactEndBuffer.add(old);
      }
    }
    final contacts = _activeContactPairs;
    _activeContactPairs = _nextContactPairs;
    _nextContactPairs = contacts;

    // Release any latch whose bodies are no longer overlapping at all, so the
    // next approach is judged fresh.
    _oneWayPassThroughPairs.removeWhere((p) => !_seenOneWayPairs.contains(p));
  }

  /// Iterate sensor-enter events from the last [update].
  ///
  /// [fn] gets the sensor first, then the body that came into it (two
  /// sensors come in no set order), as on the native backend.
  void pollSensorBeginEvents(
    void Function(Physics2DBody sensor, Physics2DBody visitor) fn,
  ) {
    for (final pair in _sensorBeginBuffer) {
      final (sensor, visitor) = _sensorFirst(pair);
      fn(sensor, visitor);
    }
  }

  /// Iterate sensor-exit events from the last [update]: the sensor first,
  /// then the body that left it.
  void pollSensorEndEvents(
    void Function(Physics2DBody sensor, Physics2DBody visitor) fn,
  ) {
    for (final pair in _sensorEndBuffer) {
      final (sensor, visitor) = _sensorFirst(pair);
      fn(sensor, visitor);
    }
  }

  /// [pair]'s bodies, the sensor first.
  static (Physics2DBody, Physics2DBody) _sensorFirst(BodyPair pair) =>
      pair.a.isSensor ? (pair.a, pair.b) : (pair.b, pair.a);

  /// Iterate solid-contact-begin events from the last [update].
  ///
  /// [fn] receives the two colliding [Physics2DBody] objects and the contact
  /// normal (nx, ny), from the first body toward the second, the same shape
  /// as the native backend's contact-begin events.
  void pollContactBeginEvents(
    void Function(Physics2DBody a, Physics2DBody b, double nx, double ny) fn,
  ) {
    for (final (pair, nx, ny) in _contactBeginBuffer) {
      fn(pair.a, pair.b, nx, ny);
    }
  }

  /// Iterate solid-contact-end events from the last [update].
  ///
  /// [fn] is called for each non-sensor (bodyA, bodyB) pair that stopped
  /// touching.
  void pollContactEndEvents(
    void Function(Physics2DBody a, Physics2DBody b) fn,
  ) {
    for (final pair in _contactEndBuffer) {
      fn(pair.a, pair.b);
    }
  }

  /// The slowest approach that bounces, in units per second: Box2D's
  /// default, 1 m/s at the 100 units per metre both backends use. Slower
  /// contacts settle instead of bouncing for ever in ever smaller hops.
  static const double restitutionThreshold = 100.0;

  /// How bouncy a contact between [a] and [b] is: the less bouncy of the
  /// two, so a bounce needs both. The native backend installs the same rule
  /// in place of Box2D's default.
  static double mixRestitution(double a, double b) => math.min(a, b);

  /// How much grip a contact between [a] and [b] has: the geometric mean,
  /// as Box2D mixes it on the native backend, so ice stays slippery on any
  /// surface.
  static double mixFriction(double a, double b) => math.sqrt(a * b);

  /// Draws every body's shapes, velocity and centre on [canvas] when
  /// [debugRender] is on.
  void renderDebug(Canvas canvas, Size size) {
    if (!debugRender) return;
    _renderDebugBodies(canvas, _bodies);
  }

  /// Clean up physics resources
  void dispose() {
    _bodies.clear();
    _bodySet.clear();
    _grid.clear();
    _activeSensorPairs.clear();
    _nextSensorPairs.clear();
    _sensorBeginBuffer.clear();
    _sensorEndBuffer.clear();
    _activeContactPairs.clear();
    _nextContactPairs.clear();
    _contactBeginBuffer.clear();
    _contactEndBuffer.clear();
    _oneWayPassThroughPairs.clear();
    _seenOneWayPairs.clear();
    _noCollide.clear();
    _accumulator = 0.0;
    _joints.clear();
    _prevAwake.clear();
    _moveEventBuffer.clear();
    if (kDebugMode) debugPrint('Physics Engine disposed');
  }

  /// Every body, in the order added: a live view, not a copy — copy it
  /// before adding or removing bodies while going through it.
  List<Physics2DBody> get bodies => _bodiesView;
  late final List<Physics2DBody> _bodiesView = UnmodifiableListView(_bodies);

  /// How many bodies the world holds, without copying them.
  int get bodyCount => _bodies.length;

  // ── Joint management ─────────────────────────────────────────────────────

  final List<Joint2D> _joints = [];
  late final List<Joint2D> _jointsView = UnmodifiableListView(_joints);

  /// Adds [joint], made by hand, to the simulation (the `add*Joint` methods
  /// add theirs). Its bodies stop colliding with each other unless it
  /// collides them ([Joint2D.collideConnected]).
  void addJoint(Joint2D joint) {
    _joints.add(joint);
    final pair = joint._quietPair;
    if (pair != null) _noCollide[pair] = (_noCollide[pair] ?? 0) + 1;
  }

  /// Removes [joint]; on the native backend it is destroyed, so its bodies
  /// come apart.
  void removeJoint(Joint2D joint) => _forgetJoint(joint);

  /// Takes [joint] out of the list and the jointed pairs.
  void _forgetJoint(Joint2D joint) {
    if (!_joints.remove(joint)) return;
    final pair = joint._quietPair;
    if (pair == null) return;
    final count = (_noCollide[pair] ?? 0) - 1;
    if (count > 0) {
      _noCollide[pair] = count;
    } else {
      _noCollide.remove(pair);
    }
  }

  /// Every joint: a live view, not a copy.
  List<Joint2D> get joints => _jointsView;

  // ── Joint factory (both backends) ─────────────────────────────────────────
  //
  // Here: the pure-Dart joints. Box2DPhysicsEngine makes native joints for
  // the bodies it holds. Anchors are world points as the bodies are now; a
  // joint's bodies pass through each other unless collideConnected.

  /// A hinge: [a] and [b] turn about [worldAnchor], which is fixed in both.
  /// Limits ([Joint2D.setLimits]) count from the angle between them now.
  Joint2D addRevoluteJoint(
    Physics2DBody a,
    Physics2DBody b,
    Offset worldAnchor, {
    bool collideConnected = false,
  }) {
    final j = RevoluteJoint2D(
      bodyA: a,
      bodyB: b,
      worldAnchor: worldAnchor,
      collideConnected: collideConnected,
    );
    addJoint(j);
    return j;
  }

  /// A rope, rod or spring from [anchorA] on [a] to [anchorB] on [b] (their
  /// centres when left out). It is [length] long (the anchors' distance now
  /// when left out), or free between [minLength] and [maxLength]. A
  /// [springHertz] above 0 makes it a spring, with [dampingRatio] (see
  /// [Joint2D.setSpring]).
  Joint2D addDistanceJoint(
    Physics2DBody a,
    Physics2DBody b, {
    Offset? anchorA,
    Offset? anchorB,
    double? length,
    double? minLength,
    double? maxLength,
    double springHertz = 0.0,
    double dampingRatio = 0.3,
    bool collideConnected = false,
  }) {
    final pa = anchorA ?? a.position.toOffset();
    final pb = anchorB ?? b.position.toOffset();
    final j = DistanceJoint2D(
      bodyA: a,
      bodyB: b,
      anchorA: pa,
      anchorB: pb,
      length: length ?? (pb - pa).distance,
      minLength: minLength,
      maxLength: maxLength,
      springHertz: springHertz,
      dampingRatio: dampingRatio,
      collideConnected: collideConnected,
    );
    addJoint(j);
    return j;
  }

  /// Welds [b] to [a] as they are now, at [anchor] (their midpoint when
  /// left out; the native solver holds that point exactly, while a rigid
  /// weld holds the bodies the same wherever it is).
  Joint2D addWeldJoint(
    Physics2DBody a,
    Physics2DBody b, {
    Offset? anchor,
    bool collideConnected = false,
  }) {
    final j = WeldJoint2D(
      bodyA: a,
      bodyB: b,
      collideConnected: collideConnected,
    );
    addJoint(j);
    return j;
  }

  /// A joint pulling [b] toward [target] (see [MouseJoint2D]): a Dart joint
  /// on both backends.
  Joint2D addMouseJoint(Physics2DBody b, Vector2 target) {
    final j = MouseJoint2D(body: b, target: target);
    addJoint(j);
    return j;
  }

  /// A slider: [b] moves along [axis] (a world direction now, fixed in [a])
  /// through [anchor] ([b]'s centre when left out), and the two do not turn
  /// against each other. Limits and the motor: [Joint2D.setLimits] and
  /// [Joint2D.setMotor].
  Joint2D addPrismaticJoint(
    Physics2DBody a,
    Physics2DBody b,
    Offset axis, {
    Offset? anchor,
    bool collideConnected = false,
  }) {
    final j = PrismaticJoint2D(
      bodyA: a,
      bodyB: b,
      axis: axis,
      collideConnected: collideConnected,
    );
    addJoint(j);
    return j;
  }

  /// A wheel: [b] turns freely about its axle, [anchor] ([b]'s centre when
  /// left out), which rides along [axis] (a world direction now, fixed in
  /// [a]) on a spring (4 Hz, damping ratio 0.7 until [Joint2D.setSpring]).
  /// A motor ([Joint2D.setMotor]) spins it.
  Joint2D addWheelJoint(
    Physics2DBody a,
    Physics2DBody b,
    Offset axis, {
    Offset? anchor,
    bool collideConnected = false,
  }) {
    final j = WheelJoint2D(
      bodyA: a,
      bodyB: b,
      axis: axis,
      anchor: anchor,
      collideConnected: collideConnected,
    );
    addJoint(j);
    return j;
  }

  // ── Body movement events ──────────────────────────────────────────────────

  // Per-body previous awake state for Dart-fallback sleep notifications.
  final Map<Physics2DBody, bool> _prevAwake = {};

  final List<({Physics2DBody body, bool fellAsleep})> _moveEventBuffer = [];

  /// Iterate body-move events from the last [update].
  ///
  /// Fires for every active moving body that is awake after a frame that
  /// stepped. [fellAsleep] is true if the body went to sleep in it.
  void pollBodyMoveEvents(
    void Function(Physics2DBody body, {required bool fellAsleep}) fn,
  ) {
    for (final ev in _moveEventBuffer) {
      fn(ev.body, fellAsleep: ev.fellAsleep);
    }
  }

  // ── Ray casting ───────────────────────────────────────────────────────────

  /// Cast a ray and return the closest hit among all physics bodies.
  ///
  /// [exclude] is an optional set of bodies to skip (e.g. the casting body).
  Ray2DHit? raycast(Ray2D ray, {Set<Physics2DBody>? exclude}) {
    Ray2DHit? best;
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
  List<Ray2DHit> raycastAll(Ray2D ray, {Set<Physics2DBody>? exclude}) {
    final hits = <Ray2DHit>[];
    for (final body in bodies) {
      if (!body.isActive) continue;
      if (exclude != null && exclude.contains(body)) continue;
      final hit = _rayHitBody(ray, body);
      if (hit != null) hits.add(hit);
    }
    hits.sort((a, b) => a.distance.compareTo(b.distance));
    return hits;
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

  /// The active bodies whose bounds overlap [rect] — round all their
  /// shapes, turned with the body — the candidates of a query there. The
  /// native backend answers from Box2D's tree.
  @internal
  @protected
  Iterable<Physics2DBody> queryCandidates(Rect rect) sync* {
    for (final body in bodies) {
      if (body.isActive && body._turnedBounds.overlaps(rect)) yield body;
    }
  }

  /// Sweep a circle of [radius] from [origin] by [motion] and return the first
  /// body hit, or null if nothing is in the way.
  ///
  /// Box2D's shape cast against every shape of every active body (turned
  /// with its body), so it stops where the circle touches, to within Box2D's
  /// linear slop (half a unit). A circle that starts out touching a body
  /// hits it at a [CircleCast2DHit.toi] of 0.
  CircleCast2DHit? circleCast(
    Offset origin,
    double radius,
    Offset motion, {
    Set<Physics2DBody>? exclude,
  }) {
    if (motion == Offset.zero) return null;
    final motionDist = motion.distance;
    final circle = Shape2DProxy([origin], radius);
    final from = Rect.fromCircle(center: origin, radius: radius);
    final sweep = from.expandToInclude(from.shift(motion));

    CircleCast2DHit? best;
    for (final body in queryCandidates(sweep)) {
      if (exclude != null && exclude.contains(body)) continue;
      for (final proxy in body.proxies()) {
        final hit = shapeCast2D(
          proxy,
          circle,
          motion,
          linearSlop: moverLinearSlop,
          maxFraction: best?.toi ?? 1.0,
        );
        if (hit == null || (best != null && hit.fraction >= best.toi)) {
          continue;
        }
        var normal = hit.normal;
        if (hit.fraction == 0) {
          final d = Shape2DDistance.between(proxy, circle);
          normal = d.overlaps ? Offset.zero : d.normal;
        }
        best = CircleCast2DHit(
          body: body,
          point: hit.point,
          normal: normal,
          toi: hit.fraction,
          distance: hit.fraction * motionDist,
        );
      }
    }
    return best;
  }

  // ── Character movers ──────────────────────────────────────────────────────

  /// The surfaces a mover of [shape] centred at [position] touches, as
  /// planes for [Mover2DSolver.solvePlanes] (Box2D's b2World_CollideMover,
  /// for a capsule or a box). Sensors, [Mover2DFilter.ignore] and bodies the
  /// filter leaves out are passed over; a one-way platform counts only from
  /// its solid side and only while the mover is not partway through it.
  List<Mover2DPlane> collideMover(
    Mover2DShape shape,
    Offset position, {
    Mover2DFilter filter = const Mover2DFilter(),
  }) => _collideMoverDart(shape, position, filter);

  /// How much of [translation] a mover of [shape] at [position] can move
  /// before it touches something: 1 for all of it (Box2D's
  /// b2World_CastMover). What it starts out touching is ignored, and a
  /// one-way platform stops it only when it comes from the solid side and
  /// is not partway through it.
  double castMover(
    Mover2DShape shape,
    Offset position,
    Offset translation, {
    Mover2DFilter filter = const Mover2DFilter(),
  }) => _castMoverDart(shape, position, translation, filter);

  // ── Overlap queries ───────────────────────────────────────────────────────

  /// Return all active bodies whose bounding box — round all their shapes,
  /// turned with the body — overlaps [rect]. The native backend answers with
  /// Box2D's boxes, which it keeps a little larger than the shapes (by a
  /// tenth of a metre), so a body just outside [rect] can come too.
  List<Physics2DBody> queryAABB(Rect rect) => queryCandidates(rect).toList();

  /// Return all active bodies with a shape (turned with its body)
  /// overlapping a circle at [center] with [radius].
  List<Physics2DBody> queryCircle(Offset center, double radius) {
    final aabb = Rect.fromCircle(center: center, radius: radius);
    final circle = Shape2DProxy([center], radius);
    final result = <Physics2DBody>[];
    for (final body in queryCandidates(aabb)) {
      for (final proxy in body.proxies()) {
        final d = Shape2DDistance.between(proxy, circle);
        if (d.overlaps || d.distance < proxy.radius + radius) {
          result.add(body);
          break;
        }
      }
    }
    return result;
  }

  /// Return all active bodies containing [point].
  List<Physics2DBody> queryPoint(Offset point) =>
      queryCircle(point, 0.5); // 0.5-unit epsilon circle
}
