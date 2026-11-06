/// The 2-D engine on native Box2D.
library;

import 'dart:collection' show UnmodifiableListView;
import 'dart:ffi'
    hide Size; // hide dart:ffi's Size to avoid conflict with dart:ui Size
import 'dart:ui' show Canvas, Offset, Rect, Size;

import 'package:flutter/foundation.dart' show debugPrint;
import 'package:just_box_2d/ffi.dart' as box2d;

import '../physics_2d_engine.dart';
import '../ray_2d.dart';
import 'box2d_body.dart';
import 'box2d_debug_draw.dart';
import 'box2d_events.dart';
import 'box2d_joint.dart';
import 'box2d_poses.dart';
import 'box2d_queries.dart';
import 'box2d_shapes.dart';
import 'box2d_world.dart';

/// The 2-D engine on Box2D v3.2, through just_box_2d: what
/// `PhysicsEngineFactory.create2D()` gives on native platforms.
///
/// It does everything [Physics2DEngine] declares, natively and with the same
/// meaning, so code written against [Physics2DEngine] needs no branch for
/// it; it adds an impact callback ([registerImpactCallback]). [initialize]
/// makes the native world. Where just_box_2d's library is not in the build
/// it says why (in the debug log) and runs the pure-Dart engine instead:
/// [isNative] tells which.
///
/// How it runs:
///  - the [Physics2DBody] objects stay what the game reads and writes: every
///    [update] pushes what the game changed (velocity, spin, forces, type,
///    gravity scale, mass, locked rotation, whether the body is active or
///    awake) into Box2D, steps, and reads every body's pose back in one FFI
///    call;
///  - each step's contact, sensor and move events are read right after it
///    and kept until the next [update], as the pure-Dart engine keeps its
///    own;
///  - joints are Box2D's ([Box2DJoint]), configured through [Joint2D]'s
///    setters; a joint for a body this engine does not hold, and the mouse
///    joint, run as Dart joints on the native bodies.
class Box2DPhysicsEngine extends Physics2DEngine {
  /// A Box2D engine with gravity ([gravityX], [gravityY]), [subSteps] Box2D
  /// sub-steps a step (4 recommended) and [numThreads] worker threads (0:
  /// cores minus one, at most 4). Call [initialize] before adding bodies.
  Box2DPhysicsEngine({
    double gravityX = 0.0,
    double gravityY = 981.0, // 9.81 m/s² at 1 unit = 1 cm
    this.subSteps = 4,
    this.numThreads = 0,
  }) : super.pureDart() {
    gravity.x = gravityX;
    gravity.y = gravityY;
  }

  /// Number of Box2D sub-steps per fixed tick (4 recommended).
  final int subSteps;

  /// Number of native worker threads (0 → cores − 1, at most 4).
  final int numThreads;

  // ── Native world ───────────────────────────────────────────────────────────
  // Nullable so that if initialize() fails (e.g. native lib not compiled yet),
  // update/dispose guard against LateInitializationError and fall back safely.

  Box2DWorld? _world;

  /// True once [initialize] has successfully created the native world.
  bool _nativeReady = false;

  // ── Body registries ────────────────────────────────────────────────────────

  /// Physics2DBody → Box2DBody: the primary mapping.
  final Map<Physics2DBody, Box2DBody> _bodyMap = {};

  /// The native bodies in the order added, for [bodies].
  final List<Physics2DBody> _nativeBodies = [];
  late final List<Physics2DBody> _nativeView = UnmodifiableListView(
    _nativeBodies,
  );

  /// Packed handle → Physics2DBody: reverse lookup for contact event callbacks.
  final Map<int, Physics2DBody> _handleToBody = {};

  /// What was last pushed into each native body (see [_pushSettings]).
  final Map<
    Physics2DBody,
    ({
      int type,
      double gravityScale,
      double mass,
      bool active,
      bool fixedRotation,
    })
  >
  _pushed = {};

  late final Box2DQueries _queries = Box2DQueries(
    () => _world!.handle,
    (handle) => _handleToBody[handle],
    (body) => _bodyMap[body]?.handle ?? 0,
  );

  /// Every body's pose, read back after the steps.
  final Box2DPoses _poses = Box2DPoses();

  /// Each step's events, kept until the next [update].
  final Box2DEvents _events = Box2DEvents();

  // ── NativeCallable for cross-thread impact audio callback ──────────────────

  NativeCallable<box2d.ImpactCallbackFnFunction>? _impactCallable;

  // ── Diagnostics ────────────────────────────────────────────────────────────

  int _lastStepCount = 0;
  double _lastStepMs = 0.0;
  int _lastContactCount = 0;

  // Persistent stopwatch — reused every frame to avoid per-update allocation.
  final Stopwatch _stepStopwatch = Stopwatch();

  @override
  bool get isNative => _nativeReady;

  // ── Lifecycle ──────────────────────────────────────────────────────────────

  /// Makes the native world. Bodies added before it go into it, with the
  /// joints made for them (as Dart joints on the native bodies, as the
  /// mouse joint runs). Calling it again does nothing.
  ///
  /// Without just_box_2d's library (an app that turned its build off) it
  /// says why in the debug log and the engine runs pure Dart.
  @override
  void initialize() {
    // Already running: keep the world, its bodies and buffers.
    if (_nativeReady) return;
    try {
      // Throws, saying why, when just_box_2d's library is not in the build.
      box2d.Box2D.ensureAvailable();
      _world = Box2DWorld(
        gravityX: gravity.x,
        gravityY: gravity.y,
        numThreads: numThreads,
        subSteps: subSteps,
      );
      _nativeReady = true;
    } catch (e) {
      // No native library (the app turned just_box_2d's build off).
      // Fall back to the pure-Dart engine so the app keeps running.
      // debugPrint makes the fallback visible rather than silent — check logs if
      // physics performance is unexpectedly low on a native platform.
      debugPrint(
        'Box2DPhysicsEngine: native init failed ($e) — falling back to pure-Dart',
      );
      super.initialize();
      return;
    }

    // Bodies added before there was a native world went to the pure-Dart
    // engine: they come over, with the Dart joints made for them.
    final early = [...super.bodies];
    if (early.isEmpty) return;
    final dartJoints = [...joints];
    for (final body in early) {
      super.removeBody(body);
    }
    for (final body in early) {
      addBody(body);
    }
    for (final joint in dartJoints) {
      if (!joints.contains(joint)) addJoint(joint);
    }
  }

  /// Advance physics by [deltaTime] seconds.
  ///
  /// Internally runs fixed-step accumulation: fires 0–N 1/60 s steps,
  /// then bulk-extracts all transforms in a single zero-copy FFI call, and
  /// writes positions/velocities back into the [Physics2DBody] objects. A
  /// [deltaTime] that is not a positive, finite number runs no step.
  @override
  void update(double deltaTime) {
    if (!_nativeReady) {
      super.update(deltaTime);
      return;
    }
    _stepStopwatch
      ..reset()
      ..start();
    _events.clear();

    final dt = deltaTime > 0 && deltaTime.isFinite ? deltaTime : 0.0;
    // The interpolator's previous pose is the one before this frame's steps;
    // a frame that runs none keeps the pair it has.
    final stepping = _world!.alpha + dt / Physics2DEngine.fixedDeltaTime >= 1.0;

    for (final entry in _bodyMap.entries) {
      final body = entry.key;
      final b2Body = entry.value;
      if (stepping) b2Body.capturePrevious();
      _pushSettings(body, b2Body);
      if (body.isActive) _pushMotion(body, b2Body, stepping: stepping);
    }

    _lastStepCount = _world!.step(
      dt,
      beforeStep: _runDartJoints,
      onStep: _gatherEvents,
    );
    _poses.readAll(_bodyMap);
    _lastContactCount = _events.contactBegins.length;

    _stepStopwatch.stop();
    _lastStepMs = _stepStopwatch.elapsedMicroseconds / 1000.0;
  }

  /// Reads the step just run's events (Box2D keeps one step's).
  void _gatherEvents() => _events.gather(_world!.handle, _handleToBody);

  /// Pushes what the game wrote to [body] since the last step into
  /// [b2Body]: whether it is awake, its velocity and spin, and (before a
  /// step) its forces — only what differs from what native last reported,
  /// so an untouched body costs no FFI call.
  ///
  /// [Physics2DBody.velocity] and friends are plain Dart fields: without
  /// this, ECS-driven movement (player input, knockback) would never reach
  /// the native body, and the next read-back would overwrite it.
  void _pushMotion(
    Physics2DBody body,
    Box2DBody b2Body, {
    required bool stepping,
  }) {
    final h = b2Body.handle;
    // Woken (or put to sleep) by a write: Box2D wakes a body for a velocity
    // only when it is not zero, so the flag is pushed itself.
    if (body.isDynamic && body.isAwake != b2Body.awake) {
      box2d.b2w_setBodyAwake(h, body.isAwake ? 1 : 0);
      b2Body.awake = body.isAwake;
    }
    // A sleeping body is held: b2Body_SetLinearVelocity wakes the body it
    // is called on, and the pure-Dart engine leaves a sleeping body alone.
    if (body.mass <= 0 || !body.isAwake) return;
    if (body.velocity.x != b2Body.velocityX ||
        body.velocity.y != b2Body.velocityY) {
      b2Body.setLinearVelocity(body.velocity.x, body.velocity.y);
    }
    if (body.angularVelocity != b2Body.angularVelocity) {
      box2d.b2w_setAngularVelocity(h, body.angularVelocity);
      b2Body.angularVelocity = body.angularVelocity;
    }
    // The pure-Dart engine uses a frame's forces up in its first step: on
    // a frame with no step they wait for the next one.
    if (stepping) _pushForces(body, b2Body);
  }

  /// Hands [body]'s accumulated force and torque to [b2Body] for the next
  /// step. The force goes in as the change of velocity it makes over the
  /// step (acceleration × dt, as an impulse), before the step moves the
  /// body — as the pure-Dart engine integrates it. Box2D holding a force
  /// across its sub-steps instead adds energy to a stiff spring (a mouse
  /// joint swung further every time).
  void _pushForces(Physics2DBody body, Box2DBody b2Body) {
    if (body.acceleration.x != 0.0 || body.acceleration.y != 0.0) {
      const dt = Physics2DEngine.fixedDeltaTime;
      box2d.b2w_applyLinearImpulse(
        b2Body.handle,
        body.acceleration.x * body.mass * dt,
        body.acceleration.y * body.mass * dt,
      );
      body.acceleration.setZero();
    }
    if (body.torque != 0.0) {
      box2d.b2w_applyTorque(b2Body.handle, body.torque);
      body.torque = 0.0;
    }
  }

  /// Before each native step, the joints with no native counterpart (the
  /// mouse joint, and one made for a body this engine does not hold), as
  /// the pure-Dart engine runs its joints every step; the forces they apply
  /// go to their native bodies for that step.
  void _runDartJoints() {
    for (final joint in joints) {
      if (joint is Box2DJoint) continue;
      final held = [
        for (final body in joint.bodies)
          if (_bodyMap[body] case final b2Body?) (body, b2Body),
      ];
      // Where they are now: a frame's second step starts where its first
      // left them.
      for (final (body, b2Body) in held) {
        _poses.readOne(body, b2Body);
      }
      joint.applyConstraint(Physics2DEngine.fixedDeltaTime);
      for (final (body, b2Body) in held) {
        _pushForces(body, b2Body);
      }
    }
  }

  /// Brings [b2Body]'s type, gravity scale, mass, locked rotation and
  /// whether it is in the simulation up to [body]'s — plain fields a game
  /// may write at any time — touching only what changed.
  ///
  /// Box2D derives mass from shape area × density, not from
  /// [Physics2DBody.mass], so the configured mass is set on top. Dynamic
  /// bodies only: Box2D gates gravity on inverse mass, not on body type, so
  /// a positive mass on a kinematic body would make it fall.
  void _pushSettings(Physics2DBody body, Box2DBody b2Body) {
    final type = body.effectiveBodyType;
    final scale = body.effectiveGravityScale;
    final mass = type == Body2DType.dynamic ? body.mass : 0.0;
    final last = _pushed[body]!;
    if (last.type == type.index &&
        last.gravityScale == scale &&
        last.mass == mass &&
        last.fixedRotation == body.fixedRotation &&
        last.active == body.isActive) {
      return;
    }
    final h = b2Body.handle;
    final retyped = last.type != type.index;
    if (retyped) {
      // Expensive: Box2D rebuilds the body's contacts and sums its mass
      // from the shapes again (so the mass below is set again too).
      b2Body.setType(type.index);
      // A sleeping kinematic body ignores the velocity it is given, and the
      // pure-Dart engine never puts one to sleep.
      box2d.b2w_setBodySleepEnabled(
        h,
        type == Body2DType.kinematic || !body.canSleep ? 0 : 1,
      );
    }
    if (retyped || last.gravityScale != scale) {
      box2d.b2w_setBodyGravityScale(h, scale);
    }
    if (mass > 0 && (retyped || last.mass != mass)) {
      box2d.b2w_setBodyMass(h, mass);
    }
    if (last.fixedRotation != body.fixedRotation) {
      box2d.b2w_setBodyFixedRotation(h, body.fixedRotation ? 1 : 0);
    }
    if (last.active != body.isActive) _pushActive(body, b2Body);
    _pushed[body] = (
      type: type.index,
      gravityScale: scale,
      mass: mass,
      fixedRotation: body.fixedRotation,
      active: body.isActive,
    );
  }

  /// Takes [b2Body] out of Box2D's simulation, or puts it back, as
  /// [Physics2DBody.isActive] says.
  void _pushActive(Physics2DBody body, Box2DBody b2Body) {
    final on = body.isActive;
    box2d.b2w_setBodyEnabled(b2Body.handle, on ? 1 : 0);
    b2Body.enabled = on;
    if (on) {
      // Box2D brings it back at rest and awake; what the Dart body holds
      // (its velocity, spin, whether it sleeps) is pushed at the next
      // update, as the pure-Dart engine goes on with it.
      b2Body
        ..velocityX = 0.0
        ..velocityY = 0.0
        ..angularVelocity = 0.0
        ..awake = true;
    }
  }

  /// Add a [Physics2DBody] to the simulation.
  ///
  /// Creates a matching native body + shape fixture. The [body] object
  /// remains the shared state carrier that the ECS reads from.
  @override
  void addBody(Physics2DBody body) {
    if (!_nativeReady) {
      super.addBody(body);
      return;
    }
    if (_bodyMap.containsKey(body)) return;

    // effectiveBodyType folds in the long-standing `mass <= 0 means static`
    // rule, so kinematic is opt-in and nothing that predates Body2DType shifts.
    final type = body.effectiveBodyType;
    final isStatic = type == Body2DType.static;

    final b2Body = Box2DBody.ofType(
      worldHandle: _world!.handle,
      bodyTypeIndex: type.index,
      posX: body.position.x,
      posY: body.position.y,
      angle: body.angle,
    );
    final h = b2Body.handle;

    if (!isStatic && body.isAwake) {
      // Carry over any velocity already set on the Dart body at creation
      // time (e.g. a projectile spawned with an initial launch velocity, or a
      // moving platform authored with its patrol speed) — otherwise it
      // silently starts at rest until the next update() push.
      //
      // Skipped for a body created asleep: b2Body_SetLinearVelocity wakes the
      // body, so pushing here would immediately undo the isAwake = false the
      // caller asked for. The velocity is still carried, just applied by
      // update() once something wakes the body.
      if (body.velocity.x != 0.0 || body.velocity.y != 0.0) {
        b2Body.setLinearVelocity(body.velocity.x, body.velocity.y);
      }
      if (body.angularVelocity != 0.0) {
        box2d.b2w_setAngularVelocity(h, body.angularVelocity);
        b2Body.angularVelocity = body.angularVelocity;
      }
    }

    // Register sensor/bullet BEFORE adding shape fixtures so the C wrapper
    // applies isSensor to b2ShapeDef at creation time. Written for every
    // body, solid ones too: the wrapper keeps the flag per handle and never
    // forgets it, and a handle comes back — in a new world, or a slot
    // reused — so a solid body would otherwise inherit `true` from a dead
    // sensor and let everything through.
    box2d.b2w_setBodySensor(h, body.isSensor ? 1 : 0);
    if (body.isBullet) {
      box2d.b2w_setBodyBullet(h, 1);
    }
    if (!body.isAwake) {
      // Native bodies default to awake — only call out when starting asleep.
      // The read-back after every step brings the resulting state back
      // step, so gameplay-driven wake-ups (collisions, forces) sync normally.
      box2d.b2w_setBodyAwake(h, 0);
    }
    b2Body.awake = body.isAwake;

    addBox2DShapes(h, body);

    // Collision filter can be applied post-creation.
    box2d.b2w_setBodyFilter(
      h,
      body.categoryBits,
      body.maskBits,
      body.groupIndex,
    );

    // Must come after the shapes: the one-way flag is stored per shape, so
    // there have to be shapes to store it on.
    if (body.isOneWay) {
      box2d.b2w_setBodyOneWay(h, 1, body.oneWayDirection.index);
    }

    // Box2D's per-body linear damping is a different curve from the pure-Dart
    // engine's, but leaving it unset entirely meant air drag simply did not
    // exist on native while it did on web.
    if (body.drag != 0.0) {
      box2d.b2w_setBodyLinearDamping(h, body.drag);
    }
    if (body.angularDamping != 0.0) {
      box2d.b2w_setBodyAngularDamping(h, body.angularDamping);
    }
    // A sleeping kinematic body ignores the velocity it is given, and the
    // pure-Dart engine never puts one to sleep.
    if (!body.canSleep || type == Body2DType.kinematic) {
      box2d.b2w_setBodySleepEnabled(h, 0);
    }

    _bodyMap[body] = b2Body;
    _nativeBodies.add(body);
    _handleToBody[h] = body;
    _poses.ensureCapacity(_bodyMap.length);

    // Gravity scale (it folds together useGravity and the per-body
    // multiplier variable jump height is built on), the configured mass
    // (after the shapes Box2D sums its own from), locked rotation and
    // whether it is active: from what a new native body has.
    _pushed[body] = (
      type: type.index,
      gravityScale: 1.0,
      mass: -1.0,
      fixedRotation: false,
      active: true,
    );
    _pushSettings(body, b2Body);
  }

  // ── Runtime body mutation (native) ────────────────────────────────────────
  //
  // Each override writes the shared Dart state via super, then pushes the
  // change into the native simulation.

  @override
  void setBodyTransform(
    Physics2DBody body,
    double x,
    double y, {
    double? angle,
  }) {
    super.setBodyTransform(body, x, y, angle: angle);
    final b2Body = _native(body);
    if (b2Body == null) return;
    // Writes prev and current together, so the interpolator does not draw a
    // streak from the old position to the new one on the next frame.
    b2Body
      ..setTransform(x, y, body.angle)
      ..awake = true;
  }

  @override
  void setBodyActive(Physics2DBody body, bool active) {
    super.setBodyActive(body, active);
    final b2Body = _native(body);
    if (b2Body != null) _pushSettings(body, b2Body);
  }

  @override
  void wakeBody(Physics2DBody body) {
    super.wakeBody(body);
    final b2Body = _native(body);
    if (b2Body == null || !body.isActive || !body.isDynamic) return;
    box2d.b2w_setBodyAwake(b2Body.handle, 1);
    b2Body.awake = true;
  }

  @override
  void setBodyType(Physics2DBody body, Body2DType type) {
    super.setBodyType(body, type);
    // Only on a change: b2Body_SetType is expensive, and a game may write
    // every body field every frame.
    final b2Body = _native(body);
    if (b2Body != null) _pushSettings(body, b2Body);
  }

  @override
  void setBodyGravityScale(Physics2DBody body, double scale) {
    super.setBodyGravityScale(body, scale);
    final b2Body = _native(body);
    if (b2Body != null) _pushSettings(body, b2Body);
  }

  @override
  void setBodyFixedRotation(Physics2DBody body, bool fixed) {
    super.setBodyFixedRotation(body, fixed);
    final b2Body = _native(body);
    if (b2Body != null) _pushSettings(body, b2Body);
  }

  @override
  void setBodyFilter(
    Physics2DBody body, {
    int? categoryBits,
    int? maskBits,
    int? groupIndex,
  }) {
    super.setBodyFilter(
      body,
      categoryBits: categoryBits,
      maskBits: maskBits,
      groupIndex: groupIndex,
    );
    final b2Body = _native(body);
    if (b2Body == null) return;
    box2d.b2w_setBodyFilter(
      b2Body.handle,
      body.categoryBits,
      body.maskBits,
      body.groupIndex,
    );
  }

  @override
  void setBodyOneWay(
    Physics2DBody body,
    bool enabled, {
    OneWayDirection direction = OneWayDirection.fromAbove,
  }) {
    super.setBodyOneWay(body, enabled, direction: direction);
    final b2Body = _native(body);
    if (b2Body == null) return;
    box2d.b2w_setBodyOneWay(b2Body.handle, enabled ? 1 : 0, direction.index);
  }

  @override
  void applyLinearImpulse(Physics2DBody body, double ix, double iy) {
    if (!_nativeReady) {
      super.applyLinearImpulse(body, ix, iy);
      return;
    }
    final b2Body = _bodyMap[body];
    if (b2Body == null || !body.isDynamic) return;

    b2Body.applyLinearImpulse(ix, iy);
    // The same change on the Dart body, and remembered as what native now
    // holds, so the next update does not push it again.
    body.velocity.x += ix * body.inverseMass;
    body.velocity.y += iy * body.inverseMass;
    body.isAwake = true;
    body.sleepTimer = 0.0;
    b2Body
      ..velocityX = body.velocity.x
      ..velocityY = body.velocity.y
      ..awake = true;
  }

  @override
  void setBodyDamping(Physics2DBody body, double damping) {
    super.setBodyDamping(body, damping);
    final b2Body = _native(body);
    if (b2Body == null) return;
    box2d.b2w_setBodyLinearDamping(b2Body.handle, damping);
  }

  @override
  void setBodyAngularDamping(Physics2DBody body, double damping) {
    super.setBodyAngularDamping(body, damping);
    final b2Body = _native(body);
    if (b2Body == null) return;
    box2d.b2w_setBodyAngularDamping(b2Body.handle, damping);
  }

  @override
  void setBodyCanSleep(Physics2DBody body, bool canSleep) {
    super.setBodyCanSleep(body, canSleep);
    final b2Body = _native(body);
    if (b2Body == null) return;
    box2d.b2w_setBodySleepEnabled(
      b2Body.handle,
      canSleep && body.effectiveBodyType != Body2DType.kinematic ? 1 : 0,
    );
    if (!canSleep) {
      box2d.b2w_setBodyAwake(b2Body.handle, 1);
      b2Body.awake = true;
    }
  }

  @override
  void setBodyBullet(Physics2DBody body, bool isBullet) {
    super.setBodyBullet(body, isBullet);
    final b2Body = _native(body);
    if (b2Body == null) return;
    box2d.b2w_setBodyBullet(b2Body.handle, isBullet ? 1 : 0);
  }

  @override
  void setBodyFriction(Physics2DBody body, double friction) {
    super.setBodyFriction(body, friction);
    final b2Body = _native(body);
    if (b2Body == null) return;
    box2d.b2w_setBodyFriction(b2Body.handle, friction);
  }

  @override
  void setBodyRestitution(Physics2DBody body, double restitution) {
    super.setBodyRestitution(body, restitution);
    final b2Body = _native(body);
    if (b2Body == null) return;
    box2d.b2w_setBodyRestitution(b2Body.handle, restitution);
  }

  @override
  void setBodySensor(Physics2DBody body, bool isSensor) {
    super.setBodySensor(body, isSensor);
    if (_native(body) == null) return;
    // Box2D forbids converting a shape between sensor and solid at runtime:
    // it would break the begin/end sensor event contract. Say so rather than
    // pretending the call worked — the Dart flag now disagrees with native.
    // Create the body as a sensor and switch it with setBodyActive or the
    // collision filter instead.
    debugPrint(
      'Box2DPhysicsEngine: setBodySensor is create-time only on the native '
      'backend (Box2D limitation) — the native shape is unchanged. Use a '
      'collision filter or setBodyActive to switch a trigger at runtime.',
    );
  }

  /// [body]'s native body, when the native world is up and holds it.
  Box2DBody? _native(Physics2DBody body) =>
      _nativeReady ? _bodyMap[body] : null;

  // ── Spatial queries (native) ──────────────────────────────────────────────
  //
  // Routed to Box2D's broad-phase tree (see Box2DQueries), because a
  // platformer casts several rays per character per frame (ground probe,
  // ledge probe, wall probe) and does it forever.

  /// Box2D's query filter cannot express "skip these specific bodies", only
  /// category/mask. So when [exclude] is non-empty we have to collect all hits
  /// and filter in Dart; otherwise the cheaper closest-hit call is enough.
  /// Either way a shape the ray starts inside is passed over.
  @override
  Ray2DHit? raycast(Ray2D ray, {Set<Physics2DBody>? exclude}) {
    if (!_nativeReady) return super.raycast(ray, exclude: exclude);
    if (exclude != null && exclude.isNotEmpty) {
      final all = raycastAll(ray, exclude: exclude);
      return all.isEmpty ? null : all.first;
    }
    return _queries.raycast(ray);
  }

  @override
  List<Ray2DHit> raycastAll(Ray2D ray, {Set<Physics2DBody>? exclude}) {
    if (!_nativeReady) return super.raycastAll(ray, exclude: exclude);
    return _queries.raycastAll(ray, exclude);
  }

  @override
  List<Mover2DPlane> collideMover(
    Mover2DShape shape,
    Offset position, {
    Mover2DFilter filter = const Mover2DFilter(),
  }) {
    if (!_nativeReady) {
      return super.collideMover(shape, position, filter: filter);
    }
    return _queries.collideMover(shape, position, filter);
  }

  @override
  double castMover(
    Mover2DShape shape,
    Offset position,
    Offset translation, {
    Mover2DFilter filter = const Mover2DFilter(),
  }) {
    if (!_nativeReady) {
      return super.castMover(shape, position, translation, filter: filter);
    }
    return _queries.castMover(shape, position, translation, filter);
  }

  @override
  List<Physics2DBody> queryAABB(Rect rect) {
    if (!_nativeReady) return super.queryAABB(rect);
    return _queries.queryAABB(rect);
  }

  /// Box2D's tree gives the candidates of [circleCast], [queryCircle] and
  /// [queryPoint], whose exact tests then run in Dart on those only.
  @override
  Iterable<Physics2DBody> queryCandidates(Rect rect) {
    if (!_nativeReady) return super.queryCandidates(rect);
    return _queries.queryAABB(rect);
  }

  @override
  void removeBody(Physics2DBody body) {
    if (!_nativeReady) {
      super.removeBody(body);
      return;
    }
    final b2Body = _bodyMap.remove(body);
    if (b2Body == null) return;
    _nativeBodies.remove(body);
    _handleToBody.remove(b2Body.handle);
    _pushed.remove(body);
    // Box2D destroys the body's joints with it; they, and the Dart joints on
    // it, leave [joints] too.
    for (final joint in [...joints]) {
      if (joint.bodies.contains(body)) removeJoint(joint);
    }
    b2Body.destroy();
  }

  @override
  void renderDebug(Canvas canvas, Size size) {
    if (!debugRender) return;
    if (!_nativeReady) {
      super.renderDebug(canvas, size);
      return;
    }
    drawBox2DBodies(canvas, _bodyMap);
  }

  // Stats include all pure-Dart keys (awakeBodies, potentialPairs,
  // resolvedCollisions, broadphaseDirtyBodies, trackedCells) so callers can
  // write backend-agnostic diagnostics regardless of which engine is active.
  @override
  Map<String, dynamic> get stats => _nativeReady
      ? {
          ...super
              .stats, // pure-Dart keys with zero values when native is running
          'bodyCount': _bodyMap.length,
          'awakeBodies': _bodyMap.keys
              .where((b) => !b.isStatic && b.isAwake)
              .length,
          'lastStepCount': _lastStepCount,
          'lastStepMs': _lastStepMs,
          'contactCount': _lastContactCount,
          'alpha': _world?.alpha ?? 0.0,
          'backend': 'box2d_v3',
        }
      : {
          ...super.stats,
          'lastStepCount': 0,
          'contactCount': 0,
          'alpha': 0.0,
          'backend': 'dart_fallback',
        };

  @override
  List<Physics2DBody> get bodies => _nativeReady ? _nativeView : super.bodies;

  @override
  int get bodyCount => _nativeReady ? _bodyMap.length : super.bodyCount;

  /// Sets gravity in both the Dart vector and the native world, and wakes
  /// every sleeping dynamic body.
  @override
  void setGravity(double gx, double gy) {
    _world?.setGravity(gx, gy);
    super.setGravity(gx, gy);
  }

  // ── Contact event polling ─────────────────────────────────────────────────
  //
  // Every step's events are read right after it and kept until the next
  // [update] (see Box2DEvents), as the pure-Dart engine keeps its
  // own: poll them after [update], before the next one.

  /// Iterate all begin-touch contact events from the last [update].
  ///
  /// [fn] receives the two colliding [Physics2DBody] objects and the contact
  /// normal (nx, ny). Calls [fn] only for pairs where both bodies are tracked.
  @override
  void pollContactBeginEvents(
    void Function(Physics2DBody a, Physics2DBody b, double nx, double ny) fn,
  ) {
    if (!_nativeReady) {
      super.pollContactBeginEvents(fn);
      return;
    }
    for (final (a, b, nx, ny) in _events.contactBegins) {
      fn(a, b, nx, ny);
    }
  }

  /// Iterate all end-touch (separation) contact events from the last
  /// [update].
  ///
  /// Unlike [pollContactBeginEvents] this has no contact normal — Box2D's
  /// end-touch event only reports which bodies stopped touching.
  @override
  void pollContactEndEvents(
    void Function(Physics2DBody a, Physics2DBody b) fn,
  ) {
    if (!_nativeReady) {
      super.pollContactEndEvents(fn);
      return;
    }
    for (final (a, b) in _events.contactEnds) {
      fn(a, b);
    }
  }

  // ── Body movement event polling ───────────────────────────────────────────

  @override
  void pollBodyMoveEvents(
    void Function(Physics2DBody body, {required bool fellAsleep}) fn,
  ) {
    if (!_nativeReady) {
      super.pollBodyMoveEvents(fn);
      return;
    }
    for (final MapEntry(key: body, value: fellAsleep)
        in _events.moves.entries) {
      fn(body, fellAsleep: fellAsleep);
    }
  }

  // ── Sensor event polling ──────────────────────────────────────────────────

  /// Iterate sensor-begin events (sensor body first touched by visitor).
  @override
  void pollSensorBeginEvents(
    void Function(Physics2DBody sensor, Physics2DBody visitor) fn,
  ) {
    if (!_nativeReady) {
      super.pollSensorBeginEvents(fn);
      return;
    }
    for (final e in _events.sensorBegins) {
      fn(e.sensor, e.visitor);
    }
  }

  /// Iterate sensor-end events (visitor left the sensor).
  @override
  void pollSensorEndEvents(
    void Function(Physics2DBody sensor, Physics2DBody visitor) fn,
  ) {
    if (!_nativeReady) {
      super.pollSensorEndEvents(fn);
      return;
    }
    for (final e in _events.sensorEnds) {
      fn(e.sensor, e.visitor);
    }
  }

  // ── Joints (native for the bodies this engine holds) ──────────────────────

  /// A native joint [make] makes between [a] and [b], added to [joints];
  /// null when the native world is not up, or either body is not in it (the
  /// caller then makes a pure-Dart joint, which runs on the native bodies).
  Box2DJoint? _nativeJoint(
    Physics2DBody a,
    Physics2DBody b,
    Box2DJoint? Function(
      int world,
      (Physics2DBody, int) a,
      (Physics2DBody, int) b,
    )
    make,
  ) {
    if (!_nativeReady) return null;
    final ba = _bodyMap[a], bb = _bodyMap[b];
    if (ba == null || bb == null) return null;
    final joint = make(_world!.handle, (a, ba.handle), (b, bb.handle));
    if (joint != null) addJoint(joint);
    return joint;
  }

  @override
  Joint2D addRevoluteJoint(
    Physics2DBody a,
    Physics2DBody b,
    Offset worldAnchor, {
    bool collideConnected = false,
  }) =>
      _nativeJoint(
        a,
        b,
        (w, ha, hb) => Box2DJoint.revolute(
          w,
          ha,
          hb,
          worldAnchor,
          collideConnected: collideConnected,
        ),
      ) ??
      super.addRevoluteJoint(
        a,
        b,
        worldAnchor,
        collideConnected: collideConnected,
      );

  @override
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
    final l = length ?? (pb - pa).distance;
    final native = _nativeJoint(
      a,
      b,
      (w, ha, hb) => Box2DJoint.distance(
        w,
        ha,
        hb,
        pa,
        pb,
        minLength ?? l,
        maxLength ?? l,
        collideConnected: collideConnected,
      ),
    );
    if (native == null) {
      return super.addDistanceJoint(
        a,
        b,
        anchorA: pa,
        anchorB: pb,
        length: l,
        minLength: minLength,
        maxLength: maxLength,
        springHertz: springHertz,
        dampingRatio: dampingRatio,
        collideConnected: collideConnected,
      );
    }
    if (springHertz > 0) native.setSpring(springHertz, dampingRatio);
    return native;
  }

  @override
  Joint2D addWeldJoint(
    Physics2DBody a,
    Physics2DBody b, {
    Offset? anchor,
    bool collideConnected = false,
  }) {
    final at =
        anchor ??
        Offset(
          (a.position.x + b.position.x) / 2,
          (a.position.y + b.position.y) / 2,
        );
    return _nativeJoint(
          a,
          b,
          (w, ha, hb) => Box2DJoint.weld(
            w,
            ha,
            hb,
            at,
            collideConnected: collideConnected,
          ),
        ) ??
        super.addWeldJoint(
          a,
          b,
          anchor: at,
          collideConnected: collideConnected,
        );
  }

  @override
  Joint2D addPrismaticJoint(
    Physics2DBody a,
    Physics2DBody b,
    Offset axis, {
    Offset? anchor,
    bool collideConnected = false,
  }) {
    final at = anchor ?? b.position.toOffset();
    return _nativeJoint(
          a,
          b,
          (w, ha, hb) => Box2DJoint.prismatic(
            w,
            ha,
            hb,
            at,
            axis,
            collideConnected: collideConnected,
          ),
        ) ??
        super.addPrismaticJoint(
          a,
          b,
          axis,
          anchor: at,
          collideConnected: collideConnected,
        );
  }

  @override
  Joint2D addWheelJoint(
    Physics2DBody a,
    Physics2DBody b,
    Offset axis, {
    Offset? anchor,
    bool collideConnected = false,
  }) {
    // The wheel turns about its axle: its own centre unless said otherwise.
    final at = anchor ?? b.position.toOffset();
    return _nativeJoint(
          a,
          b,
          (w, ha, hb) => Box2DJoint.wheel(
            w,
            ha,
            hb,
            at,
            axis,
            collideConnected: collideConnected,
          ),
        ) ??
        super.addWheelJoint(
          a,
          b,
          axis,
          anchor: at,
          collideConnected: collideConnected,
        );
  }

  /// Removes [joint]; a native one is destroyed, so its bodies come apart.
  @override
  void removeJoint(Joint2D joint) {
    if (joint is Box2DJoint) joint.destroy();
    super.removeJoint(joint);
  }

  // ── Impact callback ───────────────────────────────────────────────────────

  /// Register a Dart callback for every contact whose bodies closed faster
  /// than [speedThreshold] (units per second) in a step: Box2D's hit events.
  ///
  /// It arrives through a [NativeCallable.listener], on this isolate's event
  /// loop after the step that saw it. The listener does not keep the isolate
  /// alive: [dispose] closes it.
  ///
  /// [callback] receives packed body handles (int) and the speed they closed
  /// at.
  /// Use [bodyFromHandle] to resolve handles to [Physics2DBody] objects.
  void registerImpactCallback(
    void Function(int bodyA, int bodyB, double speed) callback, {
    double speedThreshold = 100.0,
  }) {
    if (!_nativeReady) return;
    _impactCallable?.close();
    final callable = _impactCallable =
        NativeCallable<box2d.ImpactCallbackFnFunction>.listener(callback)
          ..keepIsolateAlive = false;
    box2d.b2w_setImpactCallback(
      _world!.handle,
      callable.nativeFunction,
      speedThreshold,
    );
  }

  /// Resolve a packed body handle (from the impact callback) to a [Physics2DBody].
  Physics2DBody? bodyFromHandle(int handle) => _handleToBody[handle];

  /// Render interpolation alpha from the current frame (for ECS bridge use).
  @override
  double get alpha => _nativeReady ? _world!.alpha : super.alpha;

  // ── Dispose ───────────────────────────────────────────────────────────────

  @override
  void dispose() {
    // Native first: once closed, the callable must not be called from a step.
    final world = _world;
    if (_impactCallable != null && world != null) {
      box2d.b2w_setImpactCallback(world.handle, nullptr, 0);
    }
    _impactCallable?.close();
    _impactCallable = null;

    // Destroy native joints before bodies and the world.
    for (final joint in joints) {
      if (joint is Box2DJoint) joint.destroy();
    }

    // Destroy Box2D bodies before the world (order matters in Box2D 3.0).
    for (final b2Body in _bodyMap.values) {
      b2Body.destroy();
    }
    _bodyMap.clear();
    _nativeBodies.clear();
    _handleToBody.clear();
    _pushed.clear();
    _events.dispose();

    _poses.dispose();
    _queries.dispose();

    _world?.dispose();
    _world = null;
    _nativeReady = false;
    // Always clear the base-class lists too: a body added while there was
    // no native world lives there.
    super.dispose();
  }
}
