import 'dart:ffi'
    hide Size; // hide dart:ffi's Size to avoid conflict with dart:ui Size
import 'dart:typed_data';
import 'dart:ui'
    show Canvas, Color, Offset, Paint, PaintingStyle, Path, Rect, Size;

import 'package:flutter/foundation.dart' show debugPrint;

import 'package:ffi/ffi.dart';
import 'package:just_box_2d/ffi.dart' as box2d;

// PhysicsBody, CollisionShape, CircleShape, RectangleShape, PolygonShape are
// all part-files of physics_engine.dart — import the library, not the parts.
import '../physics_2d/physics_engine.dart';
import '../physics_2d/ray_2d.dart';
import 'box2d_body.dart';
import 'box2d_world.dart';
import 'box2d_joint.dart';
import 'physics_game_loop.dart';

/// Box2D v3.2 physics engine adapter exposing the same duck-typed API as
/// the pure-Dart [PhysicsEngine].
///
/// Drop-in replacement on native platforms — [PhysicsEngineFactory.create]
/// returns this class. The existing ECS ([PhysicsSystem], [PhysicsBridgeSystem])
/// requires no changes because both engines share the same API surface and
/// communicate through [PhysicsBody] objects.
///
/// Architecture:
///  - [Box2DWorld] owns the native world + fixed-step accumulator.
///  - [Box2DBody] wraps each native body handle with prev/current state.
///  - After each step, [_syncTransformsFromNative] extracts all transforms in
///    a single FFI call into pre-allocated native buffers (zero-copy).
///  - [pollContactBeginEvents] lets just_game_engine's Box2DCollisionSystem
///    fire CollisionEvents without coupling this package to the ECS.
///  - [registerImpactCallback] routes high-velocity collisions from native
///    worker threads to the Dart main isolate via [NativeCallable.listener].
class Box2DPhysicsEngine extends PhysicsEngine {
  // ── Native world & loop ────────────────────────────────────────────────────
  // Nullable so that if initialize() fails (e.g. native lib not compiled yet),
  // update/dispose guard against LateInitializationError and fall back safely.

  Box2DWorld? _world;
  PhysicsGameLoop? _loop;

  /// True once [initialize] has successfully created the native world.
  bool _nativeReady = false;

  // ── Body registries ────────────────────────────────────────────────────────

  /// PhysicsBody → Box2DBody: the primary mapping.
  final Map<PhysicsBody, Box2DBody> _bodyMap = {};

  /// Packed handle → PhysicsBody: reverse lookup for contact event callbacks.
  final Map<int, PhysicsBody> _handleToBody = {};

  /// Every collision category — the filter value meaning "hit anything".
  /// b2QueryFilter uses 64-bit category/mask bits.
  static const int _allCategories = -1; // all 64 bits set

  // ── Zero-copy transform buffers (calloc-allocated, persist for world life) ──

  Pointer<Int64>? _handleBuffer; // one int64 per tracked body
  /// 7 floats per body: x, y, angle, vx, vy, awake, angular velocity.
  Pointer<Float>? _transformBuffer;
  int _bufferCapacity = 0;

  /// What was last pushed into each native body: its type, gravity scale
  /// and mass (see [_pushSettings]).
  final Map<PhysicsBody, ({int type, double gravityScale, double mass})>
  _pushed = {};

  /// The two bodies of each native joint: a removed body takes its joints
  /// out of [joints], as Box2D destroys them with it.
  final Map<Box2DJoint, (PhysicsBody, PhysicsBody)> _jointBodies = {};

  // ── Events, read after every native step ──────────────────────────────────
  // Box2D keeps only the last step's events, and a frame runs zero to five
  // steps. Each step's events are read right after it ([_gatherStepEvents])
  // and kept until the next update(), as the pure-Dart engine keeps its own:
  // none lost when a frame runs two steps, none repeated when it runs none.

  final List<(PhysicsBody, PhysicsBody, double, double)> _contactBeginBuf = [];
  final List<(PhysicsBody, PhysicsBody)> _contactEndBuf = [];
  final List<({PhysicsBody sensor, PhysicsBody visitor})>
  _nativeSensorBeginBuf = [];
  final List<({PhysicsBody sensor, PhysicsBody visitor})> _nativeSensorEndBuf =
      [];

  /// The bodies that moved, each with whether it fell asleep in the last
  /// step it moved in.
  final Map<PhysicsBody, bool> _moveBuf = {};

  Pointer<Int64>? _eventBodies; // two handles
  Pointer<Float>? _eventNormal; // nx, ny
  Pointer<Int32>? _eventAsleep;

  // ── NativeCallable for cross-thread impact audio callback ──────────────────

  NativeCallable<box2d.ImpactCallbackFnFunction>? _impactCallable;

  // ── Diagnostics ────────────────────────────────────────────────────────────

  int _lastStepCount = 0;
  double _lastStepMs = 0.0;
  int _lastContactCount = 0;

  // Persistent stopwatch — reused every frame to avoid per-update allocation.
  final Stopwatch _stepStopwatch = Stopwatch();

  /// Number of Box2D sub-steps per fixed tick (4 recommended).
  final int subSteps;

  /// Number of native worker threads (0 → cores − 1, at most 4).
  final int numThreads;

  Box2DPhysicsEngine({
    double gravityX = 0.0,
    double gravityY = 981.0, // 9.81 m/s² at 1 unit = 1 cm
    this.subSteps = 4,
    this.numThreads = 0,
  }) : super.pureDart() {
    // Set the inherited gravity vector from PhysicsEngine.
    gravity.x = gravityX;
    gravity.y = gravityY;
  }

  // ── Public API — mirrors PhysicsEngine ────────────────────────────────────

  @override
  void initialize() {
    // Already running: keep the world, its bodies and buffers.
    if (_nativeReady) return;
    // Reset buffer state so a second initialize() call (after dispose()) does
    // not skip re-allocation and cause a null assertion in _syncTransformsFromNative.
    _bufferCapacity = 0;
    _handleBuffer = null;
    _transformBuffer = null;
    try {
      // Throws, saying why, when just_box_2d's library is not in the build.
      box2d.Box2D.ensureAvailable();
      _world = Box2DWorld(
        gravityX: gravity.x,
        gravityY: gravity.y,
        numThreads: numThreads,
        subSteps: subSteps,
      );
      _loop = PhysicsGameLoop(_world!);
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
    }
  }

  /// Advance physics by [deltaTime] seconds.
  ///
  /// Internally runs fixed-step accumulation: fires 0–N 1/60 s steps,
  /// then bulk-extracts all transforms in a single zero-copy FFI call, and
  /// writes positions/velocities back into the [PhysicsBody] objects so the
  /// existing ECS bridge ([PhysicsBridgeSystem]) continues to work without
  /// modification.
  @override
  void update(double deltaTime) {
    if (!_nativeReady) {
      super.update(deltaTime);
      return;
    }
    _stepStopwatch
      ..reset()
      ..start();
    _contactBeginBuf.clear();
    _contactEndBuf.clear();
    _nativeSensorBeginBuf.clear();
    _nativeSensorEndBuf.clear();
    _moveBuf.clear();

    // The interpolator's previous pose is the one before this frame's steps;
    // a frame that runs none keeps the pair it has.
    final stepping =
        _world!.alpha + deltaTime / PhysicsEngine.fixedDeltaTime >= 1.0;

    // Push this frame's ECS-set velocity into the native body before
    // stepping. PhysicsBody.velocity is a plain Dart field — writing it
    // (e.g. PhysicsSystem._syncIn's per-frame push from VelocityComponent)
    // never reaches the native simulation on its own, since nothing else
    // bridges Dart → native for velocity. Without this, the native body's
    // velocity only ever changes via its own internal forces/collisions, so
    // any ECS-driven movement (player input, knockback, etc.) is silently
    // discarded and _syncTransformsFromNative below immediately overwrites
    // the Dart-side value back to whatever native already had (unmoved).
    // Static bodies (mass <= 0) have no meaningful velocity — skip them.
    //
    // PhysicsBody.applyForce()/applyTorque() only accumulate into the plain
    // Dart fields `acceleration`/`torque` (see PhysicsBody) — nothing else
    // reads them on this backend, so without pushing them here they would
    // silently have zero effect on the native simulation. Convert
    // acceleration back to a force (F = a × m, undoing applyForce's ÷ mass)
    // and push both through b2w_applyForce/b2w_applyTorque, then reset —
    // mirroring the pure-Dart engine's per-frame consumption of the same
    // fields so gameplay code behaves identically on both backends.
    for (final entry in _bodyMap.entries) {
      final body = entry.key;
      final b2Body = entry.value;
      if (stepping) b2Body.capturePrevious();
      _pushSettings(body, b2Body);
      // Sleeping bodies are skipped entirely. b2Body_SetLinearVelocity wakes
      // the body it is called on, so an unconditional push means a body can
      // never stay asleep on this backend — it would wake on the very next
      // frame and start integrating, while the pure-Dart backend (which gates
      // its whole integration loop on isAwake) left it alone. Two backends,
      // two behaviours, from the same code.
      //
      // Gameplay is unaffected: PhysicsSystem force-wakes every body it syncs,
      // and applyLinearImpulse wakes explicitly. Only a body something has
      // deliberately put to sleep is held here, which is what asleep means.
      if (body.mass > 0 && body.isAwake) {
        b2Body.setLinearVelocity(body.velocity.x, body.velocity.y);
        // Spin only when something wrote it: native reported the value the
        // Dart body holds after the last step, so a difference is a write.
        if (body.angularVelocity != b2Body.angularVelocity) {
          box2d.b2w_setAngularVelocity(b2Body.handle, body.angularVelocity);
          b2Body.angularVelocity = body.angularVelocity;
        }
        // The pure-Dart engine uses a frame's forces up in its first step: on
        // a frame with no step they wait for the next one.
        if (stepping) _pushForces(body, b2Body);
      }
    }

    _lastStepCount = _loop!.advance(
      deltaTime,
      beforeStep: _runDartJoints,
      onStep: _gatherStepEvents,
    );
    _syncTransformsFromNative();
    _lastContactCount = _contactBeginBuf.length;

    _stepStopwatch.stop();
    _lastStepMs = _stepStopwatch.elapsedMicroseconds / 1000.0;
  }

  /// Hands [body]'s accumulated force and torque to [b2Body] for the next
  /// step. The force goes in as the change of velocity it makes over the
  /// step (acceleration × dt, as an impulse), before the step moves the
  /// body — as the pure-Dart engine integrates it. Box2D holding a force
  /// across its sub-steps instead adds energy to a stiff spring (a mouse
  /// joint swung further every time).
  void _pushForces(PhysicsBody body, Box2DBody b2Body) {
    if (body.acceleration.x != 0.0 || body.acceleration.y != 0.0) {
      const dt = PhysicsEngine.fixedDeltaTime;
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
        _readNative(body, b2Body);
      }
      joint.applyConstraint(PhysicsEngine.fixedDeltaTime);
      for (final (body, b2Body) in held) {
        _pushForces(body, b2Body);
      }
    }
  }

  Pointer<Int64>? _oneHandle;
  Pointer<Float>? _onePose; // 7 floats, as _syncTransformsFromNative reads

  /// [body]'s pose and velocity from [b2Body], between steps.
  void _readNative(PhysicsBody body, Box2DBody b2Body) {
    final handle = _oneHandle ??= calloc<Int64>();
    final pose = _onePose ??= calloc<Float>(7);
    handle.value = b2Body.handle;
    box2d.b2w_bulkExtractTransforms(handle, pose, 1);
    body.position.setValues(pose[0], pose[1]);
    body.angle = pose[2];
    body.velocity.setValues(pose[3], pose[4]);
  }

  /// Reads one native step's events into the frame's buffers (see
  /// [_contactBeginBuf]).
  void _gatherStepEvents() {
    final w = _world!.handle;
    final a = _eventBodies ??= calloc<Int64>(2);
    final b = a + 1;
    final normal = _eventNormal ??= calloc<Float>(2);
    final asleep = _eventAsleep ??= calloc<Int32>();

    for (var i = 0, n = box2d.b2w_getContactBeginCount(w); i < n; i++) {
      box2d.b2w_getContactBeginEvent(w, i, a, b, normal, normal + 1);
      final pa = _handleToBody[a.value], pb = _handleToBody[b.value];
      if (pa != null && pb != null) {
        _contactBeginBuf.add((pa, pb, normal[0], normal[1]));
      }
    }
    for (var i = 0, n = box2d.b2w_getContactEndCount(w); i < n; i++) {
      box2d.b2w_getContactEndEvent(w, i, a, b);
      final pa = _handleToBody[a.value], pb = _handleToBody[b.value];
      if (pa != null && pb != null) _contactEndBuf.add((pa, pb));
    }
    for (var i = 0, n = box2d.b2w_getSensorBeginCount(w); i < n; i++) {
      box2d.b2w_getSensorBeginEvent(w, i, a, b);
      final sensor = _handleToBody[a.value], visitor = _handleToBody[b.value];
      if (sensor != null && visitor != null) {
        _nativeSensorBeginBuf.add((sensor: sensor, visitor: visitor));
      }
    }
    for (var i = 0, n = box2d.b2w_getSensorEndCount(w); i < n; i++) {
      box2d.b2w_getSensorEndEvent(w, i, a, b);
      final sensor = _handleToBody[a.value], visitor = _handleToBody[b.value];
      if (sensor != null && visitor != null) {
        _nativeSensorEndBuf.add((sensor: sensor, visitor: visitor));
      }
    }
    for (var i = 0, n = box2d.b2w_getBodyMoveEventCount(w); i < n; i++) {
      box2d.b2w_getBodyMoveEvent(w, i, a, asleep);
      final body = _handleToBody[a.value];
      if (body == null) continue;
      _moveBuf
        ..remove(body) // reported once, in the order it last moved
        ..[body] = asleep.value != 0;
    }
  }

  /// Brings [b2Body]'s type, gravity scale and mass up to [body]'s — plain
  /// fields a game may write at any time — touching only what changed.
  ///
  /// Box2D derives mass from shape area × density, not from
  /// [PhysicsBody.mass], so the configured mass is set on top. Dynamic
  /// bodies only: Box2D gates gravity on inverse mass, not on body type, so
  /// a positive mass on a kinematic body would make it fall.
  void _pushSettings(PhysicsBody body, Box2DBody b2Body) {
    final type = body.effectiveBodyType;
    final scale = body.effectiveGravityScale;
    final mass = type == BodyType.dynamic ? body.mass : 0.0;
    final last = _pushed[body];
    if (last != null &&
        last.type == type.index &&
        last.gravityScale == scale &&
        last.mass == mass) {
      return;
    }
    final h = b2Body.handle;
    final retyped = last != null && last.type != type.index;
    if (retyped) {
      // Expensive: Box2D rebuilds the body's contacts and sums its mass
      // from the shapes again (so the mass below is set again too).
      b2Body.setType(type.index);
      // A sleeping kinematic body ignores the velocity it is given, and the
      // pure-Dart engine never puts one to sleep.
      box2d.b2w_setBodySleepEnabled(
        h,
        type == BodyType.kinematic || !body.canSleep ? 0 : 1,
      );
    }
    if (last == null ? scale != 1.0 : (retyped || last.gravityScale != scale)) {
      box2d.b2w_setBodyGravityScale(h, scale);
    }
    if (mass > 0 && (last == null || retyped || last.mass != mass)) {
      box2d.b2w_setBodyMass(h, mass);
    }
    _pushed[body] = (type: type.index, gravityScale: scale, mass: mass);
  }

  /// Add a [PhysicsBody] to the simulation.
  ///
  /// Creates a matching native body + shape fixture. The [body] object
  /// remains the shared state carrier that the ECS reads from.
  @override
  void addBody(PhysicsBody body) {
    if (!_nativeReady) {
      super.addBody(body);
      return;
    }
    if (_bodyMap.containsKey(body)) return;

    // effectiveBodyType folds in the long-standing `mass <= 0 means static`
    // rule, so kinematic is opt-in and nothing that predates BodyType shifts.
    final type = body.effectiveBodyType;
    final isStatic = type == BodyType.static;

    final b2Body = Box2DBody.ofType(
      worldHandle: _world!.handle,
      bodyTypeIndex: type.index,
      posX: body.position.x,
      posY: body.position.y,
      angle: body.angle,
    );

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
        box2d.b2w_setAngularVelocity(b2Body.handle, body.angularVelocity);
        b2Body.angularVelocity = body.angularVelocity;
      }
    }

    // Register sensor/bullet BEFORE adding shape fixtures so the C wrapper
    // applies isSensor to b2ShapeDef at creation time. Written for every
    // body, solid ones too: the wrapper keeps the flag per handle and never
    // forgets it, and a handle comes back — in a new world, or a slot
    // reused — so a solid body would otherwise inherit `true` from a dead
    // sensor and let everything through.
    box2d.b2w_setBodySensor(b2Body.handle, body.isSensor ? 1 : 0);
    if (body.isBullet) {
      box2d.b2w_setBodyBullet(b2Body.handle, 1);
    }
    if (body.fixedRotation) {
      box2d.b2w_setBodyFixedRotation(b2Body.handle, 1);
    }
    if (!body.isAwake) {
      // Native bodies default to awake — only call out when starting asleep.
      // _syncTransformsFromNative() reads the resulting state back every
      // step, so gameplay-driven wake-ups (collisions, forces) sync normally.
      box2d.b2w_setBodyAwake(b2Body.handle, 0);
    }

    _addShapeFixture(b2Body, body);

    // Gravity scale (it folds together useGravity and the per-body
    // multiplier variable jump height is built on) and the configured mass,
    // after the shapes Box2D sums its own from.
    _pushSettings(body, b2Body);

    // Collision filter can be applied post-creation.
    box2d.b2w_setBodyFilter(
      b2Body.handle,
      body.categoryBits,
      body.maskBits,
      body.groupIndex,
    );

    // Must come after _addShapeFixture: the one-way flag is stored per shape,
    // so there have to be shapes to store it on.
    if (body.isOneWay) {
      box2d.b2w_setBodyOneWay(b2Body.handle, 1, body.oneWayDirection.index);
    }

    // Box2D's per-body linear damping is a different curve from the pure-Dart
    // engine's, but leaving it unset entirely meant air drag simply did not
    // exist on native while it did on web.
    if (body.drag != 0.0) {
      box2d.b2w_setBodyLinearDamping(b2Body.handle, body.drag);
    }
    if (body.angularDamping != 0.0) {
      box2d.b2w_setBodyAngularDamping(b2Body.handle, body.angularDamping);
    }
    // A sleeping kinematic body ignores the velocity it is given, and the
    // pure-Dart engine never puts one to sleep.
    if (!body.canSleep || type == BodyType.kinematic) {
      box2d.b2w_setBodySleepEnabled(b2Body.handle, 0);
    }

    _bodyMap[body] = b2Body;
    _handleToBody[b2Body.handle] = body;
    _ensureBufferCapacity(_bodyMap.length);
  }

  // ── Runtime body mutation (native) ────────────────────────────────────────
  //
  // Each override writes the shared Dart state via super, then pushes the
  // change into the native simulation. Without the native half these were
  // silent no-ops on desktop and mobile while working correctly on web —
  // respawn, moving platforms and one-way platforms all depend on them.

  @override
  void setBodyTransform(PhysicsBody body, double x, double y, {double? angle}) {
    super.setBodyTransform(body, x, y, angle: angle);
    if (!_nativeReady) return;
    final b2Body = _bodyMap[body];
    if (b2Body == null) return;
    // Writes prev and current together, so the interpolator does not draw a
    // streak from the old position to the new one on the next frame.
    b2Body.setTransform(x, y, body.angle);
  }

  @override
  void setBodyType(PhysicsBody body, BodyType type) {
    super.setBodyType(body, type);
    if (!_nativeReady) return;
    final b2Body = _bodyMap[body];
    if (b2Body == null) return;

    // Only on a change: b2Body_SetType is expensive, and PhysicsSystem
    // re-pushes every body field every frame.
    _pushSettings(body, b2Body);
  }

  @override
  void setBodyGravityScale(PhysicsBody body, double scale) {
    super.setBodyGravityScale(body, scale);
    if (!_nativeReady) return;
    final b2Body = _bodyMap[body];
    if (b2Body == null) return;
    _pushSettings(body, b2Body);
  }

  @override
  void setBodyFilter(
    PhysicsBody body, {
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
    if (!_nativeReady) return;
    final b2Body = _bodyMap[body];
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
    PhysicsBody body,
    bool enabled, {
    OneWayDirection direction = OneWayDirection.fromAbove,
  }) {
    super.setBodyOneWay(body, enabled, direction: direction);
    if (!_nativeReady) return;
    final b2Body = _bodyMap[body];
    if (b2Body == null) return;
    box2d.b2w_setBodyOneWay(b2Body.handle, enabled ? 1 : 0, direction.index);
  }

  @override
  void applyLinearImpulse(PhysicsBody body, double ix, double iy) {
    if (!_nativeReady) {
      super.applyLinearImpulse(body, ix, iy);
      return;
    }
    final b2Body = _bodyMap[body];
    if (b2Body == null || !body.isDynamic) return;

    b2Body.applyLinearImpulse(ix, iy);
    // Mirror the resulting velocity change into the Dart body as well.
    //
    // update() pushes body.velocity into native at the top of EVERY frame, so
    // an impulse applied between frames would be integrated by Box2D and then
    // immediately overwritten by the stale Dart value on the next push. The
    // impulse would appear to work for exactly one step and then vanish.
    body.velocity.x += ix * body.inverseMass;
    body.velocity.y += iy * body.inverseMass;
    body.isAwake = true;
    body.sleepTimer = 0.0;
  }

  @override
  void setBodyDamping(PhysicsBody body, double damping) {
    super.setBodyDamping(body, damping);
    if (!_nativeReady) return;
    final b2Body = _bodyMap[body];
    if (b2Body == null) return;
    box2d.b2w_setBodyLinearDamping(b2Body.handle, damping);
  }

  @override
  void setBodyAngularDamping(PhysicsBody body, double damping) {
    super.setBodyAngularDamping(body, damping);
    if (!_nativeReady) return;
    final b2Body = _bodyMap[body];
    if (b2Body == null) return;
    box2d.b2w_setBodyAngularDamping(b2Body.handle, damping);
  }

  @override
  void setBodyCanSleep(PhysicsBody body, bool canSleep) {
    super.setBodyCanSleep(body, canSleep);
    if (!_nativeReady) return;
    final b2Body = _bodyMap[body];
    if (b2Body == null) return;
    box2d.b2w_setBodySleepEnabled(
      b2Body.handle,
      canSleep && body.effectiveBodyType != BodyType.kinematic ? 1 : 0,
    );
    if (!canSleep) box2d.b2w_setBodyAwake(b2Body.handle, 1);
  }

  @override
  void setBodyBullet(PhysicsBody body, bool isBullet) {
    super.setBodyBullet(body, isBullet);
    if (!_nativeReady) return;
    final b2Body = _bodyMap[body];
    if (b2Body == null) return;
    box2d.b2w_setBodyBullet(b2Body.handle, isBullet ? 1 : 0);
  }

  @override
  void setBodyFriction(PhysicsBody body, double friction) {
    super.setBodyFriction(body, friction);
    if (!_nativeReady) return;
    final b2Body = _bodyMap[body];
    if (b2Body == null) return;
    box2d.b2w_setBodyFriction(b2Body.handle, friction);
  }

  @override
  void setBodyRestitution(PhysicsBody body, double restitution) {
    super.setBodyRestitution(body, restitution);
    if (!_nativeReady) return;
    final b2Body = _bodyMap[body];
    if (b2Body == null) return;
    box2d.b2w_setBodyRestitution(b2Body.handle, restitution);
  }

  @override
  void setBodySensor(PhysicsBody body, bool isSensor) {
    super.setBodySensor(body, isSensor);
    if (!_nativeReady) return;
    if (_bodyMap[body] == null) return;
    // Box2D forbids converting a shape between sensor and solid at runtime:
    // it would break the begin/end sensor event contract. Say so rather than
    // pretending the call worked — the Dart flag now disagrees with native,
    // and silently diverging is exactly the class of bug this work exists to
    // remove. Create the body as a sensor and gate it with isActive or the
    // collision filter instead.
    debugPrint(
      'Box2DPhysicsEngine: setBodySensor is create-time only on the native '
      'backend (Box2D limitation) — the native shape is unchanged. Use a '
      'collision filter or isActive to toggle a trigger at runtime.',
    );
  }

  // ── Spatial queries (native) ──────────────────────────────────────────────
  //
  // The inherited pure-Dart implementations are a brute-force scan over every
  // body that also ignores body rotation and compound shapes. These route to
  // Box2D's broad-phase BVH instead, which matters because a platformer casts
  // several rays per character per frame (ground probe, ledge probe, wall
  // probe) and does it forever.
  //
  // Out-parameter buffers are allocated once per engine rather than per cast —
  // a calloc/free pair on every ground probe is real cost at 60 Hz.

  /// How many hits one native [castRayAll]/[queryAABB] call has room for.
  /// Box2D reports one hit per *shape*, so a compound body can take several;
  /// a call that fills the room is asked again with twice as much, up to
  /// [_maxQueryCapacity].
  int _queryCapacity = 32;
  static const int _maxQueryCapacity = 4096;

  Pointer<Int64>? _queryBodyBuffer; // _maxQueryHits handles
  Pointer<Float>? _queryDataBuffer; // _maxQueryHits * 5 floats
  Pointer<Int64>? _rayBodyOut; // single-hit out-params
  Pointer<Float>? _rayFloatOut; // [px, py, nx, ny, fraction]

  void _ensureQueryBuffers() {
    _queryBodyBuffer ??= calloc<Int64>(_queryCapacity);
    _queryDataBuffer ??= calloc<Float>(_queryCapacity * 5);
    _rayBodyOut ??= calloc<Int64>();
    _rayFloatOut ??= calloc<Float>(5);
  }

  /// Whether a query that wrote [count] hits may have run out of room: if
  /// so the buffers grow, and the caller asks again.
  bool _queryFull(int count) {
    if (count < _queryCapacity || _queryCapacity >= _maxQueryCapacity) {
      return false;
    }
    final qb = _queryBodyBuffer;
    final qd = _queryDataBuffer;
    _queryBodyBuffer = null;
    _queryDataBuffer = null;
    if (qb != null) calloc.free(qb);
    if (qd != null) calloc.free(qd);
    _queryCapacity *= 2;
    _ensureQueryBuffers();
    return true;
  }

  void _freeQueryBuffers() {
    final qb = _queryBodyBuffer;
    final qd = _queryDataBuffer;
    final rb = _rayBodyOut;
    final rf = _rayFloatOut;
    _queryBodyBuffer = null;
    _queryDataBuffer = null;
    _rayBodyOut = null;
    _rayFloatOut = null;
    if (qb != null) calloc.free(qb);
    if (qd != null) calloc.free(qd);
    if (rb != null) calloc.free(rb);
    if (rf != null) calloc.free(rf);
    final mp = _moverPoints;
    final ml = _moverPlanes;
    final mb = _moverBodies;
    _moverPoints = null;
    _moverPlanes = null;
    _moverBodies = null;
    if (mp != null) calloc.free(mp);
    if (ml != null) calloc.free(ml);
    if (mb != null) calloc.free(mb);
    final eb = _eventBodies;
    final en = _eventNormal;
    final ea = _eventAsleep;
    final oh = _oneHandle;
    final op = _onePose;
    _eventBodies = null;
    _eventNormal = null;
    _eventAsleep = null;
    _oneHandle = null;
    _onePose = null;
    if (eb != null) calloc.free(eb);
    if (en != null) calloc.free(en);
    if (ea != null) calloc.free(ea);
    if (oh != null) calloc.free(oh);
    if (op != null) calloc.free(op);
  }

  /// Box2D's query filter cannot express "skip these specific bodies", only
  /// category/mask. So when [exclude] is non-empty we have to collect all hits
  /// and filter in Dart; otherwise the cheaper closest-hit call is enough.
  /// Either way a shape the ray starts inside is passed over.
  @override
  RayBodyHit? castRay(Ray ray, {Set<PhysicsBody>? exclude}) {
    if (!_nativeReady) return super.castRay(ray, exclude: exclude);

    if (exclude != null && exclude.isNotEmpty) {
      final all = castRayAll(ray, exclude: exclude);
      return all.isEmpty ? null : all.first;
    }

    _ensureQueryBuffers();
    final origin = ray.origin;
    final translation = ray.direction * ray.maxDistance;

    final hit = box2d.b2w_castRayClosest(
      _world!.handle,
      origin.dx,
      origin.dy,
      translation.dx,
      translation.dy,
      _allCategories,
      _allCategories,
      _rayBodyOut!,
      (_rayFloatOut! + 0),
      (_rayFloatOut! + 1),
      (_rayFloatOut! + 2),
      (_rayFloatOut! + 3),
      (_rayFloatOut! + 4),
    );
    if (hit == 0) return null;

    final body = _handleToBody[_rayBodyOut!.value];
    if (body == null) return null;

    final f = _rayFloatOut!;
    return RayBodyHit(
      body: body,
      point: Offset(f[0], f[1]),
      normal: Offset(f[2], f[3]),
      distance: f[4] * ray.maxDistance,
    );
  }

  @override
  List<RayBodyHit> castRayAll(Ray ray, {Set<PhysicsBody>? exclude}) {
    if (!_nativeReady) return super.castRayAll(ray, exclude: exclude);

    _ensureQueryBuffers();
    final origin = ray.origin;
    final translation = ray.direction * ray.maxDistance;

    int count;
    do {
      count = box2d.b2w_castRayAll(
        _world!.handle,
        origin.dx,
        origin.dy,
        translation.dx,
        translation.dy,
        _allCategories,
        _allCategories,
        _queryBodyBuffer!,
        _queryDataBuffer!,
        _queryCapacity,
      );
    } while (_queryFull(count));

    // One hit per shape, in the tree's order: nearest first, then only a
    // body's nearest hit. A shape the ray starts inside is passed over, as
    // [castRay] passes it (Box2D reports it at fraction 0 with no normal).
    final data = _queryDataBuffer!;
    final order = List<int>.generate(count, (i) => i)
      ..sort((i, j) => data[i * 5 + 4].compareTo(data[j * 5 + 4]));
    final hits = <RayBodyHit>[];
    final seen = <PhysicsBody>{};
    for (final i in order) {
      final base = i * 5;
      if (data[base + 4] == 0 && data[base + 2] == 0 && data[base + 3] == 0) {
        continue;
      }
      final body = _handleToBody[_queryBodyBuffer![i]];
      if (body == null) continue;
      if (exclude != null && exclude.contains(body)) continue;
      if (!seen.add(body)) continue;
      hits.add(
        RayBodyHit(
          body: body,
          point: Offset(data[base], data[base + 1]),
          normal: Offset(data[base + 2], data[base + 3]),
          distance: data[base + 4] * ray.maxDistance,
        ),
      );
    }
    return hits;
  }

  // ── Character movers ──────────────────────────────────────────────────────
  //
  // Answered in C (b2w_moverCollide / b2w_moverCast) with the same rules as
  // the pure-Dart engine, on Box2D's own GJK and shape cast.

  /// Most planes one [collideMover] call reports.
  static const int _maxMoverPlanes = 32;

  Pointer<Float>? _moverPoints; // up to 8 points, x and y
  Pointer<Float>? _moverPlanes; // _maxMoverPlanes * 5 floats
  Pointer<Int64>? _moverBodies; // _maxMoverPlanes handles

  /// Writes [shape]'s core points into the buffer; returns their count.
  int _moverShape(MoverShape shape) {
    _moverPoints ??= calloc<Float>(16);
    _moverPlanes ??= calloc<Float>(_maxMoverPlanes * 5);
    _moverBodies ??= calloc<Int64>(_maxMoverPlanes);
    final out = _moverPoints!;
    final points = shape.points;
    for (var i = 0; i < points.length; i++) {
      out[2 * i] = points[i].dx;
      out[2 * i + 1] = points[i].dy;
    }
    return points.length;
  }

  int _moverIgnore(MoverFilter filter) {
    final ignore = filter.ignore;
    return ignore == null ? 0 : (_bodyMap[ignore]?.handle ?? 0);
  }

  @override
  List<MoverPlane> collideMover(
    MoverShape shape,
    Offset position, {
    MoverFilter filter = const MoverFilter(),
  }) {
    if (!_nativeReady) {
      return super.collideMover(shape, position, filter: filter);
    }
    final count = _moverShape(shape);
    final n = box2d.b2w_moverCollide(
      _world!.handle,
      position.dx,
      position.dy,
      _moverPoints!,
      count,
      shape.radius,
      filter.categoryBits,
      filter.maskBits,
      _moverIgnore(filter),
      _moverPlanes!,
      _moverBodies!,
      _maxMoverPlanes,
    );
    final f = _moverPlanes!;
    final planes = <MoverPlane>[];
    for (var i = 0; i < n; i++) {
      final plane = MoverPlane(
        normal: Offset(f[5 * i], f[5 * i + 1]),
        offset: f[5 * i + 2],
        point: Offset(f[5 * i + 3] + position.dx, f[5 * i + 4] + position.dy),
        body: _handleToBody[_moverBodies![i]],
      );
      // A chain with no thickness is in Box2D twice, once each way round
      // (see _addSingleShape), so the same plane can come twice.
      final twice = planes.any(
        (p) =>
            p.body == plane.body &&
            (p.normal - plane.normal).distanceSquared < 1e-8 &&
            (p.offset - plane.offset).abs() < 1e-4 &&
            (p.point - plane.point).distanceSquared < 1e-6,
      );
      if (!twice) planes.add(plane);
    }
    return planes;
  }

  @override
  double castMover(
    MoverShape shape,
    Offset position,
    Offset translation, {
    MoverFilter filter = const MoverFilter(),
  }) {
    if (!_nativeReady) {
      return super.castMover(shape, position, translation, filter: filter);
    }
    final count = _moverShape(shape);
    return box2d.b2w_moverCast(
      _world!.handle,
      position.dx,
      position.dy,
      _moverPoints!,
      count,
      shape.radius,
      translation.dx,
      translation.dy,
      filter.categoryBits,
      filter.maskBits,
      _moverIgnore(filter),
    );
  }

  @override
  List<PhysicsBody> queryAABB(Rect rect) {
    if (!_nativeReady) return super.queryAABB(rect);

    _ensureQueryBuffers();
    int count;
    do {
      count = box2d.b2w_queryAABB(
        _world!.handle,
        rect.left,
        rect.top,
        rect.right,
        rect.bottom,
        _allCategories,
        _allCategories,
        _queryBodyBuffer!,
        _queryCapacity,
      );
    } while (_queryFull(count));

    final result = <PhysicsBody>[];
    for (var i = 0; i < count; i++) {
      final body = _handleToBody[_queryBodyBuffer![i]];
      if (body != null) result.add(body);
    }
    return result;
  }

  @override
  void removeBody(PhysicsBody body) {
    if (!_nativeReady) {
      super.removeBody(body);
      return;
    }
    final b2Body = _bodyMap.remove(body);
    if (b2Body == null) return;
    _handleToBody.remove(b2Body.handle);
    _pushed.remove(body);
    // Box2D destroys the body's joints with it; they, and the Dart joints on
    // it, leave [joints] too.
    for (final joint in joints) {
      final pair = _jointBodies[joint];
      final holds = pair != null
          ? pair.$1 == body || pair.$2 == body
          : joint.bodies.contains(body);
      if (holds) removeJoint(joint);
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

    final strokePaint = Paint()
      ..color = const Color(0xFF00FF88)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.5;

    final staticPaint = Paint()
      ..color = const Color(0xFF888888)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.5;

    final extraPaint = Paint()
      ..color = const Color(0xFF00CCFF)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.0;

    for (final entry in _bodyMap.entries) {
      final body = entry.key;
      final b2 = entry.value;
      final paint = body.mass <= 0.0 ? staticPaint : strokePaint;

      canvas.save();
      canvas.translate(b2.currentX, b2.currentY);
      canvas.rotate(b2.currentAngle);

      _drawShape(canvas, body.shape, paint);
      for (final extra in body.additionalShapes) {
        _drawShape(canvas, extra, extraPaint);
      }

      canvas.restore();
    }
  }

  void _drawShape(Canvas canvas, CollisionShape shape, Paint paint) {
    if (shape is CircleShape) {
      final c = shape.center;
      canvas.drawCircle(c, shape.radius, paint);
      canvas.drawLine(c, c + Offset(shape.radius, 0), paint);
    } else if (shape is CapsuleShape) {
      canvas.drawCircle(shape.center1, shape.radius, paint);
      canvas.drawCircle(shape.center2, shape.radius, paint);
      canvas.drawLine(shape.center1, shape.center2, paint);
    } else if (shape is ChainShape) {
      for (int i = 0; i < shape.vertices.length - 1; i++) {
        canvas.drawLine(shape.vertices[i], shape.vertices[i + 1], paint);
      }
      if (shape.loop && shape.vertices.length > 1) {
        canvas.drawLine(shape.vertices.last, shape.vertices.first, paint);
      }
    } else if (shape is SegmentShape) {
      canvas.drawLine(shape.point1, shape.point2, paint);
    } else if (shape is RectangleShape) {
      canvas.drawRect(
        Rect.fromCenter(
          center: Offset.zero,
          width: shape.width,
          height: shape.height,
        ),
        paint,
      );
    } else if (shape is PolygonShape) {
      if (shape.vertices.isNotEmpty) {
        final path = Path()
          ..moveTo(shape.vertices.first.dx, shape.vertices.first.dy);
        for (final v in shape.vertices.skip(1)) {
          path.lineTo(v.dx, v.dy);
        }
        path.close();
        canvas.drawPath(path, paint);
      }
    }
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
  List<PhysicsBody> get bodies =>
      _nativeReady ? List.unmodifiable(_bodyMap.keys) : super.bodies;

  @override
  int get bodyCount => _nativeReady ? _bodyMap.length : super.bodyCount;

  /// Override world gravity at runtime — updates both the Dart gravity vector
  /// and the native Box2D world so they stay in sync.
  @override
  void setGravity(double gx, double gy) {
    gravity.x = gx;
    gravity.y = gy;
    _world?.setGravity(gx, gy);
  }

  // ── Contact event polling ─────────────────────────────────────────────────
  //
  // Every step's events are read right after it and kept until the next
  // [update] (see [_gatherStepEvents]), as the pure-Dart engine keeps its
  // own: poll them after [update], before the next one.

  /// Iterate all begin-touch contact events from the last [update].
  ///
  /// [fn] receives the two colliding [PhysicsBody] objects and the contact
  /// normal (nx, ny). Calls [fn] only for pairs where both bodies are tracked.
  @override
  void pollContactBeginEvents(
    void Function(PhysicsBody a, PhysicsBody b, double nx, double ny) fn,
  ) {
    if (!_nativeReady) {
      super.pollContactBeginEvents(fn);
      return;
    }
    for (final (a, b, nx, ny) in _contactBeginBuf) {
      fn(a, b, nx, ny);
    }
  }

  /// Iterate all end-touch (separation) contact events from the last
  /// [update].
  ///
  /// Unlike [pollContactBeginEvents] this has no contact normal — Box2D's
  /// end-touch event only reports which bodies stopped touching.
  @override
  void pollContactEndEvents(void Function(PhysicsBody a, PhysicsBody b) fn) {
    if (!_nativeReady) {
      super.pollContactEndEvents(fn);
      return;
    }
    for (final (a, b) in _contactEndBuf) {
      fn(a, b);
    }
  }

  // ── Body movement event polling ───────────────────────────────────────────

  @override
  void pollBodyMoveEvents(
    void Function(PhysicsBody body, {required bool fellAsleep}) fn,
  ) {
    if (!_nativeReady) {
      super.pollBodyMoveEvents(fn);
      return;
    }
    for (final MapEntry(key: body, value: fellAsleep) in _moveBuf.entries) {
      fn(body, fellAsleep: fellAsleep);
    }
  }

  // ── Sensor event polling ──────────────────────────────────────────────────

  /// Iterate sensor-begin events (sensor body first touched by visitor).
  @override
  void pollSensorBeginEvents(
    void Function(PhysicsBody sensor, PhysicsBody visitor) fn,
  ) {
    if (!_nativeReady) {
      super.pollSensorBeginEvents(fn);
      return;
    }
    for (final e in _nativeSensorBeginBuf) {
      fn(e.sensor, e.visitor);
    }
  }

  /// Iterate sensor-end events (visitor left the sensor).
  @override
  void pollSensorEndEvents(
    void Function(PhysicsBody sensor, PhysicsBody visitor) fn,
  ) {
    if (!_nativeReady) {
      super.pollSensorEndEvents(fn);
      return;
    }
    for (final e in _nativeSensorEndBuf) {
      fn(e.sensor, e.visitor);
    }
  }

  // ── Joint factory (Box2D FFI backend) ────────────────────────────────────
  //
  // These methods create native joints and also call addJoint() so the base
  // class tracks them for dispose() and the Dart fallback can list them.
  // Box2DJoint.applyConstraint() is a no-op — Box2D handles it natively.

  Box2DJoint? createRevoluteJoint(
    PhysicsBody bodyA,
    PhysicsBody bodyB,
    Offset worldAnchor,
  ) {
    if (!_nativeReady) return null;
    final b2A = _bodyMap[bodyA];
    final b2B = _bodyMap[bodyB];
    if (b2A == null || b2B == null) return null;
    final joint = Box2DJointFactory.createRevolute(
      _world!.handle,
      b2A.handle,
      b2B.handle,
      worldAnchor,
    );
    addJoint(joint);
    _jointBodies[joint] = (bodyA, bodyB);
    return joint;
  }

  Box2DJoint? createPrismaticJoint(
    PhysicsBody bodyA,
    PhysicsBody bodyB,
    Offset worldAnchor,
    Offset axis,
  ) {
    if (!_nativeReady) return null;
    final b2A = _bodyMap[bodyA];
    final b2B = _bodyMap[bodyB];
    if (b2A == null || b2B == null) return null;
    final joint = Box2DJointFactory.createPrismatic(
      _world!.handle,
      b2A.handle,
      b2B.handle,
      worldAnchor,
      axis,
    );
    addJoint(joint);
    _jointBodies[joint] = (bodyA, bodyB);
    return joint;
  }

  Box2DJoint? createDistanceJoint(
    PhysicsBody bodyA,
    PhysicsBody bodyB, {
    required double minLength,
    required double maxLength,
  }) {
    if (!_nativeReady) return null;
    final b2A = _bodyMap[bodyA];
    final b2B = _bodyMap[bodyB];
    if (b2A == null || b2B == null) return null;
    final joint = Box2DJointFactory.createDistance(
      _world!.handle,
      b2A.handle,
      b2B.handle,
      minLength,
      maxLength,
    );
    addJoint(joint);
    _jointBodies[joint] = (bodyA, bodyB);
    return joint;
  }

  Box2DJoint? createWeldJoint(
    PhysicsBody bodyA,
    PhysicsBody bodyB,
    Offset worldAnchor,
  ) {
    if (!_nativeReady) return null;
    final b2A = _bodyMap[bodyA];
    final b2B = _bodyMap[bodyB];
    if (b2A == null || b2B == null) return null;
    final joint = Box2DJointFactory.createWeld(
      _world!.handle,
      b2A.handle,
      b2B.handle,
      worldAnchor,
    );
    addJoint(joint);
    _jointBodies[joint] = (bodyA, bodyB);
    return joint;
  }

  Box2DJoint? createWheelJoint(
    PhysicsBody bodyA,
    PhysicsBody bodyB,
    Offset worldAnchor,
    Offset axis,
  ) {
    if (!_nativeReady) return null;
    final b2A = _bodyMap[bodyA];
    final b2B = _bodyMap[bodyB];
    if (b2A == null || b2B == null) return null;
    final joint = Box2DJointFactory.createWheel(
      _world!.handle,
      b2A.handle,
      b2B.handle,
      worldAnchor,
      axis,
    );
    addJoint(joint);
    _jointBodies[joint] = (bodyA, bodyB);
    return joint;
  }

  // ── Unified joint API overrides (delegates to native Box2D) ─────────────

  @override
  JointConstraint addRevoluteJoint(
    PhysicsBody a,
    PhysicsBody b,
    Offset worldAnchor,
  ) {
    final native = createRevoluteJoint(a, b, worldAnchor);
    return native ?? super.addRevoluteJoint(a, b, worldAnchor);
  }

  @override
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
    final mn = minLength ?? l;
    final mx = maxLength ?? l;
    final native = createDistanceJoint(a, b, minLength: mn, maxLength: mx);
    // Box2D's spring is a frequency (hertz) and a damping ratio.
    if (native != null && stiffness > 0) {
      native.setDistanceSpring(stiffness, damping);
    }
    return native ??
        super.addDistanceJoint(
          a,
          b,
          length: l,
          minLength: minLength,
          maxLength: maxLength,
          stiffness: stiffness,
          damping: damping,
        );
  }

  @override
  JointConstraint addWeldJoint(PhysicsBody a, PhysicsBody b) {
    final anchor = Offset(
      (a.position.x + b.position.x) / 2,
      (a.position.y + b.position.y) / 2,
    );
    final native = createWeldJoint(a, b, anchor);
    return native ?? super.addWeldJoint(a, b);
  }

  @override
  JointConstraint addPrismaticJoint(PhysicsBody a, PhysicsBody b, Offset axis) {
    final anchor = Offset(b.position.x, b.position.y);
    final native = createPrismaticJoint(a, b, anchor, axis);
    return native ?? super.addPrismaticJoint(a, b, axis);
  }

  @override
  JointConstraint addWheelJoint(PhysicsBody a, PhysicsBody b, Offset axis) {
    // The wheel turns about its own centre, as the pure-Dart wheel does.
    final anchor = Offset(b.position.x, b.position.y);
    final native = createWheelJoint(a, b, anchor, axis);
    return native ?? super.addWheelJoint(a, b, axis);
  }

  /// Destroy a Box2D joint and remove it from the engine's joint list.
  void destroyJoint(Box2DJoint joint) {
    joint.destroy();
    removeJoint(joint);
  }

  /// Removes [joint]; a native one is destroyed, so its bodies come apart.
  @override
  void removeJoint(JointConstraint joint) {
    if (joint is Box2DJoint) {
      joint.destroy();
      _jointBodies.remove(joint);
    }
    super.removeJoint(joint);
  }

  // ── Impact callback ───────────────────────────────────────────────────────

  /// Register a Dart callback for every contact whose bodies closed faster
  /// than [speedThreshold] (units per second) in a step: Box2D's hit events.
  ///
  /// It arrives through a [NativeCallable.listener], on this isolate's event
  /// loop after the step that saw it.
  ///
  /// [callback] receives packed body handles (int) and the speed they closed
  /// at.
  /// Use [physicsBodyFromHandle] to resolve handles to [PhysicsBody] objects.
  void registerImpactCallback(
    void Function(int bodyA, int bodyB, double speed) callback, {
    double speedThreshold = 100.0,
  }) {
    if (!_nativeReady) return;
    _impactCallable?.close();
    _impactCallable = NativeCallable<box2d.ImpactCallbackFnFunction>.listener(
      callback,
    );
    box2d.b2w_setImpactCallback(
      _world!.handle,
      _impactCallable!.nativeFunction,
      speedThreshold,
    );
  }

  /// Resolve a packed body handle (from the impact callback) to a [PhysicsBody].
  PhysicsBody? physicsBodyFromHandle(int handle) => _handleToBody[handle];

  /// Render interpolation alpha from the current frame (for ECS bridge use).
  @override
  double get alpha => _nativeReady ? _loop!.alpha : super.alpha;

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

    // Destroy native joints before bodies/world. Box2DJoint.destroy() is
    // documented as "must be called before the world is disposed" — the base
    // class's dispose() only clears the joint list without destroying
    // anything, so this must happen explicitly here.
    for (final joint in joints) {
      if (joint is Box2DJoint) joint.destroy();
    }

    // Destroy Box2D bodies before the world (order matters in Box2D 3.0).
    for (final b2Body in _bodyMap.values) {
      b2Body.destroy();
    }
    _bodyMap.clear();
    _handleToBody.clear();
    _pushed.clear();
    _jointBodies.clear();
    _contactBeginBuf.clear();
    _contactEndBuf.clear();
    _nativeSensorBeginBuf.clear();
    _nativeSensorEndBuf.clear();
    _moveBuf.clear();

    // Free zero-copy buffers. calloc.free() is required — Pointer has no .free().
    final hb = _handleBuffer;
    final tb = _transformBuffer;
    _handleBuffer = null;
    _transformBuffer = null;
    if (hb != null) calloc.free(hb);
    if (tb != null) calloc.free(tb);
    _freeQueryBuffers();
    _bufferCapacity = 0; // MUST reset — dispose() nulls the buffers; without
    // this, _ensureBufferCapacity() sees the old non-zero capacity on the next
    // initialize() cycle and skips re-allocation, leaving _handleBuffer null.
    // _syncTransformsFromNative() then throws a null assertion on every frame,
    // Box2D steps but positions are never written back to Dart, bodies freeze
    // at their spawn positions and act as invisible ghost colliders.

    _world?.dispose();
    _world = null;
    _loop = null;
    _nativeReady = false;
    // Always clear the base-class _bodies list too. If the native init ever
    // fails and addBody fell through to super.addBody(), those bodies live in
    // _bodies and will cause ghost collisions unless we clear them here.
    super.dispose();
  }

  // ── Private helpers ───────────────────────────────────────────────────────

  void _addShapeFixture(Box2DBody b2Body, PhysicsBody body) {
    // Box2D gives static and kinematic bodies no mass whatever their shapes'
    // density, so every shape has density 1: a body that turns dynamic later
    // then has its shapes' mass and rotational inertia, which
    // b2w_setBodyMass rescales to the configured mass.
    const density = 1.0;
    _addSingleShape(
      b2Body.handle,
      body.shape,
      density,
      body.friction,
      body.restitution,
    );
    for (final extra in body.additionalShapes) {
      _addSingleShape(
        b2Body.handle,
        extra,
        density,
        body.friction,
        body.restitution,
      );
    }
  }

  void _addSingleShape(
    int handle,
    CollisionShape shape,
    double density,
    double friction,
    double restitution,
  ) {
    if (shape is CircleShape) {
      box2d.b2w_addCircleShape(
        handle,
        shape.center.dx,
        shape.center.dy,
        shape.radius,
        density,
        friction,
        restitution,
      );
    } else if (shape is CapsuleShape) {
      box2d.b2w_addCapsuleShape(
        handle,
        shape.center1.dx,
        shape.center1.dy,
        shape.center2.dx,
        shape.center2.dy,
        shape.radius,
        density,
        friction,
        restitution,
      );
    } else if (shape is SegmentShape) {
      // The pure-Dart engine collides a segment as a capsule [thickness]
      // round it, so this one does too.
      _addLine(
        handle,
        shape.point1,
        shape.point2,
        shape.thickness,
        density,
        friction,
        restitution,
      );
    } else if (shape is ChainShape) {
      if (shape.thickness > 0) {
        // A capsule per segment, [thickness] round it, as the pure-Dart
        // engine collides each one: solid from both sides, and smooth where
        // two meet (their round ends overlap).
        final v = shape.vertices;
        final n = shape.loop && v.length > 2 ? v.length : v.length - 1;
        for (var i = 0; i < n; i++) {
          _addLine(
            handle,
            v[i],
            v[(i + 1) % v.length],
            shape.thickness,
            density,
            friction,
            restitution,
          );
        }
        return;
      }
      // A Box2D chain collides on one side only, the right of its direction.
      // The pure-Dart engine's collides on both, so the chain goes in twice,
      // once each way round.
      final pts = _chainPoints(shape);
      if (pts == null) return;
      final ptr = calloc<Float>(pts.length);
      try {
        for (final points in [pts, _reversedPoints(pts)]) {
          ptr.asTypedList(pts.length).setAll(0, points);
          box2d.b2w_addChainShape(
            handle,
            ptr,
            pts.length ~/ 2,
            shape.loop ? 1 : 0,
            friction,
            restitution,
          );
        }
      } finally {
        calloc.free(ptr);
      }
    } else if (shape is RectangleShape) {
      box2d.b2w_addBoxShape(
        handle,
        shape.width / 2,
        shape.height / 2,
        density,
        friction,
        restitution,
      );
    } else if (shape is RoundedPolygonShape) {
      final verts = Float32List(shape.vertices.length * 2);
      for (int i = 0; i < shape.vertices.length; i++) {
        verts[i * 2] = shape.vertices[i].dx;
        verts[i * 2 + 1] = shape.vertices[i].dy;
      }
      final ptr = calloc<Float>(verts.length);
      try {
        ptr.asTypedList(verts.length).setAll(0, verts);
        box2d.b2w_addRoundedPolygonShape(
          handle,
          ptr,
          shape.vertices.length,
          shape.cornerRadius,
          density,
          friction,
          restitution,
        );
      } finally {
        calloc.free(ptr);
      }
    } else if (shape is PolygonShape) {
      final verts = Float32List(shape.vertices.length * 2);
      for (int i = 0; i < shape.vertices.length; i++) {
        verts[i * 2] = shape.vertices[i].dx;
        verts[i * 2 + 1] = shape.vertices[i].dy;
      }
      final ptr = calloc<Float>(verts.length);
      try {
        ptr.asTypedList(verts.length).setAll(0, verts);
        box2d.b2w_addPolygonShape(
          handle,
          ptr,
          shape.vertices.length,
          density,
          friction,
          restitution,
        );
      } finally {
        calloc.free(ptr);
      }
    }
  }

  /// A line from [p1] to [p2]: a capsule [radius] round it, or a bare
  /// segment when [radius] is 0.
  static void _addLine(
    int handle,
    Offset p1,
    Offset p2,
    double radius,
    double density,
    double friction,
    double restitution,
  ) {
    if (radius > 0) {
      box2d.b2w_addCapsuleShape(
        handle,
        p1.dx,
        p1.dy,
        p2.dx,
        p2.dy,
        radius,
        density,
        friction,
        restitution,
      );
    } else {
      box2d.b2w_addSegmentShape(
        handle,
        p1.dx,
        p1.dy,
        p2.dx,
        p2.dy,
        density,
        friction,
        restitution,
      );
    }
  }

  /// [points] (x, y pairs) in the opposite order.
  static Float32List _reversedPoints(Float32List points) {
    final n = points.length ~/ 2;
    final out = Float32List(points.length);
    for (var i = 0; i < n; i++) {
      out[i * 2] = points[(n - 1 - i) * 2];
      out[i * 2 + 1] = points[(n - 1 - i) * 2 + 1];
    }
    return out;
  }

  /// [shape]'s points as Box2D takes a chain: at least 4, and an open
  /// chain's first and last are ghosts that only shape its ends. The
  /// pure-Dart engine collides with every segment of an open chain, so a
  /// ghost is added beyond each end (the end segment carried on) and both
  /// backends collide with the same segments; a three-point loop gets a
  /// midpoint. Points closer than Box2D's linear slop to the one before are
  /// dropped first (a segment of no length has no normal). Null when there
  /// is no chain to make.
  static Float32List? _chainPoints(ChainShape shape) {
    const slop = 0.5; // B2_LINEAR_SLOP at 100 units a metre.
    final v = <Offset>[];
    for (final p in shape.vertices) {
      if (v.isEmpty || (p - v.last).distance > slop) v.add(p);
    }
    if (shape.loop && v.length > 1 && (v.first - v.last).distance <= slop) {
      v.removeLast();
    }
    final List<Offset> points;
    if (shape.loop) {
      if (v.length < 3) return null;
      points = v.length == 3 ? [v[0], (v[0] + v[1]) / 2, v[1], v[2]] : v;
    } else {
      if (v.length < 2) return null;
      final last = v.length - 1;
      points = [v[0] + (v[0] - v[1]), ...v, v[last] + (v[last] - v[last - 1])];
    }
    final out = Float32List(points.length * 2);
    for (var i = 0; i < points.length; i++) {
      out[i * 2] = points[i].dx;
      out[i * 2 + 1] = points[i].dy;
    }
    return out;
  }

  void _ensureBufferCapacity(int needed) {
    if (needed <= _bufferCapacity) return;
    // Grow by 1.5× to amortise reallocations.
    final cap = (needed * 1.5).ceil().clamp(8, 1 << 20);
    final oldH = _handleBuffer;
    final oldT = _transformBuffer;
    if (oldH != null) calloc.free(oldH);
    if (oldT != null) calloc.free(oldT);
    _handleBuffer = calloc<Int64>(cap);
    _transformBuffer = calloc<Float>(cap * 7);
    _bufferCapacity = cap;
  }

  /// Extract all body transforms from native memory in a single FFI call.
  ///
  /// Zero-copy: [_transformBuffer] is native calloc memory; `asTypedList`
  /// creates a [Float32List] view — no copy, no GC pressure.
  void _syncTransformsFromNative() {
    final n = _bodyMap.length;
    if (n == 0) return;

    // Write handle IDs into the native handle buffer.
    final Int64List handles = _handleBuffer!.asTypedList(n);
    int i = 0;
    for (final b2Body in _bodyMap.values) {
      handles[i++] = b2Body.handle;
    }

    // One FFI call writes x, y, angle, vx, vy, awake, spin for every body.
    box2d.b2w_bulkExtractTransforms(_handleBuffer!, _transformBuffer!, n);

    // Zero-copy view — reads directly from native memory.
    final Float32List trs = _transformBuffer!.asTypedList(n * 7);

    // Scatter results back to Dart body objects.
    i = 0;
    for (final entry in _bodyMap.entries) {
      final dartBody = entry.key; // PhysicsBody (ECS holds a reference to this)
      final b2Body = entry.value; // Box2DBody (FFI handle + prev/current state)
      final base = i * 7;

      b2Body.currentX = trs[base];
      b2Body.currentY = trs[base + 1];
      b2Body.currentAngle = trs[base + 2];
      b2Body.velocityX = trs[base + 3];
      b2Body.velocityY = trs[base + 4];
      b2Body.angularVelocity = trs[base + 6];

      // Write through to PhysicsBody so existing ECS bridge works unchanged.
      dartBody.position.x = b2Body.currentX;
      dartBody.position.y = b2Body.currentY;
      dartBody.angle = b2Body.currentAngle;
      dartBody.velocity.x = b2Body.velocityX;
      dartBody.velocity.y = b2Body.velocityY;
      dartBody.angularVelocity = b2Body.angularVelocity;
      // Sleep is a dynamic-body concept — static bodies always report
      // "not awake" natively and would otherwise flip PhysicsBody.isAwake
      // to false even though the user never asked for that.
      if (dartBody.mass > 0) {
        dartBody.isAwake = trs[base + 5] != 0.0;
      }

      i++;
    }
  }
}
