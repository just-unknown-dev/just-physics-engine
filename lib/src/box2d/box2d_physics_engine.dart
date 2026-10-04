import 'dart:ffi'
    hide Size; // hide dart:ffi's Size to avoid conflict with dart:ui Size
import 'dart:typed_data';
import 'dart:ui'
    show Canvas, Color, Offset, Paint, PaintingStyle, Path, Rect, Size;

import 'package:flutter/foundation.dart' show debugPrint;

import 'package:ffi/ffi.dart';
import 'package:just_dart/just_dart.dart' show Vector2;

// PhysicsBody, CollisionShape, CircleShape, RectangleShape, PolygonShape are
// all part-files of physics_engine.dart — import the library, not the parts.
import '../physics_2d/physics_engine.dart';
import '../physics_2d/ray_2d.dart';
import 'box2d_body.dart';
import 'box2d_world.dart';
import 'ffi/box2d_bindings.dart' show ImpactCallbackFnFunction;
import 'ffi/box2d_library.dart' show box2d, loadBox2DLibrary;
import 'box2d_joint.dart';
import 'physics_game_loop.dart';

/// Box2D v3.0 physics engine adapter exposing the same duck-typed API as
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
  Pointer<Float>? _transformBuffer; // 6 floats per body: x,y,angle,vx,vy,pad
  int _bufferCapacity = 0;

  // ── Dart-side sensor event accumulators ──────────────────────────────────
  // Populated during update() immediately after b2w_step so events are never
  // lost due to ticker ordering between the game loop and the demo ticker.

  final List<({PhysicsBody sensor, PhysicsBody visitor})>
  _nativeSensorBeginBuf = [];
  final List<({PhysicsBody sensor, PhysicsBody visitor})>
  _nativeSensorEndBuf = [];

  // ── NativeCallable for cross-thread impact audio callback ──────────────────

  NativeCallable<ImpactCallbackFnFunction>? _impactCallable;

  // ── Diagnostics ────────────────────────────────────────────────────────────

  int _lastStepCount = 0;
  double _lastStepMs = 0.0;
  int _lastContactCount = 0;

  // Persistent stopwatch — reused every frame to avoid per-update allocation.
  final Stopwatch _stepStopwatch = Stopwatch();

  /// Number of Box2D sub-steps per fixed tick (4 recommended).
  final int subSteps;

  /// Number of native worker threads (0 → hardware_concurrency − 1).
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
    // Reset buffer state so a second initialize() call (after dispose()) does
    // not skip re-allocation and cause a null assertion in _syncTransformsFromNative.
    _bufferCapacity = 0;
    _handleBuffer = null;
    _transformBuffer = null;
    try {
      loadBox2DLibrary(); // throws if the shared library is missing
      _world = Box2DWorld(
        gravityX: gravity.x,
        gravityY: gravity.y,
        numThreads: numThreads,
        subSteps: subSteps,
      );
      _loop = PhysicsGameLoop(_world!);
      _nativeReady = true;
    } catch (e) {
      // Native library not compiled yet (e.g. Box2D submodule not initialized).
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
      b2Body.capturePrevious();
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
        if (body.acceleration.x != 0.0 || body.acceleration.y != 0.0) {
          box2d.b2w_applyForce(
            b2Body.handle,
            body.acceleration.x * body.mass,
            body.acceleration.y * body.mass,
          );
          body.acceleration.setZero();
        }
        if (body.torque != 0.0) {
          box2d.b2w_applyTorque(b2Body.handle, body.torque);
          body.torque = 0.0;
        }
      }
    }

    _lastStepCount = _loop!.advance(deltaTime);
    _syncTransformsFromNative();
    _lastContactCount = box2d.b2w_getContactBeginCount(_world!.handle);
    _pumpNativeSensorEvents();

    _stepStopwatch.stop();
    _lastStepMs = _stepStopwatch.elapsedMicroseconds / 1000.0;
  }

  /// Drain native sensor events into Dart-side buffers immediately after
  /// b2w_step, so events survive until [pollSensorBeginEvents] is called
  /// regardless of ticker ordering.
  void _pumpNativeSensorEvents() {
    _nativeSensorBeginBuf.clear();
    _nativeSensorEndBuf.clear();

    final beginCount = box2d.b2w_getSensorBeginCount(_world!.handle);
    if (beginCount > 0) {
      final outSensor = calloc<Int64>();
      final outVisitor = calloc<Int64>();
      try {
        for (int i = 0; i < beginCount; i++) {
          box2d.b2w_getSensorBeginEvent(_world!.handle, i, outSensor, outVisitor);
          final pSensor = _handleToBody[outSensor.value];
          final pVisitor = _handleToBody[outVisitor.value];
          if (pSensor != null && pVisitor != null) {
            _nativeSensorBeginBuf.add((sensor: pSensor, visitor: pVisitor));
          }
        }
      } finally {
        calloc
          ..free(outSensor)
          ..free(outVisitor);
      }
    }

    final endCount = box2d.b2w_getSensorEndCount(_world!.handle);
    if (endCount > 0) {
      final outSensor = calloc<Int64>();
      final outVisitor = calloc<Int64>();
      try {
        for (int i = 0; i < endCount; i++) {
          box2d.b2w_getSensorEndEvent(_world!.handle, i, outSensor, outVisitor);
          final pSensor = _handleToBody[outSensor.value];
          final pVisitor = _handleToBody[outVisitor.value];
          if (pSensor != null && pVisitor != null) {
            _nativeSensorEndBuf.add((sensor: pSensor, visitor: pVisitor));
          }
        }
      } finally {
        calloc
          ..free(outSensor)
          ..free(outVisitor);
      }
    }
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
    if (body.effectiveGravityScale != 1.0) {
      // Native default is already 1.0 — only call out when it differs, to
      // avoid an unnecessary FFI round-trip per body. effectiveGravityScale
      // folds together the useGravity switch and the per-body multiplier that
      // variable jump height is built on.
      box2d.b2w_setBodyGravityScale(b2Body.handle, body.effectiveGravityScale);
    }
    if (!body.isAwake) {
      // Native bodies default to awake — only call out when starting asleep.
      // _syncTransformsFromNative() reads the resulting state back every
      // step, so gameplay-driven wake-ups (collisions, forces) sync normally.
      box2d.b2w_setBodyAwake(b2Body.handle, 0);
    }

    _addShapeFixture(b2Body, body);

    if (type == BodyType.dynamic) {
      // Box2D derives mass from shape area × density (always 1.0/0.0 above),
      // not from PhysicsBody.mass — override it so force/impulse magnitudes
      // and collision response match what the caller configured.
      //
      // DYNAMIC ONLY. Box2D gates gravity on inverse mass, not on body type
      // (solver.c: `gravityScale = sim->invMass > 0 ? sim->gravityScale : 0`),
      // so forcing a positive mass onto a kinematic body gives it a non-zero
      // inverse mass and it starts free-falling — silently turning every
      // moving platform into a falling one.
      box2d.b2w_setBodyMass(b2Body.handle, body.mass);
    }

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
      box2d.b2w_setBodyOneWay(
        b2Body.handle,
        1,
        body.oneWayDirection.index,
      );
    }

    // Box2D's per-body linear damping is a different curve from the pure-Dart
    // engine's, but leaving it unset entirely meant air drag simply did not
    // exist on native while it did on web.
    if (body.drag != 0.0) {
      box2d.b2w_setBodyLinearDamping(b2Body.handle, body.drag);
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
  void setBodyTransform(
    PhysicsBody body,
    double x,
    double y, {
    double? angle,
  }) {
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

    final target = body.effectiveBodyType;
    // b2Body_SetType rebuilds contacts and recomputes mass properties, so it
    // is far too expensive to call every frame — and PhysicsSystem re-pushes
    // every other body field on every frame, which makes an unguarded setter
    // exactly that. Skip when nothing actually changed.
    if (b2Body.typeIndex == target.index) return;

    b2Body.setType(target.index);
    // Changing type RESETS the mass override back to shape area x density,
    // so the configured mass has to be re-applied or the body silently gets
    // a different weight than the caller asked for.
    if (target == BodyType.dynamic) {
      box2d.b2w_setBodyMass(b2Body.handle, body.mass);
    }
  }

  @override
  void setBodyGravityScale(PhysicsBody body, double scale) {
    super.setBodyGravityScale(body, scale);
    if (!_nativeReady) return;
    final b2Body = _bodyMap[body];
    if (b2Body == null) return;
    box2d.b2w_setBodyGravityScale(b2Body.handle, body.effectiveGravityScale);
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
    box2d.b2w_setBodyOneWay(
      b2Body.handle,
      enabled ? 1 : 0,
      direction.index,
    );
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

  /// Maximum hits one [castRayAll]/[queryAABB] call can report.
  ///
  /// Box2D reports one hit per *shape*, so a compound body can occupy several
  /// slots. Beyond this the cast is terminated early rather than growing the
  /// buffer mid-query.
  static const int _maxQueryHits = 32;

  Pointer<Int64>? _queryBodyBuffer; // _maxQueryHits handles
  Pointer<Float>? _queryDataBuffer; // _maxQueryHits * 5 floats
  Pointer<Int64>? _rayBodyOut; // single-hit out-params
  Pointer<Float>? _rayFloatOut; // [px, py, nx, ny, fraction]

  void _ensureQueryBuffers() {
    _queryBodyBuffer ??= calloc<Int64>(_maxQueryHits);
    _queryDataBuffer ??= calloc<Float>(_maxQueryHits * 5);
    _rayBodyOut ??= calloc<Int64>();
    _rayFloatOut ??= calloc<Float>(5);
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
  }

  /// Box2D's query filter cannot express "skip these specific bodies", only
  /// category/mask. So when [exclude] is non-empty we have to collect all hits
  /// and filter in Dart; otherwise the cheaper closest-hit call is enough.
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

    final count = box2d.b2w_castRayAll(
      _world!.handle,
      origin.dx,
      origin.dy,
      translation.dx,
      translation.dy,
      _allCategories,
      _allCategories,
      _queryBodyBuffer!,
      _queryDataBuffer!,
      _maxQueryHits,
    );

    final data = _queryDataBuffer!;
    final hits = <RayBodyHit>[];
    // Box2D reports one hit per shape, so a compound body appears once per
    // fixture — keep only its nearest hit.
    final seen = <PhysicsBody>{};
    for (var i = 0; i < count; i++) {
      final body = _handleToBody[_queryBodyBuffer![i]];
      if (body == null) continue;
      if (exclude != null && exclude.contains(body)) continue;
      if (!seen.add(body)) continue;
      final base = i * 5;
      hits.add(
        RayBodyHit(
          body: body,
          point: Offset(data[base], data[base + 1]),
          normal: Offset(data[base + 2], data[base + 3]),
          distance: data[base + 4] * ray.maxDistance,
        ),
      );
    }

    // Native order is BVH traversal order, not distance order.
    hits.sort((a, b) => a.distance.compareTo(b.distance));
    return hits;
  }

  @override
  List<PhysicsBody> queryAABB(Rect rect) {
    if (!_nativeReady) return super.queryAABB(rect);

    _ensureQueryBuffers();
    final count = box2d.b2w_queryAABB(
      _world!.handle,
      rect.left,
      rect.top,
      rect.right,
      rect.bottom,
      _allCategories,
      _allCategories,
      _queryBodyBuffer!,
      _maxQueryHits,
    );

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
    b2Body.destroy();
  }

  @override
  void renderDebug(Canvas canvas, Size size) {
    if (!debugRender) return;

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
      canvas.drawCircle(Offset.zero, shape.radius, paint);
      canvas.drawLine(Offset.zero, Offset(shape.radius, 0), paint);
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
          ...super.stats, // pure-Dart keys with zero values when native is running
          'bodyCount': _bodyMap.length,
          'awakeBodies': _bodyMap.length, // all native bodies are active
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
  // CONTRACT: call [pollContactBeginEvents] exactly once per frame, after
  // [update] and before the next [update] call. The native buffer is cleared
  // at the start of each step, so events not polled within the same frame are
  // permanently lost.
  //
  // In just_game_engine this is called from Box2DCollisionSystem (priority 88),
  // which runs after physics (priority 90) and before rendering (priority < 50).
  // Any caller must maintain the same relative ordering.

  /// Iterate all begin-touch contact events from the last step.
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
    final count = box2d.b2w_getContactBeginCount(_world!.handle);
    if (count == 0) return;

    final outA = calloc<Int64>();
    final outB = calloc<Int64>();
    final outNx = calloc<Float>();
    final outNy = calloc<Float>();
    try {
      for (int i = 0; i < count; i++) {
        box2d.b2w_getContactBeginEvent(
          _world!.handle,
          i,
          outA,
          outB,
          outNx,
          outNy,
        );
        final pA = _handleToBody[outA.value];
        final pB = _handleToBody[outB.value];
        if (pA != null && pB != null) {
          fn(pA, pB, outNx.value.toDouble(), outNy.value.toDouble());
        }
      }
    } finally {
      calloc
        ..free(outA)
        ..free(outB)
        ..free(outNx)
        ..free(outNy);
    }
  }

  /// Iterate all end-touch (separation) contact events from the last step.
  ///
  /// Unlike [pollContactBeginEvents] this has no contact normal — Box2D's
  /// end-touch event only reports which bodies stopped touching.
  @override
  void pollContactEndEvents(void Function(PhysicsBody a, PhysicsBody b) fn) {
    if (!_nativeReady) {
      super.pollContactEndEvents(fn);
      return;
    }
    final count = box2d.b2w_getContactEndCount(_world!.handle);
    if (count == 0) return;

    final outA = calloc<Int64>();
    final outB = calloc<Int64>();
    try {
      for (int i = 0; i < count; i++) {
        box2d.b2w_getContactEndEvent(_world!.handle, i, outA, outB);
        final pA = _handleToBody[outA.value];
        final pB = _handleToBody[outB.value];
        if (pA != null && pB != null) {
          fn(pA, pB);
        }
      }
    } finally {
      calloc
        ..free(outA)
        ..free(outB);
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
    final count = box2d.b2w_getBodyMoveEventCount(_world!.handle);
    if (count == 0) return;

    final outBody = calloc<Int64>();
    final outSleep = calloc<Int32>();
    try {
      for (int i = 0; i < count; i++) {
        box2d.b2w_getBodyMoveEvent(_world!.handle, i, outBody, outSleep);
        final body = _handleToBody[outBody.value];
        if (body != null) {
          fn(body, fellAsleep: outSleep.value != 0);
        }
      }
    } finally {
      calloc
        ..free(outBody)
        ..free(outSleep);
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
    return joint;
  }

  Box2DJoint? createMouseJoint(PhysicsBody bodyB, Offset target) {
    // Box2D v3.0 removed the native mouse joint; fall back to the pure-Dart
    // MouseJoint which provides equivalent spring-damper drag behaviour.
    // The returned joint is a JointConstraint, not a Box2DJoint — callers
    // that downcast to Box2DJoint must handle null.
    super.addMouseJoint(bodyB, Vector2(target.dx, target.dy));
    return null;
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
  JointConstraint addMouseJoint(PhysicsBody b, Vector2 target) {
    final native = createMouseJoint(b, Offset(target.x, target.y));
    return native ?? super.addMouseJoint(b, target);
  }

  @override
  JointConstraint addWheelJoint(PhysicsBody a, PhysicsBody b, Offset axis) {
    final anchor = Offset(
      (a.position.x + b.position.x) / 2,
      (a.position.y + b.position.y) / 2,
    );
    final native = createWheelJoint(a, b, anchor, axis);
    return native ?? super.addWheelJoint(a, b, axis);
  }

  /// Destroy a Box2D joint and remove it from the engine's joint list.
  void destroyJoint(Box2DJoint joint) {
    joint.destroy();
    removeJoint(joint);
  }

  // ── NativeCallable.listener — cross-thread audio impact callback ──────────

  /// Register a Dart callback to fire (on the main isolate) whenever a
  /// collision's approach speed exceeds [speedThreshold].
  ///
  /// [NativeCallable.listener] is used so the native Box2D worker threads can
  /// safely post to the Dart event loop without blocking.
  ///
  /// [callback] receives packed body handles (int) and the impact speed.
  /// Use [physicsBodyFromHandle] to resolve handles to [PhysicsBody] objects.
  void registerImpactCallback(
    void Function(int bodyA, int bodyB, double speed) callback, {
    double speedThreshold = 100.0,
  }) {
    if (!_nativeReady) return;
    _impactCallable?.close();
    _impactCallable = NativeCallable<ImpactCallbackFnFunction>.listener(
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
  double get alpha => _loop?.alpha ?? 1.0;

  // ── Dispose ───────────────────────────────────────────────────────────────

  @override
  void dispose() {
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
    // Zero density for static and kinematic bodies: Box2D would otherwise
    // derive a positive mass from shape area, and a positive inverse mass is
    // what makes it apply gravity (see the note in addBody).
    final density = body.effectiveBodyType == BodyType.dynamic ? 1.0 : 0.0;
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
      box2d.b2w_addSegmentShape(
        handle,
        shape.point1.dx,
        shape.point1.dy,
        shape.point2.dx,
        shape.point2.dy,
        density,
        friction,
        restitution,
      );
    } else if (shape is ChainShape && shape.vertices.length >= 2) {
      final pts = Float32List(shape.vertices.length * 2);
      for (int i = 0; i < shape.vertices.length; i++) {
        pts[i * 2] = shape.vertices[i].dx;
        pts[i * 2 + 1] = shape.vertices[i].dy;
      }
      final ptr = calloc<Float>(pts.length);
      try {
        ptr.asTypedList(pts.length).setAll(0, pts);
        box2d.b2w_addChainShape(
          handle,
          ptr,
          shape.vertices.length,
          shape.loop ? 1 : 0,
          friction,
          restitution,
        );
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

  void _ensureBufferCapacity(int needed) {
    if (needed <= _bufferCapacity) return;
    // Grow by 1.5× to amortise reallocations.
    final cap = (needed * 1.5).ceil().clamp(8, 1 << 20);
    final oldH = _handleBuffer;
    final oldT = _transformBuffer;
    if (oldH != null) calloc.free(oldH);
    if (oldT != null) calloc.free(oldT);
    _handleBuffer = calloc<Int64>(cap);
    _transformBuffer = calloc<Float>(cap * 6);
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

    // One FFI call writes x,y,angle,vx,vy,pad for every body.
    box2d.b2w_bulkExtractTransforms(_handleBuffer!, _transformBuffer!, n);

    // Zero-copy view — reads directly from native memory.
    final Float32List trs = _transformBuffer!.asTypedList(n * 6);

    // Scatter results back to Dart body objects.
    i = 0;
    for (final entry in _bodyMap.entries) {
      final dartBody = entry.key; // PhysicsBody (ECS holds a reference to this)
      final b2Body = entry.value; // Box2DBody (FFI handle + prev/current state)
      final base = i * 6;

      b2Body.currentX = trs[base];
      b2Body.currentY = trs[base + 1];
      b2Body.currentAngle = trs[base + 2];
      b2Body.velocityX = trs[base + 3];
      b2Body.velocityY = trs[base + 4];

      // Write through to PhysicsBody so existing ECS bridge works unchanged.
      dartBody.position.x = b2Body.currentX;
      dartBody.position.y = b2Body.currentY;
      dartBody.angle = b2Body.currentAngle;
      dartBody.velocity.x = b2Body.velocityX;
      dartBody.velocity.y = b2Body.velocityY;
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
