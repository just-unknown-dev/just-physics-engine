import 'dart:ffi'
    hide Size; // hide dart:ffi's Size to avoid conflict with dart:ui Size
import 'dart:typed_data';
import 'dart:ui'
    show Canvas, Color, Offset, Paint, PaintingStyle, Path, Rect, Size;

import 'package:flutter/foundation.dart' show debugPrint;

import 'package:ffi/ffi.dart';

// PhysicsBody, CollisionShape, CircleShape, RectangleShape, PolygonShape are
// all part-files of physics_engine.dart — import the library, not the parts.
import '../physics_2d/physics_engine.dart';
import 'box2d_body.dart';
import 'box2d_world.dart';
import 'ffi/box2d_bindings.dart' show ImpactCallbackFnFunction;
import 'ffi/box2d_library.dart';
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

  // ── Zero-copy transform buffers (calloc-allocated, persist for world life) ──

  Pointer<Int64>? _handleBuffer; // one int64 per tracked body
  Pointer<Float>? _transformBuffer; // 6 floats per body: x,y,angle,vx,vy,pad
  int _bufferCapacity = 0;

  // ── NativeCallable for cross-thread impact audio callback ──────────────────

  NativeCallable<ImpactCallbackFnFunction>? _impactCallable;

  // ── Diagnostics ────────────────────────────────────────────────────────────

  int _lastStepCount = 0;
  double _lastStepMs = 0.0;
  int _lastContactCount = 0;

  /// Number of Box2D sub-steps per fixed tick (4 recommended).
  final int subSteps;

  /// Number of native worker threads (0 → hardware_concurrency − 1).
  final int numThreads;

  Box2DPhysicsEngine({
    double gravityX = 0.0,
    double gravityY = 981.0, // 9.81 m/s² at 1 unit = 1 cm
    this.subSteps = 4,
    this.numThreads = 0,
  }) {
    // Set the inherited gravity vector from PhysicsEngine.
    gravity.x = gravityX;
    gravity.y = gravityY;
  }

  // ── Public API — mirrors PhysicsEngine ────────────────────────────────────

  @override
  void initialize() {
    try {
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
    final sw = Stopwatch()..start();

    for (final b2Body in _bodyMap.values) {
      b2Body.capturePrevious();
    }

    _lastStepCount = _loop!.advance(deltaTime);
    _syncTransformsFromNative();
    _lastContactCount = box2d.b2w_getContactBeginCount(_world!.handle);

    sw.stop();
    _lastStepMs = sw.elapsedMicroseconds / 1000.0;
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

    final isStatic = body.mass <= 0.0;
    final Box2DBody b2Body;

    if (isStatic) {
      b2Body = Box2DBody.static(
        worldHandle: _world!.handle,
        posX: body.position.x,
        posY: body.position.y,
        angle: body.angle,
      );
    } else {
      b2Body = Box2DBody.dynamic(
        worldHandle: _world!.handle,
        posX: body.position.x,
        posY: body.position.y,
        angle: body.angle,
      );
    }

    _addShapeFixture(b2Body, body);

    _bodyMap[body] = b2Body;
    _handleToBody[b2Body.handle] = body;
    _ensureBufferCapacity(_bodyMap.length);
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

    for (final entry in _bodyMap.entries) {
      final body = entry.key;
      final b2 = entry.value;
      final paint = body.mass <= 0.0 ? staticPaint : strokePaint;
      final shape = body.shape;

      canvas.save();
      canvas.translate(b2.currentX, b2.currentY);
      canvas.rotate(b2.currentAngle);

      if (shape is CircleShape) {
        canvas.drawCircle(Offset.zero, shape.radius, paint);
        // Draw orientation line so rotation is visible.
        canvas.drawLine(Offset.zero, Offset(shape.radius, 0), paint);
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

      canvas.restore();
    }
  }

  @override
  Map<String, dynamic> get stats => {
    'bodyCount': _nativeReady ? _bodyMap.length : super.bodies.length,
    'lastStepCount': _lastStepCount,
    'lastStepMs': _lastStepMs,
    'contactCount': _lastContactCount,
    'alpha': _world?.alpha ?? 0.0,
    'backend': _nativeReady ? 'box2d_v3' : 'dart_fallback',
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
  // Call this from just_game_engine's Box2DCollisionSystem (priority 88) to
  // translate contact events into ECS CollisionEvents on the event bus.
  // This keeps just_physics_engine free of ECS dependencies.

  /// Iterate all begin-touch contact events from the last step.
  ///
  /// [fn] receives the two colliding [PhysicsBody] objects and the contact
  /// normal (nx, ny). Calls [fn] only for pairs where both bodies are tracked.
  void pollContactBeginEvents(
    void Function(PhysicsBody a, PhysicsBody b, double nx, double ny) fn,
  ) {
    if (!_nativeReady) return;
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
  double get alpha => _loop?.alpha ?? 1.0;

  // ── Dispose ───────────────────────────────────────────────────────────────

  @override
  void dispose() {
    _impactCallable?.close();
    _impactCallable = null;

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
    _bufferCapacity = 0; // MUST reset — dispose() nulls the buffers; without
    // this, _ensureBufferCapacity() sees the old non-zero capacity on the next
    // initialize() cycle and skips re-allocation, leaving _handleBuffer null.
    // _syncTransformsFromNative() then throws a null assertion on every frame,
    // Box2D steps but positions are never written back to Dart, bodies freeze
    // at their spawn positions and act as invisible ghost colliders.

    _world?.dispose();
    _nativeReady = false;
    // Always clear the base-class _bodies list too. If the native init ever
    // fails and addBody fell through to super.addBody(), those bodies live in
    // _bodies and will cause ghost collisions unless we clear them here.
    super.dispose();
  }

  // ── Private helpers ───────────────────────────────────────────────────────

  void _addShapeFixture(Box2DBody b2Body, PhysicsBody body) {
    final shape = body.shape;
    final density = (body.mass > 0) ? 1.0 : 0.0;
    final friction = body.friction;
    final restitution = body.restitution;

    if (shape is CircleShape) {
      box2d.b2w_addCircleShape(
        b2Body.handle,
        shape.radius,
        density,
        friction,
        restitution,
      );
    } else if (shape is RectangleShape) {
      box2d.b2w_addBoxShape(
        b2Body.handle,
        shape.width / 2,
        shape.height / 2,
        density,
        friction,
        restitution,
      );
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
          b2Body.handle,
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

      i++;
    }
  }
}
