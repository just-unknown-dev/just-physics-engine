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
  PhysicsEngine.pureDart();

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
  int _lastAwakeBodyCount = 0;
  int _lastBroadphaseDirtyBodyCount = 0;
  int _lastTrackedCellCount = 0;
  double _lastStepMs = 0.0;

  // Persistent Stopwatch instance — reused every frame to avoid heap allocation.
  final Stopwatch _stepStopwatch = Stopwatch();

  /// Update physics simulation
  void update(double deltaTime) {
    _stepStopwatch
      ..reset()
      ..start();
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

    _stepStopwatch.stop();
    _lastAwakeBodyCount = awakeBodyCount;
    _lastStepMs = _stepStopwatch.elapsedMicroseconds / 1000.0;
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

    // Reset sensor buffers for this step.
    _sensorBeginBuffer.clear();
    _sensorEndBuffer.clear();
    final previousSensorPairs = Set<BodyPair>.from(_activeSensorPairs);
    final currentSensorPairs = <BodyPair>{};

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
        _lastResolvedCollisionCount++;
        _resolveCollision(bodyA, bodyB, best);
      }
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
    final inverseMassSum = a.inverseMass + b.inverseMass;

    if (inverseMassSum == 0) return; // both immovable

    const correctionPercent = 0.8;
    const slop = 0.05;
    final correctionMag =
        math.max(penetration - slop, 0.0) / inverseMassSum * correctionPercent;
    a.position.x -= normal.dx * correctionMag * a.inverseMass;
    a.position.y -= normal.dy * correctionMag * a.inverseMass;
    b.position.x += normal.dx * correctionMag * b.inverseMass;
    b.position.y += normal.dy * correctionMag * b.inverseMass;

    // ── Impulse resolution ────────────────────────────────────────────────
    final rvx = b.velocity.x - a.velocity.x;
    final rvy = b.velocity.y - a.velocity.y;
    final velAlongNormal = rvx * normal.dx + rvy * normal.dy;

    if (velAlongNormal > 0) return; // separating

    final restitution = math.min(a.restitution, b.restitution);
    final j = -(1.0 + restitution) * velAlongNormal / inverseMassSum;

    final jnx = normal.dx * j;
    final jny = normal.dy * j;
    a.velocity.x -= jnx * a.inverseMass;
    a.velocity.y -= jny * a.inverseMass;
    b.velocity.x += jnx * b.inverseMass;
    b.velocity.y += jny * b.inverseMass;

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

      a.velocity.x -= tx * fScalar * a.inverseMass;
      a.velocity.y -= ty * fScalar * a.inverseMass;
      b.velocity.x += tx * fScalar * b.inverseMass;
      b.velocity.y += ty * fScalar * b.inverseMass;
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
