import 'dart:math' as math;
import 'dart:typed_data';

import 'package:just_dart/just_dart.dart';

import '../physics_3d/physics_engine_3d.dart';
import '../physics_3d/results_3d.dart';
import '../physics_3d/settings_3d.dart';
import '../physics_3d/shapes_3d.dart';

/// A [PhysicsEngine3D] for tests, in plain Dart.
///
/// Every shape is its axis-aligned bounding box and nothing turns: dynamic
/// bodies fall, land on and slide off other bodies, sensors report what
/// enters them, rays and sweeps hit boxes, and a character walks on boxes,
/// climbs steps, is stopped by walls and ceilings, and rides what it
/// stands on. Joints are recorded, not simulated. Deterministic: the same
/// calls give the same results.
///
/// Pass `unavailableReason` to stand in for an engine that cannot run.
final class FakePhysicsEngine3D implements PhysicsEngine3D {
  /// A fake engine; unavailable, giving [unavailableReason], when that is
  /// not null.
  FakePhysicsEngine3D({this.unavailableReason, this.collisions = true});

  @override
  String get name => 'fake';

  @override
  final String? unavailableReason;

  /// Whether worlds it makes resolve collisions (off: bodies only move by
  /// velocity and gravity, for timing the ECS side).
  final bool collisions;

  /// The worlds it made that are not disposed.
  final List<FakePhysicsWorld3D> worlds = [];

  /// How often [destroyAll] was called.
  int destroyAllCalls = 0;

  @override
  PhysicsWorld3D createWorld(PhysicsWorldSettings3D settings) {
    final reason = unavailableReason;
    if (reason != null) throw Physics3DUnsupportedError(reason);
    final world = FakePhysicsWorld3D(
      settings,
      collisions: collisions,
      owner: this,
    );
    worlds.add(world);
    return world;
  }

  @override
  void destroyAll() {
    destroyAllCalls++;
    for (final world in List.of(worlds)) {
      world.dispose();
    }
  }
}

/// A body in a [FakePhysicsWorld3D], open for tests to look at.
final class FakeBody3D {
  FakeBody3D._(this.handle, this.shape, BodySettings3D s)
    : type = s.type,
      layer = s.layer,
      isSensor = s.isSensor,
      allowSleeping = s.allowSleeping,
      motionQuality = s.motionQuality,
      friction = s.friction,
      restitution = s.restitution,
      gravityFactor = s.gravityFactor,
      mass = s.mass > 0 ? s.mass : 1.0,
      linearDamping = s.linearDamping,
      position = Vector3.copy(s.position),
      rotation = Quaternion.copy(s.rotation),
      linearVelocity = Vector3.copy(s.linearVelocity),
      angularVelocity = Vector3.copy(s.angularVelocity),
      awake = s.type != BodyType3D.static {
    final (min, max) = _boundsOf(shape);
    localMin.setFrom(min);
    localMax.setFrom(max);
  }

  /// Its handle.
  final int handle;

  /// Its shape.
  final Shape3D shape;

  /// How it moves.
  BodyType3D type;

  /// Its layer.
  int layer;

  /// Whether it is a sensor.
  final bool isSensor;

  /// Whether it may sleep.
  final bool allowSleeping;

  /// Its motion quality.
  MotionQuality3D motionQuality;

  /// Its friction, bounciness and gravity factor.
  double friction, restitution, gravityFactor;

  /// Its mass (1 when the settings left it to the shape).
  final double mass;

  /// Its linear damping.
  final double linearDamping;

  /// Where it is.
  final Vector3 position;

  /// How it is turned (never changed by the simulation).
  final Quaternion rotation;

  /// Its velocity.
  final Vector3 linearVelocity;

  /// Its spin (never changed by the simulation).
  final Vector3 angularVelocity;

  /// Force waiting for the next step.
  final Vector3 force = Vector3.zero();

  /// Torque waiting for the next step (recorded only).
  final Vector3 torque = Vector3.zero();

  /// Whether it is awake.
  bool awake;

  /// The character it stands in for, or 0.
  int character = 0;

  /// Its shape's bounds in its own space.
  final Vector3 localMin = Vector3.zero(), localMax = Vector3.zero();

  int _restSteps = 0;
  Vector3? _kinematicTarget;

  double get minX => position.x + localMin.x;
  double get minY => position.y + localMin.y;
  double get minZ => position.z + localMin.z;
  double get maxX => position.x + localMax.x;
  double get maxY => position.y + localMax.y;
  double get maxZ => position.z + localMax.z;

  bool get _isSolid => !isSensor && character == 0;
}

/// A joint in a [FakePhysicsWorld3D]: recorded, never simulated.
final class FakeJoint3D {
  FakeJoint3D._(this.handle, this.bodyA, this.bodyB, this.settings)
    : motorState = settings.motorState,
      motorTarget = settings.motorTarget;

  /// Its handle.
  final int handle;

  /// The bodies; [bodyB] is 0 for the world.
  final int bodyA, bodyB;

  /// What it was made with.
  final JointSettings3D settings;

  /// Whether it is on.
  bool enabled = true;

  /// Its motor.
  MotorState3D motorState;

  /// Its motor's target.
  double motorTarget;
}

/// A character in a [FakePhysicsWorld3D].
final class FakeCharacter3D {
  FakeCharacter3D._(this.handle, this.settings, Vector3 position)
    : position = Vector3.copy(position);

  /// Its handle.
  final int handle;

  /// What it was made with.
  final CharacterSettings3D settings;

  /// Where its feet are.
  final Vector3 position;

  /// Its velocity after the last move.
  final Vector3 velocity = Vector3.zero();

  /// What it stood on after the last move.
  GroundState3D groundState = GroundState3D.inAir;

  /// The body it stood on, or 0.
  int groundBody = 0;

  /// Its stand-in body, or 0.
  int innerBody = 0;

  /// How often it was moved.
  int moves = 0;

  double get _halfHeight => settings.halfHeight + settings.radius;
}

/// The world of a [FakePhysicsEngine3D]. Its bodies, joints and
/// characters are open for tests to look at.
final class FakePhysicsWorld3D implements PhysicsWorld3D {
  /// A world made as [settings] says.
  FakePhysicsWorld3D(
    this.settings, {
    this.collisions = true,
    FakePhysicsEngine3D? owner,
  }) : _owner = owner,
       _gravity = Vector3.copy(settings.gravity);

  /// What it was made with.
  final PhysicsWorldSettings3D settings;

  /// Whether it resolves collisions.
  final bool collisions;

  final FakePhysicsEngine3D? _owner;
  final Vector3 _gravity;
  static int _next = 1;

  /// Its bodies, by handle.
  final Map<int, FakeBody3D> bodies = {};

  /// Its joints, by handle.
  final Map<int, FakeJoint3D> joints = {};

  /// Its characters, by handle.
  final Map<int, FakeCharacter3D> characters = {};

  /// How many steps it took, and how long the last was.
  int steps = 0;

  /// See [steps].
  double lastStep = 0;

  final List<PhysicsEvent3D> _events = [];
  final List<int> _fellAsleep = [];
  final Map<(int, int), bool> _touching = {}; // pair → is a sensor pair
  bool _disposed = false;

  @override
  bool get isDisposed => _disposed;

  @override
  void dispose() {
    if (_disposed) return;
    _disposed = true;
    bodies.clear();
    joints.clear();
    characters.clear();
    _owner?.worlds.remove(this);
  }

  @override
  Vector3 get gravity => Vector3.copy(_gravity);

  @override
  set gravity(Vector3 value) => _gravity.setFrom(value);

  @override
  int get bodyCount => bodies.length;

  bool _collide(int a, int b) =>
      (settings.layerMasks[a] >> b) & 1 == 1 &&
      (settings.layerMasks[b] >> a) & 1 == 1;

  // ── Bodies ────────────────────────────────────────────────────────────

  @override
  int createBody(Shape3D shape, BodySettings3D settings) {
    if (_disposed || settings.layer < 0 || settings.layer > 15) return 0;
    final mustBeStatic = _mustBeStatic(shape);
    if (mustBeStatic && settings.type == BodyType3D.dynamic) return 0;
    final body = FakeBody3D._(_next++, shape, settings);
    bodies[body.handle] = body;
    return body.handle;
  }

  static bool _mustBeStatic(Shape3D shape) => switch (shape) {
    HeightFieldShape3D() => true,
    TransformedShape3D(:final shape) => _mustBeStatic(shape),
    CompoundShape3D(:final parts) => parts.any(_mustBeStatic),
    _ => false,
  };

  @override
  void destroyBody(int body) {
    if (bodies.remove(body) == null) return;
    joints.removeWhere((_, j) => j.bodyA == body || j.bodyB == body);
  }

  @override
  bool isBody(int body) => bodies.containsKey(body);

  @override
  void setBodyPose(
    int body,
    Vector3 position,
    Quaternion rotation, {
    bool wake = true,
  }) {
    final b = bodies[body];
    if (b == null) return;
    b.position.setFrom(position);
    b.rotation.setFrom(rotation);
    if (wake) _wake(b);
  }

  @override
  void moveKinematic(
    int body,
    Vector3 position,
    Quaternion rotation,
    double dt,
  ) {
    final b = bodies[body];
    if (b == null || b.type != BodyType3D.kinematic || dt <= 0) return;
    b._kinematicTarget = Vector3.copy(position);
    b.linearVelocity
      ..setFrom(position)
      ..sub(b.position)
      ..scale(1 / dt);
    b.rotation.setFrom(rotation);
    _wake(b);
  }

  void _wake(FakeBody3D b) {
    if (b.type == BodyType3D.static) return;
    b.awake = true;
    b._restSteps = 0;
  }

  @override
  void setLinearVelocity(int body, Vector3 velocity) {
    final b = bodies[body];
    if (b == null || b.type == BodyType3D.static) return;
    b.linearVelocity.setFrom(velocity);
    _wake(b);
  }

  @override
  void setAngularVelocity(int body, Vector3 velocity) {
    final b = bodies[body];
    if (b == null || b.type == BodyType3D.static) return;
    b.angularVelocity.setFrom(velocity);
    _wake(b);
  }

  @override
  void addForce(int body, Vector3 force) {
    final b = bodies[body];
    if (b == null || b.type != BodyType3D.dynamic) return;
    b.force.add(force);
    _wake(b);
  }

  @override
  void addTorque(int body, Vector3 torque) {
    final b = bodies[body];
    if (b == null || b.type != BodyType3D.dynamic) return;
    b.torque.add(torque);
    _wake(b);
  }

  @override
  void addImpulse(int body, Vector3 impulse) {
    final b = bodies[body];
    if (b == null || b.type != BodyType3D.dynamic) return;
    b.linearVelocity.addScaled(impulse, 1 / b.mass);
    _wake(b);
  }

  @override
  void addAngularImpulse(int body, Vector3 impulse) {
    final b = bodies[body];
    if (b == null || b.type != BodyType3D.dynamic) return;
    b.angularVelocity.addScaled(impulse, 1 / b.mass);
    _wake(b);
  }

  @override
  void setFriction(int body, double friction) =>
      bodies[body]?.friction = friction;

  @override
  void setRestitution(int body, double restitution) =>
      bodies[body]?.restitution = restitution;

  @override
  void setGravityFactor(int body, double factor) =>
      bodies[body]?.gravityFactor = factor;

  @override
  void setLayer(int body, int layer) => bodies[body]?.layer = layer;

  @override
  void setMotionQuality(int body, MotionQuality3D quality) =>
      bodies[body]?.motionQuality = quality;

  @override
  void wake(int body) {
    final b = bodies[body];
    if (b != null) _wake(b);
  }

  @override
  bool isAwake(int body) => bodies[body]?.awake ?? false;

  // ── Joints ────────────────────────────────────────────────────────────

  @override
  int createJoint(int bodyA, int bodyB, JointSettings3D settings) {
    if (!bodies.containsKey(bodyA) ||
        (bodyB != 0 && !bodies.containsKey(bodyB))) {
      return 0;
    }
    if (bodyA == bodyB) return 0;
    final joint = FakeJoint3D._(_next++, bodyA, bodyB, settings);
    joints[joint.handle] = joint;
    return joint.handle;
  }

  @override
  void destroyJoint(int joint) => joints.remove(joint);

  @override
  bool isJoint(int joint) => joints.containsKey(joint);

  @override
  void setJointEnabled(int joint, bool enabled) =>
      joints[joint]?.enabled = enabled;

  @override
  void setJointMotor(int joint, MotorState3D state, double target) {
    final j = joints[joint];
    if (j == null) return;
    j
      ..motorState = state
      ..motorTarget = target;
  }

  @override
  double jointValue(int joint) {
    final j = joints[joint];
    return j != null && j.motorState == MotorState3D.position
        ? j.motorTarget
        : 0;
  }

  // ── Characters ────────────────────────────────────────────────────────

  @override
  int createCharacter(CharacterSettings3D settings, Vector3 position) {
    if (_disposed || settings.layer < 0 || settings.layer > 15) return 0;
    final c = FakeCharacter3D._(_next++, settings, position);
    characters[c.handle] = c;
    if (settings.innerBody) {
      final h = c._halfHeight;
      final inner = createBody(
        BoxShape3D(settings.radius, h, settings.radius),
        BodySettings3D(
          position: Vector3(position.x, position.y - h, position.z),
          type: BodyType3D.kinematic,
          layer: settings.layer,
          allowSleeping: false,
        ),
      );
      bodies[inner]!.character = c.handle;
      c.innerBody = inner;
    }
    return c.handle;
  }

  @override
  void destroyCharacter(int character) {
    final c = characters.remove(character);
    if (c != null && c.innerBody != 0) destroyBody(c.innerBody);
  }

  @override
  bool isCharacter(int character) => characters.containsKey(character);

  @override
  int characterBody(int character) => characters[character]?.innerBody ?? 0;

  @override
  void moveCharacter(
    int character,
    double dt,
    Vector3 velocity,
    CharacterState3D out, {
    required Vector3 stepDown,
    required Vector3 stepUp,
  }) {
    final c = characters[character];
    if (c == null) {
      out
        ..groundState = GroundState3D.inAir
        ..groundBody = 0;
      return;
    }
    c.moves++;
    final r = c.settings.radius, h = c._halfHeight;
    final p = c.position;
    final start = Vector3.copy(p);
    final wasGrounded = c.groundState == GroundState3D.onGround;
    final solids = [
      for (final b in bodies.values)
        if (b._isSolid && _collide(b.layer, c.settings.layer)) b,
    ];
    // The capsule as a box: x and z ± r, y from the feet up 2h.
    bool overlaps(FakeBody3D b) =>
        p.x + r > b.minX &&
        p.x - r < b.maxX &&
        p.z + r > b.minZ &&
        p.z - r < b.maxZ &&
        p.y > b.minY + 1e-6 &&
        p.y - 2 * h < b.maxY - 1e-6;

    final climb = stepUp.length;
    void moveAxis(int axis, double d) {
      if (d == 0) return;
      final before = Vector3.copy(p);
      if (axis == 0) {
        p.x += d;
      } else {
        p.z += d;
      }
      for (final b in solids) {
        if (!overlaps(b)) continue;
        // A step low enough: up onto it.
        final rise = p.y - b.minY;
        if (rise > 0 && rise <= climb + 1e-6) {
          final feet = p.y;
          p.y = b.minY;
          if (!solids.any(overlaps)) continue;
          p.y = feet;
        }
        p.setFrom(before);
        if (axis == 0) {
          p.x = d > 0
              ? math.min(p.x + d, b.minX - r)
              : math.max(p.x + d, b.maxX + r);
        } else {
          p.z = d > 0
              ? math.min(p.z + d, b.minZ - r)
              : math.max(p.z + d, b.maxZ + r);
        }
      }
    }

    moveAxis(0, velocity.x * dt);
    moveAxis(2, velocity.z * dt);
    var vy = velocity.y;
    p.y += vy * dt;
    for (final b in solids) {
      if (!overlaps(b)) continue;
      if (vy >= 0) {
        p.y = b.minY; // Landed (+Y is down).
      } else {
        p.y = b.maxY + 2 * h; // Head on a ceiling.
      }
      vy = 0;
    }

    // Ground: something just under the feet; within stepDown when it was
    // on the ground already, so it keeps to stairs going down.
    final reach = wasGrounded ? math.max(stepDown.length, 0.02) : 0.02;
    FakeBody3D? ground;
    for (final b in solids) {
      final below = b.minY - p.y;
      if (below < -1e-6 || below > reach) continue;
      if (p.x + r <= b.minX ||
          p.x - r >= b.maxX ||
          p.z + r <= b.minZ ||
          p.z - r >= b.maxZ) {
        continue;
      }
      if (ground == null || b.minY < ground.minY) ground = b;
    }
    if (ground != null && vy >= 0) {
      p.y = ground.minY;
      c.groundState = GroundState3D.onGround;
      c.groundBody = ground.handle;
    } else {
      c.groundState = GroundState3D.inAir;
      c.groundBody = 0;
    }
    c.velocity
      ..setFrom(p)
      ..sub(start)
      ..scale(dt > 0 ? 1 / dt : 0);
    if (c.groundState == GroundState3D.onGround) c.velocity.y = 0;
    if (c.innerBody != 0) {
      bodies[c.innerBody]?.position.setValues(p.x, p.y - h, p.z);
    }
    characterState(character, out);
  }

  @override
  void characterState(int character, CharacterState3D out) {
    final c = characters[character];
    if (c == null) {
      out
        ..groundState = GroundState3D.inAir
        ..groundBody = 0;
      return;
    }
    out.position.setFrom(c.position);
    out.velocity.setFrom(c.velocity);
    out.groundState = c.groundState;
    out.groundBody = c.groundBody;
    final ground = bodies[c.groundBody];
    if (ground != null) {
      out.groundNormal.setValues(0, -1, 0);
      out.groundVelocity.setFrom(ground.linearVelocity);
      out.groundPosition.setValues(c.position.x, ground.minY, c.position.z);
    } else {
      out.groundNormal.setZero();
      out.groundVelocity.setZero();
      out.groundPosition.setZero();
    }
    out.contactCount = ground == null ? 0 : 1;
  }

  @override
  void setCharacterPosition(int character, Vector3 position) {
    final c = characters[character];
    if (c == null) return;
    c.position.setFrom(position);
    c.groundState = GroundState3D.inAir;
    if (c.innerBody != 0) {
      bodies[c.innerBody]?.position.setValues(
        position.x,
        position.y - c._halfHeight,
        position.z,
      );
    }
  }

  @override
  List<CharacterContact3D> characterContacts(int character) {
    final c = characters[character];
    final ground = c == null ? null : bodies[c.groundBody];
    if (c == null || ground == null) return const [];
    return [
      CharacterContact3D(
        body: ground.handle,
        character: 0,
        position: Vector3(c.position.x, ground.minY, c.position.z),
        normal: Vector3(0, -1, 0),
        velocity: Vector3.copy(ground.linearVelocity),
        distance: 0,
        isSensor: false,
        hadCollision: true,
      ),
    ];
  }

  // ── Stepping ──────────────────────────────────────────────────────────

  @override
  void step(double dt, {int collisionSteps = 1}) {
    if (_disposed) return;
    steps++;
    lastStep = dt;
    final all = bodies.values.toList(growable: false);
    for (final b in all) {
      if (!b.awake) continue;
      if (b.type == BodyType3D.kinematic) {
        final target = b._kinematicTarget;
        if (target != null) {
          b.position.setFrom(target);
          b._kinematicTarget = null;
        } else {
          b.position.addScaled(b.linearVelocity, dt);
        }
        continue;
      }
      if (b.type != BodyType3D.dynamic) continue;
      final v = b.linearVelocity;
      v.addScaled(_gravity, b.gravityFactor * dt);
      v.addScaled(b.force, dt / b.mass);
      b.force.setZero();
      b.torque.setZero();
      v.scale(math.max(0.0, 1 - b.linearDamping * dt));
      b.position.addScaled(v, dt);
      if (collisions) _resolve(b, all);
      if (b.allowSleeping && v.lengthSquared < 0.05 * 0.05) {
        if (++b._restSteps > 30) {
          b.awake = false;
          v.setZero();
          _fellAsleep.add(b.handle);
        }
      } else {
        b._restSteps = 0;
      }
    }
    if (collisions) _findContacts(all);
  }

  /// Pushes [b] out of every solid it overlaps, along the shallowest axis.
  void _resolve(FakeBody3D b, List<FakeBody3D> all) {
    if (b.isSensor) return;
    for (final o in all) {
      if (identical(o, b) || !o._isSolid || !_collide(b.layer, o.layer)) {
        continue;
      }
      final dx1 = o.maxX - b.minX, dx2 = b.maxX - o.minX;
      final dy1 = o.maxY - b.minY, dy2 = b.maxY - o.minY;
      final dz1 = o.maxZ - b.minZ, dz2 = b.maxZ - o.minZ;
      if (dx1 <= 0 ||
          dx2 <= 0 ||
          dy1 <= 0 ||
          dy2 <= 0 ||
          dz1 <= 0 ||
          dz2 <= 0) {
        continue;
      }
      final px = math.min(dx1, dx2),
          py = math.min(dy1, dy2),
          pz = math.min(dz1, dz2);
      final v = b.linearVelocity;
      // The less bouncy of the two, as JoltPhysicsEngine and the 2-D engines mix it.
      final bounce = math.min(b.restitution, o.restitution);
      if (py <= px && py <= pz) {
        final up = dy2 < dy1; // Push towards −Y.
        b.position.y += up ? -dy2 : dy1;
        if ((up && v.y > 0) || (!up && v.y < 0)) v.y = -v.y * bounce;
        if (up) {
          // Resting on it: friction slows the slide.
          final keep = math.max(0.0, 1 - b.friction * o.friction * 0.5);
          v.x *= keep;
          v.z *= keep;
        }
      } else if (px <= pz) {
        b.position.x += dx2 < dx1 ? -dx2 : dx1;
        v.x = -v.x * bounce;
      } else {
        b.position.z += dz2 < dz1 ? -dz2 : dz1;
        v.z = -v.z * bounce;
      }
    }
  }

  void _findContacts(List<FakeBody3D> all) {
    final now = <(int, int), (bool, FakeBody3D, FakeBody3D)>{};
    const slop = 0.01;
    for (var i = 0; i < all.length; i++) {
      final a = all[i];
      for (var j = i + 1; j < all.length; j++) {
        final b = all[j];
        if (a.type == BodyType3D.static && b.type == BodyType3D.static) {
          continue;
        }
        if (a.isSensor && b.isSensor) continue;
        final sensor = a.isSensor || b.isSensor;
        // Characters only meet sensors.
        if ((a.character != 0 || b.character != 0) && !sensor) continue;
        if (!_collide(a.layer, b.layer)) continue;
        if (a.minX > b.maxX + slop || b.minX > a.maxX + slop) continue;
        if (a.minY > b.maxY + slop || b.minY > a.maxY + slop) continue;
        if (a.minZ > b.maxZ + slop || b.minZ > a.maxZ + slop) continue;
        now[(a.handle, b.handle)] = (sensor, a, b);
      }
    }
    for (final MapEntry(key: pair, value: (sensor, a, b)) in now.entries) {
      final known = _touching.containsKey(pair);
      if (known && !(settings.reportContactStay && !sensor)) continue;
      final point = Vector3(
        (math.max(a.minX, b.minX) + math.min(a.maxX, b.maxX)) / 2,
        (math.max(a.minY, b.minY) + math.min(a.maxY, b.maxY)) / 2,
        (math.max(a.minZ, b.minZ) + math.min(a.maxZ, b.maxZ)) / 2,
      );
      final normal = Vector3(
        b.position.x - a.position.x,
        b.position.y - a.position.y,
        b.position.z - a.position.z,
      );
      if (normal.normalize() == 0) normal.setValues(0, 1, 0);
      final type = sensor
          ? PhysicsEventType3D.sensorBegin
          : known
          ? PhysicsEventType3D.contactStay
          : PhysicsEventType3D.contactBegin;
      _events.add(
        PhysicsEvent3D(
          type,
          a.handle,
          b.handle,
          point: point,
          normal: sensor ? null : normal,
        ),
      );
    }
    for (final MapEntry(key: pair, value: sensor) in _touching.entries) {
      if (now.containsKey(pair)) continue;
      _events.add(
        PhysicsEvent3D(
          sensor ? PhysicsEventType3D.sensorEnd : PhysicsEventType3D.contactEnd,
          pair.$1,
          pair.$2,
        ),
      );
    }
    _touching
      ..clear()
      ..addEntries(now.entries.map((e) => MapEntry(e.key, e.value.$1)));
  }

  @override
  void readMovedBodies(BodyPoses3D out) {
    final moved = [
      for (final b in bodies.values)
        if (b.awake && b.type != BodyType3D.static && b.character == 0)
          b.handle,
      ..._fellAsleep.where(bodies.containsKey),
    ];
    _fellAsleep.clear();
    readBodies(moved, out);
  }

  @override
  void readBodies(List<int> handles, BodyPoses3D out) {
    final data = Float32List(handles.length * BodyPoses3D.stride);
    for (var i = 0; i < handles.length; i++) {
      final b = bodies[handles[i]];
      if (b == null) continue;
      final o = i * BodyPoses3D.stride;
      data
        ..[o] = b.position.x
        ..[o + 1] = b.position.y
        ..[o + 2] = b.position.z
        ..[o + 3] = b.rotation.x
        ..[o + 4] = b.rotation.y
        ..[o + 5] = b.rotation.z
        ..[o + 6] = b.rotation.w
        ..[o + 7] = b.linearVelocity.x
        ..[o + 8] = b.linearVelocity.y
        ..[o + 9] = b.linearVelocity.z
        ..[o + 10] = b.angularVelocity.x
        ..[o + 11] = b.angularVelocity.y
        ..[o + 12] = b.angularVelocity.z
        ..[o + 13] = b.awake ? 1 : 0;
    }
    out.set(handles.length, List.of(handles), data);
  }

  @override
  List<PhysicsEvent3D> drainEvents() {
    final events = List.of(_events);
    _events.clear();
    return events;
  }

  // ── Queries ───────────────────────────────────────────────────────────

  bool _passes(FakeBody3D b, QueryFilter3D f) =>
      b.handle != f.ignoreBody &&
      (f.layerMask >> b.layer) & 1 == 1 &&
      (f.includeSensors || !b.isSensor) &&
      (f.includeCharacters || b.character == 0);

  @override
  List<QueryHit3D> raycast(
    Vector3 origin,
    Vector3 direction, {
    QueryFilter3D filter = QueryFilter3D.all,
    bool allHits = false,
  }) => _cast(origin, direction, Vector3.zero(), filter, allHits);

  @override
  List<QueryHit3D> shapeCast(
    Shape3D shape,
    Vector3 position,
    Quaternion rotation,
    Vector3 direction, {
    QueryFilter3D filter = QueryFilter3D.all,
    bool allHits = false,
  }) {
    final (min, max) = _boundsOf(shape);
    final half = Vector3(
      (max.x - min.x) / 2,
      (max.y - min.y) / 2,
      (max.z - min.z) / 2,
    );
    final centre = Vector3(
      (max.x + min.x) / 2,
      (max.y + min.y) / 2,
      (max.z + min.z) / 2,
    )..add(position);
    return _cast(centre, direction, half, filter, allHits);
  }

  /// A ray from [origin] along [direction] against every box grown by
  /// [grow] (a swept box).
  List<QueryHit3D> _cast(
    Vector3 origin,
    Vector3 direction,
    Vector3 grow,
    QueryFilter3D f,
    bool all,
  ) {
    final hits = <QueryHit3D>[];
    final o = [origin.x, origin.y, origin.z],
        d = [direction.x, direction.y, direction.z];
    for (final b in bodies.values) {
      if (!_passes(b, f)) continue;
      final lo = [b.minX - grow.x, b.minY - grow.y, b.minZ - grow.z];
      final hi = [b.maxX + grow.x, b.maxY + grow.y, b.maxZ + grow.z];
      var tMin = 0.0, tMax = 1.0, axis = -1, sign = 0.0;
      var miss = false;
      for (var k = 0; k < 3 && !miss; k++) {
        if (d[k].abs() < 1e-12) {
          if (o[k] < lo[k] || o[k] > hi[k]) miss = true;
          continue;
        }
        var t1 = (lo[k] - o[k]) / d[k], t2 = (hi[k] - o[k]) / d[k];
        var s = -1.0;
        if (t1 > t2) {
          final t = t1;
          t1 = t2;
          t2 = t;
          s = 1.0;
        }
        if (t1 > tMin) {
          tMin = t1;
          axis = k;
          sign = s;
        }
        tMax = math.min(tMax, t2);
        if (tMin > tMax) miss = true;
      }
      if (miss) continue;
      final normal = Vector3.zero();
      if (axis == 0) normal.x = sign;
      if (axis == 1) normal.y = sign;
      if (axis == 2) normal.z = sign;
      hits.add(
        QueryHit3D(
          body: b.handle,
          fraction: tMin,
          point: Vector3.copy(origin)..addScaled(direction, tMin),
          normal: normal,
        ),
      );
    }
    hits.sort((a, b) => a.fraction.compareTo(b.fraction));
    return all || hits.isEmpty ? hits : [hits.first];
  }

  @override
  List<QueryHit3D> overlap(
    Shape3D shape,
    Vector3 position,
    Quaternion rotation, {
    QueryFilter3D filter = QueryFilter3D.all,
  }) {
    final (min, max) = _boundsOf(shape);
    min.add(position);
    max.add(position);
    return [
      for (final b in bodies.values)
        if (_passes(b, filter) &&
            min.x < b.maxX &&
            max.x > b.minX &&
            min.y < b.maxY &&
            max.y > b.minY &&
            min.z < b.maxZ &&
            max.z > b.minZ)
          QueryHit3D(
            body: b.handle,
            fraction: 0,
            point: Vector3.copy(b.position),
            normal: Vector3.zero(),
          ),
    ];
  }

  @override
  Float32List? debugLines({bool joints = false}) => null;
}

/// [shape]'s bounds in its own space.
(Vector3, Vector3) _boundsOf(Shape3D shape) {
  switch (shape) {
    case BoxShape3D(:final hx, :final hy, :final hz):
      return (Vector3(-hx, -hy, -hz), Vector3(hx, hy, hz));
    case SphereShape3D(:final radius):
      return (
        Vector3(-radius, -radius, -radius),
        Vector3(radius, radius, radius),
      );
    case CapsuleShape3D(:final halfHeight, :final radius):
      final h = halfHeight + radius;
      return (Vector3(-radius, -h, -radius), Vector3(radius, h, radius));
    case CylinderShape3D(:final halfHeight, :final radius):
      return (
        Vector3(-radius, -halfHeight, -radius),
        Vector3(radius, halfHeight, radius),
      );
    case ConvexHullShape3D(:final points):
      return _pointBounds(points);
    case MeshShape3D(:final positions):
      return _pointBounds(positions);
    case HeightFieldShape3D(
      :final heights,
      :final sampleCount,
      :final cellSize,
    ):
      var low = double.infinity, high = -double.infinity;
      for (final h in heights) {
        low = math.min(low, h);
        high = math.max(high, h);
      }
      final side = (sampleCount - 1) * cellSize;
      // Heights go up, towards −Y.
      return (Vector3(0, -high, 0), Vector3(side, -low, side));
    case TransformedShape3D(:final shape, :final x, :final y, :final z):
      final (min, max) = _boundsOf(shape);
      final offset = Vector3(x, y, z);
      return (min..add(offset), max..add(offset));
    case CompoundShape3D(:final parts):
      // The box round every part.
      final (min, max) = _boundsOf(parts.first);
      for (final part in parts.skip(1)) {
        final (lo, hi) = _boundsOf(part);
        min.setValues(math.min(min.x, lo.x), math.min(min.y, lo.y), math.min(min.z, lo.z));
        max.setValues(math.max(max.x, hi.x), math.max(max.y, hi.y), math.max(max.z, hi.z));
      }
      return (min, max);
  }
}

(Vector3, Vector3) _pointBounds(Float32List p) {
  if (p.length < 3) return (Vector3.zero(), Vector3.zero());
  final min = Vector3(p[0], p[1], p[2]), max = Vector3(p[0], p[1], p[2]);
  for (var i = 3; i + 2 < p.length; i += 3) {
    min.setValues(
      math.min(min.x, p[i]),
      math.min(min.y, p[i + 1]),
      math.min(min.z, p[i + 2]),
    );
    max.setValues(
      math.max(max.x, p[i]),
      math.max(max.y, p[i + 1]),
      math.max(max.z, p[i + 2]),
    );
  }
  return (min, max);
}
