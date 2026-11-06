part of 'fake_physics_3d_engine.dart';

/// The world of a [FakePhysics3DEngine]. Its bodies, joints and
/// characters are open for tests to look at.
final class FakePhysics3DWorld implements Physics3DWorld {
  /// A world made as [settings] says. Its handles carry [tag] (its engine
  /// gives each world its own); worlds made on their own share tag 0.
  FakePhysics3DWorld(
    this.settings, {
    this.collisions = true,
    FakePhysics3DEngine? owner,
    int tag = 0,
  }) : _owner = owner,
       _tag = tag,
       _gravity = Vector3.copy(settings.gravity),
       _layerMasks = settings.layerMasks;

  /// What it was made with.
  final Physics3DWorldSettings settings;

  /// Whether it resolves collisions.
  final bool collisions;

  final FakePhysics3DEngine? _owner;
  final Vector3 _gravity;
  List<int> _layerMasks;

  // Handles: the world's tag above 32 bits, a count below (by arithmetic,
  // not shifts, so they are the same on the web).
  final int _tag;
  int _count = 0;
  int _nextHandle() => _tag * 0x100000000 + ++_count;

  /// How long a body must be still before it sleeps, in steps.
  static const int _sleepSteps = 30;

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

  final List<Physics3DEvent> _events = [];
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
    _events.clear();
    _fellAsleep.clear();
    _touching.clear();
    _owner?.worlds.remove(this);
  }

  @override
  Vector3 get gravity => Vector3.copy(_gravity);

  @override
  set gravity(Vector3 value) {
    if (!_disposed) _gravity.setFrom(value);
  }

  @override
  int get bodyCount => bodies.length;

  @override
  List<int> get layerMasks => _layerMasks;

  @override
  void setLayerMasks(List<int> masks) {
    if (_disposed) return;
    final rows = Physics3DWorldSettings.normalizeLayerMasks(masks);
    var same = true;
    for (var i = 0; i < 16; i++) {
      same = same && rows[i] == _layerMasks[i];
    }
    if (same) return;
    _layerMasks = rows;
    // As in Jolt: every body that can move is woken, so pairs at rest are
    // judged again in the next step.
    for (final b in bodies.values) {
      if (b.character == 0) _wake(b);
    }
  }

  bool _collide(int a, int b) =>
      (_layerMasks[a] >> b) & 1 == 1 && (_layerMasks[b] >> a) & 1 == 1;

  /// Whether [a] and [b] can touch at all, by Jolt's rules: their kinds
  /// (just_jolt's object layers), then one of them dynamic or a kinematic
  /// body meeting a sensor.
  static bool _canTouch(FakeBody3D a, FakeBody3D b) {
    var ka = a._kind, kb = b._kind;
    if (ka > kb) (ka, kb) = (kb, ka);
    final kinds = switch (ka) {
      0 => kb == 1 || kb == 2,
      1 => true,
      2 => kb == 3,
      _ => false,
    };
    if (!kinds) return false;
    return a.type == Body3DType.dynamic ||
        b.type == Body3DType.dynamic ||
        (a.type == Body3DType.kinematic && b.isSensor) ||
        (b.type == Body3DType.kinematic && a.isSensor);
  }

  // ── Bodies ────────────────────────────────────────────────────────────

  /// The most bodies it holds: Jolt's default when the settings give none.
  int get _maxBodies => settings.maxBodies > 0 ? settings.maxBodies : 10240;

  @override
  int createBody(Shape3D shape, Body3DSettings settings) {
    if (_disposed ||
        settings.layer < 0 ||
        settings.layer > 15 ||
        !shape.isValid ||
        bodies.length >= _maxBodies) {
      return 0;
    }
    if (_mustBeStatic(shape) && settings.type != Body3DType.static) return 0;
    final body = FakeBody3D._(_nextHandle(), shape, settings);
    bodies[body.handle] = body;
    if (body.awake) _activated(body);
    return body.handle;
  }

  /// Meshes and height fields: static bodies only, as in Jolt.
  static bool _mustBeStatic(Shape3D shape) => switch (shape) {
    HeightField3DShape() || Mesh3DShape() => true,
    Transformed3DShape(:final shape) => _mustBeStatic(shape),
    Compound3DShape(:final parts) => parts.any(_mustBeStatic),
    _ => false,
  };

  @override
  void destroyBody(int body) {
    // A character's body goes with its character.
    if ((bodies[body]?.character ?? 1) != 0) return;
    _destroyBody(body);
  }

  void _destroyBody(int body) {
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
    b.rotation.setFrom(_unitOf(rotation));
    b._turn();
    if (wake) _wake(b);
  }

  /// [body] when it moves by itself: not static, not a character's (its
  /// character moves it), as Jolt's world treats them.
  FakeBody3D? _moving(int body) {
    final b = bodies[body];
    return b == null || b.type == Body3DType.static || b.character != 0
        ? null
        : b;
  }

  @override
  void moveKinematic(
    int body,
    Vector3 position,
    Quaternion rotation,
    double dt,
  ) {
    final b = _moving(body);
    if (b == null || b.type != Body3DType.kinematic || dt <= 0) return;
    b._kinematicTarget = Vector3.copy(position);
    b.linearVelocity
      ..setFrom(position)
      ..sub(b.position)
      ..scale(1 / dt);
    b.rotation.setFrom(_unitOf(rotation));
    b._turn();
    _wake(b);
  }

  void _wake(FakeBody3D b) {
    if (b.type == Body3DType.static) return;
    final wasAsleep = !b.awake;
    b.awake = true;
    b._restSteps = 0;
    if (wasAsleep) _activated(b);
  }

  void _activated(FakeBody3D b) {
    if (settings.reportActivation) {
      _events.add(
        Physics3DEvent(Physics3DEventType.bodyActivated, b.handle, 0),
      );
    }
  }

  void _fallAsleep(FakeBody3D b) {
    b.awake = false;
    b.linearVelocity.setZero();
    _fellAsleep.add(b.handle);
    if (settings.reportActivation) {
      _events.add(
        Physics3DEvent(Physics3DEventType.bodyDeactivated, b.handle, 0),
      );
    }
  }

  @override
  void setLinearVelocity(int body, Vector3 velocity) {
    final b = _moving(body);
    if (b == null) return;
    b.linearVelocity.setFrom(velocity);
    _wake(b);
  }

  @override
  void setAngularVelocity(int body, Vector3 velocity) {
    final b = _moving(body);
    if (b == null) return;
    b.angularVelocity.setFrom(velocity);
    _wake(b);
  }

  @override
  void addForce(int body, Vector3 force) {
    final b = bodies[body];
    if (b == null || b.type != Body3DType.dynamic) return;
    b.force.add(force);
    _wake(b);
  }

  @override
  void addTorque(int body, Vector3 torque) {
    final b = bodies[body];
    if (b == null || b.type != Body3DType.dynamic) return;
    b.torque.add(torque);
    _wake(b);
  }

  @override
  void addImpulse(int body, Vector3 impulse) {
    final b = bodies[body];
    if (b == null || b.type != Body3DType.dynamic) return;
    b.linearVelocity.addScaled(impulse, 1 / b.mass);
    _wake(b);
  }

  @override
  void addAngularImpulse(int body, Vector3 impulse) {
    final b = bodies[body];
    if (b == null || b.type != Body3DType.dynamic) return;
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
      _moving(body)?.gravityFactor = factor;

  @override
  void setLayer(int body, int layer) {
    if (layer >= 0 && layer <= 15) bodies[body]?.layer = layer;
  }

  @override
  void setMotionQuality(int body, Motion3DQuality quality) =>
      _moving(body)?.motionQuality = quality;

  @override
  void wake(int body) {
    final b = _moving(body);
    if (b != null) _wake(b);
  }

  @override
  bool isAwake(int body) => _moving(body)?.awake ?? false;

  // ── Joints ────────────────────────────────────────────────────────────

  @override
  int createJoint(int bodyA, int bodyB, Joint3DSettings settings) {
    // Bodies createBody made: not a character's.
    if (bodies[bodyA]?.character != 0 ||
        (bodyB != 0 && bodies[bodyB]?.character != 0)) {
      return 0;
    }
    if (bodyA == bodyB) return 0;
    if (settings.kind == Joint3DKind.sixDof &&
        (settings.sixDofModes.length < 6 ||
            settings.sixDofMin.length < 6 ||
            settings.sixDofMax.length < 6)) {
      return 0;
    }
    final joint = FakeJoint3D._(_nextHandle(), bodyA, bodyB, settings);
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
  void setJointMotor(int joint, Motor3DState state, double target) {
    final j = joints[joint];
    if (j == null) return;
    j
      ..motorState = state
      ..motorTarget = target;
  }

  @override
  double jointValue(int joint) {
    final j = joints[joint];
    final measured =
        j != null &&
        (j.settings.kind == Joint3DKind.hinge ||
            j.settings.kind == Joint3DKind.slider);
    return measured && j.motorState == Motor3DState.position
        ? j.motorTarget
        : 0;
  }

  // ── Characters ────────────────────────────────────────────────────────

  @override
  int createCharacter(Character3DSettings settings, Vector3 position) {
    if (_disposed ||
        settings.layer < 0 ||
        settings.layer > 15 ||
        !(settings.radius > 0) ||
        settings.halfHeight < 0) {
      return 0;
    }
    final c = FakeCharacter3D._(_nextHandle(), settings, position);
    characters[c.handle] = c;
    if (settings.innerBody) {
      final h = c._halfHeight;
      final inner = createBody(
        Box3DShape(settings.radius, h, settings.radius),
        Body3DSettings(
          position: Vector3(position.x, position.y - h, position.z),
          type: Body3DType.kinematic,
          layer: settings.layer,
          allowSleeping: false,
        ),
      );
      // A full world: the character is made without one, as in Jolt.
      if (inner != 0) {
        bodies[inner]!.character = c.handle;
        c.innerBody = inner;
      }
    }
    return c.handle;
  }

  @override
  void destroyCharacter(int character) {
    final c = characters.remove(character);
    if (c != null && c.innerBody != 0) _destroyBody(c.innerBody);
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
    Character3DState out, {
    required Vector3 stepDown,
    required Vector3 stepUp,
  }) {
    final c = characters[character];
    if (c == null) {
      _clearState(out);
      return;
    }
    c.moves++;
    final r = c.settings.radius, h = c._halfHeight;
    final p = c.position;
    final start = Vector3.copy(p);
    final wasGrounded = c.groundState == Ground3DState.onGround;
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
      // Out on the side its middle is on, so a platform rising into the
      // feet lifts the character instead of swallowing it.
      if (p.y - h <= (b.minY + b.maxY) / 2) {
        p.y = b.minY; // On top (+Y is down).
        if (vy > 0) vy = 0;
      } else {
        p.y = b.maxY + 2 * h; // Head on a ceiling.
        if (vy < 0) vy = 0;
      }
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
      c.groundState = Ground3DState.onGround;
      c.groundBody = ground.handle;
    } else {
      c.groundState = Ground3DState.inAir;
      c.groundBody = 0;
    }
    c.velocity
      ..setFrom(p)
      ..sub(start)
      ..scale(dt > 0 ? 1 / dt : 0);
    if (c.groundState == Ground3DState.onGround) c.velocity.y = 0;
    if (c.innerBody != 0) {
      bodies[c.innerBody]?.position.setValues(p.x, p.y - h, p.z);
    }
    characterState(character, out);
  }

  @override
  void characterState(int character, Character3DState out) {
    final c = characters[character];
    if (c == null) {
      _clearState(out);
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

  /// What a dead character reads as: nothing, in the air.
  static void _clearState(Character3DState out) {
    out.position.setZero();
    out.velocity.setZero();
    out.groundState = Ground3DState.inAir;
    out.groundNormal.setZero();
    out.groundVelocity.setZero();
    out.groundPosition.setZero();
    out.groundBody = 0;
    out.contactCount = 0;
  }

  @override
  void setCharacterPosition(int character, Vector3 position) {
    final c = characters[character];
    if (c == null) return;
    c.position.setFrom(position);
    c.groundState = Ground3DState.inAir;
    c.groundBody = 0;
    if (c.innerBody != 0) {
      bodies[c.innerBody]?.position.setValues(
        position.x,
        position.y - c._halfHeight,
        position.z,
      );
    }
  }

  @override
  void readCharacterContacts(int character, Character3DContacts out) {
    final c = characters[character];
    final ground = c == null ? null : bodies[c.groundBody];
    if (c == null || ground == null) {
      out.set(0, const [], Float32List(0), const []);
      return;
    }
    // What it stands on, the one contact it makes.
    final v = ground.linearVelocity;
    out.set(
      1,
      [ground.handle, 0],
      Float32List.fromList([
        c.position.x,
        ground.minY,
        c.position.z,
        0,
        -1,
        0,
        v.x,
        v.y,
        v.z,
        0,
      ]),
      const [2],
    );
  }

  @override
  List<Character3DContact> characterContacts(int character) {
    final read = Character3DContacts();
    readCharacterContacts(character, read);
    return [for (var i = 0; i < read.count; i++) read.contactAt(i)];
  }

  // ── Stepping ──────────────────────────────────────────────────────────

  @override
  void step(double dt, {int collisionSteps = 1}) {
    if (_disposed || dt <= 0) return;
    steps++;
    lastStep = dt;
    final all = bodies.values.toList(growable: false);
    for (final b in all) {
      if (!b.awake) continue;
      if (b.type == Body3DType.kinematic) {
        final target = b._kinematicTarget;
        if (target != null) {
          b.position.setFrom(target);
          b._kinematicTarget = null;
        } else {
          b.position.addScaled(b.linearVelocity, dt);
        }
        // A sensor makes no contact, so it wakes nothing (Jolt wakes bodies
        // only as it makes contacts).
        if (collisions && !b.isSensor && b.linearVelocity.lengthSquared > 0) {
          _wakeTouched(b, all);
        }
        // Still for long enough, it sleeps, as in Jolt.
        if (b.allowSleeping &&
            target == null &&
            b.linearVelocity.lengthSquared < 0.05 * 0.05) {
          if (++b._restSteps > _sleepSteps) _fallAsleep(b);
        } else {
          b._restSteps = 0;
        }
        continue;
      }
      if (b.type != Body3DType.dynamic) continue;
      final v = b.linearVelocity;
      // Moving before this step's gravity: at rest, it wakes nothing.
      final moving = v.lengthSquared >= 0.05 * 0.05;
      v.addScaled(_gravity, b.gravityFactor * dt);
      v.addScaled(b.force, dt / b.mass);
      b.force.setZero();
      b.torque.setZero();
      v.scale(math.max(0.0, 1 - b.linearDamping * dt));
      b.position.addScaled(v, dt);
      if (collisions) _resolve(b, all, wake: moving);
      if (b.allowSleeping && v.lengthSquared < 0.05 * 0.05) {
        if (++b._restSteps > _sleepSteps) _fallAsleep(b);
      } else {
        b._restSteps = 0;
      }
    }
    if (collisions) _findContacts(all);
  }

  /// Wakes the sleeping dynamic bodies a moving [b] touches; a sensor
  /// makes no contact, so it is never woken this way.
  void _wakeTouched(FakeBody3D b, List<FakeBody3D> all) {
    const slop = 0.01;
    for (final o in all) {
      if (o.awake ||
          o.isSensor ||
          o.type != Body3DType.dynamic ||
          !_collide(b.layer, o.layer) ||
          !_canTouch(b, o)) {
        continue;
      }
      if (b.minX > o.maxX + slop || o.minX > b.maxX + slop) continue;
      if (b.minY > o.maxY + slop || o.minY > b.maxY + slop) continue;
      if (b.minZ > o.maxZ + slop || o.minZ > b.maxZ + slop) continue;
      _wake(o);
    }
  }

  /// Pushes [b] out of every solid it overlaps, along the shallowest axis,
  /// waking what it runs into. Bodies at rest wake nothing, so bodies at
  /// rest together sleep together, as an island does in Jolt.
  void _resolve(FakeBody3D b, List<FakeBody3D> all, {required bool wake}) {
    if (b.isSensor) return;
    if (wake) _wakeTouched(b, all);
    for (final o in all) {
      if (identical(o, b) ||
          o.isSensor ||
          !_collide(b.layer, o.layer) ||
          !_canTouch(b, o)) {
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
        final back = dx2 < dx1; // Push towards −X.
        b.position.x += back ? -dx2 : dx1;
        if ((back && v.x > 0) || (!back && v.x < 0)) v.x = -v.x * bounce;
      } else {
        final back = dz2 < dz1; // Push towards −Z.
        b.position.z += back ? -dz2 : dz1;
        if ((back && v.z > 0) || (!back && v.z < 0)) v.z = -v.z * bounce;
      }
    }
  }

  /// Whether [b] is awake, and a body that moves.
  static bool _moves(FakeBody3D b) => b.awake && b.type != Body3DType.static;

  void _findContacts(List<FakeBody3D> all) {
    final now = <(int, int), (bool, FakeBody3D, FakeBody3D)>{};
    const slop = 0.01;
    for (var i = 0; i < all.length; i++) {
      final a = all[i];
      for (var j = i + 1; j < all.length; j++) {
        final b = all[j];
        if (!_canTouch(a, b) || !_collide(a.layer, b.layer)) continue;
        final sensor = a.isSensor || b.isSensor;
        if (a.minX > b.maxX + slop || b.minX > a.maxX + slop) continue;
        if (a.minY > b.maxY + slop || b.minY > a.maxY + slop) continue;
        if (a.minZ > b.maxZ + slop || b.minZ > a.maxZ + slop) continue;
        now[(a.handle, b.handle)] = (sensor, a, b);
      }
    }
    for (final MapEntry(key: pair, value: (sensor, a, b)) in now.entries) {
      final known = _touching.containsKey(pair);
      if (known && !(settings.reportContactStay && !sensor)) continue;
      // Jolt goes on with a contact only while one of its bodies is awake
      // and moves: a pair asleep together stays quiet.
      if (known && !_moves(a) && !_moves(b)) continue;
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
          ? Physics3DEventType.sensorBegin
          : known
          ? Physics3DEventType.contactStay
          : Physics3DEventType.contactBegin;
      _events.add(
        Physics3DEvent(
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
        Physics3DEvent(
          sensor ? Physics3DEventType.sensorEnd : Physics3DEventType.contactEnd,
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
  void readMovedBodies(Body3DPoses out) {
    final moved = [
      for (final b in bodies.values)
        if (b.awake && b.type != Body3DType.static && b.character == 0)
          b.handle,
      ..._fellAsleep.where(bodies.containsKey),
    ];
    _fellAsleep.clear();
    readBodies(moved, out);
  }

  @override
  void readBodies(List<int> handles, Body3DPoses out) {
    final data = Float32List(handles.length * Body3DPoses.stride);
    for (var i = 0; i < handles.length; i++) {
      final b = bodies[handles[i]];
      if (b == null) continue;
      final o = i * Body3DPoses.stride;
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
  void readEvents(Physics3DEvents out) {
    final n = _disposed ? 0 : _events.length;
    final types = <int>[], handles = <int>[];
    final data = Float32List(n * Physics3DEvents.stride);
    for (var i = 0; i < n; i++) {
      final e = _events[i];
      final o = i * Physics3DEvents.stride;
      types.add(e.type.index);
      handles
        ..add(e.bodyA)
        ..add(e.bodyB);
      data
        ..[o] = e.point.x
        ..[o + 1] = e.point.y
        ..[o + 2] = e.point.z
        ..[o + 3] = e.normal.x
        ..[o + 4] = e.normal.y
        ..[o + 5] = e.normal.z
        ..[o + 6] = e.depth;
    }
    _events.clear();
    out.set(n, types, handles, data);
  }

  @override
  List<Physics3DEvent> drainEvents() {
    if (_disposed) return const [];
    final events = List.of(_events);
    _events.clear();
    return events;
  }

  // ── Queries ───────────────────────────────────────────────────────────

  bool _passes(FakeBody3D b, Query3DFilter f) =>
      b.handle != f.ignoreBody &&
      (f.layerMask >> b.layer) & 1 == 1 &&
      (f.includeSensors || !b.isSensor) &&
      (f.includeCharacters || b.character == 0);

  @override
  List<Query3DHit> raycast(
    Vector3 origin,
    Vector3 direction, {
    Query3DFilter filter = Query3DFilter.all,
    bool allHits = false,
  }) => _cast(origin, direction, Vector3.zero(), filter, allHits);

  @override
  List<Query3DHit> shapeCast(
    Shape3D shape,
    Vector3 position,
    Quaternion rotation,
    Vector3 direction, {
    Query3DFilter filter = Query3DFilter.all,
    bool allHits = false,
  }) {
    if (_disposed || !shape.isValid) return const [];
    final (lo, hi) = _boundsOf(shape);
    final (min, max) = _turned(lo, hi, rotation);
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
  List<Query3DHit> _cast(
    Vector3 origin,
    Vector3 direction,
    Vector3 grow,
    Query3DFilter f,
    bool all,
  ) {
    final hits = <Query3DHit>[];
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
        Query3DHit(
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
  List<Query3DHit> overlap(
    Shape3D shape,
    Vector3 position,
    Quaternion rotation, {
    Query3DFilter filter = Query3DFilter.all,
  }) {
    if (_disposed || !shape.isValid) return const [];
    final (lo, hi) = _boundsOf(shape);
    final (min, max) = _turned(lo, hi, rotation);
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
          Query3DHit(
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
