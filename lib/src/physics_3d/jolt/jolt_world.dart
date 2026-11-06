import 'dart:ffi';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:ffi/ffi.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_jolt/ffi.dart';
import 'package:meta/meta.dart' show internal;

import '../physics_3d_engine.dart';
import '../physics_3d_results.dart';
import '../physics_3d_settings.dart';
import '../shape_3d.dart';
import 'jolt_shapes.dart';

/// A Jolt Physics world: [Physics3DWorld] on just_jolt's native layer.
///
/// Made by `JoltPhysicsEngine.createWorld`. Handles are the native layer's
/// int64 handles. A world acts only on the bodies, joints and characters it
/// made itself, so one from another world, a dead one, or a character's
/// stand-in body passed to [destroyBody] or [createJoint] is passed over.
/// Shapes are shared and reference-counted ([JoltShapes]).
///
/// Destroy worlds through `Physics3DEngine.destroyAll`, not just_jolt's
/// `Jolt.destroyAll`: a world destroyed behind its back still calls Jolt's
/// body interface through memory that has gone. Use Jolt from one isolate.
///
/// The world, its scratch memory and its shapes each carry a
/// [NativeFinalizer], so a world dropped without [dispose] is still freed —
/// in whatever order the finalizers run, which the native layer allows.
final class JoltWorld implements Physics3DWorld, Finalizable {
  /// Makes a world as [settings] say. Throws a [StateError] when Jolt
  /// cannot (it was not initialised).
  JoltWorld(Physics3DWorldSettings settings)
    : _gravity = Vector3.copy(settings.gravity),
      _layerMasks = settings.layerMasks {
    final s = _alloc<JJ_WorldSettings>();
    s.ref
      ..gravity[0] = settings.gravity.x
      ..gravity[1] = settings.gravity.y
      ..gravity[2] = settings.gravity.z
      ..maxBodies = settings.maxBodies
      ..reportContactStay = settings.reportContactStay
      ..reportActivation = settings.reportActivation
      // The less bouncy of the two bodies, as in the 2-D engines.
      ..restitutionCombine = JJ_COMBINE_MIN;
    for (var i = 0; i < 16; i++) {
      s.ref.collides[i] = settings.layerMasks[i];
    }
    handle = jj_world_create(s);
    if (handle == 0) throw StateError('Jolt could not make a world.');
    _worldFinalizer.attach(this, Pointer.fromAddress(handle), detach: this);
    _tag = jj_world_handle_tag(handle);
    _bodyInterface = JPH_PhysicsSystem_GetBodyInterface(jj_world_joltc(handle));
  }

  static final NativeFinalizer _worldFinalizer = NativeFinalizer(
    addresses.jj_world_finalize.cast(),
  );
  static final NativeFinalizer _free = NativeFinalizer(calloc.nativeFree);

  /// The native world's handle.
  late final int handle;
  late final int _tag;
  late final Pointer<JPH_BodyInterface> _bodyInterface;

  /// Bumped when every native world is destroyed at once
  /// (`Physics3DEngine.destroyAll`); a world made before is gone with them.
  static int _generation = 0;
  final int _born = _generation;

  /// Marks every world made so far as gone, after their native worlds were
  /// destroyed together.
  @internal
  static void forgetAll() => _generation++;

  bool get _gone => _disposed || _born != _generation;

  /// This world's shapes.
  final JoltShapes shapes = JoltShapes();

  final Vector3 _gravity;
  bool _disposed = false;

  /// Native memory this world owns, by address, with the object that
  /// detaches its finalizer.
  final Map<int, Object> _owned = {};

  final Map<int, Shape3D> _bodyShapes = {};
  final Map<int, Body3DType> _bodyTypes = {};
  final Map<int, (int, int)> _jointBodies = {};
  final Map<int, Set<int>> _bodyJoints = {};
  final Set<int> _characters = {};

  // Scratch memory, made once.
  late final Pointer<JPH_Vec3> _v1 = _alloc<JPH_Vec3>();
  late final Pointer<JPH_Quat> _q = _alloc<JPH_Quat>();
  late final Pointer<Float> _f = _alloc<Float>(16);
  late final Pointer<JJ_BodySettings> _bodySettings = _alloc<JJ_BodySettings>();
  late final Pointer<JJ_JointSettings> _jointSettings =
      _alloc<JJ_JointSettings>();
  late final Pointer<JJ_CharacterSettings> _characterSettings =
      _alloc<JJ_CharacterSettings>();
  late final Pointer<JJ_CharacterState> _characterState =
      _alloc<JJ_CharacterState>();

  // Buffers that grow. The pose buffers are freed by their typed views, so
  // a Body3DPoses read from them stays readable after the world is gone.
  int _poseCapacity = 0;
  Pointer<Int64> _poseIds = nullptr;
  Pointer<Float> _poses = nullptr;
  Int64List _poseIdView = Int64List(0);
  Float32List _poseView = Float32List(0);
  int _eventCapacity = 256;
  late Pointer<JJ_Event> _events = _alloc<JJ_Event>(_eventCapacity);
  int _hitCapacity = 64;
  late Pointer<JJ_Hit> _hits = _alloc<JJ_Hit>(_hitCapacity);
  int _contactCapacity = 16;
  late Pointer<JJ_CharacterContact> _contacts = _alloc<JJ_CharacterContact>(
    _contactCapacity,
  );

  Pointer<T> _alloc<T extends NativeType>([int count = 1]) {
    final p = calloc<Uint8>(_sizeOf<T>() * count);
    final token = Object();
    _owned[p.address] = token;
    _free.attach(this, p.cast(), detach: token);
    return p.cast<T>();
  }

  static int _sizeOf<T extends NativeType>() => switch (T) {
    const (JPH_Vec3) => sizeOf<JPH_Vec3>(),
    const (JPH_Quat) => sizeOf<JPH_Quat>(),
    const (Float) => sizeOf<Float>(),
    const (Int64) => sizeOf<Int64>(),
    const (JJ_WorldSettings) => sizeOf<JJ_WorldSettings>(),
    const (JJ_BodySettings) => sizeOf<JJ_BodySettings>(),
    const (JJ_JointSettings) => sizeOf<JJ_JointSettings>(),
    const (JJ_CharacterSettings) => sizeOf<JJ_CharacterSettings>(),
    const (JJ_CharacterState) => sizeOf<JJ_CharacterState>(),
    const (JJ_CharacterContact) => sizeOf<JJ_CharacterContact>(),
    const (JJ_Event) => sizeOf<JJ_Event>(),
    const (JJ_Hit) => sizeOf<JJ_Hit>(),
    const (Uint32) => sizeOf<Uint32>(),
    _ => throw ArgumentError('No size for $T'),
  };

  void _drop(Pointer<NativeType> p) {
    final token = _owned.remove(p.address);
    if (token == null) return;
    _free.detach(token);
    calloc.free(p);
  }

  /// Whether [handle] is one of this world's bodies' handles (alive or
  /// not); a cheap check before a call into Jolt's body interface.
  bool _mine(int body) => body != 0 && (body >>> 32) == _tag;

  int _id(int body) => body & 0xFFFFFFFF;

  Pointer<JPH_Vec3> _vec(Pointer<JPH_Vec3> p, Vector3 v) {
    p.ref
      ..x = v.x
      ..y = v.y
      ..z = v.z;
    return p;
  }

  /// [q] made unit length for Jolt; the identity when it has no length.
  Pointer<JPH_Quat> _quat(Quaternion q) {
    final length = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    final ok = length.isFinite && length > 1e-12;
    _q.ref
      ..x = ok ? q.x / length : 0
      ..y = ok ? q.y / length : 0
      ..z = ok ? q.z / length : 0
      ..w = ok ? q.w / length : 1;
    return _q;
  }

  @override
  bool get isDisposed => _gone;

  @override
  void dispose() {
    if (_disposed) return;
    _disposed = true;
    _worldFinalizer.detach(this);
    jj_world_destroy(handle);
    shapes.clear();
    _bodyShapes.clear();
    _bodyTypes.clear();
    _jointBodies.clear();
    _bodyJoints.clear();
    _characters.clear();
    for (final address in List.of(_owned.keys)) {
      _drop(Pointer<Uint8>.fromAddress(address));
    }
  }

  @override
  Vector3 get gravity => Vector3.copy(_gravity);

  @override
  set gravity(Vector3 value) {
    if (_gone) return;
    _gravity.setFrom(value);
    _f
      ..[0] = value.x
      ..[1] = value.y
      ..[2] = value.z;
    jj_world_set_gravity(handle, _f);
  }

  @override
  int get bodyCount => _gone ? 0 : jj_world_body_count(handle);

  List<int> _layerMasks;
  late final Pointer<Uint32> _grid = _alloc<Uint32>(16);

  @override
  List<int> get layerMasks => _layerMasks;

  @override
  void setLayerMasks(List<int> masks) {
    if (_gone) return;
    final rows = Physics3DWorldSettings.normalizeLayerMasks(masks);
    var same = true;
    for (var i = 0; i < 16; i++) {
      same = same && rows[i] == _layerMasks[i];
      _grid[i] = rows[i];
    }
    if (same) return;
    _layerMasks = rows;
    jj_world_set_collision_grid(handle, _grid);
  }

  /// Rebuilds the broad phase; worth it after adding many static bodies at
  /// once (a level loading).
  void optimize() {
    if (!_gone) jj_world_optimize(handle);
  }

  // ── Bodies ────────────────────────────────────────────────────────────

  @override
  int createBody(Shape3D shape, Body3DSettings settings) {
    if (_gone) return 0;
    final pointer = shapes.acquire(shape);
    if (pointer == null) return 0;
    final s = _bodySettings.ref;
    s.position[0] = settings.position.x;
    s.position[1] = settings.position.y;
    s.position[2] = settings.position.z;
    s.rotation[0] = settings.rotation.x;
    s.rotation[1] = settings.rotation.y;
    s.rotation[2] = settings.rotation.z;
    s.rotation[3] = settings.rotation.w;
    s.linearVelocity[0] = settings.linearVelocity.x;
    s.linearVelocity[1] = settings.linearVelocity.y;
    s.linearVelocity[2] = settings.linearVelocity.z;
    s.angularVelocity[0] = settings.angularVelocity.x;
    s.angularVelocity[1] = settings.angularVelocity.y;
    s.angularVelocity[2] = settings.angularVelocity.z;
    s
      ..motionType = settings.type.index
      ..motionQuality = settings.motionQuality.index
      ..layer = settings.layer
      // Translation X, Y and Z only (Jolt's EAllowedDOFs); 0 is all six.
      ..allowedDofs = settings.lockRotation ? 7 : 0
      ..friction = settings.friction
      ..restitution = settings.restitution
      ..linearDamping = settings.linearDamping
      ..angularDamping = settings.angularDamping
      ..gravityFactor = settings.gravityFactor
      ..mass = settings.mass
      ..isSensor = settings.isSensor
      ..allowSleeping = settings.allowSleeping
      ..enhancedInternalEdgeRemoval = false
      ..userData = settings.userData;
    final body = jj_body_create(handle, pointer, _bodySettings);
    if (body == 0) {
      shapes.release(shape);
      return 0;
    }
    _bodyShapes[body] = shape;
    _bodyTypes[body] = settings.type;
    return body;
  }

  @override
  void destroyBody(int body) {
    // Only bodies made by createBody: a character's stand-in body belongs to
    // its character, and destroying it here would destroy it twice.
    if (_gone) return;
    final shape = _bodyShapes.remove(body);
    if (shape == null) return;
    _bodyTypes.remove(body);
    jj_body_destroy(body);
    shapes.release(shape);
    // The native layer destroyed the joints holding it.
    for (final joint in _bodyJoints.remove(body)?.toList() ?? const <int>[]) {
      _forgetJoint(joint);
    }
  }

  @override
  bool isBody(int body) => !_gone && _mine(body) && jj_body_valid(body);

  bool _moving(int body) {
    if (_gone || !_mine(body)) return false;
    final type = _bodyTypes[body];
    return type != null && type != Body3DType.static;
  }

  bool _dynamic(int body) =>
      !_gone && _mine(body) && _bodyTypes[body] == Body3DType.dynamic;

  @override
  void setBodyPose(
    int body,
    Vector3 position,
    Quaternion rotation, {
    bool wake = true,
  }) {
    if (_gone || !_mine(body)) return;
    JPH_BodyInterface_SetPositionAndRotation(
      _bodyInterface,
      _id(body),
      _vec(_v1, position),
      _quat(rotation),
      wake
          ? JPH_Activation.JPH_Activation_Activate
          : JPH_Activation.JPH_Activation_DontActivate,
    );
  }

  @override
  void moveKinematic(
    int body,
    Vector3 position,
    Quaternion rotation,
    double dt,
  ) {
    if (!_moving(body) || dt <= 0) return;
    JPH_BodyInterface_MoveKinematic(
      _bodyInterface,
      _id(body),
      _vec(_v1, position),
      _quat(rotation),
      dt,
    );
  }

  @override
  void setLinearVelocity(int body, Vector3 velocity) {
    if (!_moving(body)) return;
    JPH_BodyInterface_SetLinearVelocity(
      _bodyInterface,
      _id(body),
      _vec(_v1, velocity),
    );
    JPH_BodyInterface_ActivateBody(_bodyInterface, _id(body));
  }

  @override
  void setAngularVelocity(int body, Vector3 velocity) {
    if (!_moving(body)) return;
    JPH_BodyInterface_SetAngularVelocity(
      _bodyInterface,
      _id(body),
      _vec(_v1, velocity),
    );
    JPH_BodyInterface_ActivateBody(_bodyInterface, _id(body));
  }

  @override
  void addForce(int body, Vector3 force) {
    if (!_dynamic(body)) return;
    JPH_BodyInterface_AddForce(_bodyInterface, _id(body), _vec(_v1, force));
    JPH_BodyInterface_ActivateBody(_bodyInterface, _id(body));
  }

  @override
  void addTorque(int body, Vector3 torque) {
    if (!_dynamic(body)) return;
    JPH_BodyInterface_AddTorque(_bodyInterface, _id(body), _vec(_v1, torque));
    JPH_BodyInterface_ActivateBody(_bodyInterface, _id(body));
  }

  @override
  void addImpulse(int body, Vector3 impulse) {
    if (!_dynamic(body)) return;
    JPH_BodyInterface_AddImpulse(_bodyInterface, _id(body), _vec(_v1, impulse));
  }

  @override
  void addAngularImpulse(int body, Vector3 impulse) {
    if (!_dynamic(body)) return;
    JPH_BodyInterface_AddAngularImpulse(
      _bodyInterface,
      _id(body),
      _vec(_v1, impulse),
    );
  }

  @override
  void setFriction(int body, double friction) {
    if (_gone || !_mine(body)) return;
    JPH_BodyInterface_SetFriction(_bodyInterface, _id(body), friction);
  }

  @override
  void setRestitution(int body, double restitution) {
    if (_gone || !_mine(body)) return;
    JPH_BodyInterface_SetRestitution(_bodyInterface, _id(body), restitution);
  }

  @override
  void setGravityFactor(int body, double factor) {
    if (!_moving(body)) return;
    JPH_BodyInterface_SetGravityFactor(_bodyInterface, _id(body), factor);
  }

  @override
  void setLayer(int body, int layer) {
    if (_gone || !_mine(body)) return;
    jj_body_set_layer(body, layer);
  }

  @override
  void setMotionQuality(int body, Motion3DQuality quality) {
    if (!_moving(body)) return;
    JPH_BodyInterface_SetMotionQuality(
      _bodyInterface,
      _id(body),
      quality.index,
    );
  }

  @override
  void wake(int body) {
    if (!_moving(body)) return;
    JPH_BodyInterface_ActivateBody(_bodyInterface, _id(body));
  }

  @override
  bool isAwake(int body) =>
      _moving(body) && JPH_BodyInterface_IsActive(_bodyInterface, _id(body));

  // ── Joints ────────────────────────────────────────────────────────────

  @override
  int createJoint(int bodyA, int bodyB, Joint3DSettings settings) {
    // Only bodies made by createBody: a joint must not hold a character's
    // stand-in body, which goes with its character.
    final sixDofLists =
        settings.sixDofModes.length >= 6 &&
        settings.sixDofMin.length >= 6 &&
        settings.sixDofMax.length >= 6;
    if (_gone ||
        !_bodyShapes.containsKey(bodyA) ||
        (bodyB != 0 && !_bodyShapes.containsKey(bodyB)) ||
        (settings.kind == Joint3DKind.sixDof && !sixDofLists)) {
      return 0;
    }
    final s = _jointSettings.ref;
    void put(Array<Float> to, Vector3 v) {
      to[0] = v.x;
      to[1] = v.y;
      to[2] = v.z;
    }

    s.kind = settings.kind.index;
    put(s.pointA, settings.pointA);
    put(s.pointB, settings.pointB);
    put(s.axis, settings.axis);
    put(s.normal, settings.normal);
    s
      ..hasLimits = settings.hasLimits
      ..limitMin = settings.limitMin
      ..limitMax = settings.limitMax
      ..springFrequency = settings.springFrequency
      ..springDamping = settings.springDamping
      ..maxFriction = settings.maxFriction
      ..motorState = settings.motorState.index
      ..motorTarget = settings.motorTarget
      ..motorMaxForce = settings.motorMaxForce;
    var free = 0, fixed = 0;
    for (var a = 0; a < (sixDofLists ? 6 : 0); a++) {
      switch (settings.sixDofModes[a]) {
        case SixDofAxis3DMode.free:
          free |= 1 << a;
        case SixDofAxis3DMode.fixed:
          fixed |= 1 << a;
        case SixDofAxis3DMode.limited:
          break;
      }
      s.sixDofMin[a] = settings.sixDofMin[a];
      s.sixDofMax[a] = settings.sixDofMax[a];
    }
    s
      ..sixDofFree = free
      ..sixDofFixed = fixed;
    final joint = jj_joint_create(handle, bodyA, bodyB, _jointSettings);
    if (joint != 0) {
      _jointBodies[joint] = (bodyA, bodyB);
      (_bodyJoints[bodyA] ??= {}).add(joint);
      if (bodyB != 0) (_bodyJoints[bodyB] ??= {}).add(joint);
    }
    return joint;
  }

  bool _myJoint(int joint) => !_gone && _jointBodies.containsKey(joint);

  void _forgetJoint(int joint) {
    final bodies = _jointBodies.remove(joint);
    if (bodies == null) return;
    _bodyJoints[bodies.$1]?.remove(joint);
    _bodyJoints[bodies.$2]?.remove(joint);
  }

  @override
  void destroyJoint(int joint) {
    if (!_myJoint(joint)) return;
    _forgetJoint(joint);
    jj_joint_destroy(joint);
  }

  @override
  bool isJoint(int joint) => _myJoint(joint) && jj_joint_valid(joint);

  @override
  void setJointEnabled(int joint, bool enabled) {
    if (_myJoint(joint)) jj_joint_set_enabled(joint, enabled);
  }

  @override
  void setJointMotor(int joint, Motor3DState state, double target) {
    if (_myJoint(joint)) jj_joint_set_motor(joint, state.index, target);
  }

  @override
  double jointValue(int joint) => _myJoint(joint) ? jj_joint_value(joint) : 0;

  // ── Characters ────────────────────────────────────────────────────────

  @override
  int createCharacter(Character3DSettings settings, Vector3 position) {
    if (_gone) return 0;
    final s = _characterSettings.ref;
    s
      ..radius = settings.radius
      ..halfHeight = settings.halfHeight
      ..maxSlopeAngle = settings.maxSlopeAngle
      ..mass = settings.mass
      ..maxStrength = settings.maxStrength
      ..padding = 0
      ..predictiveContactDistance = 0
      ..penetrationRecoverySpeed = 0
      ..layer = settings.layer
      ..innerBody = settings.innerBody
      ..enhancedInternalEdgeRemoval = false
      ..userData = settings.userData;
    s.up[0] = settings.up.x;
    s.up[1] = settings.up.y;
    s.up[2] = settings.up.z;
    _f
      ..[0] = position.x
      ..[1] = position.y
      ..[2] = position.z;
    final character = jj_character_create(handle, _characterSettings, _f);
    if (character != 0) _characters.add(character);
    return character;
  }

  bool _myCharacter(int character) => !_gone && _characters.contains(character);

  @override
  void destroyCharacter(int character) {
    if (!_myCharacter(character)) return;
    _characters.remove(character);
    jj_character_destroy(character);
  }

  @override
  bool isCharacter(int character) =>
      _myCharacter(character) && jj_character_valid(character);

  @override
  int characterBody(int character) =>
      _myCharacter(character) ? jj_character_inner_body(character) : 0;

  @override
  void moveCharacter(
    int character,
    double dt,
    Vector3 velocity,
    Character3DState out, {
    required Vector3 stepDown,
    required Vector3 stepUp,
  }) {
    if (!_myCharacter(character)) {
      _clearState(out);
      return;
    }
    final f = _f;
    f
      ..[0] = velocity.x
      ..[1] = velocity.y
      ..[2] = velocity.z
      ..[3] = _gravity.x
      ..[4] = _gravity.y
      ..[5] = _gravity.z
      ..[6] = stepDown.x
      ..[7] = stepDown.y
      ..[8] = stepDown.z
      ..[9] = stepUp.x
      ..[10] = stepUp.y
      ..[11] = stepUp.z;
    jj_character_step(character, dt, f, f + 3, f + 6, f + 9, _characterState);
    _readState(out);
  }

  @override
  void characterState(int character, Character3DState out) {
    if (!_myCharacter(character)) {
      _clearState(out);
      return;
    }
    jj_character_state(character, _characterState);
    _readState(out);
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

  void _readState(Character3DState out) {
    final s = _characterState.ref;
    out.position.setValues(s.position[0], s.position[1], s.position[2]);
    out.velocity.setValues(s.velocity[0], s.velocity[1], s.velocity[2]);
    out.groundState = Ground3DState.values[s.groundState.clamp(0, 3)];
    out.groundNormal.setValues(
      s.groundNormal[0],
      s.groundNormal[1],
      s.groundNormal[2],
    );
    out.groundVelocity.setValues(
      s.groundVelocity[0],
      s.groundVelocity[1],
      s.groundVelocity[2],
    );
    out.groundPosition.setValues(
      s.groundPosition[0],
      s.groundPosition[1],
      s.groundPosition[2],
    );
    out.groundBody = s.groundBody;
    out.contactCount = s.contactCount;
  }

  @override
  void setCharacterPosition(int character, Vector3 position) {
    if (!_myCharacter(character)) return;
    _f
      ..[0] = position.x
      ..[1] = position.y
      ..[2] = position.z;
    jj_character_set_position(character, _f);
  }

  @override
  void readCharacterContacts(int character, Character3DContacts out) {
    if (!_myCharacter(character)) {
      out.set(0, const [], Float32List(0), const []);
      return;
    }
    var n = jj_character_contacts(character, _contacts, _contactCapacity);
    if (n == _contactCapacity) {
      jj_character_state(character, _characterState);
      final needed = _characterState.ref.contactCount;
      if (needed > _contactCapacity) {
        _drop(_contacts);
        _contactCapacity = needed;
        _contacts = _alloc<JJ_CharacterContact>(_contactCapacity);
        n = jj_character_contacts(character, _contacts, _contactCapacity);
      }
    }
    if (n > _contactFlags.length) {
      _touched = Int64List(2 * n);
      _contactData = Float32List(n * Character3DContacts.stride);
      _contactFlags = Uint8List(n);
    }
    for (var i = 0; i < n; i++) {
      final c = _contacts[i];
      final o = i * Character3DContacts.stride;
      _touched
        ..[2 * i] = c.body
        ..[2 * i + 1] = c.character;
      _contactData
        ..[o] = c.position[0]
        ..[o + 1] = c.position[1]
        ..[o + 2] = c.position[2]
        ..[o + 3] = c.normal[0]
        ..[o + 4] = c.normal[1]
        ..[o + 5] = c.normal[2]
        ..[o + 6] = c.velocity[0]
        ..[o + 7] = c.velocity[1]
        ..[o + 8] = c.velocity[2]
        ..[o + 9] = c.distance;
      _contactFlags[i] = (c.isSensor ? 1 : 0) | (c.hadCollision ? 2 : 0);
    }
    out.set(
      n,
      Int64List.sublistView(_touched, 0, 2 * n),
      Float32List.sublistView(_contactData, 0, n * Character3DContacts.stride),
      Uint8List.sublistView(_contactFlags, 0, n),
    );
  }

  // What readCharacterContacts hands out, refilled on every read.
  Int64List _touched = Int64List(32);
  Float32List _contactData = Float32List(16 * Character3DContacts.stride);
  Uint8List _contactFlags = Uint8List(16);
  final Character3DContacts _contactsRead = Character3DContacts();

  @override
  List<Character3DContact> characterContacts(int character) {
    readCharacterContacts(character, _contactsRead);
    return [
      for (var i = 0; i < _contactsRead.count; i++) _contactsRead.contactAt(i),
    ];
  }

  // ── Stepping ──────────────────────────────────────────────────────────

  @override
  void step(double dt, {int collisionSteps = 1}) {
    if (_gone || dt <= 0) return;
    jj_world_step(handle, dt, collisionSteps);
  }

  void _growPoses(int capacity) {
    // The old buffers go with the last view of them.
    _poseCapacity = capacity;
    _poseIds = calloc<Int64>(capacity);
    _poses = calloc<Float>(capacity * Body3DPoses.stride);
    _poseIdView = _poseIds.asTypedList(
      capacity,
      finalizer: calloc.nativeFree,
      token: _poseIds.cast(),
    );
    _poseView = _poses.asTypedList(
      capacity * Body3DPoses.stride,
      finalizer: calloc.nativeFree,
      token: _poses.cast(),
    );
  }

  @override
  void readMovedBodies(Body3DPoses out) {
    if (_gone) {
      out.set(0, const [], Float32List(0));
      return;
    }
    if (_poseCapacity == 0) _growPoses(256);
    var n = jj_get_active_transforms(handle, _poseIds, _poses, _poseCapacity);
    if (n < 0) {
      _growPoses(-n * 2);
      n = jj_get_active_transforms(handle, _poseIds, _poses, _poseCapacity);
    }
    out.set(
      n,
      Int64List.sublistView(_poseIdView, 0, n),
      Float32List.sublistView(_poseView, 0, n * Body3DPoses.stride),
    );
  }

  @override
  void readBodies(List<int> bodies, Body3DPoses out) {
    if (_gone || bodies.isEmpty) {
      out.set(0, const [], Float32List(0));
      return;
    }
    using((arena) {
      final ids = arena<Int64>(bodies.length);
      for (var i = 0; i < bodies.length; i++) {
        ids[i] = bodies[i];
      }
      final data = arena<Float>(bodies.length * Body3DPoses.stride);
      jj_get_transforms(handle, ids, bodies.length, data);
      out.set(
        bodies.length,
        List.of(bodies),
        Float32List.fromList(
          data.asTypedList(bodies.length * Body3DPoses.stride),
        ),
      );
    });
  }

  static const _eventTypes = [
    Physics3DEventType.contactBegin, // 1
    Physics3DEventType.contactEnd, // 2
    Physics3DEventType.sensorBegin, // 3
    Physics3DEventType.sensorEnd, // 4
    Physics3DEventType.contactStay, // 5
    Physics3DEventType.bodyActivated, // 6
    Physics3DEventType.bodyDeactivated, // 7
  ];

  // What readEvents hands out, grown as needed and refilled on every read.
  Int32List _eventTypeBuf = Int32List(256);
  Int64List _eventBodyBuf = Int64List(512);
  Float32List _eventDataBuf = Float32List(256 * Physics3DEvents.stride);
  final Physics3DEvents _eventsRead = Physics3DEvents();

  /// Room in the event buffers for [needed] events, keeping what is there.
  void _growEventBuffers(int needed) {
    var capacity = _eventTypeBuf.length;
    if (needed <= capacity) return;
    while (capacity < needed) {
      capacity *= 2;
    }
    _eventTypeBuf = Int32List(capacity)..setAll(0, _eventTypeBuf);
    _eventBodyBuf = Int64List(2 * capacity)..setAll(0, _eventBodyBuf);
    _eventDataBuf = Float32List(capacity * Physics3DEvents.stride)
      ..setAll(0, _eventDataBuf);
  }

  @override
  void readEvents(Physics3DEvents out) {
    if (_gone) {
      out.set(0, const [], const [], Float32List(0));
      return;
    }
    var count = 0;
    while (true) {
      final n = jj_drain_events(handle, _events, _eventCapacity);
      _growEventBuffers(count + n);
      for (var i = 0; i < n; i++) {
        final e = _events[i];
        if (e.type < 1 || e.type > _eventTypes.length) continue;
        final o = count * Physics3DEvents.stride;
        _eventTypeBuf[count] = _eventTypes[e.type - 1].index;
        _eventBodyBuf
          ..[2 * count] = e.bodyA
          ..[2 * count + 1] = e.bodyB;
        _eventDataBuf
          ..[o] = e.point[0]
          ..[o + 1] = e.point[1]
          ..[o + 2] = e.point[2]
          ..[o + 3] = e.normal[0]
          ..[o + 4] = e.normal[1]
          ..[o + 5] = e.normal[2]
          ..[o + 6] = e.depth;
        count++;
      }
      if (n < _eventCapacity) break;
      if (_eventCapacity < 8192) {
        _drop(_events);
        _eventCapacity *= 2;
        _events = _alloc<JJ_Event>(_eventCapacity);
      }
    }
    out.set(
      count,
      Int32List.sublistView(_eventTypeBuf, 0, count),
      Int64List.sublistView(_eventBodyBuf, 0, 2 * count),
      Float32List.sublistView(_eventDataBuf, 0, count * Physics3DEvents.stride),
    );
  }

  @override
  List<Physics3DEvent> drainEvents() {
    readEvents(_eventsRead);
    return [for (var i = 0; i < _eventsRead.count; i++) _eventsRead.eventAt(i)];
  }

  // ── Queries ───────────────────────────────────────────────────────────

  int _flags(Query3DFilter filter, bool allHits) =>
      (allHits ? JJ_QUERY_ALL_HITS : 0) |
      (filter.includeSensors ? JJ_QUERY_SENSORS : 0) |
      (filter.includeCharacters ? 0 : JJ_QUERY_NO_CHARACTERS);

  List<Query3DHit> _query(
    int Function(Pointer<JJ_Hit> hits, int capacity) run,
    bool allHits,
  ) {
    var n = run(_hits, _hitCapacity);
    while (allHits && n == _hitCapacity && _hitCapacity < 4096) {
      _drop(_hits);
      _hitCapacity *= 4;
      _hits = _alloc<JJ_Hit>(_hitCapacity);
      n = run(_hits, _hitCapacity);
    }
    return [
      for (var i = 0; i < n; i++)
        () {
          final h = _hits[i];
          return Query3DHit(
            body: h.body,
            fraction: h.fraction,
            point: Vector3(h.point[0], h.point[1], h.point[2]),
            normal: Vector3(h.normal[0], h.normal[1], h.normal[2]),
            depth: h.depth,
          );
        }(),
    ];
  }

  void _pose(Vector3 position, Quaternion rotation) {
    _f
      ..[0] = position.x
      ..[1] = position.y
      ..[2] = position.z
      ..[3] = rotation.x
      ..[4] = rotation.y
      ..[5] = rotation.z
      ..[6] = rotation.w;
  }

  @override
  List<Query3DHit> raycast(
    Vector3 origin,
    Vector3 direction, {
    Query3DFilter filter = Query3DFilter.all,
    bool allHits = false,
  }) {
    if (_gone) return const [];
    _f
      ..[0] = origin.x
      ..[1] = origin.y
      ..[2] = origin.z
      ..[3] = direction.x
      ..[4] = direction.y
      ..[5] = direction.z;
    final flags = _flags(filter, allHits);
    return _query(
      (hits, capacity) => jj_raycast(
        handle,
        _f,
        _f + 3,
        filter.layerMask,
        filter.ignoreBody,
        flags,
        hits,
        allHits ? capacity : 1,
      ),
      allHits,
    );
  }

  @override
  List<Query3DHit> shapeCast(
    Shape3D shape,
    Vector3 position,
    Quaternion rotation,
    Vector3 direction, {
    Query3DFilter filter = Query3DFilter.all,
    bool allHits = false,
  }) {
    if (_gone) return const [];
    final pointer = shapes.borrow(shape);
    if (pointer == null) return const [];
    _pose(position, rotation);
    _f
      ..[7] = direction.x
      ..[8] = direction.y
      ..[9] = direction.z;
    final flags = _flags(filter, allHits);
    return _query(
      (hits, capacity) => jj_shape_cast(
        handle,
        pointer,
        _f,
        _f + 7,
        filter.layerMask,
        filter.ignoreBody,
        flags,
        hits,
        allHits ? capacity : 1,
      ),
      allHits,
    );
  }

  @override
  List<Query3DHit> overlap(
    Shape3D shape,
    Vector3 position,
    Quaternion rotation, {
    Query3DFilter filter = Query3DFilter.all,
  }) {
    if (_gone) return const [];
    final pointer = shapes.borrow(shape);
    if (pointer == null) return const [];
    _pose(position, rotation);
    final flags = _flags(filter, true);
    return _query(
      (hits, capacity) => jj_overlap(
        handle,
        pointer,
        _f,
        filter.layerMask,
        filter.ignoreBody,
        flags,
        hits,
        capacity,
      ),
      true,
    );
  }

  // ── Debugging ─────────────────────────────────────────────────────────

  @override
  Float32List? debugLines({bool joints = false}) {
    if (_gone || !jj_has_debug_renderer()) return null;
    final flags = joints ? 1 : 0;
    final total = jj_debug_lines(handle, flags, nullptr, 0);
    if (total <= 0) return Float32List(0);
    return using((arena) {
      final out = arena<Float>(total * JJ_DEBUG_LINE_FLOATS);
      final n = jj_debug_lines(handle, flags, out, total);
      final count = n < total ? n : total;
      // Copied as bytes: a colour's bits must not pass through a double.
      final bytes = Uint8List.fromList(
        out.cast<Uint8>().asTypedList(count * JJ_DEBUG_LINE_FLOATS * 4),
      );
      return bytes.buffer.asFloat32List();
    });
  }
}
