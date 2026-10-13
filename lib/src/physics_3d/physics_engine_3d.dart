import 'dart:typed_data';

import 'package:just_dart/just_dart.dart';

import 'results_3d.dart';
import 'settings_3d.dart';
import 'shapes_3d.dart';

/// A 3-D physics engine: makes [PhysicsWorld3D]s.
///
/// `PhysicsEngineFactory.create3D` gives the platform's (`JoltPhysicsEngine`,
/// Jolt Physics through just_jolt); `package:just_physics_engine/testing.dart`
/// has a simple one for tests. Whoever holds the worlds never needs to know
/// which.
///
/// Units are metres, seconds and radians; +Y is down, so gravity is
/// usually (0, +9.81, 0).
abstract interface class PhysicsEngine3D {
  /// A short name for logs: `jolt`, `fake`.
  String get name;

  /// Why this engine cannot run here (the web, a native library not in the
  /// build), or null when it can.
  String? get unavailableReason;

  /// Makes a world. Throws a [Physics3DUnsupportedError] saying why when
  /// [unavailableReason] is not null.
  PhysicsWorld3D createWorld(PhysicsWorldSettings3D settings);

  /// Destroys every world this engine made in this process, whoever made
  /// it — for a hot restart, when the Dart objects that owned them are gone
  /// but native memory is not. A world still held reads as disposed.
  void destroyAll();
}

/// 3-D physics cannot run here: the web (no `dart:ffi`), a 32-bit Android
/// ABI, or an app that did not ask for Jolt to be built. [message] says
/// which, and what to change when something can be.
final class Physics3DUnsupportedError extends UnsupportedError {
  /// An error saying [reason].
  Physics3DUnsupportedError(String reason)
    : super('3D physics is not available: $reason');
}

/// One simulation: bodies, joints and characters that see each other.
///
/// Bodies, joints and characters are plain `int` handles. Every method
/// accepts dead ones, and ones from another world, and does nothing with
/// them, so gameplay code never crashes on a body something else destroyed.
/// 0 is never a handle. A handle can come back for a new body once its own
/// was destroyed (Jolt reuses a body's id after 256 bodies have used its
/// slot), so forget a handle when you destroy what it names.
abstract interface class PhysicsWorld3D {
  /// Whether [dispose] was called.
  bool get isDisposed;

  /// Destroys the world and everything in it. Safe to call twice.
  void dispose();

  /// Gravity, m/s². Setting it wakes nothing; bodies at rest stay asleep.
  Vector3 get gravity;
  set gravity(Vector3 value);

  /// How many bodies it holds.
  int get bodyCount;

  // ── Bodies ────────────────────────────────────────────────────────────

  /// Makes a body of [shape]; 0 when it cannot be made (a shape that is
  /// not [Shape3D.isValid], a mesh or a height field on a body that is not
  /// static, a layer outside 0–15, a full world).
  int createBody(Shape3D shape, BodySettings3D settings);

  /// Destroys [body] and every joint holding it. Only bodies [createBody]
  /// made: a character's body ([characterBody]) goes with its character.
  void destroyBody(int body);

  /// Whether [body] is a live body of this world.
  bool isBody(int body);

  /// Teleports [body]. Static bodies are moved this way; kinematic bodies
  /// should use [moveKinematic] so what they touch is carried along.
  void setBodyPose(
    int body,
    Vector3 position,
    Quaternion rotation, {
    bool wake = true,
  });

  /// Moves a kinematic [body] so it arrives at [position] and [rotation]
  /// after [dt] seconds, giving it the velocity that takes.
  void moveKinematic(
    int body,
    Vector3 position,
    Quaternion rotation,
    double dt,
  );

  /// Sets [body]'s velocity (m/s), waking it.
  void setLinearVelocity(int body, Vector3 velocity);

  /// Sets [body]'s spin (rad/s), waking it.
  void setAngularVelocity(int body, Vector3 velocity);

  /// Pushes [body] for the next step (N).
  void addForce(int body, Vector3 force);

  /// Turns [body] for the next step (N·m).
  void addTorque(int body, Vector3 torque);

  /// Changes [body]'s velocity at once (N·s).
  void addImpulse(int body, Vector3 impulse);

  /// Changes [body]'s spin at once (N·m·s).
  void addAngularImpulse(int body, Vector3 impulse);

  /// Sets [body]'s friction.
  void setFriction(int body, double friction);

  /// Sets [body]'s bounciness.
  void setRestitution(int body, double restitution);

  /// Sets how strongly gravity pulls [body].
  void setGravityFactor(int body, double factor);

  /// Moves [body] to another collision layer.
  void setLayer(int body, int layer);

  /// Switches [body]'s continuous collision on or off.
  void setMotionQuality(int body, MotionQuality3D quality);

  /// Wakes [body].
  void wake(int body);

  /// Whether [body] is awake.
  bool isAwake(int body);

  // ── Joints ────────────────────────────────────────────────────────────

  /// Joins [bodyA] to [bodyB], or to the world when [bodyB] is 0; 0 when
  /// it cannot be made. Both must be bodies [createBody] made, not a
  /// character's body.
  int createJoint(int bodyA, int bodyB, JointSettings3D settings);

  /// Destroys [joint].
  void destroyJoint(int joint);

  /// Whether [joint] is a live joint of this world.
  bool isJoint(int joint);

  /// Turns [joint] on or off without destroying it.
  void setJointEnabled(int joint, bool enabled);

  /// Drives a hinge or slider [joint].
  void setJointMotor(int joint, MotorState3D state, double target);

  /// A hinge's angle (radians) or a slider's position (metres) — body A's,
  /// measured from body B (or the world) — the same quantity a position
  /// motor drives; 0 for other joints.
  double jointValue(int joint);

  // ── Characters ────────────────────────────────────────────────────────

  /// Makes a character whose feet are at [position]; 0 when it cannot be
  /// made.
  int createCharacter(CharacterSettings3D settings, Vector3 position);

  /// Destroys [character].
  void destroyCharacter(int character);

  /// Whether [character] is a live character of this world.
  bool isCharacter(int character);

  /// The body that stands in for [character] (see
  /// [CharacterSettings3D.innerBody]), or 0.
  int characterBody(int character);

  /// Moves [character] for one update at [velocity]: it slides along walls,
  /// keeps to the floor ([stepDown], pointing down; zero to turn off) and
  /// climbs steps up to [stepUp] (pointing up; zero to turn off). Writes
  /// where it ended up into [out].
  void moveCharacter(
    int character,
    double dt,
    Vector3 velocity,
    CharacterState3D out, {
    required Vector3 stepDown,
    required Vector3 stepUp,
  });

  /// Reads [character]'s state into [out] without moving it; a dead
  /// character reads as zeros, in the air.
  void characterState(int character, CharacterState3D out);

  /// Teleports [character]'s feet to [position].
  void setCharacterPosition(int character, Vector3 position);

  /// What [character] touched in its last update.
  List<CharacterContact3D> characterContacts(int character);

  // ── Stepping and reading back ─────────────────────────────────────────

  /// Advances the simulation by [dt] seconds, in [collisionSteps] passes
  /// (one per 1/60 s is plenty). Characters move in [moveCharacter], before
  /// it.
  void step(double dt, {int collisionSteps = 1});

  /// The bodies that moved in the last step — awake bodies, and those that
  /// fell asleep since the last read — into [out]. Static bodies and
  /// characters' bodies are left out.
  void readMovedBodies(BodyPoses3D out);

  /// [bodies]' poses, in order, into [out]; dead ones read as zeros.
  void readBodies(List<int> bodies, BodyPoses3D out);

  /// What happened since the last drain, oldest first.
  List<PhysicsEvent3D> drainEvents();

  // ── Queries ───────────────────────────────────────────────────────────

  /// Casts a ray from [origin] along [direction], whose length is the
  /// ray's. The closest hit, or every hit nearest first when [allHits].
  List<QueryHit3D> raycast(
    Vector3 origin,
    Vector3 direction, {
    QueryFilter3D filter = QueryFilter3D.all,
    bool allHits = false,
  });

  /// Sweeps [shape] from [position] and [rotation] along [direction]. The
  /// closest hit, or every hit nearest first when [allHits].
  List<QueryHit3D> shapeCast(
    Shape3D shape,
    Vector3 position,
    Quaternion rotation,
    Vector3 direction, {
    QueryFilter3D filter = QueryFilter3D.all,
    bool allHits = false,
  });

  /// The bodies [shape] at [position] and [rotation] overlaps, one hit
  /// each.
  List<QueryHit3D> overlap(
    Shape3D shape,
    Vector3 position,
    Quaternion rotation, {
    QueryFilter3D filter = QueryFilter3D.all,
  });

  // ── Debugging ─────────────────────────────────────────────────────────

  /// Every body (and joint, when [joints]) as lines, seven floats each:
  /// x1 y1 z1 x2 y2 z2, then a colour (0xAARRGGBB) stored in the float's
  /// bits (read it with `ByteData.view(lines.buffer).getUint32(…, Endian.host)`).
  /// Null when the backend cannot draw (Jolt outside its dev flavour).
  Float32List? debugLines({bool joints = false});
}
