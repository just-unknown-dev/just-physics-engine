part of 'fake_physics_3d_engine.dart';

/// A body in a [FakePhysics3DWorld], open for tests to look at.
final class FakeBody3D {
  FakeBody3D._(this.handle, this.shape, Body3DSettings s)
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
      rotation = _unitOf(s.rotation),
      linearVelocity = Vector3.copy(s.linearVelocity),
      angularVelocity = Vector3.copy(s.angularVelocity),
      awake = s.type != Body3DType.static {
    _turn();
  }

  /// Recomputes [localMin] and [localMax] for [rotation].
  void _turn() {
    final (min, max) = _boundsOf(shape);
    final (lo, hi) = _turned(min, max, rotation);
    localMin.setFrom(lo);
    localMax.setFrom(hi);
  }

  /// Its handle.
  final int handle;

  /// Its shape.
  final Shape3D shape;

  /// How it moves.
  Body3DType type;

  /// Its layer.
  int layer;

  /// Whether it is a sensor.
  final bool isSensor;

  /// Whether it may sleep.
  final bool allowSleeping;

  /// Its motion quality.
  Motion3DQuality motionQuality;

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

  /// Its shape's bounds, turned as it is, about its position.
  final Vector3 localMin = Vector3.zero(), localMax = Vector3.zero();

  int _restSteps = 0;
  Vector3? _kinematicTarget;

  /// Its box's least x, where it is now.
  double get minX => position.x + localMin.x;

  /// Its box's least y (its top: +Y is down).
  double get minY => position.y + localMin.y;

  /// Its box's least z.
  double get minZ => position.z + localMin.z;

  /// Its box's greatest x.
  double get maxX => position.x + localMax.x;

  /// Its box's greatest y (its bottom).
  double get maxY => position.y + localMax.y;

  /// Its box's greatest z.
  double get maxZ => position.z + localMax.z;

  /// What a character walks into: not sensors, not characters' bodies.
  bool get _isSolid => !isSensor && character == 0;

  /// Jolt's kind of body: 0 static, 1 moving, 2 sensor, 3 a character's.
  int get _kind => character != 0
      ? 3
      : isSensor
      ? 2
      : type == Body3DType.static
      ? 0
      : 1;
}

/// A joint in a [FakePhysics3DWorld]: recorded, never simulated.
final class FakeJoint3D {
  FakeJoint3D._(this.handle, this.bodyA, this.bodyB, this.settings)
    : motorState = settings.motorState,
      motorTarget = settings.motorTarget;

  /// Its handle.
  final int handle;

  /// The bodies; [bodyB] is 0 for the world.
  final int bodyA, bodyB;

  /// What it was made with.
  final Joint3DSettings settings;

  /// Whether it is on.
  bool enabled = true;

  /// Its motor.
  Motor3DState motorState;

  /// Its motor's target.
  double motorTarget;
}

/// A character in a [FakePhysics3DWorld].
final class FakeCharacter3D {
  FakeCharacter3D._(this.handle, this.settings, Vector3 position)
    : position = Vector3.copy(position);

  /// Its handle.
  final int handle;

  /// What it was made with.
  final Character3DSettings settings;

  /// Where its feet are.
  final Vector3 position;

  /// Its velocity after the last move.
  final Vector3 velocity = Vector3.zero();

  /// What it stood on after the last move.
  Ground3DState groundState = Ground3DState.inAir;

  /// The body it stood on, or 0.
  int groundBody = 0;

  /// Its stand-in body, or 0.
  int innerBody = 0;

  /// How often it was moved.
  int moves = 0;

  double get _halfHeight => settings.halfHeight + settings.radius;
}
