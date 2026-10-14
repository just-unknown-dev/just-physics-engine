import 'package:just_dart/just_dart.dart';

/// How a body moves.
enum BodyType3D {
  /// Never moves (walls, floors, terrain).
  static,

  /// Moved by the game, not by forces; pushes dynamic bodies and carries
  /// characters (moving platforms, doors).
  kinematic,

  /// Moved by the simulation: gravity, forces and collisions.
  dynamic,
}

/// How carefully a moving body checks for collisions.
enum MotionQuality3D {
  /// Once per step. Cheap; a small, fast body can pass through a thin wall.
  discrete,

  /// Sweeps the body along its motion each step (continuous collision), so
  /// fast bodies — bullets, thrown objects — never tunnel.
  linearCast,
}

/// Everything about a new body but its shape.
final class BodySettings3D {
  /// A body with the given settings; the defaults are a dynamic body on
  /// layer 0 that may sleep.
  BodySettings3D({
    Vector3? position,
    Quaternion? rotation,
    Vector3? linearVelocity,
    Vector3? angularVelocity,
    this.type = BodyType3D.dynamic,
    this.motionQuality = MotionQuality3D.discrete,
    this.layer = 0,
    this.isSensor = false,
    this.allowSleeping = true,
    this.lockRotation = false,
    this.friction = 0.5,
    this.restitution = 0.0,
    this.linearDamping = 0.05,
    this.angularDamping = 0.05,
    this.gravityFactor = 1.0,
    this.mass = 0.0,
    this.userData = 0,
  }) : position = position ?? Vector3.zero(),
       rotation = rotation ?? Quaternion.identity(),
       linearVelocity = linearVelocity ?? Vector3.zero(),
       angularVelocity = angularVelocity ?? Vector3.zero();

  /// Where it starts, in metres.
  final Vector3 position;

  /// How it starts turned.
  final Quaternion rotation;

  /// Velocity it starts with (m/s).
  final Vector3 linearVelocity;

  /// Spin it starts with (rad/s).
  final Vector3 angularVelocity;

  /// Static, kinematic or dynamic.
  final BodyType3D type;

  /// Discrete or continuous collision.
  final MotionQuality3D motionQuality;

  /// Collision layer, 0–15 (see [PhysicsWorldSettings3D.layerMasks]).
  final int layer;

  /// Whether it only reports what enters and leaves it, never colliding.
  final bool isSensor;

  /// Whether it may fall asleep once at rest.
  final bool allowSleeping;

  /// Whether it never turns (a crate that only slides, an upright enemy).
  final bool lockRotation;

  /// Surface friction, usually 0–1.
  final double friction;

  /// Bounciness, 0–1.
  final double restitution;

  /// How quickly it loses velocity and spin by itself.
  final double linearDamping, angularDamping;

  /// How strongly gravity pulls it; 0 floats.
  final double gravityFactor;

  /// Mass in kg; 0 works it out from the shape's volume.
  final double mass;

  /// Any number the game wants back with the body (an entity id, say).
  final int userData;
}

/// The kind of a joint.
enum JointKind3D {
  /// Holds two bodies rigidly together.
  fixed,

  /// Holds them at one shared point; they turn freely about it (a ball
  /// joint).
  point,

  /// Lets them turn about one axis (a door, a wheel).
  hinge,

  /// Lets them slide along one axis (a drawer, a piston).
  slider,

  /// Keeps their anchors within a distance range, optionally springy (a
  /// rope, a chain link).
  distance,

  /// Each of the six degrees of freedom is free, limited or fixed.
  sixDof,
}

/// What drives a hinge or slider.
enum MotorState3D {
  /// Nothing.
  off,

  /// Turns or slides at a target speed.
  velocity,

  /// Turns or slides to a target angle or position.
  position,
}

/// What a six-degrees-of-freedom joint lets one axis do.
enum SixDofAxisMode3D {
  /// Moves or turns freely.
  free,

  /// Moves or turns within its limits.
  limited,

  /// Does not move or turn at all.
  fixed,
}

/// How a joint is made. Points and axes are in world space, as the bodies
/// are when it is made.
final class JointSettings3D {
  /// A joint of [kind]; see each field for which kinds read it.
  JointSettings3D({
    required this.kind,
    Vector3? pointA,
    Vector3? pointB,
    Vector3? axis,
    Vector3? normal,
    this.hasLimits = false,
    this.limitMin = 0,
    this.limitMax = 0,
    this.springFrequency = 0,
    this.springDamping = 0,
    this.maxFriction = 0,
    this.motorState = MotorState3D.off,
    this.motorTarget = 0,
    this.motorMaxForce = 0,
    List<SixDofAxisMode3D>? sixDofModes,
    List<double>? sixDofMin,
    List<double>? sixDofMax,
  }) : pointA = pointA ?? Vector3.zero(),
       pointB = pointB ?? Vector3.copy(pointA ?? Vector3.zero()),
       axis = axis ?? Vector3(0, 1, 0),
       normal = normal ?? Vector3(1, 0, 0),
       sixDofModes = sixDofModes ?? List.filled(6, SixDofAxisMode3D.fixed),
       sixDofMin = sixDofMin ?? List.filled(6, 0.0),
       sixDofMax = sixDofMax ?? List.filled(6, 0.0);

  /// Fixed, point, hinge, slider, distance or six-DOF.
  final JointKind3D kind;

  /// Where it holds the first body.
  final Vector3 pointA;

  /// Where it holds the second; the same as [pointA] unless given (a
  /// distance joint's other end).
  final Vector3 pointB;

  /// Hinge: the axis it turns about. Slider: the axis it slides along.
  /// Six-DOF: its X axis.
  final Vector3 axis;

  /// Perpendicular to [axis]: a hinge's zero angle, a six-DOF joint's Y.
  final Vector3 normal;

  /// Whether a hinge, slider or distance joint is limited to
  /// [limitMin]..[limitMax] (radians for a hinge, metres otherwise).
  final bool hasLimits;

  /// The limits; see [hasLimits].
  final double limitMin, limitMax;

  /// A soft limit — a distance joint's spring — in Hz (0 = rigid), and its
  /// damping ratio (1 = critical).
  final double springFrequency, springDamping;

  /// Hinge and slider: friction torque or force while no motor drives it.
  final double maxFriction;

  /// Hinge and slider motor.
  final MotorState3D motorState;

  /// The motor's target speed or position.
  final double motorTarget;

  /// The most torque or force the motor may use; 0 for no limit.
  final double motorMaxForce;

  /// Six-DOF: per axis — translation X, Y, Z, then rotation X, Y, Z.
  final List<SixDofAxisMode3D> sixDofModes;

  /// Six-DOF limits for limited axes, in the same order.
  final List<double> sixDofMin, sixDofMax;
}

/// How a character is made: a capsule standing on its position.
final class CharacterSettings3D {
  /// A character 1.8 m tall and 0.3 m in radius, by default.
  CharacterSettings3D({
    this.radius = 0.3,
    this.halfHeight = 0.6,
    Vector3? up,
    this.maxSlopeAngle = 0.8726646259971648,
    this.mass = 70,
    this.maxStrength = 100,
    this.layer = 0,
    this.innerBody = true,
    this.userData = 0,
  }) : up = up ?? Vector3(0, -1, 0);

  /// Radius of the capsule.
  final double radius;

  /// Half the length of the capsule's straight part; the whole capsule is
  /// `2 * (halfHeight + radius)` tall.
  final double halfHeight;

  /// The world's up: (0, −1, 0) when +Y is down, as in the engine.
  final Vector3 up;

  /// Steepest slope it can walk up, in radians (50° by default).
  final double maxSlopeAngle;

  /// Its mass (kg), and the most force (N) it pushes bodies with.
  final double mass, maxStrength;

  /// Collision layer, 0–15.
  final int layer;

  /// Whether the rest of the world can see it: sensors detect it and rays
  /// hit it. Without it sensors and rays pass it by, though other
  /// characters still collide with it.
  /// The body belongs to the character: it cannot be destroyed or joined
  /// on its own.
  final bool innerBody;

  /// Any number the game wants back with it.
  final int userData;
}

/// How a world is made.
final class PhysicsWorldSettings3D {
  /// A world with [gravity] (0, 9.81, 0) — down, since +Y is down — and
  /// every layer colliding with every other unless [layerMasks] says
  /// otherwise.
  PhysicsWorldSettings3D({
    Vector3? gravity,
    List<int>? layerMasks,
    this.maxBodies = 10240,
    this.reportContactStay = false,
    this.reportActivation = false,
  }) : gravity = gravity ?? Vector3(0, standardGravity, 0),
       layerMasks = List.unmodifiable(
         List.generate(
           16,
           (i) => layerMasks != null && i < layerMasks.length
               ? layerMasks[i] & 0xFFFF
               : 0xFFFF,
         ),
       );

  /// Earth's gravity, m/s².
  static const double standardGravity = 9.81;

  /// Gravity, m/s².
  final Vector3 gravity;

  /// The collision grid, one row per layer: bit `b` of `layerMasks[a]` is
  /// set when layer `a` collides with layer `b`. A pair collides only when
  /// both rows say so.
  final List<int> layerMasks;

  /// The most bodies it can hold.
  final int maxBodies;

  /// Whether touching pairs report a stay event after every step.
  final bool reportContactStay;

  /// Whether bodies report falling asleep and waking up.
  final bool reportActivation;
}
