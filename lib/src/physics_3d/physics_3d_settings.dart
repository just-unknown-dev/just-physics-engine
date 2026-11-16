import 'package:just_dart/just_dart.dart';

import 'shape_3d.dart';

/// How a body moves.
enum Body3DType {
  /// Never moves (walls, floors, terrain).
  static,

  /// Moved by the game, not by forces; pushes dynamic bodies and carries
  /// characters (moving platforms, doors).
  kinematic,

  /// Moved by the simulation: gravity, forces and collisions.
  dynamic,
}

/// How carefully a moving body checks for collisions.
enum Motion3DQuality {
  /// Once per step. Cheap; a small, fast body can pass through a thin wall.
  discrete,

  /// Sweeps the body along its motion each step (continuous collision), so
  /// fast bodies — bullets, thrown objects — never tunnel.
  linearCast,
}

/// Everything about a new body but its shape.
final class Body3DSettings {
  /// A body with the given settings; the defaults are a dynamic body on
  /// layer 0 that may sleep.
  Body3DSettings({
    Vector3? position,
    Quaternion? rotation,
    Vector3? linearVelocity,
    Vector3? angularVelocity,
    this.type = Body3DType.dynamic,
    this.motionQuality = Motion3DQuality.discrete,
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
  final Body3DType type;

  /// Discrete or continuous collision.
  final Motion3DQuality motionQuality;

  /// Collision layer, 0–15 (see [Physics3DWorldSettings.layerMasks]).
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
enum Joint3DKind {
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
enum Motor3DState {
  /// Nothing.
  off,

  /// Turns or slides at a target speed.
  velocity,

  /// Turns or slides to a target angle or position.
  position,
}

/// What a six-degrees-of-freedom joint lets one axis do.
enum SixDofAxis3DMode {
  /// Moves or turns freely.
  free,

  /// Moves or turns within its limits.
  limited,

  /// Does not move or turn at all.
  fixed,
}

/// How a joint is made. Points and axes are in world space, as the bodies
/// are when it is made.
final class Joint3DSettings {
  /// A joint of [kind]; see each field for which kinds read it.
  Joint3DSettings({
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
    this.motorState = Motor3DState.off,
    this.motorTarget = 0,
    this.motorMaxForce = 0,
    List<SixDofAxis3DMode>? sixDofModes,
    List<double>? sixDofMin,
    List<double>? sixDofMax,
  }) : pointA = pointA ?? Vector3.zero(),
       pointB = pointB ?? Vector3.copy(pointA ?? Vector3.zero()),
       axis = axis ?? Vector3(0, 1, 0),
       normal = normal ?? Vector3(1, 0, 0),
       sixDofModes = sixDofModes ?? List.filled(6, SixDofAxis3DMode.fixed),
       sixDofMin = sixDofMin ?? List.filled(6, 0.0),
       sixDofMax = sixDofMax ?? List.filled(6, 0.0);

  /// Fixed, point, hinge, slider, distance or six-DOF.
  final Joint3DKind kind;

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
  final Motor3DState motorState;

  /// The motor's target speed or position.
  final double motorTarget;

  /// The most torque or force the motor may use; 0 for no limit.
  final double motorMaxForce;

  /// Six-DOF: per axis — translation X, Y, Z, then rotation X, Y, Z.
  final List<SixDofAxis3DMode> sixDofModes;

  /// Six-DOF limits for limited axes, in the same order.
  final List<double> sixDofMin, sixDofMax;
}

/// How a character is made: a capsule standing on its position.
final class Character3DSettings {
  /// A character 1.8 m tall and 0.3 m in radius, by default.
  Character3DSettings({
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
final class Physics3DWorldSettings {
  /// A world with [gravity] (0, 9.81, 0) — down, since +Y is down — and
  /// every layer colliding with every other unless [layerMasks] says
  /// otherwise.
  Physics3DWorldSettings({
    Vector3? gravity,
    List<int>? layerMasks,
    this.maxBodies = 10240,
    this.reportContactStay = false,
    this.reportActivation = false,
  }) : gravity = gravity ?? Vector3(0, standardGravity, 0),
       layerMasks = normalizeLayerMasks(layerMasks);

  /// [masks] as a world takes them: 16 rows, a missing row `0xFFFF` (that
  /// layer collides with every layer), bits past the 16 layers dropped.
  /// Unmodifiable.
  static List<int> normalizeLayerMasks(List<int>? masks) => List.unmodifiable(
    List.generate(
      16,
      (i) => masks != null && i < masks.length ? masks[i] & 0xFFFF : 0xFFFF,
    ),
  );

  /// Earth's gravity, m/s².
  static const double standardGravity = 9.81;

  /// Gravity, m/s².
  final Vector3 gravity;

  /// The collision grid, one row per layer: bit `b` of `layerMasks[a]` is
  /// set when layer `a` collides with layer `b`. A pair collides only when
  /// both rows say so. A live world's grid can change:
  /// [Physics3DWorld.setLayerMasks].
  final List<int> layerMasks;

  /// The most bodies it can hold.
  final int maxBodies;

  /// Whether touching pairs report a stay event after every step.
  final bool reportContactStay;

  /// Whether bodies report falling asleep and waking up.
  final bool reportActivation;
}

/// One part of a ragdoll ([Physics3DWorld.createRagdoll]): a body, and how
/// it hangs from the part before it. Places, pivots and axes are in the
/// world, as the ragdoll is made.
final class Ragdoll3DPart {
  /// A part of [shape] at [position], turned by [rotation], hanging from
  /// part [parent] (-1 for the root) by a swing-twist joint at [pivot].
  Ragdoll3DPart({
    required this.shape,
    this.parent = -1,
    Vector3? position,
    Quaternion? rotation,
    this.mass = 0,
    Vector3? pivot,
    Vector3? twistAxis,
    Vector3? planeAxis,
    this.normalHalfCone = 0.7853981633974483,
    this.planeHalfCone = 0.7853981633974483,
    this.twistMin = -0.39269908169872414,
    this.twistMax = 0.39269908169872414,
    this.maxFrictionTorque = 0,
  }) : position = position ?? Vector3.zero(),
       rotation = rotation ?? Quaternion.identity(),
       pivot = pivot ?? Vector3.zero(),
       twistAxis = twistAxis ?? Vector3(0, 1, 0),
       planeAxis = planeAxis ?? Vector3(1, 0, 0);

  /// Its shape, in its own space.
  final Shape3D shape;

  /// The part it hangs from, by index (lower than its own); -1 for the
  /// root.
  final int parent;

  /// Where it starts.
  final Vector3 position;

  /// How it starts turned.
  final Quaternion rotation;

  /// Mass in kg; 0 or less works it out from the shape.
  final double mass;

  /// Where it is joined to its parent.
  final Vector3 pivot;

  /// The joint's twist axis: along the part, away from its parent.
  final Vector3 twistAxis;

  /// Square to [twistAxis]: the axis the plane cone opens about.
  final Vector3 planeAxis;

  /// How far it swings one way, radians: half the cone about the normal
  /// of [planeAxis] and [twistAxis].
  final double normalHalfCone;

  /// How far it swings the other way, radians: half the cone about
  /// [planeAxis].
  final double planeHalfCone;

  /// How far it twists about [twistAxis], radians: [twistMin] is at or
  /// below 0, [twistMax] at or above.
  final double twistMin, twistMax;

  /// Friction torque in its joint, N m: how stiffly it hangs.
  final double maxFrictionTorque;
}

/// How every part of a ragdoll is made.
final class Ragdoll3DSettings {
  /// Settings for a ragdoll's parts.
  const Ragdoll3DSettings({
    this.layer = 0,
    this.friction = 0.6,
    this.restitution = 0,
    this.linearDamping = 0.05,
    this.angularDamping = 0.05,
    this.gravityFactor = 1,
    this.motionQuality = Motion3DQuality.discrete,
    this.userData = 0,
  });

  /// Collision layer, 0-15.
  final int layer;

  /// As a body's ([Body3DSettings]).
  final double friction, restitution, linearDamping, angularDamping;

  /// How much of the world's gravity pulls it.
  final double gravityFactor;

  /// Whether fast parts are cast, not stepped (continuous collision).
  final Motion3DQuality motionQuality;

  /// Kept on every part's body.
  final int userData;
}
