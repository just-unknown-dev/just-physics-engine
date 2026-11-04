import 'dart:typed_data';

import 'package:just_dart/just_dart.dart';

/// Bodies' poses after a step, packed [stride] floats each: position (3),
/// rotation quaternion x y z w (4), linear velocity (3), angular velocity
/// (3), awake (1 or 0).
///
/// A world owns one and refills it on every read; its contents are good
/// until the world's next read, into any buffer (a backend may hand out
/// views of its own memory). They stay readable after the world is gone.
final class Body3DPoses {
  /// An empty buffer.
  Body3DPoses();

  /// Floats per body.
  static const int stride = 14;

  /// How many bodies were written.
  int count = 0;

  /// The bodies, [count] of them.
  List<int> bodies = const [];

  /// [count] × [stride] floats.
  Float32List data = Float32List(0);

  /// Sets [count], [bodies] and [data] (a backend's own buffers).
  void set(int count, List<int> bodies, Float32List data) {
    this.count = count;
    this.bodies = bodies;
    this.data = data;
  }

  /// The [i]th body's position, written into [out].
  void positionInto(int i, Vector3 out) {
    final o = i * stride;
    out.setValues(data[o], data[o + 1], data[o + 2]);
  }

  /// The [i]th body's rotation, written into [out].
  void rotationInto(int i, Quaternion out) {
    final o = i * stride;
    out.setValues(data[o + 3], data[o + 4], data[o + 5], data[o + 6]);
  }

  /// The [i]th body's linear velocity, written into [out].
  void linearVelocityInto(int i, Vector3 out) {
    final o = i * stride;
    out.setValues(data[o + 7], data[o + 8], data[o + 9]);
  }

  /// The [i]th body's angular velocity, written into [out].
  void angularVelocityInto(int i, Vector3 out) {
    final o = i * stride;
    out.setValues(data[o + 10], data[o + 11], data[o + 12]);
  }

  /// Whether the [i]th body is awake.
  bool isAwake(int i) => data[i * stride + 13] != 0;
}

/// What happened between bodies during a step.
enum Physics3DEventType {
  /// Two bodies started touching.
  contactBegin,

  /// They are still touching (only when the world reports stays).
  contactStay,

  /// They stopped touching (or one of them was destroyed).
  contactEnd,

  /// A body entered a sensor.
  sensorBegin,

  /// A body left a sensor.
  sensorEnd,

  /// A body woke up (only when the world reports activation).
  bodyActivated,

  /// A body fell asleep (only when the world reports activation).
  bodyDeactivated,
}

/// One event from a step. Begin and stay events carry a contact point, the
/// normal pointing from [bodyA] to [bodyB] (zero for sensors) and how deep
/// they overlap.
final class Physics3DEvent {
  /// An event of [type] between [bodyA] and [bodyB].
  Physics3DEvent(
    this.type,
    this.bodyA,
    this.bodyB, {
    Vector3? point,
    Vector3? normal,
    this.depth = 0,
  }) : point = point ?? Vector3.zero(),
       normal = normal ?? Vector3.zero();

  /// What happened.
  final Physics3DEventType type;

  /// The first body.
  final int bodyA;

  /// The second body; 0 for activation events.
  final int bodyB;

  /// Where they touch.
  final Vector3 point;

  /// From A to B.
  final Vector3 normal;

  /// How far they overlap.
  final double depth;

  @override
  String toString() => 'Physics3DEvent(${type.name}, $bodyA, $bodyB)';
}

/// What a query looks at.
final class Query3DFilter {
  /// Every layer, no body ignored, sensors left out, characters in.
  const Query3DFilter({
    this.layerMask = 0xFFFF,
    this.ignoreBody = 0,
    this.includeSensors = false,
    this.includeCharacters = true,
  });

  /// Everything but sensors.
  static const Query3DFilter all = Query3DFilter();

  /// Layers to hit, one bit per layer.
  final int layerMask;

  /// A body to pass over (the one asking, say); 0 for none.
  final int ignoreBody;

  /// Whether sensors can be hit.
  final bool includeSensors;

  /// Whether characters' bodies can be hit.
  final bool includeCharacters;
}

/// A query's hit.
final class Query3DHit {
  /// A hit on [body].
  Query3DHit({
    required this.body,
    required this.fraction,
    required this.point,
    required this.normal,
    this.depth = 0,
  });

  /// The body hit.
  final int body;

  /// How far along the ray or sweep, 0–1; 0 for an overlap.
  final double fraction;

  /// Where.
  final Vector3 point;

  /// The surface's normal there, facing the query.
  final Vector3 normal;

  /// A sweep that starts inside or an overlap: how deep.
  final double depth;

  @override
  String toString() => 'Query3DHit($body at $point, fraction $fraction)';
}

/// Whether a character stands on something.
enum Ground3DState {
  /// On ground it can stand on.
  onGround,

  /// On a slope too steep to stand on: it slides.
  onSteepGround,

  /// Touching something that does not hold it up.
  notSupported,

  /// Touching nothing below it.
  inAir,
}

/// A character after a step. A world refills one it is given.
final class Character3DState {
  /// A character in the air at the origin.
  Character3DState();

  /// Where its feet are.
  final Vector3 position = Vector3.zero();

  /// How fast it moved.
  final Vector3 velocity = Vector3.zero();

  /// What it stands on.
  Ground3DState groundState = Ground3DState.inAir;

  /// The ground's normal (pointing up, out of the ground).
  final Vector3 groundNormal = Vector3.zero();

  /// How fast the ground under it moves (a platform).
  final Vector3 groundVelocity = Vector3.zero();

  /// Where it touches the ground.
  final Vector3 groundPosition = Vector3.zero();

  /// The body it stands on, or 0.
  int groundBody = 0;

  /// How many contacts it has.
  int contactCount = 0;

  /// Whether it stands on walkable ground.
  bool get isGrounded => groundState == Ground3DState.onGround;
}

/// Something a character touches.
final class Character3DContact {
  /// A contact with [body] or [character].
  Character3DContact({
    required this.body,
    required this.character,
    required this.position,
    required this.normal,
    required this.velocity,
    required this.distance,
    required this.isSensor,
    required this.hadCollision,
  });

  /// The body touched, or 0.
  final int body;

  /// The character touched, or 0.
  final int character;

  /// Where.
  final Vector3 position;

  /// Pointing towards the character.
  final Vector3 normal;

  /// How fast the point touched moves.
  final Vector3 velocity;

  /// Zero or less when touching; more is a contact it expects.
  final double distance;

  /// Whether the body touched is a sensor.
  final bool isSensor;

  /// Whether the character actually ran into it.
  final bool hadCollision;
}
