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

/// Events from steps, packed: [count] events, each a type, two bodies and
/// [stride] floats — the contact point (3), the normal from the first body
/// to the second (3) and how deep they overlap (1) — with no object per
/// event ([Physics3DWorld.readEvents]).
///
/// A world refills it on every read, into its own buffers; the contents are
/// good until the world's next read, and stay readable after the world is
/// gone.
final class Physics3DEvents {
  /// An empty buffer.
  Physics3DEvents();

  /// Floats per event.
  static const int stride = 7;

  /// How many events were written.
  int count = 0;

  /// Each event's [Physics3DEventType.index], [count] of them.
  List<int> types = const [];

  /// The two bodies of each event, first then second: 2 × [count].
  List<int> bodies = const [];

  /// [count] × [stride] floats.
  Float32List data = Float32List(0);

  /// Sets [count], [types], [bodies] and [data] (a backend's own buffers).
  void set(int count, List<int> types, List<int> bodies, Float32List data) {
    this.count = count;
    this.types = types;
    this.bodies = bodies;
    this.data = data;
  }

  /// What the [i]th event is.
  Physics3DEventType typeOf(int i) => Physics3DEventType.values[types[i]];

  /// The [i]th event's first body.
  int bodyAOf(int i) => bodies[2 * i];

  /// The [i]th event's second body; 0 for activation events.
  int bodyBOf(int i) => bodies[2 * i + 1];

  /// Where the [i]th event's bodies touch, written into [out].
  void pointInto(int i, Vector3 out) {
    final o = i * stride;
    out.setValues(data[o], data[o + 1], data[o + 2]);
  }

  /// The [i]th event's normal, from its first body to its second (zero for
  /// sensors), written into [out].
  void normalInto(int i, Vector3 out) {
    final o = i * stride;
    out.setValues(data[o + 3], data[o + 4], data[o + 5]);
  }

  /// How far the [i]th event's bodies overlap.
  double depthOf(int i) => data[i * stride + 6];

  /// The [i]th event as an object.
  Physics3DEvent eventAt(int i) {
    final o = i * stride;
    return Physics3DEvent(
      typeOf(i),
      bodyAOf(i),
      bodyBOf(i),
      point: Vector3(data[o], data[o + 1], data[o + 2]),
      normal: Vector3(data[o + 3], data[o + 4], data[o + 5]),
      depth: data[o + 6],
    );
  }
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

/// What a character touched, packed: [count] contacts, each the body and the
/// character touched (one of them 0), [stride] floats — position (3), normal
/// toward the character (3), velocity of the point touched (3), distance
/// (1) — and two flags, with no object per contact
/// ([Physics3DWorld.readCharacterContacts]).
///
/// A world refills it on every read, into its own buffers; the contents are
/// good until the world's next read.
final class Character3DContacts {
  /// An empty buffer.
  Character3DContacts();

  /// Floats per contact.
  static const int stride = 10;

  /// How many contacts were written.
  int count = 0;

  /// The body and the character of each contact: 2 × [count].
  List<int> touched = const [];

  /// [count] × [stride] floats.
  Float32List data = Float32List(0);

  /// Each contact's flags: bit 0 a sensor, bit 1 an actual collision.
  List<int> flags = const [];

  /// Sets [count], [touched], [data] and [flags] (a backend's own buffers).
  void set(int count, List<int> touched, Float32List data, List<int> flags) {
    this.count = count;
    this.touched = touched;
    this.data = data;
    this.flags = flags;
  }

  /// The body the [i]th contact touched, or 0.
  int bodyOf(int i) => touched[2 * i];

  /// The character the [i]th contact touched, or 0.
  int characterOf(int i) => touched[2 * i + 1];

  /// Where the [i]th contact is, written into [out].
  void positionInto(int i, Vector3 out) {
    final o = i * stride;
    out.setValues(data[o], data[o + 1], data[o + 2]);
  }

  /// The [i]th contact's normal, toward the character, written into [out].
  void normalInto(int i, Vector3 out) {
    final o = i * stride;
    out.setValues(data[o + 3], data[o + 4], data[o + 5]);
  }

  /// How fast the point the [i]th contact touched moves, written into [out].
  void velocityInto(int i, Vector3 out) {
    final o = i * stride;
    out.setValues(data[o + 6], data[o + 7], data[o + 8]);
  }

  /// The [i]th contact's distance: zero or less when touching.
  double distanceOf(int i) => data[i * stride + 9];

  /// Whether the [i]th contact's body is a sensor.
  bool isSensor(int i) => flags[i] & 1 != 0;

  /// Whether the character actually ran into the [i]th contact's body.
  bool hadCollision(int i) => flags[i] & 2 != 0;

  /// The [i]th contact as an object.
  Character3DContact contactAt(int i) {
    final o = i * stride;
    return Character3DContact(
      body: bodyOf(i),
      character: characterOf(i),
      position: Vector3(data[o], data[o + 1], data[o + 2]),
      normal: Vector3(data[o + 3], data[o + 4], data[o + 5]),
      velocity: Vector3(data[o + 6], data[o + 7], data[o + 8]),
      distance: data[o + 9],
      isSensor: isSensor(i),
      hadCollision: hadCollision(i),
    );
  }
}
