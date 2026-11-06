part of 'physics_2d_engine.dart';

/// The kind of a 2-D joint.
enum Joint2DKind {
  /// Keeps two anchor points a range of distances apart: a rope, a rod or
  /// a spring ([DistanceJoint2D]).
  distance,

  /// Pulls one body toward a target point, for dragging ([MouseJoint2D]).
  mouse,

  /// Holds two bodies together as they are ([WeldJoint2D]).
  weld,

  /// A hinge about a shared anchor ([RevoluteJoint2D]).
  revolute,

  /// A slider along an axis ([PrismaticJoint2D]).
  prismatic,

  /// A wheel on a sprung axis, free to spin ([WheelJoint2D]).
  wheel,
}

/// A joint between bodies, on either backend.
///
/// Make one with the engine's `add*Joint` methods, which give the pure-Dart
/// joints below or, on the native backend, a `Box2DJoint`; configure it
/// through this class's setters, which both kinds take in the same units,
/// so code never needs to know which it has.
///
/// The pure-Dart engine applies each joint once per sub-step, after
/// collisions. A joint moves its bodies back toward what it allows (half the
/// error each step) and takes away the relative velocity it forbids, so
/// gravity cannot pile up against it. The engine is translational — shapes
/// do not turn in collisions — so joints move bodies, and set the angle a
/// body is drawn at where they decide it (a hinged body turns as it swings).
/// The native backend solves its own joints.
abstract class Joint2D {
  /// A joint of [kind]. [collideConnected] says whether its two bodies
  /// collide with each other (see [collideConnected]).
  Joint2D(this.kind, {this.collideConnected = false});

  /// Which kind of joint this is.
  final Joint2DKind kind;

  /// Whether the two bodies it holds collide with each other. False (the
  /// default, as Box2D's): they pass through each other, on both backends,
  /// and a contact they have ends as the joint is made; sensors still see
  /// them. Fixed when the joint is made.
  final bool collideConnected;

  /// The bodies the joint holds; empty where it does not say.
  List<Physics2DBody> get bodies => const [];

  /// Apply the constraint for one physics step (the pure-Dart engine's
  /// solver; a native joint does nothing here).
  void applyConstraint(double dt);

  /// Limits the joint's travel to [lower]..[upper] ([enable] true) or frees
  /// it ([enable] false): a hinge's angle in radians, counted from the angle
  /// it was made at; a slider's travel along its axis in units, from where
  /// it was made; a distance joint's shortest and longest length (always on:
  /// [enable] is not read). Other kinds take no limits.
  void setLimits(double lower, double upper, {bool enable = true}) {}

  /// Drives the joint at [speed] with up to [maxForceOrTorque] ([enable]
  /// true), or turns the motor off: a hinge's or a wheel's spin in radians
  /// per second with a torque, a slider's travel in units per second with a
  /// force. Other kinds have no motor.
  void setMotor(double speed, double maxForceOrTorque, {bool enable = true}) {}

  /// A spring: [hertz] is how fast it swings, [dampingRatio] how soon it
  /// settles (0 never, 1 without overshoot); 0 Hz takes it away. A distance
  /// joint's pulls toward the middle of its range; a wheel's holds the
  /// wheel along its axis where it was made. Other kinds take no spring.
  void setSpring(double hertz, double dampingRatio) {}

  /// The force the joint applied in the last step, in world space. The
  /// pure-Dart joints do not measure it: zero.
  Offset get reactionForce => Offset.zero;

  /// The torque the joint applied in the last step. The pure-Dart joints do
  /// not measure it: zero.
  double get reactionTorque => 0.0;

  /// The pair whose collisions this joint switches off, or null.
  BodyPair? get _quietPair {
    if (collideConnected) return null;
    final held = bodies;
    if (held.length < 2 || identical(held[0], held[1])) return null;
    return BodyPair(held[0], held[1]);
  }
}

/// How much of a joint's position error the pure-Dart engine takes away in
/// a step.
const double _jointCorrection = 0.5;

/// How close to a limit counts as at it: Box2D's linear slop (half a unit).
const double _jointSlop = 0.5;

/// Whether both of a joint's bodies take part in the simulation.
bool _jointLive(Physics2DBody a, Physics2DBody b) => a.isActive && b.isActive;

/// A spring of [hertz] and [ratio] — what Box2D takes, and what [Joint2D]'s
/// setters take on both backends — as the pure-Dart joints apply one: the
/// pull per unit stretched and per unit of speed, both per unit of the
/// bodies' mass. Worked out as Box2D's soft step does for the engine's
/// fixed step, so a stiff spring holds rather than blowing up, and both
/// backends settle alike.
(double, double) _jointSpring(double hertz, double ratio) {
  if (!(hertz > 0)) return (0.0, 0.0);
  const h = Physics2DEngine.fixedDeltaTime;
  final omega = 2 * math.pi * hertz;
  final a1 = 2 * ratio + h * omega;
  final a2 = h * omega * a1;
  final massScale = a2 / (1 + a2);
  final biasRate = omega / a1;
  return (massScale * biasRate / h, massScale / h);
}

/// [b] moved away from [a] along ([nx], [ny]) by [amount] (negative: toward
/// it), shared by their solver masses.
void _jointSeparate(
  Physics2DBody a,
  Physics2DBody b,
  double nx,
  double ny,
  double amount,
) {
  final ia = a.solverInverseMass, ib = b.solverInverseMass;
  final k = ia + ib;
  if (k == 0) return;
  final s = amount / k;
  a.position.x -= nx * s * ia;
  a.position.y -= ny * s * ia;
  b.position.x += nx * s * ib;
  b.position.y += ny * s * ib;
}

/// An impulse [lambda] along ([nx], [ny]): [b] pushed along it, [a] against.
void _jointPush(
  Physics2DBody a,
  Physics2DBody b,
  double nx,
  double ny,
  double lambda,
) {
  final ia = a.solverInverseMass, ib = b.solverInverseMass;
  a.velocity.x -= nx * lambda * ia;
  a.velocity.y -= ny * lambda * ia;
  b.velocity.x += nx * lambda * ib;
  b.velocity.y += ny * lambda * ib;
}

/// Takes away [b]'s velocity relative to ([pvx], [pvy]) — the velocity of the
/// point of [a] it is held to — shared by their solver masses.
void _jointHold(Physics2DBody a, Physics2DBody b, double pvx, double pvy) {
  final ia = a.solverInverseMass, ib = b.solverInverseMass;
  final k = ia + ib;
  if (k == 0) return;
  final rvx = b.velocity.x - pvx, rvy = b.velocity.y - pvy;
  a.velocity.x += rvx * ia / k;
  a.velocity.y += rvy * ia / k;
  b.velocity.x -= rvx * ib / k;
  b.velocity.y -= rvy * ib / k;
}

/// Inverse rotational inertia as the joints use it: none for a body that is
/// not dynamic, or cannot turn.
double _jointInverseInertia(Physics2DBody b) =>
    b.isDynamic && !b.fixedRotation ? b.inverseInertia : 0.0;

/// A spin impulse driving `ωB − ωA` toward [speed], at most [maxTorque] × dt.
void _jointSpinMotor(
  Physics2DBody a,
  Physics2DBody b,
  double speed,
  double maxTorque,
  double dt,
) {
  final ia = _jointInverseInertia(a), ib = _jointInverseInertia(b);
  final k = ia + ib;
  if (k == 0) return;
  final most = maxTorque * dt;
  final impulse = ((speed - (b.angularVelocity - a.angularVelocity)) / k).clamp(
    -most,
    most,
  );
  a.angularVelocity -= impulse * ia;
  b.angularVelocity += impulse * ib;
}

/// ([x], [y]) turned by [angle].
(double, double) _jointTurn(double x, double y, double angle) {
  final c = math.cos(angle), s = math.sin(angle);
  return (c * x - s * y, s * x + c * y);
}

/// [body]'s point [local] (in its own frame) in the world.
(double, double) _jointPoint(Physics2DBody body, double lx, double ly) {
  final (x, y) = _jointTurn(lx, ly, body.angle);
  return (body.position.x + x, body.position.y + y);
}

/// [world] in [body]'s own frame.
(double, double) _jointLocal(Physics2DBody body, Offset world) => _jointTurn(
  world.dx - body.position.x,
  world.dy - body.position.y,
  -body.angle,
);

// ── Distance Joint ─────────────────────────────────────────────────────────

/// Keeps an anchor point on bodyA and one on bodyB [minLength] to
/// [maxLength] apart, as Box2D's distance joint.
///
/// Equal lengths make a rigid rod; a range is a rope, free inside it and
/// stopped at its ends. A spring ([setSpring]) pulls toward the middle of
/// the range, and a range still stops it at its ends; with equal lengths
/// there is only the spring. The anchors turn with their bodies, but the
/// pure-Dart engine's joints are translational: the rope moves the bodies,
/// it does not turn them.
class DistanceJoint2D extends Joint2D {
  /// A distance joint from [anchorA] on [bodyA] to [anchorB] on [bodyB]
  /// (world points; their centres when left out), [length] long or
  /// [minLength] to [maxLength], with a spring of [springHertz] and
  /// [dampingRatio] when [springHertz] is above 0.
  factory DistanceJoint2D({
    required Physics2DBody bodyA,
    required Physics2DBody bodyB,
    Offset? anchorA,
    Offset? anchorB,
    required double length,
    double? minLength,
    double? maxLength,
    double springHertz = 0.0,
    double dampingRatio = 0.3,
    bool collideConnected = false,
  }) {
    final (ax, ay) = anchorA == null ? (0.0, 0.0) : _jointLocal(bodyA, anchorA);
    final (bx, by) = anchorB == null ? (0.0, 0.0) : _jointLocal(bodyB, anchorB);
    final low = minLength ?? length, high = maxLength ?? length;
    return DistanceJoint2D._(
      bodyA,
      bodyB,
      ax,
      ay,
      bx,
      by,
      math.min(low, high),
      math.max(low, high),
      collideConnected,
    ).._setSpring(springHertz, dampingRatio);
  }

  DistanceJoint2D._(
    this.bodyA,
    this.bodyB,
    this._anchorAX,
    this._anchorAY,
    this._anchorBX,
    this._anchorBY,
    this._minLength,
    this._maxLength,
    bool collideConnected,
  ) : super(Joint2DKind.distance, collideConnected: collideConnected);

  /// The first body.
  final Physics2DBody bodyA;

  /// The second body.
  final Physics2DBody bodyB;

  // The anchors in their bodies' frames.
  final double _anchorAX;
  final double _anchorAY;
  final double _anchorBX;
  final double _anchorBY;

  double _minLength;
  double _maxLength;
  double _springHertz = 0.0;
  double _dampingRatio = 0.0;

  // The spring as the solver applies it (see _jointSpring).
  double _pull = 0.0;
  double _drag = 0.0;

  @override
  List<Physics2DBody> get bodies => [bodyA, bodyB];

  /// The shortest the joint may be.
  double get minLength => _minLength;

  /// The longest the joint may be.
  double get maxLength => _maxLength;

  /// The spring's frequency in hertz; 0 for none.
  double get springHertz => _springHertz;

  /// The spring's damping ratio.
  double get dampingRatio => _dampingRatio;

  /// The anchors' distance now.
  double get currentLength {
    final (ax, ay) = _jointPoint(bodyA, _anchorAX, _anchorAY);
    final (bx, by) = _jointPoint(bodyB, _anchorBX, _anchorBY);
    return math.sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
  }

  @override
  void setLimits(double lower, double upper, {bool enable = true}) {
    _minLength = math.min(lower, upper);
    _maxLength = math.max(lower, upper);
  }

  @override
  void setSpring(double hertz, double dampingRatio) =>
      _setSpring(hertz, dampingRatio);

  void _setSpring(double hertz, double ratio) {
    _springHertz = hertz > 0 ? hertz : 0.0;
    _dampingRatio = ratio;
    final (pull, drag) = _jointSpring(_springHertz, ratio);
    _pull = pull;
    _drag = drag;
  }

  @override
  void applyConstraint(double dt) {
    if (!_jointLive(bodyA, bodyB)) return;
    final k = bodyA.solverInverseMass + bodyB.solverInverseMass;
    if (k == 0) return;
    final (ax, ay) = _jointPoint(bodyA, _anchorAX, _anchorAY);
    final (bx, by) = _jointPoint(bodyB, _anchorBX, _anchorBY);
    final dx = bx - ax, dy = by - ay;
    final dist = math.sqrt(dx * dx + dy * dy);
    if (dist < 1e-8) return;
    final nx = dx / dist, ny = dy / dist;
    var relVel =
        (bodyB.velocity.x - bodyA.velocity.x) * nx +
        (bodyB.velocity.y - bodyA.velocity.y) * ny;

    if (_pull > 0) {
      // Toward the middle of the range.
      final stretch = dist - (_minLength + _maxLength) / 2;
      final lambda = -(_pull * stretch + _drag * relVel) * dt / k;
      _jointPush(bodyA, bodyB, nx, ny, lambda);
      relVel += lambda * k;
      // Equal lengths: only the spring.
      if (_minLength == _maxLength) return;
    }

    // How far outside the range: positive too far apart, negative too close.
    final violation = dist < _minLength
        ? dist - _minLength
        : dist > _maxLength
        ? dist - _maxLength
        : 0.0;
    if (violation != 0) {
      _jointSeparate(bodyA, bodyB, nx, ny, -violation * _jointCorrection);
    }
    // At an end, the speed that would carry it further out goes.
    final outward = dist >= _maxLength - _jointSlop && relVel > 0;
    final inward = dist <= _minLength + _jointSlop && relVel < 0;
    if (outward || inward) _jointPush(bodyA, bodyB, nx, ny, -relVel / k);
  }
}

// ── Mouse Joint ────────────────────────────────────────────────────────────

/// Pulls a body toward a target point using a spring force.
///
/// Typical use: dragging bodies with a pointer. It is a Dart joint on both
/// backends: on the native one its force goes into the native body. Its
/// spring is its own: [stiffness] is the pull per unit of distance and
/// [damping] the drag per unit of speed, both per unit of mass, the force
/// at most [maxForce].
class MouseJoint2D extends Joint2D {
  /// A joint pulling [body] toward [target].
  MouseJoint2D({
    required this.body,
    required Vector2 target,
    this.stiffness = 500.0,
    this.damping = 0.7,
    this.maxForce = 50000.0,
  }) : target = Vector2(target.x, target.y),
       super(Joint2DKind.mouse);

  /// The body it pulls.
  final Physics2DBody body;

  /// World-space target position (update each frame to drag the body).
  final Vector2 target;

  /// The pull per unit of distance to the target.
  double stiffness;

  /// The drag per unit of the body's speed.
  double damping;

  /// The largest force it applies.
  double maxForce;

  @override
  List<Physics2DBody> get bodies => [body];

  /// Move the target (call from the game loop when the pointer moves).
  void setTarget(double x, double y) {
    target.x = x;
    target.y = y;
  }

  @override
  void applyConstraint(double dt) {
    if (!body.isDynamic || !body.isActive) return;
    final dx = target.x - body.position.x;
    final dy = target.y - body.position.y;

    // Spring + damping
    var fx = stiffness * dx - damping * body.velocity.x;
    var fy = stiffness * dy - damping * body.velocity.y;

    // Clamp to maxForce
    final forceMag = math.sqrt(fx * fx + fy * fy);
    if (forceMag > maxForce) {
      final scale = maxForce / forceMag;
      fx *= scale;
      fy *= scale;
    }

    body.applyForce(Vector2(fx, fy));
  }
}

// ── Weld Joint ─────────────────────────────────────────────────────────────

/// Holds bodyB where it was against bodyA when the joint was made, turning
/// with bodyA, at the same angle to it.
class WeldJoint2D extends Joint2D {
  /// A weld holding [bodyB] to [bodyA] as they are now.
  factory WeldJoint2D({
    required Physics2DBody bodyA,
    required Physics2DBody bodyB,
    bool collideConnected = false,
  }) {
    final (x, y) = _jointTurn(
      bodyB.position.x - bodyA.position.x,
      bodyB.position.y - bodyA.position.y,
      -bodyA.angle,
    );
    return WeldJoint2D._(
      bodyA,
      bodyB,
      x,
      y,
      bodyB.angle - bodyA.angle,
      collideConnected,
    );
  }

  WeldJoint2D._(
    this.bodyA,
    this.bodyB,
    this._relX,
    this._relY,
    this._relAngle,
    bool collideConnected,
  ) : super(Joint2DKind.weld, collideConnected: collideConnected);

  /// The first body.
  final Physics2DBody bodyA;

  /// The second body.
  final Physics2DBody bodyB;

  // bodyB's offset from bodyA in bodyA's frame, and the angle between them,
  // when made.
  final double _relX;
  final double _relY;
  final double _relAngle;

  @override
  List<Physics2DBody> get bodies => [bodyA, bodyB];

  @override
  void applyConstraint(double dt) {
    if (!_jointLive(bodyA, bodyB)) return;
    final (ox, oy) = _jointTurn(_relX, _relY, bodyA.angle);
    final tx = bodyA.position.x + ox, ty = bodyA.position.y + oy;

    final ex = bodyB.position.x - tx, ey = bodyB.position.y - ty;
    final err = math.sqrt(ex * ex + ey * ey);
    if (err > 1e-9) {
      _jointSeparate(bodyA, bodyB, ex / err, ey / err, -err * _jointCorrection);
    }
    // The velocity of the point of bodyA it is welded to: bodyA's own, and
    // its spin about it.
    final w = bodyA.angularVelocity;
    _jointHold(
      bodyA,
      bodyB,
      bodyA.velocity.x - w * oy,
      bodyA.velocity.y + w * ox,
    );

    // Turned together.
    if (bodyB.isDynamic || !bodyA.isDynamic) {
      bodyB.angle = bodyA.angle + _relAngle;
      bodyB.angularVelocity = bodyA.angularVelocity;
    } else {
      bodyA.angle = bodyB.angle - _relAngle;
      bodyA.angularVelocity = bodyB.angularVelocity;
    }
  }
}

// ── Prismatic Joint ────────────────────────────────────────────────────────

/// Lets bodyB slide along an axis from where it was when the joint was
/// made, as Box2D's prismatic joint does: off the axis it is held where it
/// was, and the two do not turn against each other.
///
/// The axis is fixed in bodyA's frame (it turns with bodyA). Limits
/// ([setLimits]) are how far it may slide from where it started; a motor
/// ([setMotor]) drives the sliding speed with up to a force.
class PrismaticJoint2D extends Joint2D {
  /// A slider letting [bodyB] move along [axis] (in the world, now) from
  /// where it is.
  factory PrismaticJoint2D({
    required Physics2DBody bodyA,
    required Physics2DBody bodyB,
    required Offset axis,
    bool collideConnected = false,
  }) {
    final len = axis.distance;
    final (ax, ay) = len < 1e-9
        ? (1.0, 0.0)
        : _jointTurn(axis.dx / len, axis.dy / len, -bodyA.angle);
    final (dx, dy) = _jointTurn(
      bodyB.position.x - bodyA.position.x,
      bodyB.position.y - bodyA.position.y,
      -bodyA.angle,
    );
    return PrismaticJoint2D._(
      bodyA,
      bodyB,
      ax,
      ay,
      -dx * ay + dy * ax,
      dx * ax + dy * ay,
      bodyB.angle - bodyA.angle,
      collideConnected,
    );
  }

  PrismaticJoint2D._(
    this.bodyA,
    this.bodyB,
    this._axisX,
    this._axisY,
    this._across0,
    this._along0,
    this._relAngle,
    bool collideConnected,
  ) : super(Joint2DKind.prismatic, collideConnected: collideConnected);

  /// The body the axis is fixed in.
  final Physics2DBody bodyA;

  /// The body that slides.
  final Physics2DBody bodyB;

  // The unit axis in bodyA's frame.
  final double _axisX;
  final double _axisY;

  // bodyB's offset from bodyA, across the axis and along it, when made.
  final double _across0;
  final double _along0;
  final double _relAngle;

  bool _limitEnabled = false;
  double _lowerLimit = 0.0;
  double _upperLimit = 0.0;
  bool _motorEnabled = false;
  double _motorSpeed = 0.0;
  double _maxMotorForce = 0.0;

  @override
  List<Physics2DBody> get bodies => [bodyA, bodyB];

  /// How far bodyB has slid along the axis since the joint was made.
  double get translation {
    final (ax, ay) = _jointTurn(_axisX, _axisY, bodyA.angle);
    return (bodyB.position.x - bodyA.position.x) * ax +
        (bodyB.position.y - bodyA.position.y) * ay -
        _along0;
  }

  @override
  void setLimits(double lower, double upper, {bool enable = true}) {
    _lowerLimit = math.min(lower, upper);
    _upperLimit = math.max(lower, upper);
    _limitEnabled = enable;
  }

  @override
  void setMotor(double speed, double maxForceOrTorque, {bool enable = true}) {
    _motorSpeed = speed;
    _maxMotorForce = maxForceOrTorque;
    _motorEnabled = enable;
  }

  @override
  void applyConstraint(double dt) {
    if (!_jointLive(bodyA, bodyB)) return;
    final k = bodyA.solverInverseMass + bodyB.solverInverseMass;
    if (k == 0) return;
    final (ax, ay) = _jointTurn(_axisX, _axisY, bodyA.angle);
    final px = -ay, py = ax;

    // Off the axis: held where it was.
    final dx = bodyB.position.x - bodyA.position.x;
    final dy = bodyB.position.y - bodyA.position.y;
    final across = dx * px + dy * py - _across0;
    if (across.abs() > 1e-9) {
      _jointSeparate(bodyA, bodyB, px, py, -across * _jointCorrection);
    }
    final rvx = bodyB.velocity.x - bodyA.velocity.x;
    final rvy = bodyB.velocity.y - bodyA.velocity.y;
    _jointPush(bodyA, bodyB, px, py, -(rvx * px + rvy * py) / k);

    if (bodyB.isDynamic || !bodyA.isDynamic) {
      bodyB.angle = bodyA.angle + _relAngle;
      bodyB.angularVelocity = bodyA.angularVelocity;
    } else {
      bodyA.angle = bodyB.angle - _relAngle;
      bodyA.angularVelocity = bodyB.angularVelocity;
    }

    // Along it: the motor, then the limits.
    var along = rvx * ax + rvy * ay;
    if (_motorEnabled) {
      final most = _maxMotorForce * dt;
      final lambda = ((_motorSpeed - along) / k).clamp(-most, most);
      _jointPush(bodyA, bodyB, ax, ay, lambda);
      along += lambda * k;
    }
    if (_limitEnabled) {
      final slide = dx * ax + dy * ay - _along0;
      if (slide < _lowerLimit) {
        _jointSeparate(
          bodyA,
          bodyB,
          ax,
          ay,
          (_lowerLimit - slide) * _jointCorrection,
        );
      } else if (slide > _upperLimit) {
        _jointSeparate(
          bodyA,
          bodyB,
          ax,
          ay,
          (_upperLimit - slide) * _jointCorrection,
        );
      }
      final atLower = slide <= _lowerLimit + _jointSlop && along < 0;
      final atUpper = slide >= _upperLimit - _jointSlop && along > 0;
      if (atLower || atUpper) _jointPush(bodyA, bodyB, ax, ay, -along / k);
    }
  }
}

// ── Wheel Joint ────────────────────────────────────────────────────────────

/// A wheel on a suspension, as Box2D's wheel joint: bodyB's axle moves
/// along an axis on a spring about where it was when the joint was made, is
/// held where it was across the axis, and the wheel turns freely about it —
/// a motor ([setMotor]) spins it.
///
/// The axis is fixed in bodyA's frame. The suspension is 4 Hz with a
/// damping ratio of 0.7 until [setSpring] says otherwise (Box2D's wheel
/// starts the same); 0 Hz takes it away, leaving the axle free along the
/// axis.
class WheelJoint2D extends Joint2D {
  /// A wheel joint holding [bodyB]'s axle — [anchor], a world point, or
  /// [bodyB]'s centre when left out — on [axis] (in the world, now) through
  /// where it is.
  factory WheelJoint2D({
    required Physics2DBody bodyA,
    required Physics2DBody bodyB,
    required Offset axis,
    Offset? anchor,
    bool collideConnected = false,
  }) {
    final len = axis.distance;
    final (ax, ay) = len < 1e-9
        ? (0.0, 1.0)
        : _jointTurn(axis.dx / len, axis.dy / len, -bodyA.angle);
    final (bx, by) = anchor == null ? (0.0, 0.0) : _jointLocal(bodyB, anchor);
    final (wx, wy) = _jointPoint(bodyB, bx, by);
    final (dx, dy) = _jointTurn(
      wx - bodyA.position.x,
      wy - bodyA.position.y,
      -bodyA.angle,
    );
    return WheelJoint2D._(
      bodyA,
      bodyB,
      ax,
      ay,
      bx,
      by,
      -dx * ay + dy * ax,
      dx * ax + dy * ay,
      collideConnected,
    ).._setSpring(4.0, 0.7);
  }

  WheelJoint2D._(
    this.bodyA,
    this.bodyB,
    this._axisX,
    this._axisY,
    this._axleX,
    this._axleY,
    this._across0,
    this._along0,
    bool collideConnected,
  ) : super(Joint2DKind.wheel, collideConnected: collideConnected);

  /// The chassis: the body the axis is fixed in.
  final Physics2DBody bodyA;

  /// The wheel.
  final Physics2DBody bodyB;

  // The unit axis in bodyA's frame.
  final double _axisX;
  final double _axisY;

  // The axle in bodyB's frame.
  final double _axleX;
  final double _axleY;

  // The axle's offset from bodyA, across the axis and along it, when made.
  final double _across0;
  final double _along0;

  double _springHertz = 0.0;
  double _dampingRatio = 0.0;
  double _pull = 0.0;
  double _drag = 0.0;

  bool _motorEnabled = false;
  double _motorSpeed = 0.0;
  double _maxMotorTorque = 0.0;

  @override
  List<Physics2DBody> get bodies => [bodyA, bodyB];

  /// The suspension's frequency in hertz; 0 for none.
  double get springHertz => _springHertz;

  /// The suspension's damping ratio.
  double get dampingRatio => _dampingRatio;

  @override
  void setSpring(double hertz, double dampingRatio) =>
      _setSpring(hertz, dampingRatio);

  void _setSpring(double hertz, double ratio) {
    _springHertz = hertz > 0 ? hertz : 0.0;
    _dampingRatio = ratio;
    final (pull, drag) = _jointSpring(_springHertz, ratio);
    _pull = pull;
    _drag = drag;
  }

  @override
  void setMotor(double speed, double maxForceOrTorque, {bool enable = true}) {
    _motorSpeed = speed;
    _maxMotorTorque = maxForceOrTorque;
    _motorEnabled = enable;
  }

  @override
  void applyConstraint(double dt) {
    if (!_jointLive(bodyA, bodyB)) return;
    final k = bodyA.solverInverseMass + bodyB.solverInverseMass;
    if (k == 0) return;
    final (ax, ay) = _jointTurn(_axisX, _axisY, bodyA.angle);
    final px = -ay, py = ax;

    // The axle, and how fast it moves: bodyB's own velocity and its spin
    // about it.
    final (ox, oy) = _jointTurn(_axleX, _axleY, bodyB.angle);
    final w = bodyB.angularVelocity;

    // Across the axis: held where it was.
    final dx = bodyB.position.x + ox - bodyA.position.x;
    final dy = bodyB.position.y + oy - bodyA.position.y;
    final across = dx * px + dy * py - _across0;
    if (across.abs() > 1e-9) {
      _jointSeparate(bodyA, bodyB, px, py, -across * _jointCorrection);
    }
    final rvx = bodyB.velocity.x - w * oy - bodyA.velocity.x;
    final rvy = bodyB.velocity.y + w * ox - bodyA.velocity.y;
    _jointPush(bodyA, bodyB, px, py, -(rvx * px + rvy * py) / k);

    // Along it: the suspension spring, about where it started.
    if (_pull > 0) {
      final slide = dx * ax + dy * ay - _along0;
      final along = rvx * ax + rvy * ay;
      _jointPush(
        bodyA,
        bodyB,
        ax,
        ay,
        -(_pull * slide + _drag * along) * dt / k,
      );
    }

    if (_motorEnabled) {
      _jointSpinMotor(bodyA, bodyB, _motorSpeed, _maxMotorTorque, dt);
    }
  }
}

// ── Revolute Joint ─────────────────────────────────────────────────────────

/// A hinge: the two bodies turn about a shared anchor.
///
/// A body hinged away from its centre swings round the anchor at the
/// distance it started at, and is drawn turning as it swings; one hinged at
/// its centre stays on the anchor and spins freely. The anchor is fixed in
/// bodyA's frame. Limits ([setLimits]) and the motor ([setMotor]) act on
/// the angle between the bodies, from what it was when the joint was made,
/// as Box2D's revolute joint.
class RevoluteJoint2D extends Joint2D {
  /// A hinge between [bodyA] and [bodyB] about [worldAnchor].
  factory RevoluteJoint2D({
    required Physics2DBody bodyA,
    required Physics2DBody bodyB,
    required Offset worldAnchor,
    bool collideConnected = false,
  }) {
    final (lx, ly) = _jointLocal(bodyA, worldAnchor);
    final bx = bodyB.position.x - worldAnchor.dx;
    final by = bodyB.position.y - worldAnchor.dy;
    return RevoluteJoint2D._(
      bodyA,
      bodyB,
      lx,
      ly,
      math.sqrt(bx * bx + by * by),
      math.atan2(by, bx) - bodyA.angle,
      bodyB.angle - bodyA.angle,
      collideConnected,
    );
  }

  RevoluteJoint2D._(
    this.bodyA,
    this.bodyB,
    this._anchorX,
    this._anchorY,
    this._radius,
    this._swing0,
    this._relAngle0,
    bool collideConnected,
  ) : super(Joint2DKind.revolute, collideConnected: collideConnected);

  /// The body the anchor is fixed in.
  final Physics2DBody bodyA;

  /// The body that turns about it.
  final Physics2DBody bodyB;

  // The anchor in bodyA's frame.
  final double _anchorX;
  final double _anchorY;

  /// How far bodyB's centre is from the anchor.
  final double _radius;

  // bodyB's direction from the anchor, in bodyA's frame, and the angle
  // between the bodies, when made.
  final double _swing0;
  final double _relAngle0;

  bool _limitEnabled = false;
  double _lowerLimit = 0.0;
  double _upperLimit = 0.0;
  bool _motorEnabled = false;
  double _motorSpeed = 0.0;
  double _maxMotorTorque = 0.0;

  @override
  List<Physics2DBody> get bodies => [bodyA, bodyB];

  /// The angle between the bodies, from what it was when the joint was made
  /// (Box2D's joint angle).
  double get jointAngle => bodyB.angle - bodyA.angle - _relAngle0;

  @override
  void setLimits(double lower, double upper, {bool enable = true}) {
    _lowerLimit = math.min(lower, upper);
    _upperLimit = math.max(lower, upper);
    _limitEnabled = enable;
  }

  @override
  void setMotor(double speed, double maxForceOrTorque, {bool enable = true}) {
    _motorSpeed = speed;
    _maxMotorTorque = maxForceOrTorque;
    _motorEnabled = enable;
  }

  @override
  void applyConstraint(double dt) {
    if (!_jointLive(bodyA, bodyB)) return;
    final k = bodyA.solverInverseMass + bodyB.solverInverseMass;
    final (ox, oy) = _jointTurn(_anchorX, _anchorY, bodyA.angle);
    final anchorX = bodyA.position.x + ox, anchorY = bodyA.position.y + oy;
    // The anchor's velocity: bodyA's own, and its spin about it.
    final w = bodyA.angularVelocity;
    final pvx = bodyA.velocity.x - w * oy, pvy = bodyA.velocity.y + w * ox;

    if (_radius < _jointSlop) {
      _pinned(dt, k, anchorX, anchorY, pvx, pvy);
    } else if (k > 0) {
      _swinging(dt, k, anchorX, anchorY, pvx, pvy);
    }
  }

  /// bodyB hinged at its centre: held on the anchor, turning freely.
  void _pinned(
    double dt,
    double k,
    double anchorX,
    double anchorY,
    double pvx,
    double pvy,
  ) {
    if (k > 0) {
      final ex = bodyB.position.x - anchorX, ey = bodyB.position.y - anchorY;
      final err = math.sqrt(ex * ex + ey * ey);
      if (err > 1e-9) {
        _jointSeparate(
          bodyA,
          bodyB,
          ex / err,
          ey / err,
          -err * _jointCorrection,
        );
      }
      _jointHold(bodyA, bodyB, pvx, pvy);
    }
    if (_motorEnabled) {
      _jointSpinMotor(bodyA, bodyB, _motorSpeed, _maxMotorTorque, dt);
    }
    if (!_limitEnabled) return;
    final angle = jointAngle;
    final spin = bodyB.angularVelocity - bodyA.angularVelocity;
    double? limit;
    if (angle < _lowerLimit && spin <= 0) limit = _lowerLimit;
    if (angle > _upperLimit && spin >= 0) limit = _upperLimit;
    if (limit == null) return;
    if (_jointInverseInertia(bodyB) > 0) {
      bodyB.angle = bodyA.angle + _relAngle0 + limit;
      bodyB.angularVelocity = bodyA.angularVelocity;
    } else if (_jointInverseInertia(bodyA) > 0) {
      bodyA.angle = bodyB.angle - _relAngle0 - limit;
      bodyA.angularVelocity = bodyB.angularVelocity;
    }
  }

  /// bodyB hinged away from its centre: it swings round the anchor at
  /// [_radius], turned as it swings.
  void _swinging(
    double dt,
    double k,
    double anchorX,
    double anchorY,
    double pvx,
    double pvy,
  ) {
    var dx = bodyB.position.x - anchorX, dy = bodyB.position.y - anchorY;
    var dist = math.sqrt(dx * dx + dy * dy);
    if (dist < 1e-8) {
      // On the anchor: out again the way it started.
      final (sx, sy) = _jointTurn(1, 0, _swing0 + bodyA.angle);
      dx = sx * 1e-6;
      dy = sy * 1e-6;
      dist = 1e-6;
    }
    var nx = dx / dist, ny = dy / dist;
    _jointSeparate(bodyA, bodyB, nx, ny, (_radius - dist) * _jointCorrection);
    var rvx = bodyB.velocity.x - pvx, rvy = bodyB.velocity.y - pvy;
    _jointPush(bodyA, bodyB, nx, ny, -(rvx * nx + rvy * ny) / k);

    // Where it is on its swing, and how fast it goes round.
    dx = bodyB.position.x - anchorX;
    dy = bodyB.position.y - anchorY;
    dist = math.max(math.sqrt(dx * dx + dy * dy), 1e-6);
    nx = dx / dist;
    ny = dy / dist;
    final tx = -ny, ty = nx;
    rvx = bodyB.velocity.x - pvx;
    rvy = bodyB.velocity.y - pvy;
    var round = rvx * tx + rvy * ty;
    var swing = _wrap(math.atan2(dy, dx) - bodyA.angle - _swing0);

    if (_motorEnabled && _maxMotorTorque > 0) {
      // A torque about the anchor is a force round it of torque / radius.
      final most = _maxMotorTorque / dist * dt;
      final want = (_motorSpeed + bodyA.angularVelocity) * dist;
      final lambda = ((want - round) / k).clamp(-most, most);
      _jointPush(bodyA, bodyB, tx, ty, lambda);
      round += lambda * k;
    }
    if (_limitEnabled) {
      double? limit;
      if (swing < _lowerLimit) limit = _lowerLimit;
      if (swing > _upperLimit) limit = _upperLimit;
      if (limit != null && bodyB.isDynamic) {
        // Back onto its swing at the limit.
        final (sx, sy) = _jointTurn(1, 0, _swing0 + bodyA.angle + limit);
        bodyB.position.x = anchorX + sx * _radius;
        bodyB.position.y = anchorY + sy * _radius;
        swing = limit;
      }
      final atLower = swing <= _lowerLimit && round < 0;
      final atUpper = swing >= _upperLimit && round > 0;
      if (atLower || atUpper) {
        _jointPush(bodyA, bodyB, tx, ty, -round / k);
        round = 0;
      }
    }

    // Turned as it swings.
    if (bodyB.isDynamic) {
      bodyB.angle = bodyA.angle + _relAngle0 + swing;
      bodyB.angularVelocity = bodyA.angularVelocity + round / _radius;
    }
  }

  /// [a] into (−π, π].
  static double _wrap(double a) {
    var r = a % (2 * math.pi);
    if (r > math.pi) r -= 2 * math.pi;
    return r;
  }
}
