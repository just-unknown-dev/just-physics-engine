part of 'physics_engine.dart';

/// Enumeration of joint types.
enum JointType { distance, mouse, weld, revolute, prismatic, wheel }

/// Base class for all joint constraints.
///
/// The pure-Dart engine applies each joint once per sub-step, after
/// collisions. A joint moves its bodies back toward what it allows (half the
/// error each step) and takes away the relative velocity it forbids, so
/// gravity cannot pile up against it. The engine is translational — shapes
/// do not turn in collisions — so joints move bodies, and set the angle a
/// body is drawn at where they decide it (a hinged body turns as it swings).
/// [Box2DPhysicsEngine] solves its own joints natively.
abstract class JointConstraint {
  final JointType type;

  JointConstraint(this.type);

  /// The bodies the joint holds; empty where it does not say.
  List<PhysicsBody> get bodies => const [];

  /// Apply the constraint for one physics step.
  void applyConstraint(double dt);
}

/// How much of a joint's position error the pure-Dart engine takes away in
/// a step.
const double _jointCorrection = 0.5;

/// How close to a limit counts as at it: Box2D's linear slop (half a unit).
const double _jointSlop = 0.5;

/// Whether both of a joint's bodies take part in the simulation.
bool _jointLive(PhysicsBody a, PhysicsBody b) => a.isActive && b.isActive;

/// [b] moved away from [a] along ([nx], [ny]) by [amount] (negative: toward
/// it), shared by their solver masses.
void _jointSeparate(
  PhysicsBody a,
  PhysicsBody b,
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
  PhysicsBody a,
  PhysicsBody b,
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
void _jointHold(PhysicsBody a, PhysicsBody b, double pvx, double pvy) {
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
double _jointInverseInertia(PhysicsBody b) =>
    b.isDynamic && !b.fixedRotation ? b.inverseInertia : 0.0;

/// A spin impulse driving `ωB − ωA` toward [speed], at most [maxTorque] × dt.
void _jointSpinMotor(
  PhysicsBody a,
  PhysicsBody b,
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

// ── Distance Joint ─────────────────────────────────────────────────────────

/// Keeps two bodies' centres [minLength] to [maxLength] apart.
///
/// Equal lengths make a rigid rod; a range is a rope, free inside it and
/// stopped at its ends. A [stiffness] above 0 makes it a spring instead:
/// outside the range it pulls back with an acceleration of [stiffness] ×
/// the distance out, and [damping] × the speed apart slows it, whatever the
/// masses.
class DistanceJoint extends JointConstraint {
  final PhysicsBody bodyA;
  final PhysicsBody bodyB;

  @override
  List<PhysicsBody> get bodies => [bodyA, bodyB];

  double minLength;
  double maxLength;
  double stiffness; // 0 = rigid limit
  double damping;

  DistanceJoint({
    required this.bodyA,
    required this.bodyB,
    required double length,
    double? minLength,
    double? maxLength,
    this.stiffness = 0.0,
    this.damping = 0.3,
  }) : minLength = minLength ?? length,
       maxLength = maxLength ?? length,
       super(JointType.distance);

  @override
  void applyConstraint(double dt) {
    if (!_jointLive(bodyA, bodyB)) return;
    final k = bodyA.solverInverseMass + bodyB.solverInverseMass;
    if (k == 0) return;
    final dx = bodyB.position.x - bodyA.position.x;
    final dy = bodyB.position.y - bodyA.position.y;
    final dist = math.sqrt(dx * dx + dy * dy);
    if (dist < 1e-8) return;
    final nx = dx / dist, ny = dy / dist;
    final relVel =
        (bodyB.velocity.x - bodyA.velocity.x) * nx +
        (bodyB.velocity.y - bodyA.velocity.y) * ny;

    // How far outside the range: positive too far apart, negative too close.
    final violation = dist < minLength
        ? dist - minLength
        : dist > maxLength
        ? dist - maxLength
        : 0.0;

    if (stiffness > 0) {
      _jointPush(
        bodyA,
        bodyB,
        nx,
        ny,
        -(stiffness * violation + damping * relVel) * dt / k,
      );
      return;
    }

    if (violation != 0) {
      _jointSeparate(bodyA, bodyB, nx, ny, -violation * _jointCorrection);
    }
    // At an end, the speed that would carry it further out goes.
    final outward = dist >= maxLength - _jointSlop && relVel > 0;
    final inward = dist <= minLength + _jointSlop && relVel < 0;
    if (outward || inward) _jointPush(bodyA, bodyB, nx, ny, -relVel / k);
  }
}

// ── Mouse Joint ────────────────────────────────────────────────────────────

/// Pulls a body toward a target point using a spring force.
///
/// Typical use: dragging bodies with a pointer. It is a Dart joint on both
/// backends: on the native one its force goes into the native body.
class MouseJoint extends JointConstraint {
  final PhysicsBody body;

  @override
  List<PhysicsBody> get bodies => [body];

  /// World-space target position (update each frame to drag the body).
  final Vector2 target;

  double stiffness;
  double damping;
  double maxForce;

  MouseJoint({
    required this.body,
    required Vector2 target,
    this.stiffness = 500.0,
    this.damping = 0.7,
    this.maxForce = 50000.0,
  }) : target = Vector2(target.x, target.y),
       super(JointType.mouse);

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
    body.isAwake = true;
    body.sleepTimer = 0;
  }
}

// ── Weld Joint ─────────────────────────────────────────────────────────────

/// Holds bodyB where it was against bodyA when the joint was made, turning
/// with bodyA, at the same angle to it.
class WeldJoint extends JointConstraint {
  final PhysicsBody bodyA;
  final PhysicsBody bodyB;

  @override
  List<PhysicsBody> get bodies => [bodyA, bodyB];

  // bodyB's offset from bodyA in bodyA's frame, and the angle between them,
  // when made.
  final double _relX;
  final double _relY;
  final double _relAngle;

  factory WeldJoint({required PhysicsBody bodyA, required PhysicsBody bodyB}) {
    final (x, y) = _jointTurn(
      bodyB.position.x - bodyA.position.x,
      bodyB.position.y - bodyA.position.y,
      -bodyA.angle,
    );
    return WeldJoint._(bodyA, bodyB, x, y, bodyB.angle - bodyA.angle);
  }

  WeldJoint._(this.bodyA, this.bodyB, this._relX, this._relY, this._relAngle)
    : super(JointType.weld);

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
/// ([lowerLimit], [upperLimit]) are how far it may slide from where it
/// started; a motor drives the sliding speed, pushing with up to
/// [maxMotorForce].
class PrismaticJoint extends JointConstraint {
  final PhysicsBody bodyA;
  final PhysicsBody bodyB;

  @override
  List<PhysicsBody> get bodies => [bodyA, bodyB];

  // The unit axis in bodyA's frame.
  final double _axisX;
  final double _axisY;

  // bodyB's offset from bodyA, across the axis and along it, when made.
  final double _across0;
  final double _along0;
  final double _relAngle;

  bool limitEnabled = false;
  double lowerLimit = 0.0;
  double upperLimit = 0.0;
  bool motorEnabled = false;
  double motorSpeed = 0.0;
  double maxMotorForce = 0.0;

  factory PrismaticJoint({
    required PhysicsBody bodyA,
    required PhysicsBody bodyB,
    required Offset axis,
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
    return PrismaticJoint._(
      bodyA,
      bodyB,
      ax,
      ay,
      -dx * ay + dy * ax,
      dx * ax + dy * ay,
      bodyB.angle - bodyA.angle,
    );
  }

  PrismaticJoint._(
    this.bodyA,
    this.bodyB,
    this._axisX,
    this._axisY,
    this._across0,
    this._along0,
    this._relAngle,
  ) : super(JointType.prismatic);

  /// How far bodyB has slid along the axis since the joint was made.
  double get translation {
    final (ax, ay) = _jointTurn(_axisX, _axisY, bodyA.angle);
    return (bodyB.position.x - bodyA.position.x) * ax +
        (bodyB.position.y - bodyA.position.y) * ay -
        _along0;
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
    if (motorEnabled) {
      final most = maxMotorForce * dt;
      final lambda = ((motorSpeed - along) / k).clamp(-most, most);
      _jointPush(bodyA, bodyB, ax, ay, lambda);
      along += lambda * k;
    }
    if (limitEnabled) {
      final slide = dx * ax + dy * ay - _along0;
      if (slide < lowerLimit) {
        _jointSeparate(
          bodyA,
          bodyB,
          ax,
          ay,
          (lowerLimit - slide) * _jointCorrection,
        );
      } else if (slide > upperLimit) {
        _jointSeparate(
          bodyA,
          bodyB,
          ax,
          ay,
          (upperLimit - slide) * _jointCorrection,
        );
      }
      final atLower = slide <= lowerLimit + _jointSlop && along < 0;
      final atUpper = slide >= upperLimit - _jointSlop && along > 0;
      if (atLower || atUpper) _jointPush(bodyA, bodyB, ax, ay, -along / k);
    }
  }
}

// ── Wheel Joint ────────────────────────────────────────────────────────────

/// A wheel on a suspension, as Box2D's wheel joint: bodyB moves along an
/// axis on a spring about where it was when the joint was made, is held
/// where it was across the axis, and turns freely — a motor spins it.
///
/// The axis is fixed in bodyA's frame. The spring's [springStiffness] is an
/// acceleration per unit out, and [springDamping] per unit of speed,
/// whatever the masses (Box2D's wheel spring is a frequency in hertz and a
/// damping ratio instead).
class WheelJoint extends JointConstraint {
  final PhysicsBody bodyA;
  final PhysicsBody bodyB;

  @override
  List<PhysicsBody> get bodies => [bodyA, bodyB];

  // The unit axis in bodyA's frame.
  final double _axisX;
  final double _axisY;

  // bodyB's offset from bodyA, across the axis and along it, when made.
  final double _across0;
  final double _along0;

  /// Suspension spring stiffness. 0 = no spring force (axis is unconstrained
  /// beyond the perpendicular lock — matches Box2D's default disabled spring).
  double springStiffness = 0.0;
  double springDamping = 0.7;

  bool motorEnabled = false;
  double motorSpeed = 0.0;
  double maxMotorTorque = 0.0;

  factory WheelJoint({
    required PhysicsBody bodyA,
    required PhysicsBody bodyB,
    required Offset axis,
  }) {
    final len = axis.distance;
    final (ax, ay) = len < 1e-9
        ? (0.0, 1.0)
        : _jointTurn(axis.dx / len, axis.dy / len, -bodyA.angle);
    final (dx, dy) = _jointTurn(
      bodyB.position.x - bodyA.position.x,
      bodyB.position.y - bodyA.position.y,
      -bodyA.angle,
    );
    return WheelJoint._(
      bodyA,
      bodyB,
      ax,
      ay,
      -dx * ay + dy * ax,
      dx * ax + dy * ay,
    );
  }

  WheelJoint._(
    this.bodyA,
    this.bodyB,
    this._axisX,
    this._axisY,
    this._across0,
    this._along0,
  ) : super(JointType.wheel);

  /// Set the suspension spring (matches [Box2DJoint.setWheelSpring] for a
  /// consistent API across backends).
  void setWheelSpring(double stiffness, double damping) {
    springStiffness = stiffness;
    springDamping = damping;
  }

  /// Set the rotational drive motor (matches [Box2DJoint.setWheelMotor]).
  void setWheelMotor(double speed, double maxTorque, {bool enable = true}) {
    motorSpeed = speed;
    maxMotorTorque = maxTorque;
    motorEnabled = enable;
  }

  @override
  void applyConstraint(double dt) {
    if (!_jointLive(bodyA, bodyB)) return;
    final k = bodyA.solverInverseMass + bodyB.solverInverseMass;
    if (k == 0) return;
    final (ax, ay) = _jointTurn(_axisX, _axisY, bodyA.angle);
    final px = -ay, py = ax;

    // Across the axis: held where it was.
    final dx = bodyB.position.x - bodyA.position.x;
    final dy = bodyB.position.y - bodyA.position.y;
    final across = dx * px + dy * py - _across0;
    if (across.abs() > 1e-9) {
      _jointSeparate(bodyA, bodyB, px, py, -across * _jointCorrection);
    }
    final rvx = bodyB.velocity.x - bodyA.velocity.x;
    final rvy = bodyB.velocity.y - bodyA.velocity.y;
    _jointPush(bodyA, bodyB, px, py, -(rvx * px + rvy * py) / k);

    // Along it: the suspension spring, about where it started.
    if (springStiffness > 0) {
      final slide = dx * ax + dy * ay - _along0;
      final along = rvx * ax + rvy * ay;
      _jointPush(
        bodyA,
        bodyB,
        ax,
        ay,
        -(springStiffness * slide + springDamping * along) * dt / k,
      );
    }

    if (motorEnabled) {
      _jointSpinMotor(bodyA, bodyB, motorSpeed, maxMotorTorque, dt);
    }
  }
}

// ── Revolute Joint ─────────────────────────────────────────────────────────

/// A hinge: the two bodies turn about a shared anchor.
///
/// A body hinged away from its centre swings round the anchor at the
/// distance it started at, and is drawn turning as it swings; one hinged at
/// its centre stays on the anchor and spins freely. The anchor is fixed in
/// bodyA's frame. Limits ([lowerLimit], [upperLimit]) and the motor act on
/// the angle between the bodies, from what it was when the joint was made,
/// as Box2D's revolute joint.
class RevoluteJoint extends JointConstraint {
  final PhysicsBody bodyA;
  final PhysicsBody bodyB;

  @override
  List<PhysicsBody> get bodies => [bodyA, bodyB];

  // The anchor in bodyA's frame.
  final double _anchorX;
  final double _anchorY;

  /// How far bodyB's centre is from the anchor.
  final double _radius;

  // bodyB's direction from the anchor, in bodyA's frame, and the angle
  // between the bodies, when made.
  final double _swing0;
  final double _relAngle0;

  bool limitEnabled = false;
  double lowerLimit = 0.0;
  double upperLimit = 0.0;
  bool motorEnabled = false;
  double motorSpeed = 0.0;
  double maxMotorTorque = 0.0;

  factory RevoluteJoint({
    required PhysicsBody bodyA,
    required PhysicsBody bodyB,
    required Offset worldAnchor,
  }) {
    final (lx, ly) = _jointTurn(
      worldAnchor.dx - bodyA.position.x,
      worldAnchor.dy - bodyA.position.y,
      -bodyA.angle,
    );
    final bx = bodyB.position.x - worldAnchor.dx;
    final by = bodyB.position.y - worldAnchor.dy;
    return RevoluteJoint._(
      bodyA,
      bodyB,
      lx,
      ly,
      math.sqrt(bx * bx + by * by),
      math.atan2(by, bx) - bodyA.angle,
      bodyB.angle - bodyA.angle,
    );
  }

  RevoluteJoint._(
    this.bodyA,
    this.bodyB,
    this._anchorX,
    this._anchorY,
    this._radius,
    this._swing0,
    this._relAngle0,
  ) : super(JointType.revolute);

  /// The angle between the bodies, from what it was when the joint was made
  /// (Box2D's joint angle).
  double get jointAngle => bodyB.angle - bodyA.angle - _relAngle0;

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
    if (motorEnabled) {
      _jointSpinMotor(bodyA, bodyB, motorSpeed, maxMotorTorque, dt);
    }
    if (!limitEnabled) return;
    final angle = jointAngle;
    final spin = bodyB.angularVelocity - bodyA.angularVelocity;
    double? limit;
    if (angle < lowerLimit && spin <= 0) limit = lowerLimit;
    if (angle > upperLimit && spin >= 0) limit = upperLimit;
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

    if (motorEnabled && maxMotorTorque > 0) {
      // A torque about the anchor is a force round it of torque / radius.
      final most = maxMotorTorque / dist * dt;
      final want = (motorSpeed + bodyA.angularVelocity) * dist;
      final lambda = ((want - round) / k).clamp(-most, most);
      _jointPush(bodyA, bodyB, tx, ty, lambda);
      round += lambda * k;
    }
    if (limitEnabled) {
      double? limit;
      if (swing < lowerLimit) limit = lowerLimit;
      if (swing > upperLimit) limit = upperLimit;
      if (limit != null && bodyB.isDynamic) {
        // Back onto its swing at the limit.
        final (sx, sy) = _jointTurn(1, 0, _swing0 + bodyA.angle + limit);
        bodyB.position.x = anchorX + sx * _radius;
        bodyB.position.y = anchorY + sy * _radius;
        swing = limit;
      }
      final atLower = swing <= lowerLimit && round < 0;
      final atUpper = swing >= upperLimit && round > 0;
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
