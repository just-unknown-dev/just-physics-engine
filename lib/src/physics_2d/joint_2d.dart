part of 'physics_engine.dart';

/// Enumeration of joint types.
enum JointType {
  distance,
  mouse,
  weld,
  revolute,
  prismatic,
  wheel,
}

/// Base class for all Dart-fallback joint constraints.
///
/// Joints are applied by [PhysicsEngine] after collision resolution each step.
/// Subclasses implement [applyConstraint] to enforce their specific constraint.
abstract class JointConstraint {
  final JointType type;

  JointConstraint(this.type);

  /// Apply the constraint for one physics step.
  void applyConstraint(double dt);
}

// ── Distance Joint ─────────────────────────────────────────────────────────

/// Maintains a target distance (range [minLength, maxLength]) between two bodies.
///
/// An optional spring (stiffness, damping) makes the constraint soft; set
/// stiffness to 0 for a rigid distance limit.
class DistanceJoint extends JointConstraint {
  final PhysicsBody bodyA;
  final PhysicsBody bodyB;

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
  })  : minLength = minLength ?? length,
        maxLength = maxLength ?? length,
        super(JointType.distance);

  @override
  void applyConstraint(double dt) {
    final dx = bodyB.position.x - bodyA.position.x;
    final dy = bodyB.position.y - bodyA.position.y;
    final dist = math.sqrt(dx * dx + dy * dy);
    if (dist < 1e-8) return;

    final nx = dx / dist;
    final ny = dy / dist;

    double violation = 0.0;
    if (dist < minLength) {
      violation = dist - minLength;
    } else if (dist > maxLength) {
      violation = dist - maxLength;
    }

    final totalInvMass = bodyA.inverseMass + bodyB.inverseMass;
    if (totalInvMass < 1e-10) return;

    if (violation.abs() > 1e-6) {
      const baumgarte = 0.25;
      final correction = violation * baumgarte / totalInvMass;
      bodyA.position.x += nx * correction * bodyA.inverseMass;
      bodyA.position.y += ny * correction * bodyA.inverseMass;
      bodyB.position.x -= nx * correction * bodyB.inverseMass;
      bodyB.position.y -= ny * correction * bodyB.inverseMass;
    }

    // Soft spring impulse: positive violation means too far → pull together.
    // impulse > 0: move A in +nx (toward B), move B in -nx (toward A).
    if (stiffness > 0) {
      final relVelN = (bodyB.velocity.x - bodyA.velocity.x) * nx +
          (bodyB.velocity.y - bodyA.velocity.y) * ny;
      final springForce = stiffness * violation;
      final dampForce = damping * relVelN;
      final impulse = (springForce - dampForce) * dt / totalInvMass;
      bodyA.velocity.x += nx * impulse * bodyA.inverseMass;
      bodyA.velocity.y += ny * impulse * bodyA.inverseMass;
      bodyB.velocity.x -= nx * impulse * bodyB.inverseMass;
      bodyB.velocity.y -= ny * impulse * bodyB.inverseMass;
    }
  }
}

// ── Mouse Joint ────────────────────────────────────────────────────────────

/// Pulls a body toward a target point using a spring force.
///
/// Typical use: dragging bodies with a pointer.
class MouseJoint extends JointConstraint {
  final PhysicsBody body;

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
  })  : target = Vector2(target.x, target.y),
        super(JointType.mouse);

  /// Move the target (call from the game loop when the pointer moves).
  void setTarget(double x, double y) {
    target.x = x;
    target.y = y;
  }

  @override
  void applyConstraint(double dt) {
    if (body.mass <= 0) return;
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

/// Rigidly locks two bodies together by strongly correcting their relative
/// position and velocity each step.
class WeldJoint extends JointConstraint {
  final PhysicsBody bodyA;
  final PhysicsBody bodyB;

  // Relative offset of bodyB from bodyA (captured at creation time).
  final double _relX;
  final double _relY;
  final double _relAngle;

  WeldJoint({required this.bodyA, required this.bodyB})
      : _relX = bodyB.position.x - bodyA.position.x,
        _relY = bodyB.position.y - bodyA.position.y,
        _relAngle = bodyB.angle - bodyA.angle,
        super(JointType.weld);

  @override
  void applyConstraint(double dt) {
    final totalInvMass = bodyA.inverseMass + bodyB.inverseMass;
    if (totalInvMass < 1e-10) return;

    // Target position for bodyB based on bodyA's current transform.
    final ca = math.cos(bodyA.angle);
    final sa = math.sin(bodyA.angle);
    final targetX = bodyA.position.x + ca * _relX - sa * _relY;
    final targetY = bodyA.position.y + sa * _relX + ca * _relY;

    final errX = targetX - bodyB.position.x;
    final errY = targetY - bodyB.position.y;

    const baumgarte = 0.5;
    final corrX = errX * baumgarte / totalInvMass;
    final corrY = errY * baumgarte / totalInvMass;

    bodyA.position.x -= corrX * bodyA.inverseMass;
    bodyA.position.y -= corrY * bodyA.inverseMass;
    bodyB.position.x += corrX * bodyB.inverseMass;
    bodyB.position.y += corrY * bodyB.inverseMass;

    // Velocity correction (remove relative velocity)
    final rvx = bodyB.velocity.x - bodyA.velocity.x;
    final rvy = bodyB.velocity.y - bodyA.velocity.y;
    final imp = 0.8 / totalInvMass;
    bodyA.velocity.x += rvx * bodyA.inverseMass * imp;
    bodyA.velocity.y += rvy * bodyA.inverseMass * imp;
    bodyB.velocity.x -= rvx * bodyB.inverseMass * imp;
    bodyB.velocity.y -= rvy * bodyB.inverseMass * imp;

    // Angular weld: correct angle
    final targetAngle = bodyA.angle + _relAngle;
    final angleErr = targetAngle - bodyB.angle;
    bodyA.angle += angleErr * bodyA.inverseMass / (bodyA.inverseMass + bodyB.inverseMass) * 0.5;
    bodyB.angle -= angleErr * bodyB.inverseMass / (bodyA.inverseMass + bodyB.inverseMass) * 0.5;
  }
}

// ── Prismatic Joint ────────────────────────────────────────────────────────

/// Constrains two bodies to slide along a fixed world-space axis while
/// preventing relative rotation and off-axis movement.
///
/// Optional limits clamp the sliding range. An optional motor drives the
/// relative velocity along the axis up to a maximum force.
class PrismaticJoint extends JointConstraint {
  final PhysicsBody bodyA;
  final PhysicsBody bodyB;

  /// Unit vector (world-space) defining the allowed slide direction.
  final double _axisX;
  final double _axisY;

  bool limitEnabled = false;
  double lowerLimit = 0.0;
  double upperLimit = 0.0;
  bool motorEnabled = false;
  double motorSpeed = 0.0;
  double maxMotorForce = 0.0;

  PrismaticJoint({
    required this.bodyA,
    required this.bodyB,
    required Offset axis,
  })  : _axisX = axis.dx,
        _axisY = axis.dy,
        super(JointType.prismatic);

  @override
  void applyConstraint(double dt) {
    final totalInvMass = bodyA.inverseMass + bodyB.inverseMass;
    if (totalInvMass < 1e-10) return;

    // Perpendicular axis: constrain off-axis relative displacement.
    final perpX = -_axisY;
    final perpY = _axisX;

    final dx = bodyB.position.x - bodyA.position.x;
    final dy = bodyB.position.y - bodyA.position.y;
    final perpErr = dx * perpX + dy * perpY;

    const baumgarte = 0.3;
    final correction = perpErr * baumgarte / totalInvMass;
    bodyA.position.x += perpX * correction * bodyA.inverseMass;
    bodyA.position.y += perpY * correction * bodyA.inverseMass;
    bodyB.position.x -= perpX * correction * bodyB.inverseMass;
    bodyB.position.y -= perpY * correction * bodyB.inverseMass;

    // Limit: clamp displacement along the axis.
    if (limitEnabled) {
      final slide = dx * _axisX + dy * _axisY;
      double axisErr = 0.0;
      if (slide < lowerLimit) axisErr = slide - lowerLimit;
      if (slide > upperLimit) axisErr = slide - upperLimit;
      if (axisErr.abs() > 1e-6) {
        final axisCorr = axisErr * baumgarte / totalInvMass;
        bodyA.position.x += _axisX * axisCorr * bodyA.inverseMass;
        bodyA.position.y += _axisY * axisCorr * bodyA.inverseMass;
        bodyB.position.x -= _axisX * axisCorr * bodyB.inverseMass;
        bodyB.position.y -= _axisY * axisCorr * bodyB.inverseMass;
      }
    }

    // Motor: drive relative velocity along the axis.
    if (motorEnabled) {
      final relVel = (bodyB.velocity.x - bodyA.velocity.x) * _axisX +
          (bodyB.velocity.y - bodyA.velocity.y) * _axisY;
      var force = (motorSpeed - relVel) / totalInvMass;
      force = force.clamp(-maxMotorForce, maxMotorForce);
      final impulse = force * dt;
      bodyA.velocity.x -= _axisX * impulse * bodyA.inverseMass;
      bodyA.velocity.y -= _axisY * impulse * bodyA.inverseMass;
      bodyB.velocity.x += _axisX * impulse * bodyB.inverseMass;
      bodyB.velocity.y += _axisY * impulse * bodyB.inverseMass;
    }
  }
}

// ── Revolute Joint ─────────────────────────────────────────────────────────

/// Constrains two bodies to rotate around a shared world-space anchor point.
///
/// This is a simplified position-constraint implementation; for high-accuracy
/// revolute joints, use the Box2D FFI backend.
class RevoluteJoint extends JointConstraint {
  final PhysicsBody bodyA;
  final PhysicsBody bodyB;

  // Anchor offset in local body space (captured at creation).
  final double _anchorAX;
  final double _anchorAY;
  final double _anchorBX;
  final double _anchorBY;

  bool limitEnabled = false;
  double lowerLimit = 0.0;
  double upperLimit = 0.0;
  bool motorEnabled = false;
  double motorSpeed = 0.0;
  double maxMotorTorque = 0.0;

  RevoluteJoint({
    required this.bodyA,
    required this.bodyB,
    required Offset worldAnchor,
  })  : _anchorAX = worldAnchor.dx - bodyA.position.x,
        _anchorAY = worldAnchor.dy - bodyA.position.y,
        _anchorBX = worldAnchor.dx - bodyB.position.x,
        _anchorBY = worldAnchor.dy - bodyB.position.y,
        super(JointType.revolute);

  @override
  void applyConstraint(double dt) {
    final totalInvMass = bodyA.inverseMass + bodyB.inverseMass;
    if (totalInvMass < 1e-10) return;

    // World-space anchor on A
    final caA = math.cos(bodyA.angle);
    final saA = math.sin(bodyA.angle);
    final wAx = bodyA.position.x + caA * _anchorAX - saA * _anchorAY;
    final wAy = bodyA.position.y + saA * _anchorAX + caA * _anchorAY;

    // World-space anchor on B
    final caB = math.cos(bodyB.angle);
    final saB = math.sin(bodyB.angle);
    final wBx = bodyB.position.x + caB * _anchorBX - saB * _anchorBY;
    final wBy = bodyB.position.y + saB * _anchorBX + caB * _anchorBY;

    final errX = wAx - wBx;
    final errY = wAy - wBy;

    const baumgarte = 0.3;
    final corrX = errX * baumgarte / totalInvMass;
    final corrY = errY * baumgarte / totalInvMass;

    bodyA.position.x -= corrX * bodyA.inverseMass;
    bodyA.position.y -= corrY * bodyA.inverseMass;
    bodyB.position.x += corrX * bodyB.inverseMass;
    bodyB.position.y += corrY * bodyB.inverseMass;

    // Motor torque
    if (motorEnabled) {
      final relAngVel = bodyB.angularVelocity - bodyA.angularVelocity;
      var torque = maxMotorTorque * (motorSpeed - relAngVel).clamp(-1.0, 1.0);
      final inertiaSum = bodyA.inverseInertia + bodyB.inverseInertia;
      if (inertiaSum > 1e-10) {
        torque = torque.clamp(-maxMotorTorque, maxMotorTorque);
        bodyA.angularVelocity -= torque * bodyA.inverseInertia * dt;
        bodyB.angularVelocity += torque * bodyB.inverseInertia * dt;
      }
    }
  }
}
