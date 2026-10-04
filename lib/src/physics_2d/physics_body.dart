part of 'physics_engine.dart';

/// How a body participates in the simulation.
///
/// Ordinals match Box2D v3's `b2BodyType` exactly (static=0, kinematic=1,
/// dynamic=2) so the value can cross the FFI boundary as `index` with no
/// translation table.
///
/// [PhysicsBody.mass] `<= 0` still forces [BodyType.static] via
/// [PhysicsBody.effectiveBodyType] — that inference predates this enum and
/// several call sites depend on it, so [BodyType.kinematic] is purely opt-in.
enum BodyType {
  /// Never moves, infinite mass. Level geometry.
  static,

  /// Moved only by explicit velocity writes; unaffected by gravity, forces or
  /// collisions, and immovable by dynamic bodies that push against it.
  /// Moving platforms, elevators, crushers.
  kinematic,

  /// Fully simulated. Characters, crates, debris.
  dynamic,
}

/// Which side a one-way platform is solid from.
///
/// Compared against the contact normal, so it is independent of how fast the
/// other body is travelling. This package is screen-space (+Y is *down*;
/// default gravity is `+981`), so [fromAbove] — the platformer default — means
/// the mover must be at a smaller Y than the platform.
enum OneWayDirection {
  /// Solid only when approached from above. Standard jump-through platform.
  fromAbove,

  /// Solid only when approached from below. Ceiling-mounted pass-through.
  fromBelow,

  /// Solid only when approached from the left.
  fromLeft,

  /// Solid only when approached from the right.
  fromRight,
}

/// Physics body — the core simulation object in the physics engine.
///
/// Uses [Vector2] for position/velocity/acceleration to avoid per-frame
/// [Offset] allocations.
class PhysicsBody {
  /// Mutable position.
  final Vector2 position;

  /// Mutable velocity.
  final Vector2 velocity;

  /// Mutable acceleration accumulator.
  final Vector2 acceleration;

  /// The physical shape used for collision detection
  CollisionShape shape;

  /// Mass (0 means infinite mass / static body)
  double mass;

  /// Inverse Mass (calculated automatically)
  double get inverseMass => mass > 0 ? 1.0 / mass : 0.0;

  /// Restitution (bounciness)
  double restitution;

  /// Surface Friction
  double friction;

  /// Current angle in radians
  double angle;

  /// Angular velocity (radians per second)
  double angularVelocity;

  /// Torque accumulator
  double torque;

  /// Moment of inertia
  double inertia;

  /// Inverse Inertia (calculated automatically)
  double get inverseInertia => inertia > 0 ? 1.0 / inertia : 0.0;

  /// Linear damping: how fast the body's velocity decays on its own, per
  /// second. 0 keeps it moving; 1 loses about two thirds of it each second.
  double drag;

  /// Angular damping: how fast the body's spin decays on its own, per
  /// second, as [drag] does for its velocity. 0 by default.
  double angularDamping;

  /// Coarse gravity switch. When false this body ignores world gravity
  /// entirely, regardless of [gravityScale].
  ///
  /// Kept as a separate flag rather than folded into [gravityScale] because
  /// `PhysicsSystem._syncIn` writes `body.useGravity = !comp.isStatic` on
  /// *every* frame — a computed view would clobber any gameplay-set
  /// [gravityScale] on the next tick. Read [effectiveGravityScale] instead of
  /// either field directly.
  bool useGravity;

  /// Per-body multiplier on world gravity. 1.0 is normal, 0.5 floaty,
  /// 2.0 heavy, 0.0 weightless.
  ///
  /// This is what variable jump height is built from: hold-to-jump lowers it
  /// while the button is down and raises it on release, giving a short hop or
  /// a full jump from one impulse.
  double gravityScale;

  /// Is active
  bool isActive;

  /// Check collisions
  bool checkCollision;

  /// Whether the body may fall asleep when it comes to rest. A body that
  /// never sleeps costs a little every step, at rest or not; one that must
  /// react to something changing under it without being touched — a sensor
  /// standing still, a body whose gravity a script turns off and on — wants
  /// it. True by default.
  bool canSleep;

  /// Object Sleeping: if true, physics integration happens.
  ///
  /// On the Box2D backend this reflects Box2D's own internal sleep state
  /// (synced back every step) and can be set to start a body asleep, but
  /// the two thresholds below cannot influence Box2D's sleep decision — see
  /// their docs.
  bool isAwake;

  /// Object Sleeping: how long this body has been below movement threshold.
  ///
  /// Pure-Dart engine only; unused on the Box2D backend, which tracks its
  /// own internal sleep timer.
  double sleepTimer;

  /// Object Sleeping: max velocity squared to be considered for sleeping.
  ///
  /// Pure-Dart engine only. Box2D backend: not read from this field — Box2D
  /// uses its own fixed per-body threshold (~5 units/s at this package's
  /// centimetre scale) with no public per-body override.
  double sleepVelocityThreshold;

  /// Object Sleeping: amount of time to stay below threshold before sleeping.
  ///
  /// Pure-Dart engine only. Box2D backend: not read from this field — Box2D
  /// uses a fixed internal 0.5s time-to-sleep (`B2_TIME_TO_SLEEP`) with no
  /// public API to override it.
  double sleepTimeThreshold;

  /// Sensor mode: if true this body detects overlaps but does not resolve them.
  bool isSensor;

  /// One-way / pass-through platform flag.
  ///
  /// When true, a contact against this body is only resolved if the other
  /// side approached from [oneWayDirection]; otherwise it passes straight
  /// through. The decision is made from the contact normal, not from the
  /// mover's velocity, so a body that stalls at the apex of a jump inside the
  /// platform still passes through.
  ///
  /// Honoured on both backends: the pure-Dart engine tests the normal during
  /// narrow-phase resolution, and the Box2D backend installs a native
  /// pre-solve contact filter that applies the same test. Two one-way bodies
  /// touching each other collide normally.
  bool isOneWay;

  /// Which side [isOneWay] makes this body solid from. Ignored when
  /// [isOneWay] is false.
  OneWayDirection oneWayDirection;

  /// Bullet mode: enables Continuous Collision Detection (CCD) for fast-moving
  /// bodies so they don't tunnel through thin static geometry.
  /// Only effective on the Box2D FFI backend; the Dart fallback ignores it.
  bool isBullet;

  /// Locks rotation: the body's [angle] never changes from torque/angular
  /// impulses (e.g. friction against a static obstacle), only from explicit
  /// external writes. The standard fix for top-down characters that should
  /// slide along scenery instead of visibly spinning on contact.
  bool fixedRotation;

  /// How this body participates in the simulation. Defaults to
  /// [BodyType.dynamic]; see [effectiveBodyType] for the `mass <= 0` override.
  BodyType bodyType;

  /// Additional collision shapes attached to this body (compound body support).
  ///
  /// All shapes share the same position as [position]. The [shape] field is
  /// always the primary shape; [additionalShapes] are extra fixtures.
  final List<CollisionShape> additionalShapes;

  /// Collision filter category bits.
  int categoryBits;

  /// Collision filter mask bits — collides with bodies whose categoryBits
  /// overlaps with this mask (bitwise AND != 0).
  int maskBits;

  /// Collision group index.
  /// Positive: always collide with same index.
  /// Negative: never collide with same index.
  /// Zero: use category/mask filtering only.
  int groupIndex;

  /// Create a physics body
  PhysicsBody({
    required Vector2 position,
    required this.shape,
    Vector2? velocity,
    Vector2? acceleration,
    this.mass = 1.0,
    this.restitution = 0.5,
    this.friction = 0.2,
    this.angle = 0.0,
    this.angularVelocity = 0.0,
    this.torque = 0.0,
    this.inertia = 1.0,
    this.drag = 0.1,
    this.angularDamping = 0.0,
    this.useGravity = true,
    this.gravityScale = 1.0,
    this.isActive = true,
    this.checkCollision = true,
    this.isAwake = true,
    this.canSleep = true,
    this.sleepTimer = 0.0,
    this.sleepVelocityThreshold = 5.0,
    this.sleepTimeThreshold = 0.5,
    this.isSensor = false,
    this.isOneWay = false,
    this.oneWayDirection = OneWayDirection.fromAbove,
    this.isBullet = false,
    this.fixedRotation = false,
    this.bodyType = BodyType.dynamic,
    List<CollisionShape>? additionalShapes,
    this.categoryBits = 0x0001,
    this.maskBits = 0xFFFF,
    this.groupIndex = 0,
  }) : additionalShapes = additionalShapes ?? [],
       position = Vector2(position.x, position.y),
       velocity = velocity != null
           ? Vector2(velocity.x, velocity.y)
           : Vector2.zero(),
       acceleration = acceleration != null
           ? Vector2(acceleration.x, acceleration.y)
           : Vector2.zero();

  /// The gravity multiplier actually applied this step: [gravityScale] when
  /// [useGravity], otherwise zero. Always read this, never the raw fields.
  double get effectiveGravityScale => useGravity ? gravityScale : 0.0;

  /// The body type actually simulated.
  ///
  /// `mass <= 0` has meant "static" since before [bodyType] existed, and
  /// `PhysicsSystem` still expresses staticness that way, so that inference
  /// wins over an explicitly-set [bodyType].
  BodyType get effectiveBodyType =>
      mass <= 0 ? BodyType.static : bodyType;

  /// True when this body is fully simulated (moved by gravity and forces, and
  /// pushed by collisions).
  bool get isDynamic => effectiveBodyType == BodyType.dynamic;

  /// True when this body moves only by explicit velocity writes and cannot be
  /// pushed by anything.
  bool get isKinematic => effectiveBodyType == BodyType.kinematic;

  /// True when this body never moves.
  bool get isStatic => effectiveBodyType == BodyType.static;

  /// Inverse mass as the solver should use it: zero for anything that is not
  /// [BodyType.dynamic], so static *and* kinematic bodies are immovable while
  /// the dynamic side is still pushed out of them.
  double get solverInverseMass => isDynamic ? inverseMass : 0.0;

  /// True when this body has more than one collision shape.
  bool get isCompound => additionalShapes.isNotEmpty;

  /// Returns the AABB that covers all shapes on this body.
  Rect getCompoundBounds(Offset position) {
    var bounds = shape.getBounds(position);
    for (final s in additionalShapes) {
      final b = s.getBounds(position);
      bounds = Rect.fromLTRB(
        bounds.left < b.left ? bounds.left : b.left,
        bounds.top < b.top ? bounds.top : b.top,
        bounds.right > b.right ? bounds.right : b.right,
        bounds.bottom > b.bottom ? bounds.bottom : b.bottom,
      );
    }
    return bounds;
  }

  /// Apply force
  void applyForce(Vector2 force) {
    if (mass > 0) {
      acceleration.x += force.x * inverseMass;
      acceleration.y += force.y * inverseMass;
    }
  }

  /// Apply torque
  void applyTorque(double applicationTorque) {
    if (inertia > 0) {
      torque += applicationTorque;
    }
  }

  /// Apply impulse
  void applyImpulse(Vector2 impulse) {
    velocity.x += impulse.x;
    velocity.y += impulse.y;
  }
}

/// Represents a rigid body with 2D force accumulation (convenience wrapper).
///
/// For most use cases, prefer [PhysicsBody] directly. This class exists for
/// code that needs a simpler interface without collision shape requirements.
class RigidBody {
  /// Mass of the rigid body
  double mass = 1.0;

  /// Accumulated force (reset each integration step).
  final Vector2 _force = Vector2.zero();

  /// Current velocity.
  final Vector2 velocity = Vector2.zero();

  /// Current position.
  final Vector2 position = Vector2.zero();

  /// Apply a 2D force.
  void applyForce(double x, double y) {
    _force.x += x;
    _force.y += y;
  }

  /// Integrate forces → velocity → position for one timestep.
  void integrate(double dt) {
    if (mass <= 0) return;
    final inverseMass = 1.0 / mass;
    velocity.x += _force.x * inverseMass * dt;
    velocity.y += _force.y * inverseMass * dt;
    position.x += velocity.x * dt;
    position.y += velocity.y * dt;
    _force.setZero();
  }
}

/// Utility for manual broad-phase collision queries outside [PhysicsEngine].
///
/// Wraps the [SpatialGrid]-based check used internally by the engine and
/// exposes it for gameplay code that needs ad-hoc overlap tests.
class CollisionDetector {
  final List<PhysicsBody> _bodies = [];

  /// Register a body for detection.
  void addBody(PhysicsBody body) => _bodies.add(body);

  /// Remove a body.
  void removeBody(PhysicsBody body) => _bodies.remove(body);

  /// Return all overlapping body pairs using brute-force AABB check.
  List<(PhysicsBody, PhysicsBody)> detectCollisions() {
    final pairs = <(PhysicsBody, PhysicsBody)>[];
    for (var i = 0; i < _bodies.length; i++) {
      final a = _bodies[i];
      if (!a.isActive) continue;
      final aBounds = a.shape.getBounds(a.position.toOffset());
      for (var j = i + 1; j < _bodies.length; j++) {
        final b = _bodies[j];
        if (!b.isActive) continue;
        final bBounds = b.shape.getBounds(b.position.toOffset());
        if (aBounds.overlaps(bBounds)) {
          pairs.add((a, b));
        }
      }
    }
    return pairs;
  }
}

/// Applies global forces (gravity, wind, etc.) to a set of [PhysicsBody]s.
class ForceManager {
  /// Current gravity vector. Default matches [PhysicsEngine] and [Box2DPhysicsEngine]:
  /// 981 units/s² (9.81 m/s² at 1 unit = 1 cm).
  final Vector2 gravity = Vector2(0, 981.0);

  /// Set gravity.
  void setGravity(double x, double y) {
    gravity.x = x;
    gravity.y = y;
  }

  /// Apply gravity to all [bodies] that have [PhysicsBody.useGravity] enabled.
  void applyGravity(Iterable<PhysicsBody> bodies) {
    for (final body in bodies) {
      if (!body.isActive || !body.isAwake || !body.useGravity) continue;
      if (body.mass <= 0) continue; // static
      body.acceleration.x += gravity.x;
      body.acceleration.y += gravity.y;
    }
  }
}
