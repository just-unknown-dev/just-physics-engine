part of 'physics_engine.dart';

// A kinematic character mover: Box2D's (samples/sample_character.cpp, after
// Quake's pmove) on top of two world queries every backend answers —
// [PhysicsEngine.collideMover] and [PhysicsEngine.castMover]. Native Box2D
// answers them in C; the pure-Dart engine with the same algorithm in Dart.
// The plane solver and the rest of the move are shared, so a character walks
// the same on the web as on a phone.

/// The Box2D linear slop in world units: 0.005 m at the 100 units per
/// metre both backends use. Movers keep this close to what they touch.
const double moverLinearSlop = 0.5;

/// Whether a one-way platform holds a mover: within ~60° of the way it is
/// solid from, the same tolerance as for bodies.
const double _moverOneWayCos = 0.5;

/// How far a mover may sit into a one-way platform's surface and still be
/// held by it; any deeper and it is passing through.
const double _moverOneWayDepth = 2 * moverLinearSlop;

/// The shape a character moves as, centred on its position: a capsule
/// standing upright, or a box with slightly rounded corners.
final class MoverShape {
  /// An upright capsule [height] tall and [radius] across, end to end.
  factory MoverShape.capsule({required double radius, required double height}) {
    final r = math.max(radius, 2 * moverLinearSlop + 0.01);
    final half = math.max(height / 2 - r, 0.0);
    return MoverShape._(
      half == 0 ? const [Offset.zero] : [Offset(0, -half), Offset(0, half)],
      r,
      2 * r,
      math.max(height, 2 * r),
    );
  }

  /// A [width] × [height] box whose corners are rounded by [cornerRadius]
  /// (a tenth of its shorter side by default), which lets it slide over
  /// seams instead of catching on them.
  factory MoverShape.box({
    required double width,
    required double height,
    double? cornerRadius,
  }) {
    final shorter = math.min(width, height);
    final r = (cornerRadius ?? shorter * 0.1)
        .clamp(2 * moverLinearSlop + 0.01, math.max(shorter / 2, 1.01))
        .toDouble();
    final hx = math.max(width / 2 - r, 0.0), hy = math.max(height / 2 - r, 0.0);
    return MoverShape._(
      [Offset(-hx, -hy), Offset(hx, -hy), Offset(hx, hy), Offset(-hx, hy)],
      r,
      width,
      height,
    );
  }

  const MoverShape._(this.points, this.radius, this.width, this.height);

  /// The core's points, relative to the centre.
  final List<Offset> points;

  /// How far the shape reaches past its core.
  final double radius;

  /// Its full width.
  final double width;

  /// Its full height.
  final double height;

  /// Its proxy centred on [position].
  ShapeProxy2D proxyAt(Offset position) =>
      ShapeProxy2D([for (final p in points) p + position], radius);
}

/// Which bodies a mover meets: those whose category is in [maskBits] and
/// whose mask holds [categoryBits] (Box2D's query filter), never a sensor,
/// and never [ignore] (the character's own body).
final class MoverFilter {
  /// A filter; by default everything but sensors.
  const MoverFilter({
    this.categoryBits = 1,
    this.maskBits = 0xFFFFFFFF,
    this.ignore,
  });

  /// The mover's own category bits.
  final int categoryBits;

  /// The categories it collides with.
  final int maskBits;

  /// A body to pass through, usually the character's own.
  final PhysicsBody? ignore;

  /// Whether the mover meets [body].
  bool passes(PhysicsBody body) =>
      body.isActive &&
      !body.isSensor &&
      !identical(body, ignore) &&
      (body.categoryBits & maskBits) != 0 &&
      (body.maskBits & categoryBits) != 0;
}

/// A surface a mover touches: Box2D's b2CollisionPlane. The mover is kept
/// on the [normal] side, at least [offset] along it.
final class MoverPlane {
  /// A plane of [body]'s with [normal] (towards the mover), [offset] (how
  /// far the mover is into it) and the touching [point].
  MoverPlane({
    required this.normal,
    required this.offset,
    required this.point,
    this.body,
    this.pushLimit = double.infinity,
    this.clipVelocity = true,
  });

  /// The surface's normal, towards the mover.
  final Offset normal;

  /// How far the mover is into the surface's reach: the two shapes' radii
  /// less the distance between their cores. Never negative.
  final double offset;

  /// The closest point to the mover on the other shape's core (its outline
  /// less its radius: a circle's centre, a capsule's spine), in the world.
  final Offset point;

  /// Whose surface it is.
  final PhysicsBody? body;

  /// The most the solver may push the mover out (soft collision when
  /// finite); below 0, or not a number, counts as 0.
  double pushLimit;

  /// The push the solver gave; 0 for a plane it did not need.
  double push = 0;

  /// Whether the mover's velocity is clipped against it.
  bool clipVelocity;

  /// How far [delta] leaves the mover outside the plane (negative inside).
  double separation(Offset delta) =>
      normal.dx * delta.dx + normal.dy * delta.dy - offset;

  @override
  String toString() =>
      'MoverPlane(normal: $normal, offset: ${offset.toStringAsFixed(3)})';
}

/// Box2D's plane solver for movers (b2SolvePlanes and b2ClipVector).
abstract final class MoverSolver {
  /// Where a mover wanting to move by [targetDelta] can go without breaking
  /// [planes]: the target pushed out of each plane, iterated (Box2D's
  /// b2SolvePlanes). Each plane's [MoverPlane.push] is set.
  static ({Offset translation, int iterations}) solvePlanes(
    Offset targetDelta,
    List<MoverPlane> planes,
  ) {
    for (final p in planes) {
      p.push = 0;
    }
    var dx = targetDelta.dx, dy = targetDelta.dy;
    var iteration = 0;
    for (; iteration < 20; iteration++) {
      var totalPush = 0.0;
      for (final plane in planes) {
        // Slop prevents jitter.
        final separation =
            plane.normal.dx * dx +
            plane.normal.dy * dy -
            plane.offset +
            moverLinearSlop;
        var push = -separation;
        final accumulated = plane.push;
        final limit = plane.pushLimit >= 0 ? plane.pushLimit : 0.0;
        plane.push = (plane.push + push).clamp(0.0, limit);
        push = plane.push - accumulated;
        dx += push * plane.normal.dx;
        dy += push * plane.normal.dy;
        totalPush += push.abs();
      }
      if (totalPush < moverLinearSlop) break;
    }
    return (translation: Offset(dx, dy), iterations: iteration);
  }

  /// [vector] with what runs into [planes] taken off (Box2D's
  /// b2ClipVector): planes the solver did not push, or that do not clip,
  /// are skipped.
  static Offset clipVector(Offset vector, List<MoverPlane> planes) {
    var vx = vector.dx, vy = vector.dy;
    for (final plane in planes) {
      if (plane.push == 0 || !plane.clipVelocity) continue;
      final into = math.min(0.0, vx * plane.normal.dx + vy * plane.normal.dy);
      vx -= into * plane.normal.dx;
      vy -= into * plane.normal.dy;
    }
    return Offset(vx, vy);
  }
}

/// What a character stands on after a move.
enum GroundState2D {
  /// On ground it can stand on.
  onGround,

  /// On a slope too steep to stand on: it slides.
  onSteepGround,

  /// Touching something that does not hold it up.
  notSupported,

  /// Touching nothing.
  inAir,
}

/// The outcome of [CharacterMover2D.move].
final class CharacterMove2D {
  CharacterMove2D._({
    required this.position,
    required this.velocity,
    required this.groundState,
    required this.groundNormal,
    required this.groundBody,
    required this.contacts,
  });

  /// Where the character ended up (its shape's centre).
  final Offset position;

  /// Its velocity with what it ran into taken off.
  final Offset velocity;

  /// What it stands on.
  final GroundState2D groundState;

  /// The ground's normal, or zero in the air.
  final Offset groundNormal;

  /// The body it stands on, or null.
  final PhysicsBody? groundBody;

  /// The surfaces it touched at the end of the move.
  final List<MoverPlane> contacts;

  /// Whether it stands on walkable ground.
  bool get isGrounded => groundState == GroundState2D.onGround;
}

/// Moves a character through a [PhysicsEngine]'s world as a kinematic
/// shape: it slides along walls and slopes, stops at ceilings, climbs steps
/// up to [stepHeight], keeps to the floor going down within
/// [stickToFloor], stands on one-way platforms and passes up through them,
/// and pushes dynamic bodies it walks into.
///
/// It is not a body: nothing collides with it unless the caller also keeps
/// a body where it is (and passes it as [MoverFilter.ignore]).
final class CharacterMover2D {
  /// A mover in [engine]'s world.
  CharacterMover2D(this.engine);

  /// The world it moves through.
  final PhysicsEngine engine;

  static const _iterations = 5;
  static const _tolerance = 2 * moverLinearSlop; // Box2D's 0.01 m

  /// Moves [shape] from [position] at [velocity] for [dt] seconds.
  ///
  /// [up] points against gravity. [maxSlopeAngle] (radians) is the steepest
  /// ground it stands on. [wasGrounded] is whether it stood on ground before
  /// this move: steps and keeping to the floor need it. With [pushStrength]
  /// above 0, it pushes dynamic bodies in its way as a body of [mass] would,
  /// with at most that force.
  CharacterMove2D move({
    required MoverShape shape,
    required Offset position,
    required Offset velocity,
    required double dt,
    Offset up = const Offset(0, -1),
    double maxSlopeAngle = 0.8726646259971648,
    double stepHeight = 0,
    double stickToFloor = 0,
    bool wasGrounded = false,
    MoverFilter filter = const MoverFilter(),
    double mass = 1,
    double pushStrength = 0,
  }) {
    // Made unit length: the slope and step tests measure along it.
    final upLength = up.distance;
    up = upLength > 0 && upLength.isFinite
        ? up / upLength
        : const Offset(0, -1);
    final minUp = math.cos(maxSlopeAngle);
    final delta = velocity * dt;
    var (pos, planes) = _slide(shape, position, delta, filter, up, minUp);
    var v = _clip(_offSteep(velocity, planes, up, minUp), planes, up, minUp);

    // A step in the way: up, across, back down — only while it still stands
    // after this move, as Jolt's character walks stairs: a jump that clips
    // a step's edge carries on, and a character on a rising lift still
    // steps off it.
    if (wasGrounded &&
        stepHeight > 0 &&
        _ground(
              engine.collideMover(shape, pos, filter: filter),
              up,
              minUp,
            ).$1 ==
            GroundState2D.onGround) {
      final stepped = _tryStep(
        shape,
        position,
        pos,
        delta,
        up,
        stepHeight,
        stickToFloor,
        minUp,
        filter,
      );
      if (stepped != null) {
        pos = stepped;
        // Walking up a step leaves no speed into it or up from it.
        final across = _along(velocity, up);
        v = Offset(across.dx, across.dy) + up * math.min(0.0, _dot(v, up));
      }
    }

    var contacts = engine.collideMover(shape, pos, filter: filter);
    var ground = _ground(contacts, up, minUp);

    // Keep to the floor going down slopes and off small ledges.
    if (wasGrounded &&
        ground.$1 != GroundState2D.onGround &&
        stickToFloor > 0 &&
        _dot(v, up) <= 0) {
      final down = up * -stickToFloor;
      final f = engine.castMover(shape, pos, down, filter: filter);
      if (f < 1) {
        final candidate = pos + down * f;
        final below = engine.collideMover(shape, candidate, filter: filter);
        final g = _ground(below, up, minUp);
        if (g.$1 == GroundState2D.onGround) {
          pos = candidate;
          contacts = below;
          ground = g;
        }
      }
    }

    // Pushed as hard as it tries to walk into them, not as hard as it got.
    if (pushStrength > 0) _push(contacts, velocity, dt, mass, pushStrength);

    return CharacterMove2D._(
      position: pos,
      velocity: v,
      groundState: ground.$1,
      groundNormal: ground.$2,
      groundBody: ground.$3,
      contacts: contacts,
    );
  }

  /// Box2D's mover loop: collide, solve, cast, advance, a few times.
  (Offset, List<MoverPlane>) _slide(
    MoverShape shape,
    Offset start,
    Offset delta,
    MoverFilter filter,
    Offset up,
    double minUp,
  ) {
    var target = start + delta;
    var pos = start;
    var planes = const <MoverPlane>[];
    for (var i = 0; i < _iterations; i++) {
      planes = engine.collideMover(shape, pos, filter: filter);
      final wanted = _offSteep(target - pos, planes, up, minUp);
      target = pos + wanted;
      final t = MoverSolver.solvePlanes(wanted, planes).translation;
      final fraction = engine.castMover(shape, pos, t, filter: filter);
      final step = t * fraction;
      pos += step;
      if (step.distanceSquared < _tolerance * _tolerance) break;
    }
    return (pos, planes);
  }

  /// A position past a step in the way, or null when there is none to
  /// climb (or climbing gets no further).
  Offset? _tryStep(
    MoverShape shape,
    Offset start,
    Offset slid,
    Offset delta,
    Offset up,
    double stepHeight,
    double stickToFloor,
    double minUp,
    MoverFilter filter,
  ) {
    final across = _along(delta, up);
    final want = across.distance;
    if (want < _tolerance) return null;
    final dir = across / want;
    final got = _dot(slid - start, dir);
    if (got > want * 0.9) return null; // Not blocked.

    final lift = up * stepHeight;
    final pUp =
        start + lift * engine.castMover(shape, start, lift, filter: filter);
    final (pAcross, _) = _slide(shape, pUp, across, filter, up, minUp);
    if (_dot(pAcross - start, dir) <= got + _tolerance) return null;
    final drop = up * -(stepHeight + stickToFloor);
    final f = engine.castMover(shape, pAcross, drop, filter: filter);
    if (f >= 1) return null; // Nothing to land on: a ledge, not a step.
    final landed = pAcross + drop * f;
    final g = _ground(
      engine.collideMover(shape, landed, filter: filter),
      up,
      minUp,
    );
    if (g.$1 != GroundState2D.onGround) return null;
    if (_dot(landed - start, up) > stepHeight + moverLinearSlop) return null;
    return landed;
  }

  /// [velocity] clipped against [planes] as [MoverSolver.clipVector] does,
  /// except that a slope too steep to stand on never turns running into it
  /// into speed up it.
  static Offset _clip(
    Offset velocity,
    List<MoverPlane> planes,
    Offset up,
    double minUp,
  ) {
    var v = velocity;
    for (final p in planes) {
      if (p.push == 0 || !p.clipVelocity) continue;
      final into = math.min(0.0, _dot(v, p.normal));
      if (into == 0) continue;
      var next = v - p.normal * into;
      final u = _dot(p.normal, up);
      if (u > 0.01 && u < minUp) {
        final before = math.max(_dot(v, up), 0.0), after = _dot(next, up);
        if (after > before) next -= up * (after - before);
      }
      v = next;
    }
    return v;
  }

  /// [motion] without what runs into the slopes among [planes] that are
  /// too steep to stand on, across them (Jolt treats them as walls the
  /// same way): the solver would otherwise turn running into one into
  /// sliding up it. Gravity still slides the character down them.
  static Offset _offSteep(
    Offset motion,
    List<MoverPlane> planes,
    Offset up,
    double minUp,
  ) {
    var out = motion;
    for (final p in planes) {
      final u = _dot(p.normal, up);
      if (u <= 0.01 || u >= minUp) continue;
      final across = _along(p.normal, up);
      final length = across.distance;
      if (length == 0) continue;
      final wall = across / length;
      final into = _dot(out, wall);
      if (into < 0) out -= wall * into;
    }
    return out;
  }

  /// The ground among [planes]: the one facing most up.
  (GroundState2D, Offset, PhysicsBody?) _ground(
    List<MoverPlane> planes,
    Offset up,
    double minUp,
  ) {
    if (planes.isEmpty) return (GroundState2D.inAir, Offset.zero, null);
    MoverPlane? best;
    var bestUp = -2.0;
    for (final p in planes) {
      final u = _dot(p.normal, up);
      if (u > bestUp) {
        bestUp = u;
        best = p;
      }
    }
    if (bestUp >= minUp) {
      return (GroundState2D.onGround, best!.normal, best.body);
    }
    if (bestUp > 0.01) {
      return (GroundState2D.onSteepGround, best!.normal, best.body);
    }
    return (GroundState2D.notSupported, Offset.zero, null);
  }

  /// Pushes the dynamic bodies among [contacts] that [velocity] runs into.
  void _push(
    List<MoverPlane> contacts,
    Offset velocity,
    double dt,
    double mass,
    double pushStrength,
  ) {
    for (final p in contacts) {
      final body = p.body;
      if (body == null || body.effectiveBodyType != BodyType.dynamic) continue;
      final approach = -_dot(velocity, p.normal);
      if (approach <= 0) continue;
      final impulse = math.min(mass * approach, pushStrength * dt);
      engine.applyLinearImpulse(
        body,
        -p.normal.dx * impulse,
        -p.normal.dy * impulse,
      );
    }
  }

  static double _dot(Offset a, Offset b) => a.dx * b.dx + a.dy * b.dy;

  /// [v] without its part along [up].
  static Offset _along(Offset v, Offset up) => v - up * _dot(v, up);
}

double _moverDot(Offset a, Offset b) => a.dx * b.dx + a.dy * b.dy;

/// The proxies of [body]'s shapes, relative to [origin], each with the
/// ends of the chain segment it is (for the one-sided test), if it is one.
List<(ShapeProxy2D, Offset?, Offset?)> _moverProxiesOf(
  PhysicsBody body,
  Offset origin,
) {
  final c = math.cos(body.angle), s = math.sin(body.angle);
  final px = body.position.x - origin.dx, py = body.position.y - origin.dy;
  Offset at(Offset p) =>
      Offset(c * p.dx - s * p.dy + px, s * p.dx + c * p.dy + py);
  final out = <(ShapeProxy2D, Offset?, Offset?)>[];
  void add(CollisionShape shape) {
    switch (shape) {
      case CircleShape():
        out.add((ShapeProxy2D([at(shape.center)], shape.radius), null, null));
      case CapsuleShape():
        out.add((
          ShapeProxy2D([at(shape.center1), at(shape.center2)], shape.radius),
          null,
          null,
        ));
      case RoundedPolygonShape():
        out.add((
          ShapeProxy2D([
            for (final v in shape.vertices) at(v),
          ], shape.cornerRadius),
          null,
          null,
        ));
      case PolygonShape():
        out.add((
          ShapeProxy2D([for (final v in shape.vertices) at(v)], 0),
          null,
          null,
        ));
      case SegmentShape():
        // Box2D's segments have no thickness; nor do the mover's.
        out.add((
          ShapeProxy2D([at(shape.point1), at(shape.point2)], 0),
          null,
          null,
        ));
      case ChainShape():
        final v = shape.vertices;
        final n = shape.loop ? v.length : v.length - 1;
        for (var i = 0; i < n; i++) {
          final a = at(v[i]), b = at(v[(i + 1) % v.length]);
          out.add((ShapeProxy2D([a, b], 0), a, b));
        }
      default:
        break;
    }
  }

  add(body.shape);
  body.additionalShapes.forEach(add);
  return out;
}

/// The way a one-way [body] is solid from, as the normal of a surface
/// facing a mover it holds; null for an ordinary body.
Offset? _moverSolidSide(PhysicsBody body) {
  if (!body.isOneWay) return null;
  return switch (body.oneWayDirection) {
    OneWayDirection.fromAbove => const Offset(0, -1),
    OneWayDirection.fromBelow => const Offset(0, 1),
    OneWayDirection.fromLeft => const Offset(-1, 0),
    OneWayDirection.fromRight => const Offset(1, 0),
  };
}

/// [body]'s bounds turned as it is: the box round its unturned bounds
/// turned about its position (the proxies turn; its bounds do not).
Rect _moverBoundsOf(PhysicsBody body) {
  final position = body.position.toOffset();
  final bounds = body.getCompoundBounds(position);
  if (body.angle == 0) return bounds;
  final c = math.cos(body.angle), s = math.sin(body.angle);
  final d = bounds.center - position;
  final centre = Offset(c * d.dx - s * d.dy, s * d.dx + c * d.dy) + position;
  final hw = bounds.width / 2, hh = bounds.height / 2;
  return Rect.fromCenter(
    center: centre,
    width: 2 * (c.abs() * hw + s.abs() * hh),
    height: 2 * (s.abs() * hw + c.abs() * hh),
  );
}

/// The pure-Dart answers to [PhysicsEngine.collideMover] and
/// [PhysicsEngine.castMover]; the native backend answers in C with the same
/// rules.
extension on PhysicsEngine {
  Iterable<PhysicsBody> _moverCandidates(Rect box, MoverFilter filter) sync* {
    for (final body in bodies) {
      if (!filter.passes(body)) continue;
      if (!_moverBoundsOf(body).overlaps(box)) continue;
      yield body;
    }
  }

  List<MoverPlane> _collideMoverDart(
    MoverShape shape,
    Offset position,
    MoverFilter filter,
  ) {
    final mover = shape.proxyAt(Offset.zero);
    final box = Rect.fromCenter(
      center: position,
      width: shape.width + 2 * moverLinearSlop,
      height: shape.height + 2 * moverLinearSlop,
    );
    final planes = <MoverPlane>[];
    for (final body in _moverCandidates(box, filter)) {
      final solid = _moverSolidSide(body);
      for (final (proxy, _, _) in _moverProxiesOf(body, position)) {
        final d = ShapeDistance2D.between(proxy, mover);
        final totalRadius = proxy.radius + mover.radius;
        if (d.overlaps || d.distance > totalRadius) continue;
        final offset = totalRadius - d.distance;
        if (solid != null &&
            (_moverDot(d.normal, solid) < _moverOneWayCos ||
                offset > _moverOneWayDepth)) {
          continue;
        }
        planes.add(
          MoverPlane(
            normal: d.normal,
            offset: offset,
            point: d.pointA + position,
            body: body,
          ),
        );
      }
    }
    return planes;
  }

  double _castMoverDart(
    MoverShape shape,
    Offset position,
    Offset translation,
    MoverFilter filter,
  ) {
    if (translation == Offset.zero) return 1;
    final mover = shape.proxyAt(Offset.zero);
    final here = Rect.fromCenter(
      center: position,
      width: shape.width,
      height: shape.height,
    );
    final box = here
        .expandToInclude(here.shift(translation))
        .inflate(moverLinearSlop);
    var fraction = 1.0;
    for (final body in _moverCandidates(box, filter)) {
      final solid = _moverSolidSide(body);
      // A one-way platform stops only a mover coming at it from its solid side.
      if (solid != null && _moverDot(translation, solid) >= 0) continue;
      for (final (proxy, a, b) in _moverProxiesOf(body, position)) {
        if (solid != null) {
          // Partway through it already (rising through, or carried into
          // it): passed over, as collideMover passes over it.
          final d = ShapeDistance2D.between(proxy, mover);
          if (d.overlaps ||
              proxy.radius + mover.radius - d.distance > _moverOneWayDepth) {
            continue;
          }
        }
        if (a != null && b != null) {
          // A chain is one-sided: a mover behind a segment passes it.
          final edge = b - a;
          var cx = 0.0, cy = 0.0;
          for (final p in mover.points) {
            cx += p.dx;
            cy += p.dy;
          }
          final r =
              Offset(cx / mover.points.length, cy / mover.points.length) - a;
          if (r.dx * edge.dy - r.dy * edge.dx < 0) continue;
        }
        final hit = castShape2D(
          proxy,
          mover,
          translation,
          maxFraction: fraction,
          canEncroach: true,
          linearSlop: moverLinearSlop,
        );
        // Shapes it starts out touching are ignored: the planes deal with
        // those, and the cast must be free to slide away from them.
        if (hit == null || hit.fraction == 0) continue;
        if (solid != null && _moverDot(hit.normal, solid) < _moverOneWayCos) {
          continue;
        }
        fraction = hit.fraction;
      }
    }
    return fraction;
  }
}
