/// The native Box2D backend's shapes: each [Shape2D] as the fixtures
/// Box2D collides the same as the pure-Dart engine does.
library;

import 'dart:ffi';
import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui' show Offset;

import 'package:ffi/ffi.dart';
import 'package:just_box_2d/ffi.dart' as box2d;
import 'package:meta/meta.dart' show internal;

import '../physics_2d_engine.dart';

/// Box2D's linear slop (`B2_LINEAR_SLOP`) at the 100 units a metre the
/// native wrapper sets: points closer than a few of these are one point to
/// Box2D.
const double _linearSlop = 0.5;

/// How close Box2D's hull welds two of a polygon's points into one (four
/// linear slops). A polygon thinner than this has fewer than three points
/// left, and Box2D makes no shape from it.
const double _weldTolerance = 4 * _linearSlop;

/// Adds [body]'s shapes — [Physics2DBody.shape], then its
/// [Physics2DBody.additionalShapes] — to the native body [handle], with the
/// body's friction and restitution.
///
/// Box2D gives static and kinematic bodies no mass whatever their shapes'
/// density, so every shape has density 1: a body that turns dynamic later
/// then has its shapes' mass and rotational inertia, which `b2w_setBodyMass`
/// rescales to the configured mass.
@internal
void addBox2DShapes(int handle, Physics2DBody body) {
  const density = 1.0;
  _addSingleShape(handle, body.shape, density, body.friction, body.restitution);
  for (final extra in body.additionalShapes) {
    _addSingleShape(handle, extra, density, body.friction, body.restitution);
  }
}

void _addSingleShape(
  int handle,
  Shape2D shape,
  double density,
  double friction,
  double restitution,
) {
  if (shape is Circle2DShape) {
    box2d.b2w_addCircleShape(
      handle,
      shape.center.dx,
      shape.center.dy,
      shape.radius,
      density,
      friction,
      restitution,
    );
  } else if (shape is Capsule2DShape) {
    _addCapsule(
      handle,
      shape.center1,
      shape.center2,
      shape.radius,
      density,
      friction,
      restitution,
    );
  } else if (shape is Segment2DShape) {
    // The pure-Dart engine collides a segment as a capsule [thickness]
    // round it, so this one does too.
    _addLine(
      handle,
      shape.point1,
      shape.point2,
      shape.thickness,
      density,
      friction,
      restitution,
    );
  } else if (shape is Chain2DShape) {
    if (shape.thickness > 0) {
      // A capsule per segment, [thickness] round it, as the pure-Dart
      // engine collides each one: solid from both sides, and smooth where
      // two meet (their round ends overlap).
      final v = shape.vertices;
      final n = shape.loop && v.length > 2 ? v.length : v.length - 1;
      for (var i = 0; i < n; i++) {
        _addLine(
          handle,
          v[i],
          v[(i + 1) % v.length],
          shape.thickness,
          density,
          friction,
          restitution,
        );
      }
      return;
    }
    // A Box2D chain collides on one side only, the right of its direction.
    // The pure-Dart engine's collides on both, so the chain goes in twice,
    // once each way round.
    final pts = _chainPoints(shape);
    if (pts == null) return;
    final ptr = calloc<Float>(pts.length);
    try {
      for (final points in [pts, _reversedPoints(pts)]) {
        ptr.asTypedList(pts.length).setAll(0, points);
        box2d.b2w_addChainShape(
          handle,
          ptr,
          pts.length ~/ 2,
          shape.loop ? 1 : 0,
          friction,
          restitution,
        );
      }
    } finally {
      calloc.free(ptr);
    }
  } else if (shape is Rectangle2DShape) {
    box2d.b2w_addBoxShape(
      handle,
      shape.width / 2,
      shape.height / 2,
      density,
      friction,
      restitution,
    );
  } else if (shape is Polygon2DShape) {
    // A plain polygon, or one rounded by its corner radius.
    final radius = shape is RoundedPolygon2DShape ? shape.cornerRadius : 0.0;
    final thin = _thinPolygonCore(shape.vertices);
    if (thin != null) {
      // Too thin for Box2D's hull: the capsule round its middle line, as
      // wide and as long as the polygon grown by its radius. A box rounded
      // by half its height or more, which the game engine builds round a
      // box next to nothing tall, is this capsule.
      final (c1, c2, halfWidth) = thin;
      _addCapsule(
        handle,
        c1,
        c2,
        math.max(radius + halfWidth, 1e-3),
        density,
        friction,
        restitution,
      );
      return;
    }
    final verts = Float32List(shape.vertices.length * 2);
    for (int i = 0; i < shape.vertices.length; i++) {
      verts[i * 2] = shape.vertices[i].dx;
      verts[i * 2 + 1] = shape.vertices[i].dy;
    }
    final ptr = calloc<Float>(verts.length);
    try {
      ptr.asTypedList(verts.length).setAll(0, verts);
      if (radius > 0) {
        box2d.b2w_addRoundedPolygonShape(
          handle,
          ptr,
          shape.vertices.length,
          radius,
          density,
          friction,
          restitution,
        );
      } else {
        box2d.b2w_addPolygonShape(
          handle,
          ptr,
          shape.vertices.length,
          density,
          friction,
          restitution,
        );
      }
    } finally {
      calloc.free(ptr);
    }
  }
}

/// The middle line of the convex polygon [vertices] and half its width,
/// when the polygon is too thin or too small for Box2D's hull (which would
/// weld it to fewer than three points and make no shape); null when Box2D
/// can make it. The line is the polygon's length less half its width at
/// each end, so a capsule round it that is half the width wide reaches as
/// far as the polygon does every way.
(Offset, Offset, double)? _thinPolygonCore(List<Offset> vertices) {
  final n = vertices.length;
  if (n == 0) return null;

  // Box2D keeps a point only when it is a weld tolerance from every one
  // before it.
  var kept = 0;
  for (var i = 0; i < n && kept < 3; i++) {
    var unique = true;
    for (var j = 0; j < i; j++) {
      if ((vertices[i] - vertices[j]).distance < _weldTolerance) {
        unique = false;
        break;
      }
    }
    if (unique) kept++;
  }

  // The narrowest way across: square to one of the sides, for a convex
  // polygon.
  var width = double.infinity;
  var along = const Offset(1, 0);
  for (var i = 0; i < n; i++) {
    final side = vertices[(i + 1) % n] - vertices[i];
    final length = side.distance;
    if (length < 1e-9) continue;
    final d = side / length;
    var lo = double.infinity, hi = double.negativeInfinity;
    for (final v in vertices) {
      final across = d.dx * v.dy - d.dy * v.dx;
      lo = math.min(lo, across);
      hi = math.max(hi, across);
    }
    if (hi - lo < width) {
      width = hi - lo;
      along = d;
    }
  }
  if (width == double.infinity) width = 0; // Every point in one place.

  // A margin over the tolerance covers the float32 rounding of the points.
  if (kept >= 3 && width >= _weldTolerance + 1e-3) return null;

  final across = Offset(-along.dy, along.dx);
  var aLo = double.infinity, aHi = double.negativeInfinity;
  var bLo = double.infinity, bHi = double.negativeInfinity;
  for (final v in vertices) {
    final a = v.dx * along.dx + v.dy * along.dy;
    final b = v.dx * across.dx + v.dy * across.dy;
    aLo = math.min(aLo, a);
    aHi = math.max(aHi, a);
    bLo = math.min(bLo, b);
    bHi = math.max(bHi, b);
  }
  final half = (bHi - bLo) / 2, middle = (bLo + bHi) / 2;
  final a1 = aLo + half, a2 = aHi - half;
  if (a2 <= a1) {
    // No longer than it is wide: a circle on its middle.
    final c = along * ((aLo + aHi) / 2) + across * middle;
    return (c, c, math.max(half, (aHi - aLo) / 2));
  }
  return (along * a1 + across * middle, along * a2 + across * middle, half);
}

/// A capsule [radius] round the line from [c1] to [c2], or a circle where
/// the line is too short for one: Box2D makes no capsule whose line is a
/// linear slop long or less.
void _addCapsule(
  int handle,
  Offset c1,
  Offset c2,
  double radius,
  double density,
  double friction,
  double restitution,
) {
  final length = (c2 - c1).distance;
  // A margin over the slop covers the float32 rounding of the points.
  if (length <= _linearSlop + 1e-3) {
    final c = (c1 + c2) / 2;
    box2d.b2w_addCircleShape(
      handle,
      c.dx,
      c.dy,
      radius + length / 2,
      density,
      friction,
      restitution,
    );
    return;
  }
  box2d.b2w_addCapsuleShape(
    handle,
    c1.dx,
    c1.dy,
    c2.dx,
    c2.dy,
    radius,
    density,
    friction,
    restitution,
  );
}

/// A line from [p1] to [p2]: a capsule [radius] round it, or a bare
/// segment when [radius] is 0.
void _addLine(
  int handle,
  Offset p1,
  Offset p2,
  double radius,
  double density,
  double friction,
  double restitution,
) {
  if (radius > 0) {
    _addCapsule(handle, p1, p2, radius, density, friction, restitution);
  } else {
    box2d.b2w_addSegmentShape(
      handle,
      p1.dx,
      p1.dy,
      p2.dx,
      p2.dy,
      density,
      friction,
      restitution,
    );
  }
}

/// [points] (x, y pairs) in the opposite order.
Float32List _reversedPoints(Float32List points) {
  final n = points.length ~/ 2;
  final out = Float32List(points.length);
  for (var i = 0; i < n; i++) {
    out[i * 2] = points[(n - 1 - i) * 2];
    out[i * 2 + 1] = points[(n - 1 - i) * 2 + 1];
  }
  return out;
}

/// [shape]'s points as Box2D takes a chain: at least 4, and an open
/// chain's first and last are ghosts that only shape its ends. The
/// pure-Dart engine collides with every segment of an open chain, so a
/// ghost is added beyond each end (the end segment carried on) and both
/// backends collide with the same segments; a three-point loop gets a
/// midpoint. Points closer than Box2D's linear slop to the one before are
/// dropped first (a segment of no length has no normal). Null when there
/// is no chain to make.
Float32List? _chainPoints(Chain2DShape shape) {
  const slop = _linearSlop;
  final v = <Offset>[];
  for (final p in shape.vertices) {
    if (v.isEmpty || (p - v.last).distance > slop) v.add(p);
  }
  if (shape.loop && v.length > 1 && (v.first - v.last).distance <= slop) {
    v.removeLast();
  }
  final List<Offset> points;
  if (shape.loop) {
    if (v.length < 3) return null;
    points = v.length == 3 ? [v[0], (v[0] + v[1]) / 2, v[1], v[2]] : v;
  } else {
    if (v.length < 2) return null;
    final last = v.length - 1;
    points = [v[0] + (v[0] - v[1]), ...v, v[last] + (v[last] - v[last - 1])];
  }
  final out = Float32List(points.length * 2);
  for (var i = 0; i < points.length; i++) {
    out[i * 2] = points[i].dx;
    out[i * 2 + 1] = points[i].dy;
  }
  return out;
}
