/// The native Box2D backend's shapes: each [Shape2D] as the fixtures
/// Box2D collides the same as the pure-Dart engine does.
library;

import 'dart:ffi';
import 'dart:typed_data';
import 'dart:ui' show Offset;

import 'package:ffi/ffi.dart';
import 'package:just_box_2d/ffi.dart' as box2d;
import 'package:meta/meta.dart' show internal;

import '../physics_2d_engine.dart';

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
    box2d.b2w_addCapsuleShape(
      handle,
      shape.center1.dx,
      shape.center1.dy,
      shape.center2.dx,
      shape.center2.dy,
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
  } else if (shape is RoundedPolygon2DShape) {
    final verts = Float32List(shape.vertices.length * 2);
    for (int i = 0; i < shape.vertices.length; i++) {
      verts[i * 2] = shape.vertices[i].dx;
      verts[i * 2 + 1] = shape.vertices[i].dy;
    }
    final ptr = calloc<Float>(verts.length);
    try {
      ptr.asTypedList(verts.length).setAll(0, verts);
      box2d.b2w_addRoundedPolygonShape(
        handle,
        ptr,
        shape.vertices.length,
        shape.cornerRadius,
        density,
        friction,
        restitution,
      );
    } finally {
      calloc.free(ptr);
    }
  } else if (shape is Polygon2DShape) {
    final verts = Float32List(shape.vertices.length * 2);
    for (int i = 0; i < shape.vertices.length; i++) {
      verts[i * 2] = shape.vertices[i].dx;
      verts[i * 2 + 1] = shape.vertices[i].dy;
    }
    final ptr = calloc<Float>(verts.length);
    try {
      ptr.asTypedList(verts.length).setAll(0, verts);
      box2d.b2w_addPolygonShape(
        handle,
        ptr,
        shape.vertices.length,
        density,
        friction,
        restitution,
      );
    } finally {
      calloc.free(ptr);
    }
  }
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
    box2d.b2w_addCapsuleShape(
      handle,
      p1.dx,
      p1.dy,
      p2.dx,
      p2.dy,
      radius,
      density,
      friction,
      restitution,
    );
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
  const slop = 0.5; // B2_LINEAR_SLOP at 100 units a metre.
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
