/// The native Box2D backend's debug drawing.
library;

import 'dart:ui' show Canvas, Color, Offset, Paint, PaintingStyle, Path, Rect;

import 'package:meta/meta.dart' show internal;

import '../physics_2d_engine.dart';
import 'box2d_body.dart';

/// Draws [bodies] on [canvas] where Box2D has them, turned with them:
/// static bodies grey, moving ones green, extra shapes blue.
@internal
void drawBox2DBodies(Canvas canvas, Map<Physics2DBody, Box2DBody> bodies) {
  final strokePaint = Paint()
    ..color = const Color(0xFF00FF88)
    ..style = PaintingStyle.stroke
    ..strokeWidth = 1.5;

  final staticPaint = Paint()
    ..color = const Color(0xFF888888)
    ..style = PaintingStyle.stroke
    ..strokeWidth = 1.5;

  final extraPaint = Paint()
    ..color = const Color(0xFF00CCFF)
    ..style = PaintingStyle.stroke
    ..strokeWidth = 1.0;

  for (final entry in bodies.entries) {
    final body = entry.key;
    final b2 = entry.value;
    final paint = body.mass <= 0.0 ? staticPaint : strokePaint;

    canvas.save();
    canvas.translate(b2.currentX, b2.currentY);
    canvas.rotate(b2.currentAngle);

    _drawShape(canvas, body.shape, paint);
    for (final extra in body.additionalShapes) {
      _drawShape(canvas, extra, extraPaint);
    }

    canvas.restore();
  }
}

void _drawShape(Canvas canvas, Shape2D shape, Paint paint) {
  if (shape is Circle2DShape) {
    final c = shape.center;
    canvas.drawCircle(c, shape.radius, paint);
    canvas.drawLine(c, c + Offset(shape.radius, 0), paint);
  } else if (shape is Capsule2DShape) {
    canvas.drawCircle(shape.center1, shape.radius, paint);
    canvas.drawCircle(shape.center2, shape.radius, paint);
    canvas.drawLine(shape.center1, shape.center2, paint);
  } else if (shape is Chain2DShape) {
    for (int i = 0; i < shape.vertices.length - 1; i++) {
      canvas.drawLine(shape.vertices[i], shape.vertices[i + 1], paint);
    }
    if (shape.loop && shape.vertices.length > 1) {
      canvas.drawLine(shape.vertices.last, shape.vertices.first, paint);
    }
  } else if (shape is Segment2DShape) {
    canvas.drawLine(shape.point1, shape.point2, paint);
  } else if (shape is Rectangle2DShape) {
    canvas.drawRect(
      Rect.fromCenter(
        center: Offset.zero,
        width: shape.width,
        height: shape.height,
      ),
      paint,
    );
  } else if (shape is Polygon2DShape) {
    if (shape.vertices.isNotEmpty) {
      final path = Path()
        ..moveTo(shape.vertices.first.dx, shape.vertices.first.dy);
      for (final v in shape.vertices.skip(1)) {
        path.lineTo(v.dx, v.dy);
      }
      path.close();
      canvas.drawPath(path, paint);
    }
  }
}
