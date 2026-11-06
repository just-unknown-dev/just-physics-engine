part of 'physics_2d_engine.dart';

// The pure-Dart engine's debug drawing (Physics2DEngine.renderDebug): each
// body's shapes where its collisions see them — at its position, unturned —
// its velocity and its centre. Inactive bodies are drawn red.

final Paint _debugActivePaint = Paint()
  ..color = Colors.green
  ..style = PaintingStyle.stroke
  ..strokeWidth = 2.0;
final Paint _debugInactivePaint = Paint()
  ..color = Colors.red
  ..style = PaintingStyle.stroke
  ..strokeWidth = 2.0;
final Paint _debugVelocityPaint = Paint()
  ..color = Colors.blue
  ..strokeWidth = 2.0;
final Paint _debugCenterPaint = Paint()..color = Colors.red;

final Paint _debugExtraShapePaint = Paint()
  ..color = const Color(0xFF00CCFF)
  ..strokeWidth = 1.0;

/// Draws [bodies] on [canvas].
void _renderDebugBodies(Canvas canvas, Iterable<Physics2DBody> bodies) {
  for (final body in bodies) {
    final paint = body.isActive ? _debugActivePaint : _debugInactivePaint;
    _renderBodyShape(canvas, body, body.shape, paint);
    for (final extra in body.additionalShapes) {
      _renderBodyShape(canvas, body, extra, _debugExtraShapePaint);
    }

    if (body.velocity.lengthSquared > 0) {
      canvas.drawLine(
        body.position.toOffset(),
        Offset(
          body.position.x + body.velocity.x * 0.1,
          body.position.y + body.velocity.y * 0.1,
        ),
        _debugVelocityPaint,
      );
    }

    canvas.drawCircle(body.position.toOffset(), 3, _debugCenterPaint);
  }
}

void _renderBodyShape(
  Canvas canvas,
  Physics2DBody body,
  Shape2D shape,
  Paint paint,
) {
  if (shape is Circle2DShape) {
    canvas.drawCircle(
      shape.centerAt(body.position.toOffset()),
      shape.radius,
      paint,
    );
  } else if (shape is Capsule2DShape) {
    final wa1 = Offset(
      body.position.x + shape.center1.dx,
      body.position.y + shape.center1.dy,
    );
    final wa2 = Offset(
      body.position.x + shape.center2.dx,
      body.position.y + shape.center2.dy,
    );
    canvas.drawCircle(wa1, shape.radius, paint);
    canvas.drawCircle(wa2, shape.radius, paint);
    canvas.drawLine(wa1, wa2, paint);
  } else if (shape is Chain2DShape) {
    for (int i = 0; i < shape.vertices.length - 1; i++) {
      canvas.drawLine(
        Offset(
          body.position.x + shape.vertices[i].dx,
          body.position.y + shape.vertices[i].dy,
        ),
        Offset(
          body.position.x + shape.vertices[i + 1].dx,
          body.position.y + shape.vertices[i + 1].dy,
        ),
        paint,
      );
    }
    if (shape.loop && shape.vertices.length > 1) {
      canvas.drawLine(
        Offset(
          body.position.x + shape.vertices.last.dx,
          body.position.y + shape.vertices.last.dy,
        ),
        Offset(
          body.position.x + shape.vertices.first.dx,
          body.position.y + shape.vertices.first.dy,
        ),
        paint,
      );
    }
  } else if (shape is Segment2DShape) {
    canvas.drawLine(
      Offset(
        body.position.x + shape.point1.dx,
        body.position.y + shape.point1.dy,
      ),
      Offset(
        body.position.x + shape.point2.dx,
        body.position.y + shape.point2.dy,
      ),
      paint,
    );
  } else if (shape is Polygon2DShape) {
    final path = Path();
    if (shape.vertices.isNotEmpty) {
      path.moveTo(
        body.position.x + shape.vertices[0].dx,
        body.position.y + shape.vertices[0].dy,
      );
      for (int i = 1; i < shape.vertices.length; i++) {
        path.lineTo(
          body.position.x + shape.vertices[i].dx,
          body.position.y + shape.vertices[i].dy,
        );
      }
      path.close();
    }
    canvas.drawPath(path, paint);
  }
}
