import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  // ── Physics2DBody ───────────────────────────────────────────────────────────

  group('Physics2DBody', () {
    test('inverseMass is 1/mass for positive mass', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        mass: 4.0,
      );
      expect(body.inverseMass, closeTo(0.25, 1e-9));
    });

    test('inverseMass is 0 for static body (mass=0)', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        mass: 0.0,
      );
      expect(body.inverseMass, 0.0);
    });

    test('inverseInertia is 0 when inertia is 0', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        inertia: 0.0,
      );
      expect(body.inverseInertia, 0.0);
    });

    test('applyForce accumulates into acceleration', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        mass: 2.0,
      );
      body.applyForce(Vector2(10, 0));
      // a += F * inverseMass = 10 * 0.5 = 5
      expect(body.acceleration.x, closeTo(5.0, 1e-9));
    });

    test('applyForce does nothing for static body', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        mass: 0.0,
      );
      body.applyForce(Vector2(999, 999));
      expect(body.acceleration.x, 0.0);
      expect(body.acceleration.y, 0.0);
    });

    test('applyTorque accumulates', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        inertia: 2.0,
      );
      body.applyTorque(10.0);
      expect(body.torque, closeTo(10.0, 1e-9));
    });

    test('applyTorque does nothing when inertia is 0', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        inertia: 0.0,
      );
      body.applyTorque(999.0);
      expect(body.torque, 0.0);
    });
  });

  group('forces', () {
    test('a force wakes a sleeping dynamic body', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        isAwake: false,
      )..sleepTimer = 0.4;
      body.applyForce(Vector2(1, 0));
      expect(body.isAwake, isTrue);
      expect(body.sleepTimer, 0);
      body.isAwake = false;
      body.applyTorque(1);
      expect(body.isAwake, isTrue);
    });

    test('a kinematic body takes no force and no torque', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(1),
        bodyType: Body2DType.kinematic,
        isAwake: false,
      );
      body
        ..applyForce(Vector2(10, 0))
        ..applyTorque(10);
      expect(body.acceleration.x, 0);
      expect(body.torque, 0);
      expect(body.isAwake, isFalse);
    });
  });

  group('shapes', () {
    test('boundsAt covers every shape', () {
      final body = Physics2DBody(
        position: Vector2.zero(),
        shape: Circle2DShape(5),
        additionalShapes: [Circle2DShape(5, center: const Offset(20, 0))],
      );
      final bounds = body.boundsAt(const Offset(100, 0));
      expect(bounds.left, closeTo(95, 1e-9));
      expect(bounds.right, closeTo(125, 1e-9));
    });

    test('proxies are its shapes, turned with it and shifted', () {
      final body = Physics2DBody(
        position: Vector2(100, 0),
        shape: Circle2DShape(5, center: const Offset(10, 0)),
        additionalShapes: [Rectangle2DShape(4, 2)],
        angle: math.pi / 2,
      );
      final proxies = body.proxies(shift: const Offset(0, 50));
      expect(proxies, hasLength(2));
      // The circle's centre, (10, 0) from the body, turned a quarter.
      expect(proxies[0].points.single.dx, closeTo(100, 1e-9));
      expect(proxies[0].points.single.dy, closeTo(60, 1e-9));
      expect(proxies[0].radius, 5);
      expect(proxies[1].points, hasLength(4));
      expect(proxies[1].radius, 0);
    });
  });
}
