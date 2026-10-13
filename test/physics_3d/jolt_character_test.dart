// Jolt's CharacterVirtual through the engine, in a +Y-down world: stairs,
// slopes, ceilings and moving platforms, the way PhysicsSystem3D drives it.

import 'dart:math' as math;

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/physics_3d.dart';

const _dt = 1 / 60;
final _backend = JoltPhysicsEngine();
final _up = Vector3(0, -1, 0);

PhysicsWorld3D _world({List<int>? masks}) {
  final world = _backend.createWorld(PhysicsWorldSettings3D(layerMasks: masks));
  addTearDown(world.dispose);
  return world;
}

int _static(
  PhysicsWorld3D w,
  Shape3D shape,
  Vector3 at, {
  Quaternion? rotation,
}) => w.createBody(
  shape,
  BodySettings3D(position: at, rotation: rotation, type: BodyType3D.static),
);

/// A 40 m floor whose top is at y = 0.
int _floor(PhysicsWorld3D w) =>
    _static(w, const BoxShape3D(20, 0.5, 20), Vector3(0, 0.5, 0));

/// Moves a character as PhysicsSystem3D does: on the ground it moves with
/// the ground (and may jump), in the air it keeps its vertical speed; then
/// gravity, then the requested movement.
final class _Walker {
  _Walker(this.w, Vector3 at, {int layer = 0, bool innerBody = true})
    : character = w.createCharacter(
        CharacterSettings3D(layer: layer, innerBody: innerBody),
        at,
      ) {
    w.characterState(character, state);
  }

  final PhysicsWorld3D w;
  final int character;
  final CharacterState3D state = CharacterState3D();

  Vector3 get position => state.position;

  void step(Vector3 move, {double jump = 0, bool stepWorld = true}) {
    final vertical = state.velocity.dot(_up);
    final groundVertical = state.groundVelocity.dot(_up);
    final v = Vector3.zero();
    if (state.isGrounded && vertical - groundVertical < 0.1) {
      v.setFrom(state.groundVelocity);
      if (jump > 0) v.addScaled(_up, jump);
    } else {
      v
        ..setFrom(_up)
        ..scale(vertical);
    }
    v.addScaled(w.gravity, _dt);
    v.add(move);
    w.moveCharacter(
      character,
      _dt,
      v,
      state,
      stepDown: Vector3(0, 0.5, 0),
      stepUp: Vector3(0, -0.4, 0),
    );
    if (stepWorld) w.step(_dt);
  }

  void walk(int frames, Vector3 move) {
    for (var i = 0; i < frames; i++) {
      step(move);
    }
  }
}

/// A ramp rising towards +X at [angle] whose top surface meets the floor at
/// x = [startX].
int _ramp(PhysicsWorld3D w, double angle, double startX) {
  final c = math.cos(angle), s = math.sin(angle);
  return _static(
    w,
    const BoxShape3D(3, 0.1, 2),
    Vector3(startX + 3 * c + 0.1 * s, 0.1 * c - 3 * s, 0),
    rotation: Quaternion(0, 0, math.sin(-angle / 2), math.cos(-angle / 2)),
  );
}

void main() {
  test('stands on the floor with its feet at its position', () {
    final w = _world();
    _floor(w);
    final hero = _Walker(w, Vector3(0, -0.5, 0));
    hero.walk(60, Vector3.zero());
    expect(hero.state.groundState, GroundState3D.onGround);
    expect(hero.position.y, closeTo(0, 0.03));
    expect(
      hero.state.groundNormal.y,
      closeTo(-1, 1e-3),
      reason: 'the ground faces up',
    );
  });

  test('climbs stairs of 0.2 m steps', () {
    final w = _world();
    _floor(w);
    // Five steps, 0.2 m up and 0.5 m deep each, from x = 2.
    for (var k = 1; k <= 5; k++) {
      final x0 = 2 + 0.5 * (k - 1);
      _static(w, BoxShape3D(2.5, 0.1 * k, 2), Vector3(x0 + 2.5, -0.1 * k, 0));
    }
    final hero = _Walker(w, Vector3.zero());
    hero.walk(150, Vector3(2, 0, 0));
    expect(hero.position.x, greaterThan(4.5), reason: 'past the last step');
    expect(hero.position.y, closeTo(-1, 0.05), reason: 'on top of the stairs');
    expect(hero.state.isGrounded, isTrue);
  });

  test('a step too tall is a wall', () {
    final w = _world();
    _floor(w);
    _static(w, const BoxShape3D(2, 0.4, 2), Vector3(4, -0.4, 0)); // 0.8 m tall
    final hero = _Walker(w, Vector3.zero());
    hero.walk(120, Vector3(2, 0, 0));
    expect(hero.position.x, closeTo(2 - 0.3, 0.05), reason: 'stopped by it');
    expect(hero.position.y, closeTo(0, 0.03));
    final contacts = w.characterContacts(hero.character);
    expect(
      contacts.any((c) => c.normal.x < -0.9),
      isTrue,
      reason: 'touching the wall',
    );
  });

  test('walks up a 30° slope but not a 60° one', () {
    double climb(double degrees) {
      final w = _world();
      _floor(w);
      _ramp(w, degrees * math.pi / 180, 1);
      final hero = _Walker(w, Vector3.zero());
      hero.walk(180, Vector3(2, 0, 0));
      return -hero.position.y;
    }

    expect(climb(30), greaterThan(0.8));
    expect(climb(60), lessThan(0.3));
  });

  test('a ceiling stops a jump', () {
    final w = _world();
    _floor(w);
    // A ceiling whose underside is 2.2 m up; the character is 1.8 m tall.
    _static(w, const BoxShape3D(5, 0.25, 5), Vector3(0, -2.45, 0));
    final hero = _Walker(w, Vector3.zero());
    hero.walk(10, Vector3.zero());
    hero.step(Vector3.zero(), jump: 6);
    var highest = 0.0;
    var frames = 0;
    while (frames < 120) {
      hero.step(Vector3.zero());
      frames++;
      highest = math.min(highest, hero.position.y);
      if (hero.state.isGrounded) break;
    }
    expect(highest, greaterThan(-0.45), reason: 'head against the ceiling');
    expect(highest, lessThan(-0.3));
    expect(hero.state.isGrounded, isTrue);
    expect(
      frames * _dt,
      lessThan(1.0),
      reason: 'shorter than the 1.22 s of an open jump',
    );
  });

  test('rides a moving platform', () {
    final w = _world();
    final platform = w.createBody(
      const BoxShape3D(2, 0.25, 2),
      BodySettings3D(position: Vector3(0, 0.25, 0), type: BodyType3D.kinematic),
    );
    final hero = _Walker(w, Vector3.zero());
    hero.walk(10, Vector3.zero());
    final start = hero.position.x;
    for (var i = 1; i <= 60; i++) {
      w.moveKinematic(
        platform,
        Vector3(i * 2 * _dt, 0.25, 0),
        Quaternion.identity(),
        _dt,
      );
      hero.step(Vector3.zero());
    }
    expect(hero.state.groundBody, platform);
    expect(hero.position.x - start, closeTo(2, 0.1), reason: 'carried 2 m');
    expect(hero.state.isGrounded, isTrue);
  });

  test('sensors detect it and rays hit it, through its own body', () {
    final w = _world();
    _floor(w);
    final coin = w.createBody(
      const SphereShape3D(0.3),
      BodySettings3D(
        position: Vector3(2, -1, 0),
        type: BodyType3D.static,
        isSensor: true,
      ),
    );
    final hero = _Walker(w, Vector3.zero());
    final body = w.characterBody(hero.character);
    expect(body, isNot(0));
    final events = <PhysicsEvent3D>[];
    for (var i = 0; i < 90; i++) {
      hero.step(Vector3(2, 0, 0));
      events.addAll(w.drainEvents());
    }
    final enter = events.firstWhere(
      (e) => e.type == PhysicsEventType3D.sensorBegin,
    );
    expect({enter.bodyA, enter.bodyB}, {coin, body});

    final hit = w.raycast(Vector3(hero.position.x, -1, -5), Vector3(0, 0, 10));
    expect(hit.single.body, body);
    expect(
      w.raycast(
        Vector3(hero.position.x, -1, -5),
        Vector3(0, 0, 10),
        filter: const QueryFilter3D(includeCharacters: false),
      ),
      isEmpty,
    );
  });

  test('teleports, and two characters on colliding layers never overlap', () {
    final w = _world(masks: [0xFFFF, 0xFFFF & ~2]); // Layer 1 ignores itself.
    _floor(w);
    final a = _Walker(w, Vector3.zero());
    final b = _Walker(w, Vector3(2, 0, 0));
    for (var i = 0; i < 90; i++) {
      a.step(Vector3(2, 0, 0), stepWorld: false);
      b.step(Vector3.zero());
    }
    // A walks into B: Jolt pushes B along rather than walking through it.
    expect(
      b.position.x - a.position.x,
      greaterThan(0.55),
      reason: 'two radii apart, or more',
    );

    w.setCharacterPosition(a.character, Vector3(10, 0, 0));
    w.characterState(a.character, a.state);
    expect(a.position.x, 10);

    final ghost1 = _Walker(w, Vector3(0, 0, 5), layer: 1);
    final ghost2 = _Walker(w, Vector3(2, 0, 5), layer: 1);
    for (var i = 0; i < 90; i++) {
      ghost1.step(Vector3(2, 0, 0), stepWorld: false);
      ghost2.step(Vector3.zero());
    }
    expect(ghost1.position.x, greaterThan(2.5), reason: 'passed through');
  });

  test('a dead character answers with nothing', () {
    final w = _world();
    final hero = _Walker(w, Vector3.zero());
    w.destroyCharacter(hero.character);
    expect(w.isCharacter(hero.character), isFalse);
    hero.step(Vector3(1, 0, 0));
    expect(hero.state.groundState, GroundState3D.inAir);
    expect(w.characterContacts(hero.character), isEmpty);
    expect(w.bodyCount, 0, reason: 'its body went with it');
  });
}
