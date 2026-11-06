# Just Physics Engine

Standalone physics package for Flutter projects, focused on fast 2D simulation with collision detection, rigid bodies, spatial broad-phase, and ray helpers.

This package is part of the Just Game Engine workspace, but can be used independently.

## Features

- 2D rigid-body simulation (`Physics2DEngine`, `Physics2DBody`) on two backends with one API: Box2D v3.2 on native platforms, a pure-Dart engine on the web. Every method means the same on both, so game code needs no backend branch
- Gravity, drag, angular damping, restitution, friction, torque, and sleeping (per body: `canSleep`)
- Eight collision shapes: `Circle2DShape` (optionally off-centre), `Rectangle2DShape`, `Polygon2DShape`, `RoundedPolygon2DShape`, `Capsule2DShape`, `Segment2DShape`, `Chain2DShape`, and compounds (`additionalShapes`)
- Body settings changeable while the game runs (`setBodyActive`, `wakeBody`, `setBodyFixedRotation`, `setBodyFriction`, `setBodyAngularDamping`, `setBodyCanSleep`, `setBodyBullet`, …)
- Joints (distance, revolute, prismatic, weld, wheel, mouse) with anchors, `collideConnected`, and limits, motors and springs set through `Joint2D` in the same units on both backends (springs in hertz)
- Static bodies never test against each other, so large tile-map levels stay cheap
- Broad phase: Box2D's tree natively; a uniform grid in the pure-Dart engine
- Collision response with impulse + friction + positional correction
- A kinematic character mover (`CharacterMover2D`): slopes, steps, one-way platforms and pushing, reporting the ground it stands on, the same on both backends
- Runtime simulation stats (`engine.stats`)
- Debug rendering (`engine.renderDebug`)
- 2D ray utility (`Ray2D`, `Ray2D.fromPoints`)
- 3D physics on [Jolt Physics](https://github.com/jrouwe/JoltPhysics) through just_jolt (`PhysicsEngineFactory.create3D()`, `Physics3DEngine`, `Physics3DWorld`): rigid bodies of every common shape, compounds, joints, characters, queries and buffered events. Native platforms only (on Android, arm64 and x86_64); an app opts in to building Jolt in its pubspec, so a 2D app neither builds nor ships it

## Getting Started

### Requirements

- Dart SDK `^3.11.0`
- Flutter `>=3.47.0`

### Add Dependency

Use the package from pub.dev (2.0.0 renames the API after the Just Game Engine's 2D/3D naming; see [Migrating from 1.x](#migrating-from-1x)):

```yaml
dependencies:
  just_physics_engine: ^2.0.0
  just_dart: ^0.3.0 # Vector2 and the other maths types
```

Or add it with Flutter tooling:

```bash
flutter pub add 'just_physics_engine:^2.0.0'
```

Or use a local path dependency in a monorepo:

```yaml
dependencies:
  just_physics_engine:
    path: ../packages/just_physics_engine
```

Then fetch packages:

```bash
flutter pub get
```

## Usage

### Basic 2D World

```dart
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  // Box2D on native platforms, the pure-Dart engine on the web.
  final engine = PhysicsEngineFactory.create2D()..initialize();

  final floor = Physics2DBody(
    position: Vector2(200, 500),
    shape: Rectangle2DShape(600, 40),
    mass: 0, // static body
    restitution: 0.2,
    friction: 0.8,
  );

  final ball = Physics2DBody(
    position: Vector2(220, 120),
    shape: Circle2DShape(18),
    mass: 1.0,
    restitution: 0.55,
    friction: 0.35,
    drag: 0.02,
  );

  engine.addBody(floor);
  engine.addBody(ball);

  // Fixed step example (60 Hz)
  const dt = 1.0 / 60.0;
  engine.update(dt);

  print('Bodies: ${engine.bodyCount}');
  print('Ball at ${ball.position}');

  engine.dispose();
}
```

### Apply Forces

```dart
import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void kickBody(Physics2DEngine engine, Physics2DBody body) {
  // For the next step; a sleeping body wakes.
  body.applyForce(Vector2(1500, -800));
  body.applyTorque(20);
  // At once: knockback, bounce pads.
  engine.applyLinearImpulse(body, 8, -3);
}
```

### Joints

Anchors are world points, as the bodies are when the joint is made. A joint's two bodies pass through each other unless `collideConnected` says otherwise, on both backends. Limits, motors and springs go through `Joint2D`, in the same units on both: springs in hertz and a damping ratio, as Box2D and the 3D joints take them.

```dart
import 'dart:ui';

import 'package:just_physics_engine/just_physics_engine.dart';

void hang(Physics2DEngine engine, Physics2DBody beam, Physics2DBody lamp) {
  // A bungee from the beam's end to the top of the lamp: 2 Hz.
  engine.addDistanceJoint(
    beam,
    lamp,
    anchorA: const Offset(50, 0),
    anchorB: const Offset(50, 35),
    springHertz: 2,
    dampingRatio: 0.3,
  );
}

void door(Physics2DEngine engine, Physics2DBody wall, Physics2DBody door) {
  engine.addRevoluteJoint(wall, door, const Offset(0, 0))
    // Radians from the angle it was made at.
    ..setLimits(-1.5, 0)
    ..setMotor(0.5, 200);
}
```

### Body Settings

```dart
import 'dart:ui';

import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void addPlayer(Physics2DEngine engine) {
  final player = Physics2DBody(
    position: Vector2(100, 100),
    // Feet: a circle below the body's position. The native backend turns
    // it with the body; the pure-Dart backend keeps it where it is.
    shape: Circle2DShape(10, center: const Offset(0, 14)),
    angularDamping: 2.0, // spin dies down on its own, as drag does for velocity
    canSleep: false, // stays awake while standing still
  );
  engine.addBody(player);

  // Change settings through the engine while the game runs, so the native
  // backend sees the change too.
  engine.setBodyAngularDamping(player, 0.5);
  engine.setBodyCanSleep(player, true);
  engine.setBodyBullet(player, true); // continuous collision (native only)
}
```

### Polygon Body

```dart
import 'dart:ui';

import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

final crate = Physics2DBody(
  position: Vector2(400, 260),
  shape: Polygon2DShape([
    const Offset(-20, -20),
    const Offset(20, -20),
    const Offset(20, 20),
    const Offset(-20, 20),
  ]),
  mass: 2,
);
```

### 2D Ray

```dart
import 'dart:ui';

import 'package:just_physics_engine/just_physics_engine.dart';

void main() {
  final ray = Ray2D.fromPoints(
    const Offset(10, 10),
    const Offset(250, 150),
  );

  final samplePoint = ray.at(120);
  print(samplePoint);
}
```

### Debug Rendering in a CustomPainter

```dart
import 'package:flutter/material.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

class PhysicsDebugPainter extends CustomPainter {
  PhysicsDebugPainter(this.engine);

  final Physics2DEngine engine;

  @override
  void paint(Canvas canvas, Size size) {
    engine.debugRender = true;
    engine.renderDebug(canvas, size);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}
```

### Character Mover

A character moved by gameplay rather than by the simulation: a capsule or a box that slides along walls and slopes, climbs steps and stands on one-way platforms. It is not a body; keep one where it is (and pass it as `Mover2DFilter.ignore`) if sensors should see it.

```dart
import 'dart:ui';

import 'package:just_physics_engine/just_physics_engine.dart';

final engine = PhysicsEngineFactory.create2D()..initialize();
final mover = CharacterMover2D(engine);
final shape = Mover2DShape.box(width: 32, height: 48);
var position = const Offset(0, 0), velocity = Offset.zero;
var grounded = false;

void step(double dt, double run, bool jump) {
  var vy = grounded && velocity.dy > 0 ? 0.0 : velocity.dy;
  if (jump && grounded) vy = -800; // +Y is down
  vy += 2000 * dt;
  final move = mover.move(
    shape: shape,
    position: position,
    velocity: Offset(run, vy),
    dt: dt,
    stepHeight: 8,
    stickToFloor: 8,
    wasGrounded: grounded,
  );
  position = move.position;
  velocity = move.velocity;
  grounded = move.isGrounded;
}
```

It moves no ground with it: to ride a moving platform, add the platform's velocity (`move.groundBody`) to the character's.

### 3D

3D needs Jolt built into the app. Ask for it in the app's `pubspec.yaml`:

```yaml
hooks:
  user_defines:
    just_jolt:
      build: true
```

Without it, and on the web and 32-bit Android, `PhysicsEngineFactory.create3D()` throws a `Physics3DUnsupportedError` saying why. Units are metres, seconds and radians, and **+Y is down**: gravity is (0, +9.81, 0) and a character's up is (0, −1, 0).

```dart
import 'package:just_physics_engine/physics_3d.dart';

void main() {
  final engine = PhysicsEngineFactory.create3D();
  final world = engine.createWorld(Physics3DWorldSettings());
  world.createBody(
    const Box3DShape(10, 0.5, 10),
    Body3DSettings(position: Vector3(0, 0.5, 0), type: Body3DType.static),
  );
  final crate = world.createBody(
    const Box3DShape(0.5, 0.5, 0.5),
    Body3DSettings(position: Vector3(0, -5, 0)),
  );
  for (var i = 0; i < 120; i++) {
    world.step(1 / 60);
  }
  final poses = Body3DPoses();
  world.readBodies([crate], poses);
  final at = Vector3.zero();
  poses.positionInto(0, at);
  print('The crate rests at $at'); // On the floor: y is about -0.5.
  world.dispose();
}
```

A world's collision grid can change while it runs (`world.setLayerMasks`). Read events and character contacts every step with `readEvents` and `readCharacterContacts`, which fill packed buffers (`Physics3DEvents`, `Character3DContacts`) and make no object per event; `drainEvents` and `characterContacts` give objects, for tests and code that runs rarely.

`package:just_physics_engine/testing.dart` has `FakePhysics3DEngine`, a plain-Dart engine for tests that need no native code. `package:just_physics_engine/jolt.dart` (`JoltWorld`, `JoltShapes`) imports `dart:ffi` and is for native code only. just_jolt's own notes apply: on Windows the app's folder path must stay under about 175 characters for Jolt to build.

## Migrating from 1.x

2.0.0 renames the public API after the Just Game Engine's naming rule, gives both 2D backends one API (game code needs no backend branch), and fixes what behaved differently between them.

- **The 2D or 3D tag goes right after the subject**: `PhysicsEngine3D` is `Physics3DEngine`, `PhysicsWorld3D` is `Physics3DWorld`, `BoxShape3D` is `Box3DShape`, `GroundState2D` is `Ground2DState`, `QueryHit3D` is `Query3DHit`.
- **A 2D type with a 3D twin says its dimension too**: `PhysicsEngine` is `Physics2DEngine`, `PhysicsBody` is `Physics2DBody`, `CollisionShape` is `Shape2D` (the twin of `Shape3D`), `CircleShape` is `Circle2DShape` and so on for every shape, `BodyType` is `Body2DType`, `Ray` is `Ray2D`.
- **Their families follow**: the joints (`JointConstraint` is `Joint2D`, `DistanceJoint` is `DistanceJoint2D`, …), the mover's types (`MoverShape` is `Mover2DShape`, …) and the query results (`RayBodyHit` is `Ray2DHit`, `ShapeCastResult` is `CircleCast2DHit`).
- **One word for joints in both dimensions**: `JointType` is `Joint2DKind`, `JointKind3D` is `Joint3DKind`, and a 2D joint's `type` is its `kind`.
- **Members**: `castRay` and `castRayAll` are `raycast` and `raycastAll`, `castCircle` is `circleCast`, `PhysicsEngineFactory.create()` is `create2D()`, `Box2DPhysicsEngine.physicsBodyFromHandle` is `bodyFromHandle`, `castShape2D` is `shapeCast2D`.
- **The platform's engine** comes from `PhysicsEngineFactory.create2D()`; the unnamed `Physics2DEngine()` constructor is gone (`Physics2DEngine.pureDart()` stays).
- **Members say what they give**: `Shape2D.getBounds(p)` is `boundsAt(p)`, `getManifold` is `manifoldWith`, `Physics2DBody.getCompoundBounds(p)` is `Physics2DBody.boundsAt(p)`. `engine.isNative` says whether Box2D runs.
- **Joints**: configure them through `Joint2D` (`setLimits`, `setMotor`, `setSpring`, `reactionForce`, `reactionTorque`) on both backends; the pure-Dart joints' fields and `Box2DJoint`'s kind-named setters are gone, and so are `Box2DPhysicsEngine.create*Joint` and `destroyJoint` (use `add*Joint` and `removeJoint`). Springs are in hertz with a damping ratio on both backends (`addDistanceJoint(springHertz:, dampingRatio:)`; `stiffness`/`damping` are gone). `add*Joint` take anchors and `collideConnected`.
- **Removed**: `RigidBody`, a shapeless point mass nothing used; `CollisionDetector`, `ForceManager`, `PhysicsGameLoop`, `TransformInterpolator`, `Physics2DBody.applyImpulse` (use `engine.applyLinearImpulse`), `Physics2DBody.checkCollision` (use `setBodyActive` or the filter), the polygon cache. `Box2DWorld`, `Box2DBody`, `SpatialGrid` and `BodyPair` are internal.

The full old-to-new table, and what now behaves the same on both backends, is in the [CHANGELOG](CHANGELOG.md). Enum values did not change, so anything saved by name (body types, one-way directions, joint kinds) reads as before. The imports are the same libraries.

## Notes

- For deterministic gameplay, use a fixed timestep (for example, `1/60`) rather than variable-frame integration.
- Use the native engines from one isolate per process: Box2D's and Jolt's worlds belong to the whole process. Call `dispose()` on what you make; the finalizers are a safety net that runs late.

## Compatibility

- Version `2.0.0`
- Native Box2D comes from [just_box_2d](https://pub.dev/packages/just_box_2d) ^0.2.0, built from source by its own hook (`hooks` ^2, the same generation as `just_graphics_engine`, so both resolve in one app). An app that wants the pure-Dart engine everywhere turns that build off:

  ```yaml
  hooks:
    user_defines:
      just_box_2d:
        build: false
  ```
- Dart SDK: `^3.11.0`
- Flutter: `>=3.47.0`
- Platforms: Android, iOS, Linux, macOS, Web, Windows
- Native platforms use the Box2D FFI backend (Box2D v3.2, through just_box_2d): the library is found by its code asset id, not by file name. 3D uses [just_jolt](https://pub.dev/packages/just_jolt) ^0.2.0. This package itself is plain Dart and compiles no C.
- Web and WASM targets use the pure-Dart backend.

## Development

Inside this package directory:

```bash
flutter pub get
flutter analyze
flutter test
```

## Project Docs

- Architecture: [ARCHITECTURE.md](ARCHITECTURE.md)
- API Reference: [API.md](API.md)
- Changelog: [CHANGELOG.md](CHANGELOG.md)
- Contributing: [CONTRIBUTING.md](https://github.com/just-unknown-dev/just-physics-engine/blob/main/CONTRIBUTING.md)
- Code of Conduct: [CODE_OF_CONDUCT.md](https://github.com/just-unknown-dev/just-physics-engine/blob/main/CODE_OF_CONDUCT.md)
