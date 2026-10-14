# Just Physics Engine

Standalone physics package for Flutter projects, focused on fast 2D simulation with collision detection, rigid bodies, spatial broad-phase, and ray helpers.

This package is part of the Just Game Engine workspace, but can be used independently.

## Features

- 2D rigid-body simulation (`PhysicsEngine`, `PhysicsBody`)
- Gravity, drag, angular damping, restitution, friction, torque, and sleeping (per body: `canSleep`)
- Collision shapes: `CircleShape` (optionally off-centre), `PolygonShape`, `RectangleShape`
- Body settings changeable while the game runs (`setBodyFriction`, `setBodyAngularDamping`, `setBodyCanSleep`, `setBodyBullet`, …)
- Static bodies never test against each other, so large tile-map levels stay cheap
- Broad-phase collision culling via `SpatialGrid`
- Collision response with impulse + friction + positional correction
- A kinematic character mover (`CharacterMover2D`): slopes, steps, one-way platforms and pushing, reporting the ground it stands on, the same on both backends
- Runtime simulation stats (`engine.stats`)
- Debug rendering (`engine.renderDebug`)
- 2D ray utility (`Ray`, `Ray.fromPoints`)
- 3D physics on [Jolt Physics](https://github.com/jrouwe/JoltPhysics) through just_jolt (`PhysicsEngineFactory.create3D()`, `PhysicsEngine3D`, `PhysicsWorld3D`): rigid bodies of every common shape, compounds, joints, characters, queries and buffered events. Native platforms only (on Android, arm64 and x86_64); an app opts in to building Jolt in its pubspec, so a 2D app neither builds nor ships it

## Getting Started

### Requirements

- Dart SDK `^3.11.0`
- Flutter `>=3.41.0`

### Add Dependency

Use the package from pub.dev (1.4.0 adds the character mover and 3D):

```yaml
dependencies:
  just_physics_engine: ^1.4.0
  just_dart: ^0.2.0 # Vector2 and the other maths types
```

Or add it with Flutter tooling:

```bash
flutter pub add 'just_physics_engine:^1.4.0'
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
  final engine = PhysicsEngine()..initialize();

  final floor = PhysicsBody(
    position: Vector2(200, 500),
    shape: RectangleShape(600, 40),
    mass: 0, // static body
    restitution: 0.2,
    friction: 0.8,
  );

  final ball = PhysicsBody(
    position: Vector2(220, 120),
    shape: CircleShape(18),
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

void kickBody(PhysicsBody body) {
  body.applyForce(Vector2(1500, -800));
  body.applyImpulse(Vector2(8, -3));
  body.applyTorque(20);
}
```

### Body Settings

```dart
import 'dart:ui';

import 'package:just_dart/just_dart.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

void addPlayer(PhysicsEngine engine) {
  final player = PhysicsBody(
    position: Vector2(100, 100),
    // Feet: a circle below the body's position. The native backend turns
    // it with the body; the pure-Dart backend keeps it where it is.
    shape: CircleShape(10, center: const Offset(0, 14)),
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

final crate = PhysicsBody(
  position: Vector2(400, 260),
  shape: PolygonShape([
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
  final ray = Ray.fromPoints(
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

  final PhysicsEngine engine;

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

A character moved by gameplay rather than by the simulation: a capsule or a box that slides along walls and slopes, climbs steps and stands on one-way platforms. It is not a body; keep one where it is (and pass it as `MoverFilter.ignore`) if sensors should see it.

```dart
import 'dart:ui';

import 'package:just_physics_engine/just_physics_engine.dart';

final engine = PhysicsEngine()..initialize();
final mover = CharacterMover2D(engine);
final shape = MoverShape.box(width: 32, height: 48);
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
  final world = engine.createWorld(PhysicsWorldSettings3D());
  world.createBody(
    const BoxShape3D(10, 0.5, 10),
    BodySettings3D(position: Vector3(0, 0.5, 0), type: BodyType3D.static),
  );
  final crate = world.createBody(
    const BoxShape3D(0.5, 0.5, 0.5),
    BodySettings3D(position: Vector3(0, -5, 0)),
  );
  for (var i = 0; i < 120; i++) {
    world.step(1 / 60);
  }
  final poses = BodyPoses3D();
  world.readBodies([crate], poses);
  final at = Vector3.zero();
  poses.positionInto(0, at);
  print('The crate rests at $at'); // On the floor: y is about -0.5.
  world.dispose();
}
```

`package:just_physics_engine/testing.dart` has `FakePhysicsEngine3D`, a plain-Dart engine for tests that need no native code. `package:just_physics_engine/jolt.dart` (`JoltWorld`, `JoltShapes`) imports `dart:ffi` and is for native code only. just_jolt's own notes apply: on Windows the app's folder path must stay under about 175 characters for Jolt to build.

## Notes

- For deterministic gameplay, use a fixed timestep (for example, `1/60`) rather than variable-frame integration.
- Use the native engines from one isolate per process: Box2D's and Jolt's worlds belong to the whole process. Call `dispose()` on what you make; the finalizers are a safety net that runs late.

## Compatibility

- Version `1.4.0`
- Native Box2D comes from [just_box_2d](https://pub.dev/packages/just_box_2d) ^0.1.0, built from source by its own hook (`hooks` ^2, the same generation as `just_graphics_engine`, so both resolve in one app). An app that wants the pure-Dart engine everywhere turns that build off:

  ```yaml
  hooks:
    user_defines:
      just_box_2d:
        build: false
  ```
- Dart SDK: `^3.11.0`
- Flutter: `>=3.41.0`
- Platforms: Android, iOS, Linux, macOS, Web, Windows
- Native platforms use the Box2D FFI backend (Box2D v3.2, through just_box_2d): the library is found by its code asset id, not by file name. This package itself is plain Dart and compiles no C.
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
