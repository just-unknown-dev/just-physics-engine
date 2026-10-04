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
- Runtime simulation stats (`engine.stats`)
- Debug rendering (`engine.renderDebug`)
- 2D ray utility (`Ray`, `Ray.fromPoints`)
- 3D API stubs (`PhysicsEngine3D`)

## Getting Started

### Requirements

- Dart SDK `^3.11.0`
- Flutter `>=3.27.0`

### Add Dependency

Use the package from pub.dev:

```yaml
dependencies:
	just_physics_engine: ^1.3.0
	just_dart: ^0.2.0 # Vector2 and the other maths types
```

Or add it with Flutter tooling:

```bash
flutter pub add just_physics_engine
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

	final stats = engine.stats;
	print('Bodies: ${stats['bodyCount']}');
	print('Collisions resolved: ${stats['resolvedCollisions']}');

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

final ray = Ray.fromPoints(
	const Offset(10, 10),
	const Offset(250, 150),
);

final samplePoint = ray.at(120);
print(samplePoint);
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

## Notes

- The 2D engine is the production-ready part of this package.
- 3D classes are currently scaffolding/stubs and do not provide full simulation yet.
- For deterministic gameplay, use a fixed timestep (for example, `1/60`) rather than variable-frame integration.

## Compatibility

> **Known blocker for 3-D:** the native build hook pins `hooks: ^1.0.0` and
> `native_toolchain_c: ^0.17.0`. `just_graphics_engine` (the flutter_gpu
> renderer) needs `hooks: ^2.0.0`, so the two cannot resolve in one app
> until this package moves to `hooks ^2` / `code_assets ^1.2` /
> `native_toolchain_c ^0.19` — planned with the 3-D release of the engine.

- Version `1.3.0`
- Dart SDK: `^3.11.0`
- Flutter: `>=3.27.0`
- Platforms: Android, iOS, Linux, macOS, Web, Windows
- Native platforms use the Box2D FFI backend when available.
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
- Contributing: [CONTRIBUTING.md](CONTRIBUTING.md)
- Code of Conduct: [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)
