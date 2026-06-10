# Just Physics Engine

Standalone physics package for Flutter projects, focused on fast 2D simulation with collision detection, rigid bodies, spatial broad-phase, and ray helpers.

This package is part of the Just Game Engine workspace, but can be used independently.

## Features

- 2D rigid-body simulation (`PhysicsEngine`, `PhysicsBody`)
- Gravity, drag, restitution, friction, torque, and sleeping
- Collision shapes: `CircleShape`, `PolygonShape`, `RectangleShape`
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
	just_physics_engine: ^1.2.1
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
import 'package:just_physics_engine/just_physics_engine.dart';

void kickBody(PhysicsBody body) {
	body.applyForce(Vector2(1500, -800));
	body.applyImpulse(Vector2(8, -3));
	body.applyTorque(20);
}
```

### Polygon Body

```dart
import 'dart:ui';

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

- Version `1.2.1`
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
