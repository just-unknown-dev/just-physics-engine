# Just Physics Engine

Standalone physics package for Flutter projects, focused on fast 2D simulation with collision detection, rigid bodies, spatial broad-phase, and ray helpers.

This package is part of the Just Game Engine workspace, but can be used independently.

## Features

- 2D rigid-body simulation (`PhysicsEngine`, `PhysicsBody`)
- Gravity, drag, restitution, friction, torque, and sleeping
- Joint-connected wake propagation for sleeping dynamic bodies
- Collision shapes: `CircleShape`, `PolygonShape`, `RectangleShape`
- Broad-phase collision culling via `SpatialGrid`
- Collision response with impulse + friction + positional correction
- Runtime simulation stats (`engine.stats`)
- Contact warm-start cache for persistent collision pairs
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
	just_physics_engine: ^1.2.0
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
- Contact solve and warm-start behavior in `PhysicsEngine.pureDart` can be tuned with `experimentalContactVelocityIterations`, `experimentalContactTwoPointBlockNormalSolveEnabled`, `experimentalContactTwoPointBlockFrictionSolveEnabled`, `experimentalContactTwoPointBlockNormalSolveMinWarmStates`, `experimentalContactTwoPointBlockFrictionSolveMinWarmStates`, `experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates`, `experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates`, `experimentalContactBlockHysteresisTransitionRateWindowSteps`, `experimentalContactWarmStartImpulseDecay`, `experimentalContactWarmStartNormalImpulseDecay`, `experimentalContactWarmStartTangentImpulseDecay`, `experimentalContactWarmStartMaxImpulse`, `experimentalContactWarmStartMaxAgeSteps`, `experimentalContactWarmStartNormalAlignmentThreshold`, `experimentalContactWarmStartAnchorDistanceThreshold`, and `experimentalContactWarmStartManifoldSlots`.
- Warm-start anchor continuity uses manifold contact-point estimates when available, with midpoint fallback.
- Warm-start cache selection can optionally require manifold feature-id consistency via `experimentalContactWarmStartFeatureIdMatchingEnabled`.
- When a manifold provides multiple contact points, warm-start now caches each point (bounded by `experimentalContactWarmStartManifoldSlots`) for better replay continuity on edge contacts.
- Warm-start solve pre-injection now aggregates matched cached states across manifold points (`warmStartMatchedStates` in stats) while the solver transitions toward full multi-point iteration.
- Contact impulse solve now iterates per manifold point with rotational lever-arm response (linear + angular impulse) and reports `resolvedManifoldPoints` in stats for multi-point validation.
- Contact velocity solve supports configurable sequential passes via `experimentalContactVelocityIterations` and reports executed passes as `resolvedVelocityIterations` in stats.
- Optional coupled 2-point edge-contact normal solve can be enabled via `experimentalContactTwoPointBlockNormalSolveEnabled` and reports executed block passes as `resolvedBlockSolves`.
- Optional coupled 2-point edge-contact friction solve can be enabled via `experimentalContactTwoPointBlockFrictionSolveEnabled` and reports executed block passes as `resolvedBlockFrictionSolves`.
- Both block solve paths can be deferred until persistent contacts are warm-started by setting `experimentalContactTwoPointBlockNormalSolveMinWarmStates` and `experimentalContactTwoPointBlockFrictionSolveMinWarmStates`.
- Pair-level hysteresis for block solves can be configured via `experimentalContactTwoPointBlockNormalSolveDisableBelowWarmStates` and `experimentalContactTwoPointBlockFrictionSolveDisableBelowWarmStates` (enable at min-warm threshold, disable below the corresponding disable threshold).
- Runtime diagnostics include hysteresis-active contact counts: `blockNormalHysteresisActiveContacts` and `blockFrictionHysteresisActiveContacts`.
- Runtime diagnostics also expose per-step hysteresis transitions: `blockNormalHysteresisActivations`, `blockNormalHysteresisDeactivations`, `blockFrictionHysteresisActivations`, and `blockFrictionHysteresisDeactivations`.
- Reason-level transition diagnostics are also available: `blockNormalHysteresisActivatedByThreshold`, `blockNormalHysteresisDeactivatedBelowDisable`, `blockNormalHysteresisDeactivatedNonTwoPoint`, `blockFrictionHysteresisActivatedByThreshold`, `blockFrictionHysteresisDeactivatedBelowDisable`, and `blockFrictionHysteresisDeactivatedNonTwoPoint`.
- Run-level cumulative transition diagnostics are available as `totalBlockNormalHysteresisActivations`, `totalBlockNormalHysteresisDeactivations`, `totalBlockFrictionHysteresisActivations`, and `totalBlockFrictionHysteresisDeactivations`.
- Optional rolling per-step transition rates (`rollingBlockNormalHysteresisActivationRate`, `rollingBlockNormalHysteresisDeactivationRate`, `rollingBlockFrictionHysteresisActivationRate`, `rollingBlockFrictionHysteresisDeactivationRate`) are computed over `experimentalContactBlockHysteresisTransitionRateWindowSteps`.
- Warm-start pre-injection now replays per-point angular preload at cached anchors and reports `warmStartAngularPreloadCount` in stats for off-center contact continuity diagnostics.
- Warm-start cache writes now prefer per-feature impulses resolved in the current step before falling back to averaged manifold impulses (`warmStartResolvedFeatureSeeded` in stats).
- Warm-start normal/tangent impulse decay can be tuned independently (`experimentalContactWarmStartNormalImpulseDecay` and `experimentalContactWarmStartTangentImpulseDecay`) when a single shared decay is too coarse.
- Polygon-vs-polygon manifolds now use reference/incident edge clipping to estimate up to two stable contact points with paired feature ids.
- Rounded-polygon vs polygon and polygon vs capsule manifolds now also expose stable contact-point feature identifiers for better warm-start continuity.
- Warm-start caches track feature-scoped impulse history reuse (`warmStartFeatureHistoryReused` in stats) so persistent manifold points retain their own impulse memory.

## Compatibility

- Version `0.1.0`
- Dart SDK: `^3.11.0`
- Flutter: `>=1.17.0`

## Development

Inside this package directory:

```bash
flutter pub get
flutter analyze
flutter test
```

### Phase 1 Benchmark Baseline

Run deterministic benchmark baselines (pure Dart, best available backend, and
just_memory arena micro-benchmark):

```bash
flutter pub get
flutter test benchmark/phase1_deterministic_benchmark.dart \
	--dart-define=JPE_BODIES=10000 \
	--dart-define=JPE_STEPS=600 \
	--dart-define=JPE_SEED=1337 \
	--dart-define=JPE_ISOLATE_MIN_BODIES=256 \
	--dart-define=JPE_ISOLATE_DISPATCH_EVERY=1 \
	--dart-define=JPE_ADAPTIVE_ENABLED=true \
	--dart-define=JPE_ADAPTIVE_TARGET_MS=16.67 \
	--dart-define=JPE_ADAPTIVE_CHECK_EVERY=30 \
	--dart-define=JPE_ADAPTIVE_MARGIN=0.15 \
	--dart-define=JPE_ADAPTIVE_MIN_HOLD_STEPS=30
```

For mid-range Android target validation, run with `--dart-define=JPE_BODIES=2000`.

### Determinism and Parity Tests

```bash
flutter test test/deterministic_replay_test.dart
flutter test test/pure_vs_box2d_parity_test.dart
```

The parity suite auto-skips strict backend comparison when Box2D is not active.

## Project Docs

- Architecture: [ARCHITECTURE.md](ARCHITECTURE.md)
- API Reference: [API.md](API.md)
- Changelog: [CHANGELOG.md](CHANGELOG.md)
- Contributing: [CONTRIBUTING.md](CONTRIBUTING.md)
- Code of Conduct: [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)
