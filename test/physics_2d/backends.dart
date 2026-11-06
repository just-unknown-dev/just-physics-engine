// The two 2-D backends, for tests that hold both to the same behaviour.

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/just_physics_engine.dart';

/// Runs [tests] in a group per backend — the pure-Dart engine and Box2D —
/// handing them that backend's engine, made and initialised afresh for each
/// test. A test on Box2D where its library is not in the build should skip
/// itself: see [noBox2D].
void onBothBackends(void Function(Physics2DEngine Function() engine) tests) {
  for (final (name, make) in <(String, Physics2DEngine Function())>[
    ('pure Dart', Physics2DEngine.pureDart),
    ('Box2D', Box2DPhysicsEngine.new),
  ]) {
    group(name, () {
      late Physics2DEngine engine;
      setUp(() => engine = make()..initialize());
      tearDown(() => engine.dispose());
      tests(() => engine);
    });
  }
}

/// Whether [engine] is a Box2D engine running without its library (it then
/// runs pure Dart, and a Box2D test has nothing to test).
bool noBox2D(Physics2DEngine engine) =>
    engine is Box2DPhysicsEngine && !engine.isNative;

/// Steps [engine] [frames] frames of [dt].
void runFrames(Physics2DEngine engine, int frames, {double dt = 1 / 60}) {
  for (var i = 0; i < frames; i++) {
    engine.update(dt);
  }
}
