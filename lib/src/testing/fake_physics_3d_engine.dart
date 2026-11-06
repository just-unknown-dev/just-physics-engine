/// A [Physics3DEngine] for tests, in plain Dart: the engine, its world
/// (`fake_physics_3d_world.dart`), what the world holds
/// (`fake_body_3d.dart`) and the box geometry it all stands on
/// (`fake_3d_geometry.dart`).
library;

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:just_dart/just_dart.dart';

import '../physics_3d/physics_3d_engine.dart';
import '../physics_3d/physics_3d_results.dart';
import '../physics_3d/physics_3d_settings.dart';
import '../physics_3d/shape_3d.dart';

part 'fake_3d_geometry.dart';
part 'fake_body_3d.dart';
part 'fake_physics_3d_world.dart';

/// A [Physics3DEngine] for tests, in plain Dart.
///
/// Every shape is its axis-aligned bounding box (a turned shape, the box
/// round it) and the simulation turns nothing: dynamic bodies fall, land on
/// and slide off other bodies, sensors report what enters them, rays and
/// sweeps hit boxes, and a character walks on boxes, climbs steps, is
/// stopped by walls and ceilings, and rides what it stands on. Joints are
/// recorded, not simulated. It keeps Jolt's rules: which pairs touch (one
/// body dynamic, or a kinematic body and a sensor), which shapes a body may
/// have, what dead handles do. Deterministic: the same calls on a new
/// engine give the same results, handles included (each world numbers its
/// own, tagged with its place among the engine's worlds, so one world's
/// handle never names another's body).
///
/// Pass `unavailableReason` to stand in for an engine that cannot run.
final class FakePhysics3DEngine implements Physics3DEngine {
  /// A fake engine; unavailable, giving [unavailableReason], when that is
  /// not null.
  FakePhysics3DEngine({this.unavailableReason, this.collisions = true});

  @override
  String get name => 'fake';

  @override
  final String? unavailableReason;

  /// Whether worlds it makes resolve collisions (off: bodies only move by
  /// velocity and gravity, for timing the ECS side).
  final bool collisions;

  /// The worlds it made that are not disposed.
  final List<FakePhysics3DWorld> worlds = [];

  /// How often [destroyAll] was called.
  int destroyAllCalls = 0;

  /// How many worlds it made: the next one's tag.
  int _worldsMade = 0;

  @override
  Physics3DWorld createWorld(Physics3DWorldSettings settings) {
    final reason = unavailableReason;
    if (reason != null) throw Physics3DUnsupportedError(reason);
    final world = FakePhysics3DWorld(
      settings,
      collisions: collisions,
      owner: this,
      tag: ++_worldsMade,
    );
    worlds.add(world);
    return world;
  }

  @override
  void destroyAll() {
    destroyAllCalls++;
    for (final world in List.of(worlds)) {
      world.dispose();
    }
  }
}
