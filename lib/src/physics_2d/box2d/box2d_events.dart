/// The native Box2D backend's events.
library;

import 'dart:ffi';

import 'package:ffi/ffi.dart';
import 'package:just_box_2d/ffi.dart' as box2d;
import 'package:meta/meta.dart' show internal;

import '../physics_2d_engine.dart';

/// A frame's events from a native world.
///
/// Box2D keeps only the last step's events, and a frame runs zero to five
/// steps: each step's are read right after it ([gather]) and kept until
/// [clear], as the pure-Dart engine keeps its own — none lost when a frame
/// runs two steps, none repeated when it runs none.
@internal
final class Box2DEvents {
  /// Contacts that began: the two bodies, and the normal from the first to
  /// the second.
  final List<(Physics2DBody, Physics2DBody, double, double)> contactBegins = [];

  /// Contacts that ended.
  final List<(Physics2DBody, Physics2DBody)> contactEnds = [];

  /// Bodies that came into a sensor.
  final List<({Physics2DBody sensor, Physics2DBody visitor})> sensorBegins = [];

  /// Bodies that left a sensor.
  final List<({Physics2DBody sensor, Physics2DBody visitor})> sensorEnds = [];

  /// The bodies that moved, each with whether it fell asleep in the last
  /// step it moved in.
  final Map<Physics2DBody, bool> moves = {};

  Pointer<Int64>? _bodies; // two handles
  Pointer<Float>? _normal; // nx, ny
  Pointer<Int32>? _asleep;

  /// Forgets the last frame's events.
  void clear() {
    contactBegins.clear();
    contactEnds.clear();
    sensorBegins.clear();
    sensorEnds.clear();
    moves.clear();
  }

  /// Reads one native step's events of world [world], naming bodies
  /// through [bodyOf] (a handle it does not know is passed over).
  void gather(int world, Map<int, Physics2DBody> bodyOf) {
    final a = _bodies ??= calloc<Int64>(2);
    final b = a + 1;
    final normal = _normal ??= calloc<Float>(2);
    final asleep = _asleep ??= calloc<Int32>();

    for (var i = 0, n = box2d.b2w_getContactBeginCount(world); i < n; i++) {
      box2d.b2w_getContactBeginEvent(world, i, a, b, normal, normal + 1);
      final pa = bodyOf[a.value], pb = bodyOf[b.value];
      if (pa != null && pb != null) {
        contactBegins.add((pa, pb, normal[0], normal[1]));
      }
    }
    for (var i = 0, n = box2d.b2w_getContactEndCount(world); i < n; i++) {
      box2d.b2w_getContactEndEvent(world, i, a, b);
      final pa = bodyOf[a.value], pb = bodyOf[b.value];
      if (pa != null && pb != null) contactEnds.add((pa, pb));
    }
    for (var i = 0, n = box2d.b2w_getSensorBeginCount(world); i < n; i++) {
      box2d.b2w_getSensorBeginEvent(world, i, a, b);
      final sensor = bodyOf[a.value], visitor = bodyOf[b.value];
      if (sensor != null && visitor != null) {
        sensorBegins.add((sensor: sensor, visitor: visitor));
      }
    }
    for (var i = 0, n = box2d.b2w_getSensorEndCount(world); i < n; i++) {
      box2d.b2w_getSensorEndEvent(world, i, a, b);
      final sensor = bodyOf[a.value], visitor = bodyOf[b.value];
      if (sensor != null && visitor != null) {
        sensorEnds.add((sensor: sensor, visitor: visitor));
      }
    }
    for (var i = 0, n = box2d.b2w_getBodyMoveEventCount(world); i < n; i++) {
      box2d.b2w_getBodyMoveEvent(world, i, a, asleep);
      final body = bodyOf[a.value];
      if (body == null) continue;
      moves
        ..remove(body) // reported once, in the order it last moved
        ..[body] = asleep.value != 0;
    }
  }

  /// Forgets the events and frees the native out-parameters; the next
  /// [gather] allocates them again.
  void dispose() {
    clear();
    for (final p in <Pointer<NativeType>?>[_bodies, _normal, _asleep]) {
      if (p != null) calloc.free(p);
    }
    _bodies = null;
    _normal = null;
    _asleep = null;
  }
}
