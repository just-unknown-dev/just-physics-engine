// Tearing a native world down in any order.
//
// A hot restart shuts the old isolate down without anyone calling dispose():
// every world and body still attached to a NativeFinalizer is finalized at
// once, in whatever order the GC finds them — so a world can go before its own
// bodies, joints and chains. Before the native side checked its handles, the
// bodies' finalizers then read a wiped world and took the process down. If one
// of these regresses, the test runner crashes rather than failing.

import 'dart:ffi';

import 'package:ffi/ffi.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/src/box2d/ffi/box2d_bindings.dart' as box2d;
import 'package:just_physics_engine/src/box2d/ffi/box2d_library.dart';

void main() {
  var nativeLoaded = false;
  try {
    loadBox2DLibrary();
    nativeLoaded = true;
  } on Object {
    // No native build here (the Box2D submodule missing): nothing to test.
  }

  // The finalizers exactly as the GC calls them: a token that is the handle.
  void finalizeWorld(int handle) => box2dFinalizerWorld
      .asFunction<void Function(Pointer<Void>)>()(
        Pointer<Void>.fromAddress(handle),
      );
  void finalizeBody(int handle) => box2dFinalizerBody
      .asFunction<void Function(Pointer<Void>)>()(
        Pointer<Void>.fromAddress(handle),
      );

  int world() => box2d.b2w_createWorld(0, 981, 1);
  int body(int world, double x) {
    final b = box2d.b2w_createDynamicBody(world, x, 0, 0);
    box2d.b2w_addBoxShape(b, 8, 8, 1, 0.5, 0);
    return b;
  }

  group('native teardown in any order', () {
    test('a world finalized before its bodies', () {
      if (!nativeLoaded) return markTestSkipped('native Box2D unavailable');
      final w = world();
      final bodies = [for (var i = 0; i < 8; i++) body(w, i * 20.0)];
      box2d.b2w_step(w, 1 / 60, 4);
      finalizeWorld(w);
      for (final b in bodies) {
        finalizeBody(b);
      }
    });

    test('bodies finalized before their world', () {
      if (!nativeLoaded) return markTestSkipped('native Box2D unavailable');
      final w = world();
      final bodies = [for (var i = 0; i < 8; i++) body(w, i * 20.0)];
      for (final b in bodies) {
        finalizeBody(b);
      }
      finalizeWorld(w);
    });

    test('a body destroyed twice, and a world destroyed twice', () {
      if (!nativeLoaded) return markTestSkipped('native Box2D unavailable');
      final w = world();
      final b = body(w, 0);
      box2d.b2w_destroyBody(b);
      box2d.b2w_destroyBody(b);
      box2d.b2w_destroyWorld(w);
      box2d.b2w_destroyWorld(w);
    });

    test('a joint and a chain destroyed after their world', () {
      if (!nativeLoaded) return markTestSkipped('native Box2D unavailable');
      final w = world();
      final a = body(w, 0);
      final b = body(w, 40);
      final joint = box2d.b2w_createRevoluteJoint(w, a, b, 20, 0);
      final ground = box2d.b2w_createStaticBody(w, 0, 100, 0);
      final points = calloc<Float>(6);
      try {
        for (var i = 0; i < 3; i++) {
          points[i * 2] = i * 50.0;
          points[i * 2 + 1] = 0;
        }
        final chain = box2d.b2w_addChainShape(ground, points, 3, 0, 0.5, 0);
        box2d.b2w_destroyWorld(w);
        box2d.b2w_destroyJoint(joint);
        box2d.b2w_destroyChain(chain);
        box2d.b2w_destroyBody(a);
      } finally {
        calloc.free(points);
      }
    });

    test('a joint destroyed after one of its bodies', () {
      if (!nativeLoaded) return markTestSkipped('native Box2D unavailable');
      final w = world();
      final a = body(w, 0);
      final b = body(w, 40);
      final joint = box2d.b2w_createRevoluteJoint(w, a, b, 20, 0);
      box2d.b2w_destroyBody(a);
      box2d.b2w_destroyJoint(joint);
      box2d.b2w_destroyWorld(w);
    });

    test("a gone world's bodies leave another world alone", () {
      if (!nativeLoaded) return markTestSkipped('native Box2D unavailable');
      final first = world();
      final second = world();
      final stale = body(first, 0);
      final live = body(second, 0);
      finalizeWorld(first);
      finalizeBody(stale);
      // The second world still steps, and its body still moves under gravity.
      for (var i = 0; i < 10; i++) {
        box2d.b2w_step(second, 1 / 60, 4);
      }
      final handles = calloc<Int64>()..[0] = live;
      final out = calloc<Float>(7); // x, y, angle, vx, vy, awake, spin
      try {
        box2d.b2w_bulkExtractTransforms(handles, out, 1);
        expect(out[1], greaterThan(0), reason: 'fell under gravity');
      } finally {
        calloc.free(handles);
        calloc.free(out);
      }
      box2d.b2w_destroyWorld(second);
    });

    test('a step on a world that has gone does nothing', () {
      if (!nativeLoaded) return markTestSkipped('native Box2D unavailable');
      final w = world();
      body(w, 0);
      box2d.b2w_destroyWorld(w);
      box2d.b2w_step(w, 1 / 60, 4);
    });

    /// Whether [body] is a live native body: a live dynamic body reads back
    /// awake after it is made, a dead one as zeros.
    bool alive(int body) {
      final handles = calloc<Int64>()..[0] = body;
      final out = calloc<Float>(7); // x, y, angle, vx, vy, awake, spin
      try {
        box2d.b2w_bulkExtractTransforms(handles, out, 1);
        return out[5] == 1;
      } finally {
        calloc.free(handles);
        calloc.free(out);
      }
    }

    test("a gone world's body never names a newer world's", () {
      if (!nativeLoaded) return markTestSkipped('native Box2D unavailable');
      final first = world();
      final stale = body(first, 0);
      box2d.b2w_destroyWorld(first);
      // Most likely the same slot, its bodies numbered from the start again.
      final second = world();
      final live = body(second, 0);
      expect(live, isNot(stale));
      box2d.b2w_destroyBody(stale);
      finalizeBody(stale);
      box2d.b2w_setLinearVelocity(stale, 0, -500);
      box2d.b2w_step(second, 1 / 60, 4); // Runs the queued finalizer.
      expect(alive(live), isTrue);
      expect(alive(stale), isFalse);
      box2d.b2w_destroyWorld(second);
    });

    test('a finalized body goes on the next call from Dart', () {
      if (!nativeLoaded) return markTestSkipped('native Box2D unavailable');
      final w = world();
      final b = body(w, 0);
      expect(alive(b), isTrue);
      finalizeBody(b);
      box2d.b2w_step(w, 1 / 60, 4);
      expect(alive(b), isFalse);
      box2d.b2w_destroyWorld(w);
    });

    test('a world past the last free slot is 0, and nothing breaks', () {
      if (!nativeLoaded) return markTestSkipped('native Box2D unavailable');
      final worlds = <int>[];
      try {
        while (worlds.length < 200) {
          final w = box2d.b2w_createWorld(0, 981, 1);
          if (w == 0) break;
          worlds.add(w);
        }
        expect(worlds.length, lessThan(200));
        expect(box2d.b2w_createDynamicBody(0, 0, 0, 0), 0);
        box2d.b2w_step(0, 1 / 60, 4);
      } finally {
        worlds.forEach(box2d.b2w_destroyWorld);
      }
    });

    test('200 worlds made and torn down, in both orders', () {
      if (!nativeLoaded) return markTestSkipped('native Box2D unavailable');
      for (var round = 0; round < 200; round++) {
        final w = world();
        final bodies = [for (var i = 0; i < 4; i++) body(w, i * 20.0)];
        box2d.b2w_step(w, 1 / 60, 4);
        if (round.isEven) {
          finalizeWorld(w);
          bodies.forEach(finalizeBody);
        } else {
          bodies.forEach(finalizeBody);
          finalizeWorld(w);
        }
      }
    });
  });
}
