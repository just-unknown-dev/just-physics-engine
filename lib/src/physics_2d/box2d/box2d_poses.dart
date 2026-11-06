/// The native Box2D backend's pose read-back.
library;

import 'dart:ffi';
import 'dart:typed_data';

import 'package:ffi/ffi.dart';
import 'package:just_box_2d/ffi.dart' as box2d;
import 'package:meta/meta.dart' show internal;

import '../physics_2d_engine.dart';
import 'box2d_body.dart';

/// Reads native bodies' poses back into their [Physics2DBody]s, in one FFI
/// call for all of them, through native buffers allocated once and grown as
/// the world does.
@internal
final class Box2DPoses {
  Pointer<Int64>? _handles; // one int64 per tracked body
  /// 7 floats per body: x, y, angle, vx, vy, awake, angular velocity.
  Pointer<Float>? _poses;
  int _capacity = 0;

  Pointer<Int64>? _oneHandle;
  Pointer<Float>? _onePose; // 7 floats

  /// Room for [needed] bodies.
  void ensureCapacity(int needed) {
    if (needed <= _capacity) return;
    // Grow by 1.5× to amortise reallocations.
    final cap = (needed * 1.5).ceil().clamp(8, 1 << 20);
    final oldH = _handles;
    final oldP = _poses;
    if (oldH != null) calloc.free(oldH);
    if (oldP != null) calloc.free(oldP);
    _handles = calloc<Int64>(cap);
    _poses = calloc<Float>(cap * 7);
    _capacity = cap;
  }

  /// Reads every body of [bodies] back: zero-copy (`asTypedList` views of
  /// the native buffers). A body out of the simulation is not written: it
  /// keeps what it had, as on the pure-Dart engine.
  void readAll(Map<Physics2DBody, Box2DBody> bodies) {
    final n = bodies.length;
    if (n == 0) return;
    ensureCapacity(n);

    final Int64List handles = _handles!.asTypedList(n);
    int i = 0;
    for (final b2Body in bodies.values) {
      handles[i++] = b2Body.handle;
    }

    // One FFI call writes x, y, angle, vx, vy, awake, spin for every body.
    box2d.b2w_bulkExtractTransforms(_handles!, _poses!, n);
    final Float32List trs = _poses!.asTypedList(n * 7);

    i = 0;
    for (final entry in bodies.entries) {
      final dartBody = entry.key;
      final b2Body = entry.value;
      final base = i++ * 7;
      if (!b2Body.enabled) continue;

      b2Body.currentX = trs[base];
      b2Body.currentY = trs[base + 1];
      b2Body.currentAngle = trs[base + 2];
      b2Body.velocityX = trs[base + 3];
      b2Body.velocityY = trs[base + 4];
      b2Body.angularVelocity = trs[base + 6];
      b2Body.awake = trs[base + 5] != 0.0;

      dartBody.position.x = b2Body.currentX;
      dartBody.position.y = b2Body.currentY;
      dartBody.angle = b2Body.currentAngle;
      dartBody.velocity.x = b2Body.velocityX;
      dartBody.velocity.y = b2Body.velocityY;
      dartBody.angularVelocity = b2Body.angularVelocity;
      // Sleep is a dynamic-body concept — static bodies always report
      // "not awake" natively and would otherwise flip Physics2DBody.isAwake
      // to false even though the user never asked for that.
      if (dartBody.mass > 0) {
        dartBody.isAwake = b2Body.awake;
      }
    }
  }

  /// [body]'s pose and velocity from [b2Body], between steps.
  void readOne(Physics2DBody body, Box2DBody b2Body) {
    final handle = _oneHandle ??= calloc<Int64>();
    final pose = _onePose ??= calloc<Float>(7);
    handle.value = b2Body.handle;
    box2d.b2w_bulkExtractTransforms(handle, pose, 1);
    body.position.setValues(pose[0], pose[1]);
    body.angle = pose[2];
    body.velocity.setValues(pose[3], pose[4]);
    b2Body
      ..velocityX = pose[3]
      ..velocityY = pose[4];
  }

  /// Frees the native buffers (calloc's: a Pointer has no free of its own);
  /// the next read allocates them again.
  void dispose() {
    for (final p in <Pointer<NativeType>?>[
      _handles,
      _poses,
      _oneHandle,
      _onePose,
    ]) {
      if (p != null) calloc.free(p);
    }
    _handles = null;
    _poses = null;
    _oneHandle = null;
    _onePose = null;
    _capacity = 0;
  }
}
