/// The native Box2D backend's queries: rays, boxes and the character
/// mover's two queries, on Box2D's broad-phase tree.
library;

import 'dart:ffi';
import 'dart:ui' show Offset, Rect;

import 'package:ffi/ffi.dart';
import 'package:just_box_2d/ffi.dart' as box2d;
import 'package:meta/meta.dart' show internal;

import '../physics_2d_engine.dart';
import '../ray_2d.dart';

/// The queries of one native world, with the native buffers they write
/// into: allocated once (a calloc/free pair on every ground probe is real
/// cost at 60 Hz) and freed by [dispose].
@internal
final class Box2DQueries {
  /// Queries answered in the world [world] gives, naming bodies through
  /// [bodyOf] (a handle's body, or null) and [handleOf] (a body's handle,
  /// or 0).
  Box2DQueries(this.world, this.bodyOf, this.handleOf);

  /// The native world's handle, read at each query.
  final int Function() world;

  /// The body a handle names, or null.
  final Physics2DBody? Function(int handle) bodyOf;

  /// The handle of a body, or 0.
  final int Function(Physics2DBody body) handleOf;

  /// Every collision category — the filter value meaning "hit anything".
  /// b2QueryFilter uses 64-bit category/mask bits.
  static const int _allCategories = -1; // all 64 bits set

  /// How many hits one native [raycastAll]/[queryAABB] call has room for.
  /// Box2D reports one hit per *shape*, so a compound body can take several;
  /// a call that fills the room is asked again with twice as much, up to
  /// [_maxQueryCapacity].
  int _queryCapacity = 32;
  static const int _maxQueryCapacity = 4096;

  Pointer<Int64>? _queryBodyBuffer; // _queryCapacity handles
  Pointer<Float>? _queryDataBuffer; // _queryCapacity * 5 floats
  Pointer<Int64>? _rayBodyOut; // single-hit out-params
  Pointer<Float>? _rayFloatOut; // [px, py, nx, ny, fraction]

  /// Most planes one [collideMover] call reports.
  static const int _maxMoverPlanes = 32;

  Pointer<Float>? _moverPoints; // up to 8 points, x and y
  Pointer<Float>? _moverPlanes; // _maxMoverPlanes * 5 floats
  Pointer<Int64>? _moverBodies; // _maxMoverPlanes handles

  void _ensureQueryBuffers() {
    _queryBodyBuffer ??= calloc<Int64>(_queryCapacity);
    _queryDataBuffer ??= calloc<Float>(_queryCapacity * 5);
    _rayBodyOut ??= calloc<Int64>();
    _rayFloatOut ??= calloc<Float>(5);
  }

  /// Whether a query that wrote [count] hits may have run out of room: if
  /// so the buffers grow, and the caller asks again.
  bool _queryFull(int count) {
    if (count < _queryCapacity || _queryCapacity >= _maxQueryCapacity) {
      return false;
    }
    final qb = _queryBodyBuffer;
    final qd = _queryDataBuffer;
    _queryBodyBuffer = null;
    _queryDataBuffer = null;
    if (qb != null) calloc.free(qb);
    if (qd != null) calloc.free(qd);
    _queryCapacity *= 2;
    _ensureQueryBuffers();
    return true;
  }

  /// The closest hit of [ray] on a shape it does not start inside, with
  /// every category: Box2D's closest-hit cast.
  Ray2DHit? raycast(Ray2D ray) {
    _ensureQueryBuffers();
    final origin = ray.origin;
    final translation = ray.direction * ray.maxDistance;
    final f = _rayFloatOut!;
    final hit = box2d.b2w_castRayClosest(
      world(),
      origin.dx,
      origin.dy,
      translation.dx,
      translation.dy,
      _allCategories,
      _allCategories,
      _rayBodyOut!,
      f,
      f + 1,
      f + 2,
      f + 3,
      f + 4,
    );
    if (hit == 0) return null;
    final body = bodyOf(_rayBodyOut!.value);
    if (body == null) return null;
    return Ray2DHit(
      body: body,
      point: Offset(f[0], f[1]),
      normal: Offset(f[2], f[3]),
      distance: f[4] * ray.maxDistance,
    );
  }

  /// Every body [ray] hits, nearest first, each once at its nearest hit,
  /// leaving out [exclude] and shapes the ray starts inside.
  List<Ray2DHit> raycastAll(Ray2D ray, Set<Physics2DBody>? exclude) {
    _ensureQueryBuffers();
    final origin = ray.origin;
    final translation = ray.direction * ray.maxDistance;

    int count;
    do {
      count = box2d.b2w_castRayAll(
        world(),
        origin.dx,
        origin.dy,
        translation.dx,
        translation.dy,
        _allCategories,
        _allCategories,
        _queryBodyBuffer!,
        _queryDataBuffer!,
        _queryCapacity,
      );
    } while (_queryFull(count));

    // One hit per shape, in the tree's order: nearest first, then only a
    // body's nearest hit. A shape the ray starts inside is passed over, as
    // the closest-hit cast passes it (Box2D reports it at fraction 0 with no
    // normal).
    final data = _queryDataBuffer!;
    final order = List<int>.generate(count, (i) => i)
      ..sort((i, j) => data[i * 5 + 4].compareTo(data[j * 5 + 4]));
    final hits = <Ray2DHit>[];
    final seen = <Physics2DBody>{};
    for (final i in order) {
      final base = i * 5;
      if (data[base + 4] == 0 && data[base + 2] == 0 && data[base + 3] == 0) {
        continue;
      }
      final body = bodyOf(_queryBodyBuffer![i]);
      if (body == null) continue;
      if (exclude != null && exclude.contains(body)) continue;
      if (!seen.add(body)) continue;
      hits.add(
        Ray2DHit(
          body: body,
          point: Offset(data[base], data[base + 1]),
          normal: Offset(data[base + 2], data[base + 3]),
          distance: data[base + 4] * ray.maxDistance,
        ),
      );
    }
    return hits;
  }

  /// The bodies, each once, with a shape whose box (Box2D's, a little
  /// larger than the shape) overlaps [rect].
  List<Physics2DBody> queryAABB(Rect rect) {
    _ensureQueryBuffers();
    int count;
    do {
      count = box2d.b2w_queryAABB(
        world(),
        rect.left,
        rect.top,
        rect.right,
        rect.bottom,
        _allCategories,
        _allCategories,
        _queryBodyBuffer!,
        _queryCapacity,
      );
    } while (_queryFull(count));

    final result = <Physics2DBody>[];
    for (var i = 0; i < count; i++) {
      final body = bodyOf(_queryBodyBuffer![i]);
      if (body != null) result.add(body);
    }
    return result;
  }

  // ── Character movers ──────────────────────────────────────────────────────
  //
  // Answered in C (b2w_moverCollide / b2w_moverCast) with the same rules as
  // the pure-Dart engine, on Box2D's own GJK and shape cast.

  /// Writes [shape]'s core points into the buffer; returns their count.
  int _moverShape(Mover2DShape shape) {
    _moverPoints ??= calloc<Float>(16);
    _moverPlanes ??= calloc<Float>(_maxMoverPlanes * 5);
    _moverBodies ??= calloc<Int64>(_maxMoverPlanes);
    final out = _moverPoints!;
    final points = shape.points;
    for (var i = 0; i < points.length; i++) {
      out[2 * i] = points[i].dx;
      out[2 * i + 1] = points[i].dy;
    }
    return points.length;
  }

  int _moverIgnore(Mover2DFilter filter) {
    final ignore = filter.ignore;
    return ignore == null ? 0 : handleOf(ignore);
  }

  /// See [Physics2DEngine.collideMover].
  List<Mover2DPlane> collideMover(
    Mover2DShape shape,
    Offset position,
    Mover2DFilter filter,
  ) {
    final count = _moverShape(shape);
    final n = box2d.b2w_moverCollide(
      world(),
      position.dx,
      position.dy,
      _moverPoints!,
      count,
      shape.radius,
      filter.categoryBits,
      filter.maskBits,
      _moverIgnore(filter),
      _moverPlanes!,
      _moverBodies!,
      _maxMoverPlanes,
    );
    final f = _moverPlanes!;
    final planes = <Mover2DPlane>[];
    for (var i = 0; i < n; i++) {
      final plane = Mover2DPlane(
        normal: Offset(f[5 * i], f[5 * i + 1]),
        offset: f[5 * i + 2],
        point: Offset(f[5 * i + 3] + position.dx, f[5 * i + 4] + position.dy),
        body: bodyOf(_moverBodies![i]),
      );
      // A chain with no thickness is in Box2D twice, once each way round
      // (see box2d_shapes.dart), so the same plane can come twice.
      final twice = planes.any(
        (p) =>
            p.body == plane.body &&
            (p.normal - plane.normal).distanceSquared < 1e-8 &&
            (p.offset - plane.offset).abs() < 1e-4 &&
            (p.point - plane.point).distanceSquared < 1e-6,
      );
      if (!twice) planes.add(plane);
    }
    return planes;
  }

  /// See [Physics2DEngine.castMover].
  double castMover(
    Mover2DShape shape,
    Offset position,
    Offset translation,
    Mover2DFilter filter,
  ) {
    final count = _moverShape(shape);
    return box2d.b2w_moverCast(
      world(),
      position.dx,
      position.dy,
      _moverPoints!,
      count,
      shape.radius,
      translation.dx,
      translation.dy,
      filter.categoryBits,
      filter.maskBits,
      _moverIgnore(filter),
    );
  }

  /// Frees every native buffer; the next query allocates them again.
  void dispose() {
    for (final p in <Pointer<NativeType>?>[
      _queryBodyBuffer,
      _queryDataBuffer,
      _rayBodyOut,
      _rayFloatOut,
      _moverPoints,
      _moverPlanes,
      _moverBodies,
    ]) {
      if (p != null) calloc.free(p);
    }
    _queryBodyBuffer = null;
    _queryDataBuffer = null;
    _rayBodyOut = null;
    _rayFloatOut = null;
    _moverPoints = null;
    _moverPlanes = null;
    _moverBodies = null;
    _queryCapacity = 32;
  }
}
