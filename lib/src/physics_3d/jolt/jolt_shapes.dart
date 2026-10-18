import 'dart:collection';
import 'dart:ffi';
import 'dart:math' as math;

import 'package:ffi/ffi.dart';
import 'package:just_jolt/ffi.dart';

import '../shapes_3d.dart';

/// Jolt's default convex radius: corners are rounded by this much.
const double _convexRadius = 0.05;

/// One native shape and how many bodies hold it.
final class _ShapeRef implements Finalizable {
  _ShapeRef(this.pointer) {
    _finalizer.attach(this, pointer.cast(), detach: this);
  }

  static final NativeFinalizer _finalizer = NativeFinalizer(
    addresses.jj_shape_release.cast(),
  );

  final Pointer<JPH_Shape> pointer;
  int bodies = 0;

  void release() {
    _finalizer.detach(this);
    jj_shape_release(pointer.cast());
  }
}

/// The native shapes of one world, shared between every body and query
/// that asks for an equal [Shape3D] and reference-counted: a shape goes
/// when its last body does. A few shapes no body holds — a camera's sweep
/// sphere — are kept for the next query.
///
/// Each native shape also carries a [NativeFinalizer], so a world dropped
/// without `dispose()` still lets its shapes go.
final class JoltShapes {
  /// Shapes for one world.
  JoltShapes();

  /// How many unheld shapes are kept for queries.
  static const int idleLimit = 32;

  final Map<Shape3D, _ShapeRef> _refs = {};
  final LinkedHashSet<Shape3D> _idle = LinkedHashSet();

  /// How many native shapes there are.
  int get length => _refs.length;

  /// [shape] for a new body: made, or shared. Null when Jolt cannot make it.
  Pointer<JPH_Shape>? acquire(Shape3D shape) {
    final ref = _ref(shape);
    if (ref == null) return null;
    ref.bodies++;
    _idle.remove(shape);
    return ref.pointer;
  }

  /// A body made of [shape] has gone.
  void release(Shape3D shape) {
    final ref = _refs[shape];
    if (ref == null) return;
    if (--ref.bodies > 0) return;
    _refs.remove(shape);
    ref.release();
  }

  /// [shape] for a query, kept while no body holds it among the last
  /// [idleLimit] used that way.
  Pointer<JPH_Shape>? borrow(Shape3D shape) {
    final ref = _ref(shape);
    if (ref == null) return null;
    if (ref.bodies == 0) {
      _idle
        ..remove(shape)
        ..add(shape);
      while (_idle.length > idleLimit) {
        final oldest = _idle.first;
        _idle.remove(oldest);
        _refs.remove(oldest)?.release();
      }
    }
    return ref.pointer;
  }

  /// Lets every shape go.
  void clear() {
    for (final ref in _refs.values) {
      ref.release();
    }
    _refs.clear();
    _idle.clear();
  }

  _ShapeRef? _ref(Shape3D shape) {
    final known = _refs[shape];
    if (known != null) return known;
    final pointer = make(shape);
    if (pointer == null) return null;
    return _refs[shape] = _ShapeRef(pointer);
  }

  /// A new native shape for [shape] that the caller owns (one reference);
  /// null when it is not [Shape3D.isValid] or Jolt refuses it.
  static Pointer<JPH_Shape>? make(Shape3D shape) =>
      shape.isValid ? _make(shape) : null;

  static Pointer<JPH_Shape>? _make(Shape3D shape) => using((arena) {
    final made = switch (shape) {
      BoxShape3D(:final hx, :final hy, :final hz) => _box(arena, hx, hy, hz),
      SphereShape3D(:final radius) => _settled(
        JPH_SphereShapeSettings_Create(radius),
        (s) => JPH_SphereShapeSettings_CreateShape(s).cast(),
      ),
      CapsuleShape3D(:final halfHeight, :final radius) => _settled(
        JPH_CapsuleShapeSettings_Create(halfHeight, radius),
        (s) => JPH_CapsuleShapeSettings_CreateShape(s).cast(),
      ),
      CylinderShape3D(:final halfHeight, :final radius) => _settled(
        JPH_CylinderShapeSettings_Create(
          halfHeight,
          radius,
          math.min(_convexRadius, math.min(halfHeight, radius) * 0.9),
        ),
        (s) => JPH_CylinderShapeSettings_CreateShape(s).cast(),
      ),
      ConvexHullShape3D(:final points) => _hull(arena, points),
      MeshShape3D(:final positions, :final indices) => _mesh(
        arena,
        positions,
        indices,
      ),
      HeightFieldShape3D() => _heightField(arena, shape),
      TransformedShape3D() => _transformed(arena, shape),
      CompoundShape3D(:final parts) => _compound(arena, parts),
    };
    return made == null || made == nullptr ? null : made;
  });

  static Pointer<JPH_Shape>? _settled<S extends NativeType>(
    Pointer<S> settings,
    Pointer<JPH_Shape> Function(Pointer<S>) create,
  ) {
    if (settings == nullptr) return null;
    final shape = create(settings);
    JPH_ShapeSettings_Destroy(settings.cast());
    return shape;
  }

  static Pointer<JPH_Shape>? _box(
    Arena arena,
    double hx,
    double hy,
    double hz,
  ) {
    final half = arena<JPH_Vec3>();
    half.ref
      ..x = hx
      ..y = hy
      ..z = hz;
    // The rounding must fit inside the box.
    final rounding = math.min(
      _convexRadius,
      math.min(hx, math.min(hy, hz)) * 0.9,
    );
    return _settled(
      JPH_BoxShapeSettings_Create(half, rounding),
      (s) => JPH_BoxShapeSettings_CreateShape(s).cast(),
    );
  }

  static Pointer<JPH_Shape>? _hull(Arena arena, List<double> points) {
    final n = points.length ~/ 3;
    if (n < 4) return null;
    final native = arena<JPH_Vec3>(n);
    for (var i = 0; i < n; i++) {
      native[i]
        ..x = points[i * 3]
        ..y = points[i * 3 + 1]
        ..z = points[i * 3 + 2];
    }
    return _settled(
      JPH_ConvexHullShapeSettings_Create(native, n, _convexRadius),
      (s) => JPH_ConvexHullShapeSettings_CreateShape(s).cast(),
    );
  }

  static Pointer<JPH_Shape>? _mesh(
    Arena arena,
    List<double> positions,
    List<int> indices,
  ) {
    final vertices = positions.length ~/ 3;
    final triangles = indices.length ~/ 3;
    if (vertices < 3 || triangles < 1) return null;
    final v = arena<JPH_Vec3>(vertices);
    for (var i = 0; i < vertices; i++) {
      v[i]
        ..x = positions[i * 3]
        ..y = positions[i * 3 + 1]
        ..z = positions[i * 3 + 2];
    }
    final t = arena<JPH_IndexedTriangle>(triangles);
    for (var i = 0; i < triangles; i++) {
      t[i]
        ..i1 = indices[i * 3]
        ..i2 = indices[i * 3 + 1]
        ..i3 = indices[i * 3 + 2]
        ..materialIndex = 0
        ..userData = 0;
    }
    return _settled(
      JPH_MeshShapeSettings_Create2(v, vertices, t, triangles),
      (s) => JPH_MeshShapeSettings_CreateShape(s).cast(),
    );
  }

  /// Jolt's no-collision height: the largest float.
  static const double _noCollision = 3.4028234663852886e38;

  /// Jolt's height fields rise along +Y, which is down in the engine. The
  /// field is built with its rows reversed and turned half a turn about X,
  /// so its heights rise towards −Y and row 0 stays at z = 0. A hole goes
  /// in as Jolt's no-collision value.
  static Pointer<JPH_Shape>? _heightField(
    Arena arena,
    HeightFieldShape3D shape,
  ) {
    final n = shape.sampleCount;
    if (n < 4 || shape.heights.length < n * n) return null;
    final samples = arena<Float>(n * n);
    for (var row = 0; row < n; row++) {
      final from = (n - 1 - row) * n;
      for (var column = 0; column < n; column++) {
        final h = shape.heights[from + column];
        samples[row * n + column] = h >= HeightFieldShape3D.hole ? _noCollision : h;
      }
    }
    final offset = arena<JPH_Vec3>();
    final scale = arena<JPH_Vec3>();
    scale.ref
      ..x = shape.cellSize
      ..y = 1
      ..z = shape.cellSize;
    final field = _settled(
      JPH_HeightFieldShapeSettings_Create(samples, offset, scale, n, nullptr),
      (s) => JPH_HeightFieldShapeSettings_CreateShape(s).cast(),
    );
    if (field == null || field == nullptr) return null;
    final position = arena<JPH_Vec3>();
    position.ref.z = (n - 1) * shape.cellSize;
    final rotation = arena<JPH_Quat>();
    rotation.ref
      ..x =
          1 // Half a turn about X.
      ..w = 0;
    final placed = JPH_RotatedTranslatedShape_Create(
      position,
      rotation,
      field,
    ).cast<JPH_Shape>();
    JPH_Shape_Destroy(field);
    return placed;
  }

  /// A static compound of [parts], each placed by its own transform (a
  /// [TransformedShape3D]) or at the origin. Static means its parts are
  /// fixed once made, not that its body must be.
  static Pointer<JPH_Shape>? _compound(Arena arena, List<Shape3D> parts) {
    if (parts.isEmpty) return null;
    final made = <Pointer<JPH_Shape>>[];
    void releaseMade() {
      for (final part in made) {
        JPH_Shape_Destroy(part);
      }
    }

    final settings = JPH_StaticCompoundShapeSettings_Create();
    if (settings == nullptr) return null;
    final position = arena<JPH_Vec3>();
    final rotation = arena<JPH_Quat>();
    for (final part in parts) {
      final placed = part is TransformedShape3D
          ? part
          : TransformedShape3D(part);
      final inner = make(placed.shape);
      if (inner == null) {
        JPH_ShapeSettings_Destroy(settings.cast());
        releaseMade();
        return null;
      }
      made.add(inner);
      position.ref
        ..x = placed.x
        ..y = placed.y
        ..z = placed.z;
      _unit(rotation, placed);
      JPH_CompoundShapeSettings_AddShape2(
        settings.cast(),
        position,
        rotation,
        inner,
        0,
      );
    }
    final compound = JPH_StaticCompoundShape_Create(settings).cast<JPH_Shape>();
    // The compound holds its parts now; our references go.
    JPH_ShapeSettings_Destroy(settings.cast());
    releaseMade();
    return compound == nullptr ? null : compound;
  }

  static Pointer<JPH_Shape>? _transformed(
    Arena arena,
    TransformedShape3D shape,
  ) {
    final inner = make(shape.shape);
    if (inner == null) return null;
    if (shape.isIdentity) return inner;
    final position = arena<JPH_Vec3>();
    position.ref
      ..x = shape.x
      ..y = shape.y
      ..z = shape.z;
    final rotation = arena<JPH_Quat>();
    _unit(rotation, shape);
    final placed = JPH_RotatedTranslatedShape_Create(
      position,
      rotation,
      inner,
    ).cast<JPH_Shape>();
    JPH_Shape_Destroy(inner);
    return placed;
  }

  /// [shape]'s rotation, made unit length, into [out]: Jolt takes it as it
  /// is, and a compound keeps only its first three parts.
  static void _unit(Pointer<JPH_Quat> out, TransformedShape3D shape) {
    final length = math.sqrt(
      shape.qx * shape.qx +
          shape.qy * shape.qy +
          shape.qz * shape.qz +
          shape.qw * shape.qw,
    );
    out.ref
      ..x = shape.qx / length
      ..y = shape.qy / length
      ..z = shape.qz / length
      ..w = shape.qw / length;
  }
}
