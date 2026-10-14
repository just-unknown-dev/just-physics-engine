// PhysicsBody is a part of physics_engine.dart — import the library, not the part.
import '../physics_2d/physics_engine.dart';
import 'box2d_body.dart';

/// Sub-frame render interpolation utility.
///
/// Eliminates visual jitter caused by the mismatch between the fixed physics
/// timestep (1/60 s) and the variable Flutter frame rate.
///
/// Formula:
///   renderPos = current × α  +  previous × (1 − α)
///
/// Where α = accumulator / fixedDt (exposed via Box2DWorld.alpha / PhysicsGameLoop.alpha).
///
/// Call [interpolate] once per render pass (before the RenderSystem draws),
/// passing the current alpha from [PhysicsGameLoop.alpha].
///
/// It is for [Box2DBody] objects of your own, on a [Box2DWorld] you step
/// yourself: set each body's `prev*` before a step and `current*` after it.
/// [Box2DPhysicsEngine] keeps its own bodies' poses; with it, use its
/// [PhysicsEngine.alpha] between the [PhysicsBody] positions instead.
abstract final class TransformInterpolator {
  /// Interpolate all bodies in [bodyMap] and write results via [write].
  ///
  /// [alpha] comes from [PhysicsGameLoop.alpha] — a value in [0, 1).
  /// [write] receives the [PhysicsBody] key and the interpolated x, y, angle.
  ///
  /// For large angular velocities (fast spinning objects), replace the angle
  /// lerp with slerp on the (cos, sin) rotation pair.
  static void interpolate({
    required Map<PhysicsBody, Box2DBody> bodyMap,
    required double alpha,
    required void Function(PhysicsBody body, double x, double y, double angle)
    write,
  }) {
    final oneMinusAlpha = 1.0 - alpha;
    for (final entry in bodyMap.entries) {
      final b = entry.value;
      write(
        entry.key,
        b.currentX * alpha + b.prevX * oneMinusAlpha,
        b.currentY * alpha + b.prevY * oneMinusAlpha,
        b.currentAngle * alpha + b.prevAngle * oneMinusAlpha,
      );
    }
  }
}
