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
/// Note: This class lives in just_physics_engine because it only depends on
/// [PhysicsBody] and [Box2DBody]. The ECS write-back (to TransformComponent)
/// is handled in just_game_engine's PhysicsBridgeSystem, which receives the
/// interpolated values via the [write] callback.
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
