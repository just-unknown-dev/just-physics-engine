import 'box2d_world.dart';

/// Thin coordinator that drives [Box2DWorld.step] and exposes the render
/// interpolation alpha for the current frame.
///
/// The fixed timestep (1/60 s) lives inside [Box2DWorld]. This class is a
/// façade that makes the alpha available at the call site without exposing
/// [Box2DWorld] internals to higher layers.
///
/// Usage (inside Box2DPhysicsEngine.update):
/// ```dart
/// _loop.advance(deltaTime);
/// final alpha = _loop.alpha;  // pass to TransformInterpolator
/// ```
class PhysicsGameLoop {
  final Box2DWorld world;

  PhysicsGameLoop(this.world);

  /// Render interpolation factor α ∈ [0, 1).
  ///
  /// Interpretation:
  ///   - 0.0 → render state matches the previous physics tick exactly
  ///   - 1.0 → render state matches the current physics tick exactly
  ///
  /// Apply as:
  ///   renderPos = current * alpha + previous * (1.0 - alpha)
  double get alpha => world.alpha;

  /// Advance physics by [dt] seconds.
  ///
  /// Delegates to [Box2DWorld.step] which accumulates time internally and
  /// fires as many 1/60 s fixed steps as needed.
  int advance(double dt) => world.step(dt);
}
