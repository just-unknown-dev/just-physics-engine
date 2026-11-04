import '../physics_3d_engine.dart';
import '../physics_3d_settings.dart';

/// Jolt Physics as a [Physics3DEngine] — on the web, where it cannot run:
/// [unavailableReason] says so, and `PhysicsEngineFactory.create3D()`
/// throws.
final class JoltPhysicsEngine implements Physics3DEngine {
  /// An engine; [workerThreads] means nothing here.
  JoltPhysicsEngine({this.workerThreads = -1});

  /// Unused on the web.
  final int workerThreads;

  @override
  String get name => 'jolt';

  @override
  String? get unavailableReason =>
      'Jolt Physics needs dart:ffi, which the web does not have.';

  /// Always null here.
  String? get joltVersion => null;

  @override
  Physics3DWorld createWorld(Physics3DWorldSettings settings) =>
      throw Physics3DUnsupportedError(unavailableReason!);

  @override
  void destroyAll() {}

  /// Always 0 here.
  int get liveWorlds => 0;
}
