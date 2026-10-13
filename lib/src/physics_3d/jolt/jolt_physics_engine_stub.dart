import '../physics_engine_3d.dart';
import '../settings_3d.dart';

/// Jolt Physics as a [PhysicsEngine3D] — on the web, where it cannot run:
/// [unavailableReason] says so, and `PhysicsEngineFactory.create3D()`
/// throws.
final class JoltPhysicsEngine implements PhysicsEngine3D {
  /// An engine; [workerThreads] means nothing here.
  JoltPhysicsEngine({this.workerThreads = -1});

  /// Unused on the web.
  final int workerThreads;

  @override
  String get name => 'jolt';

  @override
  String? get unavailableReason => 'Jolt Physics needs dart:ffi, which the web does not have.';

  /// Always null here.
  String? get joltVersion => null;

  @override
  PhysicsWorld3D createWorld(PhysicsWorldSettings3D settings) =>
      throw Physics3DUnsupportedError(unavailableReason!);

  @override
  void destroyAll() {}

  /// Always 0 here.
  int get liveWorlds => 0;
}
