import 'package:just_jolt/ffi.dart';

import '../physics_engine_3d.dart';
import '../settings_3d.dart';
import 'jolt_world.dart';

/// Jolt Physics (through just_jolt) as a [PhysicsEngine3D].
///
/// `PhysicsEngineFactory.create3D()` makes one and throws when it cannot
/// run. Where it cannot — the web, a 32-bit Android ABI, an app whose
/// pubspec did not ask for Jolt to be built — [unavailableReason] says why.
///
/// All worlds share one job system for the whole process, with
/// [workerThreads] threads (cores minus one by default; the first engine
/// started decides). Contacts bounce as much as the less bouncy of their
/// two bodies, as in the 2-D engines.
final class JoltPhysicsEngine implements PhysicsEngine3D {
  /// An engine whose job system has [workerThreads] threads; below 0 is
  /// "cores minus one".
  JoltPhysicsEngine({this.workerThreads = -1});

  /// Threads in the process-wide job system, if this engine starts it.
  final int workerThreads;

  @override
  String get name => 'jolt';

  @override
  String? get unavailableReason => Jolt.start(workerThreads: workerThreads);

  /// Jolt's version, `major.minor.patch`; null when it is not available.
  String? get joltVersion => Jolt.version;

  @override
  PhysicsWorld3D createWorld(PhysicsWorldSettings3D settings) {
    final reason = unavailableReason;
    if (reason != null) throw Physics3DUnsupportedError(reason);
    return JoltWorld(settings);
  }

  @override
  void destroyAll() => Jolt.destroyAll();

  /// How many native worlds are alive in this process (for leak checks).
  int get liveWorlds => Jolt.liveWorlds;
}
