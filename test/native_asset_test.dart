// The native library is found through its code asset id, never by a file
// name. DynamicLibrary.open('box2d_flutter') worked on Windows by luck (the
// loader adds .dll) and never on Android, whose file is libbox2d_flutter.so:
// Android games ran the pure-Dart fallback without anyone noticing.

import 'dart:io';

import 'package:flutter_test/flutter_test.dart';
import 'package:just_physics_engine/just_physics_engine.dart';
import 'package:just_physics_engine/src/box2d/ffi/box2d_library.dart';

void main() {
  // The hook builds nothing when the Box2D submodule is missing.
  final hasSources = Directory(
    'src/native/third_party/just_box_2d/src',
  ).existsSync();

  test('the bindings resolve through the asset, not a file name', () {
    for (final file in Directory('lib').listSync(recursive: true)) {
      if (file is! File || !file.path.endsWith('.dart')) continue;
      final text = file.readAsStringSync();
      expect(
        text.contains('DynamicLibrary.open'),
        isFalse,
        reason: '${file.path} opens a library by name',
      );
    }
    final bindings = File(
      'lib/src/box2d/ffi/box2d_bindings.dart',
    ).readAsStringSync();
    expect(
      bindings,
      contains(
        "@ffi.DefaultAsset('package:just_physics_engine/box2d_flutter')",
      ),
    );
  });

  test('the probe finds the bundled library, and the engine uses it', () {
    if (!hasSources) return markTestSkipped('Box2D sources missing');
    expect(isBox2DAvailable, isTrue);
    final engine = Box2DPhysicsEngine()..initialize();
    addTearDown(engine.dispose);
    expect(engine.stats['backend'], 'box2d_v3');
  });

  test('the probe can be asked again', () {
    if (!hasSources) return markTestSkipped('Box2D sources missing');
    loadBox2DLibrary();
    loadBox2DLibrary();
    expect(isBox2DAvailable, isTrue);
  });
}
