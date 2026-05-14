import 'dart:io';

import 'package:code_assets/code_assets.dart'; // OS enum + HookConfigCodeConfig extension
import 'package:hooks/hooks.dart';
import 'package:native_toolchain_c/native_toolchain_c.dart';

void main(List<String> args) async {
  await build(args, (input, output) async {
    if (!input.config.buildCodeAssets) return;

    final nativeDir = input.packageRoot.resolve('src/native/');
    final box2dSrcDir = nativeDir.resolve('third_party/box2d/src/');

    // Skip gracefully when the Box2D git submodule has not been initialized.
    // Box2DPhysicsEngine.initialize() will catch the missing library and fall
    // back to the pure-Dart PhysicsEngine automatically.
    final box2dSrcDirEntity = Directory.fromUri(box2dSrcDir);
    if (!box2dSrcDirEntity.existsSync()) return;

    // Collect all Box2D 3.0 C source files recursively.
    final box2dSources = box2dSrcDirEntity
        .listSync(recursive: true)
        .whereType<File>()
        .where((f) => f.path.endsWith('.c'))
        .map((f) => f.path)
        .toList();

    if (box2dSources.isEmpty) return;

    final includes = [
      nativeDir.toFilePath(),
      nativeDir.resolve('third_party/box2d/include/').toFilePath(),
    ];
    final sharedFlags = [
      '-O2',
      '-DB2_ENABLE_MULTITHREADING',
      if (input.config.code.targetOS == OS.android)
        '-Wno-unused-command-line-argument',
    ];

    final staticOutput = BuildOutputBuilder();
    await CBuilder.library(
      name: 'box2d_core',
      assetName: 'box2d_core',
      sources: box2dSources,
      includes: includes,
      flags: sharedFlags,
      std: 'c17',
      buildMode: BuildMode.release,
      linkModePreference: LinkModePreference.static,
    ).run(input: input, output: staticOutput);

    await CBuilder.library(
      name: 'box2d_flutter',
      assetName: 'box2d_flutter',
      sources: [
        nativeDir.resolve('thread_pool.cpp').toFilePath(),
        nativeDir.resolve('box2d_wrapper.cpp').toFilePath(),
      ],
      includes: includes,
      libraryDirectories: const ['.'],
      libraries: const ['box2d_core'],
      flags: sharedFlags,
      std: 'c++17',
      language: Language.cpp,
      cppLinkStdLib: input.config.code.targetOS == OS.android
          ? 'c++_static'
          : null,
      buildMode: BuildMode.release,
    ).run(input: input, output: output);
  });
}
