// How lib/ is laid out (ARCHITECTURE.md, "Layout and layers"), checked from
// the directives of every file:
// - every file under a top-level folder of lib/src is exported by a public
//   library, or is internal: each of its public declarations is `@internal`;
// - only a folder's barrel or a public library exports, and a barrel only
//   its own folder; a library that holds parts is not a barrel;
// - no file imports a barrel that leads back to it;
// - no import loop crosses from one feature (a top-level folder of lib/src)
//   to another, no feature imports one in a layer above its own, and the
//   two dimensions never import each other;
// - src/physics_engine_factory.dart, above both dimensions, is the only file
//   directly in lib/src, and only the public libraries import it;
// - the web never reaches dart:ffi: following the `dart.library.js_interop`
//   branch of every conditional directive from the web's libraries;
// - each web stub declares what its native file adds to the public API.
// Untagged: it only reads files.

import 'dart:io';
import 'dart:math' show min;

import 'package:flutter_test/flutter_test.dart';

/// The public libraries, under lib/.
const _public = [
  'just_physics_engine.dart',
  'physics_3d.dart',
  'jolt.dart',
  'testing.dart',
];

/// The public libraries the web compiles (jolt.dart is native only).
const _webLibraries = [
  'just_physics_engine.dart',
  'physics_3d.dart',
  'testing.dart',
];

/// The one file directly in lib/src: above both dimensions.
const _factory = 'src/physics_engine_factory.dart';

/// The features — the top-level folders of lib/src — in layers, bottom to
/// top. A feature imports features in its own layer or below, never above;
/// within a layer imports go one way only (no loops), and the two
/// dimensions, though in one layer, none at all.
const _layers = [
  ['physics_2d', 'physics_3d'],
  ['testing'],
];

/// Each native file with `dart:ffi` behind it and its web stub.
const _stubs = {
  'src/physics_2d/box2d/box2d_physics_engine.dart':
      'src/physics_2d/box2d/box2d_physics_engine_stub.dart',
  'src/physics_2d/box2d/box2d_joint.dart':
      'src/physics_2d/box2d/box2d_joint_stub.dart',
  'src/physics_2d/box2d/create_physics_2d_engine.dart':
      'src/physics_2d/box2d/create_physics_2d_engine_stub.dart',
  'src/physics_3d/jolt/jolt_physics_engine.dart':
      'src/physics_3d/jolt/jolt_physics_engine_stub.dart',
};

/// What the web must never reach.
const _ffi = [
  'dart:ffi',
  'package:ffi/',
  'package:just_box_2d/ffi.dart',
  'package:just_jolt/ffi.dart',
];

final _directive = RegExp(
  r'^[ \t]*(import|export|part[ \t]+of|part)\b([^;]*);',
  multiLine: true,
);
final _uri = RegExp(r"'([^']+)'");
final _webBranch = RegExp(
  r"if\s*\(\s*dart\.library\.js_interop\s*\)\s*'([^']+)'",
);

/// One `import`, `export`, `part` or `part of` of a file, resolved.
final class _Directive {
  _Directive(this.kind, this.uri, this.target, {required this.web});

  final String kind;
  final String uri;

  /// The file it names, relative to lib/; null for another package's.
  final String? target;

  /// Whether the web takes this URI: the only one, or a conditional
  /// directive's `dart.library.js_interop` branch.
  final bool web;
}

/// A file under lib/: its path from lib/, text and directives.
final class _File {
  _File(this.path, this.text, this.directives);

  final String path;
  final String text;
  final List<_Directive> directives;

  Iterable<_Directive> of(String kind) =>
      directives.where((d) => d.kind == kind);

  /// The library it is part of, or itself.
  String get library =>
      of('part of').map((d) => d.target).nonNulls.firstOrNull ?? path;

  bool get holdsParts => of('part').isNotEmpty;

  /// A top-level folder's barrel: `src/<folder>/<folder>.dart`.
  bool get isBarrel {
    final parts = path.split('/');
    return parts.length == 3 &&
        parts[0] == 'src' &&
        parts[2] == '${parts[1]}.dart';
  }

  bool get isPublic => _public.contains(path);
}

String? _resolve(String from, String uri) {
  if (uri.startsWith('package:just_physics_engine/')) {
    return uri.substring('package:just_physics_engine/'.length);
  }
  if (uri.startsWith('package:') || uri.startsWith('dart:')) return null;
  final segments = from.split('/')..removeLast();
  for (final s in uri.split('/')) {
    if (s == '..') {
      segments.removeLast();
    } else if (s != '.') {
      segments.add(s);
    }
  }
  return segments.join('/');
}

/// Every Dart file under lib/, by path from lib/.
Map<String, _File> _readLib() {
  final root = Directory('lib');
  final files = <String, _File>{};
  for (final f in root.listSync(recursive: true).whereType<File>()) {
    if (!f.path.endsWith('.dart')) continue;
    final path = f.path.substring(root.path.length + 1).replaceAll(r'\', '/');
    var text = f.readAsStringSync().replaceAll('\r\n', '\n');
    if (text.startsWith('﻿')) text = text.substring(1);
    final directives = <_Directive>[];
    for (final m in _directive.allMatches(text)) {
      final body = m.group(2)!;
      final web = _webBranch.firstMatch(body)?.group(1);
      for (final u in _uri.allMatches(body)) {
        final uri = u.group(1)!;
        directives.add(
          _Directive(
            m.group(1)!.replaceAll(RegExp(r'\s+'), ' '),
            uri,
            _resolve(path, uri),
            web: web == null || web == uri,
          ),
        );
      }
    }
    files[path] = _File(path, text, directives);
  }
  return files;
}

/// What [from] exports, itself included, following exports onward.
Set<String> _exportedFrom(Map<String, _File> files, Iterable<String> from) {
  final seen = <String>{};
  final stack = [...from];
  while (stack.isNotEmpty) {
    final path = stack.removeLast();
    if (!seen.add(path)) continue;
    for (final d in files[path]?.of('export') ?? const <_Directive>[]) {
      if (d.target != null) stack.add(d.target!);
    }
  }
  return seen;
}

final _typeDeclaration = RegExp(
  r'^(?:(?:abstract|base|final|interface|sealed|mixin)\s+)*'
  r'(?:class|mixin|enum|typedef|extension\s+type|extension)\s+([A-Za-z_]\w*)',
);

/// Whether the annotations just above line [i] of [lines] (past its doc
/// comment) include [annotation].
bool _annotated(List<String> lines, int i, String annotation) {
  for (var j = i - 1; j >= 0; j--) {
    final above = lines[j].trim();
    if (above.startsWith('@')) {
      if (RegExp('^@$annotation\\b').hasMatch(above)) return true;
    } else if (!above.startsWith('///') && !above.startsWith('//')) {
      return false;
    }
  }
  return false;
}

/// [line] without strings and its trailing comment, for counting braces.
String _code(String line) => line
    .replaceAll(RegExp(r"'(?:[^'\\]|\\.)*'"), '')
    .replaceAll(RegExp(r'"(?:[^"\\]|\\.)*"'), '')
    .split('//')
    .first;

/// The public top-level declarations of [text]: their line and name, and
/// whether `@internal` is among their annotations.
List<({int line, String name, bool internal})> _publicDeclarations(
  String text,
) {
  final lines = text.split('\n');
  final found = <({int line, String name, bool internal})>[];
  var depth = 0;
  for (var i = 0; i < lines.length; i++) {
    final line = lines[i].trimRight();
    if (depth == 0 &&
        line.isNotEmpty &&
        !line.startsWith(RegExp(r'[\s/*@})\]]')) &&
        !line.startsWith(RegExp(r'(import|export|part|library)\b')) &&
        !line.startsWith(RegExp(r'extension\s+on\b'))) {
      final type = _typeDeclaration.firstMatch(line);
      String? name;
      if (type != null) {
        name = type.group(1);
      } else {
        // A function or variable: the last name before its parameters or
        // value, past any type arguments of its own.
        final head = line
            .split(RegExp(r'[(=;{]'))
            .first
            .trim()
            .replaceFirst(RegExp(r'<[^<>]*>$'), '');
        name = RegExp(r'[A-Za-z_]\w*').allMatches(head).lastOrNull?.group(0);
      }
      if (name != null && !name.startsWith('_')) {
        found.add((
          line: i + 1,
          name: name,
          internal: _annotated(lines, i, 'internal'),
        ));
      }
    }
    final code = _code(line);
    depth += '{'.allMatches(code).length - '}'.allMatches(code).length;
    if (depth < 0) depth = 0;
  }
  return found;
}

/// The public classes of [text], each with the public members it declares
/// that are not overrides (`@override`) nor `@internal`: what it adds to
/// the API. Constructors are the class's name, or `.named`.
Map<String, Set<String>> _classMembers(String text) {
  final lines = text.split('\n');
  final classes = <String, Set<String>>{};
  String? current;
  var depth = 0, parens = 0;
  // Whether the declaration before is still going on (a type over several
  // lines, an expression body): its lines are not new members.
  var open = false;
  const keywords = {
    'get',
    'set',
    'operator',
    'static',
    'final',
    'const',
    'late',
    'external',
    'factory',
  };
  for (var i = 0; i < lines.length; i++) {
    final line = lines[i].trimRight();
    final trimmed = line.trim();
    if (depth == 0) {
      final type = _typeDeclaration.firstMatch(line);
      if (type != null &&
          line.startsWith(RegExp(r'(abstract |final )?class'))) {
        final name = type.group(1)!;
        current = name.startsWith('_') ? null : name;
        if (current != null) classes[current] = {};
      }
    } else if (depth == 1 &&
        parens == 0 &&
        !open &&
        current != null &&
        trimmed.isNotEmpty &&
        !trimmed.startsWith(RegExp(r'[/*@})\][:.]')) &&
        !_annotated(lines, i, 'override') &&
        !_annotated(lines, i, 'internal')) {
      final accessor = RegExp(
        r'\b(?:get|set)\s+([A-Za-z_]\w*)',
      ).firstMatch(trimmed);
      String? name;
      if (accessor != null) {
        name = accessor.group(1);
      } else {
        var head = trimmed.split(RegExp(r'[(=;{]')).first.trim();
        // Type arguments out; one left open is a type going on below.
        final typeArguments = RegExp(r'<[^<>]*>');
        while (typeArguments.hasMatch(head)) {
          head = head.replaceAll(typeArguments, '');
        }
        if (!head.contains('<')) {
          name = RegExp(r'[A-Za-z_][\w.]*')
              .allMatches(head)
              .map((m) => m.group(0)!)
              .where((w) => !keywords.contains(w))
              .lastOrNull;
        }
      }
      if (name != null) {
        if (name.startsWith('$current.')) {
          name = name.substring(current.length);
        }
        if (!name.startsWith('_') && !name.startsWith('._')) {
          classes[current]!.add(name);
        }
      }
    }
    final code = _code(line);
    final atClassLevel = depth >= 1;
    depth += '{'.allMatches(code).length - '}'.allMatches(code).length;
    parens += '('.allMatches(code).length - ')'.allMatches(code).length;
    if (depth < 0) depth = 0;
    if (parens < 0) parens = 0;
    if (depth == 0) current = null;
    final end = code.trimRight();
    if (atClassLevel &&
        end.isNotEmpty &&
        !trimmed.startsWith('@') &&
        !trimmed.startsWith('//')) {
      open =
          depth != 1 ||
          parens != 0 ||
          !(end.endsWith(';') || end.endsWith('}') || end.endsWith('{'));
    }
    if (depth == 0) open = false;
  }
  return classes;
}

// The feature a file under lib/ belongs to — its top-level folder of
/// lib/src — or null for a public library and the factory.
String? _featureOf(String path) {
  final parts = path.split('/');
  return parts.length > 2 && parts[0] == 'src' ? parts[1] : null;
}

/// Every import and export from one feature to another (parts are inside
/// their library's), by (from, to): the directives that make it.
Map<(String, String), List<String>> _featureEdges(Map<String, _File> files) {
  final edges = <(String, String), List<String>>{};
  for (final file in files.values) {
    final from = _featureOf(file.path);
    if (from == null) continue;
    for (final d in [...file.of('import'), ...file.of('export')]) {
      final target = d.target;
      final to = target == null ? null : _featureOf(target);
      if (to == null || to == from) continue;
      (edges[(from, to)] ??= []).add('${file.path} ${d.kind}s ${d.uri}');
    }
  }
  return edges;
}

/// The loops of [graph]: its strongly connected parts of more than one
/// node, each sorted (Tarjan's algorithm).
List<List<String>> _loops(Map<String, Set<String>> graph) {
  final index = <String, int>{};
  final low = <String, int>{};
  final stack = <String>[];
  final onStack = <String>{};
  final loops = <List<String>>[];
  var next = 0;
  void visit(String v) {
    index[v] = low[v] = next++;
    stack.add(v);
    onStack.add(v);
    for (final w in graph[v] ?? const <String>{}) {
      if (!index.containsKey(w)) {
        visit(w);
        low[v] = min(low[v]!, low[w]!);
      } else if (onStack.contains(w)) {
        low[v] = min(low[v]!, index[w]!);
      }
    }
    if (low[v] != index[v]) return;
    final loop = <String>[];
    String w;
    do {
      w = stack.removeLast();
      onStack.remove(w);
      loop.add(w);
    } while (w != v);
    if (loop.length > 1) loops.add(loop..sort());
  }

  final nodes = {...graph.keys, for (final to in graph.values) ...to}.toList()
    ..sort();
  for (final v in nodes) {
    if (!index.containsKey(v)) visit(v);
  }
  return loops;
}

void main() {
  final files = _readLib();
  final exported = _exportedFrom(files, _public);

  test('every file is exported by a public library or is internal', () {
    final problems = <String>[];
    for (final file in files.values) {
      if (!file.path.startsWith('src/')) continue;
      if (exported.contains(file.library)) continue;
      for (final d in _publicDeclarations(file.text)) {
        if (!d.internal) {
          problems.add(
            '${file.path}:${d.line} ${d.name} — exported by no public '
            'library: export it from its barrel, or mark it @internal',
          );
        }
      }
    }
    expect(problems, isEmpty);
  });

  test('only a barrel or a public library exports, a barrel only its '
      'own folder', () {
    final problems = <String>[];
    for (final file in files.values) {
      final exports = file.of('export').toList();
      if (exports.isEmpty || file.isPublic) continue;
      if (!file.isBarrel) {
        problems.add('${file.path} exports, but is not a barrel');
        continue;
      }
      final folder = '${file.path.substring(0, file.path.lastIndexOf('/'))}/';
      for (final d in exports) {
        if (d.target != null && !d.target!.startsWith(folder)) {
          problems.add('${file.path} exports ${d.uri}, outside its folder');
        }
      }
    }
    expect(problems, isEmpty);
  });

  test('a library that holds parts is not a barrel', () {
    expect([
      for (final file in files.values)
        if (file.holdsParts && file.of('export').isNotEmpty) file.path,
    ], isEmpty);
  });

  test('no file imports a barrel that leads back to it', () {
    final problems = <String>[];
    for (final file in files.values) {
      for (final d in file.of('import')) {
        final target = d.target;
        if (target == null || files[target]?.of('export').isEmpty != false) {
          continue;
        }
        final reached = _exportedFrom(files, [target]);
        if (reached.contains(file.library) && target != file.library) {
          problems.add('${file.path} imports ${d.uri}, which exports it');
        }
      }
    }
    expect(problems, isEmpty);
  });

  final features = _featureEdges(files);
  final layerOf = {
    for (var i = 0; i < _layers.length; i++)
      for (final feature in _layers[i]) feature: i,
  };

  test('every feature has one layer, and every layer names a feature', () {
    final folders = files.keys.map(_featureOf).nonNulls.toSet();
    final listed = [for (final layer in _layers) ...layer];
    expect([
      for (final f in folders)
        if (!listed.contains(f)) '$f has no layer',
      for (final f in listed.toSet())
        if (listed.where((l) => l == f).length > 1) '$f is in two layers',
      for (final f in listed)
        if (!folders.contains(f)) '$f is no folder of lib/src',
    ], isEmpty);
  });

  test('no import loop crosses from one feature to another', () {
    final graph = <String, Set<String>>{};
    for (final (from, to) in features.keys) {
      (graph[from] ??= {}).add(to);
    }
    final problems = <String>[];
    for (final loop in _loops(graph)) {
      problems.add('a loop of ${loop.join(', ')}:');
      for (final MapEntry(key: (from, to), value: why) in features.entries) {
        if (loop.contains(from) && loop.contains(to)) {
          problems.addAll(['  $from -> $to', for (final w in why) '    $w']);
        }
      }
    }
    expect(problems, isEmpty);
  });

  test('no feature imports one in a layer above its own', () {
    expect([
      for (final MapEntry(key: (from, to), value: why) in features.entries)
        if ((layerOf[to] ?? -1) > (layerOf[from] ?? _layers.length))
          for (final w in why)
            '$from (layer ${layerOf[from]}) imports $to '
                '(layer ${layerOf[to]}): $w',
    ], isEmpty);
  });

  test('the two dimensions import nothing of each other', () {
    expect([
      for (final MapEntry(key: (from, to), value: why) in features.entries)
        if ({from, to}.containsAll(['physics_2d', 'physics_3d'])) ...why,
    ], isEmpty);
  });

  test('the factory is the only file directly in lib/src, and only the '
      'public libraries import it', () {
    expect([
      for (final path in files.keys)
        if (path.split('/').length == 2 &&
            path.startsWith('src/') &&
            path != _factory)
          path,
    ], isEmpty);
    expect(files.containsKey(_factory), isTrue);
    expect([
      for (final file in files.values)
        if (!file.isPublic)
          for (final d in [...file.of('import'), ...file.of('export')])
            if (d.target == _factory) '${file.path} ${d.kind}s ${d.uri}',
    ], isEmpty);
  });

  test('the web never reaches dart:ffi', () {
    // Every file the web compiles: from its libraries, along the web's
    // branch of each directive, parts included.
    final reached = <String>{};
    final via = <String, String>{};
    final stack = [..._webLibraries];
    while (stack.isNotEmpty) {
      final path = stack.removeLast();
      if (!reached.add(path)) continue;
      for (final d in files[path]?.directives ?? const <_Directive>[]) {
        if (!d.web || d.target == null || d.kind == 'part of') continue;
        via.putIfAbsent(d.target!, () => path);
        stack.add(d.target!);
      }
    }
    String chain(String path) {
      final out = [path];
      var at = path;
      while (via[at] != null) {
        at = via[at]!;
        out.add(at);
      }
      return out.reversed.join(' -> ');
    }

    expect(
      reached,
      contains('src/physics_2d/box2d/box2d_physics_engine_stub.dart'),
    );
    expect(
      reached,
      isNot(contains('src/physics_2d/box2d/box2d_physics_engine.dart')),
    );
    expect([
      for (final path in reached)
        for (final d in files[path]?.of('import') ?? const <_Directive>[])
          if (d.web && _ffi.any(d.uri.startsWith)) '${chain(path)}: ${d.uri}',
    ], isEmpty);
  });

  test('each web stub declares what its native file adds', () {
    final problems = <String>[];
    for (final MapEntry(key: native, value: stub) in _stubs.entries) {
      expect(files.containsKey(native), isTrue, reason: native);
      expect(files.containsKey(stub), isTrue, reason: stub);
      final nativeClasses = _classMembers(files[native]!.text);
      final stubClasses = _classMembers(files[stub]!.text);
      final nativeTop = {
        for (final d in _publicDeclarations(files[native]!.text)) d.name,
      };
      final stubTop = {
        for (final d in _publicDeclarations(files[stub]!.text)) d.name,
      };
      for (final name in nativeTop.difference(stubTop)) {
        problems.add('$stub has no $name');
      }
      for (final MapEntry(key: name, value: members) in nativeClasses.entries) {
        final mine = stubClasses[name];
        if (mine == null) continue; // reported above
        for (final m in members.difference(mine)) {
          problems.add('$stub: $name has no $m');
        }
      }
    }
    expect(problems, isEmpty);
  });

  test('the scanners read declarations, members and their annotations', () {
    final source = '''
/// Doc.
@internal
final class Kept {
  /// A field.
  final int count;

  Kept(this.count);

  Kept.named() : count = 0;

  @override
  String toString() => '';

  @internal
  void hidden() {}

  bool get ready => true;

  void method(
    int a,
  ) {}

  static const x = 1;

  void _private() {}
}

class Plain {}

@internal
Future<int> load() async => 1;

const int limit = 3;

T pick<T>(T a) => a;

extension on int {}

final class _Hidden {}
''';
    expect(_publicDeclarations(source).map((d) => '${d.name}:${d.internal}'), [
      'Kept:true',
      'Plain:false',
      'load:true',
      'limit:false',
      'pick:false',
    ]);
    expect(_classMembers(source), {
      'Kept': {'count', 'Kept', '.named', 'ready', 'method', 'x'},
      'Plain': <String>{},
    });
  });
}
