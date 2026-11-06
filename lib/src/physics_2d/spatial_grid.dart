part of 'physics_2d_engine.dart';

/// Two bodies, in no order: equal to the same two the other way round. The
/// pure-Dart engine's contact, sensor and jointed-pair bookkeeping key.
@internal
class BodyPair {
  /// The pair of [a] and [b].
  BodyPair(this.a, this.b);

  /// One body.
  final Physics2DBody a;

  /// The other.
  final Physics2DBody b;

  @override
  bool operator ==(Object other) =>
      other is BodyPair &&
      ((other.a == a && other.b == b) || (other.a == b && other.b == a));

  @override
  int get hashCode => a.hashCode ^ b.hashCode;
}

/// The pure-Dart engine's broad phase: a uniform grid of [cellSize] cells,
/// each holding the active bodies whose bounds reach into it.
///
/// Nothing is allocated per body or per pair a step once it has settled:
/// cell lists are pooled, each body's cell range is updated in place, and a
/// [BodyPair] is made only for a pair that can touch, once however many
/// cells the two share.
@internal
class SpatialGrid {
  /// A grid of [cellSize] × [cellSize] cells.
  SpatialGrid(this.cellSize);

  /// The side of a cell, in units.
  final double cellSize;

  final Map<int, List<_GridEntry>> _cells = {};

  /// Pool of previously allocated cell lists, reused on next rebuild.
  final List<List<_GridEntry>> _listPool = [];

  /// Each body in the grid, with its id and the cells it occupies (updated
  /// in place).
  final Map<Physics2DBody, _GridEntry> _trackedBodies = {};

  /// Ids of bodies that left, for the next to come: ids stay small, so a
  /// pair's key (see [_pairKey]) stays a small int.
  final List<int> _freeIds = [];
  int _nextId = 0;
  final Set<Physics2DBody> _seenBodies = <Physics2DBody>{};

  // Persistent scratch buffers — cleared and reused each frame to avoid
  // heap allocation in hot paths.
  final _CellRange _scratch = _CellRange();
  final List<Physics2DBody> _removedScratch = [];
  final List<BodyPair> _pairBuffer = [];

  /// The pairs found this step, by their key (see [_pairKey]).
  final Set<int> _seenPairs = {};

  int _lastDirtyBodyCount = 0;

  int _hash(int x, int y) {
    // Two independent primes — no bit-shift so y's entropy is fully preserved.
    return (x * 73856093) ^ (y * 83492791);
  }

  /// Clear all cells, returning their lists to the pool for reuse.
  void clear() {
    for (final list in _cells.values) {
      list.clear();
      _listPool.add(list);
    }
    _cells.clear();
    _trackedBodies.clear();
    _freeIds.clear();
    _nextId = 0;
    _seenBodies.clear();
    _lastDirtyBodyCount = 0;
  }

  /// Obtain a list from the pool or allocate a new one.
  List<_GridEntry> _acquireList() {
    return _listPool.isNotEmpty ? _listPool.removeLast() : <_GridEntry>[];
  }

  /// [body]'s entry, with a free id, placed at [_scratch]'s range.
  _GridEntry _enter(Physics2DBody body) {
    final id = _freeIds.isNotEmpty ? _freeIds.removeLast() : _nextId++;
    final entry = _GridEntry(body, id)..setFrom(_scratch);
    _insertIntoRange(entry);
    _trackedBodies[body] = entry;
    return entry;
  }

  /// Takes [entry] out of its cells and frees its id.
  void _leave(_GridEntry entry) {
    _removeFromRange(entry);
    _freeIds.add(entry.id);
  }

  /// Puts [body] in the cells it covers now.
  void insert(Physics2DBody body) {
    if (_trackedBodies.containsKey(body)) return;
    if (_computeRange(body, _scratch)) _enter(body);
  }

  /// Brings the grid up to [bodies]: each one moved to the cells it covers
  /// now, those gone or inactive taken out.
  void syncBodies(Iterable<Physics2DBody> bodies) {
    _seenBodies.clear();
    var dirtyBodies = 0;

    for (final body in bodies) {
      _seenBodies.add(body);

      final covers = _computeRange(body, _scratch);
      final previousRange = _trackedBodies[body];

      if (!covers) {
        if (previousRange != null) {
          _leave(previousRange);
          _trackedBodies.remove(body);
          dirtyBodies++;
        }
        continue;
      }

      if (previousRange != null && previousRange.sameAs(_scratch)) {
        continue;
      }

      if (previousRange != null) {
        _removeFromRange(previousRange);
        previousRange.setFrom(_scratch);
        _insertIntoRange(previousRange);
      } else {
        _enter(body);
      }
      dirtyBodies++;
    }

    _removedScratch.clear();
    for (final body in _trackedBodies.keys) {
      if (!_seenBodies.contains(body)) {
        _removedScratch.add(body);
      }
    }

    for (final body in _removedScratch) {
      final previousRange = _trackedBodies.remove(body);
      if (previousRange != null) {
        _leave(previousRange);
        dirtyBodies++;
      }
    }

    _lastDirtyBodyCount = dirtyBodies;
  }

  /// Takes [body] out of the grid.
  void removeBody(Physics2DBody body) {
    final previousRange = _trackedBodies.remove(body);
    if (previousRange != null) _leave(previousRange);
  }

  /// Writes the cells [body] covers into [out]; false for an inactive body,
  /// which is in no cell.
  bool _computeRange(Physics2DBody body, _CellRange out) {
    if (!body.isActive) return false;

    final bounds = body.boundsAt(body.position.toOffset());
    out
      ..minX = (bounds.left / cellSize).floor()
      ..minY = (bounds.top / cellSize).floor()
      ..maxX = (bounds.right / cellSize).floor()
      ..maxY = (bounds.bottom / cellSize).floor();
    return true;
  }

  void _insertIntoRange(_GridEntry entry) {
    for (int x = entry.minX; x <= entry.maxX; x++) {
      for (int y = entry.minY; y <= entry.maxY; y++) {
        final hash = _hash(x, y);
        (_cells[hash] ?? (_cells[hash] = _acquireList())).add(entry);
      }
    }
  }

  void _removeFromRange(_GridEntry entry) {
    for (int x = entry.minX; x <= entry.maxX; x++) {
      for (int y = entry.minY; y <= entry.maxY; y++) {
        final hash = _hash(x, y);
        final bucket = _cells[hash];
        if (bucket == null) continue;

        bucket.remove(entry);
        if (bucket.isEmpty) {
          _cells.remove(hash);
          _listPool.add(bucket);
        }
      }
    }
  }

  /// Whether [a] and [b] can touch at all: never two static bodies, and
  /// never two that are neither dynamic nor a sensor (Box2D makes no contact
  /// unless one side is dynamic; a sensor sees kinematic and static bodies).
  static bool _canTouch(Physics2DBody a, Physics2DBody b) {
    if (a.isStatic && b.isStatic) return false;
    return a.isDynamic || b.isDynamic || a.isSensor || b.isSensor;
  }

  /// One int for the pair of ids [x] and [y], whichever order they come
  /// in: below 2^30 while both ids are below 2^15, so it stays a small int
  /// (unboxed) on every platform; past that, keys from 2^62 up.
  static int _pairKey(int x, int y) {
    final lo = x < y ? x : y, hi = x < y ? y : x;
    return hi < 32768 ? lo * 32768 + hi : (1 << 62) | (lo << 31) | hi;
  }

  /// The pairs of bodies sharing a cell that can touch, each once.
  ///
  /// Returns a pre-allocated internal buffer; do not hold a reference beyond
  /// the current physics tick.
  List<BodyPair> potentialPairs() {
    _pairBuffer.clear();
    _seenPairs.clear();
    for (final bin in _cells.values) {
      if (bin.length > 1) {
        for (int i = 0; i < bin.length; i++) {
          final ea = bin[i];
          final a = ea.body;
          for (int j = i + 1; j < bin.length; j++) {
            final eb = bin[j];
            final b = eb.body;
            // Cell-hash collisions can place the same body in a bucket twice,
            // producing meaningless self-pairs.
            if (identical(a, b) || !_canTouch(a, b)) continue;
            if (_seenPairs.add(_pairKey(ea.id, eb.id))) {
              _pairBuffer.add(BodyPair(a, b));
            }
          }
        }
      }
    }
    return _pairBuffer;
  }

  /// How many bodies changed cells in the last [syncBodies].
  int get dirtyBodyCount => _lastDirtyBodyCount;

  /// How many cells hold a body.
  int get trackedCellCount => _cells.length;

  /// How many bodies are in the grid.
  int get trackedBodyCount => _trackedBodies.length;
}

/// A body in the grid: its id there, and the cells it covers.
class _GridEntry extends _CellRange {
  _GridEntry(this.body, this.id);

  final Physics2DBody body;
  final int id;
}

/// A rectangle of cells, updated in place.
class _CellRange {
  int minX = 0;
  int minY = 0;
  int maxX = 0;
  int maxY = 0;

  bool sameAs(_CellRange o) =>
      o.minX == minX && o.minY == minY && o.maxX == maxX && o.maxY == maxY;

  void setFrom(_CellRange o) {
    minX = o.minX;
    minY = o.minY;
    maxX = o.maxX;
    maxY = o.maxY;
  }
}
