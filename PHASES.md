# Phase Tracking

This document tracks major implementation phases for `just_physics_engine`.

## Phase 1 - Baseline + Determinism Foundations (Completed)

Scope
- Deterministic replay baseline and parity harness setup.
- Arena/hash scaffolding in pure-Dart backend.
- Benchmark and test harness stabilization.

Highlights
- Added deterministic replay and parity coverage.
- Added deterministic hash diagnostics and cadence controls.
- Added baseline benchmark matrix and tuning defines.

Status
- Completed.

## Phase 2 - Solver Parity + Warm-Start/Block-Solve Stabilization (Completed)

Scope
- Improve pure-Dart contact solve parity and edge-contact stability.
- Add warm-start controls and multi-point manifold continuity.
- Add block normal/friction solving with warm-state gating and hysteresis.
- Expand deterministic diagnostics and regression coverage.

Highlights
- Multi-point manifold solving with rotational response.
- Warm-start continuity: age, normal, anchor, feature-id controls.
- Two-point block normal/friction solve with hysteresis thresholds.
- Transition observability: per-step counters, reason counters, cumulative totals,
  and rolling transition rates.

Status
- Completed and validated by the Phase 2 regression suite.

## Phase 3 - Long-Run Observability and Churn Attribution (Defined, In Progress)

Goals
- Make solver-state transitions easier to debug in long scenarios.
- Reduce ambiguity in hysteresis deactivation reasons.
- Keep deterministic diagnostics stable while adding finer attribution.

Planned Work
- Add finer reason diagnostics for stale-pair removal vs in-contact non-two-point transitions.
- Add long-run churn views and optional rate/aggregation extensions.
- Add focused deterministic regressions for reason attribution paths.

### Phase 3 Start - Slice 1 (Implemented)

Added
- Split non-two-point hysteresis deactivation attribution into:
  - `blockNormalHysteresisDeactivatedNonTwoPointContact`
  - `blockNormalHysteresisDeactivatedPairDropped`
  - `blockFrictionHysteresisDeactivatedNonTwoPointContact`
  - `blockFrictionHysteresisDeactivatedPairDropped`
- Preserved existing aggregate counters:
  - `blockNormalHysteresisDeactivatedNonTwoPoint`
  - `blockFrictionHysteresisDeactivatedNonTwoPoint`

Coverage
- Extended non-two-point topology-change regression assertions.
- Added deterministic regression for stale-pair (pair-dropped) deactivation reason.
- Included new counters in deterministic stats snapshot comparison.

Validation
- `flutter analyze lib test`
- `flutter test test/phase2_arena_and_hash_test.dart`
