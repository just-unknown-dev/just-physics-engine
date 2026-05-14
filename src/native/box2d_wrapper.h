// box2d_wrapper.h — pure-C public API consumed by Dart FFI via ffigen.
//
// All Box2D 3.0 opaque ID structs (b2WorldId, b2BodyId) are packed into
// int64_t so Dart FFI never has to pass structs by value across the boundary.
//
// ID encoding:
//   b2WorldId → int64: index1[15:0] | generation[31:16]
//   b2BodyId  → int64: index1[31:0] | world0[47:32] | generation[63:48]

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── World ────────────────────────────────────────────────────────────────────

/// Create a Box2D world with a native thread pool injected into the task system.
/// numThreads <= 0 → hardware_concurrency - 1.
/// Returns packed int64 world handle; 0 on failure.
int64_t b2w_createWorld(float gravityX, float gravityY, int32_t numThreads);

/// Destroy a Box2D world and its associated thread pool.
/// The pool is destroyed AFTER b2DestroyWorld to allow any in-flight
/// finishTask calls to complete.
void b2w_destroyWorld(int64_t worldHandle);

/// Advance the simulation by timeStep seconds with subSteps internal iterations.
/// Box2D 3.0 recommends subSteps = 4 for most games.
void b2w_step(int64_t worldHandle, float timeStep, int32_t subSteps);

/// Overwrite world gravity at runtime.
void b2w_setGravity(int64_t worldHandle, float gx, float gy);

// ── Bodies ───────────────────────────────────────────────────────────────────

/// Create a dynamic (simulated) body. Returns packed int64 body handle.
int64_t b2w_createDynamicBody(int64_t worldHandle,
                               float posX, float posY, float angle);

/// Create a static (immovable) body. Returns packed int64 body handle.
int64_t b2w_createStaticBody(int64_t worldHandle,
                              float posX, float posY, float angle);

/// Destroy a body and detach all its shapes.
void b2w_destroyBody(int64_t bodyHandle);

// ── Shapes ───────────────────────────────────────────────────────────────────

/// Attach a circle fixture to a body.
void b2w_addCircleShape(int64_t bodyHandle, float radius,
                         float density, float friction, float restitution);

/// Attach a box (axis-aligned rectangle) fixture. halfW/halfH are half-extents.
void b2w_addBoxShape(int64_t bodyHandle, float halfW, float halfH,
                      float density, float friction, float restitution);

/// Attach a convex polygon fixture.
/// verts: interleaved (x0,y0, x1,y1, ...) array of `count` vertices.
/// Box2D 3.0 will compute the convex hull internally.
void b2w_addPolygonShape(int64_t bodyHandle,
                          const float* verts, int32_t count,
                          float density, float friction, float restitution);

// ── Forces & impulses ────────────────────────────────────────────────────────

/// Apply a force at the body's mass centre (accumulates until next step).
void b2w_applyForce(int64_t bodyHandle, float fx, float fy);

/// Apply an instantaneous linear impulse at the body's mass centre.
void b2w_applyLinearImpulse(int64_t bodyHandle, float ix, float iy);

/// Apply a torque (angular force) to the body.
void b2w_applyTorque(int64_t bodyHandle, float torque);

/// Directly set the linear velocity (bypasses force accumulation).
void b2w_setLinearVelocity(int64_t bodyHandle, float vx, float vy);

// ── Zero-copy bulk transform extraction ──────────────────────────────────────
//
// Designed for the hot path after each physics step.
// Both buffers are allocated by Dart via calloc and passed in — no heap
// allocation occurs inside this function.
//
// `handles`  Pointer to a Dart calloc<Int64>(n) buffer of packed body IDs.
// `buffer`   Pointer to a Dart calloc<Float>(n*6) buffer.
//            Layout per body: [posX, posY, angle, velX, velY, 0.0f]
// `count`    Number of bodies.
//
// Dart reads the result via Pointer<Float>.asTypedList(n*6) — zero copy.
void b2w_bulkExtractTransforms(const int64_t* handles,
                                float* buffer,
                                int32_t count);

// ── Contact events (polled once per step) ────────────────────────────────────
//
// Box2D 3.0 uses a polling model: after b2w_step, call these functions to
// read contact begin/end events for the last step.
// The returned arrays are valid only until the next b2w_step call.

/// Number of begin-touch contact events from the last step.
int32_t b2w_getContactBeginCount(int64_t worldHandle);

/// Read one begin-touch event by index.
/// Writes packed body handles and contact normal into the out-parameters.
void b2w_getContactBeginEvent(int64_t worldHandle, int32_t index,
                               int64_t* outBodyA, int64_t* outBodyB,
                               float* outNormalX, float* outNormalY);

/// Number of end-touch contact events from the last step.
int32_t b2w_getContactEndCount(int64_t worldHandle);

/// Read one end-touch event by index.
void b2w_getContactEndEvent(int64_t worldHandle, int32_t index,
                             int64_t* outBodyA, int64_t* outBodyB);

// ── NativeCallable.listener — cross-thread collision audio callback ───────────
//
// Register a Dart function (via NativeCallable<ImpactCallbackC>.listener)
// that fires on the Dart main isolate whenever a collision's approach speed
// exceeds speedThreshold.
//
// Box2D steps on worker threads; NativeCallable.listener safely marshals
// the call to the main Dart isolate's event loop.

typedef void (*ImpactCallbackFn)(int64_t bodyA, int64_t bodyB, float speed);

/// Install or replace the impact callback for a world.
/// Pass NULL to remove the callback.
void b2w_setImpactCallback(int64_t worldHandle,
                            ImpactCallbackFn callback,
                            float speedThreshold);

// ── NativeFinalizer-compatible destructors ────────────────────────────────────
//
// NativeFinalizer requires a native function with signature void(void*).
// These wrappers receive the packed int64 handle reinterpreted as a void*
// token (set via Pointer<Void>.fromAddress(handle) on the Dart side), cast
// it back to int64_t, and call the real destroy function.
// Only used as safety-net GC finalizers — prefer explicit dispose().

void b2w_finalizer_world(void* token);
void b2w_finalizer_body(void* token);

#ifdef __cplusplus
}
#endif
