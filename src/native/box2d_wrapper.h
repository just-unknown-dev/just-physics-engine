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

// Windows DLLs export nothing by default (unlike ELF/Mach-O, where non-static
// symbols are visible automatically) — without this, link.exe silently
// produces a DLL with an empty export table and every dart:ffi
// DynamicLibrary.lookup() call fails with "procedure not found", even though
// the .dll itself loads fine. See CHANGELOG for the fix history.
#if defined(_WIN32)
  #define B2W_EXPORT __declspec(dllexport)
#else
  #define B2W_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ── World ────────────────────────────────────────────────────────────────────

/// Create a Box2D world with a native thread pool injected into the task system.
/// numThreads <= 0 → hardware_concurrency - 1.
/// Returns packed int64 world handle; 0 on failure.
B2W_EXPORT int64_t b2w_createWorld(float gravityX, float gravityY, int32_t numThreads);

/// Destroy a Box2D world and its associated thread pool.
/// The pool is destroyed AFTER b2DestroyWorld to allow any in-flight
/// finishTask calls to complete.
B2W_EXPORT void b2w_destroyWorld(int64_t worldHandle);

/// Advance the simulation by timeStep seconds with subSteps internal iterations.
/// Box2D 3.0 recommends subSteps = 4 for most games.
B2W_EXPORT void b2w_step(int64_t worldHandle, float timeStep, int32_t subSteps);

/// Overwrite world gravity at runtime.
B2W_EXPORT void b2w_setGravity(int64_t worldHandle, float gx, float gy);

// ── Bodies ───────────────────────────────────────────────────────────────────

/// Create a dynamic (simulated) body. Returns packed int64 body handle.
B2W_EXPORT int64_t b2w_createDynamicBody(int64_t worldHandle,
                               float posX, float posY, float angle);

/// Create a static (immovable) body. Returns packed int64 body handle.
B2W_EXPORT int64_t b2w_createStaticBody(int64_t worldHandle,
                              float posX, float posY, float angle);

/// Destroy a body and detach all its shapes.
B2W_EXPORT void b2w_destroyBody(int64_t bodyHandle);

// ── Shapes ───────────────────────────────────────────────────────────────────

/// Attach a circle fixture to a body, centred at (cx, cy) in body space.
B2W_EXPORT void b2w_addCircleShape(int64_t bodyHandle, float cx, float cy,
                         float radius, float density, float friction,
                         float restitution);

/// Attach a box (axis-aligned rectangle) fixture. halfW/halfH are half-extents.
B2W_EXPORT void b2w_addBoxShape(int64_t bodyHandle, float halfW, float halfH,
                      float density, float friction, float restitution);

/// Attach a convex polygon fixture.
/// verts: interleaved (x0,y0, x1,y1, ...) array of `count` vertices.
/// Box2D 3.0 will compute the convex hull internally.
B2W_EXPORT void b2w_addPolygonShape(int64_t bodyHandle,
                          const float* verts, int32_t count,
                          float density, float friction, float restitution);

/// Attach a convex polygon with rounded corners (Box2D radius parameter).
/// verts: interleaved (x0,y0, x1,y1, ...) array of `count` vertices.
B2W_EXPORT void b2w_addRoundedPolygonShape(int64_t bodyHandle,
                                 const float* verts, int32_t count,
                                 float cornerRadius,
                                 float density, float friction, float restitution);

/// Attach a capsule (two circles of equal radius joined by a segment).
/// (cx1,cy1) and (cx2,cy2) are the two center offsets relative to body origin.
B2W_EXPORT void b2w_addCapsuleShape(int64_t bodyHandle,
                          float cx1, float cy1, float cx2, float cy2,
                          float radius,
                          float density, float friction, float restitution);

/// Attach a single line-segment fixture (static geometry).
B2W_EXPORT void b2w_addSegmentShape(int64_t bodyHandle,
                          float x1, float y1, float x2, float y2,
                          float density, float friction, float restitution);

/// Attach a chain-of-segments fixture.
/// points: interleaved (x0,y0, x1,y1, ...) array of `count` vertices.
/// If loop != 0 the last point connects back to the first.
/// Returns a packed int64 chain handle; pass to b2w_destroyChain to remove it.
B2W_EXPORT int64_t b2w_addChainShape(int64_t bodyHandle,
                           const float* points, int32_t count, int32_t loop,
                           float friction, float restitution);

/// Destroy a chain shape previously created with b2w_addChainShape.
B2W_EXPORT void b2w_destroyChain(int64_t chainHandle);

// ── Forces & impulses ────────────────────────────────────────────────────────

/// Apply a force at the body's mass centre (accumulates until next step).
B2W_EXPORT void b2w_applyForce(int64_t bodyHandle, float fx, float fy);

/// Apply an instantaneous linear impulse at the body's mass centre.
B2W_EXPORT void b2w_applyLinearImpulse(int64_t bodyHandle, float ix, float iy);

/// Apply a torque (angular force) to the body.
B2W_EXPORT void b2w_applyTorque(int64_t bodyHandle, float torque);

/// Directly set the linear velocity (bypasses force accumulation).
B2W_EXPORT void b2w_setLinearVelocity(int64_t bodyHandle, float vx, float vy);

// ── Zero-copy bulk transform extraction ──────────────────────────────────────
//
// Designed for the hot path after each physics step.
// Both buffers are allocated by Dart via calloc and passed in — no heap
// allocation occurs inside this function.
//
// `handles`  Pointer to a Dart calloc<Int64>(n) buffer of packed body IDs.
// `buffer`   Pointer to a Dart calloc<Float>(n*7) buffer.
//            Layout per body:
//            [posX, posY, angle, velX, velY, isAwake(1.0/0.0), angularVel]
// `count`    Number of bodies.
//
// Dart reads the result via Pointer<Float>.asTypedList(n*7) — zero copy.
B2W_EXPORT void b2w_bulkExtractTransforms(const int64_t* handles,
                                float* buffer,
                                int32_t count);

// ── Contact events (polled once per step) ────────────────────────────────────
//
// Box2D 3.0 uses a polling model: after b2w_step, call these functions to
// read contact begin/end events for the last step.
// The returned arrays are valid only until the next b2w_step call.

/// Number of begin-touch contact events from the last step.
B2W_EXPORT int32_t b2w_getContactBeginCount(int64_t worldHandle);

/// Read one begin-touch event by index.
/// Writes packed body handles and contact normal into the out-parameters.
B2W_EXPORT void b2w_getContactBeginEvent(int64_t worldHandle, int32_t index,
                               int64_t* outBodyA, int64_t* outBodyB,
                               float* outNormalX, float* outNormalY);

/// Number of end-touch contact events from the last step.
B2W_EXPORT int32_t b2w_getContactEndCount(int64_t worldHandle);

/// Read one end-touch event by index.
B2W_EXPORT void b2w_getContactEndEvent(int64_t worldHandle, int32_t index,
                             int64_t* outBodyA, int64_t* outBodyB);

// ── Sensor events (polled once per step) ─────────────────────────────────────

/// Number of sensor-begin events from the last step.
B2W_EXPORT int32_t b2w_getSensorBeginCount(int64_t worldHandle);

/// Read one sensor-begin event by index.
/// Writes the sensor body and visitor body as packed int64 handles.
B2W_EXPORT void b2w_getSensorBeginEvent(int64_t worldHandle, int32_t index,
                              int64_t* outSensorBody, int64_t* outVisitorBody);

/// Number of sensor-end events from the last step.
B2W_EXPORT int32_t b2w_getSensorEndCount(int64_t worldHandle);

/// Read one sensor-end event by index.
B2W_EXPORT void b2w_getSensorEndEvent(int64_t worldHandle, int32_t index,
                            int64_t* outSensorBody, int64_t* outVisitorBody);

// ── Sensor / filter setters ───────────────────────────────────────────────────

/// Mark all shapes on a body as sensor (isSensor != 0) or solid (isSensor == 0).
B2W_EXPORT void b2w_setBodySensor(int64_t bodyHandle, int32_t isSensor);

/// Set the collision filter on all shapes of a body.
B2W_EXPORT void b2w_setBodyFilter(int64_t bodyHandle,
                        uint32_t categoryBits, uint32_t maskBits,
                        int32_t groupIndex);

/// Enable or disable Continuous Collision Detection (bullet mode) on a body.
/// isBullet != 0 → CCD enabled; prevents fast bodies from tunnelling through
/// thin static geometry. Only meaningful for dynamic bodies.
B2W_EXPORT void b2w_setBodyBullet(int64_t bodyHandle, int32_t isBullet);

/// Scale gravity's effect on a body. 1.0 = normal gravity (the default),
/// 0.0 = unaffected by world gravity (matches PhysicsBody.useGravity = false).
B2W_EXPORT void b2w_setBodyGravityScale(int64_t bodyHandle, float gravityScale);

/// Force a body's awake/asleep state. Bodies default to awake at creation;
/// use this to start a body asleep (matches PhysicsBody.isAwake = false).
/// Not needed to WAKE a body — collisions and applied forces/impulses do
/// that automatically, and b2w_bulkExtractTransforms reports the resulting
/// state back every step.
B2W_EXPORT void b2w_setBodyAwake(int64_t bodyHandle, int32_t awake);

/// Allow or forbid a body to fall asleep. enabled == 0 keeps it awake (and
/// simulated) for as long as it exists, at rest or not; bodies may sleep by
/// default.
B2W_EXPORT void b2w_setBodySleepEnabled(int64_t bodyHandle, int32_t enabled);

/// Lock or unlock a body's rotation. fixed != 0 → the body's angle never
/// changes, regardless of torque/angular impulses from collisions (e.g.
/// friction against a static obstacle) — the standard fix for top-down
/// characters that should slide, not tumble, when they clip scenery.
/// Only meaningful for dynamic bodies.
B2W_EXPORT void b2w_setBodyFixedRotation(int64_t bodyHandle, int32_t fixed);

/// Override a body's simulated mass, replacing Box2D's density-derived value.
///
/// Box2D computes mass from shape area × density (b2ShapeDef.density, always
/// passed as 1.0 or 0.0 by this wrapper — see b2w_addCircleShape etc.), NOT
/// from any gameplay-facing "mass" the caller intended. Without this call, a
/// body's actual simulated mass silently depends on its shape's area (e.g. a
/// circle of radius 5 masses ~78.5, not 1.0), decoupling force/impulse
/// magnitudes and collision response from PhysicsBody.mass. Preserves the
/// shape-derived center of mass and scales rotationalInertia proportionally
/// so the mass/inertia ratio — and therefore angular response — is
/// unaffected by the override. Call after all shape fixtures are attached.
B2W_EXPORT void b2w_setBodyMass(int64_t bodyHandle, float mass);

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
B2W_EXPORT void b2w_setImpactCallback(int64_t worldHandle,
                            ImpactCallbackFn callback,
                            float speedThreshold);

// ── Body movement events (polled once per step) ───────────────────────────────

/// Number of body-move events from the last step.
/// A body-move event fires for every dynamic body that moved, including when it
/// goes to sleep (check fellAsleep flag).
B2W_EXPORT int32_t b2w_getBodyMoveEventCount(int64_t worldHandle);

/// Read one body-move event by index.
/// outBodyHandle: packed body int64 handle.
/// outFellAsleep: 1 if the body fell asleep this step, else 0.
B2W_EXPORT void b2w_getBodyMoveEvent(int64_t worldHandle, int32_t index,
                           int64_t* outBodyHandle, int32_t* outFellAsleep);

// ── Joints ────────────────────────────────────────────────────────────────────
//
// Joint IDs are packed into int64_t using the same scheme as body IDs.
// Returns 0 if creation fails.

/// Create a revolute joint (hinge) between two bodies at a shared world anchor.
B2W_EXPORT int64_t b2w_createRevoluteJoint(int64_t worldHandle,
                                 int64_t bodyA, int64_t bodyB,
                                 float anchorX, float anchorY);

/// Create a prismatic (slider) joint along the given world axis.
B2W_EXPORT int64_t b2w_createPrismaticJoint(int64_t worldHandle,
                                  int64_t bodyA, int64_t bodyB,
                                  float anchorX, float anchorY,
                                  float axisX,   float axisY);

/// Create a distance joint keeping bodies within [minLen, maxLen].
B2W_EXPORT int64_t b2w_createDistanceJoint(int64_t worldHandle,
                                 int64_t bodyA, int64_t bodyB,
                                 float minLen, float maxLen);

/// Create a mouse joint that pulls bodyB toward a target point.
B2W_EXPORT int64_t b2w_createMouseJoint(int64_t worldHandle,
                               int64_t bodyB,
                               float targetX, float targetY);

/// Create a weld (rigid) joint locking two bodies at a world anchor.
B2W_EXPORT int64_t b2w_createWeldJoint(int64_t worldHandle,
                              int64_t bodyA, int64_t bodyB,
                              float anchorX, float anchorY);

/// Create a wheel joint (body attached via spring along an axis).
B2W_EXPORT int64_t b2w_createWheelJoint(int64_t worldHandle,
                               int64_t bodyA, int64_t bodyB,
                               float anchorX, float anchorY,
                               float axisX,   float axisY);

/// Destroy a joint by handle.
B2W_EXPORT void b2w_destroyJoint(int64_t jointHandle);

// ── Joint configuration ───────────────────────────────────────────────────────

/// Enable/disable revolute joint limits.
B2W_EXPORT void b2w_setRevoluteLimits(int64_t jointHandle,
                            float lower, float upper, int32_t enable);

/// Enable/disable revolute joint motor.
B2W_EXPORT void b2w_setRevoluteMotor(int64_t jointHandle,
                           float speed, float maxTorque, int32_t enable);

/// Enable/disable prismatic joint limits.
B2W_EXPORT void b2w_setPrismaticLimits(int64_t jointHandle,
                             float lower, float upper, int32_t enable);

/// Enable/disable prismatic joint motor.
B2W_EXPORT void b2w_setPrismaticMotor(int64_t jointHandle,
                            float speed, float maxForce, int32_t enable);

/// Set distance joint length range.
B2W_EXPORT void b2w_setDistanceLimits(int64_t jointHandle, float minLen, float maxLen);

/// Set distance joint spring (stiffness + damping, 0 = rigid).
B2W_EXPORT void b2w_setDistanceSpring(int64_t jointHandle,
                            float stiffness, float damping);

/// Update the mouse joint target position.
B2W_EXPORT void b2w_setMouseJointTarget(int64_t jointHandle, float x, float y);

/// Set wheel joint spring stiffness and damping.
B2W_EXPORT void b2w_setWheelSpring(int64_t jointHandle,
                         float stiffness, float damping);

/// Enable/disable wheel joint motor.
B2W_EXPORT void b2w_setWheelMotor(int64_t jointHandle,
                        float speed, float maxTorque, int32_t enable);

// ── Joint queries ─────────────────────────────────────────────────────────────

/// Get the reaction force on joint bodyA (world-space, per second).
B2W_EXPORT void b2w_getJointReactionForce(int64_t jointHandle,
                                float* outFx, float* outFy);

/// Get the reaction torque on joint bodyA (per second).
B2W_EXPORT float b2w_getJointReactionTorque(int64_t jointHandle);

// ── NativeFinalizer-compatible destructors ────────────────────────────────────
// ── Platformer parity extensions ─────────────────────────────────────────────
//
// Everything below closes a gap where the native backend silently behaved
// differently from the pure-Dart one. Added together because a 2D platformer
// needs all of them; see CHANGELOG for the per-feature rationale.

/// Body type values — numerically identical to b2BodyType, so Dart can pass
/// an enum index straight through with no translation table.
#define B2W_BODY_STATIC    0
#define B2W_BODY_KINEMATIC 1
#define B2W_BODY_DYNAMIC   2

/// Create a body of any type. b2w_createDynamicBody / b2w_createStaticBody
/// remain as thin forwarders so existing bindings keep working unchanged.
B2W_EXPORT int64_t b2w_createBody(int64_t worldHandle, int32_t bodyType,
                                  float posX, float posY, float angle);

/// Teleport a body to a world transform.
///
/// Box2D documents SetTransform as "fairly expensive" — it rebuilds broad-phase
/// proxies and discards the solver's warm-start for this body. Call it for
/// respawns, checkpoints, level loads and warps; drive ordinary motion with
/// velocity instead.
///
/// wake != 0 also forces the body awake. b2Body_SetTransform on its own does
/// NOT wake it, so a teleported sleeping body would otherwise sit inert at the
/// new location until something else disturbed it.
B2W_EXPORT void b2w_setBodyTransform(int64_t bodyHandle,
                                     float x, float y, float angle,
                                     int32_t wake);

/// Set angular velocity directly (companion to b2w_setLinearVelocity).
B2W_EXPORT void b2w_setAngularVelocity(int64_t bodyHandle, float omega);

/// Change a body's type at runtime.
///
/// EXPENSIVE, and it RESETS the mass override — re-apply b2w_setBodyMass after
/// any real change. Callers should skip the call entirely when the type
/// already matches (see b2w_getBodyType).
B2W_EXPORT void    b2w_setBodyType(int64_t bodyHandle, int32_t bodyType);
B2W_EXPORT int32_t b2w_getBodyType(int64_t bodyHandle);

/// Drive a kinematic body toward a target transform over one timeStep by
/// setting the velocity that gets there.
///
/// Preferred over teleporting moving platforms: it preserves correct contact
/// velocity, so a body standing on the platform is carried along instead of
/// being left behind.
B2W_EXPORT void b2w_setBodyTargetTransform(int64_t bodyHandle,
                                           float x, float y, float angle,
                                           float timeStep, int32_t wake);

// ── One-way platforms ────────────────────────────────────────────────────────
//
// Implemented as a world pre-solve contact filter. Per-shape state is packed
// into the b2Shape userData pointer, which is NEVER dereferenced — it is used
// purely as an integer box. See the thread-safety note in the .cpp.

#define B2W_SHAPE_FLAG_ONE_WAY  0x1u
#define B2W_ONEWAY_DIR_SHIFT    1
#define B2W_ONEWAY_FROM_ABOVE   0
#define B2W_ONEWAY_FROM_BELOW   1
#define B2W_ONEWAY_FROM_LEFT    2
#define B2W_ONEWAY_FROM_RIGHT   3

/// Mark every shape on a body as a one-way / pass-through platform, solid only
/// when approached from solidFromDirection (a B2W_ONEWAY_FROM_* value).
/// enabled == 0 clears the flag and disables pre-solve events on the shapes.
B2W_EXPORT void b2w_setBodyOneWay(int64_t bodyHandle,
                                  int32_t enabled,
                                  int32_t solidFromDirection);

// ── Spatial queries ──────────────────────────────────────────────────────────

/// Cast a ray, returning only the closest hit.
/// translation is the FULL ray vector (direction * maxDistance), not a unit
/// vector. Pass categoryBits = maskBits = ~0ull to hit everything.
/// Returns 1 on hit (out-params written), 0 on miss.
///
/// NOTE: this ignores initial overlap — a ray starting inside a body does not
/// report that body. Use b2w_castRayAll when that matters.
B2W_EXPORT int32_t b2w_castRayClosest(int64_t worldHandle,
                                      float originX, float originY,
                                      float translationX, float translationY,
                                      uint64_t categoryBits, uint64_t maskBits,
                                      int64_t* outBody,
                                      float* outPointX, float* outPointY,
                                      float* outNormalX, float* outNormalY,
                                      float* outFraction);

/// Cast a ray, collecting up to maxHits hits, unsorted and one per SHAPE — a
/// compound body can appear more than once, so de-duplicate on the Dart side.
/// outBodies: caller-allocated int64[maxHits].
/// outBuffer: caller-allocated float[maxHits*5], per hit:
///            [pointX, pointY, normalX, normalY, fraction]
/// Returns the number of hits written.
B2W_EXPORT int32_t b2w_castRayAll(int64_t worldHandle,
                                  float originX, float originY,
                                  float translationX, float translationY,
                                  uint64_t categoryBits, uint64_t maskBits,
                                  int64_t* outBodies, float* outBuffer,
                                  int32_t maxHits);

/// Broad-phase AABB overlap. Reports bodies whose shapes' FAT AABBs overlap the
/// box — a conservative superset. Narrow-test on the Dart side if exactness
/// matters. Returns the number of body handles written to outBodies.
B2W_EXPORT int32_t b2w_queryAABB(int64_t worldHandle,
                                 float minX, float minY, float maxX, float maxY,
                                 uint64_t categoryBits, uint64_t maskBits,
                                 int64_t* outBodies, int32_t maxBodies);

// ── Runtime material / damping mutation ──────────────────────────────────────

/// Velocity decay. Box2D's model is 1/(1 + dt*damping), which is not identical
/// to the pure-Dart engine's (1 - drag*dt) but behaves equivalently.
B2W_EXPORT void b2w_setBodyLinearDamping(int64_t bodyHandle, float damping);
B2W_EXPORT void b2w_setBodyAngularDamping(int64_t bodyHandle, float damping);

/// Set friction / restitution on every shape of a body. Takes effect on the
/// next step — Box2D re-reads shape material each step. Use for ice, mud, and
/// bounce pads that change state.
B2W_EXPORT void b2w_setBodyFriction(int64_t bodyHandle, float friction);
B2W_EXPORT void b2w_setBodyRestitution(int64_t bodyHandle, float restitution);

/// 64-bit collision filter. b2Filter.categoryBits/maskBits are uint64_t in this
/// Box2D; the 32-bit b2w_setBodyFilter above is kept unchanged so existing
/// bindings stay ABI-correct.
B2W_EXPORT void b2w_setBodyFilter64(int64_t bodyHandle,
                                    uint64_t categoryBits, uint64_t maskBits,
                                    int32_t groupIndex);

//
// NativeFinalizer requires a native function with signature void(void*).
// These wrappers receive the packed int64 handle reinterpreted as a void*
// token (set via Pointer<Void>.fromAddress(handle) on the Dart side), cast
// it back to int64_t, and call the real destroy function.
// Only used as safety-net GC finalizers — prefer explicit dispose().

B2W_EXPORT void b2w_finalizer_world(void* token);
B2W_EXPORT void b2w_finalizer_body(void* token);

#ifdef __cplusplus
}
#endif
