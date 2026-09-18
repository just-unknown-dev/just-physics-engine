// box2d_wrapper.cpp — C++ implementation of the pure-C API in box2d_wrapper.h.
//
// Adapted for Box2D 3.x API (commit fa173b5+):
//  - b2TaskCallback: void(void* ctx)  — no start/end indices.
//  - b2EnqueueTaskCallback: void*(task, taskCtx, userCtx)  — 3 args.
//  - b2ShapeDef.material.friction/restitution  — nested in b2SurfaceMaterial.
//  - B2_MAX_POLYGON_VERTICES macro  — not b2_maxPolygonVertices constant.
//  - b2Shape_GetBody(shapeId)  — function, not shapeId.bodyId.

#include "box2d_wrapper.h"
#include "thread_pool.h"

#include <box2d/box2d.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

// ── Global state ─────────────────────────────────────────────────────────────

struct WorldState {
    ThreadPool*      pool;
    ImpactCallbackFn impactCallback;
    float            impactThreshold;
    b2ContactEvents  lastContacts;
    b2SensorEvents   lastSensors;
};

// Separate mutex for g_sensorFlags: shape-creation functions read it while
// b2w_setBodySensor writes it. Using a dedicated lock avoids contention with
// g_worldsMutex and prevents the TOCTOU race between the two operations.

static std::unordered_map<int64_t, WorldState> g_worlds;
static std::mutex g_worldsMutex;

// Per-body sensor flag: set via b2w_setBodySensor BEFORE adding shapes so the
// shape creation functions can apply isSensor to b2ShapeDef at creation time.
static std::unordered_map<int64_t, bool> g_sensorFlags;
static std::mutex g_sensorMutex;

static inline bool _isSensorBody(int64_t bh) {
    std::lock_guard<std::mutex> lk(g_sensorMutex);
    auto it = g_sensorFlags.find(bh);
    return it != g_sensorFlags.end() && it->second;
}

// ── ID packing / unpacking ────────────────────────────────────────────────────

static inline int64_t packWorldId(b2WorldId id) {
    return (int64_t)id.index1 | ((int64_t)id.generation << 16);
}
static inline b2WorldId unpackWorldId(int64_t h) {
    b2WorldId id;
    id.index1     = (uint16_t)(h & 0xFFFF);
    id.generation = (uint16_t)((h >> 16) & 0xFFFF);
    return id;
}
static inline int64_t packBodyId(b2BodyId id) {
    return (int64_t)(uint32_t)id.index1
         | ((int64_t)id.world0      << 32)
         | ((int64_t)id.generation  << 48);
}
static inline b2BodyId unpackBodyId(int64_t h) {
    b2BodyId id;
    id.index1     = (int32_t)(h & 0xFFFFFFFF);
    id.world0     = (uint16_t)((h >> 32) & 0xFFFF);
    id.generation = (uint16_t)((h >> 48) & 0xFFFF);
    return id;
}

// ── Static callback stubs (avoids MSVC lambda-to-function-pointer issues) ────

// Called by Box2D to submit a task to our thread pool.
// Returns a TaskGroup* token that Box2D passes back to s_finishTask.
static void* s_enqueueTask(b2TaskCallback* task, void* taskCtx, void* userCtx) {
    ThreadPool* pool = static_cast<ThreadPool*>(userCtx);
    return pool->enqueueTask(task, taskCtx);
}

// Called by Box2D to wait for a previously enqueued task group.
static void s_finishTask(void* userTask, void* userCtx) {
    ThreadPool* pool = static_cast<ThreadPool*>(userCtx);
    pool->finishTask(static_cast<TaskGroup*>(userTask));
}

// ── Platformer parity extensions ─────────────────────────────────────────────

static inline b2BodyType _toBodyType(int32_t t) {
    return (t == B2W_BODY_STATIC)    ? b2_staticBody
         : (t == B2W_BODY_KINEMATIC) ? b2_kinematicBody
                                     : b2_dynamicBody;
}

extern "C" int64_t b2w_createBody(int64_t wh, int32_t bodyType,
                                   float x, float y, float angle) {
    b2BodyDef def = b2DefaultBodyDef();
    def.type      = _toBodyType(bodyType);
    def.position  = b2ToPos({x, y});
    def.rotation  = b2MakeRot(angle);
    return packBodyId(b2CreateBody(unpackWorldId(wh), &def));
}

extern "C" void b2w_setBodyTransform(int64_t bh, float x, float y,
                                      float angle, int32_t wake) {
    b2BodyId id = unpackBodyId(bh);
    b2Body_SetTransform(id, b2ToPos({x, y}), b2MakeRot(angle));
    // SetTransform does not wake the body on its own; a teleported sleeping
    // body would stay inert at its new location.
    if (wake != 0) b2Body_SetAwake(id, true);
}

extern "C" void b2w_setAngularVelocity(int64_t bh, float omega) {
    b2Body_SetAngularVelocity(unpackBodyId(bh), omega);
}

extern "C" void b2w_setBodyType(int64_t bh, int32_t bodyType) {
    b2Body_SetType(unpackBodyId(bh), _toBodyType(bodyType));
}

extern "C" int32_t b2w_getBodyType(int64_t bh) {
    return (int32_t)b2Body_GetType(unpackBodyId(bh));
}

extern "C" void b2w_setBodyTargetTransform(int64_t bh, float x, float y,
                                            float angle, float timeStep,
                                            int32_t wake) {
    b2WorldTransform t;
    t.p = b2ToPos({x, y});
    t.q = b2MakeRot(angle);
    b2Body_SetTargetTransform(unpackBodyId(bh), t, timeStep, wake != 0);
}

// ── One-way platforms ────────────────────────────────────────────────────────
//
// THREAD SAFETY — read this before touching s_preSolveOneWay.
//
// The callback runs on Box2D WORKER THREADS, once per touching contact per
// step, from inside b2UpdateContact. It must therefore be a pure function of
// its arguments plus immutable per-shape state. Specifically it must NEVER:
//   * take a lock — a global mutex here serialises the entire narrow phase,
//     turning the parallel collide stage back into a single-threaded one;
//   * read body velocity — b2Body_GetLinearVelocity reads b2BodyState, which
//     solver threads write during the same step. That is a data race, and it
//     is tempting precisely because it would match the pure-Dart engine's
//     velocity test exactly;
//   * mutate the world in any way (Box2D documents this explicitly);
//   * allocate, or call back into Dart.
//
// Per-shape state therefore lives in the shape's userData pointer, used purely
// as an integer box and never dereferenced. That is safe because there is
// exactly one writer and it can never run concurrently with a reader: every
// b2w_* entry point is a synchronous FFI call from the Dart main isolate, and
// b2w_step blocks that isolate until b2World_Step has joined every worker
// task. So b2w_setBodyOneWay can only run BETWEEN steps, and the thread pool's
// mutex/condvar wake-up provides the release/acquire edge that publishes the
// write to the workers.

/// Cosine threshold, ~60 degrees of tolerance. Kept identical to the pure-Dart
/// engine's _oneWaySolidCos so both backends agree.
///
/// Tighter values drop a player who lands on the very edge of a platform;
/// looser values let a player rising steeply from below catch on it.
static constexpr float kOneWaySolidCos = 0.5f;

static bool s_preSolveOneWay(b2ShapeId a, b2ShapeId b,
                              b2Pos point, b2Vec2 normal, void* ctx) {
    (void)point;
    (void)ctx;

    const uint32_t fa = (uint32_t)(uintptr_t)b2Shape_GetUserData(a);
    const uint32_t fb = (uint32_t)(uintptr_t)b2Shape_GetUserData(b);
    const bool oneA = (fa & B2W_SHAPE_FLAG_ONE_WAY) != 0;
    const bool oneB = (fb & B2W_SHAPE_FLAG_ONE_WAY) != 0;

    // Neither is one-way, or both are: an ordinary solid contact. Two one-way
    // bodies have no claim to pass through each other.
    if (oneA == oneB) return true;

    // The manifold normal points from shape A to shape B. Flip it so it always
    // reads platform -> mover.
    const uint32_t flags = oneA ? fa : fb;
    const float    sign  = oneA ? 1.0f : -1.0f;
    const float    px    = sign * normal.x;
    const float    py    = sign * normal.y;

    // Screen-space convention: +Y is DOWN (default gravity is +981), so the
    // mover being *above* the platform means py < 0.
    switch ((flags >> B2W_ONEWAY_DIR_SHIFT) & 0x3u) {
        case B2W_ONEWAY_FROM_ABOVE: return py < -kOneWaySolidCos;
        case B2W_ONEWAY_FROM_BELOW: return py >  kOneWaySolidCos;
        case B2W_ONEWAY_FROM_LEFT:  return px < -kOneWaySolidCos;
        default:                    return px >  kOneWaySolidCos;
    }
}

extern "C" void b2w_setBodyOneWay(int64_t bh, int32_t enabled, int32_t dir) {
    const bool on = enabled != 0;
    const uint32_t flags =
        on ? (B2W_SHAPE_FLAG_ONE_WAY |
              ((uint32_t)(dir & 0x3) << B2W_ONEWAY_DIR_SHIFT))
           : 0u;

    b2BodyId bodyId = unpackBodyId(bh);
    static constexpr int kMaxShapes = 64;
    b2ShapeId shapes[kMaxShapes];
    int count = b2Body_GetShapes(bodyId, shapes, kMaxShapes);
    assert(count < kMaxShapes && "body has >= 64 shapes — raise kMaxShapes");
    for (int i = 0; i < count; ++i) {
        b2Shape_SetUserData(shapes[i], (void*)(uintptr_t)flags);
        // Enable pre-solve ONLY on one-way shapes. Box2D flags a contact for
        // pre-solve if EITHER shape opts in, so this is sufficient — and it
        // keeps the callback off every other contact in the world.
        b2Shape_EnablePreSolveEvents(shapes[i], on);
    }
}

// ── Spatial queries ──────────────────────────────────────────────────────────
//
// Unlike the pre-solve callback, these all run on the Dart main isolate
// between steps, single-threaded, so a plain stack-local context is safe.

extern "C" int32_t b2w_castRayClosest(int64_t wh, float ox, float oy,
                                       float tx, float ty,
                                       uint64_t cat, uint64_t mask,
                                       int64_t* outBody,
                                       float* opx, float* opy,
                                       float* onx, float* ony,
                                       float* ofrac) {
    b2QueryFilter filter;
    filter.categoryBits = cat;
    filter.maskBits     = mask;

    b2RayResult r = b2World_CastRayClosest(unpackWorldId(wh),
                                           b2ToPos({ox, oy}), {tx, ty},
                                           filter);
    if (!r.hit) return 0;

    b2Vec2 p = b2ToVec2(r.point);
    *outBody = packBodyId(b2Shape_GetBody(r.shapeId));
    *opx = p.x;
    *opy = p.y;
    *onx = r.normal.x;
    *ony = r.normal.y;
    *ofrac = r.fraction;
    return 1;
}

struct RayAllCtx {
    int64_t* bodies;
    float*   buf;
    int32_t  max;
    int32_t  count;
};

static float s_rayAllCallback(b2ShapeId shapeId, b2Pos point,
                               b2Vec2 normal, float fraction, void* ctx) {
    RayAllCtx* c = static_cast<RayAllCtx*>(ctx);
    if (c->count >= c->max) return 0.0f;  // budget spent — terminate the cast

    b2Vec2 p = b2ToVec2(point);
    float* slot = c->buf + (ptrdiff_t)c->count * 5;
    slot[0] = p.x;
    slot[1] = p.y;
    slot[2] = normal.x;
    slot[3] = normal.y;
    slot[4] = fraction;
    c->bodies[c->count++] = packBodyId(b2Shape_GetBody(shapeId));

    return 1.0f;  // 1 = do not clip the ray, keep collecting
}

extern "C" int32_t b2w_castRayAll(int64_t wh, float ox, float oy,
                                   float tx, float ty,
                                   uint64_t cat, uint64_t mask,
                                   int64_t* outBodies, float* outBuffer,
                                   int32_t maxHits) {
    b2QueryFilter filter;
    filter.categoryBits = cat;
    filter.maskBits     = mask;

    RayAllCtx c{outBodies, outBuffer, maxHits, 0};
    b2World_CastRay(unpackWorldId(wh), b2ToPos({ox, oy}), {tx, ty},
                    filter, s_rayAllCallback, &c);
    return c.count;
}

struct QueryCtx {
    int64_t* bodies;
    int32_t  max;
    int32_t  count;
};

static bool s_overlapCallback(b2ShapeId shapeId, void* ctx) {
    QueryCtx* c = static_cast<QueryCtx*>(ctx);
    if (c->count >= c->max) return false;  // stop the query

    const int64_t bh = packBodyId(b2Shape_GetBody(shapeId));
    // One body can own several shapes; report each body once.
    for (int32_t i = 0; i < c->count; ++i) {
        if (c->bodies[i] == bh) return true;
    }
    c->bodies[c->count++] = bh;
    return true;
}

extern "C" int32_t b2w_queryAABB(int64_t wh,
                                  float minX, float minY,
                                  float maxX, float maxY,
                                  uint64_t cat, uint64_t mask,
                                  int64_t* outBodies, int32_t maxBodies) {
    b2QueryFilter filter;
    filter.categoryBits = cat;
    filter.maskBits     = mask;

    b2AABB aabb;
    aabb.lowerBound = {minX, minY};
    aabb.upperBound = {maxX, maxY};

    QueryCtx c{outBodies, maxBodies, 0};
    // Box2D's own docs: near the origin, pass b2Pos_zero and a world AABB.
    b2World_OverlapAABB(unpackWorldId(wh), b2Pos_zero, aabb, filter,
                        s_overlapCallback, &c);
    return c.count;
}

// ── Runtime material / damping mutation ──────────────────────────────────────

extern "C" void b2w_setBodyLinearDamping(int64_t bh, float damping) {
    b2Body_SetLinearDamping(unpackBodyId(bh), damping);
}

extern "C" void b2w_setBodyAngularDamping(int64_t bh, float damping) {
    b2Body_SetAngularDamping(unpackBodyId(bh), damping);
}

extern "C" void b2w_setBodyFriction(int64_t bh, float friction) {
    b2BodyId bodyId = unpackBodyId(bh);
    static constexpr int kMaxShapes = 64;
    b2ShapeId shapes[kMaxShapes];
    int count = b2Body_GetShapes(bodyId, shapes, kMaxShapes);
    assert(count < kMaxShapes && "body has >= 64 shapes — raise kMaxShapes");
    for (int i = 0; i < count; ++i) {
        b2Shape_SetFriction(shapes[i], friction);
    }
}

extern "C" void b2w_setBodyRestitution(int64_t bh, float restitution) {
    b2BodyId bodyId = unpackBodyId(bh);
    static constexpr int kMaxShapes = 64;
    b2ShapeId shapes[kMaxShapes];
    int count = b2Body_GetShapes(bodyId, shapes, kMaxShapes);
    assert(count < kMaxShapes && "body has >= 64 shapes — raise kMaxShapes");
    for (int i = 0; i < count; ++i) {
        b2Shape_SetRestitution(shapes[i], restitution);
    }
}

extern "C" void b2w_setBodyFilter64(int64_t bh,
                                     uint64_t categoryBits, uint64_t maskBits,
                                     int32_t groupIndex) {
    b2BodyId bodyId = unpackBodyId(bh);
    static constexpr int kMaxShapes = 64;
    b2ShapeId shapes[kMaxShapes];
    int count = b2Body_GetShapes(bodyId, shapes, kMaxShapes);
    assert(count < kMaxShapes && "body has >= 64 shapes — raise kMaxShapes");
    b2Filter filter;
    filter.categoryBits = categoryBits;
    filter.maskBits     = maskBits;
    filter.groupIndex   = groupIndex;
    for (int i = 0; i < count; ++i) {
        b2Shape_SetFilter(shapes[i], filter);
    }
}

// ── NativeFinalizer-compatible destructors ────────────────────────────────────

extern "C" void b2w_finalizer_world(void* token) {
    int64_t h = (int64_t)(intptr_t)token;
    b2w_destroyWorld(h);
}
extern "C" void b2w_finalizer_body(void* token) {
    int64_t h = (int64_t)(intptr_t)token;
    b2w_destroyBody(h);
}

// ── World management ──────────────────────────────────────────────────────────

// This package's gameplay units are centimetres (see box2d_wrapper.h header
// comment and Box2DPhysicsEngine's default gravityY = 981, i.e. 9.81 m/s² at
// 1 unit = 1 cm), but Box2D's tuning constants — b2DefaultWorldDef's
// maximumLinearSpeed ("400 m/s, faster than the speed of sound"),
// b2DefaultBodyDef's sleepThreshold, restitutionThreshold, contactSpeed, etc.
// — are all derived from b2GetLengthUnitsPerMeter(), which defaults to 1.0
// (i.e. "1 simulation unit = 1 meter"). Left unset, maximumLinearSpeed
// resolves to 400 *centimetres*/s (4 m/s), a safety clamp meant to be
// effectively unbounded that instead silently caps any reasonably fast body
// (a falling object reaches it in well under a second). Scale it once, up
// front, to match: 1 meter = 100 of our centimetre units.
static std::once_flag g_lengthUnitsOnce;
static void _ensureLengthUnitsConfigured() {
    std::call_once(g_lengthUnitsOnce, [] { b2SetLengthUnitsPerMeter(100.0f); });
}

// Box2D's built-in default restitution mixing is max(restitutionA,
// restitutionB) (see b2RestitutionCallback docs in box2d.h). The pure-Dart
// fallback engine (PhysicsEngine._resolveCollision) mixes with
// min(a.restitution, b.restitution) instead, and that's the behavior
// documented to engine users (see the Bounciness demo's code sample:
// "Resolution uses: min(a.restitution, b.restitution)"). Left on Box2D's
// default, a perfectly-restitutive floor (restitution 1.0, used so "the
// ball governs bounce" under min-mixing) instead forces every contact to
// max(1.0, ball) == 1.0 — every ball bounces identically no matter what
// restitution it was given. Install a matching min-mixing callback so both
// backends agree.
static float _minRestitutionCallback(float restitutionA, uint64_t,
                                      float restitutionB, uint64_t) {
    return restitutionA < restitutionB ? restitutionA : restitutionB;
}

extern "C" int64_t b2w_createWorld(float gx, float gy, int32_t numThreads) {
    _ensureLengthUnitsConfigured();

    if (numThreads <= 0) {
        // hardware_concurrency - 1 spins up one OS thread per core for every
        // world (e.g. 23 on a 24-core machine). Each Box2DWorld gets its own
        // dedicated ThreadPool (see b2w_destroyWorld), so on higher core-count
        // devices this both wastes threads competing with rendering/audio for
        // one physics world and, under rapid create/destroy churn (tests,
        // hot-reload), risks exhausting OS thread/handle limits. Cap the
        // auto-detected default; callers who want more can still pass an
        // explicit numThreads.
        unsigned hw = std::thread::hardware_concurrency();
        unsigned auto_ = hw > 1 ? hw - 1 : 1;
        constexpr unsigned kMaxAutoThreads = 4;
        numThreads = (int32_t)(auto_ < kMaxAutoThreads ? auto_ : kMaxAutoThreads);
    }

    ThreadPool* pool = new ThreadPool(numThreads);

    b2WorldDef def          = b2DefaultWorldDef();
    def.gravity             = {gx, gy};
    def.workerCount         = pool->workerCount();
    def.userTaskContext     = pool;
    def.enqueueTask         = s_enqueueTask;
    def.finishTask          = s_finishTask;
    def.restitutionCallback = _minRestitutionCallback;

    b2WorldId id = b2CreateWorld(&def);
    int64_t   h  = packWorldId(id);

    // Installed unconditionally at creation rather than lazily on the first
    // b2w_setBodyOneWay call: registration is just a pointer store, and doing
    // it here removes any window where a step could race it. The nullptr
    // context keeps the callback world-agnostic, so there is nothing to tear
    // down and nothing to get wrong with multiple worlds.
    //
    // This costs nothing when no one-way bodies exist — Box2D only invokes the
    // callback for contacts whose shapes opted in via
    // b2Shape_EnablePreSolveEvents.
    b2World_SetPreSolveCallback(id, s_preSolveOneWay, nullptr);

    {
        std::lock_guard<std::mutex> lk(g_worldsMutex);
        g_worlds[h] = {pool, nullptr, 0.0f, {}};
    }
    return h;
}

extern "C" void b2w_destroyWorld(int64_t h) {
    ThreadPool* pool = nullptr;
    {
        std::lock_guard<std::mutex> lk(g_worldsMutex);
        auto it = g_worlds.find(h);
        if (it == g_worlds.end()) return;
        pool = it->second.pool;
        g_worlds.erase(it);
    }
    // Destroy world first — it may call finishTask during teardown.
    b2DestroyWorld(unpackWorldId(h));
    delete pool;
}

extern "C" void b2w_step(int64_t h, float timeStep, int32_t subSteps) {
    b2WorldId wid = unpackWorldId(h);
    b2World_Step(wid, timeStep, subSteps);

    std::lock_guard<std::mutex> lk(g_worldsMutex);
    auto it = g_worlds.find(h);
    if (it == g_worlds.end()) return;

    it->second.lastContacts = b2World_GetContactEvents(wid);
    it->second.lastSensors  = b2World_GetSensorEvents(wid);

    // Fire impact callback for high-velocity begin-touch events.
    if (it->second.impactCallback) {
        const b2ContactEvents& contacts = it->second.lastContacts;
        float            threshold = it->second.impactThreshold;
        ImpactCallbackFn cb        = it->second.impactCallback;

        for (int32_t i = 0; i < contacts.beginCount; ++i) {
            const b2ContactBeginTouchEvent& ev = contacts.beginEvents[i];
            b2BodyId bodyA = b2Shape_GetBody(ev.shapeIdA);
            b2BodyId bodyB = b2Shape_GetBody(ev.shapeIdB);
            b2Vec2   vA    = b2Body_GetLinearVelocity(bodyA);
            b2Vec2   vB    = b2Body_GetLinearVelocity(bodyB);
            float dvx  = vA.x - vB.x;
            float dvy  = vA.y - vB.y;
            float speed = std::sqrt(dvx * dvx + dvy * dvy);
            if (speed >= threshold) {
                cb(packBodyId(bodyA), packBodyId(bodyB), speed);
            }
        }
    }
}

extern "C" void b2w_setGravity(int64_t h, float gx, float gy) {
    b2World_SetGravity(unpackWorldId(h), {gx, gy});
}

// ── Body management ───────────────────────────────────────────────────────────

extern "C" int64_t b2w_createDynamicBody(int64_t wh,
                                          float x, float y, float angle) {
    return b2w_createBody(wh, B2W_BODY_DYNAMIC, x, y, angle);
}

extern "C" int64_t b2w_createStaticBody(int64_t wh,
                                         float x, float y, float angle) {
    return b2w_createBody(wh, B2W_BODY_STATIC, x, y, angle);
}

extern "C" void b2w_destroyBody(int64_t bh) {
    b2DestroyBody(unpackBodyId(bh));
}

// ── Shapes ────────────────────────────────────────────────────────────────────

extern "C" void b2w_addCircleShape(int64_t bh, float radius,
                                    float density, float friction,
                                    float restitution) {
    bool sensor = _isSensorBody(bh);
    b2ShapeDef sd        = b2DefaultShapeDef();
    // b2DefaultShapeDef zero-initialises, so enableContactEvents is FALSE and
    // Box2D reports no begin/end touch events at all for the shape. That made
    // PhysicsEngine.pollContactBeginEvents permanently silent on this backend
    // while the pure-Dart one generated events normally — so anything built on
    // contacts (PhysicsBodyComponent.isGrounded, and therefore every jump)
    // worked on web and silently did nothing on desktop and mobile.
    sd.enableContactEvents = true;
    sd.density           = density;
    sd.material.friction    = friction;
    sd.material.restitution = restitution;
    sd.isSensor             = sensor;
    sd.enableSensorEvents   = true; // must be true on BOTH sides for sensor events to fire

    b2Circle circle = {{0.0f, 0.0f}, radius};
    b2CreateCircleShape(unpackBodyId(bh), &sd, &circle);
}

extern "C" void b2w_addBoxShape(int64_t bh, float halfW, float halfH,
                                 float density, float friction,
                                 float restitution) {
    bool sensor = _isSensorBody(bh);
    b2ShapeDef sd        = b2DefaultShapeDef();
    // b2DefaultShapeDef zero-initialises, so enableContactEvents is FALSE and
    // Box2D reports no begin/end touch events at all for the shape. That made
    // PhysicsEngine.pollContactBeginEvents permanently silent on this backend
    // while the pure-Dart one generated events normally — so anything built on
    // contacts (PhysicsBodyComponent.isGrounded, and therefore every jump)
    // worked on web and silently did nothing on desktop and mobile.
    sd.enableContactEvents = true;
    sd.density           = density;
    sd.material.friction    = friction;
    sd.material.restitution = restitution;
    sd.isSensor             = sensor;
    sd.enableSensorEvents   = true;

    b2Polygon box = b2MakeBox(halfW, halfH);
    b2CreatePolygonShape(unpackBodyId(bh), &sd, &box);
}

extern "C" void b2w_addPolygonShape(int64_t bh,
                                     const float* verts, int32_t count,
                                     float density, float friction,
                                     float restitution) {
    bool sensor = _isSensorBody(bh);
    b2ShapeDef sd        = b2DefaultShapeDef();
    // b2DefaultShapeDef zero-initialises, so enableContactEvents is FALSE and
    // Box2D reports no begin/end touch events at all for the shape. That made
    // PhysicsEngine.pollContactBeginEvents permanently silent on this backend
    // while the pure-Dart one generated events normally — so anything built on
    // contacts (PhysicsBodyComponent.isGrounded, and therefore every jump)
    // worked on web and silently did nothing on desktop and mobile.
    sd.enableContactEvents = true;
    sd.density           = density;
    sd.material.friction    = friction;
    sd.material.restitution = restitution;
    sd.isSensor             = sensor;
    sd.enableSensorEvents   = true;

    b2Vec2 pts[B2_MAX_POLYGON_VERTICES];
    int32_t n = count < B2_MAX_POLYGON_VERTICES ? count : B2_MAX_POLYGON_VERTICES;
    for (int32_t i = 0; i < n; ++i) {
        pts[i] = {verts[i * 2], verts[i * 2 + 1]};
    }

    b2Hull    hull = b2ComputeHull(pts, n);
    b2Polygon poly = b2MakePolygon(&hull, 0.0f);
    b2CreatePolygonShape(unpackBodyId(bh), &sd, &poly);
}

extern "C" void b2w_addRoundedPolygonShape(int64_t bh,
                                            const float* verts, int32_t count,
                                            float cornerRadius,
                                            float density, float friction,
                                            float restitution) {
    bool sensor = _isSensorBody(bh);
    b2ShapeDef sd        = b2DefaultShapeDef();
    // b2DefaultShapeDef zero-initialises, so enableContactEvents is FALSE and
    // Box2D reports no begin/end touch events at all for the shape. That made
    // PhysicsEngine.pollContactBeginEvents permanently silent on this backend
    // while the pure-Dart one generated events normally — so anything built on
    // contacts (PhysicsBodyComponent.isGrounded, and therefore every jump)
    // worked on web and silently did nothing on desktop and mobile.
    sd.enableContactEvents = true;
    sd.density           = density;
    sd.material.friction    = friction;
    sd.material.restitution = restitution;
    sd.isSensor             = sensor;
    sd.enableSensorEvents   = true;

    b2Vec2 pts[B2_MAX_POLYGON_VERTICES];
    int32_t n = count < B2_MAX_POLYGON_VERTICES ? count : B2_MAX_POLYGON_VERTICES;
    for (int32_t i = 0; i < n; ++i) {
        pts[i] = {verts[i * 2], verts[i * 2 + 1]};
    }

    b2Hull    hull = b2ComputeHull(pts, n);
    b2Polygon poly = b2MakePolygon(&hull, cornerRadius);
    b2CreatePolygonShape(unpackBodyId(bh), &sd, &poly);
}

extern "C" void b2w_addCapsuleShape(int64_t bh,
                                     float cx1, float cy1, float cx2, float cy2,
                                     float radius,
                                     float density, float friction,
                                     float restitution) {
    bool sensor = _isSensorBody(bh);
    b2ShapeDef sd        = b2DefaultShapeDef();
    // b2DefaultShapeDef zero-initialises, so enableContactEvents is FALSE and
    // Box2D reports no begin/end touch events at all for the shape. That made
    // PhysicsEngine.pollContactBeginEvents permanently silent on this backend
    // while the pure-Dart one generated events normally — so anything built on
    // contacts (PhysicsBodyComponent.isGrounded, and therefore every jump)
    // worked on web and silently did nothing on desktop and mobile.
    sd.enableContactEvents = true;
    sd.density           = density;
    sd.material.friction    = friction;
    sd.material.restitution = restitution;
    sd.isSensor             = sensor;
    sd.enableSensorEvents   = true;

    b2Capsule capsule;
    capsule.center1 = {cx1, cy1};
    capsule.center2 = {cx2, cy2};
    capsule.radius  = radius;
    b2CreateCapsuleShape(unpackBodyId(bh), &sd, &capsule);
}

extern "C" void b2w_addSegmentShape(int64_t bh,
                                     float x1, float y1, float x2, float y2,
                                     float density, float friction,
                                     float restitution) {
    bool sensor = _isSensorBody(bh);
    b2ShapeDef sd        = b2DefaultShapeDef();
    // b2DefaultShapeDef zero-initialises, so enableContactEvents is FALSE and
    // Box2D reports no begin/end touch events at all for the shape. That made
    // PhysicsEngine.pollContactBeginEvents permanently silent on this backend
    // while the pure-Dart one generated events normally — so anything built on
    // contacts (PhysicsBodyComponent.isGrounded, and therefore every jump)
    // worked on web and silently did nothing on desktop and mobile.
    sd.enableContactEvents = true;
    sd.density           = density;
    sd.material.friction    = friction;
    sd.material.restitution = restitution;
    sd.isSensor             = sensor;
    sd.enableSensorEvents   = true;

    b2Segment seg;
    seg.point1 = {x1, y1};
    seg.point2 = {x2, y2};
    b2CreateSegmentShape(unpackBodyId(bh), &sd, &seg);
}

// ── Chain shape ID packing ────────────────────────────────────────────────────

static inline int64_t packChainId(b2ChainId id) {
    return (int64_t)(uint32_t)id.index1
         | ((int64_t)id.world0     << 32)
         | ((int64_t)id.generation << 48);
}
static inline b2ChainId unpackChainId(int64_t h) {
    b2ChainId id;
    id.index1     = (int32_t)(h & 0xFFFFFFFF);
    id.world0     = (uint16_t)((h >> 32) & 0xFFFF);
    id.generation = (uint16_t)((h >> 48) & 0xFFFF);
    return id;
}

extern "C" int64_t b2w_addChainShape(int64_t bh,
                                      const float* points, int32_t count,
                                      int32_t loop,
                                      float friction, float restitution) {
    b2SurfaceMaterial mat = b2DefaultSurfaceMaterial();
    mat.friction    = friction;
    mat.restitution = restitution;

    b2ChainDef cd      = b2DefaultChainDef();
    cd.isLoop          = (loop != 0);
    cd.materials       = &mat;
    cd.materialCount   = 1;

    auto pts = std::make_unique<b2Vec2[]>(count);
    for (int32_t i = 0; i < count; ++i) {
        pts[i] = {points[i * 2], points[i * 2 + 1]};
    }
    cd.points = pts.get();
    cd.count  = count;

    b2ChainId chainId = b2CreateChain(unpackBodyId(bh), &cd);
    return packChainId(chainId);
}

extern "C" void b2w_destroyChain(int64_t chainHandle) {
    b2DestroyChain(unpackChainId(chainHandle));
}

// ── Forces ────────────────────────────────────────────────────────────────────

extern "C" void b2w_applyForce(int64_t bh, float fx, float fy) {
    b2Body_ApplyForceToCenter(unpackBodyId(bh), {fx, fy}, true);
}

extern "C" void b2w_applyLinearImpulse(int64_t bh, float ix, float iy) {
    b2BodyId id = unpackBodyId(bh);
    b2Vec2 point = b2Body_GetPosition(id);
    b2Body_ApplyLinearImpulse(id, {ix, iy}, point, true);
}

extern "C" void b2w_applyTorque(int64_t bh, float torque) {
    b2Body_ApplyTorque(unpackBodyId(bh), torque, true);
}

extern "C" void b2w_setLinearVelocity(int64_t bh, float vx, float vy) {
    b2Body_SetLinearVelocity(unpackBodyId(bh), {vx, vy});
}

// ── Zero-copy bulk transform extraction ──────────────────────────────────────

extern "C" void b2w_bulkExtractTransforms(const int64_t* handles,
                                           float* buffer, int32_t count) {
    for (int32_t i = 0; i < count; ++i) {
        b2BodyId    bodyId = unpackBodyId(handles[i]);
        b2Transform t      = b2Body_GetTransform(bodyId);
        b2Vec2      v      = b2Body_GetLinearVelocity(bodyId);
        float*      slot   = buffer + (ptrdiff_t)i * 6;
        slot[0] = t.p.x;
        slot[1] = t.p.y;
        slot[2] = b2Rot_GetAngle(t.q);
        slot[3] = v.x;
        slot[4] = v.y;
        slot[5] = b2Body_IsAwake(bodyId) ? 1.0f : 0.0f;
    }
}

// ── Contact events ────────────────────────────────────────────────────────────

extern "C" int32_t b2w_getContactBeginCount(int64_t wh) {
    std::lock_guard<std::mutex> lk(g_worldsMutex);
    auto it = g_worlds.find(wh);
    return (it != g_worlds.end()) ? it->second.lastContacts.beginCount : 0;
}

extern "C" void b2w_getContactBeginEvent(int64_t wh, int32_t index,
                                          int64_t* outBodyA, int64_t* outBodyB,
                                          float* outNx, float* outNy) {
    std::lock_guard<std::mutex> lk(g_worldsMutex);
    auto it = g_worlds.find(wh);
    if (it == g_worlds.end()) return;

    const b2ContactBeginTouchEvent& ev = it->second.lastContacts.beginEvents[index];
    *outBodyA = packBodyId(b2Shape_GetBody(ev.shapeIdA));
    *outBodyB = packBodyId(b2Shape_GetBody(ev.shapeIdB));

    // Manifold normal points from shapeA to shapeB (world space). The contact
    // may already be gone (e.g. a shape was destroyed this same step) — guard
    // with b2Contact_IsValid per the box2d.h contract on b2ContactId.
    *outNx = 0.0f;
    *outNy = 0.0f;
    if (b2Contact_IsValid(ev.contactId)) {
        b2ContactData data = b2Contact_GetData(ev.contactId);
        if (data.manifold.pointCount > 0) {
            *outNx = data.manifold.normal.x;
            *outNy = data.manifold.normal.y;
        }
    }
}

extern "C" int32_t b2w_getContactEndCount(int64_t wh) {
    std::lock_guard<std::mutex> lk(g_worldsMutex);
    auto it = g_worlds.find(wh);
    return (it != g_worlds.end()) ? it->second.lastContacts.endCount : 0;
}

extern "C" void b2w_getContactEndEvent(int64_t wh, int32_t index,
                                        int64_t* outBodyA, int64_t* outBodyB) {
    std::lock_guard<std::mutex> lk(g_worldsMutex);
    auto it = g_worlds.find(wh);
    if (it == g_worlds.end()) return;

    const b2ContactEndTouchEvent& ev = it->second.lastContacts.endEvents[index];
    *outBodyA = packBodyId(b2Shape_GetBody(ev.shapeIdA));
    *outBodyB = packBodyId(b2Shape_GetBody(ev.shapeIdB));
}

// ── Sensor events ─────────────────────────────────────────────────────────────

extern "C" int32_t b2w_getSensorBeginCount(int64_t wh) {
    std::lock_guard<std::mutex> lk(g_worldsMutex);
    auto it = g_worlds.find(wh);
    return (it != g_worlds.end()) ? it->second.lastSensors.beginCount : 0;
}

extern "C" void b2w_getSensorBeginEvent(int64_t wh, int32_t index,
                                         int64_t* outSensorBody,
                                         int64_t* outVisitorBody) {
    std::lock_guard<std::mutex> lk(g_worldsMutex);
    auto it = g_worlds.find(wh);
    if (it == g_worlds.end()) return;

    const b2SensorBeginTouchEvent& ev = it->second.lastSensors.beginEvents[index];
    *outSensorBody  = packBodyId(b2Shape_GetBody(ev.sensorShapeId));
    *outVisitorBody = packBodyId(b2Shape_GetBody(ev.visitorShapeId));
}

extern "C" int32_t b2w_getSensorEndCount(int64_t wh) {
    std::lock_guard<std::mutex> lk(g_worldsMutex);
    auto it = g_worlds.find(wh);
    return (it != g_worlds.end()) ? it->second.lastSensors.endCount : 0;
}

extern "C" void b2w_getSensorEndEvent(int64_t wh, int32_t index,
                                       int64_t* outSensorBody,
                                       int64_t* outVisitorBody) {
    std::lock_guard<std::mutex> lk(g_worldsMutex);
    auto it = g_worlds.find(wh);
    if (it == g_worlds.end()) return;

    const b2SensorEndTouchEvent& ev = it->second.lastSensors.endEvents[index];
    *outSensorBody  = packBodyId(b2Shape_GetBody(ev.sensorShapeId));
    *outVisitorBody = packBodyId(b2Shape_GetBody(ev.visitorShapeId));
}

// ── Sensor / filter setters (applied to all shapes on a body) ─────────────────

extern "C" void b2w_setBodySensor(int64_t bh, int32_t isSensor) {
    { std::lock_guard<std::mutex> lk(g_sensorMutex); g_sensorFlags[bh] = (isSensor != 0); }
    // Update sensor events on shapes already attached to this body.
    // enableSensorEvents stays true on all shapes; only isSensor changes here.
    b2BodyId bodyId = unpackBodyId(bh);
    // Capacity 64 covers all practical use; assert fires in debug if a body
    // somehow has more shapes so we can raise the limit rather than silently
    // leaving shapes unprocessed.
    static constexpr int kMaxShapes = 64;
    b2ShapeId shapes[kMaxShapes];
    int count = b2Body_GetShapes(bodyId, shapes, kMaxShapes);
    assert(count < kMaxShapes && "body has >= 64 shapes — raise kMaxShapes");
    for (int i = 0; i < count; ++i) {
        b2Shape_EnableSensorEvents(shapes[i], true);
    }
}

extern "C" void b2w_setBodyFilter(int64_t bh,
                                   uint32_t categoryBits,
                                   uint32_t maskBits,
                                   int32_t  groupIndex) {
    b2BodyId bodyId = unpackBodyId(bh);
    static constexpr int kMaxShapes = 64;
    b2ShapeId shapes[kMaxShapes];
    int count = b2Body_GetShapes(bodyId, shapes, kMaxShapes);
    assert(count < kMaxShapes && "body has >= 64 shapes — raise kMaxShapes");
    b2Filter filter;
    filter.categoryBits = categoryBits;
    filter.maskBits     = maskBits;
    filter.groupIndex   = groupIndex;
    for (int i = 0; i < count; ++i) {
        b2Shape_SetFilter(shapes[i], filter);
    }
}

// ── Bullet / CCD ─────────────────────────────────────────────────────────────

extern "C" void b2w_setBodyBullet(int64_t bh, int32_t isBullet) {
    b2Body_SetBullet(unpackBodyId(bh), isBullet != 0);
}

extern "C" void b2w_setBodyGravityScale(int64_t bh, float gravityScale) {
    b2Body_SetGravityScale(unpackBodyId(bh), gravityScale);
}

extern "C" void b2w_setBodyAwake(int64_t bh, int32_t awake) {
    b2Body_SetAwake(unpackBodyId(bh), awake != 0);
}

extern "C" void b2w_setBodyFixedRotation(int64_t bh, int32_t fixed) {
    // Box2D v3.1 replaced the old bool `fixedRotation` with per-axis
    // b2MotionLocks; only the angular lock is touched here, translation
    // stays free.
    b2BodyId id = unpackBodyId(bh);
    b2MotionLocks locks = b2Body_GetMotionLocks(id);
    locks.angularZ = fixed != 0;
    b2Body_SetMotionLocks(id, locks);
}

extern "C" void b2w_setBodyMass(int64_t bh, float mass) {
    b2BodyId id = unpackBodyId(bh);
    b2MassData data = b2Body_GetMassData(id);
    if (data.mass > 0.0f) {
        // Keep the mass/inertia ratio (and therefore angular response)
        // consistent with the shape-derived values Box2D just computed.
        data.rotationalInertia *= mass / data.mass;
    }
    data.mass = mass;
    b2Body_SetMassData(id, data);
}

// ── Body movement events ──────────────────────────────────────────────────────

extern "C" int32_t b2w_getBodyMoveEventCount(int64_t wh) {
    std::lock_guard<std::mutex> lk(g_worldsMutex);
    auto it = g_worlds.find(wh);
    if (it == g_worlds.end()) return 0;
    b2BodyEvents ev = b2World_GetBodyEvents(unpackWorldId(wh));
    return ev.moveCount;
}

extern "C" void b2w_getBodyMoveEvent(int64_t wh, int32_t index,
                                      int64_t* outBodyHandle,
                                      int32_t* outFellAsleep) {
    b2BodyEvents ev = b2World_GetBodyEvents(unpackWorldId(wh));
    if (index < 0 || index >= ev.moveCount) return;
    const b2BodyMoveEvent& e = ev.moveEvents[index];
    *outBodyHandle = packBodyId(e.bodyId);
    *outFellAsleep = e.fellAsleep ? 1 : 0;
}

// ── Joint ID packing/unpacking ────────────────────────────────────────────────

static inline int64_t packJointId(b2JointId id) {
    return (int64_t)(uint32_t)id.index1
         | ((int64_t)id.world0      << 32)
         | ((int64_t)id.generation  << 48);
}
static inline b2JointId unpackJointId(int64_t h) {
    b2JointId id;
    id.index1     = (int32_t)(h & 0xFFFFFFFF);
    id.world0     = (uint16_t)((h >> 32) & 0xFFFF);
    id.generation = (uint16_t)((h >> 48) & 0xFFFF);
    return id;
}

// ── Joint creation ────────────────────────────────────────────────────────────

extern "C" int64_t b2w_createRevoluteJoint(int64_t wh,
                                             int64_t bA, int64_t bB,
                                             float anchorX, float anchorY) {
    b2WorldId wid   = unpackWorldId(wh);
    b2BodyId  bodyA = unpackBodyId(bA);
    b2BodyId  bodyB = unpackBodyId(bB);
    b2Vec2    world = {anchorX, anchorY};

    b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
    def.base.bodyIdA       = bodyA;
    def.base.bodyIdB       = bodyB;
    def.base.localFrameA.p = b2Body_GetLocalPoint(bodyA, world);
    def.base.localFrameA.q = b2Rot_identity;
    def.base.localFrameB.p = b2Body_GetLocalPoint(bodyB, world);
    def.base.localFrameB.q = b2Rot_identity;
    return packJointId(b2CreateRevoluteJoint(wid, &def));
}

extern "C" int64_t b2w_createPrismaticJoint(int64_t wh,
                                              int64_t bA, int64_t bB,
                                              float anchorX, float anchorY,
                                              float axisX,   float axisY) {
    b2WorldId wid   = unpackWorldId(wh);
    b2BodyId  bodyA = unpackBodyId(bA);
    b2BodyId  bodyB = unpackBodyId(bB);
    b2Vec2    world = {anchorX, anchorY};
    b2Vec2    axis  = {axisX, axisY};

    b2Vec2 localAxis = b2Body_GetLocalVector(bodyA, axis);
    float  angle     = atan2f(localAxis.y, localAxis.x);

    b2PrismaticJointDef def  = b2DefaultPrismaticJointDef();
    def.base.bodyIdA         = bodyA;
    def.base.bodyIdB         = bodyB;
    def.base.localFrameA.p   = b2Body_GetLocalPoint(bodyA, world);
    def.base.localFrameA.q   = b2MakeRot(angle);
    def.base.localFrameB.p   = b2Body_GetLocalPoint(bodyB, world);
    def.base.localFrameB.q   = b2Rot_identity;
    return packJointId(b2CreatePrismaticJoint(wid, &def));
}

extern "C" int64_t b2w_createDistanceJoint(int64_t wh,
                                             int64_t bA, int64_t bB,
                                             float minLen, float maxLen) {
    b2WorldId wid   = unpackWorldId(wh);
    b2BodyId  bodyA = unpackBodyId(bA);
    b2BodyId  bodyB = unpackBodyId(bB);

    b2DistanceJointDef def = b2DefaultDistanceJointDef();
    def.base.bodyIdA       = bodyA;
    def.base.bodyIdB       = bodyB;
    def.base.localFrameA.p = {0.0f, 0.0f};
    def.base.localFrameA.q = b2Rot_identity;
    def.base.localFrameB.p = {0.0f, 0.0f};
    def.base.localFrameB.q = b2Rot_identity;
    def.minLength          = minLen;
    def.maxLength          = maxLen;
    def.length             = (minLen + maxLen) * 0.5f;
    return packJointId(b2CreateDistanceJoint(wid, &def));
}

extern "C" int64_t b2w_createMouseJoint(int64_t wh,
                                          int64_t bB,
                                          float targetX, float targetY) {
    // Mouse joint is not available in this Box2D version.
    // The Dart pure-fallback MouseJoint handles this constraint instead.
    (void)wh; (void)bB; (void)targetX; (void)targetY;
    return 0;
}

extern "C" int64_t b2w_createWeldJoint(int64_t wh,
                                         int64_t bA, int64_t bB,
                                         float anchorX, float anchorY) {
    b2WorldId wid   = unpackWorldId(wh);
    b2BodyId  bodyA = unpackBodyId(bA);
    b2BodyId  bodyB = unpackBodyId(bB);
    b2Vec2    world = {anchorX, anchorY};

    b2WeldJointDef def     = b2DefaultWeldJointDef();
    def.base.bodyIdA       = bodyA;
    def.base.bodyIdB       = bodyB;
    def.base.localFrameA.p = b2Body_GetLocalPoint(bodyA, world);
    def.base.localFrameA.q = b2Rot_identity;
    def.base.localFrameB.p = b2Body_GetLocalPoint(bodyB, world);
    def.base.localFrameB.q = b2Rot_identity;
    return packJointId(b2CreateWeldJoint(wid, &def));
}

extern "C" int64_t b2w_createWheelJoint(int64_t wh,
                                          int64_t bA, int64_t bB,
                                          float anchorX, float anchorY,
                                          float axisX,   float axisY) {
    b2WorldId wid   = unpackWorldId(wh);
    b2BodyId  bodyA = unpackBodyId(bA);
    b2BodyId  bodyB = unpackBodyId(bB);
    b2Vec2    world = {anchorX, anchorY};
    b2Vec2    axis  = {axisX, axisY};

    b2Vec2 localAxis = b2Body_GetLocalVector(bodyA, axis);
    float  angle     = atan2f(localAxis.y, localAxis.x);

    b2WheelJointDef def    = b2DefaultWheelJointDef();
    def.base.bodyIdA       = bodyA;
    def.base.bodyIdB       = bodyB;
    def.base.localFrameA.p = b2Body_GetLocalPoint(bodyA, world);
    def.base.localFrameA.q = b2MakeRot(angle);
    def.base.localFrameB.p = b2Body_GetLocalPoint(bodyB, world);
    def.base.localFrameB.q = b2Rot_identity;
    def.hertz              = 4.0f;
    def.dampingRatio       = 0.7f;
    return packJointId(b2CreateWheelJoint(wid, &def));
}

extern "C" void b2w_destroyJoint(int64_t jh) {
    b2DestroyJoint(unpackJointId(jh), true);
}

// ── Joint configuration ───────────────────────────────────────────────────────

// Box2D's joint motor setters (b2*Joint_EnableMotor/SetMotor*) never wake the
// jointed bodies themselves — they just write into the joint's solver state.
// A body that's fallen asleep (Box2D's default ~0.5s time-to-sleep after
// settling, e.g. a car at rest on its suspension) has its island skipped by
// the solver entirely, so an asleep body silently never sees the motor
// torque: SetMotorSpeed/EnableMotor "work" but produce no motion until
// something else disturbs the body. Call this whenever a motor is (re-)
// enabled so driving input reliably wakes a resting body.
static void _wakeJointBodies(b2JointId id) {
    b2Body_SetAwake(b2Joint_GetBodyA(id), true);
    b2Body_SetAwake(b2Joint_GetBodyB(id), true);
}

extern "C" void b2w_setRevoluteLimits(int64_t jh,
                                       float lower, float upper, int32_t enable) {
    b2JointId id = unpackJointId(jh);
    b2RevoluteJoint_SetLimits(id, lower, upper);
    b2RevoluteJoint_EnableLimit(id, enable != 0);
}

extern "C" void b2w_setRevoluteMotor(int64_t jh,
                                      float speed, float maxTorque, int32_t enable) {
    b2JointId id = unpackJointId(jh);
    b2RevoluteJoint_SetMotorSpeed(id, speed);
    b2RevoluteJoint_SetMaxMotorTorque(id, maxTorque);
    b2RevoluteJoint_EnableMotor(id, enable != 0);
    if (enable != 0) _wakeJointBodies(id);
}

extern "C" void b2w_setPrismaticLimits(int64_t jh,
                                        float lower, float upper, int32_t enable) {
    b2JointId id = unpackJointId(jh);
    b2PrismaticJoint_SetLimits(id, lower, upper);
    b2PrismaticJoint_EnableLimit(id, enable != 0);
}

extern "C" void b2w_setPrismaticMotor(int64_t jh,
                                       float speed, float maxForce, int32_t enable) {
    b2JointId id = unpackJointId(jh);
    b2PrismaticJoint_SetMotorSpeed(id, speed);
    b2PrismaticJoint_SetMaxMotorForce(id, maxForce);
    b2PrismaticJoint_EnableMotor(id, enable != 0);
    if (enable != 0) _wakeJointBodies(id);
}

extern "C" void b2w_setDistanceLimits(int64_t jh, float minLen, float maxLen) {
    b2DistanceJoint_SetLengthRange(unpackJointId(jh), minLen, maxLen);
}

extern "C" void b2w_setDistanceSpring(int64_t jh,
                                       float stiffness, float damping) {
    b2JointId id = unpackJointId(jh);
    b2DistanceJoint_SetSpringHertz(id, stiffness);
    b2DistanceJoint_SetSpringDampingRatio(id, damping);
}

extern "C" void b2w_setMouseJointTarget(int64_t jh, float x, float y) {
    // Mouse joint not available in this Box2D version — no-op.
    (void)jh; (void)x; (void)y;
}

extern "C" void b2w_setWheelSpring(int64_t jh,
                                    float stiffness, float damping) {
    b2JointId id = unpackJointId(jh);
    b2WheelJoint_SetSpringHertz(id, stiffness);
    b2WheelJoint_SetSpringDampingRatio(id, damping);
}

extern "C" void b2w_setWheelMotor(int64_t jh,
                                   float speed, float maxTorque, int32_t enable) {
    b2JointId id = unpackJointId(jh);
    b2WheelJoint_SetMotorSpeed(id, speed);
    b2WheelJoint_SetMaxMotorTorque(id, maxTorque);
    b2WheelJoint_EnableMotor(id, enable != 0);
    if (enable != 0) _wakeJointBodies(id);
}

// ── Joint queries ─────────────────────────────────────────────────────────────

extern "C" void b2w_getJointReactionForce(int64_t jh,
                                           float* outFx, float* outFy) {
    b2Vec2 f = b2Joint_GetConstraintForce(unpackJointId(jh));
    *outFx = f.x;
    *outFy = f.y;
}

extern "C" float b2w_getJointReactionTorque(int64_t jh) {
    return b2Joint_GetConstraintTorque(unpackJointId(jh));
}

// ── Impact callback ───────────────────────────────────────────────────────────

extern "C" void b2w_setImpactCallback(int64_t wh,
                                       ImpactCallbackFn callback,
                                       float speedThreshold) {
    std::lock_guard<std::mutex> lk(g_worldsMutex);
    auto it = g_worlds.find(wh);
    if (it == g_worlds.end()) return;
    it->second.impactCallback  = callback;
    it->second.impactThreshold = speedThreshold;
}
