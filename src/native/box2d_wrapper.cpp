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
#include <cmath>
#include <mutex>
#include <thread>
#include <unordered_map>

// ── Global state ─────────────────────────────────────────────────────────────

struct WorldState {
    ThreadPool*      pool;
    ImpactCallbackFn impactCallback;
    float            impactThreshold;
    b2ContactEvents  lastContacts;
};

static std::unordered_map<int64_t, WorldState> g_worlds;
static std::mutex g_worldsMutex;

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

extern "C" int64_t b2w_createWorld(float gx, float gy, int32_t numThreads) {
    if (numThreads <= 0) {
        unsigned hw = std::thread::hardware_concurrency();
        numThreads  = (int32_t)(hw > 1 ? hw - 1 : 1);
    }

    ThreadPool* pool = new ThreadPool(numThreads);

    b2WorldDef def      = b2DefaultWorldDef();
    def.gravity         = {gx, gy};
    def.workerCount     = pool->workerCount();
    def.userTaskContext = pool;
    def.enqueueTask     = s_enqueueTask;
    def.finishTask      = s_finishTask;

    b2WorldId id = b2CreateWorld(&def);
    int64_t   h  = packWorldId(id);

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
    b2BodyDef def = b2DefaultBodyDef();
    def.type      = b2_dynamicBody;
    def.position  = {x, y};
    def.rotation  = b2MakeRot(angle);
    return packBodyId(b2CreateBody(unpackWorldId(wh), &def));
}

extern "C" int64_t b2w_createStaticBody(int64_t wh,
                                         float x, float y, float angle) {
    b2BodyDef def = b2DefaultBodyDef();
    def.type      = b2_staticBody;
    def.position  = {x, y};
    def.rotation  = b2MakeRot(angle);
    return packBodyId(b2CreateBody(unpackWorldId(wh), &def));
}

extern "C" void b2w_destroyBody(int64_t bh) {
    b2DestroyBody(unpackBodyId(bh));
}

// ── Shapes ────────────────────────────────────────────────────────────────────

extern "C" void b2w_addCircleShape(int64_t bh, float radius,
                                    float density, float friction,
                                    float restitution) {
    b2ShapeDef sd        = b2DefaultShapeDef();
    sd.density           = density;
    sd.material.friction    = friction;
    sd.material.restitution = restitution;

    b2Circle circle = {{0.0f, 0.0f}, radius};
    b2CreateCircleShape(unpackBodyId(bh), &sd, &circle);
}

extern "C" void b2w_addBoxShape(int64_t bh, float halfW, float halfH,
                                 float density, float friction,
                                 float restitution) {
    b2ShapeDef sd        = b2DefaultShapeDef();
    sd.density           = density;
    sd.material.friction    = friction;
    sd.material.restitution = restitution;

    b2Polygon box = b2MakeBox(halfW, halfH);
    b2CreatePolygonShape(unpackBodyId(bh), &sd, &box);
}

extern "C" void b2w_addPolygonShape(int64_t bh,
                                     const float* verts, int32_t count,
                                     float density, float friction,
                                     float restitution) {
    b2ShapeDef sd        = b2DefaultShapeDef();
    sd.density           = density;
    sd.material.friction    = friction;
    sd.material.restitution = restitution;

    b2Vec2 pts[B2_MAX_POLYGON_VERTICES];
    int32_t n = count < B2_MAX_POLYGON_VERTICES ? count : B2_MAX_POLYGON_VERTICES;
    for (int32_t i = 0; i < n; ++i) {
        pts[i] = {verts[i * 2], verts[i * 2 + 1]};
    }

    b2Hull    hull = b2ComputeHull(pts, n);
    b2Polygon poly = b2MakePolygon(&hull, 0.0f);
    b2CreatePolygonShape(unpackBodyId(bh), &sd, &poly);
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
        slot[5] = 0.0f;
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
    *outNx = 0.0f;
    *outNy = 0.0f;
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
