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
    bool sensor = _isSensorBody(bh);
    b2ShapeDef sd        = b2DefaultShapeDef();
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
