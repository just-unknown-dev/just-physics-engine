# Just Physics Engine API

This document summarizes the main public API exported by `package:just_physics_engine/just_physics_engine.dart`.

## Core Entry Points

## `Physics2DEngine`

Primary entry point and common API surface.

Constructors:

- `factory Physics2DEngine()`
- `Physics2DEngine.pureDart({bool fixedTimestep = true})` — fixed 1/60 s sub-steps, the leftover kept as `alpha`

`Physics2DEngine()` creates the best backend for the current platform:

- native: `Box2DPhysicsEngine`
- web: pure-Dart fallback

## `PhysicsEngineFactory`

- `static Physics2DEngine create2D()` (the 2-D twin of `create3D`, below)

Compatibility entry point for explicit factory-style construction.

`Physics2DEngine.pureDart()` forces the Dart implementation regardless of platform.

Key members:

- `void initialize()`
- `void update(double deltaTime)`
- `void dispose()`
- `void addBody(Physics2DBody body)`
- `void removeBody(Physics2DBody body)`
- `void setGravity(double gx, double gy)`
- `void renderDebug(Canvas canvas, Size size)`
- `List<Physics2DBody> get bodies`
- `int get bodyCount`
- `Map<String, dynamic> get stats`
- `bool debugRender`
- `final Vector2 gravity` — change it with `setGravity`, which the native world hears too
- `double get alpha` — the fixed-step leftover, for drawing between steps
- `void cachePolygonShape(String cacheId, List<Offset> vertices)`
- `List<Offset>? getCachedPolygonShape(String cacheId)`

Body settings, changed while the simulation runs (see [`Physics2DBody`](#physics2dbody)):

- `void setBodyDamping(Physics2DBody body, double damping)` — linear (`drag`)
- `void setBodyAngularDamping(Physics2DBody body, double damping)` — spin decay per second
- `void setBodyCanSleep(Physics2DBody body, bool canSleep)` — `false` also wakes the body
- `void setBodyBullet(Physics2DBody body, bool isBullet)` — continuous collision; only the native backend acts on it

`stats` includes diagnostics such as body count, awake bodies, broad-phase stats, resolved collisions, and step time.

How two bodies' materials combine at a contact, the same on both backends:

- `static double mixFriction(double a, double b)` — the geometric mean, `sqrt(a * b)`
- `static double mixRestitution(double a, double b)` — the less bouncy, `min(a, b)`
- `static const double restitutionThreshold` — no bounce below this approach speed (100 units/s, 1 m/s)

Two static bodies are never paired, on either backend: they cannot move, so they raise no contacts or sensor events between themselves.

Events, read after `update` — that `update`'s own, however many fixed steps it ran, on both backends:

- `void pollContactBeginEvents(void Function(Physics2DBody a, Physics2DBody b, double nx, double ny) fn)`, `void pollContactEndEvents(void Function(Physics2DBody a, Physics2DBody b) fn)`
- `void pollSensorBeginEvents(void Function(Physics2DBody sensor, Physics2DBody visitor) fn)`, `pollSensorEndEvents` — the sensor first
- `void pollBodyMoveEvents(void Function(Physics2DBody body, {required bool fellAsleep}) fn)`

Queries see every shape of a body, turned with the body as Box2D turns it (the pure-Dart engine's own collisions do not turn shapes):

- `Ray2DHit? raycast(Ray2D ray, {Set<Physics2DBody>? exclude})`, `List<Ray2DHit> raycastAll(...)` — nearest first, one hit per body; a shape the ray starts inside is passed over
- `CircleCast2DHit? circleCast(Offset origin, double radius, Offset motion, {Set<Physics2DBody>? exclude})` — Box2D's shape cast, stopping where the circle touches (to half a unit); one that starts out touching hits at `toi` 0
- `List<Physics2DBody> queryAABB(Rect rect)`, `queryCircle(Offset center, double radius)`, `queryPoint(Offset point)` — on the native backend `queryAABB` answers with Box2D's boxes, kept a tenth of a metre larger than the shapes

Joints — `addRevoluteJoint`, `addDistanceJoint`, `addWeldJoint`, `addPrismaticJoint`, `addWheelJoint`, `addMouseJoint`, `removeJoint`, `joints` (each `Joint2D` names its `bodies`). The Box2D backend makes native joints, all but the mouse joint, a Dart joint whose force reaches the native body. `removeJoint` destroys a native joint, and `removeBody` takes a body's joints with it. A slider's or a wheel's travel, and a hinge's angle, count from where the joint was made, as in Box2D. The pure-Dart engine moves bodies only: a body hinged away from its centre swings round the anchor, drawn turning as it swings.

`torque` turns a body by its `inertia` on the pure-Dart engine, and by its shapes' rotational inertia (Box2D's) on the native one.

## `Box2DPhysicsEngine` (native backend)

Extends `Physics2DEngine` with Box2D-backed simulation.

Constructor:

- `Box2DPhysicsEngine({double gravityX = 0.0, double gravityY = 981.0, int subSteps = 4, int numThreads = 0})`

`numThreads` 0 means cores minus one, at most 4.

Important extras:

- `Box2DJoint? createRevoluteJoint(a, b, worldAnchor)`, `createPrismaticJoint(a, b, worldAnchor, axis)`, `createDistanceJoint(a, b, {minLength, maxLength})`, `createWeldJoint(a, b, worldAnchor)`, `createWheelJoint(a, b, worldAnchor, axis)`, `void destroyJoint(Box2DJoint joint)`
- `void registerImpactCallback(void Function(int bodyA, int bodyB, double speed) callback, {double speedThreshold = 100.0})`: Box2D's hit events, every contact that closed faster than the threshold

Box2D v3 has no mouse joint: `addMouseJoint` gives the pure-Dart `MouseJoint2D` on every backend. A native distance or wheel joint's spring is a frequency in hertz and a damping ratio (`Box2DJoint.setDistanceSpring`, `setWheelSpring`).
- `Physics2DBody? bodyFromHandle(int handle)`

`stats` includes backend details (`box2d_v3` or fallback), contact count, and fixed-step metrics.

On the web the class has the same API over the pure-Dart engine: the joint creators return null, `registerImpactCallback` does nothing and `bodyFromHandle` returns null.

## Character Mover

- `CharacterMover2D(Physics2DEngine engine)` — `CharacterMove2D move({required Mover2DShape shape, required Offset position, required Offset velocity, required double dt, Offset up, double maxSlopeAngle, double stepHeight, double stickToFloor, bool wasGrounded, Mover2DFilter filter, double mass, double pushStrength})`
- `CharacterMove2D` — `position`, `velocity`, `groundState` (`Ground2DState`: `onGround`, `onSteepGround`, `notSupported`, `inAir`), `groundNormal`, `groundBody`, `contacts`, `isGrounded`
- `Mover2DShape.capsule({radius, height})`, `Mover2DShape.box({width, height, cornerRadius})` — centred on the mover's position
- `Mover2DFilter({categoryBits, maskBits, ignore})` — never sensors, never `ignore`
- `Physics2DEngine.collideMover(shape, position, {filter})` → `List<Mover2DPlane>`; `Physics2DEngine.castMover(shape, position, translation, {filter})` → fraction `0..1` (native: in C)
- `Mover2DSolver.solvePlanes(targetDelta, planes)`, `Mover2DSolver.clipVector(vector, planes)` — Box2D's plane solver
- `Shape2DProxy`, `Shape2DDistance.between(a, b)`, `shapeCast2D(still, moving, translation, {required linearSlop, maxFraction, canEncroach})` — Box2D's GJK and shape cast in Dart
- `moverLinearSlop` — Box2D's linear slop in world units (0.5)

## Body and Shapes

## `Physics2DBody`

Shared simulation object used by both backends.

Constructor:

- `Physics2DBody({required Vector2 position, required Shape2D shape, Vector2? velocity, Vector2? acceleration, double mass = 1.0, double restitution = 0.5, double friction = 0.2, double angle = 0.0, double angularVelocity = 0.0, double torque = 0.0, double inertia = 1.0, double drag = 0.1, double angularDamping = 0.0, bool useGravity = true, bool isActive = true, bool checkCollision = true, bool isAwake = true, bool canSleep = true, double sleepTimer = 0.0, double sleepVelocityThreshold = 5.0, double sleepTimeThreshold = 0.5, …})`

Methods:

- `void applyForce(Vector2 force)`
- `void applyTorque(double applicationTorque)`
- `void applyImpulse(Vector2 impulse)`

Common fields:

- `position`, `velocity`, `acceleration`
- `shape`
- `mass`, `inverseMass`
- `restitution`, `friction`, `drag`, `angularDamping` — `drag` slows only the velocity and `angularDamping` only the spin, on both backends
- `angle`, `angularVelocity`, `torque`, `inertia`, `inverseInertia`
- sleep and activity flags (`isActive`, `isAwake`, `canSleep`, etc.)
- `isBullet` (continuous collision; native backend only)

Changed while the simulation runs through the engine, so the native backend hears it: `setBodyFriction`, `setBodyRestitution`, `setBodyDamping`, `setBodyAngularDamping`, `setBodyCanSleep` (forbidding sleep wakes the body), `setBodyBullet`, `setBodyType`, `setBodyGravityScale`, `setBodyFilter`, `setBodyOneWay`, `setBodyTransform`.

## `Shape2D`

Abstract base for collision geometry.

Methods:

- `CollisionManifold getManifold(Offset posA, Shape2D other, Offset posB)`
- `Rect getBounds(Offset position)`
- `double get area` — how much ground it covers (0 for a segment or a chain); a subclass must give it

Implementations:

- `Circle2DShape(double radius, {Offset center = Offset.zero})` — `center` puts the circle off the body's position
- `Polygon2DShape(List<Offset> vertices)` — convex; on the native backend one with more than 8 vertices is made of several Box2D polygons, so it keeps its whole outline
- `RoundedPolygon2DShape(List<Offset> vertices, double cornerRadius)`
- `Rectangle2DShape(double width, double height)`
- `Capsule2DShape(...)`, `Segment2DShape(Offset point1, Offset point2, {double thickness = 2.0})` — collides as the capsule `thickness` round it, on both backends
- `Chain2DShape(List<Offset> vertices, {bool loop = false, double thickness = 2.0})` — every segment solid from both sides on both backends, `thickness` round it (natively a capsule per segment, or for a `thickness` of 0 Box2D's own chain, put in both ways round); an open chain needs 2 points, a loop 3

`Circle2DShape` members:

- `double radius`
- `Offset center` — relative to the body's position. The native backend rotates it with the body; the pure-Dart backend, which does not rotate shapes, leaves it where it is.
- `Offset centerAt(Offset position)` — the circle's centre in the world for a body at `position`

## `CollisionManifold`

Narrow-phase collision result.

Constructor and factory:

- `CollisionManifold({required bool isColliding, Offset normal = Offset.zero, double penetration = 0.0})`
- `factory CollisionManifold.empty()`

Fields:

- `isColliding`
- `normal`
- `penetration`

## Broad-Phase and Helpers

## `SpatialGrid`

Uniform-grid broad-phase helper.

Constructor:

- `SpatialGrid(double cellSize)`

Methods and properties:

- `void clear()`
- `void insert(Physics2DBody body)`
- `void syncBodies(Iterable<Physics2DBody> bodies)`
- `void removeBody(Physics2DBody body)`
- `List<BodyPair> getPotentialCollisions()`
- `int get dirtyBodyCount`
- `int get trackedCellCount`
- `int get trackedBodyCount`

## `BodyPair`

Simple pair container for broad-phase candidates.

- `BodyPair(Physics2DBody a, Physics2DBody b)`
- Fields: `a`, `b`

## `Ray2D`

2D ray descriptor.

Constructors:

- `Ray2D({required Offset origin, required Offset direction, double maxDistance = 2000.0})`
- `factory Ray2D.fromPoints(Offset from, Offset to, {double? maxDistance})`

Members:

- `Offset at(double t)`
- Fields: `origin`, `direction`, `maxDistance`

## Utility Types

## `CollisionDetector`

Manual overlap helper using AABB checks.

Members:

- `void addBody(Physics2DBody body)`
- `void removeBody(Physics2DBody body)`
- `List<(Physics2DBody, Physics2DBody)> detectCollisions()`

## `ForceManager`

Applies global forces to a set of bodies.

Members:

- `Vector2 gravity`
- `void setGravity(double x, double y)`
- `void applyGravity(Iterable<Physics2DBody> bodies)`

## Native Box2D Public Types

These are the native classes, and what the API docs show. On the web the same API is there and does nothing (the pure-Dart engine runs instead); the static `Box2DJointFactory` creators, which need a native world, throw `UnsupportedError`.

## `Box2DWorld`

- `factory Box2DWorld({double gravityX = 0.0, double gravityY = 981.0, int numThreads = 0, int subSteps = 4})` — `numThreads` 0 means cores minus one, at most 4
- `int step(double dt)`
- `void setGravity(double gx, double gy)`
- `void dispose()`
- Getters: `double alpha`, `int handle`, `int subSteps`

## `Box2DBody`

Constructors:

- `Box2DBody.dynamic(...)`
- `Box2DBody.static(...)`
- `Box2DBody.ofType({required int worldHandle, required int bodyTypeIndex, required double posX, required double posY, double angle = 0.0})`

Members:

- `void setTransform(double x, double y, double angle, {bool wake = true})`
- `int get typeIndex`, `void setType(int bodyTypeIndex)`
- `void capturePrevious()`
- `void destroy()`
- `void applyForce(double fx, double fy)`
- `void applyLinearImpulse(double ix, double iy)`
- `void applyTorque(double t)`
- `void setLinearVelocity(double vx, double vy)`
- state fields: `prevX`, `prevY`, `prevAngle`, `currentX`, `currentY`, `currentAngle`, `velocityX`, `velocityY`, `angularVelocity` (the spin Box2D reported after the last step)

## `PhysicsGameLoop`

- `PhysicsGameLoop(Box2DWorld world)`
- `int advance(double dt)`
- `double get alpha`

## `TransformInterpolator`

- `static void interpolate({required Map<Physics2DBody, Box2DBody> bodyMap, required double alpha, required void Function(Physics2DBody body, double x, double y, double angle) write})`

## 3D (`package:just_physics_engine/physics_3d.dart`)

## `PhysicsEngineFactory` (3D)

- `static Physics3DEngine create3D({int workerThreads = -1})` — Jolt; throws `Physics3DUnsupportedError` where it cannot run
- `static String? get unavailable3DReason`
- `static bool get supports3D`

Asking starts Jolt where it can run, with its default job system (cores minus one threads): call `create3D` first when the thread count matters.

## `Physics3DEngine` (interface)

- `String get name`
- `String? get unavailableReason`
- `Physics3DWorld createWorld(Physics3DWorldSettings settings)`
- `void destroyAll()`

Implementations: `JoltPhysicsEngine` (native; stub on the web), `FakePhysics3DEngine` (`testing.dart`).

## `Physics3DWorld` (interface)

Int handles for bodies, joints and characters. Dead handles, and handles from another world, do nothing. A character's body (`characterBody`) belongs to its character: `destroyBody` and `createJoint` pass it over. A destroyed body's handle can come back for a new body (Jolt reuses ids), so forget it.

- world: `dispose`, `isDisposed`, `gravity`, `bodyCount`
- bodies: `createBody` (0 for a shape that is not `isValid`, a mesh or height field on a body that is not static, a layer outside 0–15, a full world), `destroyBody`, `isBody`, `setBodyPose`, `moveKinematic`, `setLinearVelocity`, `setAngularVelocity`, `addForce`, `addTorque`, `addImpulse`, `addAngularImpulse`, `setFriction`, `setRestitution`, `setGravityFactor`, `setLayer`, `setMotionQuality`, `wake`, `isAwake`
- joints: `createJoint`, `destroyJoint`, `isJoint`, `setJointEnabled`, `setJointMotor`, `jointValue`
- characters: `createCharacter`, `destroyCharacter`, `isCharacter`, `characterBody`, `moveCharacter`, `characterState`, `setCharacterPosition`, `characterContacts`
- stepping: `step`, `readMovedBodies`, `readBodies`, `drainEvents`
- queries: `raycast`, `shapeCast`, `overlap`; `debugLines`

## 3D data

Shapes `Box3DShape`, `Sphere3DShape`, `Capsule3DShape`, `Cylinder3DShape`, `ConvexHull3DShape`, `Mesh3DShape`, `HeightField3DShape`, `Compound3DShape`, `Transformed3DShape`; settings `Physics3DWorldSettings`, `Body3DSettings`, `Joint3DSettings`, `Character3DSettings`; results `Body3DPoses`, `Query3DHit`, `Physics3DEvent`, `Character3DState`, `Character3DContact`; the query filter `Query3DFilter` (`Query3DFilter.all`); enums `Body3DType`, `Motion3DQuality`, `Joint3DKind`, `Motor3DState`, `SixDofAxis3DMode`, `Physics3DEventType`, `Ground3DState`; `Physics3DUnsupportedError`.

`Shape3D.isValid` says whether a backend can make a shape (finite sizes above zero, enough points, mesh indices inside its vertex list); both backends make nothing of one that is not.

Units are metres, seconds and radians, and +Y is down: gravity is (0, +9.81, 0) and a character's up is (0, −1, 0).
