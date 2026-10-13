# Just Physics Engine API

This document summarizes the main public API exported by `package:just_physics_engine/just_physics_engine.dart`.

## Core Entry Points

## `PhysicsEngine`

Primary entry point and common API surface.

Constructors:

- `factory PhysicsEngine()`
- `PhysicsEngine.pureDart()`

`PhysicsEngine()` creates the best backend for the current platform:

- native: `Box2DPhysicsEngine`
- web: pure-Dart fallback

## `PhysicsEngineFactory`

- `static PhysicsEngine create()`

Compatibility entry point for explicit factory-style construction.

`PhysicsEngine.pureDart()` forces the Dart implementation regardless of platform.

Key members:

- `void initialize()`
- `void update(double deltaTime)`
- `void dispose()`
- `void addBody(PhysicsBody body)`
- `void removeBody(PhysicsBody body)`
- `void setGravity(double gx, double gy)`
- `void renderDebug(Canvas canvas, Size size)`
- `List<PhysicsBody> get bodies`
- `int get bodyCount`
- `Map<String, dynamic> get stats`
- `bool debugRender`
- `Vector2 gravity`
- `void cachePolygonShape(String cacheId, List<Offset> vertices)`
- `List<Offset>? getCachedPolygonShape(String cacheId)`

Body settings, changed while the simulation runs (see [`PhysicsBody`](#physicsbody)):

- `void setBodyDamping(PhysicsBody body, double damping)` — linear (`drag`)
- `void setBodyAngularDamping(PhysicsBody body, double damping)` — spin decay per second
- `void setBodyCanSleep(PhysicsBody body, bool canSleep)` — `false` also wakes the body
- `void setBodyBullet(PhysicsBody body, bool isBullet)` — continuous collision; only the native backend acts on it

`stats` includes diagnostics such as body count, awake bodies, broad-phase stats, resolved collisions, and step time.

How two bodies' materials combine at a contact, the same on both backends:

- `static double mixFriction(double a, double b)` — the geometric mean, `sqrt(a * b)`
- `static double mixRestitution(double a, double b)` — the less bouncy, `min(a, b)`
- `static const double restitutionThreshold` — no bounce below this approach speed (100 units/s, 1 m/s)

Two static bodies are never paired, on either backend: they cannot move, so they raise no contacts or sensor events between themselves.

## `Box2DPhysicsEngine` (native backend)

Extends `PhysicsEngine` with Box2D-backed simulation.

Constructor:

- `Box2DPhysicsEngine({double gravityX = 0.0, double gravityY = 981.0, int subSteps = 4, int numThreads = 0})`

`numThreads` 0 means cores minus one, at most 4.

Important extras:

- `void pollContactBeginEvents(void Function(PhysicsBody a, PhysicsBody b, double nx, double ny) fn)`
- `Box2DJoint? createRevoluteJoint(a, b, worldAnchor)`, `createPrismaticJoint(a, b, worldAnchor, axis)`, `createDistanceJoint(a, b, {minLength, maxLength})`, `createMouseJoint(b, target)`, `createWeldJoint(a, b, worldAnchor)`, `createWheelJoint(a, b, worldAnchor, axis)`, `void destroyJoint(Box2DJoint joint)`
- `void registerImpactCallback(void Function(int bodyA, int bodyB, double speed) callback, {double speedThreshold = 100.0})`
- `PhysicsBody? physicsBodyFromHandle(int handle)`
- `double get alpha`

`stats` includes backend details (`box2d_v3` or fallback), contact count, and fixed-step metrics.

On the web the class has the same API over the pure-Dart engine: the joint creators return null, `registerImpactCallback` does nothing and `physicsBodyFromHandle` returns null.

## Character Mover

- `CharacterMover2D(PhysicsEngine engine)` — `CharacterMove2D move({required MoverShape shape, required Offset position, required Offset velocity, required double dt, Offset up, double maxSlopeAngle, double stepHeight, double stickToFloor, bool wasGrounded, MoverFilter filter, double mass, double pushStrength})`
- `CharacterMove2D` — `position`, `velocity`, `groundState` (`GroundState2D`: `onGround`, `onSteepGround`, `notSupported`, `inAir`), `groundNormal`, `groundBody`, `contacts`, `isGrounded`
- `MoverShape.capsule({radius, height})`, `MoverShape.box({width, height, cornerRadius})` — centred on the mover's position
- `MoverFilter({categoryBits, maskBits, ignore})` — never sensors, never `ignore`
- `PhysicsEngine.collideMover(shape, position, {filter})` → `List<MoverPlane>`; `PhysicsEngine.castMover(shape, position, translation, {filter})` → fraction `0..1` (native: in C)
- `MoverSolver.solvePlanes(targetDelta, planes)`, `MoverSolver.clipVector(vector, planes)` — Box2D's plane solver
- `ShapeProxy2D`, `ShapeDistance2D.between(a, b)`, `castShape2D(still, moving, translation, {required linearSlop, maxFraction, canEncroach})` — Box2D's GJK and shape cast in Dart
- `moverLinearSlop` — Box2D's linear slop in world units (0.5)

## Body and Shapes

## `PhysicsBody`

Shared simulation object used by both backends.

Constructor:

- `PhysicsBody({required Vector2 position, required CollisionShape shape, Vector2? velocity, Vector2? acceleration, double mass = 1.0, double restitution = 0.5, double friction = 0.2, double angle = 0.0, double angularVelocity = 0.0, double torque = 0.0, double inertia = 1.0, double drag = 0.1, double angularDamping = 0.0, bool useGravity = true, bool isActive = true, bool checkCollision = true, bool isAwake = true, bool canSleep = true, double sleepTimer = 0.0, double sleepVelocityThreshold = 5.0, double sleepTimeThreshold = 0.5, …})`

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

## `CollisionShape`

Abstract base for collision geometry.

Methods:

- `CollisionManifold getManifold(Offset posA, CollisionShape other, Offset posB)`
- `Rect getBounds(Offset position)`
- `double get area` — how much ground it covers (0 for a segment or a chain); a subclass must give it

Implementations:

- `CircleShape(double radius, {Offset center = Offset.zero})` — `center` puts the circle off the body's position
- `PolygonShape(List<Offset> vertices)` — convex; on the native backend one with more than 8 vertices is made of several Box2D polygons, so it keeps its whole outline
- `RoundedPolygonShape(List<Offset> vertices, double cornerRadius)`
- `RectangleShape(double width, double height)`
- `CapsuleShape(...)`, `SegmentShape(Offset point1, Offset point2, {double thickness = 2.0})`
- `ChainShape(List<Offset> vertices, {bool loop = false, double thickness = 2.0})` — every segment solid on both backends (an open chain needs 2 points, a loop 3)

`CircleShape` members:

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
- `void insert(PhysicsBody body)`
- `void syncBodies(Iterable<PhysicsBody> bodies)`
- `void removeBody(PhysicsBody body)`
- `List<BodyPair> getPotentialCollisions()`
- `int get dirtyBodyCount`
- `int get trackedCellCount`
- `int get trackedBodyCount`

## `BodyPair`

Simple pair container for broad-phase candidates.

- `BodyPair(PhysicsBody a, PhysicsBody b)`
- Fields: `a`, `b`

## `Ray`

2D ray descriptor.

Constructors:

- `Ray({required Offset origin, required Offset direction, double maxDistance = 2000.0})`
- `factory Ray.fromPoints(Offset from, Offset to, {double? maxDistance})`

Members:

- `Offset at(double t)`
- Fields: `origin`, `direction`, `maxDistance`

## Utility Types

## `RigidBody`

Lightweight force-integration helper.

Members:

- `double mass`
- `Vector2 velocity`
- `Vector2 position`
- `void applyForce(double x, double y)`
- `void integrate(double dt)`

## `CollisionDetector`

Manual overlap helper using AABB checks.

Members:

- `void addBody(PhysicsBody body)`
- `void removeBody(PhysicsBody body)`
- `List<(PhysicsBody, PhysicsBody)> detectCollisions()`

## `ForceManager`

Applies global forces to a set of bodies.

Members:

- `Vector2 gravity`
- `void setGravity(double x, double y)`
- `void applyGravity(Iterable<PhysicsBody> bodies)`

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

- `static void interpolate({required Map<PhysicsBody, Box2DBody> bodyMap, required double alpha, required void Function(PhysicsBody body, double x, double y, double angle) write})`

## 3D (`package:just_physics_engine/physics_3d.dart`)

## `PhysicsEngineFactory` (3D)

- `static PhysicsEngine3D create3D({int workerThreads = -1})` — Jolt; throws `Physics3DUnsupportedError` where it cannot run
- `static String? get unavailable3DReason`
- `static bool get supports3D`

Asking starts Jolt where it can run, with its default job system (cores minus one threads): call `create3D` first when the thread count matters.

## `PhysicsEngine3D` (interface)

- `String get name`
- `String? get unavailableReason`
- `PhysicsWorld3D createWorld(PhysicsWorldSettings3D settings)`
- `void destroyAll()`

Implementations: `JoltPhysicsEngine` (native; stub on the web), `FakePhysicsEngine3D` (`testing.dart`).

## `PhysicsWorld3D` (interface)

Int handles for bodies, joints and characters. Dead handles, and handles from another world, do nothing. A character's body (`characterBody`) belongs to its character: `destroyBody` and `createJoint` pass it over. A destroyed body's handle can come back for a new body (Jolt reuses ids), so forget it.

- world: `dispose`, `isDisposed`, `gravity`, `bodyCount`
- bodies: `createBody` (0 for a shape that is not `isValid`, a mesh or height field on a body that is not static, a layer outside 0–15, a full world), `destroyBody`, `isBody`, `setBodyPose`, `moveKinematic`, `setLinearVelocity`, `setAngularVelocity`, `addForce`, `addTorque`, `addImpulse`, `addAngularImpulse`, `setFriction`, `setRestitution`, `setGravityFactor`, `setLayer`, `setMotionQuality`, `wake`, `isAwake`
- joints: `createJoint`, `destroyJoint`, `isJoint`, `setJointEnabled`, `setJointMotor`, `jointValue`
- characters: `createCharacter`, `destroyCharacter`, `isCharacter`, `characterBody`, `moveCharacter`, `characterState`, `setCharacterPosition`, `characterContacts`
- stepping: `step`, `readMovedBodies`, `readBodies`, `drainEvents`
- queries: `raycast`, `shapeCast`, `overlap`; `debugLines`

## 3D data

Shapes `BoxShape3D`, `SphereShape3D`, `CapsuleShape3D`, `CylinderShape3D`, `ConvexHullShape3D`, `MeshShape3D`, `HeightFieldShape3D`, `CompoundShape3D`, `TransformedShape3D`; settings `PhysicsWorldSettings3D`, `BodySettings3D`, `JointSettings3D`, `CharacterSettings3D`; results `BodyPoses3D`, `QueryHit3D`, `PhysicsEvent3D`, `CharacterState3D`, `CharacterContact3D`; the query filter `QueryFilter3D` (`QueryFilter3D.all`); enums `BodyType3D`, `MotionQuality3D`, `JointKind3D`, `MotorState3D`, `SixDofAxisMode3D`, `PhysicsEventType3D`, `GroundState3D`; `Physics3DUnsupportedError`.

`Shape3D.isValid` says whether a backend can make a shape (finite sizes above zero, enough points, mesh indices inside its vertex list); both backends make nothing of one that is not.

Units are metres, seconds and radians, and +Y is down: gravity is (0, +9.81, 0) and a character's up is (0, −1, 0).
