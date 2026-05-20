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
- `Map<String, dynamic> get stats`
- `bool debugRender`
- `Vector2 gravity`
- `void cachePolygonShape(String cacheId, List<Offset> vertices)`
- `List<Offset>? getCachedPolygonShape(String cacheId)`

`stats` includes diagnostics such as body count, awake bodies, broad-phase stats, resolved collisions, and step time.

## `Box2DPhysicsEngine` (native backend)

Extends `PhysicsEngine` with Box2D-backed simulation.

Constructor:

- `Box2DPhysicsEngine({double gravityX = 0.0, double gravityY = 981.0, int subSteps = 4, int numThreads = 0})`

Important extras:

- `void pollContactBeginEvents(void Function(PhysicsBody a, PhysicsBody b, double nx, double ny) fn)`
- `void registerImpactCallback(void Function(int bodyA, int bodyB, double speed) callback, {double speedThreshold = 100.0})`
- `PhysicsBody? physicsBodyFromHandle(int handle)`
- `double get alpha`

`stats` includes backend details (`box2d_v3` or fallback), contact count, and fixed-step metrics.

## Body and Shapes

## `PhysicsBody`

Shared simulation object used by both backends.

Constructor:

- `PhysicsBody({required Vector2 position, required CollisionShape shape, Vector2? velocity, Vector2? acceleration, double mass = 1.0, double restitution = 0.5, double friction = 0.2, double angle = 0.0, double angularVelocity = 0.0, double torque = 0.0, double inertia = 1.0, double drag = 0.1, bool useGravity = true, bool isActive = true, bool checkCollision = true, bool isAwake = true, double sleepTimer = 0.0, double sleepVelocityThreshold = 5.0, double sleepTimeThreshold = 0.5})`

Methods:

- `void applyForce(Vector2 force)`
- `void applyTorque(double applicationTorque)`
- `void applyImpulse(Vector2 impulse)`

Common fields:

- `position`, `velocity`, `acceleration`
- `shape`
- `mass`, `inverseMass`
- `restitution`, `friction`, `drag`
- `angle`, `angularVelocity`, `torque`, `inertia`, `inverseInertia`
- sleep and activity flags (`isActive`, `isAwake`, etc.)

## `CollisionShape`

Abstract base for collision geometry.

Methods:

- `CollisionManifold getManifold(Offset posA, CollisionShape other, Offset posB)`
- `Rect getBounds(Offset position)`

Implementations:

- `CircleShape(double radius)`
- `PolygonShape(List<Offset> vertices)`
- `RectangleShape(double width, double height)`

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

These are exported on native platforms and as stubs on web for compatibility.

## `Box2DWorld`

- `factory Box2DWorld({double gravityX = 0.0, double gravityY = 981.0, int numThreads = 0, int subSteps = 4})`
- `int step(double dt)`
- `void setGravity(double gx, double gy)`
- `void dispose()`
- Getters: `double alpha`, `int handle`, `int subSteps`

## `Box2DBody`

Constructors:

- `Box2DBody.dynamic(...)`
- `Box2DBody.static(...)`

Members:

- `void capturePrevious()`
- `void destroy()`
- `void applyForce(double fx, double fy)`
- `void applyLinearImpulse(double ix, double iy)`
- `void applyTorque(double t)`
- `void setLinearVelocity(double vx, double vy)`
- state fields: `prevX`, `prevY`, `prevAngle`, `currentX`, `currentY`, `currentAngle`, `velocityX`, `velocityY`

## `PhysicsGameLoop`

- `PhysicsGameLoop(Box2DWorld world)`
- `int advance(double dt)`
- `double get alpha`

## `TransformInterpolator`

- `static void interpolate({required Map<PhysicsBody, Box2DBody> bodyMap, required double alpha, required void Function(PhysicsBody body, double x, double y, double angle) write})`

## 3D API Status

## `PhysicsEngine3D`

Current placeholder API:

- `void initialize()`
- `void update(double deltaTime)`
- `void dispose()`

3D simulation is not yet implemented.
