# Just Physics Engine API

This document summarizes the main public API exported by `package:just_physics_engine/just_physics_engine.dart`.

## Core Entry Points

## `PhysicsEngineFactory`

- `static Physics2DEngine create2D()` — the platform's 2-D engine: `Box2DPhysicsEngine` on native platforms (falling back to the pure-Dart engine where just_box_2d's library is not built), the pure-Dart `Physics2DEngine` on the web
- `static Physics3DEngine create3D({int workerThreads = -1})`, `unavailable3DReason`, `supports3D` — see [3D](#3d-packagejust_physics_enginephysics_3ddart)

## `Physics2DEngine`

The 2-D engine: the pure-Dart one, and the API the native `Box2DPhysicsEngine` runs with the same meaning, so game code needs no backend branch.

Constructor:

- `Physics2DEngine.pureDart({bool fixedTimestep = true})` — the pure-Dart engine on any platform; fixed 1/60 s sub-steps, the leftover kept as `alpha`

Key members:

- `void initialize()` — before adding bodies (the Box2D backend makes its world here)
- `void update(double deltaTime)` — a `deltaTime` that is not a positive, finite number runs no step
- `void dispose()`
- `void addBody(Physics2DBody body)`, `void removeBody(Physics2DBody body)` (and its joints)
- `void setGravity(double gx, double gy)` — wakes sleeping dynamic bodies
- `void renderDebug(Canvas canvas, Size size)`, `bool debugRender`
- `List<Physics2DBody> get bodies` — a live view, not a copy; `int get bodyCount`
- `bool get isNative` — whether Box2D runs (false for the pure-Dart engine, and for a Box2D engine without its library)
- `Map<String, dynamic> get stats`
- `final Vector2 gravity` — change it with `setGravity`, which the native world hears too
- `double get alpha` — the fixed-step leftover, for drawing between steps

Body settings, changed while the simulation runs (see [`Physics2DBody`](#physics2dbody)); the native backend hears each at once:

- `void setBodyActive(Physics2DBody body, bool active)` — out of the simulation (no motion, contacts, sensor overlaps, queries, movers or joints) and back, woken
- `void wakeBody(Physics2DBody body)` — a sleeping body is simulated again
- `void setBodyFixedRotation(Physics2DBody body, bool fixed)` — on a live body
- `void setBodyTransform(body, x, y, {angle})`, `setBodyType`, `setBodyGravityScale` (wakes the body), `setBodyFilter`, `setBodyOneWay`
- `void setBodyDamping(Physics2DBody body, double damping)` — linear (`drag`)
- `void setBodyAngularDamping(Physics2DBody body, double damping)` — spin decay per second
- `void setBodyCanSleep(Physics2DBody body, bool canSleep)` — `false` also wakes the body
- `void setBodyBullet(Physics2DBody body, bool isBullet)` — continuous collision; only the native backend acts on it
- `void setBodyFriction`, `setBodyRestitution`, `setBodySensor` (create-time only natively)
- `void applyLinearImpulse(Physics2DBody body, double ix, double iy)`

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

Queries see every shape of every active body, turned with the body as Box2D turns it (the pure-Dart engine's own collisions do not turn shapes):

- `Ray2DHit? raycast(Ray2D ray, {Set<Physics2DBody>? exclude})`, `List<Ray2DHit> raycastAll(...)` — nearest first, one hit per body; a shape the ray starts inside is passed over
- `CircleCast2DHit? circleCast(Offset origin, double radius, Offset motion, {Set<Physics2DBody>? exclude})` — Box2D's shape cast, stopping where the circle touches (to half a unit); one that starts out touching hits at `toi` 0
- `List<Physics2DBody> queryAABB(Rect rect)`, `queryCircle(Offset center, double radius)`, `queryPoint(Offset point)` — on the native backend `queryAABB` answers with Box2D's boxes, kept a tenth of a metre larger than the shapes, and the others take their candidates from Box2D's tree

## Joints

Made by the engine; anchors are world points as the bodies are now, and a joint's two bodies pass through each other unless `collideConnected` (on both backends; sensors still see them):

- `Joint2D addRevoluteJoint(a, b, Offset worldAnchor, {bool collideConnected = false})`
- `Joint2D addDistanceJoint(a, b, {Offset? anchorA, Offset? anchorB, double? length, double? minLength, double? maxLength, double springHertz = 0, double dampingRatio = 0.3, bool collideConnected = false})` — the anchors default to the centres, the length to their distance
- `Joint2D addWeldJoint(a, b, {Offset? anchor, bool collideConnected = false})` — at their midpoint by default
- `Joint2D addPrismaticJoint(a, b, Offset axis, {Offset? anchor, bool collideConnected = false})`, `Joint2D addWheelJoint(a, b, Offset axis, {Offset? anchor, bool collideConnected = false})` — through `b`'s centre by default; a wheel's suspension is 4 Hz, damping ratio 0.7, until `setSpring`
- `Joint2D addMouseJoint(Physics2DBody b, Vector2 target)` — a Dart joint on both backends (Box2D v3 has none)
- `void addJoint(Joint2D joint)`, `void removeJoint(Joint2D joint)` (destroys a native one), `List<Joint2D> get joints`

`Joint2D`, the same calls in the same units on both backends:

- `Joint2DKind kind`, `bool collideConnected`, `List<Physics2DBody> bodies`
- `void setLimits(double lower, double upper, {bool enable = true})` — a hinge's angle (radians, from the angle it was made at), a slider's travel (from where it was made), a distance joint's shortest and longest length
- `void setMotor(double speed, double maxForceOrTorque, {bool enable = true})` — a hinge's or wheel's spin, a slider's travel
- `void setSpring(double hertz, double dampingRatio)` — a distance joint's (toward the middle of its range) or a wheel's suspension; 0 Hz takes it away
- `Offset get reactionForce`, `double get reactionTorque` — what a native joint applied in the last step (zero for the pure-Dart joints)

The kinds: `DistanceJoint2D` (`minLength`, `maxLength`, `springHertz`, `dampingRatio`, `currentLength`), `RevoluteJoint2D` (`jointAngle`), `PrismaticJoint2D` (`translation`), `WheelJoint2D`, `WeldJoint2D`, `MouseJoint2D` (`setTarget`, its own `stiffness`, `damping`, `maxForce`), and natively `Box2DJoint` (`isDestroyed`). A slider's or a wheel's travel, and a hinge's angle, count from where the joint was made, on both backends. The pure-Dart engine moves bodies only: a body hinged away from its centre swings round the anchor, drawn turning as it swings.

`torque` turns a body by its `inertia` on the pure-Dart engine, and by its shapes' rotational inertia (Box2D's) on the native one.

## `Box2DPhysicsEngine` (native backend)

Extends `Physics2DEngine` with Box2D-backed simulation.

Constructor:

- `Box2DPhysicsEngine({double gravityX = 0.0, double gravityY = 981.0, int subSteps = 4, int numThreads = 0})`

`numThreads` 0 means cores minus one, at most 4. Bodies added before `initialize` go into the native world with it.

Extras:

- `void registerImpactCallback(void Function(int bodyA, int bodyB, double speed) callback, {double speedThreshold = 100.0})`: Box2D's hit events, every contact that closed faster than the threshold
- `Physics2DBody? bodyFromHandle(int handle)`

`stats` includes backend details (`box2d_v3` or fallback), contact count, and fixed-step metrics; `isNative` says which runs.

On the web the class has the same API over the pure-Dart engine: `registerImpactCallback` does nothing and `bodyFromHandle` returns null.

## Character Mover

- `CharacterMover2D(Physics2DEngine engine)` — `CharacterMove2D move({required Mover2DShape shape, required Offset position, required Offset velocity, required double dt, Offset up, double maxSlopeAngle, double stepHeight, double stickToFloor, bool wasGrounded, Mover2DFilter filter, double mass, double pushStrength})`
- `CharacterMove2D` — `position`, `velocity`, `groundState` (`Ground2DState`: `onGround`, `onSteepGround`, `notSupported`, `inAir`), `groundNormal`, `groundBody`, `contacts`, `isGrounded`
- `Mover2DShape.capsule({radius, height})`, `Mover2DShape.box({width, height, cornerRadius})` — centred on the mover's position
- `Mover2DFilter({categoryBits, maskBits, ignore})` — never sensors, never `ignore`, never an inactive body
- `Physics2DEngine.collideMover(shape, position, {filter})` → `List<Mover2DPlane>`; `Physics2DEngine.castMover(shape, position, translation, {filter})` → fraction `0..1` (native: in C)
- `Mover2DSolver.solvePlanes(targetDelta, planes)`, `Mover2DSolver.clipVector(vector, planes)` — Box2D's plane solver
- `Shape2DProxy`, `Shape2DDistance.between(a, b)`, `shapeCast2D(still, moving, translation, {required linearSlop, maxFraction, canEncroach})` — Box2D's GJK and shape cast in Dart
- `moverLinearSlop` — Box2D's linear slop in world units (0.5)

## Body and Shapes

## `Physics2DBody`

Shared simulation object used by both backends.

Constructor:

- `Physics2DBody({required Vector2 position, required Shape2D shape, Vector2? velocity, Vector2? acceleration, double mass = 1.0, double restitution = 0.5, double friction = 0.2, double angle = 0.0, double angularVelocity = 0.0, double torque = 0.0, double inertia = 1.0, double drag = 0.1, double angularDamping = 0.0, bool useGravity = true, double gravityScale = 1.0, bool isActive = true, bool isAwake = true, bool canSleep = true, double sleepTimer = 0.0, double sleepVelocityThreshold = 5.0, double sleepTimeThreshold = 0.5, …})`

Methods:

- `void applyForce(Vector2 force)`, `void applyTorque(double applicationTorque)` — for the next step; a dynamic body wakes, anything else takes none
- `Rect boundsAt(Offset position)` — the box round all its shapes, unturned
- `List<Shape2DProxy> proxies({Offset shift = Offset.zero})` — its shapes as the distance and cast tests see them, turned with it and moved by `shift`

Common fields:

- `position`, `velocity`, `acceleration`
- `shape`, `additionalShapes`
- `mass`, `inverseMass`
- `restitution`, `friction`, `drag`, `angularDamping` — `drag` slows only the velocity and `angularDamping` only the spin, on both backends
- `angle`, `angularVelocity`, `torque`, `inertia`, `inverseInertia`, `fixedRotation`
- activity and sleep (`isActive`, `isAwake`, `canSleep`, …)
- `isBullet` (continuous collision; native backend only)

A field written directly reaches the native backend at the next `update` for `velocity`, `angularVelocity`, forces, `isAwake`, `isActive`, `fixedRotation`, the type, gravity scale and mass; the others need the engine's `setBody*` method.

## `Shape2D`

Abstract base for collision geometry.

Methods:

- `CollisionManifold manifoldWith(Offset posA, Shape2D other, Offset posB)`
- `Rect boundsAt(Offset position)`
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

## `Ray2D`

2D ray descriptor.

Constructors:

- `Ray2D({required Offset origin, required Offset direction, double maxDistance = 2000.0})`
- `factory Ray2D.fromPoints(Offset from, Offset to, {double? maxDistance})`

Members:

- `Offset at(double t)`
- Fields: `origin`, `direction`, `maxDistance`

## 3D (`package:just_physics_engine/physics_3d.dart`)

## `PhysicsEngineFactory` (3D)

- `static Physics3DEngine create3D({int workerThreads = -1})` — Jolt; throws `Physics3DUnsupportedError` where it cannot run
- `static String? get unavailable3DReason`
- `static bool get supports3D`

Asking starts Jolt where it can run, with its default job system (cores minus one threads): call `create3D` first when the thread count matters. `create2D` is there too, but its `Physics2DEngine` is exported by `just_physics_engine.dart`.

## `Physics3DEngine` (interface)

- `String get name`
- `String? get unavailableReason`
- `Physics3DWorld createWorld(Physics3DWorldSettings settings)`
- `void destroyAll()`

Implementations: `JoltPhysicsEngine` (native; stub on the web), `FakePhysics3DEngine` (`testing.dart`).

## `Physics3DWorld` (interface)

Int handles for bodies, joints and characters. Dead handles, and handles from another world, do nothing. A character's body (`characterBody`) belongs to its character: `destroyBody` and `createJoint` pass it over. A destroyed body's handle can come back for a new body (Jolt reuses ids), so forget it.

- world: `dispose`, `isDisposed`, `gravity`, `bodyCount`, `layerMasks`, `setLayerMasks` (the collision grid of a live world: every body that can move wakes, so pairs at rest are judged again)
- bodies: `createBody` (0 for a shape that is not `isValid`, a mesh or height field on a body that is not static, a layer outside 0–15, a full world), `destroyBody`, `isBody`, `setBodyPose`, `moveKinematic`, `setLinearVelocity`, `setAngularVelocity`, `addForce`, `addTorque`, `addImpulse`, `addAngularImpulse`, `setFriction`, `setRestitution`, `setGravityFactor`, `setLayer`, `setMotionQuality`, `wake`, `isAwake`
- joints: `createJoint`, `destroyJoint`, `isJoint`, `setJointEnabled`, `setJointMotor`, `jointValue`
- characters: `createCharacter`, `destroyCharacter`, `isCharacter`, `characterBody`, `moveCharacter`, `characterState`, `setCharacterPosition`, `readCharacterContacts` (packed, into `Character3DContacts`), `characterContacts` (objects)
- stepping: `step`, `readMovedBodies`, `readBodies`, `readEvents` (packed, into `Physics3DEvents`: no object per event), `drainEvents` (objects, for tests and code that runs rarely)
- queries: `raycast`, `shapeCast`, `overlap`; `debugLines`

## 3D data

Shapes `Box3DShape`, `Sphere3DShape`, `Capsule3DShape`, `Cylinder3DShape`, `ConvexHull3DShape`, `Mesh3DShape`, `HeightField3DShape`, `Compound3DShape`, `Transformed3DShape`; settings `Physics3DWorldSettings` (`normalizeLayerMasks`), `Body3DSettings`, `Joint3DSettings`, `Character3DSettings`; results `Body3DPoses`, `Physics3DEvents`, `Character3DContacts`, `Query3DHit`, `Physics3DEvent`, `Character3DState`, `Character3DContact`; the query filter `Query3DFilter` (`Query3DFilter.all`); enums `Body3DType`, `Motion3DQuality`, `Joint3DKind`, `Motor3DState`, `SixDofAxis3DMode`, `Physics3DEventType`, `Ground3DState`; `Physics3DUnsupportedError`. Both public libraries export just_dart's `Vector3` and `Quaternion`, which the 3D API takes.

`Shape3D.isValid` says whether a backend can make a shape (finite sizes above zero, enough points, mesh indices inside its vertex list); both backends make nothing of one that is not.

Units are metres, seconds and radians, and +Y is down: gravity is (0, +9.81, 0) and a character's up is (0, −1, 0).
