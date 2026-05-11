/// just_physics_engine
///
/// Standalone physics engine with 2D simulation and 3D stubs.
///
/// Import this library to access:
/// - [Vector2]        — mutable 2-D vector (math primitive)
/// - [PhysicsEngine]  — 2D physics simulation
/// - [PhysicsBody]    — 2D rigid body
/// - [CollisionShape], [CircleShape], [PolygonShape], [RectangleShape]
/// - [CollisionManifold], [SpatialGrid], [BodyPair]
/// - [RigidBody], [CollisionDetector], [ForceManager]
/// - [Ray]            — 2D ray descriptor
/// - [PhysicsEngine3D], [PhysicsBody3D], [SphereShape3D], [BoxShape3D]
library;

export 'src/physics_2d/physics_2d.dart';
export 'src/physics_3d/physics_3d.dart';
