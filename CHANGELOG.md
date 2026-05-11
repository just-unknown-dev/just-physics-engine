## 0.1.0 - 2026-05-11

Initial release of just_physics_engine.

### Added

- 2D rigid-body simulation with PhysicsEngine and PhysicsBody.
- Core physical properties including gravity, drag, restitution, friction, torque, and sleeping.
- Collision shapes: CircleShape, RectangleShape, and PolygonShape.
- Broad-phase collision culling via SpatialGrid.
- Collision resolution with impulse response, friction, and positional correction.
- Runtime simulation stats via engine.stats.
- Debug rendering support via engine.renderDebug.
- 2D ray utility helpers with Ray and Ray.fromPoints.
- Initial 3D API scaffolding stubs: PhysicsEngine3D

### Notes

- This release targets Dart SDK ^3.11.0 and Flutter >=1.17.0.
- 2D simulation is production-ready; 3D APIs are currently stubs/scaffolding.

