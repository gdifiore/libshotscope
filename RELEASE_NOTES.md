# v2.0.0

## Terrain System

The library now supports position-dependent terrain with sloped surfaces. You can model elevated greens, fairway transitions, rough patches, and other realistic course features.

Two new interfaces:
- `TerrainInterface` - for custom terrain implementations with height queries and surface normals
- `GroundProvider` - for position-dependent ground materials on flat terrain

Physics calculations now handle sloped surfaces correctly. Bounces decompose velocity along the surface normal, and rolling accounts for gravity components down the slope.

`FlightSimulator` has new constructors that accept terrain or ground providers. Old code still works - we didn't break the existing API.

## Fixes
- Cache comparison now uses epsilon tolerance instead of exact float equality
- Constructor initialization order matches declaration order
- Roll phase no longer stops the ball on slopes when it shouldn't

## Code cleanup
- Ground surface properties validated at construction
- Thread-safety documented for terrain adapter
- Performance optimizations with noexcept on hot paths

See `docs/terrain.md` and `docs/ground_providers.md` for examples and migration guide.
