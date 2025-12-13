#include <gtest/gtest.h>

#include <libshotscope.hpp>
#include <terrain_interface.hpp>

TEST(TerrainInterfaceTest, FlatTerrainReturnsConstantHeight)
{
    GroundSurface surface;
    surface.height = 10.0F;

    FlatTerrain terrain(surface);

    EXPECT_NEAR(terrain.getHeight(0.0F, 0.0F), 10.0F, 0.001F);
    EXPECT_NEAR(terrain.getHeight(100.0F, 200.0F), 10.0F, 0.001F);
    EXPECT_NEAR(terrain.getHeight(-50.0F, -75.0F), 10.0F, 0.001F);
}

TEST(TerrainInterfaceTest, FlatTerrainReturnsVerticalNormal)
{
    GroundSurface surface;
    FlatTerrain terrain(surface);

    Vector3D normal1 = terrain.getNormal(0.0F, 0.0F);
    Vector3D normal2 = terrain.getNormal(100.0F, 200.0F);

    EXPECT_NEAR(normal1[0], 0.0F, 0.001F);
    EXPECT_NEAR(normal1[1], 0.0F, 0.001F);
    EXPECT_NEAR(normal1[2], 1.0F, 0.001F);

    EXPECT_NEAR(normal2[0], 0.0F, 0.001F);
    EXPECT_NEAR(normal2[1], 0.0F, 0.001F);
    EXPECT_NEAR(normal2[2], 1.0F, 0.001F);
}

TEST(TerrainInterfaceTest, FlatTerrainReturnsConstantSurfaceProperties)
{
    GroundSurface surface;
    surface.height = 5.0F;
    surface.restitution = 0.6F;
    surface.frictionStatic = 0.4F;
    surface.frictionDynamic = 0.3F;
    surface.firmness = 0.9F;
    surface.spinRetention = 0.8F;

    FlatTerrain terrain(surface);

    const GroundSurface& props1 = terrain.getSurfaceProperties(0.0F, 0.0F);
    const GroundSurface& props2 = terrain.getSurfaceProperties(50.0F, 100.0F);

    EXPECT_NEAR(props1.height, 5.0F, 0.001F);
    EXPECT_NEAR(props1.restitution, 0.6F, 0.001F);
    EXPECT_NEAR(props1.frictionStatic, 0.4F, 0.001F);
    EXPECT_NEAR(props1.frictionDynamic, 0.3F, 0.001F);
    EXPECT_NEAR(props1.firmness, 0.9F, 0.001F);
    EXPECT_NEAR(props1.spinRetention, 0.8F, 0.001F);

    EXPECT_NEAR(props2.height, 5.0F, 0.001F);
    EXPECT_NEAR(props2.restitution, 0.6F, 0.001F);
}

TEST(TerrainInterfaceTest, FlatTerrainNormalIsUnitLength)
{
    GroundSurface surface;
    FlatTerrain terrain(surface);

    Vector3D normal = terrain.getNormal(0.0F, 0.0F);
    float magnitude = math_utils::magnitude(normal);

    EXPECT_NEAR(magnitude, 1.0F, 0.001F);
}

TEST(TerrainInterfaceTest, FlatTerrainWorksWithDefaultGroundSurface)
{
    GroundSurface surface;  // Default values
    FlatTerrain terrain(surface);

    EXPECT_NEAR(terrain.getHeight(0.0F, 0.0F), 0.0F, 0.001F);

    const GroundSurface& props = terrain.getSurfaceProperties(0.0F, 0.0F);
    EXPECT_NEAR(props.restitution, 0.4F, 0.001F);  // Default COR
}
