/**
 * @file terrain_interface.cpp
 * @author Gabriel DiFiore
 * @brief Implementation of terrain interface adapters.
 *
 * @copyright Copyright (c) 2025, Gabriel DiFiore
 */

#include "terrain_interface.hpp"
#include "GroundProvider.hpp"
#include "physics_constants.hpp"
#include <stdexcept>
#include <cmath>

TerrainProviderAdapter::TerrainProviderAdapter(const GroundProvider& provider)
	: provider_(provider.clone()), cachedSurface_{}
{
	if (!provider_)
	{
		throw std::invalid_argument(std::string(__func__) + ": GroundProvider clone failed");
	}
}

void TerrainProviderAdapter::updateCache(float x, float y) const
{
	if (!cacheValid_ ||
		std::abs(x - cachedX_) > physics_constants::POSITION_EPSILON ||
		std::abs(y - cachedY_) > physics_constants::POSITION_EPSILON)
	{
		cachedSurface_ = provider_->getGroundAt(x, y);
		cachedX_ = x;
		cachedY_ = y;
		cacheValid_ = true;
	}
}

float TerrainProviderAdapter::getHeight(float x, float y) const
{
	// GroundProvider assumes flat terrain, so query the ground height from provider
	updateCache(x, y);
	return cachedSurface_.height;
}

Vector3D TerrainProviderAdapter::getNormal([[maybe_unused]] float x, [[maybe_unused]] float y) const noexcept
{
	// GroundProvider assumes flat terrain, so normal is always vertical
	return {0.0F, 0.0F, 1.0F};
}

const GroundSurface& TerrainProviderAdapter::getSurfaceProperties(float x, float y) const
{
	// Query provider for ground properties at this position (uses cache)
	updateCache(x, y);
	return cachedSurface_;
}
