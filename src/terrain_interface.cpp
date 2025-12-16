/**
 * @file terrain_interface.cpp
 * @author Gabriel DiFiore
 * @brief Implementation of terrain interface adapters.
 *
 * @copyright Copyright (c) 2025, Gabriel DiFiore
 */

#include "terrain_interface.hpp"
#include "GroundProvider.hpp"
#include <stdexcept>

TerrainProviderAdapter::TerrainProviderAdapter(const GroundProvider* provider)
	: provider_(provider ? provider->clone() : nullptr), cachedSurface_{}
{
	if (!provider)
	{
		throw std::invalid_argument("GroundProvider pointer must not be null");
	}
}

void TerrainProviderAdapter::updateCache(float x, float y) const
{
	if (!cacheValid_ || x != cachedX_ || y != cachedY_)
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

Vector3D TerrainProviderAdapter::getNormal(float x, float y) const
{
	(void)x;
	(void)y;
	// GroundProvider assumes flat terrain, so normal is always vertical
	return {0.0F, 0.0F, 1.0F};
}

const GroundSurface& TerrainProviderAdapter::getSurfaceProperties(float x, float y) const
{
	// Query provider for ground properties at this position (uses cache)
	updateCache(x, y);
	return cachedSurface_;
}
