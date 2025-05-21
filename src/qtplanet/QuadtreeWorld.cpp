#include "qtplanet/QuadtreeWorld.h"
#include <algorithm>
#include <cmath> // For std::fabs and std::ceil

//---------------------------------------------------------------------
// Constructor
//---------------------------------------------------------------------
QuadtreeWorld::QuadtreeWorld(float tileSize, int viewRangeInTiles, float splitThreshold, float mergeThreshold)
    : tileSize(tileSize),
      viewRangeInTiles(viewRangeInTiles),
      splitThreshold(splitThreshold),
      mergeThreshold(mergeThreshold) {
    // Load the terrain data.
    loader.load("../resources/apollo.TIFF");
}

//---------------------------------------------------------------------
// Destructor: Cleans up all allocated tiles.
//---------------------------------------------------------------------
QuadtreeWorld::~QuadtreeWorld() {
    for (auto& pair : tiles) {
        delete pair.second;
    }
}

//---------------------------------------------------------------------
// update
//
// Updates the world based on the camera position. Creates new tiles
// if needed, removes tiles that are out of view, and updates the LOD
// for each active tile.
//---------------------------------------------------------------------

void QuadtreeWorld::update(float cameraX, float cameraY, float cameraZ) {
    
    this->cameraX = cameraX;
    this->cameraY = cameraY;
    this->cameraZ = cameraZ;
    
    const float baseHeight = -0.0f; // Adjust this as needed.
        const int minViewRange = 10;
    const int maxViewRange = 30;
    float dZ = cameraZ - baseHeight;
    int dynamicViewRange = 15;
    // dynamicViewRange = std::max(minViewRange, std::min(maxViewRange, dynamicViewRange));

    auto [ centerTileX, centerTileY ] = CoordinateTraits<Spherical>::computeTileIndices({cameraX, cameraY}, tileSize);

    std::unordered_map<TileKey, bool> neededTiles;
    for (int dy = -dynamicViewRange; dy <= dynamicViewRange; ++dy) {
        for (int dx = -dynamicViewRange; dx <= dynamicViewRange; ++dx) {
            int tileX = centerTileX + dx;
            int tileY = centerTileY + dy;
            TileKey key{ tileX, tileY };
            neededTiles[key] = true;

            // If this tile does not exist yet, create and initialize it.
            if (tiles.find(key) == tiles.end()) {
                auto [ centerPosX, centerPosY ] = CoordinateTraits<Spherical>::tileCenterPosition(key, tileSize);
                float halfSize = tileSize * 0.5f;
                tiles[key] = new QuadtreeTile<Spherical>({centerPosX, centerPosY, halfSize, halfSize}, &loader);
            }
        }
    }

    // Remove tiles that are no longer within the view range.
    for (auto it = tiles.begin(); it != tiles.end(); ) {
        if (neededTiles.find(it->first) == neededTiles.end()) {
            delete it->second;
            it = tiles.erase(it);
        } else {
            ++it;
        }
    }

    // Update each active tile's LOD based on the camera's position.
    for (auto& pair : tiles) {
        int subdivisions = 0;
        pair.second->updateLOD(cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold, subdivisions);
    }

    // Update ticks for each tile.
    for (auto& pair : tiles) {
        pair.second->tick();
    }
}


//---------------------------------------------------------------------
// getTotalTiles
//
// Returns the total number of active tiles.
//---------------------------------------------------------------------
int QuadtreeWorld::getTotalTiles() const {
    return static_cast<int>(tiles.size());
}

//---------------------------------------------------------------------
// getMemoryUsage
//
// Aggregates the memory usage of all active tiles in bytes.
//---------------------------------------------------------------------
size_t QuadtreeWorld::getMemoryUsage() const {
    size_t totalMemory = 0;
    for (const auto& pair : tiles) {
        totalMemory += pair.second->getMemoryUsage();
    }
    totalMemory += loader.getMemoryUsage();
    return totalMemory;
}

//---------------------------------------------------------------------
// getAllMeshes
//
// Aggregates and returns all the meshes from each tile's buckets.
//---------------------------------------------------------------------
std::unordered_map<QuadTree<TileMetadata,Spherical>*, Mesh> QuadtreeWorld::getAllMeshes() {
    std::unordered_map<QuadTree<TileMetadata,Spherical>*, Mesh> allMeshes;
    for (auto& pair : tiles) {
        auto tileMeshes = pair.second->getMeshes();
        allMeshes.insert(tileMeshes.begin(), tileMeshes.end());
    }
    return allMeshes;
}

void QuadtreeWorld::deformVertex(float x, float y, float dz) {
    auto [ tileX, tileY ] = CoordinateTraits<Spherical>::computeTileIndices({x,y}, tileSize);
    TileKey key{ tileX, tileY };

    auto it = tiles.find(key);
    if (it != tiles.end()) {
        it->second->deformVertex({x,y}, dz);
    } else {
        return;
    }
}

float QuadtreeWorld::getElevation(float x, float y) {
    // Compute the grid indices for the tile that should cover (x,y).
    int tileX = static_cast<int>(std::floor(x / tileSize));
    int tileY = static_cast<int>(std::floor(y / tileSize));
    TileKey key{ tileX, tileY };
    
    // Look up the tile in the active tiles map.
    auto it = tiles.find(key);
    if (it != tiles.end()) {
        // Forward the deformation to the tile.
        return it->second->getElevation({x,y});
    } else {
        return -1;
    }
}

std::pair<float, float> QuadtreeWorld::getCameraPositionLonLat() const {
    auto [ lon, lat ] = CoordinateTraits<Spherical>::projectXYZToLatLon(cameraX, cameraY, cameraZ);
    return { lon, lat };
}