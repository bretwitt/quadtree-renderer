#include "QuadtreeWorld.h"


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
    // Determine the grid index of the tile that contains the camera.
    int centerTileX = static_cast<int>(std::floor(cameraX / tileSize));
    int centerTileY = static_cast<int>(std::floor(cameraY / tileSize));

    // Create a temporary map representing the tiles that should be active.
    std::unordered_map<TileKey, bool> neededTiles;

    // Loop over a square region around the camera.
    for (int dy = -viewRangeInTiles; dy <= viewRangeInTiles; ++dy) {
        for (int dx = -viewRangeInTiles; dx <= viewRangeInTiles; ++dx) {
            int tileX = centerTileX + dx;
            int tileY = centerTileY + dy;
            TileKey key{ tileX, tileY };
            neededTiles[key] = true;

            // If this tile does not exist yet, create and initialize it.
            if (tiles.find(key) == tiles.end()) {
                // Compute the center position of the tile.
                float centerPosX = tileX * tileSize + tileSize * 0.5f;
                float centerPosY = tileY * tileSize + tileSize * 0.5f;
                float halfSize = tileSize * 0.5f;
                tiles[key] = new QuadtreeTile(centerPosX, centerPosY, halfSize, halfSize, &loader);
                std::cout << "Tile created at grid (" << tileX << ", " << tileY << ")\n";
            }
        }
    }

    // Remove tiles that are no longer within the view range.
    for (auto it = tiles.begin(); it != tiles.end(); ) {
        if (neededTiles.find(it->first) == neededTiles.end()) {
            std::cout << "Tile removed at grid (" << it->first.x << ", " << it->first.y << ")\n";
            delete it->second;
            it = tiles.erase(it);
        } else {
            ++it;
        }
    }

    // Update each active tile's level-of-detail based on the camera's position.
    for (auto& pair : tiles) {
        int subdivisions = 0;
        pair.second->updateLOD(cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold, subdivisions);
    }

    // Update ticks
    for (auto& pair : tiles) {
        QuadtreeTile* tile = pair.second;
        tile->tick();
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
std::unordered_map<QuadTree<TileMetadata>*, Mesh> QuadtreeWorld::getAllMeshes() {
    std::unordered_map<QuadTree<TileMetadata>*, Mesh> allMeshes;
    for (auto& pair : tiles) {
        auto tileMeshes = pair.second->getMeshes();
        allMeshes.insert(tileMeshes.begin(), tileMeshes.end());
    }
    return allMeshes;
}
