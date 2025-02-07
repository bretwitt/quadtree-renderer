#ifndef QUADTREE_WORLD_H
#define QUADTREE_WORLD_H

#include "QuadtreeTile.h"  // Make sure this header is in your include path.
#include <unordered_map>
#include <cmath>
#include <iostream>
#include "GeoTIFFLoader.h"

//---------------------------------------------------------------------
// A helper structure to uniquely identify a tile by its grid indices.
//---------------------------------------------------------------------
struct TileKey {
    int x;
    int y;
    
    bool operator==(const TileKey& other) const {
        return x == other.x && y == other.y;
    }
};

// Provide a hash function for TileKey so it can be used in an unordered_map.
namespace std {
    template <>
    struct hash<TileKey> {
        std::size_t operator()(const TileKey& key) const {
            // Combine the two integers into a single hash value.
            std::size_t hx = std::hash<int>()(key.x);
            std::size_t hy = std::hash<int>()(key.y);
            return hx ^ (hy << 1);
        }
    };
}

//---------------------------------------------------------------------
// QuadtreeWorld
//
// This class spawns and manages terrain tiles based on the camera's
// position. Each tile is a QuadtreeTile that updates its own LOD.
//---------------------------------------------------------------------
class QuadtreeWorld {
public:
    // Constructor.
    // Parameters:
    //   tileSize          - The size of each tile (tiles are square).
    //   viewRangeInTiles  - How many tiles (in each direction) to keep active.
    //   splitThreshold    - Base distance threshold to split nodes.
    //   mergeThreshold    - Base distance threshold to merge nodes.
    QuadtreeWorld(float tileSize, int viewRangeInTiles, float splitThreshold, float mergeThreshold)
        : tileSize(tileSize),
          viewRangeInTiles(viewRangeInTiles),
          splitThreshold(splitThreshold),
          mergeThreshold(mergeThreshold) {

            loader.load("../resources/apollo.TIFF");
          }

    // Destructor: Clean up all allocated tiles.
    ~QuadtreeWorld() {
        for (auto& pair : tiles) {
            delete pair.second;
        }
    }

    // Updates the world based on the camera position.
    // This function creates new tiles if needed, removes tiles that are out of view,
    // and updates the LOD for each active tile.
    void update(float cameraX, float cameraY, float cameraZ) {
        // Determine the grid index of the tile that contains the camera.
        int centerTileX = static_cast<int>(std::floor(cameraX / tileSize));
        int centerTileY = static_cast<int>(std::floor(cameraY / tileSize));

        // Create a temporary set of keys representing the tiles that should be active.
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
                    tiles[key] = new QuadtreeTile<int>(centerPosX, centerPosY, halfSize, halfSize, &loader);
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
            }
            else {
                ++it;
            }
        }

        // Update each active tile's level-of-detail based on the camera's position.
        for (auto& pair : tiles) {
            int subdivisions = 0;
            pair.second->updateLOD(cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold, subdivisions);
        }
    }

    // Returns the total number of active tiles.
    int getTotalTiles() const {
        return static_cast<int>(tiles.size());
    }

    // Aggregates the memory usage of all active tiles in bytes.
    // This implementation assumes that each QuadtreeTile provides a method getMemoryUsage().
    // If not, you can approximate the memory usage by using sizeof(QuadtreeTile<int>)
    // plus any dynamic memory allocations that the tile performs.
    size_t getMemoryUsage() const {
        size_t totalMemory = 0;
        for (const auto& pair : tiles) {
            totalMemory += pair.second->getMemoryUsage();
        }
        totalMemory += + loader.getMemoryUsage();
        return totalMemory;
    }

    // Optionally, aggregate and return all the meshes from each tile's buckets.
    // This may be useful for rendering.
    std::unordered_map<QuadTree<int>*, Mesh> getAllMeshes() {
        std::unordered_map<QuadTree<int>*, Mesh> allMeshes;
        for (auto& pair : tiles) {
            auto tileMeshes = pair.second->getMeshes();
            allMeshes.insert(tileMeshes.begin(), tileMeshes.end());
        }
        return allMeshes;
    }

private:
    float tileSize;              // Size of each square tile.
    int viewRangeInTiles;        // Number of tiles to keep active in each direction.
    float splitThreshold;        // Base threshold for splitting quadtree nodes.
    float mergeThreshold;        // Base threshold for merging quadtree nodes.

    // A mapping from a grid key to a pointer to a QuadtreeTile.
    std::unordered_map<TileKey, QuadtreeTile<int>*> tiles;
    GeoTIFFLoader loader;
};

#endif // QUADTREE_WORLD_H
