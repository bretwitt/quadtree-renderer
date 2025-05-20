#ifndef QUADTREE_WORLD_H
#define QUADTREE_WORLD_H

#include "QuadtreeTile.h"  // Ensure this header is in your include path.
#include "GeoTIFFLoader.h"
#include <unordered_map>
#include <cmath>
#include <iostream>

// Forward declarations for types used in getAllMeshes().
// Make sure these types (QuadTree and Mesh) are defined elsewhere.
template <typename T, typename CoordSystem>
class QuadTree; 

class Mesh;


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
    QuadtreeWorld(float tileSize, int viewRangeInTiles, float splitThreshold, float mergeThreshold);

    // Destructor: Clean up all allocated tiles.
    ~QuadtreeWorld();

    // Updates the world based on the camera position.
    // This function creates new tiles if needed, removes tiles that are out of view,
    // and updates the LOD and ticks for each active tile.
    void update(float cameraX, float cameraY, float cameraZ);

    // Returns the total number of active tiles.
    int getTotalTiles() const;

    // Aggregates the memory usage of all active tiles in bytes.
    size_t getMemoryUsage() const;

    // Aggregates and returns all the meshes from each tile's buckets.
    std::unordered_map<QuadTree<TileMetadata,Spherical>*, Mesh> getAllMeshes();

    float getElevation(float x, float y);
    std::unordered_map<TileKey, QuadtreeTile<Spherical>*> getTiles() { return tiles; };


    void deformVertex(float x, float y, float dz);
private:
    float tileSize;              // Size of each square tile.
    int viewRangeInTiles;        // Number of tiles to keep active in each direction.
    float splitThreshold;        // Base threshold for splitting quadtree nodes.
    float mergeThreshold;        // Base threshold for merging quadtree nodes.

    // A mapping from a grid key to a pointer to a QuadtreeTile.
    std::unordered_map<TileKey, QuadtreeTile<Spherical>*> tiles;
    //std::unordered_set<QuadtreeTile*> dirtyTiles;

    GeoTIFFLoader loader;
};

#endif // QUADTREE_WORLD_H
