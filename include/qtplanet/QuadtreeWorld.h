#ifndef QUADTREE_WORLD_H
#define QUADTREE_WORLD_H

#include "QuadtreeTile.h"  // Ensure this header is in your include path.
#include <unordered_map>
#include <cmath>
#include <iostream>
#include "MultiGeoTIFFManager.h"

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
    QuadtreeWorld(double tileSize, int viewRangeInTiles, double splitThreshold, double mergeThreshold);

    // Destructor: Clean up all allocated tiles.
    ~QuadtreeWorld();

    // Updates the world based on the camera position.
    // This function creates new tiles if needed, removes tiles that are out of view,
    // and updates the LOD and ticks for each active tile.
    void update(double cameraX, double cameraY, double cameraZ);

    // Returns the total number of active tiles.
    int getTotalTiles() const;

    // Aggregates the memory usage of all active tiles in bytes.
    size_t getMemoryUsage() const;

    // Aggregates and returns all the meshes from each tile's buckets.
    std::unordered_map<QuadTree<TileMetadata,Spherical>*, Mesh> getAllMeshes();

    double getElevation(double x, double y);
    std::unordered_map<TileKey, QuadtreeTile<Spherical>*> getTiles() { return tiles; };

    void deformVertex(double x, double y, double dz);

    // Returns the camera's latitude and longitude.
    std::pair<double, double> getCameraPositionLonLat() const;

    double getCameraPositionElevation() const;

private:
    double tileSize;              // Size of each square tile.
    int viewRangeInTiles;        // Number of tiles to keep active in each direction.
    double splitThreshold;        // Base threshold for splitting quadtree nodes.
    double mergeThreshold;        // Base threshold for merging quadtree nodes.

    double cameraX;            // Camera's X position.
    double cameraY;            // Camera's Y position.
    double cameraZ;            // Camera's Z position.

    // A mapping from a grid key to a pointer to a QuadtreeTile.
    std::unordered_map<TileKey, QuadtreeTile<Spherical>*> tiles;
    //std::unordered_set<QuadtreeTile*> dirtyTiles;

    GeoTIFFLoader loader;

    std::shared_ptr<MultiGeoTIFFManager> geoMgr;


};

#endif // QUADTREE_WORLD_H
