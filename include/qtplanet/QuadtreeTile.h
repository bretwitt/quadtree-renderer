#ifndef QUADTREE_TILE_H
#define QUADTREE_TILE_H

#include "Quadtree.h"
#include <cmath>      // For std::sqrt, std::floor
#include <unordered_map>
#include <vector>
#include <iostream>
#include "Perlin.h"
#include "GeoTIFFLoader.h"  // Include the GeoTIFF loader
#include <unordered_set>
#include "TileMetadata.h"  // Include the TileMetadata definition

//----------------------------------------------------------
// Mesh structure and QuadtreeTile declaration
//----------------------------------------------------------

struct Mesh {
    std::vector<double> vertices;  // position data, 3 doubles per vertex
    std::vector<double> normals;   // normal data, 3 doubles per vertex
    std::vector<double> texCoords; // texture coords, 2 doubles per vertex
    std::vector<double> coarseNormals; // texture coords, 2 doubles per vertex
    std::vector<unsigned int> indices;
};

template<typename CoordSystem>
class QuadtreeTile {
public:
    /**
    * Constructor.
    *
    * @param x Center x coordinate of the tile.
    * @param y Center y coordinate of the tile.
    * @param width Half-width of the tile.
    * @param height Half-height of the tile.
    * @param geoLoader Optional pointer to a GeoTIFFLoader.
    *
    * If geoLoader is provided, the elevation values will be taken from the GeoTIFF.
    * Otherwise, Perlin noise is used as a fallback.
    */
    using Boundary = typename CoordSystem::Boundary;
    QuadtreeTile(Boundary b, const std::shared_ptr<MultiGeoTIFFManager>& geoLoader = nullptr);
    
    // Destructor: cleans up the allocated QuadTree.
    ~QuadtreeTile();

    void updateLOD(double cameraX, double cameraY, double cameraZ,
                double splitThreshold, double mergeThreshold, int& subdivisions);

    void tick();
    void tickLeaves(QuadTree<TileMetadata,CoordSystem>* node);

    void deformVertex(typename CoordSystem::Position pos, double dz)
    {
        QuadTree<TileMetadata, CoordSystem>* leaf = findLeafNode(tree, pos);
        if (!leaf) {
            return;
        }
        
        // Retrieve the tile’s metadata.
        TileMetadata* metadata = leaf->getType();
        
        // Instead of initializing a full grid from the mesh,
        // we simply check whether there is already a deformed point near (x,y).
        const int level = leaf->getLevel();
        const double baseThreshold = 0.05f; // adjust this base value as needed
        bool updated = false;
        for (auto& dp : metadata->dirtyVertices) {
            double dist = CoordinateTraits<CoordSystem>::distance({dp.x,dp.y}, pos);
            if (dist < baseThreshold) {
                dp.z += dz;
                updated = true;
                break;
            }
        }
        
        if (!updated) {
            // No existing dirty point is nearby, so add a new one.
            vec3 newPoint{pos.asArray()[0],pos.asArray()[1],computeBaseElevation(pos,0)+dz};
            metadata->dirtyVertices.push_back(newPoint);
        }

        updateMesh(leaf);   
    }

    QuadTree<TileMetadata,CoordSystem>* getTree() const;
    std::unordered_map<QuadTree<TileMetadata,CoordSystem>*, Mesh> getMeshes();
    size_t getMemoryUsage() const;

    /**
    * Retrieves the elevation at a given (x, y) coordinate.
    *
    * If a GeoTIFFLoader is provided, it converts world coordinates (x, y) to pixel
    * coordinates using the geotransform and returns the elevation from the raster data.
    * Otherwise, it falls back to generating elevation using Perlin noise.
    */
    double getElevation(typename CoordSystem::Position pos, int zoomLevel);

    double computeBaseElevation(typename CoordSystem::Position pos, int zoomLevel);

    void deduplicateVertices(std::vector<vec3>& vertices);

    //std::unordered_set<QuadTree<TileMetadata>*> getDirtyTiles() { return dirtyTiles; }

private:
    // Recursive function to update level-of-detail.
    void updateLODRec(QuadTree<TileMetadata, CoordSystem>* node,
                    double cameraX, double cameraY, double cameraZ,
                    double splitThreshold, double mergeThreshold,
                    int& subdivisions);

    // Called when a new bucket (node) is created.
    void onNewBucket(QuadTree<TileMetadata, CoordSystem>* node);
    void onSplit(QuadTree<TileMetadata, CoordSystem>* parent);
    void onMerge(QuadTree<TileMetadata, CoordSystem>* node);

    // Called when a bucket (node) is unloaded.
    void onUnloadBucket(QuadTree<TileMetadata,CoordSystem>* node);


    void updateMesh(QuadTree<TileMetadata, CoordSystem>* node);


    // Search down from any CoordSystem–typed tree to find the leaf containing pos.
    QuadTree<TileMetadata, CoordSystem>* findLeafNode( 
                    QuadTree<TileMetadata, CoordSystem>* node,
                    typename CoordSystem::Position pos );

    enum Direction { North, South, East, West };

    QuadTree<TileMetadata,CoordSystem>*  getNeighbor( QuadTree<TileMetadata,CoordSystem>* node, Direction dir);

    /**
    * Generates a triangular mesh for the tile.
    *
    * The elevation for each vertex is computed using either the GeoTIFF data
    * (if available) or Perlin noise.
    */
    // Mesh generateTriangularMesh(QuadTree<TileMetadata,CoordSystem>* node, typename CoordSystem::Boundary bounds, int level);
    Mesh generateTriangularMesh(typename CoordSystem::Boundary bounds, int level);

    // Cross product helper.
    static inline void cross(const double* a, const double* b, double* result);

    // Normalize helper.
    static inline void normalize(double* v);

    // Estimate normals for the mesh.
    void calculateNormals(Mesh& mesh);


    QuadTree<TileMetadata, CoordSystem>* tree;
    // Mapping from a quadtree node to its mesh.
    std::unordered_map<QuadTree<TileMetadata, CoordSystem>*, Mesh> bucketMeshes;
    // Optional pointer to a GeoTIFFLoader for elevation data.
    // GeoTIFFLoader* geoTIFFLoader;
    // Optional pointer to a MultiGeoTIFFManager for elevation data.
    std::shared_ptr<MultiGeoTIFFManager> geoTIFFLoaderPtr;

    std::unordered_set<QuadTree<TileMetadata, CoordSystem>*> dirtyTiles;
};

#endif // QUADTREE_TILE_H
