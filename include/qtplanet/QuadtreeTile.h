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

    //----------------------------------------------------------
    // Mesh structure and QuadtreeTile declaration
    //----------------------------------------------------------

    struct Mesh {
        std::vector<float> vertices;  // position data, 3 floats per vertex
        std::vector<float> normals;   // normal data, 3 floats per vertex
        std::vector<float> texCoords; // texture coords, 2 floats per vertex
        std::vector<float> coarseNormals; // texture coords, 2 floats per vertex
        std::vector<unsigned int> indices;
    };

    struct vec3 {
        vec3(float x, float y, float z) {
            this->x = x;
            this->y = y;
            this->z = z;
        }
        float x = 0;
        float y = 0;
        float z = 0;
    };

    struct TileMetadata {
        int ticksSinceSplit = 0; // -1 if expired/never happened
        int ticksSinceMerge = 0; // -1 if expired/never happened
        std::vector<vec3> dirtyVertices; // Deformations at highest LOD level
        bool dirtyVerticesTransferred = false;
    };

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
        QuadtreeTile(Cartesian::Boundary b, GeoTIFFLoader* geoLoader = nullptr);
        
        // Destructor: cleans up the allocated QuadTree.
        ~QuadtreeTile();

        void updateLOD(float cameraX, float cameraY, float cameraZ,
                    float splitThreshold, float mergeThreshold, int& subdivisions);

        void tick();
        void tickLeaves(QuadTree<TileMetadata>* node);

        template<typename CoordSystem>
        void deformVertex(typename CoordSystem::Position pos, float dz)
        {
            QuadTree<TileMetadata>* leaf = findLeafNode(tree, pos);
            if (!leaf) {
                return;
            }
            
            // Retrieve the tile’s metadata.
            TileMetadata* metadata = leaf->getType();
            
            // Instead of initializing a full grid from the mesh,
            // we simply check whether there is already a deformed point near (x,y).
            const int level = leaf->getLevel();
            const float baseThreshold = 0.05f; // adjust this base value as needed
            bool updated = false;
            for (auto& dp : metadata->dirtyVertices) {
                float dist = CoordinateTraits<Cartesian>::distance({dp.x,dp.y}, pos);
                if (dist < baseThreshold) {
                    dp.z += dz;
                    updated = true;
                    break;
                }
            }
            
            if (!updated) {
                // No existing dirty point is nearby, so add a new one.
                vec3 newPoint{pos.x,pos.y,computeBaseElevation(pos)+dz};
                metadata->dirtyVertices.push_back(newPoint);
            }

            updateMesh(leaf);   
        }

        QuadTree<TileMetadata>* getTree() const;
        std::unordered_map<QuadTree<TileMetadata>*, Mesh> getMeshes();
        size_t getMemoryUsage() const;

        /**
        * Retrieves the elevation at a given (x, y) coordinate.
        *
        * If a GeoTIFFLoader is provided, it converts world coordinates (x, y) to pixel
        * coordinates using the geotransform and returns the elevation from the raster data.
        * Otherwise, it falls back to generating elevation using Perlin noise.
        */
        float getElevation(Cartesian::Position pos);
        float computeBaseElevation(Cartesian::Position pos);
        void deduplicateVertices(std::vector<vec3>& vertices);

        //std::unordered_set<QuadTree<TileMetadata>*> getDirtyTiles() { return dirtyTiles; }

    private:
        // Recursive function to update level-of-detail.
        void updateLODRec(QuadTree<TileMetadata>* node,
                        float cameraX, float cameraY, float cameraZ,
                        float splitThreshold, float mergeThreshold,
                        int& subdivisions);

        // Called when a new bucket (node) is created.
        void onNewBucket(QuadTree<TileMetadata>* node);
        void onSplit(QuadTree<TileMetadata>* parent);
        void onMerge(QuadTree<TileMetadata>* node);

        // Called when a bucket (node) is unloaded.
        void onUnloadBucket(QuadTree<TileMetadata>* node);


        template<typename CoordSystem>
        void updateMesh(QuadTree<TileMetadata, CoordSystem>* node);


        // Search down from any CoordSystem–typed tree to find the leaf containing pos.
        // template<typename CoordSystem>
        // QuadTree<TileMetadata, CoordSystem>* findLeafNode( QuadTree<TileMetadata, CoordSystem>* node,
        //            typename CoordinateTraits<CoordSystem>::Position pos );

        template<typename CoordSystem>
        QuadTree<TileMetadata, CoordSystem>* findLeafNode( 
                        QuadTree<TileMetadata, CoordSystem>* node,
                        typename CoordSystem::Position pos ) 
        {
            if (!node)
                return nullptr;

            // Get the node’s boundary.
            QuadTree<TileMetadata>::Boundary boundary = node->getBoundary();
            float left   = boundary.x - boundary.width;
            float right  = boundary.x + boundary.width;
            float top    = boundary.y - boundary.height;
            float bottom = boundary.y + boundary.height;

            // If (x,y) lies outside this node, return nullptr.
            if (pos.x < left || pos.x > right || pos.y < top || pos.y > bottom)
                return nullptr;

            // If not divided, this is the leaf.
            if (!node->isDivided())
                return node;

            // Otherwise, search the children.
            QuadTree<TileMetadata>* found = findLeafNode<Cartesian>(node->getNortheastNonConst(), pos);
            if (found) return found;
            found = findLeafNode(node->getNorthwestNonConst(), pos);
            if (found) return found;
            found = findLeafNode(node->getSoutheastNonConst(), pos);
            if (found) return found;
            return findLeafNode(node->getSouthwestNonConst(), pos);
        }

        /**
        * Generates a triangular mesh for the tile.
        *
        * The elevation for each vertex is computed using either the GeoTIFF data
        * (if available) or Perlin noise.
        */
        template<typename CoordSystem>
        Mesh generateTriangularMesh(typename CoordSystem::Boundary bounds, int level)
        {
            Mesh mesh;

            int divisions = 1 << (level + 1);
            int numVerticesPerSide = divisions + 1;

            for (int j = 0; j < numVerticesPerSide; ++j) {
                for (int i = 0; i < numVerticesPerSide; ++i) {
                    auto [x, y] = CoordinateTraits<CoordSystem>::cartesianAt(bounds, i, j, divisions);
                    float z = getElevation({x, y}); //TODO: this is dumb, wont work on polar

                    mesh.vertices.push_back(x);
                    mesh.vertices.push_back(y);
                    mesh.vertices.push_back(z);

                    float u = static_cast<float>(i) / divisions;
                    float v = static_cast<float>(j) / divisions;
                    mesh.texCoords.push_back(u);
                    mesh.texCoords.push_back(v);
                }
            }

            for (int j = 0; j < divisions; ++j) {
                for (int i = 0; i < divisions; ++i) {
                    int topLeft     =  j      * numVerticesPerSide + i;
                    int topRight    =  topLeft + 1;
                    int bottomLeft  = (j + 1) * numVerticesPerSide + i;
                    int bottomRight =  bottomLeft + 1;

                    mesh.indices.push_back(topLeft);
                    mesh.indices.push_back(bottomLeft);
                    mesh.indices.push_back(topRight);

                    mesh.indices.push_back(topRight);
                    mesh.indices.push_back(bottomLeft);
                    mesh.indices.push_back(bottomRight);
                }
            }

            calculateNormals(mesh);
            return mesh;
        }
        // Cross product helper.
        static inline void cross(const float* a, const float* b, float* result);

        // Normalize helper.
        static inline void normalize(float* v);

        void calculateNormals(Mesh& mesh);

        // Pointer to the underlying QuadTree.

        QuadTree<TileMetadata, Cartesian>* tree;
        // Mapping from a quadtree node to its mesh.
        std::unordered_map<QuadTree<TileMetadata>*, Mesh> bucketMeshes;
        // Optional pointer to a GeoTIFFLoader for elevation data.
        GeoTIFFLoader* geoTIFFLoader;

        std::unordered_set<QuadTree<TileMetadata>*> dirtyTiles;
    };

    #endif // QUADTREE_TILE_H
