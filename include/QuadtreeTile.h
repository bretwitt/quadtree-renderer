#ifndef QUADTREE_TILE_H
#define QUADTREE_TILE_H

#include "Quadtree.h"
#include <cmath>      // For std::sqrt, std::floor
#include <unordered_map>
#include <vector>
#include <iostream>

//----------------------------------------------------------
// A simple Perlin noise implementation in 2D.
// (This is a basic version; for more robust noise consider using an external library.)
//----------------------------------------------------------
namespace Perlin {
    // Permutation table. The same list is repeated twice.
    static int permutation[256] = {
        151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,
        140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,
        247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,
        57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,
        74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,
        60,211,133,230,220,105,92,41,55,46,245,40,244,102,143,54,
        65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,
        200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,
        52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,212,
        207,206,59,227,47,16,58,17,182,189,28,42,223,183,170,213,
        119,248,152,2,44,154,163,70,221,153,101,155,167,43,172,9,
        129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,
        218,246,97,228,251,34,242,193,238,210,144,12,191,179,162,241,
        81,51,145,235,249,14,239,107,49,192,214,31,181,199,106,157,
        184,84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,
        222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
    };

    // Fade function as defined by Ken Perlin. This eases coordinate values
    // so that they will "ease" towards integral values. This ends up smoothing the final output.
    inline float fade(float t) {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    // Linear interpolation function.
    inline float lerp(float t, float a, float b) {
        return a + t * (b - a);
    }

    // Gradient function. Convert lower 4 bits of hash code into 12 gradient directions.
    inline float grad(int hash, float x, float y) {
        int h = hash & 7;      // Convert low 3 bits of hash into 8 directions
        float u = h < 4 ? x : y;
        float v = h < 4 ? y : x;
        return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
    }

    // 2D Perlin noise. Returns a noise value in roughly the range [-1, 1].
    inline float noise(float x, float y) {
        int X = static_cast<int>(std::floor(x)) & 255;
        int Y = static_cast<int>(std::floor(y)) & 255;

        x = x - std::floor(x);
        y = y - std::floor(y);

        float u = fade(x);
        float v = fade(y);

        int A = permutation[X] + Y;
        int B = permutation[(X + 1) & 255] + Y;

        float res = lerp(v,
                         lerp(u, grad(permutation[A & 255], x, y),
                                 grad(permutation[B & 255], x - 1, y)),
                         lerp(u, grad(permutation[(A + 1) & 255], x, y - 1),
                                 grad(permutation[(B + 1) & 255], x - 1, y - 1)));
        return res;
    }
} // namespace Perlin

//----------------------------------------------------------
// Mesh structure and QuadtreeTile definition
//----------------------------------------------------------

struct Mesh {
    std::vector<float> vertices; // Each vertex: x, y, z
    std::vector<int> indices;
};

template<typename T>
class QuadtreeTile {
public:
    // Constructor: creates a quadtree tile with a given boundary and type.
    // Parameters:
    //   x, y: Center of the tile.
    //   width, height: Half-dimensions of the tile.
    //   capacity: Maximum number of subdivisions before stopping.
    //   type: The data type to store.
    QuadtreeTile(float x, float y, float width, float height)
    {
        tree = new QuadTree<int>(x, y, width, height);
        tree->bucketInitializedCallback = [this](QuadTree<int>* node) {
            this->onNewBucket(node);
        };
        tree->bucketUnloadCallback = [this](QuadTree<int>* node) {
            this->onUnloadBucket(node);
        };
    }

    // Destructor: cleans up the allocated QuadTree.
    ~QuadtreeTile() {
        delete tree;
    }

    // Public method to update the LOD based on the camera position.
    // Parameters:
    //   cameraX, cameraY, cameraZ: The camera's position.
    //   splitThreshold: Distance threshold below which nodes should subdivide.
    //   mergeThreshold: Distance threshold above which nodes should merge.
    void updateLOD(float cameraX, float cameraY, float cameraZ, float splitThreshold, float mergeThreshold, int& subdivisions) {
        updateLODRec(tree, cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold, subdivisions);
    }

    QuadTree<T>* getTree() const { return tree; }

    std::unordered_map<QuadTree<int>*, Mesh> getMeshes() {
        return bucketMeshes;
    }

private:
    // Recursive function to update level-of-detail.
    void updateLODRec(QuadTree<T>* node,
                      float cameraX, float cameraY, float cameraZ,
                      float splitThreshold, float mergeThreshold,
                      int& subdivisions)
    {
        // Retrieve the node's boundary.
        typename QuadTree<T>::QuadBoundary boundary = node->getBoundary();

        float left   = boundary.x - boundary.width;
        float right  = boundary.x + boundary.width;
        float top    = boundary.y - boundary.height;
        float bottom = boundary.y + boundary.height;

        // Compute horizontal (dx) and vertical (dy) distances from the camera to the boundary.
        float dx = 0.0f;
        if (cameraX < left)
            dx = left - cameraX;
        else if (cameraX > right)
            dx = cameraX - right;

        float dy = 0.0f;
        if (cameraY < top)
            dy = top - cameraY;
        else if (cameraY > bottom)
            dy = cameraY - bottom;
        
        float dz = 0.0f - cameraZ;  // Assuming terrain is at z=0

        float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        int level = node->getLevel();
        float effectiveSplitThreshold = splitThreshold / (level + 1);
        float effectiveMergeThreshold = mergeThreshold / (level + 1);

        if (distance < effectiveSplitThreshold && level < 4) {  
            if (!node->isDivided()) {
                node->subdivide();
                subdivisions++; 
            }
            if (node->isDivided()) {
                updateLODRec(node->getNortheastNonConst(), cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold, subdivisions);
                updateLODRec(node->getNorthwestNonConst(), cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold, subdivisions);
                updateLODRec(node->getSoutheastNonConst(), cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold, subdivisions);
                updateLODRec(node->getSouthwestNonConst(), cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold, subdivisions);
            }
        }
        else if (distance > effectiveMergeThreshold) {
            if (node->isDivided()) {
                node->merge();
            }
        }
    }

    // Called when a new bucket (node) is created.
    void onNewBucket(QuadTree<int>* node) {
        std::cout << "New bucket created at level " << node->getLevel() << std::endl;
        int level = node->getLevel();
        typename QuadTree<int>::QuadBoundary bounds = node->getBoundary();

        Mesh mesh = generateTriangularMesh(bounds.x, bounds.y, bounds.width, bounds.height, level);
        std::cout << "Mesh has " << mesh.vertices.size() / 3 << " vertices and " << mesh.indices.size() / 3 << " triangles." << std::endl;
        bucketMeshes[node] = mesh;
    }

    void onUnloadBucket(QuadTree<int>* node) {
        std::cout << "Bucket unloaded at level " << node->getLevel() << std::endl;
        auto it = bucketMeshes.find(node);
        if (it != bucketMeshes.end()) {
            bucketMeshes.erase(it);
            std::cout << "Mesh for bucket at level " << node->getLevel() << " removed." << std::endl;
        }
    }

    Mesh generateTriangularMesh(float centerX, float centerY, float halfWidth, float halfHeight, int level) {
        Mesh mesh;

        int divisions = 1 << (level + 1); // 2^(level + 1)
        int numVerticesPerSide = divisions + 1;

        float stepX = (2.0f * halfWidth) / divisions;
        float stepY = (2.0f * halfHeight) / divisions;

        float startX = centerX - halfWidth;
        float startY = centerY - halfHeight;

        float frequency = 0.1f; 
        float amplitude = 1.0f; 

        for (int j = 0; j < numVerticesPerSide; ++j) {
            for (int i = 0; i < numVerticesPerSide; ++i) {
                float x = startX + i * stepX;
                float y = startY + j * stepY;
                float noiseValue = Perlin::noise(x * frequency, y * frequency) * amplitude;
                mesh.vertices.push_back(x);
                mesh.vertices.push_back(y);
                mesh.vertices.push_back(noiseValue);
            }
        }

        for (int j = 0; j < divisions; ++j) {
            for (int i = 0; i < divisions; ++i) {
                int topLeft = j * numVerticesPerSide + i;
                int topRight = topLeft + 1;
                int bottomLeft = (j + 1) * numVerticesPerSide + i;
                int bottomRight = bottomLeft + 1;

                // First triangle.
                mesh.indices.push_back(topLeft);
                mesh.indices.push_back(bottomLeft);
                mesh.indices.push_back(topRight);

                // Second triangle.
                mesh.indices.push_back(topRight);
                mesh.indices.push_back(bottomLeft);
                mesh.indices.push_back(bottomRight);
            }
        }

        return mesh;
    }

    // Pointer to the underlying QuadTree.
    QuadTree<T>* tree;
    std::unordered_map<QuadTree<int>*, Mesh> bucketMeshes;
};

#endif // QUADTREE_TILE_H
