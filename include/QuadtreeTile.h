#ifndef QUADTREE_TILE_H
#define QUADTREE_TILE_H

#include "Quadtree.h"
#include <cmath>  // For std::sqrt
#include <unordered_map>


struct Mesh {
    std::vector<float> vertices;
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

        // Compute dx: horizontal distance from the camera to the boundary.
        float dx = 0.0f;
        if (cameraX < left)
            dx = left - cameraX;
        else if (cameraX > right)
            dx = cameraX - right;
        // Otherwise, cameraX is within [left, right] and dx remains 0.

        // Compute dy: vertical distance from the camera to the boundary.
        float dy = 0.0f;
        if (cameraY < top)
            dy = top - cameraY;
        else if (cameraY > bottom)
            dy = cameraY - bottom;
        // float dx = boundary.x - cameraX;
        // float dy = boundary.y - cameraY;
        
        float dz = 0.0f - cameraZ;

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

    void onNewBucket(QuadTree<int>* node) {
        cout << "New bucket created at level " << node->getLevel() << endl;
        int level = node->getLevel();
        typename QuadTree<int>::QuadBoundary bounds = node->getBoundary();

        Mesh mesh = generateTriangularMesh(bounds.x, bounds.y, bounds.width, bounds.height, level);
        cout << "Mesh has " << mesh.vertices.size()/2 << " vertices and " << mesh.indices.size()/3 << " triangles." << endl;
        bucketMeshes[node] = mesh;
    }

    void onUnloadBucket(QuadTree<int>* node) {
        cout << "Bucket unloaded at level " << node->getLevel() << std::endl;
        auto it = bucketMeshes.find(node);
        if (it != bucketMeshes.end()) {
            bucketMeshes.erase(it);
            std::cout << "Mesh for bucket at level " << node->getLevel() << " removed." << std::endl;
        }
    }

    Mesh generateTriangularMesh(float centerX, float centerY, float halfWidth, float halfHeight, int level) {
        Mesh mesh;

        int divisions = 1 << (level + 1); // This computes 2^(level + 1).
        int numVerticesPerSide = divisions + 1;

        float stepX = (2.0f * halfWidth) / divisions;
        float stepY = (2.0f * halfHeight) / divisions;

        float startX = centerX - halfWidth;
        float startY = centerY - halfHeight;

        for (int j = 0; j < numVerticesPerSide; ++j) {
            for (int i = 0; i < numVerticesPerSide; ++i) {
                float x = startX + i * stepX;
                float y = startY + j * stepY;
                mesh.vertices.push_back(x);
                mesh.vertices.push_back(y);
                mesh.vertices.push_back(0.0);
            }
        }

        for (int j = 0; j < divisions; ++j) {
            for (int i = 0; i < divisions; ++i) {
                int topLeft = j * numVerticesPerSide + i;
                int topRight = topLeft + 1;
                int bottomLeft = (j + 1) * numVerticesPerSide + i;
                int bottomRight = bottomLeft + 1;

                mesh.indices.push_back(topLeft);
                mesh.indices.push_back(bottomLeft);
                mesh.indices.push_back(topRight);

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
