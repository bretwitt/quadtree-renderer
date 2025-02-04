#ifndef QUADTREE_TILE_H
#define QUADTREE_TILE_H

#include "Quadtree.h"
#include <cmath>  // For std::sqrt

// The QuadtreeTile class wraps a QuadTree instance and provides an updateLOD method.
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
        tree->bucketInitializedCallback = onNewBucket;
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

private:
    void updateLODRec(QuadTree<T>* node,
                      float cameraX, float cameraY, float cameraZ,
                      float splitThreshold, float mergeThreshold,
                      int& subdivisions)
    {
        // Retrieve the node's boundary.
        typename QuadTree<T>::QuadBoundary boundary = node->getBoundary();
        
        float dx = boundary.x - cameraX;
        float dy = boundary.y - cameraY;
        float dz = 0.0f - cameraZ;
        float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        int level = node->getLevel();
        float effectiveSplitThreshold = splitThreshold / (level + 1);
        float effectiveMergeThreshold = mergeThreshold / (level + 1);

        if (distance < effectiveSplitThreshold && level < 5) {  
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

    static void onNewBucket(QuadTree<int>* node) {
        cout << "New bucket created at level " << node->getLevel() << endl;
    }

    // Pointer to the underlying QuadTree.
    QuadTree<T>* tree;
    
};

#endif // QUADTREE_TILE_H
