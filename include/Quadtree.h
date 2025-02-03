#ifndef QUADTREE_H
#define QUADTREE_H

#include <iostream>
#include <cmath>  // For std::sqrt

using namespace std;

// The QuadTree class represents a hierarchical spatial subdivision.
// Each bucket stores a specific type.
template<typename T>
class QuadTree {
public:
    // Constructs a quadtree node.
    // Parameters:
    //   x, y: Center of the node's boundary.
    //   width, height: Half-dimensions of the boundary.
    //   capacity: Maximum number of subdivisions before stopping.
    //   type: The data type to store.
    //   level: The current level of this node (0 for root).
    QuadTree(float x, float y, float width, float height, int capacity = 1, T type = T{}, int level = 0)
        : boundary{ x, y, width, height },
          capacity(capacity),
          type(type),
          divided(false),
          level(level),
          northeast(nullptr),
          northwest(nullptr),
          southeast(nullptr),
          southwest(nullptr) {}

    // Destructor: recursively deletes any child nodes.
    ~QuadTree() {
        delete northeast;
        delete northwest;
        delete southeast;
        delete southwest;
    }

    // A simple structure representing the rectangular boundary.
    struct QuadBoundary {
        float x, y;   // Center of the rectangle.
        float width;  // Half-width.
        float height; // Half-height.
    };

    QuadBoundary getBoundary() const {
        return boundary;
    }

    bool isDivided() const { return divided; }

    const QuadTree* getNortheast() const { return northeast; }
    const QuadTree* getNorthwest() const { return northwest; }
    const QuadTree* getSoutheast() const { return southeast; }
    const QuadTree* getSouthwest() const { return southwest; }

    T getType() const { return type; }
    void setType(T newType) { type = newType; }

    // Sets the scale (or depth) of the quadtree uniformly.
    void setScale(int targetDepth) {
        adjustScale(targetDepth, 0);
    }

    void adjustScale(int targetDepth, int currentDepth) {
        if (currentDepth < targetDepth) {
            if (!divided) {
                subdivide();
            }
            northeast->adjustScale(targetDepth, currentDepth + 1);
            northwest->adjustScale(targetDepth, currentDepth + 1);
            southeast->adjustScale(targetDepth, currentDepth + 1);
            southwest->adjustScale(targetDepth, currentDepth + 1);
        } else {
            merge();
        }
    }

    // Subdivides the current node into four child quadrants.
    // Each child gets level = parent level + 1.
    void subdivide() {
        if (divided) return;
        
        float x = boundary.x;
        float y = boundary.y;
        float w = boundary.width / 2.0f;
        float h = boundary.height / 2.0f;

        // Create children with inherited type and increased level.
        northeast = new QuadTree(x + w, y - h, w, h, capacity, type, level + 1);
        northwest = new QuadTree(x - w, y - h, w, h, capacity, type, level + 1);
        southeast = new QuadTree(x + w, y + h, w, h, capacity, type, level + 1);
        southwest = new QuadTree(x - w, y + h, w, h, capacity, type, level + 1);
        
        divided = true;
    }
    
    // Merges child nodes back into the parent if possible.
    void merge() {
        if (!divided) return;
        
        delete northeast;
        delete northwest;
        delete southeast;
        delete southwest;
        
        northeast = northwest = southeast = southwest = nullptr;
        divided = false;
    }

    void updateLOD(float cameraX, float cameraY, float cameraZ, float splitThreshold, float mergeThreshold, int& subdivisions) {
        float dx = boundary.x - cameraX;
        float dy = boundary.y - cameraY;
        float dz = 0 - cameraZ;
        float distance = std::sqrt(dx * dx + dy * dy);

        float effectiveSplitThreshold = splitThreshold / (level + 1);
        float effectiveMergeThreshold = mergeThreshold / (level + 1);

        if (distance < effectiveSplitThreshold && level < 5.0f) {
            if (!divided) {
                subdivide();
                subdivisions++;
            }
            if (divided) {
                northeast->updateLOD(cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold,subdivisions);
                northwest->updateLOD(cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold,subdivisions);
                southeast->updateLOD(cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold,subdivisions);
                southwest->updateLOD(cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold,subdivisions);
            }
        }
        else if (distance > effectiveMergeThreshold) {
            if (divided) {
                merge();
            }
        }
    }

    int getLevel() const { return level; }

private:
    QuadBoundary boundary;
    int capacity;
    T type;
    bool divided;
    int level;
    QuadTree* northeast;
    QuadTree* northwest;
    QuadTree* southeast;
    QuadTree* southwest;
};

#endif // QUADTREE_H
