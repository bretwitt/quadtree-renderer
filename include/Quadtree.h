#ifndef QUADTREE_H
#define QUADTREE_H

#include <iostream>

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
    QuadTree(float x, float y, float width, float height, int capacity = 1, T type = T{})
        : boundary{ x, y, width, height },
          capacity(capacity),
          type(type),
          divided(false),
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
        float x, y;  // Center of the rectangle.
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
    void subdivide() {
        if (divided) return;
        
        float x = boundary.x;
        float y = boundary.y;
        float w = boundary.width / 2.0f;
        float h = boundary.height / 2.0f;

        northeast = new QuadTree(x + w, y - h, w, h, capacity, type);
        northwest = new QuadTree(x - w, y - h, w, h, capacity, type);
        southeast = new QuadTree(x + w, y + h, w, h, capacity, type);
        southwest = new QuadTree(x - w, y + h, w, h, capacity, type);
        
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

private:
    // The boundary of this node.
    QuadBoundary boundary;
    // Maximum number of subdivisions before stopping.
    int capacity;
    // The type stored in this bucket.
    T type;
    // Flag to indicate whether this node has been subdivided.
    bool divided;
    // Pointers to the four child nodes.
    QuadTree* northeast;
    QuadTree* northwest;
    QuadTree* southeast;
    QuadTree* southwest;
};

#endif // QUADTREE_H
