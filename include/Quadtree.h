#ifndef QUADTREE_H
#define QUADTREE_H

#include <iostream>
#include <cmath>         // For std::sqrt
#include <functional>    // For std::function

using std::cout;
using std::endl;

// The QuadTree class represents a hierarchical spatial subdivision.
// Each bucket stores a specific type.
template<typename T>
class QuadTree {
public:
    using BucketCallback = std::function<void(QuadTree<T>*)>;

    BucketCallback bucketInitializedCallback;
    
    // Constructs a quadtree node.
    // Parameters:
    //   x, y: Center of the node's boundary.
    //   width, height: Half-dimensions of the boundary.
    //   type: The data type to store.
    //   level: The current level of this node (0 for root).
    QuadTree(float x, float y, float width, float height, T type = T{}, int level = 0, BucketCallback bucketInit = nullptr)
        : boundary{ x, y, width, height },
          type(type),
          divided(false),
          level(level),
          bucketInitializedCallback(bucketInit)
    {
        if (bucketInitializedCallback) {
            bucketInitializedCallback(this);
        }

        northeast = nullptr;
        northwest = nullptr;
        southeast = nullptr;
        southwest = nullptr; 

        // cout << "Initialized tile @ LVL " << level << endl;
    }

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
        cout << "setScale" << endl;
        adjustScale(targetDepth, 0);
    }

    void adjustScale(int targetDepth, int currentDepth) {
        // cout << "SET " << targetDepth << " " << currentDepth << endl;

        if (currentDepth < targetDepth) {
            if (!divided) {
                subdivide();
            }
            int nextDepth = currentDepth + 1;
            northeast->adjustScale(targetDepth, nextDepth);
            northwest->adjustScale(targetDepth, nextDepth);
            southeast->adjustScale(targetDepth, nextDepth);
            southwest->adjustScale(targetDepth, nextDepth);
        } else if (currentDepth >= targetDepth) {
            if (divided) {
                merge();
            }
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

        // cout << "lvl " << level << endl;
        int childLevel = level + 1;
        // Create children with inherited type and increased level.
        northeast = new QuadTree(x + w, y - h, w, h, type, childLevel, bucketInitializedCallback);
        northwest = new QuadTree(x - w, y - h, w, h, type, childLevel, bucketInitializedCallback);
        southeast = new QuadTree(x + w, y + h, w, h, type, childLevel, bucketInitializedCallback);
        southwest = new QuadTree(x - w, y + h, w, h, type, childLevel, bucketInitializedCallback);
        
        divided = true;
    }
    
    // Merges child nodes back into the parent.
    void merge() {
        if (!divided) return;

        // cout << "merge at level " << level << endl;
        // cout << "Current node: " << this << endl;
        // cout << "northeast: " << northeast << endl;
        // cout << "northwest: " << northwest << endl;
        // cout << "southeast: " << southeast << endl;
        // cout << "southwest: " << southwest << endl;

        // Delete all children
        delete northeast;
        delete northwest;
        delete southeast;
        delete southwest;
        
        northeast = northwest = southeast = southwest = nullptr;
        divided = false;
    }

    int getLevel() const { return level; }
    QuadTree* getNortheastNonConst() { return northeast; }
    QuadTree* getNorthwestNonConst() { return northwest; }
    QuadTree* getSoutheastNonConst() { return southeast; }
    QuadTree* getSouthwestNonConst() { return southwest; }

private:
    QuadBoundary boundary;
    T type;
    bool divided = false;
    int level;
    QuadTree* northeast;
    QuadTree* northwest;
    QuadTree* southeast;
    QuadTree* southwest;
};

#endif // QUADTREE_H
