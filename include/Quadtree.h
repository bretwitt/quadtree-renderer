#ifndef QUADTREE_H
#define QUADTREE_H

#include <vector>
#include <cstddef>
#include <iostream>

using namespace std;
// A simple structure to represent a 2D point with an associated data item.
template<typename T>
struct Point2D {
    float x;  // x-coordinate
    float y;  // y-coordinate
    T data;   // Additional data
};

// The QuadTree class stores points within a rectangular region.
// For this implementation, the rectangle is defined by its center (x, y)
// and half-dimensions (width and height). That is, the full width is 2*width.
template<typename T>
class QuadTree {
public:
    // Constructs a quadtree node.
    // Parameters:
    //   x, y: Center of the node's boundary.
    //   width, height: Half-dimensions of the boundary.
    //   capacity: Maximum number of points a node can hold before subdividing.
    QuadTree(float x, float y, float width, float height, int capacity = 1)
        : boundary{ x, y, width, height },
          capacity(capacity),
          divided(false),
          northeast(nullptr),
          northwest(nullptr),
          southeast(nullptr),
          southwest(nullptr)
    {}

    // Destructor: recursively deletes any child nodes.
    ~QuadTree() {
        delete northeast;
        delete northwest;
        delete southeast;
        delete southwest;
    }

    // A simple structure representing the rectangular boundary.
    struct QuadBoundary {
        float x, y;      // Center of the rectangle.
        float width;     // Half-width.
        float height;    // Half-height.
    };

    QuadBoundary getBoundary() const {
        return QuadBoundary{ boundary.x, boundary.y, boundary.width, boundary.height };
    }

    bool isDivided() const { return divided; }

    const QuadTree* getNortheast() const { return northeast; }
    const QuadTree* getNorthwest() const { return northwest; }
    const QuadTree* getSoutheast() const { return southeast; }
    const QuadTree* getSouthwest() const { return southwest; }

    // Inserts a point into the quadtree.
    // Returns true if the point is inserted; false if the point is outside the boundary.
    bool insert(const Point2D<T>& point) {
        // Ignore points that do not belong in this quad tree node
        if (!boundary.contains(point))
            return false;

        std::cout << "Start Insert" << std::endl;

        // If there is room in this node and it is not subdivided, add the point here.
        if (points.size() < static_cast<size_t>(1)) {
            points.push_back(point);
            return true;
        } else {
            // Otherwise, subdivide if necessary and then add the point into whichever child will accept it.
            if (!divided) {
                subdivide();
                std::cout << "Subdiv" << std::endl;
            }
            if (northeast->insert(point)) return true;
            if (northwest->insert(point)) return true;
            if (southeast->insert(point)) return true;
            if (southwest->insert(point)) return true;
        }
        // Should not reach here (the point must fall within one of the children).
        return false;
    }
    
    // Query the quadtree for all points that lie within a given rectangular range.
    // The range is specified by its center (x, y) and half-dimensions (width, height).
    std::vector<Point2D<T>> queryRange(float x, float y, float width, float height) const {
        std::vector<Point2D<T>> found;
        Rect range{ x, y, width, height };
        queryRange(range, found);
        return found;
    }

    void setScale(int targetDepth, int minCapacity) {
        adjustScale(targetDepth, minCapacity, 0);
    }

    void adjustScale(int targetDepth, int minCapacity, int currentDepth) {
        if (currentDepth < targetDepth) {
            if (!divided) {
                subdivide();
            }
            northeast->adjustScale(targetDepth, minCapacity, currentDepth + 1);
            northwest->adjustScale(targetDepth, minCapacity, currentDepth + 1);
            southeast->adjustScale(targetDepth, minCapacity, currentDepth + 1);
            southwest->adjustScale(targetDepth, minCapacity, currentDepth + 1);
        } else {
            merge();
        }
    }
    
    void merge() {
        if (!divided) return;

        std::vector<QuadTree*> children = { northeast, northwest, southeast, southwest };
        int totalPoints = 0;
        bool canMerge = true;

        for (auto child : children) {
            totalPoints += child->points.size();
            if (child->divided) {
                canMerge = false;
            }
        }

        if (canMerge && totalPoints <= capacity) {
            for (auto child : children) {
                points.insert(points.end(), child->points.begin(), child->points.end());
                delete child;
            }
            northeast = northwest = southeast = southwest = nullptr;
            divided = false;
        }
    }

        // Upscales the quadtree by further subdividing all nodes.
    void upscale(int targetDepth, int currentDepth = 0) {
        if (currentDepth >= targetDepth) return;

        if (!divided) {
            subdivide();
        }

        // Recursively upscale the children
        northeast->upscale(targetDepth, currentDepth + 1);
        northwest->upscale(targetDepth, currentDepth + 1);
        southeast->upscale(targetDepth, currentDepth + 1);
        southwest->upscale(targetDepth, currentDepth + 1);
    }

    void downscale(int minCapacity) {
        if (!divided) return;

        // Check if all children can be merged
        bool canMerge = true;
        std::vector<QuadTree*> children = {northeast, northwest, southeast, southwest};
        
        int totalPoints = 0;
        for (auto child : children) {
            totalPoints += child->points.size();
            if (child->divided) {
                canMerge = false; // If any child is further divided, we cannot merge
            }
        }

        if (canMerge && totalPoints <= minCapacity) {
            // Move all points from children to parent
            for (auto child : children) {
                points.insert(points.end(), child->points.begin(), child->points.end());
                delete child;
            }
            // Reset children pointers
            northeast = northwest = southeast = southwest = nullptr;
            divided = false;
        } else {
            // Recursively attempt to downscale the children
            northeast->downscale(minCapacity);
            northwest->downscale(minCapacity);
            southeast->downscale(minCapacity);
            southwest->downscale(minCapacity);
        }
    }

private:
    // Internal rectangle structure representing an axis-aligned bounding box.
    // Here, (x, y) is the center, and width/height represent the half-dimensions.
    struct Rect {
        float x, y;      // Center of the rectangle
        float width;     // Half of the rectangle's full width
        float height;    // Half of the rectangle's full height

        // Returns true if the rectangle contains the given point.
        bool contains(const Point2D<T>& point) const {
            return (point.x >= x - width && point.x <= x + width &&
                    point.y >= y - height && point.y <= y + height);
        }

        // Returns true if this rectangle intersects with another rectangle.
        bool intersects(const Rect& range) const {
            return !(range.x - range.width > x + width ||
                     range.x + range.width < x - width ||
                     range.y - range.height > y + height ||
                     range.y + range.height < y - height);
        }
    };

    // The boundary of this node.
    Rect boundary;
    // Maximum number of points a node can hold before subdividing.
    int capacity;
    // Points contained in this node.
    std::vector<Point2D<T>> points;
    // Flag to indicate whether this node has been subdivided.
    bool divided;
    // Pointers to the four child nodes.
    QuadTree* northeast;
    QuadTree* northwest;
    QuadTree* southeast;
    QuadTree* southwest;

    // Subdivides the current node into four child quadrants.
    void subdivide() {
        // Calculate new half-dimensions for children.
        float x = boundary.x;
        float y = boundary.y;
        float w = boundary.width / 2.0f;
        float h = boundary.height / 2.0f;

        // Create four children with boundaries corresponding to the four quadrants.
        // Note: The coordinate system here assumes that increasing y goes downward.
        northeast = new QuadTree(x + w, y - h, w, h, capacity);
        northwest = new QuadTree(x - w, y - h, w, h, capacity);
        southeast = new QuadTree(x + w, y + h, w, h, capacity);
        southwest = new QuadTree(x - w, y + h, w, h, capacity);

        divided = true;
    }

    // Helper function to recursively query points within a given range.
    void queryRange(const Rect& range, std::vector<Point2D<T>>& found) const {
        // If the range does not intersect this node's boundary, return immediately.
        if (!boundary.intersects(range))
            return;

        // Otherwise, check objects at this node.
        for (const auto& point : points) {
            if (range.contains(point)) {
                found.push_back(point);
            }
        }
        // If subdivided, search the children.
        if (divided) {
            northeast->queryRange(range, found);
            northwest->queryRange(range, found);
            southeast->queryRange(range, found);
            southwest->queryRange(range, found);
        }
    }

};

#endif // QUADTREE_H