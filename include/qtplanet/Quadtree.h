#ifndef QUADTREE_H
#define QUADTREE_H

#include <iostream>
#include <cmath>
#include <functional>
#include <vector>
#include <stack>
#include "CoordinateSystems.h"

using std::cout;
using std::endl;

template<typename T, typename CoordSystem>
class QuadTree {
public:
    using NodeInitCallback = std::function<void(QuadTree<T,CoordSystem>*)>;
    using NodeDestroyCallback = std::function<void(QuadTree<T,CoordSystem>*)>;
    using NodeSplitCallback = std::function<void(QuadTree<T,CoordSystem>*)>;
    using NodeMergeCallback = std::function<void(QuadTree<T,CoordSystem>*)>;

    NodeInitCallback nodeInitCallback;
    NodeDestroyCallback nodeDestroyCallback;
    NodeSplitCallback nodeSplitCallback;
    NodeMergeCallback nodeMergeCallback;
   
    using Boundary = typename CoordSystem::Boundary;
    using Traits = CoordinateTraits<CoordSystem>;
    // QuadTree(Boundary boundary,
    //          T type = T{},
    //          CoordSystem ctype = CoordSystem{},
    //          int level = 0, 
    //          NodeInitCallback nodeInitCallback = nullptr,
    //          NodeDestroyCallback nodeDestroyCallback = nullptr, 
    //          NodeSplitCallback nodeSplitCallback = nullptr,
    //          NodeMergeCallback nodeMergeCallback = nullptr,
    //          QuadTree<T, CoordSystem>* parent = nullptr)

    QuadTree(Boundary boundary, T type = T{}, CoordSystem ctype = CoordSystem{}, int level = 0, 
             NodeInitCallback nodeInitCallback = nullptr,
             NodeDestroyCallback nodeDestroyCallback = nullptr, 
             NodeSplitCallback nodeSplitCallback = nullptr,
             NodeMergeCallback nodeMergeCallback = nullptr,
             QuadTree<T, CoordSystem>* parent = nullptr)
        : boundary{ boundary },
          type(type),
          ctype(ctype),
          divided(false),
          level(level),
          nodeInitCallback(nodeInitCallback),
          nodeDestroyCallback(nodeDestroyCallback),
          nodeSplitCallback(nodeSplitCallback),
          nodeMergeCallback(nodeMergeCallback),
          m_parent(parent),
          cacheValid(false) // Initialize cache as invalid
    {
        if (nodeInitCallback) {
            nodeInitCallback(this);
        }

        northeast = nullptr;
        northwest = nullptr;
        southeast = nullptr;
        southwest = nullptr;
    }

    ~QuadTree() {
        if(nodeDestroyCallback) {
            nodeDestroyCallback(this);
        }
        delete northeast;
        delete northwest;
        delete southeast;
        delete southwest;
    }


    Boundary getBoundary() const { return boundary; }
    bool isDivided() const { return divided; }

    T* getType() { return &type; }
    void setType(T newType) { type = newType; }

    void setScale(int targetDepth) {
        adjustScale(targetDepth, 0);
    }

    void adjustScale(int targetDepth, int currentDepth) {
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

    void subdivide() {
        if (divided) return;

        int childLevel = level + 1;

        auto childBounds = Traits::getChildBounds(boundary);

        northeast = new QuadTree(childBounds[0], type, ctype, childLevel, nodeInitCallback, nodeDestroyCallback, nodeSplitCallback, nodeMergeCallback, this);
        northwest = new QuadTree(childBounds[1], type, ctype, childLevel, nodeInitCallback, nodeDestroyCallback, nodeSplitCallback, nodeMergeCallback, this);
        southeast = new QuadTree(childBounds[2], type, ctype, childLevel, nodeInitCallback, nodeDestroyCallback, nodeSplitCallback, nodeMergeCallback, this);
        southwest = new QuadTree(childBounds[3], type, ctype, childLevel, nodeInitCallback, nodeDestroyCallback, nodeSplitCallback, nodeMergeCallback, this);

        divided = true;

        clearCache(); // Invalidate cache on structure change

        if(nodeSplitCallback) {
            nodeSplitCallback(this);
        }
    }
    
    void merge() {
        if (!divided) return;

        if(nodeMergeCallback) {
            nodeMergeCallback(this);
        }

        delete northeast;
        delete northwest;
        delete southeast;
        delete southwest;
        
        northeast = northwest = southeast = southwest = nullptr;
        divided = false;

        clearCache(); // Invalidate cache on structure change
    }

    int getLevel() const { return level; }

    int getNumChildren() const {
        if (!divided)
            return 0;
        return 4 + northeast->getNumChildren() +
                   northwest->getNumChildren() +
                   southeast->getNumChildren() +
                   southwest->getNumChildren();
    }

    void collectLeaves(std::vector<QuadTree<T,CoordSystem>*>& leaves) {
        std::stack<QuadTree<T,CoordSystem>*> stack;
        stack.push(this);
        while(!stack.empty()) {
            QuadTree<T,CoordSystem>* node = stack.top();
            stack.pop();
            if(!node->divided) {
                leaves.push_back(node);
            } else {
                if(node->northeast) stack.push(node->northeast);
                if(node->northwest) stack.push(node->northwest);
                if(node->southeast) stack.push(node->southeast);
                if(node->southwest) stack.push(node->southwest);
            }
        }
        
    }

    std::vector<QuadTree<T,CoordSystem>*> getLeaves() {
        // if (cacheValid) {
        //     return leafCache; // Return cached result if valid
        // }
        // std::cout <<"Invalid" << std::endl;

        leafCache.clear();
        collectLeaves(leafCache);
        cacheValid = true; // Mark cache as valid
        return leafCache;
    }

    bool getCacheValid() { return cacheValid; }

    QuadTree<T, CoordSystem>* getNortheastNonConst() { return northeast; }
    QuadTree<T, CoordSystem>* getNorthwestNonConst() { return northwest; }
    QuadTree<T, CoordSystem>* getSoutheastNonConst() { return southeast; }
    QuadTree<T, CoordSystem>* getSouthwestNonConst() { return southwest; }
    QuadTree<T, CoordSystem>* getNortheast() const { return northeast; }
    QuadTree<T, CoordSystem>* getNorthwest() const { return northwest; }
    QuadTree<T, CoordSystem>* getSoutheast() const { return southeast; }
    QuadTree<T, CoordSystem>* getSouthwest() const { return southwest; }

private:
    Boundary boundary;
    T type;
    CoordSystem ctype;
    bool divided = false;
    int level;
    QuadTree<T, CoordSystem>* northeast;
    QuadTree<T, CoordSystem>* northwest;
    QuadTree<T, CoordSystem>* southeast;
    QuadTree<T, CoordSystem>* southwest;
    QuadTree<T, CoordSystem>* m_parent;

    
    std::vector<QuadTree<T,CoordSystem>*> leafCache; // Cache for leaf nodes
    bool cacheValid; // Flag to track cache validity

    void clearCache() {
        cacheValid = false; // Invalidate cache when the tree structure changes
    }
};

#endif // QUADTREE_H
