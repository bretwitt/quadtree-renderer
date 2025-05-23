#include "qtplanet/QuadtreeTile.h"
#include <cmath>
#include <cstdlib>
#include <algorithm> 
#include "qtplanet/CoordinateSystems.h"
#include "qtplanet/MultiGeoTIFFManager.h"

// ------------------------------
// Constructor & Destructor
// ------------------------------
template<typename CoordSystem>
QuadtreeTile<CoordSystem>::QuadtreeTile(typename CoordSystem::Boundary b, const std::shared_ptr<MultiGeoTIFFManager>& geoLoader)
    : geoTIFFLoaderPtr(geoLoader)
{
    tree = new QuadTree<TileMetadata,CoordSystem>(b);
    
    // Set up callbacks.
    tree->nodeInitCallback = [this](QuadTree<TileMetadata, CoordSystem>* node) {
        this->onNewBucket(node);
    };

    tree->nodeDestroyCallback = [this](QuadTree<TileMetadata, CoordSystem>* node) {
        this->onUnloadBucket(node);
    };
    tree->nodeSplitCallback = [this](QuadTree<TileMetadata, CoordSystem>* parent) {
        this->onUnloadBucket(parent);
        this->onSplit(parent);
    };
    tree->nodeMergeCallback = [this](QuadTree<TileMetadata, CoordSystem>* node) {
        this->onNewBucket(node);
        this->onMerge(node);
    };

    // onNewBucket(tree);

}

template<typename CoordSystem>
QuadtreeTile<CoordSystem>::~QuadtreeTile() {
    delete tree;
}

// ------------------------------
// Public Methods
// ------------------------------
template<typename CoordSystem>
void QuadtreeTile<CoordSystem>::updateLOD(double cameraX, double cameraY, double cameraZ,
                             double splitThreshold, double mergeThreshold, int& subdivisions)
{
    updateLODRec(tree, cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold, subdivisions);
}

template<typename CoordSystem>
QuadTree<TileMetadata,CoordSystem>* QuadtreeTile<CoordSystem>::getTree() const {
    return tree;
}

template<typename CoordSystem>
std::unordered_map<QuadTree<TileMetadata,CoordSystem>*, Mesh> QuadtreeTile<CoordSystem>::getMeshes() {
    return bucketMeshes;
}

template<typename CoordSystem>
void QuadtreeTile<CoordSystem>::tick() {
    tickLeaves(getTree());
}

template<typename CoordSystem>
void QuadtreeTile<CoordSystem>::tickLeaves(QuadTree<TileMetadata,CoordSystem>* node) {
    if (!node) return;  

    if (node->isDivided()) {
        tickLeaves(node->getNortheastNonConst());
        tickLeaves(node->getNorthwestNonConst());
        tickLeaves(node->getSoutheastNonConst());
        tickLeaves(node->getSouthwestNonConst());
    } else {
        auto* data = node->getType();
        int expirationTicks = 100;

        // If a split is active, cancel the merge and update only the split counter.
        if (data->ticksSinceSplit != -1) {
            data->ticksSinceMerge = -1;  // Ensure merge is canceled.
            data->ticksSinceSplit++;
            if (data->ticksSinceSplit > expirationTicks)
                data->ticksSinceSplit = -1;
        }
        // Otherwise, if no split is active, update the merge counter if it is active.
        else if (data->ticksSinceMerge != -1) {
            data->ticksSinceMerge++;
            if (data->ticksSinceMerge > expirationTicks)
                data->ticksSinceMerge = -1;
        }
    }
}

template<typename CoordSystem>
size_t QuadtreeTile<CoordSystem>::getMemoryUsage() const {
    size_t totalMemory = 0;
    for (const auto& bucketMesh : bucketMeshes) {
        const Mesh& mesh = bucketMesh.second;
        totalMemory += mesh.vertices.capacity() * sizeof(double);
        totalMemory += mesh.indices.capacity() * sizeof(int);
        totalMemory += mesh.normals.capacity() * sizeof(double);
        totalMemory += mesh.texCoords.capacity() * sizeof(int);
        totalMemory += mesh.coarseNormals.capacity() * sizeof(int);
        
    }
    return totalMemory;
}

// ------------------------------
// Private Methods
// ------------------------------

template<typename CoordSystem>
void QuadtreeTile<CoordSystem>::updateLODRec(QuadTree<TileMetadata,CoordSystem>* node,
                                double cameraX, double cameraY, double cameraZ,
                                double splitThreshold, double mergeThreshold,
                                int& subdivisions)
{
    // Retrieve the node's boundary.
    typename CoordSystem::Boundary boundary = node->getBoundary();


    double distance = CoordinateTraits<CoordSystem>::distanceToBounds
        (
            boundary, cameraX, cameraY, cameraZ, //0
            getElevation( {boundary.x,boundary.y }, 0)
        );

    // std::cout << "Distance: " << distance << std::endl;

    int level = node->getLevel();

    double radius = 1000.0;

    splitThreshold = radius * 250.0f;
    mergeThreshold = radius * 0.1;

    // double effectiveSplitThreshold = splitThreshold / (level + 1);
    double effectiveMergeThreshold = mergeThreshold / (level + 1);

    // effectiveSplitThreshold and effectiveMergeThreshold table
    // define mergetable
    double r = 1737400.0;

    std::array<std::pair<int, double>, 8> mergeTable = 
    {

        std::make_pair(0, r*0.5f),
        std::make_pair(1, r*0.25f),
        std::make_pair(2, r*0.125f),
        std::make_pair(3, r*0.0625f),
        std::make_pair(4, r*0.03125f*0.25f),
        std::make_pair(5, r*0.015625f*0.25f),
        std::make_pair(6, r*0.0078125f*0.25f),
        // std::make_pair(7, r*0.0078125f)
    };
    
    double effectiveSplitThreshold = mergeTable[level].second;
    

    // std::cout << "Level : " << level << std::endl;

    // if (std::abs(node->getBoundary().y) + node->getBoundary().height >= 85.0f) {
    //     return;
    // }
    if (std::abs(boundary.y) + boundary.height >= 89.9f)
        return;

    if (((distance < effectiveSplitThreshold) && level < 7)) {  
        if (!node->isDivided()) {
            node->subdivide();
            subdivisions++; 
            onUnloadBucket(node);
        }
        if (node->isDivided()) {
            updateLODRec(node->getNortheastNonConst(), cameraX, cameraY, cameraZ,
                           splitThreshold, mergeThreshold, subdivisions);
            updateLODRec(node->getNorthwestNonConst(), cameraX, cameraY, cameraZ,
                           splitThreshold, mergeThreshold, subdivisions);
            updateLODRec(node->getSoutheastNonConst(), cameraX, cameraY, cameraZ,
                           splitThreshold, mergeThreshold, subdivisions);
            updateLODRec(node->getSouthwestNonConst(), cameraX, cameraY, cameraZ,
                           splitThreshold, mergeThreshold, subdivisions);
        }
    }
    else if (distance > effectiveMergeThreshold ) {
        if (node->isDivided()) {
            node->merge();
        }
    }
}

template<typename CoordSystem>
void QuadtreeTile<CoordSystem>::onNewBucket(QuadTree<TileMetadata,CoordSystem>* node) {
    // if (!node->isDivided()) {
        updateMesh(node);
    // }
}


template<typename CoordSystem>
void QuadtreeTile<CoordSystem>::onSplit(QuadTree<TileMetadata,CoordSystem>* parent) {
    if (!parent) return;
    
    TileMetadata* parentMeta = parent->getType();
    if (!parentMeta->dirtyVerticesTransferred) {
        // Transfer dirty vertices to children using half-open bounds:
        auto transferToChild = [parentMeta](QuadTree<TileMetadata,CoordSystem>* child) {
            if (!child) return;
            TileMetadata* childMeta = child->getType();
            childMeta->dirtyVertices.clear();
            for(const auto& dp : parentMeta->dirtyVertices) {
                 if(CoordinateTraits<CoordSystem>::contains(child->getBoundary(), dp.x,dp.y)) {
                     childMeta->dirtyVertices.push_back(dp);
                 }
            }
        };

        transferToChild(parent->getNortheastNonConst());
        transferToChild(parent->getNorthwestNonConst());
        transferToChild(parent->getSoutheastNonConst());
        transferToChild(parent->getSouthwestNonConst());

        parentMeta->dirtyVerticesTransferred = true;
        parentMeta->dirtyVertices.clear();
    }
 
    parent->getType()->ticksSinceSplit = 0; 
}

template<typename CoordSystem>
void QuadtreeTile<CoordSystem>::deduplicateVertices(std::vector<vec3>& vertices) {
    std::sort(vertices.begin(), vertices.end(), [](const vec3& a, const vec3& b) {
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    });

    vertices.erase(std::unique(vertices.begin(), vertices.end(), [](const vec3& a, const vec3& b) {
        const double epsilon = 1e-6f;
        return (std::fabs(a.x - b.x) < epsilon &&
                std::fabs(a.y - b.y) < epsilon &&
                std::fabs(a.z - b.z) < epsilon);
    }), vertices.end());
}

template<typename CoordSystem>
void QuadtreeTile<CoordSystem>::onMerge(QuadTree<TileMetadata,CoordSystem>* node) {
    if (!node) return;

    TileMetadata* parentMeta = node->getType();
    parentMeta->dirtyVertices.clear();

    for (auto* child : { node->getNortheastNonConst(), node->getNorthwestNonConst(),
                         node->getSoutheastNonConst(), node->getSouthwestNonConst() }) {
        if (!child) continue;
        TileMetadata* childMeta = child->getType();
        parentMeta->dirtyVertices.insert(
            parentMeta->dirtyVertices.end(),
            childMeta->dirtyVertices.begin(),
            childMeta->dirtyVertices.end()
        );
        childMeta->dirtyVertices.clear();
    }

    deduplicateVertices(parentMeta->dirtyVertices);
    parentMeta->ticksSinceMerge = 0;
}

template<typename CoordSystem>
void QuadtreeTile<CoordSystem>::onUnloadBucket(QuadTree<TileMetadata,CoordSystem>* node) {
    auto it = bucketMeshes.find(node);
    if (it != bucketMeshes.end()) {
        bucketMeshes.erase(it);
    }
}

template<typename CoordSystem>
double QuadtreeTile<CoordSystem>::computeBaseElevation(typename CoordSystem::Position pos, int zoomLevel) {
    if (geoTIFFLoaderPtr == nullptr) {
        return 0.0;
    }
    double res = CoordinateTraits<CoordSystem>::computeBaseElevation(pos, geoTIFFLoaderPtr, zoomLevel);
    
    return res;
}

template<typename CoordSystem>
void QuadtreeTile<CoordSystem>::updateMesh(QuadTree<TileMetadata, CoordSystem>* node) {
    int level = node->getLevel();
    typename CoordSystem::Boundary bounds = node->getBoundary();

    // Generate the mesh for the child.
    Mesh mesh = generateTriangularMesh(bounds, level);
    bucketMeshes[node] = mesh;
}

template<typename CoordSystem>
void QuadtreeTile<CoordSystem>::cross(const double* a, const double* b, double* result) {
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

template<typename CoordSystem>
void QuadtreeTile<CoordSystem>::normalize(double* v) {
    double length = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (length > 1e-8f) {
        v[0] /= length;
        v[1] /= length;
        v[2] /= length;
    }
}

template<typename CoordSystem>
void QuadtreeTile<CoordSystem>::calculateNormals(Mesh& mesh)
{
    const size_t vertexCount = mesh.vertices.size() / 3;
    mesh.normals.resize(mesh.vertices.size(), 0.0f); // 3 doubles per vertex, initialized to 0

    // Accumulate face normals.
    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
        int i0 = mesh.indices[i];
        int i1 = mesh.indices[i + 1];
        int i2 = mesh.indices[i + 2];

        double v0[3] = {
            mesh.vertices[3 * i0 + 0],
            mesh.vertices[3 * i0 + 1],
            mesh.vertices[3 * i0 + 2]
        };
        double v1[3] = {
            mesh.vertices[3 * i1 + 0],
            mesh.vertices[3 * i1 + 1],
            mesh.vertices[3 * i1 + 2]
        };
        double v2[3] = {
            mesh.vertices[3 * i2 + 0],
            mesh.vertices[3 * i2 + 1],
            mesh.vertices[3 * i2 + 2]
        };

        double e1[3] = { v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2] };
        double e2[3] = { v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2] };

        double faceNormal[3];
        cross(e1, e2, faceNormal);

        mesh.normals[3 * i0 + 0] += faceNormal[0];
        mesh.normals[3 * i0 + 1] += faceNormal[1];
        mesh.normals[3 * i0 + 2] += faceNormal[2];

        mesh.normals[3 * i1 + 0] += faceNormal[0];
        mesh.normals[3 * i1 + 1] += faceNormal[1];
        mesh.normals[3 * i1 + 2] += faceNormal[2];

        mesh.normals[3 * i2 + 0] += faceNormal[0];
        mesh.normals[3 * i2 + 1] += faceNormal[1];
        mesh.normals[3 * i2 + 2] += faceNormal[2];
    }

    // Normalize each accumulated normal.
    for (size_t i = 0; i < vertexCount; i++) {
        double* normal = &mesh.normals[3 * i];
        normalize(normal);
    }
}

template<typename CoordSystem>
Mesh QuadtreeTile<CoordSystem>::generateTriangularMesh(typename CoordSystem::Boundary bounds, int level)
{
    Mesh mesh;

    // auto quant = [](double v) {
    //     // snap to millimeter grid
    //     return std::round(v * 1000.0f) / 1000.0f;
    // };

    int divisions = 1 << (level + 1);
    int numVerticesPerSide = divisions + 1;

    for (int j = 0; j < numVerticesPerSide; ++j) {
        for (int i = 0; i < numVerticesPerSide; ++i) {
            auto [x, y, z] = CoordinateTraits<CoordSystem>::cartesianAt(bounds, i, j, divisions, tree, geoTIFFLoaderPtr, level);

            mesh.vertices.push_back( (x) );
            mesh.vertices.push_back( (y) );
            mesh.vertices.push_back( (z) );


            double lonMin = bounds.x - bounds.width;
            double lonMax = bounds.x + bounds.width;
            double latMin = bounds.y - bounds.height;
            double latMax = bounds.y + bounds.height;

            // Assumes a texture mapped over the whole globe
            // Longitude from -180 to 180 → map to U: 0.0 to 1.0
            // Latitude from -90 to 90   → map to V: 0.0 to 1.0

            double lon = lonMin + (lonMax - lonMin) * (static_cast<double>(i) / divisions);
            double lat = latMin + (latMax - latMin) * (static_cast<double>(j) / divisions);

            double u = (lon + 180.0) / 360.0;
            double v = (lat + 90.0) / 180.0;

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

template<typename CoordSystem>
double QuadtreeTile<CoordSystem>::getElevation(typename CoordSystem::Position pos, int zoomLevel) {
    return CoordinateTraits<CoordSystem>::getElevation(pos, tree, geoTIFFLoaderPtr, zoomLevel);
}

template<typename CoordSystem>
QuadTree<TileMetadata, CoordSystem>* QuadtreeTile<CoordSystem>::findLeafNode( 
                QuadTree<TileMetadata, CoordSystem>* node,
                typename CoordSystem::Position pos ) 
{
    return CoordinateTraits<CoordSystem>::findLeafNode(node, pos);
}

/*
 * Explicit template instantiation for Cartesian coordinate system.
*/
template class QuadtreeTile<Cartesian>;
template class QuadtreeTile<Spherical>;