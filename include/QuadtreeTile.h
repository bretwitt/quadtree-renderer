#ifndef QUADTREE_TILE_H
#define QUADTREE_TILE_H

#include "Quadtree.h"
#include <cmath>      // For std::sqrt, std::floor
#include <unordered_map>
#include <vector>
#include <iostream>
#include "Perlin.h"
#include "GeoTIFFLoader.h"  // Include the GeoTIFF loader

//----------------------------------------------------------
// Mesh structure and QuadtreeTile definition
//----------------------------------------------------------

struct Mesh {
    std::vector<float> vertices; // Each vertex: x, y, z
    std::vector<int> indices;
    std::vector<float> normals;
};

template<typename T>
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
    QuadtreeTile(float x, float y, float width, float height, GeoTIFFLoader* geoLoader = nullptr)
      : geoTIFFLoader(geoLoader)
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

    void updateLOD(float cameraX, float cameraY, float cameraZ,
                   float splitThreshold, float mergeThreshold, int& subdivisions)
    {
        updateLODRec(tree, cameraX, cameraY, cameraZ, splitThreshold, mergeThreshold, subdivisions);
    }

    QuadTree<T>* getTree() const { return tree; }

    std::unordered_map<QuadTree<int>*, Mesh> getMeshes() {
        return bucketMeshes;
    }

    size_t getMemoryUsage() const {
        size_t totalMemory = 0;
        for (const auto& bucketMesh : bucketMeshes) {
            const Mesh& mesh = bucketMesh.second;
            totalMemory += mesh.vertices.capacity() * sizeof(float);
            totalMemory += mesh.indices.capacity() * sizeof(int);
        }
        return totalMemory;
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
        
        float dz = getElevation(boundary.x,boundary.y) - cameraZ;  // Assuming terrain is at z=0

        float distance = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));

        int level = node->getLevel();

        float effectiveSplitThreshold = splitThreshold / (level + 1);
        float effectiveMergeThreshold = mergeThreshold / (level + 1);

        if(level == 0) {
            effectiveSplitThreshold = 1200.;
        }
        if(level == 1) {
            effectiveSplitThreshold = 600.;
        }
        if(level == 2) {
           effectiveSplitThreshold = 300.;
        }
        if(level == 3) {
            effectiveSplitThreshold = 50.;
        }

        if (distance < effectiveSplitThreshold && level < 4) {  
            if (!node->isDivided()) {
                node->subdivide();
                subdivisions++; 
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
        else if (distance > effectiveMergeThreshold) {
            if (node->isDivided()) {
                node->merge();
            }
        }
    }

    // Called when a new bucket (node) is created.
    void onNewBucket(QuadTree<int>* node) {
        int level = node->getLevel();
        typename QuadTree<int>::QuadBoundary bounds = node->getBoundary();

        Mesh mesh = generateTriangularMesh(bounds.x, bounds.y, bounds.width, bounds.height, level);
        bucketMeshes[node] = mesh;
    }

    // Called when a bucket (node) is unloaded.
    void onUnloadBucket(QuadTree<int>* node) {
        auto it = bucketMeshes.find(node);
        if (it != bucketMeshes.end()) {
            bucketMeshes.erase(it);
        }
    }

    /**
     * Retrieves the elevation at a given (x, y) coordinate.
     *
     * If a GeoTIFFLoader is provided, it converts world coordinates (x, y) to pixel
     * coordinates using the geotransform and returns the elevation from the raster data.
     * Otherwise, it falls back to generating elevation using Perlin noise.
     */
    float getElevation(float x, float y) {
        if (geoTIFFLoader) {
            const std::vector<double>& gt = geoTIFFLoader->getGeoTransform();
            // Assuming the geotransform is of the form:
            // [ originX, pixelWidth, rotationX, originY, rotationY, pixelHeight ]
            double originX = gt[0];
            double pixelWidth = gt[1];
            // double originY = gt[3];
            double originY = 0.0; // as used in the original code
            double pixelHeight = gt[5];
            
            // Compute fractional pixel indices
            double col_f = (x - originX) / pixelWidth;
            double row_f = (y - originY) / pixelHeight;

            // Determine the indices of the four surrounding pixels.
            int col0 = static_cast<int>(std::floor(col_f));
            int row0 = static_cast<int>(std::floor(row_f));
            int col1 = col0 + 1;
            int row1 = row0 + 1;

            int width = geoTIFFLoader->getWidth();
            int height = geoTIFFLoader->getHeight();
            const std::vector<float>& elevationData = geoTIFFLoader->getElevationData();
            float interpolatedValue = 0.0;

            if (col0 >= 0 && row0 >= 0 && col1 < width && row1 < height) {
                // Retrieve the elevation values from the four surrounding pixels.
                float v00 = elevationData[row0 * width + col0]; // top-left
                float v10 = elevationData[row0 * width + col1]; // top-right
                float v01 = elevationData[row1 * width + col0]; // bottom-left
                float v11 = elevationData[row1 * width + col1]; // bottom-right

                // Compute the fractional part.
                double tx = col_f - col0;
                double ty = row_f - row0;

                // Perform bilinear interpolation.
                interpolatedValue = static_cast<float>(
                    (1 - tx) * (1 - ty) * v00 +
                    tx       * (1 - ty) * v10 +
                    (1 - tx) * ty       * v01 +
                    tx       * ty       * v11
                );
            }

            // Finally, perform bicubic interpolation on the 4×4 patch

            float alpha = 0.7;
            float frequency = 0.1f;
            float amplitude = 0.25f;

            float upscalePerlin = Perlin::noise(x * frequency, y * frequency) * amplitude;

            // upscalePerlin *= Perlin::noise(x * frequency, y * frequency) * amplitude;

            return interpolatedValue + (alpha*upscalePerlin);
            
            // std::cout << y << " " << originY << std::endl;

            return 0.0f;
        } else {
            // Fallback: use Perlin noise.
            float frequency = 0.1f;
            float amplitude = 1.0f;
            return Perlin::noise(x * frequency, y * frequency) * amplitude;
        }
    }

    // Cubic interpolation between four known values p0, p1, p2, p3, given a
    // fractional position t in [0, 1]. 
    static float cubicInterpolate(float p0, float p1, float p2, float p3, float t)
    {
        float a = (-0.5f * p0) + (1.5f * p1) - (1.5f * p2) + (0.5f * p3);
        float b = (p0) - (2.5f * p1) + (2.0f * p2) - (0.5f * p3);
        float c = (-0.5f * p0) + (0.5f * p2);
        float d = p1;

        return a * (t * t * t) + b * (t * t) + c * t + d;
    }

    static float cubicHermite(float p0, float p1, float p2, float p3, float t)
    {
        float a0 = p3 - p2 - p0 + p1;
        float a1 = p0 - p1 - a0;
        float a2 = p2 - p0;
        float a3 = p1;

        return a0*(t*t*t) + a1*(t*t) + a2*t + a3;
    }

    static float bicubicInterpolate(
        const float patch[4][4],  // 4×4 neighborhood
        float tx,                 // fractional coordinate in x
        float ty)                 // fractional coordinate in y
    {
        // First, interpolate each of the 4 rows in the x direction
        float col0 = cubicInterpolate(patch[0][0], patch[0][1], patch[0][2], patch[0][3], tx);
        float col1 = cubicInterpolate(patch[1][0], patch[1][1], patch[1][2], patch[1][3], tx);
        float col2 = cubicInterpolate(patch[2][0], patch[2][1], patch[2][2], patch[2][3], tx);
        float col3 = cubicInterpolate(patch[3][0], patch[3][1], patch[3][2], patch[3][3], tx);

        // Now interpolate the resulting 4 values in the y direction
        return cubicInterpolate(col0, col1, col2, col3, ty);
    }


    /**
     * Generates a triangular mesh for the tile.
     *
     * The elevation for each vertex is computed using either the GeoTIFF data
     * (if available) or Perlin noise.
     */
    Mesh generateTriangularMesh(float centerX, float centerY, float halfWidth, float halfHeight, int level) {
        Mesh mesh;

        int divisions = 1 << (level + 1); // 2^(level + 1)
        int numVerticesPerSide = divisions + 1;

        float stepX = (2.0f * halfWidth) / divisions;
        float stepY = (2.0f * halfHeight) / divisions;

        float startX = centerX - halfWidth;
        float startY = centerY - halfHeight;

        // Generate vertices.
        for (int j = 0; j < numVerticesPerSide; ++j) {
            for (int i = 0; i < numVerticesPerSide; ++i) {
                float x = startX + i * stepX;
                float y = startY + j * stepY;
                float z = getElevation(x, y);
                mesh.vertices.push_back(x);
                mesh.vertices.push_back(y);
                mesh.vertices.push_back(z);
            }
        }

        // Generate indices.
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

        calculateNormals(mesh);
        return mesh;
    }

    // Cross product helper
    static inline void cross(const float* a, const float* b, float* result) {
        result[0] = a[1] * b[2] - a[2] * b[1];
        result[1] = a[2] * b[0] - a[0] * b[2];
        result[2] = a[0] * b[1] - a[1] * b[0];
    }

    // Normalize helper
    static inline void normalize(float* v) {
        float length = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (length > 1e-8f) {
            v[0] /= length;
            v[1] /= length;
            v[2] /= length;
        }
    }

    void calculateNormals(Mesh& mesh)
    {
        const size_t vertexCount = mesh.vertices.size() / 3;
        mesh.normals.resize(mesh.vertices.size(), 0.0f); // 3 floats per vertex, initialized to 0

        // Accumulate face normals
        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            int i0 = mesh.indices[i];
            int i1 = mesh.indices[i + 1];
            int i2 = mesh.indices[i + 2];

            // Get the three vertices of this face (triangle)
            float v0[3] = {
                mesh.vertices[3 * i0 + 0],
                mesh.vertices[3 * i0 + 1],
                mesh.vertices[3 * i0 + 2]
            };
            float v1[3] = {
                mesh.vertices[3 * i1 + 0],
                mesh.vertices[3 * i1 + 1],
                mesh.vertices[3 * i1 + 2]
            };
            float v2[3] = {
                mesh.vertices[3 * i2 + 0],
                mesh.vertices[3 * i2 + 1],
                mesh.vertices[3 * i2 + 2]
            };

            // Edges of the triangle: v1 - v0, v2 - v0
            float e1[3] = { v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2] };
            float e2[3] = { v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2] };

            // Face normal via cross product
            float faceNormal[3];
            cross(e1, e2, faceNormal);

            // Accumulate this face normal into each of the triangle's vertices
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

        // Now normalize each accumulated normal
        for (size_t i = 0; i < vertexCount; i++) {
            float* normal = &mesh.normals[3 * i];
            normalize(normal);
        }
    }


    // Pointer to the underlying QuadTree.
    QuadTree<T>* tree;
    // Mapping from a quadtree node to its mesh.
    std::unordered_map<QuadTree<int>*, Mesh> bucketMeshes;
    // Optional pointer to a GeoTIFFLoader for elevation data.
    GeoTIFFLoader* geoTIFFLoader = nullptr;
};

#endif // QUADTREE_TILE_H