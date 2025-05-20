#include "qtplanet/CartesianCoordinates.h"
#include "qtplanet/Quadtree.h"
#include "qtplanet/GeoTIFFLoader.h"

#include <cmath>
#include <algorithm>

std::array<Cartesian::Boundary,4>
CoordinateTraits<Cartesian>::getChildBounds(const Boundary& b) {
    float hw = b.width * 0.5f;
    float hh = b.height * 0.5f;
    return {{
        { b.x + hw, b.y - hh, hw, hh }, // NE
        { b.x - hw, b.y - hh, hw, hh }, // NW
        { b.x + hw, b.y + hh, hw, hh }, // SE
        { b.x - hw, b.y + hh, hw, hh }  // SW
    }};
}

bool CoordinateTraits<Cartesian>::contains(const Boundary& b, float x, float y) {
    float left   = b.x - b.width;
    float right  = b.x + b.width;
    float top    = b.y - b.height;
    float bottom = b.y + b.height;
    return x >= left && x <= right && y >= top && y <= bottom;
}

float CoordinateTraits<Cartesian>::distanceToBounds(
    const Boundary& b,
    float cx, float cy, float cz,
    float elevation)
{
    float dx = std::max(std::fabs(cx - b.x) - b.width, 0.0f);
    float dy = std::max(std::fabs(cy - b.y) - b.height, 0.0f);
    float dz = elevation - cz;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::array<Cartesian::Boundary,4>
CoordinateTraits<Cartesian>::subdivide(const Boundary& b) {
    return getChildBounds(b);
}

float CoordinateTraits<Cartesian>::distance(Cartesian::Position p1,
                                           Cartesian::Position p2)
{
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return std::sqrt(dx*dx + dy*dy);
}


std::tuple<float,float,float>
CoordinateTraits<Cartesian>::cartesianAt(const Boundary& b,
                                         int i, int j, int divisions,
                                         QuadTree<TileMetadata,Cartesian>* tree,
                                         const GeoTIFFLoader* geoLoader)
{
    float stepX = (2 * b.width)  / divisions;
    float stepY = (2 * b.height) / divisions;
    float startX = b.x - b.width;
    float startY = b.y - b.height;
    float x = startX + i * stepX;
    float y = startY + j * stepY;
    float z = getElevation({ x, y }, tree, geoLoader);
    return { x, y, z };
}

float CoordinateTraits<Cartesian>::computeBaseElevation(
    const Cartesian::Position& pos,
    const GeoTIFFLoader* geoLoader)
{
    if (geoLoader) {
        const auto& gt = geoLoader->getGeoTransform();
        double originX     = gt[0];
        double pixelWidth  = gt[1];
        double originY     = 0.0;
        double pixelHeight = gt[5];

        double col_f = (pos.x - originX) / pixelWidth;
        double row_f = (pos.y - originY) / pixelHeight;
        int col0 = static_cast<int>(std::floor(col_f));
        int row0 = static_cast<int>(std::floor(row_f));
        int col1 = col0 + 1;
        int row1 = row0 + 1;

        int width  = geoLoader->getWidth();
        int height = geoLoader->getHeight();
        const auto& data = geoLoader->getElevationData();
        float interpolated = 0.0f;

        if (col0 >= 0 && row0 >= 0 && col1 < width && row1 < height) {
            float v00 = data[row0*width + col0];
            float v10 = data[row0*width + col1];
            float v01 = data[row1*width + col0];
            float v11 = data[row1*width + col1];
            double tx = col_f - col0;
            double ty = row_f - row0;
            interpolated = static_cast<float>(
                (1 - tx)*(1 - ty)*v00 +
                 tx*(1 - ty)*v10 +
                (1 - tx)*ty   *v01 +
                 tx*ty       *v11
            );
        }
        // add Perlin noise
        float alpha     = 0.7f;
        float frequency = 0.1f;
        float amplitude = 0.25f;
        float noise     = Perlin::noise(pos.x * frequency, pos.y * frequency) * amplitude;
        return interpolated + alpha * noise;
    } else {
        // pure noise if no GeoTIFF
        return Perlin::noise(pos.x * 0.1, pos.y * 0.1);
    }
}

QuadTree<TileMetadata,Cartesian>*
CoordinateTraits<Cartesian>::findLeafNode(QuadTree<TileMetadata,Cartesian>* node,
                                          Cartesian::Position pos)
{
    if (!node)
        return nullptr;

    // Get the node’s boundary.
    QuadTree<TileMetadata>::Boundary boundary = node->getBoundary();
    float left   = boundary.x - boundary.width;
    float right  = boundary.x + boundary.width;
    float top    = boundary.y - boundary.height;
    float bottom = boundary.y + boundary.height;

    // If (x,y) lies outside this node, return nullptr.
    if (pos.x < left || pos.x > right || pos.y < top || pos.y > bottom)
        return nullptr;

    // If not divided, this is the leaf.
    if (!node->isDivided())
        return node;

    // Otherwise, search the children.
    QuadTree<TileMetadata>* found = findLeafNode(node->getNortheastNonConst(), pos);
    if (found) return found;
    found = findLeafNode(node->getNorthwestNonConst(), pos);
    if (found) return found;
    found = findLeafNode(node->getSoutheastNonConst(), pos);
    if (found) return found;
    return findLeafNode(node->getSouthwestNonConst(), pos);
}

float CoordinateTraits<Cartesian>::getElevation(
    const Cartesian::Position& pos,
    QuadTree<TileMetadata,Cartesian>* tree,
    const GeoTIFFLoader* geoLoader)
{
    float base = computeBaseElevation(pos, geoLoader);
    auto* leaf = findLeafNode(tree, pos);
    if (!leaf) return base;

    const auto* meta = leaf->getType();
    if (meta->dirtyVertices.empty()) return base;

    float sum = 0.f;
    float weight = 0.f;
    constexpr float radius = 1e-1f;

    for (const auto& dp : meta->dirtyVertices) {
        float d = distance(pos, { dp.x, dp.y });
        if (d < radius) {
            float w = 1.0f / d;
            sum += dp.z * w;
            weight += w;
        }
    }
    return (weight > 0.f) ? (sum / weight) : base;
}

std::pair<int,int> CoordinateTraits<Cartesian>::computeTileIndices(const Position& pos, float tileSize) {
    return { static_cast<int>(std::floor(pos.x/tileSize)),
             static_cast<int>(std::floor(pos.y/tileSize)) };
}

std::pair<float,float> CoordinateTraits<Cartesian>::tileCenterPosition(const TileKey& key, float tileSize) {
    return { key.x*tileSize + tileSize*0.5f,
             key.y*tileSize + tileSize*0.5f };
}