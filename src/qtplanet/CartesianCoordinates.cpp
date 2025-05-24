#include "qtplanet/CartesianCoordinates.h"
#include "qtplanet/Quadtree.h"
#include "qtplanet/GeoTIFFLoader.h"

#include <cmath>
#include <algorithm>

std::array<Cartesian::Boundary,4>
CoordinateTraits<Cartesian>::getChildBounds(const Boundary& b) {
    double hw = b.width * 0.5f;
    double hh = b.height * 0.5f;
    return {{
        { b.x + hw, b.y - hh, hw, hh }, // NE
        { b.x - hw, b.y - hh, hw, hh }, // NW
        { b.x + hw, b.y + hh, hw, hh }, // SE
        { b.x - hw, b.y + hh, hw, hh }  // SW
    }};
}

bool CoordinateTraits<Cartesian>::contains(const Boundary& b, double x, double y) {
    double left   = b.x - b.width;
    double right  = b.x + b.width;
    double top    = b.y - b.height;
    double bottom = b.y + b.height;
    return x >= left && x <= right && y >= top && y <= bottom;
}

double CoordinateTraits<Cartesian>::distanceToBounds(
    const Boundary& b,
    double cx, double cy, double cz,
    double elevation)
{
    double dx = std::max(std::fabs(cx - b.x) - b.width, 0.0);
    double dy = std::max(std::fabs(cy - b.y) - b.height, 0.0);
//     double dz = std::max(std::fabs(cz - elevation), 0.0f);
//     return std::sqrt(dx*dx + dy*dy + dz*dz);
    double dz = elevation - cz;
    // double version

    
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::array<Cartesian::Boundary,4>
CoordinateTraits<Cartesian>::subdivide(const Boundary& b) {
    return getChildBounds(b);
}

double CoordinateTraits<Cartesian>::distance(Cartesian::Position p1,
                                           Cartesian::Position p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx*dx + dy*dy);
}


std::tuple<double,double,double>
CoordinateTraits<Cartesian>::cartesianAt(const Boundary& b,
                                         int i, int j, int divisions,
                                         QuadTree<TileMetadata,Cartesian>* tree,
                                         const std::shared_ptr<MultiGeoTIFFManager>& geoLoader,
                                         int zoomLevel) 
{
    double stepX = (2 * b.width)  / divisions;
    double stepY = (2 * b.height) / divisions;
    double startX = b.x - b.width;
    double startY = b.y - b.height;
    double x = startX + i * stepX;
    double y = startY + j * stepY;
    double z = getElevation({ x, y }, tree, geoLoader);
    return { x, y, z };
}


double CoordinateTraits<Cartesian>::computeBaseElevation(
    const Cartesian::Position& pos,
    const std::shared_ptr<MultiGeoTIFFManager>& geoLoader,
    int zoomLevel)
{
    if (geoLoader) {
        // const auto& gt = geoLoader->getGeoTransform();
        // double originX     = gt[0];
        // double pixelWidth  = gt[1];
        // double originY     = 0.0;
        // double pixelHeight = gt[5];

        // double col_f = (pos.x - originX) / pixelWidth;
        // double row_f = (pos.y - originY) / pixelHeight;
        // int col0 = static_cast<int>(std::floor(col_f));
        // int row0 = static_cast<int>(std::floor(row_f));
        // int col1 = col0 + 1;
        // int row1 = row0 + 1;

        // int width  = geoLoader->getWidth();
        // int height = geoLoader->getHeight();
        // const auto& data = geoLoader->getElevationData();
        // double interpolated = 0.0f;

        // if (col0 >= 0 && row0 >= 0 && col1 < width && row1 < height) {
        //     double v00 = data[row0*width + col0];
        //     double v10 = data[row0*width + col1];
        //     double v01 = data[row1*width + col0];
        //     double v11 = data[row1*width + col1];
        //     double tx = col_f - col0;
        //     double ty = row_f - row0;
        //     interpolated = static_cast<double>(
        //         (1 - tx)*(1 - ty)*v00 +
        //          tx*(1 - ty)*v10 +
        //         (1 - tx)*ty   *v01 +
        //          tx*ty       *v11
        //     );
        // }
        // // add Perlin noise
        // double alpha     = 0.7f;
        // double frequency = 1.0f;
        // double amplitude = 0.25f;
        // double noise     = Perlin::noise(pos.x * frequency, pos.y * frequency) * amplitude;
        // return interpolated + alpha * noise;
        
        return geoLoader->sample(pos.x, pos.y, 0).value_or(
            Perlin::noise(pos.x * 0.1f, pos.y * 0.1f)); // fallback
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
    Cartesian::Boundary boundary = node->getBoundary();
    double left   = boundary.x - boundary.width;
    double right  = boundary.x + boundary.width;
    double top    = boundary.y - boundary.height;
    double bottom = boundary.y + boundary.height;

    // If (x,y) lies outside this node, return nullptr.
    if (pos.x < left || pos.x > right || pos.y < top || pos.y > bottom)
        return nullptr;

    // If not divided, this is the leaf.
    if (!node->isDivided())
        return node;

    // Otherwise, search the children.
    QuadTree<TileMetadata, Cartesian>* found = findLeafNode(node->getNortheastNonConst(), pos);
    if (found) return found;
    found = findLeafNode(node->getNorthwestNonConst(), pos);
    if (found) return found;
    found = findLeafNode(node->getSoutheastNonConst(), pos);
    if (found) return found;
    return findLeafNode(node->getSouthwestNonConst(), pos);
}

double CoordinateTraits<Cartesian>::getElevation(
    const Cartesian::Position& pos,
    QuadTree<TileMetadata,Cartesian>* tree,
    const std::shared_ptr<MultiGeoTIFFManager>& geoLoader,
    int zoomLevel)
{
    double base = computeBaseElevation(pos, geoLoader);
    auto* leaf = findLeafNode(tree, pos);
    if (!leaf) return base;

    const auto* meta = leaf->getType();
    if (meta->dirtyVertices.empty()) return base;

    double sum = 0.f;
    double weight = 0.f;
    constexpr double radius = 1e-1f;

    for (const auto& dp : meta->dirtyVertices) {
        double d = distance(pos, { dp.x, dp.y });
        if (d < radius) {
            double w = 1.0f / d;
            sum += dp.z * w;
            weight += w;
        }
    }
    return (weight > 0.f) ? (sum / weight) : base;
}

std::pair<int,int> CoordinateTraits<Cartesian>::computeTileIndices(const Position& pos, double tileSize) {
    return { static_cast<int>(std::floor(pos.x/tileSize)),
             static_cast<int>(std::floor(pos.y/tileSize)) };
}

std::pair<double,double> CoordinateTraits<Cartesian>::tileCenterPosition(const TileKey& key, double tileSize) {
    return { key.x*tileSize + tileSize*0.5,
             key.y*tileSize + tileSize*0.5 };
}