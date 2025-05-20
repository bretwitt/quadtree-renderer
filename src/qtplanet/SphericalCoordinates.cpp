#include "qtplanet/CartesianCoordinates.h"  // for Boundary, Position templates
#include "qtplanet/Quadtree.h"
#include "qtplanet/GeoTIFFLoader.h"
#include <cmath>
#include <algorithm>

namespace {
    // Earth radius in meters
    const double kEarthRadiusMeters = 100.0;
    // Pi to full precision
    const double kPi = 3.14159265358979323846;

    // Convert degrees to radians
    inline float degreesToRadians(double degrees) {
        return degrees * (kPi / 180.0);
    }

    // Convert radians to degrees
    inline float radiansToDegrees(double radians) {
        return radians * (180.0 / kPi);
    }

    // Wrap a longitude value into the interval [-180, 180]
    inline float wrapLongitude(float lon) {
        // shift into [0, 360)
        float shifted = fmod(lon + 180.0, 360.0);
        if (shifted < 0.0)
            shifted += 360.0;
        // shift into [-180, 180)
        return shifted - 180.0;
    }

    // Compute the great-circle distance (haversine) between two lon/lat points
    inline float haversineDistance(float lon1, float lat1,
                                    float lon2, float lat2) {
        float radLat1 = degreesToRadians(lat1);
        float radLat2 = degreesToRadians(lat2);
        float deltaLat = degreesToRadians(lat2 - lat1);
        float deltaLon = degreesToRadians(wrapLongitude(lon2 - lon1));

        float a = sin(deltaLat * 0.5) * sin(deltaLat * 0.5)
                 + cos(radLat1) * cos(radLat2)
                 * sin(deltaLon * 0.5) * sin(deltaLon * 0.5);
        float c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
        return kEarthRadiusMeters * c;
    }
}

std::array<Spherical::Boundary,4>
CoordinateTraits<Spherical>::getChildBounds(const Boundary& b) {
    float halfWidth  = b.width  * 0.5;
    float halfHeight = b.height * 0.5;

    // Quadrants: northeast, northwest, southeast, southwest
    return {{
        // northeast: longitude plus halfWidth, latitude plus halfHeight
        { wrapLongitude(b.x + halfWidth),
          b.y + halfHeight,
          halfWidth,
          halfHeight },

        // northwest: longitude minus halfWidth, latitude plus halfHeight
        { wrapLongitude(b.x - halfWidth),
          b.y + halfHeight,
          halfWidth,
          halfHeight },

        // southeast: longitude plus halfWidth, latitude minus halfHeight
        { wrapLongitude(b.x + halfWidth),
          b.y - halfHeight,
          halfWidth,
          halfHeight },

        // southwest: longitude minus halfWidth, latitude minus halfHeight
        { wrapLongitude(b.x - halfWidth),
          b.y - halfHeight,
          halfWidth,
          halfHeight }
    }};
}

bool CoordinateTraits<Spherical>::contains(const Boundary& b,
                                           float lon, float lat) {
    // Latitude containment is simple range check
    if (lat < (b.y - b.height) || lat > (b.y + b.height))
        return false;

    // Compute wrapped difference in longitude
    float deltaLon = wrapLongitude(lon - b.x);
    return (deltaLon >= -b.width && deltaLon <= b.width);
}

float CoordinateTraits<Spherical>::distanceToBounds(
    const Boundary& b,
    float lon, float lat, float cz,
    float elevation)
{
    // Horizontal distance in longitude direction first
    float deltaLon = wrapLongitude(lon - b.x);
    float excessLon = 0.0;
    if (deltaLon > b.width)
        excessLon = deltaLon - b.width;
    else if (deltaLon < -b.width)
        excessLon = -b.width - deltaLon;
    float metersLon = degreesToRadians(excessLon) * kEarthRadiusMeters;

    // Vertical distance in latitude direction
    float deltaLat = lat - b.y;
    float excessLat = 0.0;
    if (deltaLat > b.height)
        excessLat = deltaLat - b.height;
    else if (deltaLat < -b.height)
        excessLat = -b.height - deltaLat;
    float metersLat = degreesToRadians(excessLat) * kEarthRadiusMeters;

    // Elevation difference
    float dz = elevation - cz;

    return static_cast<float>(
        sqrt(metersLon * metersLon +
             metersLat * metersLat +
             dz * dz)
    );
}

std::array<Spherical::Boundary,4>
CoordinateTraits<Spherical>::subdivide(const Boundary& b) {
    return getChildBounds(b);
}

float CoordinateTraits<Spherical>::distance(
    Spherical::Position p1,
    Spherical::Position p2)
{
    // Use great-circle (haversine) distance, ignore elevation
    return static_cast<float>(
        haversineDistance(p1.lat, p1.lon, p2.lat, p2.lon)
    );
}

std::tuple<float,float,float>
CoordinateTraits<Spherical>::cartesianAt(
    const Boundary& b,
    int i, int j, int divisions,
    QuadTree<TileMetadata,Spherical>* tree,
    const GeoTIFFLoader* geoLoader)
{
    double stepLon = (2.0 * b.width)  / divisions;
    double stepLat = (2.0 * b.height) / divisions;
    double startLon = wrapLongitude(b.x - b.width);
    double startLat = b.y - b.height;

    double lon = wrapLongitude(startLon + i * stepLon);
    double lat = startLat  + j * stepLat;
    float z = getElevation(
        { static_cast<float>(lon), static_cast<float>(lat) },
        tree, geoLoader
    );

    // Convert to spatial coordinates
    // Note: lon and lat are in degrees, convert to radians
    double radLon = degreesToRadians(lon);
    double radLat = degreesToRadians(lat);
    double cosLat = cos(radLat);
    double sinLat = sin(radLat);
    double cosLon = cos(radLon);
    double sinLon = sin(radLon);
    double x = kEarthRadiusMeters * cosLat * cosLon;
    double y = kEarthRadiusMeters * cosLat * sinLon;
    double zCoord = kEarthRadiusMeters * sinLat;
    // Add elevation
    z += static_cast<float>(zCoord);
    return { static_cast<float>(x),
             static_cast<float>(y),
             z };
}

float CoordinateTraits<Spherical>::computeBaseElevation(
    const Spherical::Position& pos,
    const GeoTIFFLoader* geoLoader)
{
    if (geoLoader == nullptr) {
        // Pure noise if no GeoTIFF
        return Perlin::noise(pos.lat * 0.1, pos.lon * 0.1);
    }

    const auto& gt = geoLoader->getGeoTransform();
    double originLon   = gt[0];  // top-left corner longitude
    double pixelWidth  = gt[1];  // degrees per pixel in X
    double originLat   = gt[3];  // top-left corner latitude
    double pixelHeight = gt[5];  // degrees per pixel in Y (usually negative)

    // Convert from geographic to image coordinates
    double colF = wrapLongitude(pos.lat - originLon) / pixelWidth;
    double rowF =       (pos.lon - originLat) / pixelHeight;

    int col0 = static_cast<int>(floor(colF));
    int row0 = static_cast<int>(floor(rowF));
    int col1 = col0 + 1;
    int row1 = row0 + 1;

    int w = geoLoader->getWidth();
    int h = geoLoader->getHeight();
    const auto& data = geoLoader->getElevationData();

    float interpValue = 0.0f;
    if (col0 >= 0 && row0 >= 0 && col1 < w && row1 < h) {
        float v00 = data[row0 * w + col0];
        float v10 = data[row0 * w + col1];
        float v01 = data[row1 * w + col0];
        float v11 = data[row1 * w + col1];
        double tx = colF - col0;
        double ty = rowF - row0;
        interpValue = static_cast<float>(
            (1.0 - tx) * (1.0 - ty) * v00 +
             tx      * (1.0 - ty) * v10 +
            (1.0 - tx) * ty       * v01 +
             tx      * ty         * v11
        );
    }

    // Add Perlin noise
    float alpha     = 0.7f;
    float frequency = 0.1f;
    float amplitude = 0.25f;
    float noise = Perlin::noise(pos.lat * frequency, pos.lon * frequency) * amplitude;

    return interpValue + alpha * noise;
}

QuadTree<TileMetadata,Spherical>*
CoordinateTraits<Spherical>::findLeafNode(
    QuadTree<TileMetadata,Spherical>* node,
    Spherical::Position pos)
{
    if (node == nullptr)
        return nullptr;

    Boundary b = node->getBoundary();
    if (!contains(b, pos.lat, pos.lon))
        return nullptr;

    if (!node->isDivided())
        return node;

    // Recurse into each child
    QuadTree<TileMetadata,Spherical>* children[4] = {
        node->getNortheastNonConst(),
        node->getNorthwestNonConst(),
        node->getSoutheastNonConst(),
        node->getSouthwestNonConst()
    };

    for (int k = 0; k < 4; ++k) {
        if (QuadTree<TileMetadata,Spherical>* found =
            findLeafNode(children[k], pos))
        {
            return found;
        }
    }
    return nullptr;
}

float CoordinateTraits<Spherical>::getElevation(
    const Spherical::Position& pos,
    QuadTree<TileMetadata,Spherical>* tree,
    const GeoTIFFLoader* geoLoader)
{
    float baseHeight = computeBaseElevation(pos, geoLoader);
    QuadTree<TileMetadata,Spherical>* leaf = findLeafNode(tree, pos);
    if (leaf == nullptr)
        return baseHeight;

    const TileMetadata* meta = leaf->getType();
    if (meta->dirtyVertices.empty())
        return baseHeight;

    float sum = 0.0f;
    float weightSum = 0.0f;
    // Radius in degrees for blending
    const float blendRadiusDeg = 0.1f;

    for (auto const& dp : meta->dirtyVertices) {
        double dMeters = haversineDistance(
            pos.lat, pos.lon,
            dp.x,  dp.y
        );
        // Convert radius degrees to meters
        double radiusMeters = degreesToRadians(blendRadiusDeg)
                              * kEarthRadiusMeters;
        if (dMeters < radiusMeters) {
            float w = 1.0f / static_cast<float>(dMeters);
            sum += dp.z * w;
            weightSum += w;
        }
    }

    return (weightSum > 0.0f)
           ? (sum / weightSum)
           : baseHeight;
}

std::pair<int,int>
CoordinateTraits<Spherical>::computeTileIndices(
    const Position& pos,
    float tileSizeDegrees)
{
    int xIndex = static_cast<int>(
        floor(wrapLongitude(pos.lon) / tileSizeDegrees)
    );
    // Shift latitude range [-90,90] to [0,180] for indexing
    int yIndex = static_cast<int>(
        floor((pos.lat + 90.0f) / tileSizeDegrees)
    );
    return { xIndex, yIndex };
}

std::pair<float,float>
CoordinateTraits<Spherical>::tileCenterPosition(
    const TileKey& key,
    float tileSizeDegrees)
{
    // Longitude center: key.x * size + half size
    double lon = (key.x + 0.5) * tileSizeDegrees;
    // Latitude center: shift back from [0,180] to [-90,90]
    double lat = (key.y + 0.5) * tileSizeDegrees - 90.0;
    return { static_cast<float>(wrapLongitude(lon)),
             static_cast<float>(lat) };
}
