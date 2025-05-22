#include "qtplanet/SphericalCoordinates.h"
#include "qtplanet/CartesianCoordinates.h"
#include "qtplanet/Quadtree.h"
#include "qtplanet/GeoTIFFLoader.h"
#include "qtplanet/TileMetadata.h"
#include <cmath>
#include <algorithm>
#include <limits>

std::array<Spherical::Boundary, 4>
CoordinateTraits<Spherical>::getChildBounds(const Boundary& b) {
    float halfWidth  = b.width  * 0.5f;
    float halfHeight = b.height * 0.5f;

    return {{
        clampAtPoles({ wrapLongitude(b.x + halfWidth), b.y + halfHeight, halfWidth, halfHeight }),
        clampAtPoles({ wrapLongitude(b.x - halfWidth), b.y + halfHeight, halfWidth, halfHeight }),
        clampAtPoles({ wrapLongitude(b.x + halfWidth), b.y - halfHeight, halfWidth, halfHeight }),
        clampAtPoles({ wrapLongitude(b.x - halfWidth), b.y - halfHeight, halfWidth, halfHeight })
    }};
}

bool CoordinateTraits<Spherical>::contains(const Boundary& b, float lon, float lat) {
    if (lat < (b.y - b.height) || lat > (b.y + b.height))
        return false;

    float deltaLon = wrapLongitude(lon - b.x);
    return (deltaLon >= -b.width && deltaLon <= b.width);
}

float CoordinateTraits<Spherical>::distanceToBounds(
    const Boundary& b,
    float camX, float camY, float camZ,
    float tileElev)
{
    // 1) Clamp lon/lat to the tile rectangle
    auto [camLon, camLat] = projectXYZToLatLon(camX, camY, camZ);
    float clampedLat = std::clamp(camLat, b.y - b.height, b.y + b.height);
    float deltaLonRaw = wrapLongitude(camLon - b.x);
    float clampedDeltaLon = std::clamp(deltaLonRaw, -b.width, b.width);
    float clampedLon = wrapLongitude(b.x + clampedDeltaLon);

    // 2) Convert that clamped point back into 3D on the sphere+tile elevation
    double radLon = degreesToRadians(clampedLon);
    double radLat = degreesToRadians(clampedLat);
    double rSurf  = double(kEarthRadiusMeters) + tileElev;
    double bx = rSurf * std::cos(radLat) * std::cos(radLon);
    double by = rSurf * std::cos(radLat) * std::sin(radLon);
    double bz = rSurf * std::sin(radLat);

    // 3) Straight‐line (chord) distance in 3D
    double dx = camX - bx;
    double dy = camY - by;
    double dz = camZ - bz;
    return static_cast<float>(std::sqrt(dx*dx + dy*dy + dz*dz));
}



// float CoordinateTraits<Spherical>::distanceToBounds(
//     const Boundary& b,
//     float camX, float camY, float camZ,
//     float tileElev
// ) {
//     // double r = std::sqrt(camX * camX + camY * camY + camZ * camZ);
//     // float camElev = static_cast<float>(r - kEarthRadiusMeters);

//     // float s = camZ / r;
//     // s = std::clamp(s, -1.0f, 1.0f);
//     // float camLat = radiansToDegrees(std::asin(s));
//     // float camLon = wrapLongitude(radiansToDegrees(std::atan2(camY, camX)));

//     // use projectXYZToLatLon instead
//     auto [ camLon, camLat ] = projectXYZToLatLon(camX, camY, camZ);
//     auto camElev = getElevationFromXYZ(camX, camY, camZ);

//     return distanceToBoundsLatLon(b, camLon, camLat, camElev, tileElev);
// }

float CoordinateTraits<Spherical>::distanceToBoundsLatLon(
    const Boundary& b,
    float lon, float lat,
    float camElev, float elevation)
{
    // 1) Find the nearest point on the rectangle [b.x±b.width]×[b.y±b.height]
    float clampedLat = std::clamp(lat, b.y - b.height, b.y + b.height);
    float deltaLonRaw = wrapLongitude(lon - b.x);
    float clampedDeltaLon = std::clamp(deltaLonRaw, -b.width, b.width);
    float clampedLon = wrapLongitude(b.x + clampedDeltaLon);

    // 2) Great-circle distance on sphere of radius R+elevation
    double effectiveR = double(kEarthRadiusMeters) + elevation;
    float horizontal = haversineDistance(lon, lat, clampedLon, clampedLat)
                     * static_cast<float>(effectiveR);

    // 3) Vertical difference
    float dz = camElev - elevation;

    return std::sqrt(horizontal*horizontal + dz*dz);
}


std::array<Spherical::Boundary, 4>
CoordinateTraits<Spherical>::subdivide(const Boundary& b) {
    return getChildBounds(b);
}

float CoordinateTraits<Spherical>::distance(Position p1, Position p2) {
    return haversineDistance(p1.lon, p1.lat, p2.lon, p2.lat);
}

std::tuple<float, float, float>
CoordinateTraits<Spherical>::cartesianAt(
    const Boundary& b, int i, int j, int divisions,
    QuadTree<TileMetadata, Spherical>* tree,
    const GeoTIFFLoader* geoLoader
) {
    double stepLon = (2.0 * b.width)  / divisions;
    double stepLat = (2.0 * b.height) / divisions;
    double startLon = CoordinateTraits<Spherical>::wrapLongitude(b.x - b.width);
    double startLat = b.y - b.height;

    double lon = CoordinateTraits<Spherical>::wrapLongitude(startLon + i * stepLon);
    double lat = startLat + j * stepLat;

    float z = getElevation({ static_cast<float>(lon), static_cast<float>(lat) }, tree, geoLoader);

    double radLon = degreesToRadians(lon);
    double radLat = degreesToRadians(lat);

    double dirX = cos(radLat) * cos(radLon);
    double dirY = cos(radLat) * sin(radLon);
    double dirZ = sin(radLat);

    double r = kEarthRadiusMeters + z;          // z == elevation
    return { static_cast<float>(r * dirX),
            static_cast<float>(r * dirY),
            static_cast<float>(r * dirZ) };

}


float CoordinateTraits<Spherical>::computeBaseElevation(
    const Spherical::Position& pos,
    const GeoTIFFLoader* geoLoader
) {
    if (!geoLoader)
        return Perlin::noise(pos.lat * 0.1f, pos.lon * 0.1f);

    const auto& gt = geoLoader->getGeoTransform();
    double originLon = gt[0];
    double pixelWidth = gt[1];
    double originLat = gt[3];
    double pixelHeight = gt[5];

    // Compute the pixel coordinates of the position

    // double colF = CoordinateTraits<Spherical>::wrapLongitude(pos.lon - originLon) / pixelWidth;
    // double rowF = (pos.lat - originLat) / pixelHeight;
    double colF = (pos.lon - originLon) / pixelWidth;
    double rowF = (originLat - pos.lat) / std::abs(pixelHeight);

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
             tx       * (1.0 - ty) * v10 +
            (1.0 - tx) * ty        * v01 +
             tx       * ty         * v11
        );
    }
    
    /* ---------- Isotropic 2‑D Perlin on the sphere --------- */
    auto dir = dirFromLonLat(pos.lon, pos.lat);   // helper from previous reply
    constexpr float freq       = 20.0f;          // tweak for coarser/finer bumps
    constexpr float amplitude  = 20.0f;          // metres
    float noise = Perlin::perlinOnSphere2D(dir, freq) * amplitude;

    return interpValue;// + 0.7f * noise;
    // return 200.0;
}

float CoordinateTraits<Spherical>::getElevation(
    const Position& pos,
    QuadTree<TileMetadata, Spherical>* tree,
    const GeoTIFFLoader* geoLoader
) {
    float baseHeight = computeBaseElevation(pos, geoLoader);
    QuadTree<TileMetadata, Spherical>* leaf = findLeafNode(tree, pos);
    if (!leaf)
        return baseHeight;

    const TileMetadata* meta = leaf->getType();
    if (meta->dirtyVertices.empty())
        return baseHeight;

    float sum = 0.0f, weightSum = 0.0f;
    const float blendRadiusDeg = 0.1f;
    double radiusMeters = degreesToRadians(blendRadiusDeg) * kEarthRadiusMeters;

    for (const auto& dp : meta->dirtyVertices) {
        double dMeters = haversineDistance(pos.lon, pos.lat, dp.x, dp.y);
        if (dMeters < radiusMeters) {
            float w = 1.0f / static_cast<float>(dMeters);
            sum += dp.z * w;
            weightSum += w;
        }
    }

    return (weightSum > 0.0f) ? (sum / weightSum) : baseHeight;
}

QuadTree<TileMetadata, Spherical>*
CoordinateTraits<Spherical>::findLeafNode(
    QuadTree<TileMetadata, Spherical>* node,
    Position pos
) {
    if (!node || !contains(node->getBoundary(), pos.lon, pos.lat))
        return nullptr;

    if (!node->isDivided())
        return node;

    auto* found = findLeafNode(node->getNortheastNonConst(), pos);
    if (found) return found;
    found = findLeafNode(node->getNorthwestNonConst(), pos);
    if (found) return found;
    found = findLeafNode(node->getSoutheastNonConst(), pos);
    if (found) return found;
    found = findLeafNode(node->getSouthwestNonConst(), pos);
    if (found) return found;


    return nullptr;
}

std::pair<int, int>
CoordinateTraits<Spherical>::computeTileIndices(const Position& pos, float tileSizeDegrees) {
    int xIndex = static_cast<int>(floor(wrapLongitude(pos.lon) / tileSizeDegrees));
    int yIndex = static_cast<int>(floor((pos.lat + 90.0f) / tileSizeDegrees));
    return { xIndex, yIndex };
}

std::pair<float, float>
CoordinateTraits<Spherical>::tileCenterPosition(const TileKey& key, float tileSizeDegrees) {
    double lon = (key.x + 0.5) * tileSizeDegrees;
    double lat = (key.y + 0.5) * tileSizeDegrees - 90.0;
    return { static_cast<float>(wrapLongitude(lon)), static_cast<float>(lat) };
}
