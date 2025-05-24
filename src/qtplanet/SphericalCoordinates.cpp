#include "qtplanet/SphericalCoordinates.h"
#include "qtplanet/CartesianCoordinates.h"
#include "qtplanet/Quadtree.h"
#include "qtplanet/TileMetadata.h"
#include <cmath>
#include <algorithm>
#include <limits>

std::array<Spherical::Boundary, 4>
CoordinateTraits<Spherical>::getChildBounds(const Boundary& b) {
    double halfWidth  = b.width  * 0.5;
    double halfHeight = b.height * 0.5;

    return {{
        clampAtPoles({ wrapLongitude(b.x + halfWidth), b.y + halfHeight, halfWidth, halfHeight }),
        clampAtPoles({ wrapLongitude(b.x - halfWidth), b.y + halfHeight, halfWidth, halfHeight }),
        clampAtPoles({ wrapLongitude(b.x + halfWidth), b.y - halfHeight, halfWidth, halfHeight }),
        clampAtPoles({ wrapLongitude(b.x - halfWidth), b.y - halfHeight, halfWidth, halfHeight })
    }};
}

bool CoordinateTraits<Spherical>::contains(const Boundary& b, double lon, double lat) {
    if (lat < (b.y - b.height) || lat > (b.y + b.height))
        return false;

    double deltaLon = wrapLongitude(lon - b.x);
    return (deltaLon >= -b.width && deltaLon <= b.width);
}

double CoordinateTraits<Spherical>::distanceToBounds(
    const Boundary& b,
    double camX, double camY, double camZ,
    double tileElev)
{
    // 1) Clamp lon/lat to the tile rectangle
    auto [camLon, camLat] = projectXYZToLatLon(camX, camY, camZ);
    double clampedLat = std::clamp(camLat, b.y - b.height, b.y + b.height);
    double deltaLonRaw = wrapLongitude(camLon - b.x);
    double clampedDeltaLon = std::clamp(deltaLonRaw, -b.width, b.width);
    double clampedLon = wrapLongitude(b.x + clampedDeltaLon);

    // 2) Convert that clamped point back into 3D on the sphere+tile elevation
    double radLon = degreesToRadians(clampedLon);
    double radLat = degreesToRadians(clampedLat);
    double rSurf  = kPlanetRadiusMeters + tileElev;
    double bx = rSurf * std::cos(radLat) * std::cos(radLon);
    double by = rSurf * std::cos(radLat) * std::sin(radLon);
    double bz = rSurf * std::sin(radLat);

    // 3) Straight‐line (chord) distance in 3D
    double dx = camX - bx;
    double dy = camY - by;
    double dz = camZ - bz;
    return static_cast<double>(std::sqrt(dx*dx + dy*dy + dz*dz));
}

double CoordinateTraits<Spherical>::distanceToBoundsLatLon(
    const Boundary& b,
    double lon, double lat,
    double camElev, double elevation)
{
    // 1) Find the nearest point on the rectangle [b.x±b.width]×[b.y±b.height]
    double clampedLat = std::clamp(lat, b.y - b.height, b.y + b.height);
    double deltaLonRaw = wrapLongitude(lon - b.x);
    double clampedDeltaLon = std::clamp(deltaLonRaw, -b.width, b.width);
    double clampedLon = wrapLongitude(b.x + clampedDeltaLon);

    // 2) Great-circle distance on sphere of radius R+elevation
    double effectiveR = kPlanetRadiusMeters + elevation;
    double horizontal = haversineDistance(lon, lat, clampedLon, clampedLat)
                     * static_cast<double>(effectiveR);

    // 3) Vertical difference
    double dz = camElev - elevation;

    return std::sqrt(horizontal*horizontal + dz*dz);
}


std::array<Spherical::Boundary, 4>
CoordinateTraits<Spherical>::subdivide(const Boundary& b) {
    return getChildBounds(b);
}

double CoordinateTraits<Spherical>::distance(Position p1, Position p2) {
    return haversineDistance(p1.lon, p1.lat, p2.lon, p2.lat);
}

std::tuple<double, double, double>
CoordinateTraits<Spherical>::cartesianAt(
    const Boundary& b, int i, int j, int divisions,
    QuadTree<TileMetadata, Spherical>* tree,
    const std::shared_ptr<MultiGeoTIFFManager>& geoLoader,
    int zoomLevel
) {
    double stepLon = (2.0 * b.width)  / divisions;
    double stepLat = (2.0 * b.height) / divisions;
    double startLon = CoordinateTraits<Spherical>::wrapLongitude(b.x - b.width);
    double startLat = b.y - b.height;

    double lon = CoordinateTraits<Spherical>::wrapLongitude(startLon + i * stepLon);
    double lat = startLat + j * stepLat;

    double z = getElevation({ lon,lat }, tree, geoLoader, zoomLevel);

    double radLon = degreesToRadians(lon);
    double radLat = degreesToRadians(lat);

    double dirX = cos(radLat) * cos(radLon);
    double dirY = cos(radLat) * sin(radLon);
    double dirZ = sin(radLat);

    double r = kPlanetRadiusMeters + z;          // z == elevation
    return { static_cast<double>(r * dirX),
            static_cast<double>(r * dirY),
            static_cast<double>(r * dirZ) };

}


double CoordinateTraits<Spherical>::computeBaseElevation(
    const Spherical::Position& pos,
    const std::shared_ptr<MultiGeoTIFFManager>& geoLoader,
    int zoomLevel
) {
    if (!geoLoader)
        return Perlin::noise(pos.lat * 0.1, pos.lon * 0.1);

    // int zoomLevel = 0; 

    auto maybe = geoLoader->sample(pos.lon, pos.lat, zoomLevel); 
    double interpValue = maybe.value_or(
            Perlin::noise(pos.lat * 0.1, pos.lon * 0.1));       // fallback
    
            // moon radius ratio
    interpValue *= 1.0; // 1737400m
            

    /* ---------- Isotropic 2‑D Perlin on the sphere --------- */
    auto dir = dirFromLonLat(pos.lon, pos.lat);   // helper from previous reply
    constexpr double freq       = 20.0;          // tweak for coarser/finer bumps
    constexpr double amplitude  = 1000.0;          // metres
    double noise = Perlin::perlinOnSphere2D(dir, freq) * amplitude;

    return interpValue + 0.7f * noise;
}

double CoordinateTraits<Spherical>::getElevation(
    const Position& pos,
    QuadTree<TileMetadata, Spherical>* tree,
    const std::shared_ptr<MultiGeoTIFFManager>& geoLoader,
    int zoomLevel
) {
    double baseHeight = computeBaseElevation(pos, geoLoader, zoomLevel);
    QuadTree<TileMetadata, Spherical>* leaf = findLeafNode(tree, pos);
    if (!leaf)
        return baseHeight;

    const TileMetadata* meta = leaf->getType();
    if (meta->dirtyVertices.empty())
        return baseHeight;

    double sum = 0.0f, weightSum = 0.0f;
    const double blendRadiusDeg = 0.1f;
    double radiusMeters = degreesToRadians(blendRadiusDeg) * kPlanetRadiusMeters;

    for (const auto& dp : meta->dirtyVertices) {
        double dMeters = haversineDistance(pos.lon, pos.lat, dp.x, dp.y);
        if (dMeters < radiusMeters) {
            double w = 1.0f / static_cast<double>(dMeters);
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
CoordinateTraits<Spherical>::computeTileIndices(const Position& pos, double tileSizeDegrees) {
    int xIndex = static_cast<int>(floor(wrapLongitude(pos.lon) / tileSizeDegrees));
    int yIndex = static_cast<int>(floor((pos.lat + 90.0) / tileSizeDegrees));
    return { xIndex, yIndex };
}

std::pair<double, double>
CoordinateTraits<Spherical>::tileCenterPosition(const TileKey& key, double tileSizeDegrees) {
    double lon = (key.x + 0.5) * tileSizeDegrees;
    double lat = (key.y + 0.5) * tileSizeDegrees - 90.0;
    return { static_cast<double>(wrapLongitude(lon)), static_cast<double>(lat) };
}
