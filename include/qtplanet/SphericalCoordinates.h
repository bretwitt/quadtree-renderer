#ifndef QTPLANET_COORDINATE_TRAITS_SPHERICAL_H
#define QTPLANET_COORDINATE_TRAITS_SPHERICAL_H

#include <array>
#include <tuple>
#include <utility>
#include <cmath>
#include <algorithm>
#include "qtplanet/CartesianCoordinates.h"
#include "qtplanet/Quadtree.h"
#include "qtplanet/GeoTIFFLoader.h"
#include "qtplanet/TileMetadata.h"

const static double kEarthRadiusMeters = 100.0;
const static double kPi = 3.14159265358979323846;

template<>
class CoordinateTraits<Spherical> {
public:
    using Boundary = Spherical::Boundary;
    using Position = Spherical::Position;

    static std::array<Boundary, 4> getChildBounds(const Boundary& b);
    static std::array<Boundary, 4> subdivide(const Boundary& b);
    static bool contains(const Boundary& b, float lon, float lat);
    static float distanceToBounds(
        const Boundary& b,
        float camX, float camY, float camZ,
        float tileElev
    );
    static float distanceToBoundsLatLon(
        const Boundary& b,
        float lon, float lat,
        float cz, float elevation
    );
    static float distance(Position p1, Position p2);
    static float computeBaseElevation(const Position& pos, const GeoTIFFLoader* geoLoader);
    static float getElevation(const Position& pos,
                              QuadTree<TileMetadata, Spherical>* tree,
                              const GeoTIFFLoader* geoLoader);
    static std::tuple<float, float, float> cartesianAt(
        const Boundary& b, int i, int j, int divisions,
        QuadTree<TileMetadata, Spherical>* tree,
        const GeoTIFFLoader* geoLoader
    );
    static QuadTree<TileMetadata, Spherical>* findLeafNode(
        QuadTree<TileMetadata, Spherical>* node,
        Position pos
    );
    static std::pair<int, int> computeTileIndices(const Position& pos, float tileSizeDegrees);
    static std::pair<float, float> tileCenterPosition(const TileKey& key, float tileSizeDegrees);
    static std::pair<float,float> projectXYZToLatLon(float x, float y, float z) {
        float lat = radiansToDegrees(asin(z / kEarthRadiusMeters));
        float lon = radiansToDegrees(atan2(y, x));
        return { lon, lat };
    }

    // ---------------------------------------------------------
    // Helper: direction unit‑vector for a lon/lat pair
    // ---------------------------------------------------------
    static inline std::tuple<float,float,float> dirFromLonLat(float lonDeg, float latDeg)
    {
        double θ = degreesToRadians(lonDeg);
        double φ = degreesToRadians(latDeg);
        double cx = std::cos(φ);
        return { static_cast<float>(cx * std::cos(θ)),
                static_cast<float>(cx * std::sin(θ)),
                static_cast<float>(std::sin(φ)) };
    }


    inline static float wrapLongitude(float lon) {
        float shifted = fmod(lon + 180.0f, 360.0f);
        if (shifted < 0.0f) shifted += 360.0f;
        return shifted - 180.0f;
    }

    inline static float degreesToRadians(double degrees) {
        return degrees * (kPi / 180.0);
    }

    inline static float radiansToDegrees(double radians) {
        return radians * (180.0 / kPi);
    }
    // ------------------------------------------------------------
    // Map (lon,lat) → (u,v) so that every texel covers ~equal area
    //   u in [0,1]  wraps 360° in longitude
    //   v in [0,1]  comes from v = (sin lat + 1)/2
    // ------------------------------------------------------------
    inline static std::pair<float,float> sphericalUV(float lonDeg, float latDeg)
    {
        float u = CoordinateTraits<Spherical>::wrapLongitude(lonDeg) * (1.0f / 360.0f);               // 0 … 1
        float v = (std::sin(CoordinateTraits<Spherical>::degreesToRadians(latDeg)) + 1.0f) * 0.5f;    // 0 … 1
        return {u, v};
    }



    inline static float haversineDistance(float lon1, float lat1, float lon2, float lat2) {
        float radLat1 = degreesToRadians(lat1);
        float radLat2 = degreesToRadians(lat2);
        float deltaLat = degreesToRadians(lat2 - lat1);
        float deltaLon = degreesToRadians(wrapLongitude(lon2 - lon1));

        float a = sin(deltaLat * 0.5f) * sin(deltaLat * 0.5f) +
                  cos(radLat1) * cos(radLat2) *
                  sin(deltaLon * 0.5f) * sin(deltaLon * 0.5f);
        float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
        return kEarthRadiusMeters * c;
    }

    static Boundary clampAtPoles(Boundary b) {
        if (b.y > 90.0f)  b.y = 90.0f;
        if (b.y < -90.0f) b.y = -90.0f;

        if ((b.y + b.height) > 90.0f)
            b.height = 90.0f - b.y;
        if ((b.y - b.height) < -90.0f)
            b.height = b.y + 90.0f;

        b.x = wrapLongitude(b.x);
        return b;
    }
};

#endif // QTPLANET_COORDINATE_TRAITS_SPHERICAL_H
