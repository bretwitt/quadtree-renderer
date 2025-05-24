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
#include "qtplanet/MultiGeoTIFFManager.h"
#include "qtplanet/TileMetadata.h"

const static double kPlanetRadiusMeters = 1737400.0;
const static double kPi = 3.14159265358979323846;

template<>
class CoordinateTraits<Spherical> {
public:
    using Boundary = Spherical::Boundary;
    using Position = Spherical::Position;

    static std::array<Boundary, 4> getChildBounds(const Boundary& b);
    static std::array<Boundary, 4> subdivide(const Boundary& b);
    static bool contains(const Boundary& b, double lon, double lat);
    static double distanceToBounds(
        const Boundary& b,
        double camX, double camY, double camZ,
        double tileElev
    );
    static double distanceToBoundsLatLon(
        const Boundary& b,
        double lon, double lat,
        double cz, double elevation
    );
    static double distance(Position p1, Position p2);
    static double computeBaseElevation(const Position& pos, const std::shared_ptr<MultiGeoTIFFManager>& geoLoader, int zoomLevel = 0);

    static double getElevation(const Position& pos,
                              QuadTree<TileMetadata, Spherical>* tree,
                              const std::shared_ptr<MultiGeoTIFFManager>& geoLoader,
                              int zoomLevel
                            );

    static std::tuple<double, double, double> cartesianAt(
        const Boundary& b, int i, int j, int divisions,
        QuadTree<TileMetadata, Spherical>* tree,
        const std::shared_ptr<MultiGeoTIFFManager>& geoLoader,
        int zoomLevel
    );
    static QuadTree<TileMetadata, Spherical>* findLeafNode(
        QuadTree<TileMetadata, Spherical>* node,
        Position pos
    );
    static std::pair<int, int> computeTileIndices(const Position& pos, double tileSizeDegrees);
    static std::pair<double, double> tileCenterPosition(const TileKey& key, double tileSizeDegrees);
    static std::pair<double,double> projectXYZToLatLon(double x, double y, double z) {
        double lat = radiansToDegrees(asin(z / kPlanetRadiusMeters));
        double lon = radiansToDegrees(atan2(y, x));
        return { lon, lat };
    }

    // ---------------------------------------------------------
    // Helper: direction unit‑vector for a lon/lat pair
    // ---------------------------------------------------------
    static inline std::tuple<double,double,double> dirFromLonLat(double lonDeg, double latDeg)
    {
        double lam = degreesToRadians(lonDeg);
        double phi = degreesToRadians(latDeg);
        double cx = std::cos(phi);
        return { static_cast<double>(cx * std::cos(lam)),
                static_cast<double>(cx * std::sin(lam)),
                static_cast<double>(std::sin(phi)) };
    }

    static inline double getElevationFromXYZ(double x, double y, double z) {
        // Convert to spherical coordinates
        double r = std::sqrt((x * x) + (y * y) + (z * z));
        double elevation = r - kPlanetRadiusMeters;
        // std::cout << "Elevation: " << elevation << " " << "Radius " << r << std::endl;
        return elevation;
    }

    inline static double wrapLongitude(double lon) {
        double shifted = fmod(lon + 180.0, 360.0);
        if (shifted < 0.0f) shifted += 360.0;
        return shifted - 180.0;
    }

    inline static double degreesToRadians(double degrees) {
        return degrees * (kPi / 180.0);
    }

    inline static double radiansToDegrees(double radians) {
        return radians * (180.0 / kPi);
    }
    // ------------------------------------------------------------
    // Map (lon,lat) → (u,v) so that every texel covers ~equal area
    //   u in [0,1]  wraps 360° in longitude
    //   v in [0,1]  comes from v = (sin lat + 1)/2
    // ------------------------------------------------------------
    inline static std::pair<double,double> sphericalUV(double lonDeg, double latDeg)
    {
        double u = CoordinateTraits<Spherical>::wrapLongitude(lonDeg) * (1.0f / 360.0f);               // 0 … 1
        double v = (std::sin(CoordinateTraits<Spherical>::degreesToRadians(latDeg)) + 1.0f) * 0.5f;    // 0 … 1
        return {u, v};
    }


    inline static double haversineDistance(double lon1, double lat1, double lon2, double lat2) {
        double radLat1 = degreesToRadians(lat1);
        double radLat2 = degreesToRadians(lat2);
        double deltaLat = degreesToRadians(lat2 - lat1);
        double deltaLon = degreesToRadians(wrapLongitude(lon2 - lon1));

        double a = sin(deltaLat * 0.5f) * sin(deltaLat * 0.5f) +
                  cos(radLat1) * cos(radLat2) *
                  sin(deltaLon * 0.5f) * sin(deltaLon * 0.5f);
        double c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
        return kPlanetRadiusMeters * c;
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
