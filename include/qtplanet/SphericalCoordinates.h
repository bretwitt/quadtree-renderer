#ifndef QTPLANET_COORDINATE_TRAITS_SPHERICAL_H
#define QTPLANET_COORDINATE_TRAITS_SPHERICAL_H

#include <array>
#include <tuple>
#include <utility>
#include "qtplanet/CartesianCoordinates.h"  // for Spherical::Boundary, Spherical::Position
#include "qtplanet/Quadtree.h"
#include "qtplanet/GeoTIFFLoader.h"
#include "qtplanet/TileMetadata.h"


// Specialization of CoordinateTraits for Spherical (latitude/longitude) coordinates.
template<>
class CoordinateTraits<Spherical> {
public:
    using Boundary = Spherical::Boundary;
    using Position = Spherical::Position;
    // Get the four child quadrant bounds of a parent boundary.
    static std::array<Boundary, 4> getChildBounds(const Boundary& b);

    // Check if a lon/lat point lies within a given boundary (with wrapping).
    static bool contains(const Boundary& b, float lon, float lat);

    // Compute distance (in meters) from a point (lon,lat,cz) to the boundary box, using great-circle for horizontal.
    static float distanceToBounds(const Boundary& b,
                                  float lon, float lat,
                                  float cz, float elevation);

    // Alias for getChildBounds
    static std::array<Boundary, 4> subdivide(const Boundary& b);

    // Great-circle distance (meters) between two lon/lat positions.
    static float distance(Position p1, Position p2);

    // Compute the lon/lat and elevation at a subdivision grid index.
    static std::tuple<float, float, float>
    cartesianAt(const Boundary& b,
                int i, int j, int divisions,
                QuadTree<TileMetadata, Spherical>* tree,
                const GeoTIFFLoader* geoLoader);

    // Base elevation lookup (GeoTIFF bilinear plus Perlin noise) at a lon/lat.
    static float computeBaseElevation(const Position& pos,
                                      const GeoTIFFLoader* geoLoader);

    // Find the leaf quadtree node that contains the given lon/lat position.
    static QuadTree<TileMetadata, Spherical>*
    findLeafNode(QuadTree<TileMetadata, Spherical>* node,
                 Position pos);

    // Full elevation including dirty-vertex blending.
    static float getElevation(const Position& pos,
                              QuadTree<TileMetadata, Spherical>* tree,
                              const GeoTIFFLoader* geoLoader);

    // Compute integer tile indices from lon/lat using a fixed degree tile size.
    static std::pair<int, int>
    computeTileIndices(const Position& pos, float tileSizeDegrees);

    // Compute the lon/lat center position of a given integer tile key.
    static std::pair<float, float>
    tileCenterPosition(const TileKey& key, float tileSizeDegrees);
};

#endif // QTPLANET_COORDINATE_TRAITS_SPHERICAL_H
