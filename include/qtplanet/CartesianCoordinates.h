#ifndef QTPLANET_CARTESIANCOORDINATES_H
#define QTPLANET_CARTESIANCOORDINATES_H

#include <array>
#include <cmath>
#include <tuple>
#include "CoordinateSystems.h"
#include "TileMetadata.h"
#include "Perlin.h"

// forward declares
template<typename T, typename CoordSystem> class QuadTree;
class GeoTIFFLoader;

template<>
struct CoordinateTraits<Cartesian> {
  using Boundary = Cartesian::Boundary;
  using Position = Cartesian::Position;

  static std::array<Boundary,4> getChildBounds(const Boundary& b);
  static bool contains(const Boundary& b, float x, float y);
  static float distanceToBounds(const Boundary& b,
                                float cx, float cy, float cz,
                                float elevation);
  static std::array<Boundary,4> subdivide(const Boundary& b);
  static float distance(Position p1, Position p2);

  static std::tuple<float,float,float>
    cartesianAt(const Boundary& b,
                int i, int j, int divisions,
                QuadTree<TileMetadata,Cartesian>* tree,
                const GeoTIFFLoader* geoLoader);

  static float computeBaseElevation(const Position& pos,
                                    const GeoTIFFLoader* geoLoader);

  static QuadTree<TileMetadata,Cartesian>*
    findLeafNode(QuadTree<TileMetadata,Cartesian>* node,
                 Position pos);

  static float getElevation(const Position& pos,
                            QuadTree<TileMetadata,Cartesian>* tree,
                            const GeoTIFFLoader* geoLoader);

  static std::pair<float,float> tileCenterPosition(const TileKey& key, float tileSize);
  static std::pair<int,int> computeTileIndices(const Position& pos, float tileSize);

};

#endif // QTPLANET_CARTESIANCOORDINATES_H
