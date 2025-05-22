#include "qtplanet/QuadtreeWorld.h"
#include "qtplanet/CoordinateSystems.h"
#include <unordered_set>
#include <algorithm>
#include <cmath>

//---------------------------------------------------------------------
// Constructor
//---------------------------------------------------------------------
QuadtreeWorld::QuadtreeWorld(float tileSz, int baseViewRange, float splitThr, float mergeThr)
    : tileSize(tileSz),
      viewRangeInTiles(baseViewRange),
      splitThreshold(splitThr),
      mergeThreshold(mergeThr)
{
    // Load the terrain height‑map (GeoTIFF).
    // loader.load("../resources/apollo.TIFF");
    loader.load("../resources/ldem_64_fixed.tif");
    // loader.load("../resources/Lunar_LRO_LOLAKaguya_DEMmerge_60N60S_512ppd.tif");
    // loader.load("../resources/WAC_CSHADE_0000N1800_128P_4WEB.tif");
}

//---------------------------------------------------------------------
// Destructor
//---------------------------------------------------------------------
QuadtreeWorld::~QuadtreeWorld()
{
    for (auto &p : tiles) {
        delete p.second;
    }
}

//---------------------------------------------------------------------
// update – called every frame
//---------------------------------------------------------------------
void QuadtreeWorld::update(float camX, float camY, float camZ)
{
    /* ----------------------------------------------------------
     * cache the Cartesian camera pos for LOD tests               */
    cameraX = camX;  cameraY = camY;  cameraZ = camZ;

    /* ----------------------------------------------------------
     * project to geodetic lon/lat/elev                           */
    auto [camLon, camLat] =
        CoordinateTraits<Spherical>::projectXYZToLatLon(camX, camY, camZ);
    float camElev =
        CoordinateTraits<Spherical>::getElevationFromXYZ(camX, camY, camZ);

    /* ----------------------------------------------------------
     * figure out which tile centre we sit in                     */
    auto [centreTileX, centreTileY] =
        CoordinateTraits<Spherical>::computeTileIndices({camLon, camLat}, tileSize);

    /* ----------------------------------------------------------
     * number of tiles around the camera we want alive            */
    const int extraPer10km = static_cast<int>(std::ceil(camElev / 10000.0f));
    const int dynRange = std::clamp(viewRangeInTiles + extraPer10km, 2, 90);

    /* ----------------------------------------------------------
     * helpers for wrapping/clamping indices                      */
    const int nLonTiles = static_cast<int>(std::round(360.0f / tileSize));
    const int nLatTiles = static_cast<int>(std::round(180.0f / tileSize));

    auto wrapX = [nLonTiles](int x) {
        return ((x % nLonTiles) + nLonTiles) % nLonTiles; // 0..nLonTiles-1
    };
    auto clampY = [nLatTiles](int y) {
        return std::clamp(y, 0, nLatTiles - 1);
    };

    /* ----------------------------------------------------------
     * mark which tiles we need this frame                        */
    std::unordered_set<TileKey> needed;

    for (int dy = -dynRange; dy <= dynRange; ++dy) {
        for (int dx = -dynRange; dx <= dynRange; ++dx) {
            TileKey key{ wrapX(centreTileX + dx), clampY(centreTileY + dy) };
            needed.insert(key);

            // spawn if absent
            if (tiles.find(key) == tiles.end()) {
                auto [cLon, cLat] =
                    CoordinateTraits<Spherical>::tileCenterPosition(key, tileSize);
                float half = tileSize * 0.5f;
                tiles[key] = new QuadtreeTile<Spherical>({cLon, cLat, half, half}, &loader);
            }
        }
    }

    /* ----------------------------------------------------------
     * cull tiles that scrolled out of the view window            */
    for (auto it = tiles.begin(); it != tiles.end(); ) {
        if (needed.find(it->first) == needed.end()) {
            delete it->second;
            it = tiles.erase(it);
        } else {
            ++it;
        }
    }

    /* ----------------------------------------------------------
     * let each tile adapt its internal quadtree                  */
    for (auto &p : tiles) {
        int dummy = 0;
        p.second->updateLOD(camX, camY, camZ, splitThreshold, mergeThreshold, dummy);
    }

    /* ----------------------------------------------------------
     * tick counters (merge/split cooldowns etc.)                 */
    for (auto &p : tiles) {
        p.second->tick();
    }
}

//---------------------------------------------------------------------
// getTotalTiles – active globe patches
//---------------------------------------------------------------------
int QuadtreeWorld::getTotalTiles() const { return static_cast<int>(tiles.size()); }

//---------------------------------------------------------------------
// getMemoryUsage – aggregate GPU/Sys mem of all tiles + height‑map
//---------------------------------------------------------------------
size_t QuadtreeWorld::getMemoryUsage() const
{
    size_t total = loader.getMemoryUsage();
    for (auto &p : tiles) total += p.second->getMemoryUsage();
    return total;
}

//---------------------------------------------------------------------
// getAllMeshes – collect renderable batches from all tiles
//---------------------------------------------------------------------
std::unordered_map<QuadTree<TileMetadata,Spherical>*, Mesh>
QuadtreeWorld::getAllMeshes()
{
    std::unordered_map<QuadTree<TileMetadata,Spherical>*, Mesh> out;
    for (auto &p : tiles) {
        auto sub = p.second->getMeshes();
        out.insert(sub.begin(), sub.end());
    }
    return out;
}

//---------------------------------------------------------------------
// deformVertex – raise/lower a point on the surface
//---------------------------------------------------------------------
void QuadtreeWorld::deformVertex(float lon, float lat, float dz)
{
    auto [tileX, tileY] =
        CoordinateTraits<Spherical>::computeTileIndices({lon, lat}, tileSize);
    TileKey key{ tileX, tileY };

    auto it = tiles.find(key);
    if (it != tiles.end()) {
        it->second->deformVertex({lon, lat}, dz);
    }
}

//---------------------------------------------------------------------
// getElevation – sample height at lon/lat in metres
//---------------------------------------------------------------------
float QuadtreeWorld::getElevation(float lon, float lat)
{
    auto [tileX, tileY] =
        CoordinateTraits<Spherical>::computeTileIndices({lon, lat}, tileSize);
    TileKey key{ tileX, tileY };

    auto it = tiles.find(key);
    return (it != tiles.end()) ? it->second->getElevation({lon, lat}) : -1.0f;
}

//---------------------------------------------------------------------
// helper: camera lon/lat and elevation
//---------------------------------------------------------------------
std::pair<float,float> QuadtreeWorld::getCameraPositionLonLat() const
{
    return CoordinateTraits<Spherical>::projectXYZToLatLon(cameraX, cameraY, cameraZ);
}

float QuadtreeWorld::getCameraPositionElevation() const
{
    return CoordinateTraits<Spherical>::getElevationFromXYZ(cameraX, cameraY, cameraZ);
}
