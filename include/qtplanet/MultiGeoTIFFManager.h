// ─── MultiGeoTIFFManager.h ──────────────────────────────────────────────────
#pragma once
#include "GeoTIFFLoader.h"
#include <memory>
#include <optional>
#include <vector>
#include <gdal_priv.h>
#include <ogr_spatialref.h>

class MultiGeoTIFFManager
{
public:
    bool addSource(const std::string& filename,
                   int minZoomInclusive,
                   int maxZoomInclusive);

    std::optional<double> sample(double lon /* deg E */,
                                double lat /* deg N */,
                                int    zoomLevel) const;

private:
    // ---------------------------------------------------------------------
    struct Source {
        std::shared_ptr<GeoTIFFLoader> loader;
        int minZoom, maxZoom;
        double minLon, maxLon, minLat, maxLat;      // always degrees
        bool isGeographic;
        std::unique_ptr<OGRCoordinateTransformation> toDataset; // MoonLL → ds
    };
    std::vector<Source> sources_;

    // One shared base CRS:   Moon_2000 lon/lat (Plate‑Carrée)
    static OGRSpatialReference makeMoonLonLat()
    {
        OGRSpatialReference s;
        s.SetGeogCS("GCS_Moon_2000",
                    "D_Moon_2000",
                    "Moon_2000_IAU_IAG",
                    1737400.0,           // radius (m)
                    0.0);               // flattening
        s.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
        return s;
    }
    const OGRSpatialReference moonLL_ = makeMoonLonLat();
};

// ─── implementation ─────────────────────────────────────────────────────────
inline bool MultiGeoTIFFManager::addSource(const std::string& file,
                                           int minZoom, int maxZoom)
{
    auto ldr = std::make_shared<GeoTIFFLoader>();
    try { ldr->load(file); }
    catch (const std::exception& e) {
        std::cerr << "[MGTM] load failed: " << e.what() << '\n';
        return false;
    }    


    // 1.  Get dataset CRS ---------------------------------------------------
    GDALDataset *raw = static_cast<GDALDataset*>(
                       GDALOpen(file.c_str(), GA_ReadOnly));
    OGRSpatialReference dsSRS = ldr->getDatasetSRS(raw);
    GDALClose(raw);


    dsSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    bool dsIsGeo = dsSRS.IsGeographic();


    // 2.  Build Moon‑LL → dataset transform (only if needed) ---------------
    std::unique_ptr<OGRCoordinateTransformation> fwd;
    if (!dsIsGeo) {
        fwd.reset(OGRCreateCoordinateTransformation(&moonLL_, &dsSRS));
        if (!fwd) {
            std::cerr << "[MGTM] can't build transform into " << file << '\n';
            return false;
        }
    }

    // 3.  Map raster corners to lon/lat for fast extent check --------------
    const auto& gt = ldr->getGeoTransform();
    int w = ldr->getWidth(), h = ldr->getHeight();

    std::array<double,4> px = {0, double(w-1), 0, double(w-1)};
    std::array<double,4> py = {0, 0, double(h-1), double(h-1)};
    std::array<double,4> lon, lat;

    std::unique_ptr<OGRCoordinateTransformation> toDeg(
        dsIsGeo ? nullptr
                : std::unique_ptr<OGRCoordinateTransformation>(
                      OGRCreateCoordinateTransformation(&dsSRS,&moonLL_)));

    for (int i = 0; i < 4; ++i) {
        lon[i] = gt[0] + px[i]*gt[1] + py[i]*gt[2];
        lat[i] = gt[3] + px[i]*gt[4] + py[i]*gt[5];
        if (toDeg) toDeg->Transform(1, &lon[i], &lat[i]);
    }
    auto mm = [](auto a, auto b, auto c, auto d, bool min)
              { return min ? std::min({a,b,c,d}) : std::max({a,b,c,d}); };
    double minLon = mm(lon[0], lon[1], lon[2], lon[3], true);
    double maxLon = mm(lon[0], lon[1], lon[2], lon[3], false);
    double minLat = mm(lat[0], lat[1], lat[2], lat[3], true);
    double maxLat = mm(lat[0], lat[1], lat[2], lat[3], false);

    sources_.push_back({ldr, minZoom, maxZoom,
                        minLon, maxLon, minLat, maxLat,
                        dsIsGeo, std::move(fwd)});
    
    std::cout << "[MGTM] added source: " << file << '\n';
    return true;
}

// --------------------------------------------------------------------------
// --------------------------------------------------------------------------
inline std::optional<double>
MultiGeoTIFFManager::sample(double lonDeg, double latDeg, int zoom) const
{
    // ------------------------------------------------------------------ ①
    // Helper that selects the best source among a collection of candidates
    auto pickBest = [&](const std::vector<const Source*>& cand) -> const Source*
    {
        const Source* best = nullptr;
        int           bestSpan = std::numeric_limits<int>::max();
        for (const Source* s : cand) {
            int span = s->maxZoom - s->minZoom;          // prefer narrower range
            if (span < bestSpan) { bestSpan = span; best = s; }
        }
        return best;
    };

    // ------------------------------------------------------------------ ②
    // Gather all sources covering the point, split by “contains the zoom”
    std::vector<const Source*> exact, lower;
    for (const auto& s : sources_) {
        if (latDeg < s.minLat || latDeg > s.maxLat) continue;

        double lonWrap = lonDeg;
        if (lonWrap < s.minLon) lonWrap += 360.0;
        if (lonWrap > s.maxLon) lonWrap -= 360.0;
        if (lonWrap < s.minLon || lonWrap > s.maxLon) continue;

        if (zoom >= s.minZoom && zoom <= s.maxZoom)
            exact.push_back(&s);
        else if (s.maxZoom < zoom)                      // strictly coarser
            lower.push_back(&s);
    }

    // Pick the best exact‑match source first; otherwise best lower‑zoom one.
    const Source* src = pickBest(exact);
    if (!src) {
        // choose the lower‑zoom source whose maxZoom is *closest* below request
        std::stable_sort(lower.begin(), lower.end(),
                         [](const Source* a, const Source* b)
                         { return a->maxZoom > b->maxZoom; });
        src = pickBest(lower);
    }
    if (!src) return std::nullopt;

    // ------------------------------------------------------------------ ③ (unchanged)
    double x = lonDeg, y = latDeg;
    if (!src->isGeographic) {
        if (!src->toDataset->Transform(1, &x, &y)) return std::nullopt;
    }

    const auto& gt = src->loader->getGeoTransform();
    const double det = gt[1]*gt[5] - gt[2]*gt[4];
    if (std::abs(det) < 1e-12) return std::nullopt;

    double colF = ((x - gt[0])*gt[5] - (y - gt[3])*gt[2]) / det;
    double rowF = (-(x - gt[0])*gt[4] + (y - gt[3])*gt[1]) / det;

    int col0 = int(std::floor(colF)), row0 = int(std::floor(rowF));
    int col1 = col0 + 1,              row1 = row0 + 1;
    int w = src->loader->getWidth(),  h  = src->loader->getHeight();
    if (col0 < 0 || row0 < 0 || col1 >= w || row1 >= h) return std::nullopt;

    const auto& d = src->loader->getElevationData();
    double v00 = d[row0*w + col0], v10 = d[row0*w + col1];
    double v01 = d[row1*w + col0], v11 = d[row1*w + col1];

    double tx = colF - col0, ty = rowF - row0;
    return (1-tx)*(1-ty)*v00 + tx*(1-ty)*v10 +
           (1-tx)*ty*v01   + tx*ty*v11;
}
