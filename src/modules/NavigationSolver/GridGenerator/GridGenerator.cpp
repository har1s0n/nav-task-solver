#include "gridgenerator.h"

using namespace navsolver;

GridGenerator::GridGenerator(double lonStepDeg, ELLIPSOID::TYPE ellipsoid)
   : lonStep_(lonStepDeg), ellipsoid_(ellipsoid) {}

QVector<GRID_POINT> GridGenerator::generateGrid(double minLat, double maxLat,
                                                double minLon, double maxLon) const {
   QVector<GRID_POINT> grid;

   constexpr double baseLatStep  = 1.0;
   const double     referenceLat = minLat;

   for (double lat = minLat; lat <= maxLat;) {
      double cosLat          = qCos(qDegreesToRadians(lat));
      double cosRef          = qCos(qDegreesToRadians(referenceLat));
      double latStep         = baseLatStep;
      double adjustedLonStep = lonStep_;

      if (!qFuzzyCompare(cosLat, 0.0)) {
         adjustedLonStep = lonStep_ / cosLat * cosRef;
      }

      for (double lon = minLon; lon <= maxLon; lon += adjustedLonStep) {
         COORD_LLH llh(lat, lon, 0.0);
         COORD_XYZ xyz = Coordinates::convertLLH2XYZ(llh, ellipsoid_);
         grid.append(GRID_POINT{ llh, xyz });
      }

      lat += latStep;
   }

   return grid;
}

bool GridGenerator::execute(pipeline::Context& ctx) {
   ctx.gridPoints = std::move(generateGrid());
   return true;
}
