#include "gridgenerator.h"

using namespace navsolver;

/** Центральный угол между (lat0,lon0) и (lat,lon) на сфере (в радианах) */
double centralAngleRad(double lat0Deg, double lon0Deg,
                       double latDeg,  double lonDeg) {
   double lat0 = qDegreesToRadians(lat0Deg);
   double lon0 = qDegreesToRadians(lon0Deg);
   double lat  = qDegreesToRadians(latDeg);
   double lon  = qDegreesToRadians(lonDeg);

   double dlon = lon - lon0;

   double sinLat0 = std::sin(lat0);
   double cosLat0 = std::cos(lat0);
   double sinLat  = std::sin(lat);
   double cosLat  = std::cos(lat);

   double arg = sinLat0 * sinLat + cosLat0 * cosLat * std::cos(dlon);

   if (arg >  1.0) {
      arg =  1.0;
   }

   if (arg < -1.0) {
      arg = -1.0;
   }

   return std::acos(arg);
}

/** Обратное преобразование LAEA: (x,y) -> (lat,lon) [градусы] */
void laeaInverse(double x, double y,
                 double lat0Deg, double lon0Deg,
                 double R,
                 double& outLatDeg, double& outLonDeg) {
   const double lat0 = qDegreesToRadians(lat0Deg);
   const double lon0 = qDegreesToRadians(lon0Deg);

   const double rho = std::sqrt(x * x + y * y);
   const double eps = 1e-12;

   if (rho < eps) {
      outLatDeg = lat0Deg;
      outLonDeg = lon0Deg;
      return;
   }

   const double c     = 2.0 * std::asin(rho / (2.0 * R));
   const double sin_c = std::sin(c);
   const double cos_c = std::cos(c);

   const double sinLat0 = std::sin(lat0);
   const double cosLat0 = std::cos(lat0);

   // широта
   double arg = cos_c * sinLat0 + (y * sin_c * cosLat0) / rho;

   if (arg >  1.0) {
      arg =  1.0;
   }

   if (arg < -1.0) {
      arg = -1.0;
   }
   const double lat = std::asin(arg);

   // долгота
   const double num   = x * sin_c;
   const double denom = rho * cosLat0 * cos_c - y * sinLat0 * sin_c;
   const double lon   = lon0 + std::atan2(num, denom);

   outLatDeg = qRadiansToDegrees(lat);
   outLonDeg = qRadiansToDegrees(lon);
}

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

QVector<GRID_POINT> GridGenerator::generateGridLAEA(double minLat,
                                                    double maxLat,
                                                    double minLon,
                                                    double maxLon,
                                                    double stepKm) const {
   QVector<GRID_POINT> grid;

   if ((maxLat <= minLat) || (maxLon <= minLon) || (stepKm <= 0.0)) {
      return grid;
   }

   // Центр проекции – центр прямоугольника области
   const double lat0 = 0.5 * (minLat + maxLat);
   const double lon0 = 0.5 * (minLon + maxLon);

   // Радиус Земли (по выбранному эллипсоиду)
   double R = constants::WGS84::a;

   switch (ellipsoid_) {
     case ELLIPSOID::TYPE::PZ90_11:
        R = constants::PZ_9011::a;
        break;
     case ELLIPSOID::TYPE::WGS84:
        R = constants::WGS84::a;
        break;
     default:
        R = constants::WGS84::a;
        break;
   }

   // 1. Центральный угол до углов области
   const double c1 = centralAngleRad(lat0, lon0, minLat, minLon);
   const double c2 = centralAngleRad(lat0, lon0, minLat, maxLon);
   const double c3 = centralAngleRad(lat0, lon0, maxLat, minLon);
   const double c4 = centralAngleRad(lat0, lon0, maxLat, maxLon);

   double cMax = std::max(std::max(c1, c2), std::max(c3, c4));

   // LAEA не определена в антиподе – чуть отрежем от π
   const double cMaxLimit = M_PI - 1e-6;

   if (cMax > cMaxLimit) {
      cMax = cMaxLimit;
   }

   // 2. Максимальный радиус диска в проекции
   const double rMax = 2.0 * R * std::sin(0.5 * cMax);

   // 3. Шаг в проекции (м)
   const double step_m = stepKm * 1000.0;

   const double fullSize = 2.0 * rMax;
   const int    nSteps   = static_cast<int> (std::floor(fullSize / step_m)) + 1;
   const double xStart   = -rMax;
   const double yStart   = -rMax;

   const double rMaxSq = rMax * rMax;
   const double epsR   = 1e-6 * rMax;

   for (int iy = 0; iy < nSteps; ++iy) {
      const double y = yStart + iy * step_m;

      for (int ix = 0; ix < nSteps; ++ix) {
         const double x = xStart + ix * step_m;

         const double rhoSq = x * x + y * y;

         if (rhoSq > (rMaxSq + epsR)) {
            continue; // вне диска LAEA
         }

         double latDeg = 0.0;
         double lonDeg = 0.0;
         laeaInverse(x, y, lat0, lon0, R, latDeg, lonDeg);

         // Обрезка по заданной области
         if ((latDeg < minLat) || (latDeg > maxLat) ||
             (lonDeg < minLon) || (lonDeg > maxLon)) {
            continue;
         }

         COORD_LLH llh(latDeg, lonDeg, 0.0);
         COORD_XYZ xyz = Coordinates::convertLLH2XYZ(llh, ellipsoid_);
         grid.append(GRID_POINT{ llh, xyz });
      }
   }

   return grid;
}

bool GridGenerator::execute(pipeline::Context& ctx) {
   // ctx.gridPoints = std::move(generateGrid());
   ctx.gridPoints = std::move(generateGridLAEA());
   // saveGridToCsv("/Users/tarasovevgenij/Documents/24.04.25_test/grid_points_LAEA_FROM_QT.csv", ctx.gridPoints);

   return true;
}

bool GridGenerator::saveGridToCsv(const QString& filePath, const QVector<GRID_POINT>& grid) {
   QFile file(filePath);

   if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      return false;
   }

   QTextStream out(&file);

   out << "latitude\tlongitude\n";

   for (const auto& point : grid) {
      out << QString::number(point.llh.latitude,  'f', 10) << '\t'
          << QString::number(point.llh.longitude, 'f', 10) /*<< ','
                                                              << QString::number(point.llh.height,    'f', 3)  << ','
                                                              << QString::number(point.ecef.x,        'f', 4)  << ','
                                                              << QString::number(point.ecef.y,        'f', 4)  << ','
                                                              << QString::number(point.ecef.z,        'f', 4)*/
          << '\n';
   }

   return true;
}
