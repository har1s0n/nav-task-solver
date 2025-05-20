#include "GridGenerator.h"
#include "data.h"
#include <cmath>


std::vector<GridPoint> GridGenerator::generateGrid() {
   std::vector<GridPoint> gridPoints;

   for (double longitude = -180.0; longitude <= 180.0; longitude += 1.0) {
      double latitudeStep = 1.0;

      for (double latitude = 85.0; latitude >= -85.0; latitude -= latitudeStep) {
         GridPoint gridPoint;
         gridPoint.longitude = longitude;
         gridPoint.latitude  = latitude;
         gridPoint.height    = 0.0;
         gridPoints.push_back(gridPoint);
      }
   }
   return gridPoints;
}

void GridGenerator::convertToLambertProjection(double longitude, double latitude, double& x, double& y) {
   // Пример реализации преобразования координат в равноугловую проекцию Ламберта
   double k0      = 0.9996;
   double phi0    = 64.0 * PI / 180.0;
   double lambda0 = 96.0 * PI / 180.0;
   double phi     = latitude * PI / 180.0;
   double lambda  = longitude * PI / 180.0;
   double e       = sqrt(E2);
   double n       = (1.0 - e * e * sin(phi) * sin(phi)) / (1.0 - e * e);
   double F       = 0.5 * (1.0 + n * sin(phi) * (1.0 + n / 4.0 * pow(sin(phi), 3)));
   double M       = A *
                    ((1.0 - e * e / 4.0 - 3.0 * e * e * e * e / 64.0) * phi - (3.0 * e / 2.0 - 27.0 * e * e * e / 32.0) * sin(phi) * cos(
                        phi) +
                     (21.0 * e * e / 16.0 - 55.0 * e * e * e * e / 32.0) * sin(2 * phi) * cos(2 * phi));
   double xi  = M / (k0 * F);
   double eta = n * (lambda - lambda0) * cos(phi);

   x = xi * sin(eta) + 500000.0;
   y = -xi* cos(eta) + 3000000.0;
}
