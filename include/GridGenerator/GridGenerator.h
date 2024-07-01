#ifndef GRID_GENERATOR_H
#define GRID_GENERATOR_H

#include <vector>

struct GridPoint {
   double longitude;
   double latitude;
   double height;
};

class GridGenerator {
public:

   GridGenerator() = default;
   std::vector<GridPoint> generateGrid();
   void                   convertToLambertProjection(double  longitude,
                                                     double  latitude,
                                                     double& x,
                                                     double& y);
};

#endif // GRID_GENERATOR_H
