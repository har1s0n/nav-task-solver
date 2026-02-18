#ifndef GRIDGENERATOR_H
#define GRIDGENERATOR_H

#include <QVector>
#include <inav/Coordinates>
#include "modules/IModule/imodule.h"

namespace navsolver {
/**
 * Модуль генерации географической сетки.
 * Возвращает массив точек GridPoint (LLH + ECEF) и сохраняет его в Context.
 */
class GridGenerator : public pipeline::IModule  {
public:

   explicit GridGenerator(double          lonStepDeg = 1.0,
                          ELLIPSOID::TYPE ellipsoid  = ELLIPSOID::TYPE::PZ90_11);

   QVector<GRID_POINT> generateGrid(double minLat = 41.0,
                                    double maxLat = 82.0,
                                    double minLon = 19.0,
                                    double maxLon = 170.0) const;

   QVector<GRID_POINT> generateGridLAEA(double minLat = 41.0,
                                        double maxLat = 82.0,
                                        double minLon = 19.0,
                                        double maxLon = 170.0,
                                        double stepKm = 100) const;
   bool    execute(pipeline::Context& ctx) override;
   QString name()   const override {
      return QStringLiteral("Генерация сетки");
   }

   static bool saveGridToCsv(const QString&             filePath,
                             const QVector<GRID_POINT>& grid);

private:

   double lonStep_;
   ELLIPSOID::TYPE ellipsoid_;
};
}

#endif // GRIDGENERATOR_H
