#ifndef NAVIGATIONTASKSOLVER_H
#define NAVIGATIONTASKSOLVER_H

#include <QString>
#include <QVector>
#include <QMap>

#include "modules/IModule/imodule.h"
#include "geometry.h"
#include "weights.h"
#include "solver.h"

namespace navsolver {
class NavigationTaskSolver : public pipeline::IModule {
public:

   struct Config {
      bool preferWls     = true;
      bool fallbackToOls = true;

      GeometryBuilder::Config     geomCfg{};
      WeightModel::Config         weightCfg{};
      LeastSquaresSolver::Options wlsOpt{ LeastSquaresSolver::Mode::WLS, 1e-12, true, false };
      LeastSquaresSolver::Options olsOpt{ LeastSquaresSolver::Mode::OLS, 1e-12, true, false };

      enum ConstellationMode { GPS_GLO, GLO_ONLY, GPS_ONLY };
      ConstellationMode constellation = GLO_ONLY;

      bool    writeCsv = true;
      QString csvPath  = QStringLiteral("nav_solutions.csv");

      // Итоговый файл по «Описанию программы ПОЭХ», 7 характеристик услуги СДКМ
      bool    writePoehCsv = true;
      QString poehCsvPath  = QStringLiteral("poeh_accuracy.csv");
   };

   explicit NavigationTaskSolver(const Config& cfg);
   bool    execute(pipeline::Context& ctx) override;
   QString name() const override {
      return QStringLiteral("Навигационное решение (ВМНК/МНК)");
   }

private:

   DOP  computeDop(const QVector<LeastSquaresSolver::Observation>& obs,
                   const GRID_POINT&                               user) const noexcept;
   void exportToCsv(const pipeline::Context& ctx) const;

private:

   Config cfg_;
};
} // namespace navsolver

#endif // NAVIGATIONTASKSOLVER_H
