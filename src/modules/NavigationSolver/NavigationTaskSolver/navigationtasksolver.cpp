#include "navigationtasksolver.h"
#include "modules/IO/datamanager.h"

#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QtMath>

using namespace navsolver;

NavigationTaskSolver::NavigationTaskSolver(const Config& cfg) : cfg_(cfg) {}

// Локальная структура для формирования двойного CSV
struct DualNavSolution {
   NavSolution absMode;
   NavSolution sbasMode;
};

bool NavigationTaskSolver::execute(pipeline::Context& ctx) {
   ctx.solutions.clear();

   if (!ctx.dm || ctx.residualErrors.isEmpty()) {
      qWarning() << "[NavigationTaskSolver] Нет данных для расчета.";
      return false;
   }

   WeightModel weightModel(ctx.dm->getSBASStore(), cfg_.weightCfg);
   const auto* navOrig = ctx.dm->getRinexFile();

   // Локальное хранилище для двойного решения
   QMap<QDateTime, QMap<GRID_POINT, DualNavSolution> > dualSolutions;

   for (auto itEpoch = ctx.residualErrors.constBegin(); itEpoch != ctx.residualErrors.constEnd(); ++itEpoch) {
      const QDateTime& epoch = itEpoch.key();

      for (auto itPoint = itEpoch.value().constBegin(); itPoint != itEpoch.value().constEnd(); ++itPoint) {
         const GRID_POINT& point = itPoint.key();
         const auto& residuals   = itPoint.value();

         // 1. Сбор геометрии
         QVector<GeometryBuilder::SatInput> satInputs;
         satInputs.reserve(residuals.size());

         for (const auto& res : residuals) {
            COORD_XYZ satEcef = navOrig->navRecords.value(epoch).value(res.satellite).coord;

            // Приведение км -> м
            const double norm = std::sqrt(satEcef.x * satEcef.x +
                                          satEcef.y * satEcef.y +
                                          satEcef.z * satEcef.z);

            if (std::isfinite(norm) && (norm > 1e3) && (norm < 1e6)) {
               satEcef.x *= 1000.0;
               satEcef.y *= 1000.0;
               satEcef.z *= 1000.0;
            }

            satInputs.push_back({ res.satellite, satEcef });
         }

         GeometryBuilder geomBuilder(cfg_.geomCfg);
         const auto geoms        = geomBuilder.compute(point, satInputs);
         const auto weightResult = weightModel.buildRDiagonal(epoch, point, geoms);

         // 2. Формирование двух наборов наблюдений
         QVector<LeastSquaresSolver::Observation> absObs;  // Абсолютный режим
         QVector<LeastSquaresSolver::Observation> sbasObs; // Режим СДКМ/WLS

         absObs.reserve(geoms.size());
         sbasObs.reserve(geoms.size());

         for (const auto& geom : geoms) {
            auto itRes = std::find_if(residuals.constBegin(), residuals.constEnd(),
                                      [&geom](const ResidualError& r) {
               return r.satellite == geom.sat;
            });

            if (itRes == residuals.constEnd()) {
               continue;
            }

            LeastSquaresSolver::Observation base;
            base.H[0] = -geom.los.x;
            base.H[1] = -geom.los.y;
            base.H[2] = -geom.los.z;
            base.H[3] = 1.0;

            // Абсолютный режим: deltaSp3
            if (qIsFinite(itRes->deltaSp3)) {
               LeastSquaresSolver::Observation oAbs = base;
               oAbs.y = itRes->deltaSp3;
               absObs.push_back(oAbs);
            }

            // Режим СДКМ/WLS: residual + sigma2
            if (!qIsFinite(itRes->residual)) {
               continue;
            }

            auto itWeight = std::find_if(weightResult.weights.constBegin(), weightResult.weights.constEnd(),
                                         [&geom](const WeightModel::SatWeight& w) {
               return w.sat == geom.sat;
            });

            if (itWeight == weightResult.weights.constEnd()) {
               continue;
            }

            if (!qIsFinite(itWeight->sigma2_m2) || (itWeight->sigma2_m2 <= 0.0)) {
               continue;
            }

            LeastSquaresSolver::Observation oSbas = base;
            oSbas.y      = itRes->residual;
            oSbas.sigma2 = qMax(itWeight->sigma2_m2, cfg_.weightCfg.minSigma2_m2);
            sbasObs.push_back(oSbas);
         }

         // 3. Хелпер решения
         auto solveMode = [&](const QVector<LeastSquaresSolver::Observation>& obs,
                              const LeastSquaresSolver::Options& opt) -> NavSolution {
                             NavSolution sol;

                             if (obs.size() < 4) {
                                return sol;
                             }

                             const auto res = LeastSquaresSolver::solve(obs, opt);

                             sol.num_sats  = res.success ? res.m : 0;
                             sol.converged = res.success;

                             if (!res.success) {
                                return sol;
                             }

                             sol.delta_pos_ecef = { res.dx[0], res.dx[1], res.dx[2] };
                             sol.delta_clk_s    = res.dx[3] / 299792458.0;
                             sol.postfit_rms    = res.postfit_rms;
                             sol.err3d          = std::sqrt(res.dx[0] * res.dx[0] +
                                                            res.dx[1] * res.dx[1] +
                                                            res.dx[2] * res.dx[2]);

                             const auto R_enu     = GeometryBuilder::rotationEcefToEnu(point.llh);
                             const COORD_ENU denu = GeometryBuilder::apply(R_enu, sol.delta_pos_ecef);

                             sol.horiz_err = std::sqrt(denu.east * denu.east + denu.north * denu.north);
                             sol.vert_err  = std::abs(denu.up);
                             sol.dop       = computeDop(obs, point);

                             return sol;
                          };

         // 4. Решение двух штатных режимов
         DualNavSolution dSol;
         dSol.absMode  = solveMode(absObs, cfg_.olsOpt);
         dSol.sbasMode = solveMode(sbasObs, cfg_.wlsOpt);

         dualSolutions[epoch][point] = dSol;

         // В контекст отдаём приоритетно решение СДКМ, иначе фолбэк на абсолютное
         if (dSol.sbasMode.converged) {
            ctx.solutions[epoch][point] = dSol.sbasMode;
         } else if (dSol.absMode.converged) {
            ctx.solutions[epoch][point] = dSol.absMode;
         }
      }
   }

   // 5. Экспорт двойного CSV
   if (cfg_.writeCsv) {
      QFile file(cfg_.csvPath);

      if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
         QTextStream out(&file);
         out << "Epoch\tLat\tLon\tPDOP\tAbs_Sats\tAbs_Err3D\tAbs_Horiz\tAbs_Vert\tSbas_Sats\tSbas_Err3D\tSbas_Horiz\tSbas_Vert\n";

         for (auto itEpoch = dualSolutions.constBegin(); itEpoch != dualSolutions.constEnd(); ++itEpoch) {
            for (auto itPoint = itEpoch.value().constBegin(); itPoint != itEpoch.value().constEnd(); ++itPoint) {
               const auto& sol = itPoint.value();

               if (!sol.absMode.converged && !sol.sbasMode.converged) {
                  continue;
               }

               out << itEpoch.key().toString(Qt::ISODate) << "\t"
                   << QString::number(itPoint.key().llh.latitude, 'f', 4) << "\t"
                   << QString::number(itPoint.key().llh.longitude, 'f', 4) << "\t"
                   << QString::number(sol.absMode.converged ? sol.absMode.dop.PDOP
                                                                 : sol.sbasMode.dop.PDOP, 'f', 2) << "\t"
                   << sol.absMode.num_sats << "\t"
                   << (sol.absMode.converged ? QString::number(sol.absMode.err3d, 'f', 3) : "NaN") << "\t"
                   << (sol.absMode.converged ? QString::number(sol.absMode.horiz_err, 'f', 3) : "NaN") << "\t"
                   << (sol.absMode.converged ? QString::number(sol.absMode.vert_err, 'f', 3) : "NaN") << "\t"
                   << sol.sbasMode.num_sats << "\t"
                   << (sol.sbasMode.converged ? QString::number(sol.sbasMode.err3d, 'f', 3) : "NaN") << "\t"
                   << (sol.sbasMode.converged ? QString::number(sol.sbasMode.horiz_err, 'f', 3) : "NaN") << "\t"
                   << (sol.sbasMode.converged ? QString::number(sol.sbasMode.vert_err, 'f', 3) : "NaN") << "\n";
            }
         }
      }
   }

   qInfo() << "[NavigationTaskSolver] Расчет завершен. Двойной CSV сохранен.";
   return true;
}

DOP NavigationTaskSolver::computeDop(const QVector<LeastSquaresSolver::Observation>& obs,
                                     const GRID_POINT&                               user) const noexcept {
   DOP dop;

   if (obs.size() < 4) {
      return dop;
   }
   // DOP зависит только от геометрии, поэтому решаем фиктивную задачу OLS
   // с опцией вычисления априорной ковариации (H^T H)^-1
   LeastSquaresSolver::Options dopOpt = cfg_.olsOpt;
   dopOpt.computeCovarianceApriori = true;

   auto res = LeastSquaresSolver::solve(obs, dopOpt);

   if (!res.success) {
      return dop;
   }

   const auto& Q = res.P_apriori; // Матрица (H^T H)^-1

   double pdop2 = qMax(0.0, Q[0][0] + Q[1][1] + Q[2][2]);
   dop.PDOP = std::sqrt(pdop2);
   dop.GDOP = std::sqrt(qMax(0.0, pdop2 + Q[3][3]));

   // Проекция 3x3 субматрицы Q в ENU: Q_enu = R * Q_xyz * R^T
   auto   R          = GeometryBuilder::rotationEcefToEnu(user.llh);
   double Qpos[3][3] = { { Q[0][0], Q[0][1], Q[0][2] },
      { Q[1][0], Q[1][1], Q[1][2] },
      { Q[2][0], Q[2][1], Q[2][2] } };
   double tmp[3][3]{};

   for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
         for (int k = 0; k < 3; ++k) {
            tmp[i][j] += Qpos[i][k] * R.t[j][k];
         }
      }
   }
   double Qenu[3][3]{};

   for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
         for (int k = 0; k < 3; ++k) {
            Qenu[i][j] += R.t[i][k] * tmp[k][j];
         }
      }
   }

   dop.HDOP = std::sqrt(qMax(0.0, Qenu[0][0] + Qenu[1][1]));
   dop.VDOP = std::sqrt(qMax(0.0, Qenu[2][2]));

   return dop;
}

void NavigationTaskSolver::exportToCsv(const pipeline::Context& ctx) const {
   QFile file(cfg_.csvPath);

   if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      qWarning() << "[NavigationTaskSolver] Ошибка создания CSV файла:" << cfg_.csvPath;
      return;
   }

   QTextStream out(&file);
   out << "Epoch,Lat,Lon,NumSats,Converged,Err3D_m,Horiz_m,Vert_m,PDOP,HDOP,VDOP,RMS_m\n";

   for (auto itEpoch = ctx.solutions.constBegin(); itEpoch != ctx.solutions.constEnd(); ++itEpoch) {
      for (auto itPoint = itEpoch.value().constBegin(); itPoint != itEpoch.value().constEnd(); ++itPoint) {
         const auto& sol = itPoint.value();
         out << itEpoch.key().toString(Qt::ISODate) << ","
             << QString::number(itPoint.key().llh.latitude, 'f', 4) << ","
             << QString::number(itPoint.key().llh.longitude, 'f', 4) << ","
             << sol.num_sats << ","
             << (sol.converged ? "1" : "0") << ","
             << QString::number(sol.err3d, 'f', 3) << ","
             << QString::number(sol.horiz_err, 'f', 3) << ","
             << QString::number(sol.vert_err, 'f', 3) << ","
             << QString::number(sol.dop.PDOP, 'f', 2) << ","
             << QString::number(sol.dop.HDOP, 'f', 2) << ","
             << QString::number(sol.dop.VDOP, 'f', 2) << ","
             << QString::number(sol.postfit_rms, 'f', 3) << "\n";
      }
   }
}
