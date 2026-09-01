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

static void exportPoehCsv(
   const QString&                                             poehCsvPath,
   const pipeline::Context&                                   ctx,
   const QMap<QDateTime, QMap<GRID_POINT, DualNavSolution> >& dualSolutions) {
   QMap<GRID_POINT, double> sumSq;
   QMap<GRID_POINT, qint64> nTerms;

   for (auto itEpoch = ctx.residualErrors.constBegin();
        itEpoch != ctx.residualErrors.constEnd(); ++itEpoch) {
      for (auto itPoint = itEpoch.value().constBegin();
           itPoint != itEpoch.value().constEnd(); ++itPoint) {
         for (const auto& res : itPoint.value()) {
            if (!qIsFinite(res.residual)) {
               continue;
            }
            sumSq[itPoint.key()]  += res.residual * res.residual;
            nTerms[itPoint.key()] += 1;
         }
      }
   }

   QFile file(poehCsvPath);

   if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      qWarning() << "[NavigationTaskSolver] Ошибка создания CSV ПОЭХ:" << poehCsvPath;
      return;
   }

   QTextStream out(&file);
   out << "Epoch\tLat\tLon\tRMSE\tSigma3D\tSigmaH\tSigmaV\tPDOP\tHDOP\tVDOP\n";

   qint64 rows = 0;

   for (auto itEpoch = dualSolutions.constBegin(); itEpoch != dualSolutions.constEnd(); ++itEpoch) {
      for (auto itPoint = itEpoch.value().constBegin();
           itPoint != itEpoch.value().constEnd(); ++itPoint) {
         const NavSolution& sol = itPoint.value().sbasMode;

         if (!sol.converged) {
            continue;
         }

         const GRID_POINT& point = itPoint.key();
         const qint64 n          = nTerms.value(point, 0);
         const double rmse       = (n > 0) ? std::sqrt(sumSq.value(point, 0.0) / double(n))
                                           : qQNaN();

         out << itEpoch.key().toString(Qt::ISODate)                       << "\t"
             << QString::number(point.llh.latitude,  'f', 4)              << "\t"
             << QString::number(point.llh.longitude, 'f', 4)              << "\t"
             << QString::number(rmse,                'f', 3)              << "\t"
             << QString::number(sol.err3d,           'f', 3)              << "\t"
             << QString::number(sol.horiz_err,       'f', 3)              << "\t"
             << QString::number(sol.vert_err,        'f', 3)              << "\t"
             << QString::number(sol.dop.PDOP,        'f', 2)              << "\t"
             << QString::number(sol.dop.HDOP,        'f', 2)              << "\t"
             << QString::number(sol.dop.VDOP,        'f', 2)              << "\n";
         ++rows;
      }
   }

   qInfo().noquote()
      << QString("[NavigationTaskSolver] CSV ПОЭХ сохранён: %1, строк %2, точек %3")
      .arg(poehCsvPath).arg(rows).arg(nTerms.size());
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

bool NavigationTaskSolver::execute(pipeline::Context& ctx) {
   ctx.solutions.clear();

   if (!ctx.dm || ctx.residualErrors.isEmpty()) {
      qWarning() << "[NavigationTaskSolver] Нет данных для расчета.";
      return false;
   }

   WeightModel weightModel(ctx.dm->getSBASStore(), cfg_.weightCfg);

   // Локальное хранилище для двойного решения
   QMap<QDateTime, QMap<GRID_POINT, DualNavSolution> > dualSolutions;

   // ###
   struct WeightBudgetAcc {
      quint64 nSat     = 0; // НКА с построенным весом (по всем точкам/эпохам)
      quint64 nIonoPos = 0; // из них с sigma2_iono > 0 (GIVE реально применён)
      double  sumUdre = 0.0, sumIono = 0.0, sumTropo = 0.0, sumAir = 0.0;
      quint64 elTot[4]  = { 0, 0, 0, 0 };
      quint64 elIono[4] = { 0, 0, 0, 0 };
   } wba;
   // ###

   for (auto itEpoch = ctx.residualErrors.constBegin(); itEpoch != ctx.residualErrors.constEnd(); ++itEpoch) {
      const QDateTime& epoch = itEpoch.key();

      for (auto itPoint = itEpoch.value().constBegin(); itPoint != itEpoch.value().constEnd(); ++itPoint) {
         const GRID_POINT& point = itPoint.key();
         const auto& residuals   = itPoint.value();

         // 1. Сбор геометрии
         QVector<GeometryBuilder::SatInput> satInputs;
         satInputs.reserve(residuals.size());

         for (const auto& res : residuals) {
            if ((cfg_.constellation == Config::GLO_ONLY) &&
                (res.satellite.getSystem() != SatelliteSystem::TYPE::GLONASS)) {
               continue;
            }

            if ((cfg_.constellation == Config::GPS_ONLY) &&
                (res.satellite.getSystem() != SatelliteSystem::TYPE::GPS)) {
               continue;
            }
            // Координата SP3 phase center уже в метрах, проброшена из ErrorCalculator
            const COORD_XYZ& satEcef = res.satEcef;
            const double     norm    = std::sqrt(satEcef.x * satEcef.x +
                                                 satEcef.y * satEcef.y +
                                                 satEcef.z * satEcef.z);

            if (!std::isfinite(norm) || (norm < 1e6)) {
               continue;
            }

            satInputs.push_back({ res.satellite, satEcef });
         }

         // === DIAGNOSTIC: проверка качества координат спутников ===

         int zeroCount = 0;
         int totalSats = satInputs.size();

         for (const auto& si : satInputs) {
            double n = std::sqrt(si.satEcef.x * si.satEcef.x +
                                 si.satEcef.y * si.satEcef.y +
                                 si.satEcef.z * si.satEcef.z);

            if (!std::isfinite(n) || (n < 1e6)) { // в метрах, ожидаем ~26e6
               ++zeroCount;
            }
         }

         if ((zeroCount > 0) && (itPoint == itEpoch.value().constBegin())) {
            qWarning().noquote()
               << QString("[NTS][GEOM_DIAG] epoch=%1 totalSats=%2 badCoords=%3")
               .arg(epoch.toString(Qt::ISODate))
               .arg(totalSats)
               .arg(zeroCount);
         }


         GeometryBuilder geomBuilder(cfg_.geomCfg);
         const auto geoms        = geomBuilder.compute(point, satInputs);
         const auto weightResult = weightModel.buildRDiagonal(epoch, point, geoms);


         // ###
         for (const auto& w : weightResult.weights) {
            ++wba.nSat;

            if (qIsFinite(w.sigma2_iono_m2) && (w.sigma2_iono_m2 > 0.0)) {
               ++wba.nIonoPos;
            }
            wba.sumUdre  += w.sigma2_udre_m2;
            wba.sumIono  += w.sigma2_iono_m2;
            wba.sumTropo += w.sigma2_tropo_m2;
            wba.sumAir   += w.sigma2_air_m2;

            const int b = (w.elevation_deg < 10.0) ? 0
                          : (w.elevation_deg < 20.0) ? 1
                          : (w.elevation_deg < 30.0) ? 2 : 3;
            ++wba.elTot[b];

            if (qIsFinite(w.sigma2_iono_m2) && (w.sigma2_iono_m2 > 0.0)) {
               ++wba.elIono[b];
            }
         }
         // ###

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

            if (geom.elevation_deg < 5.0) {
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
            // Outlier rejection: |y| > K * sigma
            constexpr double OUTLIER_K = 6.0;

            if (std::abs(oSbas.y) > OUTLIER_K * std::sqrt(oSbas.sigma2)) {
               if (itPoint == itEpoch.value().constBegin()) {
                  qWarning().noquote()
                     << QString("[NTS][OUTLIER] epoch=%1 sat=%2 y=%3 sigma=%4 sig2_udre=%5 ratio=%6")
                     .arg(epoch.toString(Qt::ISODate))
                     .arg(geom.sat.toString())
                     .arg(oSbas.y,                                     0, 'f', 3)
                     .arg(std::sqrt(oSbas.sigma2),                     0, 'f', 3)
                     .arg(itWeight->sigma2_udre_m2,                    0, 'f', 4)
                     .arg(std::abs(oSbas.y) / std::sqrt(oSbas.sigma2), 0, 'f', 1);
               }
               continue;
            }
            sbasObs.push_back(oSbas);
         }

         // 3. Хелпер решения
         auto solveMode = [&](const QVector<LeastSquaresSolver::Observation>& obs,
                              const LeastSquaresSolver::Options& opt,
                              const QString& modeName) -> NavSolution {
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

         // ###
         if (itPoint == itEpoch.value().constBegin()) {
            qInfo().noquote()
               << QString("[NTS][SBAS_COUNTS] epoch=%1 geoms=%2 absObs=%3 sbasObs=%4")
               .arg(epoch.toString(Qt::ISODate))
               .arg(geoms.size())
               .arg(absObs.size())
               .arg(sbasObs.size());
         }
         // ###

         // 4. Решение двух штатных режимов
         DualNavSolution dSol;
         dSol.absMode  = solveMode(absObs,  cfg_.olsOpt, "ABS");
         dSol.sbasMode = solveMode(sbasObs, cfg_.wlsOpt, "SBAS");

         dualSolutions[epoch][point] = dSol;

         // === DIAGNOSTIC: сравнение abs vs sbas ===
         if (itPoint == itEpoch.value().constBegin()) {
            const bool aOk = dSol.absMode.converged;
            const bool sOk = dSol.sbasMode.converged;
            qDebug().noquote()
               << QString("[NTS][DUAL] epoch=%1 "
                          "abs_ok=%2 abs_err3d=%3 abs_sats=%4 abs_pdop=%5 "
                          "sbas_ok=%6 sbas_err3d=%7 sbas_sats=%8 sbas_pdop=%9")
               .arg(epoch.toString(Qt::ISODate))
               .arg(aOk)
               .arg(aOk ? QString::number(dSol.absMode.err3d, 'f', 3) : "NaN")
               .arg(dSol.absMode.num_sats)
               .arg(aOk ? QString::number(dSol.absMode.dop.PDOP, 'f', 2) : "NaN")
               .arg(sOk)
               .arg(sOk ? QString::number(dSol.sbasMode.err3d, 'f', 3) : "NaN")
               .arg(dSol.sbasMode.num_sats)
               .arg(sOk ? QString::number(dSol.sbasMode.dop.PDOP, 'f', 2) : "NaN");
         }

         // В контекст отдаём приоритетно решение СДКМ, иначе фолбэк на абсолютное
         if (dSol.sbasMode.converged) {
            ctx.solutions[epoch][point] = dSol.sbasMode;
         } else if (dSol.absMode.converged) {
            ctx.solutions[epoch][point] = dSol.absMode;
         }
      }
   }

   // === STATS: aggregate comparison abs vs sbas ===
   {
      struct ModeStats {
         int    count      = 0;
         int    sbasWins   = 0;
         int    absWins    = 0;
         double sumAbsErr  = 0.0;
         double sumSbasErr = 0.0;
      };

      ModeStats all;

      for (auto itE = dualSolutions.constBegin(); itE != dualSolutions.constEnd(); ++itE) {
         for (auto itP = itE.value().constBegin(); itP != itE.value().constEnd(); ++itP) {
            // Берём только первую точку для агрегации (как в DUAL-логе)
            if (itP != itE.value().constBegin()) {
               continue;
            }

            const auto& ds = itP.value();

            if (!ds.absMode.converged || !ds.sbasMode.converged) {
               continue;
            }

            ++all.count;
            all.sumAbsErr  += ds.absMode.err3d;
            all.sumSbasErr += ds.sbasMode.err3d;

            if (ds.sbasMode.err3d + 0.01 < ds.absMode.err3d) {
               ++all.sbasWins;
            } else if (ds.absMode.err3d + 0.01 < ds.sbasMode.err3d) {
               ++all.absWins;
            }
         }
      }

      if (all.count > 0) {
         qInfo().noquote()
            << QString("[NTS][RESULT] epochs=%1 "
                       "meanAbs3D=%2 meanSbas3D=%3 "
                       "sbasWins=%4 absWins=%5 ratio=%6")
            .arg(all.count)
            .arg(all.sumAbsErr  / all.count,                  0, 'f', 3)
            .arg(all.sumSbasErr / all.count,                  0, 'f', 3)
            .arg(all.sbasWins)
            .arg(all.absWins)
            .arg(all.sumSbasErr / qMax(all.sumAbsErr, 1e-12), 0, 'f', 3);
      }
   }

   // ###
   if (wba.nSat > 0) {
      qInfo().noquote()
         << QString("[NTS][WEIGHTBUDGET] satEpochs=%1 ionoApplied=%2 (%3%) "
                    "meanSig2_m2: udre=%4 iono=%5 tropo=%6 air=%7")
         .arg(wba.nSat)
         .arg(wba.nIonoPos)
         .arg(100.0 * double(wba.nIonoPos) / double(wba.nSat), 0, 'f', 1)
         .arg(wba.sumUdre  / double(wba.nSat),                 0, 'f', 4)
         .arg(wba.sumIono  / double(wba.nSat),                 0, 'f', 4)
         .arg(wba.sumTropo / double(wba.nSat),                 0, 'f', 4)
         .arg(wba.sumAir   / double(wba.nSat),                 0, 'f', 4);
   }

   if (wba.nSat > 0) {
      qInfo().noquote()
         << QString("[NTS][IONO_BY_ELEV] <10: %1/%2  10-20: %3/%4  20-30: %5/%6  30+: %7/%8  (iono/всего)")
         .arg(wba.elIono[0]).arg(wba.elTot[0])
         .arg(wba.elIono[1]).arg(wba.elTot[1])
         .arg(wba.elIono[2]).arg(wba.elTot[2])
         .arg(wba.elIono[3]).arg(wba.elTot[3]);
   }
   // ###

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

   // 6. Экспорт итогового CSV по ПОЭХ (7 характеристик услуги СДКМ)
   if (cfg_.writePoehCsv) {
      exportPoehCsv(cfg_.poehCsvPath, ctx, dualSolutions);
   }

   qInfo() << "[NavigationTaskSolver] Расчет завершен. Двойной CSV сохранен.";
   return true;
}
