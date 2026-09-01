#include <gtest/gtest.h>
#include <QtCore/QDateTime>

#define private public
#include "modules/IO/datamanager.h"
#include "modules/NavigationSolver/NavigationTaskSolver/navigationtasksolver.h"
#undef private

#include <inav/Constants>
#include <inav/Coordinates>

using io::DataManager;
using navsolver::NavigationTaskSolver;

static Satellite makeSat(SatelliteSystem::TYPE sys, int prn) {
   Satellite s;

   s.setSystem(sys);
   s.setNumber(prn);
   return s;
}

static GRID_POINT makePointECEF(double x, double y, double z) {
   GRID_POINT p{};

   p.llh.latitude  = 0.0;
   p.llh.longitude = 0.0;
   p.llh.height    = 0.0;
   p.ecef          = COORD_XYZ{ x, y, z };
   return p;
}

static sp3::SP3_RECORD makeSp3(const QDateTime& ep,
                               const Satellite& sat,
                               double x, double y, double z,
                               double clock = 0.0) {
   sp3::SP3_RECORD rec{};

   rec.dt        = ep;
   rec.satNumber = static_cast<unsigned short> (sat.getNumber());
   rec.satSys    = sat.getSystem();
   rec.coord     = COORD_XYZ{ x, y, z };
   rec.clock     = clock;
   return rec;
}

static std::array<double, 4> computeH(const COORD_XYZ& obs, const COORD_XYZ& sat) {
   COORD_XYZ d{ sat.x - obs.x, sat.y - obs.y, sat.z - obs.z };
   const double r = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);

   EXPECT_GT(r, 0.0);

   const double ux = d.x / r;
   const double uy = d.y / r;
   const double uz = d.z / r;

   // H = [-u_x,-u_y,-u_z,1]
   return { -ux, -uy, -uz, 1.0 };
}

static double dotHdx(const std::array<double, 4>& H, const std::array<double, 4>& dx) {
   return H[0] * dx[0] + H[1] * dx[1] + H[2] * dx[2] + H[3] * dx[3];
}

// -----------------------------
// execute() guards
// -----------------------------
TEST(NavigationTaskSolver_Basic, Execute_ReturnsFalse_WhenNoDataManager) {
   pipeline::Context ctx{};
   navsolver::NavigationTaskSolver::Config navSolverCfg{};
   NavigationTaskSolver solver{ navSolverCfg };

   // Сделаем "не пустые" residualErrors, чтобы проверить именно dm==nullptr
   const auto ep = QDateTime::fromString("2020-01-01T00:00:00", Qt::ISODate);
   const auto p  = makePointECEF(0, 0, 0);

   const auto sat = makeSat(SatelliteSystem::TYPE::GPS, 999);

   ctx.residualErrors[ep][p] = { ResidualError{ sat, 0.0, 0.0, 0.0 } };

   EXPECT_FALSE(solver.execute(ctx));
}

TEST(NavigationTaskSolver_Basic, Execute_ReturnsFalse_WhenNoSp3OrEmpty) {
   pipeline::Context ctx{};

   DataManager::Config cfg{};
   DataManager dm(cfg);

   ctx.dm = &dm;

   // residualErrors можно и пустыми: проверяем, что "нет SP3" отсекается раньше
   navsolver::NavigationTaskSolver::Config navSolverCfg{};
   NavigationTaskSolver solver{ navSolverCfg };
   EXPECT_FALSE(solver.execute(ctx));

   // Теперь создадим SP3_FILE, но оставим records пустым
   dm.sp3File_ = std::make_unique<sp3::SP3_FILE>();
   EXPECT_FALSE(solver.execute(ctx));
}

TEST(NavigationTaskSolver_Basic, Execute_ReturnsTrue_WhenResidualErrorsEmpty) {
   pipeline::Context ctx{};

   DataManager::Config cfg{};
   DataManager dm(cfg);

   dm.sp3File_ = std::make_unique<sp3::SP3_FILE>();
   ctx.dm      = &dm;

   // Сделаем SP3 непустым, иначе solver вернёт false раньше
   const auto ep  = QDateTime::fromString("2020-01-01T00:00:00", Qt::ISODate);
   const auto sat = makeSat(SatelliteSystem::TYPE::GPS, 999);
   dm.sp3File_->records[ep].insert(sat, makeSp3(ep, sat, 1000.0, 0.0, 0.0, 0.0));

   // residualErrors пустой
   navsolver::NavigationTaskSolver::Config navSolverCfg{};
   NavigationTaskSolver solver{ navSolverCfg };
   EXPECT_TRUE(solver.execute(ctx));
}

// -----------------------------
// E2E: OLS fallback (нет UDRE/GIVE) => должен восстановить δx
// -----------------------------
TEST(NavigationTaskSolver_E2E, OlsFallback_RecoversDxExactly_WhenResidualMatchesModel) {
   pipeline::Context ctx{};

   DataManager::Config cfg{};
   DataManager dm(cfg);

   dm.sp3File_ = std::make_unique<sp3::SP3_FILE>();
   ctx.dm      = &dm;

   // Конфиг: WLS пытаемся, но UDRE/GIVE нет => должна сработать OLS fallback
   NavigationTaskSolver::Config scfg;
   scfg.preferWls     = true;
   scfg.fallbackToOls = true;

   NavigationTaskSolver solver(scfg);

   const auto ep = QDateTime::fromString("2020-01-01T00:00:00", Qt::ISODate);
   const auto p  = makePointECEF(0, 0, 0);

   // 5 спутников, все с "up" (x) > 0 (при lat=0 lon=0 это гарантирует El>=0)
   const std::array<Satellite, 5> sats = {
      makeSat(SatelliteSystem::TYPE::GPS, 901),
      makeSat(SatelliteSystem::TYPE::GPS, 902),
      makeSat(SatelliteSystem::TYPE::GPS, 903),
      makeSat(SatelliteSystem::TYPE::GPS, 904),
      makeSat(SatelliteSystem::TYPE::GPS, 905)
   };

   const std::array<COORD_XYZ, 5> satPos = {
      COORD_XYZ{ 10.0, 0.0,   0.0           },
      COORD_XYZ{ 5.0,  5.0,   0.0           },
      COORD_XYZ{ 5.0,  -5.0,  0.0           },
      COORD_XYZ{ 5.0,  0.0,   5.0           },
      COORD_XYZ{ 5.0,  0.0,   -5.0          }
   };

   for (int i = 0; i < 5; ++i) {
      dm.sp3File_->records[ep].insert(sats[i], makeSp3(ep, sats[i], satPos[i].x, satPos[i].y, satPos[i].z, 0.0));
   }

   // Истинное δx (в метрах), включая cδt:
   const std::array<double, 4> dxTrue = { 1.0, 2.0, 3.0, 4.0 };

   QVector<ResidualError> residuals;
   residuals.reserve(5);

   for (int i = 0; i < 5; ++i) {
      const auto   H = computeH(p.ecef, satPos[i]);
      const double y = dotHdx(H, dxTrue); // y = H * δx

      residuals.push_back(ResidualError{
         .satellite = sats[i],
         .deltaSp3  = 0.0,
         .deltaSdcm = 0.0,
         .residual  = y
      });
   }

   ctx.residualErrors[ep][p] = residuals;

   ASSERT_TRUE(solver.execute(ctx));

   ASSERT_TRUE(ctx.solutions.contains(ep));
   ASSERT_TRUE(ctx.solutions[ep].contains(p));

   const auto& sol = ctx.solutions[ep][p];
   ASSERT_TRUE(sol.converged);
   EXPECT_EQ(sol.num_sats, 5);

   EXPECT_NEAR(sol.delta_pos_ecef.x, dxTrue[0],     1e-9);
   EXPECT_NEAR(sol.delta_pos_ecef.y, dxTrue[1],     1e-9);
   EXPECT_NEAR(sol.delta_pos_ecef.z, dxTrue[2],     1e-9);

   const double expectedClk_s = dxTrue[3] / constants::PZ_9011::c;
   EXPECT_NEAR(sol.delta_clk_s,      expectedClk_s, 1e-18);

   // Метрики по δpos в ENU при lat=0 lon=0:
   // ENU(e,n,u) = (Y,Z,X)
   const double expectedHoriz = std::sqrt(dxTrue[1] * dxTrue[1] + dxTrue[2] * dxTrue[2]);
   const double expectedVert  = std::abs(dxTrue[0]);
   const double expected3d    = std::sqrt(dxTrue[0] * dxTrue[0] + dxTrue[1] * dxTrue[1] + dxTrue[2] * dxTrue[2]);

   EXPECT_NEAR(sol.horiz_err,   expectedHoriz, 1e-9);
   EXPECT_NEAR(sol.vert_err,    expectedVert,  1e-9);
   EXPECT_NEAR(sol.err3d,       expected3d,    1e-9);

   // Постфит должен быть ~0
   EXPECT_NEAR(sol.postfit_rms, 0.0,           1e-9);

   // DOP: проверим конечность и конкретные значения для этой геометрии (через (H^T H)^-1)
   EXPECT_TRUE(std::isfinite(sol.dop.PDOP));
   EXPECT_TRUE(std::isfinite(sol.dop.HDOP));
   EXPECT_TRUE(std::isfinite(sol.dop.VDOP));
   EXPECT_TRUE(std::isfinite(sol.dop.GDOP));

   EXPECT_NEAR(sol.dop.PDOP, 4.0707576459, 1e-6);
   EXPECT_NEAR(sol.dop.HDOP, 1.4142135624, 1e-6);
   EXPECT_NEAR(sol.dop.VDOP, 3.8172068060, 1e-6);
   EXPECT_NEAR(sol.dop.GDOP, 5.0312730495, 1e-6);
}

// -----------------------------
// WLS-only: succeed with relaxed policy (no UDRE/GIVE required)
// -----------------------------
TEST(NavigationTaskSolver_E2E, WlsOnly_Succeeds_WhenUdreGiveNotRequired) {
   pipeline::Context ctx{};

   DataManager::Config cfg{};
   DataManager dm(cfg);

   dm.sp3File_ = std::make_unique<sp3::SP3_FILE>();
   ctx.dm      = &dm;

   NavigationTaskSolver::Config scfg;
   scfg.preferWls             = true;
   scfg.fallbackToOls         = false;
   scfg.weightCfg.requireUdre = false;
   scfg.weightCfg.requireGive = false;

   NavigationTaskSolver solver(scfg);

   const auto ep = QDateTime::fromString("2020-01-01T00:00:00", Qt::ISODate);
   const auto p  = makePointECEF(0, 0, 0);

   const std::array<Satellite, 4> sats = {
      makeSat(SatelliteSystem::TYPE::GPS, 901),
      makeSat(SatelliteSystem::TYPE::GPS, 902),
      makeSat(SatelliteSystem::TYPE::GPS, 903),
      makeSat(SatelliteSystem::TYPE::GPS, 904)
   };

   const std::array<COORD_XYZ, 4> satPos = {
      COORD_XYZ{ 10.0, 0.0,   0.0          },
      COORD_XYZ{ 5.0,  5.0,   0.0          },
      COORD_XYZ{ 5.0,  -5.0,  0.0          },
      COORD_XYZ{ 5.0,  0.0,   5.0          }
   };

   for (int i = 0; i < 4; ++i) {
      dm.sp3File_->records[ep].insert(sats[i], makeSp3(ep, sats[i], satPos[i].x, satPos[i].y, satPos[i].z, 0.0));
   }

   const std::array<double, 4> dxTrue = { 1.0, 2.0, 3.0, 4.0 };

   QVector<ResidualError> residuals;
   residuals.reserve(4);

   for (int i = 0; i < 4; ++i) {
      const auto H = computeH(p.ecef, satPos[i]);
      residuals.push_back(ResidualError{
         .satellite = sats[i],
         .deltaSp3  = 0.0,
         .deltaSdcm = 0.0,
         .residual  = dotHdx(H, dxTrue)
      });
   }

   ctx.residualErrors[ep][p] = residuals;

   ASSERT_TRUE(solver.execute(ctx));
   const auto& sol = ctx.solutions[ep][p];
   ASSERT_TRUE(sol.converged);

   EXPECT_NEAR(sol.delta_pos_ecef.x, dxTrue[0], 1e-9);
   EXPECT_NEAR(sol.delta_pos_ecef.y, dxTrue[1], 1e-9);
   EXPECT_NEAR(sol.delta_pos_ecef.z, dxTrue[2], 1e-9);
}

// -----------------------------
// WLS-only: fail when UDRE/GIVE required but store empty
// -----------------------------
TEST(NavigationTaskSolver_E2E, WlsOnly_Fails_WhenUdreGiveRequiredButNoData) {
   pipeline::Context ctx{};

   DataManager::Config cfg{};
   DataManager dm(cfg);

   dm.sp3File_ = std::make_unique<sp3::SP3_FILE>();
   ctx.dm      = &dm;

   NavigationTaskSolver::Config scfg;
   scfg.preferWls             = true;
   scfg.fallbackToOls         = false;
   scfg.weightCfg.requireUdre = true;
   scfg.weightCfg.requireGive = true;

   NavigationTaskSolver solver(scfg);

   const auto ep = QDateTime::fromString("2020-01-01T00:00:00", Qt::ISODate);
   const auto p  = makePointECEF(0, 0, 0);

   const auto sat = makeSat(SatelliteSystem::TYPE::GPS, 901);
   dm.sp3File_->records[ep].insert(sat, makeSp3(ep, sat, 10.0, 0.0, 0.0, 0.0));

   ctx.residualErrors[ep][p] = { ResidualError{ sat, 0.0, 0.0, 1.0 } };

   // Нужно >=4, иначе модуль отсекает раньше. Добавим 3 фейковых, но без UDRE/GIVE всё равно WLS невозможен.
   for (int prn = 902; prn <= 904; ++prn) {
      const auto s = makeSat(SatelliteSystem::TYPE::GPS, prn);
      dm.sp3File_->records[ep].insert(s, makeSp3(ep, s, 5.0, prn - 902, 0.0, 0.0));
      ctx.residualErrors[ep][p].push_back(ResidualError{ s, 0.0, 0.0, 1.0 });
   }

   ASSERT_TRUE(solver.execute(ctx));
   const auto& sol = ctx.solutions[ep][p];
   EXPECT_FALSE(sol.converged);
}
