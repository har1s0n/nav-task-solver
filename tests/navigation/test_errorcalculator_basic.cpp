#include <gtest/gtest.h>
#include <QtCore/QDateTime>

#define private public
#include "errorcalculator.h"
#include "modules/IO/datamanager.h"
#undef private

using navsolver::ErrorCalculator;
using io::DataManager;

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

static rinex::NAV_RECORD makeNav(double x, double y, double z,
                                 double clkBias      = 0.0,
                                 double clkDrift     = 0.0,
                                 double clkDriftRate = 0.0,
                                 double tk           = 0.0,
                                 double week_time    = -1.0) {
   rinex::NAV_RECORD r{};

   r.coord = COORD_XYZ{ x, y, z };

   // Для GPS в ErrorCalculator используется только svClockBias,
   // для GLO используется полином и tk/week_time.
   r.clock.svClockBias      = clkBias;
   r.clock.svClockDrift     = clkDrift;
   r.clock.svClockDriftRate = clkDriftRate;
   r.clock.tk               = tk;
   r.clock.week_time        = week_time;
   return r;
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

// -----------------------------
// execute() guards
// -----------------------------
TEST(ErrorCalculator_Basic, Execute_ReturnsFalse_WhenNoDataManager) {
   pipeline::Context ctx{};

   ErrorCalculator ec;

   // делаем "не пустые" gridPoints/visibleSats, чтобы проверить именно dm==nullptr
   const auto ep = QDateTime::fromString("2020-01-01T00:00:00", Qt::ISODate);
   const auto p  = makePointECEF(0, 0, 0);
   const auto s  = makeSat(SatelliteSystem::TYPE::GPS, 5);

   ctx.gridPoints         = { p };
   ctx.visibleSats[ep][p] = { s };

   EXPECT_FALSE(ec.execute(ctx));
}

TEST(ErrorCalculator_Basic, Execute_ReturnsFalse_WhenGridPointsEmpty) {
   pipeline::Context ctx{};

   DataManager::Config cfg{};
   DataManager dm(cfg);

   ctx.dm = &dm;

   const auto ep = QDateTime::fromString("2020-01-01T00:00:00", Qt::ISODate);
   const auto p  = makePointECEF(0, 0, 0);
   const auto s  = makeSat(SatelliteSystem::TYPE::GPS, 5);

   // gridPoints пустой
   ctx.visibleSats[ep][p] = { s };

   ErrorCalculator ec;
   EXPECT_FALSE(ec.execute(ctx));
}

TEST(ErrorCalculator_Basic, Execute_ReturnsFalse_WhenVisibleSatsEmpty) {
   pipeline::Context ctx{};

   DataManager::Config cfg{};
   DataManager dm(cfg);

   ctx.dm = &dm;

   const auto p = makePointECEF(0, 0, 0);
   ctx.gridPoints = { p };
   // visibleSats пустой

   ErrorCalculator ec;
   EXPECT_FALSE(ec.execute(ctx));
}

// -----------------------------
// correctSp3ClockL3toL1()
// -----------------------------
TEST(ErrorCalculator_Basic, CorrectSp3Clock_Glonass_UnitsAndDcb) {
   pipeline::Context ctx{};

   DataManager::Config cfg{};
   DataManager dm(cfg);

   ctx.dm = &dm;

   ErrorCalculator ec;

   const auto ep  = QDateTime::fromString("2020-01-01T00:00:00", Qt::ISODate);
   const auto sat = makeSat(SatelliteSystem::TYPE::GLONASS, 3);

   // Подкладываем DCB L3-L1 = +10 нс
   dm.glonassDcbL3L1_[sat] = 10.0;

   // 1) |clockL3| > 1e-3 -> трактуется как микросекунды
   {
      const double clockL3  = 2.0; // "мкс" по условию кода -> 2e-6 сек
      const double got      = ec.correctSp3ClockL3toL1(ctx, sat, ep, clockL3);
      const double expected = 2.0e-6 + 10.0e-9;
      EXPECT_NEAR(got, expected, 1e-18);
   }

   // 2) |clockL3| <= 1e-3 -> трактуется как секунды
   {
      const double clockL3  = 1.0e-4; // сек
      const double got      = ec.correctSp3ClockL3toL1(ctx, sat, ep, clockL3);
      const double expected = 1.0e-4 + 10.0e-9;
      EXPECT_NEAR(got, expected, 1e-18);
   }
}

// -----------------------------
// computeBroadcastClockL1_GLO()
// -----------------------------
TEST(ErrorCalculator_Basic, BroadcastClock_GLO_UsesWeekTimeOrTk_AndWrapWeek) {
   ErrorCalculator ec;

   // epoch = GPS epoch + 10 секунд => secondsOfWeek(epoch) = 10
   const QDateTime epoch(QDate(1980, 1, 6), QTime(0, 0, 10), Qt::UTC);

   rinex::R_TIME tr{};

   tr.svClockBias      = 1.0;
   tr.svClockDrift     = 2.0;
   tr.svClockDriftRate = 0.0;

   // A) week_time >= 0 -> берём week_time
   {
      tr.week_time = 0.0;
      tr.tk        = 123.0; // не должен использоваться
      const double got = ec.computeBroadcastClockL1_GLO(tr, epoch);

      // dt = 10 - 0 = 10
      const double expected = 1.0 + 2.0 * 10.0;
      EXPECT_DOUBLE_EQ(got, expected);
   }

   // B) week_time < 0 -> берём tk и проверяем wrapWeek
   {
      tr.week_time = -1.0;
      tr.tk        = 604790.0; // tow - tk = 10 - 604790 = -604780 -> wrapWeek -> +604800 => 20
      const double got = ec.computeBroadcastClockL1_GLO(tr, epoch);

      const double expected = 1.0 + 2.0 * 20.0;
      EXPECT_DOUBLE_EQ(got, expected);
   }
}

// -----------------------------
// E2E: computeResidualErrors() via execute()
// -----------------------------
TEST(ErrorCalculator_E2E, ResidualZero_WhenNavCorrectedEqualsNavOrig) {
   pipeline::Context ctx{};

   // DataManager + SP3
   DataManager::Config cfg{};
   DataManager dm(cfg);

   dm.sp3File_ = std::make_unique<sp3::SP3_FILE>();
   ctx.dm      = &dm;

   // Эпоха/точка/спутник
   const auto ep = QDateTime::fromString("2020-01-01T00:00:00", Qt::ISODate);
   const auto p  = makePointECEF(0, 0, 0);

   // Берём GPS и большой номер PRN, чтобы в Coordinates::m_ephPC2MCOffset почти наверняка не было смещения
   const auto sat = makeSat(SatelliteSystem::TYPE::GPS, 999);

   ctx.gridPoints         = { p };
   ctx.visibleSats[ep][p] = { sat };

   // SP3: спутник на оси X, часы = 0
   dm.sp3File_->records[ep].insert(sat, makeSp3(ep, sat, 10000.0, 0.0, 0.0, 0.0));

   // NAV raw/corr одинаковые
   static rinex::RINEX_FILE navOrig{};
   navOrig.navRecords.clear();
   navOrig.navRecords[ep].insert(sat, makeNav(9990.0, 0.0, 0.0, 0.0));

   auto navCorr = std::make_unique<rinex::RINEX_FILE>();
   navCorr->navRecords[ep].insert(sat, makeNav(9990.0, 0.0, 0.0, 0.0));

   ctx.navOrig      = &navOrig;
   ctx.navCorrected = std::move(navCorr);

   ErrorCalculator ec;
   ASSERT_TRUE(ec.execute(ctx));

   ASSERT_TRUE(ctx.residualErrors.contains(ep));
   ASSERT_TRUE(ctx.residualErrors[ep].contains(p));
   ASSERT_EQ(ctx.residualErrors[ep][p].size(), 1);

   const auto& e = ctx.residualErrors[ep][p].front();
   EXPECT_NEAR(e.deltaSp3,  10.0, 1e-12); // 10000 - 9990 по LOS
   EXPECT_NEAR(e.deltaSdcm, 10.0, 1e-12);
   EXPECT_NEAR(e.residual,  0.0,  1e-12);
}

TEST(ErrorCalculator_E2E, ResidualEqualsAppliedShiftAlongLOS) {
   pipeline::Context ctx{};

   DataManager::Config cfg{};
   DataManager dm(cfg);

   dm.sp3File_ = std::make_unique<sp3::SP3_FILE>();
   ctx.dm      = &dm;

   const auto ep  = QDateTime::fromString("2020-01-01T00:00:00", Qt::ISODate);
   const auto p   = makePointECEF(0, 0, 0);
   const auto sat = makeSat(SatelliteSystem::TYPE::GPS, 999);

   ctx.gridPoints         = { p };
   ctx.visibleSats[ep][p] = { sat };

   dm.sp3File_->records[ep].insert(sat, makeSp3(ep, sat, 10000.0, 0.0, 0.0, 0.0));

   static rinex::RINEX_FILE navOrig{};
   navOrig.navRecords.clear();
   navOrig.navRecords[ep].insert(sat, makeNav(9990.0, 0.0, 0.0, 0.0));  // deltaSp3 = 10

   auto navCorr = std::make_unique<rinex::RINEX_FILE>();
   navCorr->navRecords[ep].insert(sat, makeNav(9997.0, 0.0, 0.0, 0.0)); // deltaSdcm = 3

   ctx.navOrig      = &navOrig;
   ctx.navCorrected = std::move(navCorr);

   ErrorCalculator ec;
   ASSERT_TRUE(ec.execute(ctx));

   const auto& e = ctx.residualErrors[ep][p].front();
   EXPECT_NEAR(e.deltaSp3,  10.0, 1e-12);
   EXPECT_NEAR(e.deltaSdcm, 3.0,  1e-12);
   EXPECT_NEAR(e.residual,  7.0,  1e-12);
}

TEST(ErrorCalculator_E2E, ResidualIncludesClockDifference_GPS) {
   pipeline::Context ctx{};

   DataManager::Config cfg{};
   DataManager dm(cfg);

   dm.sp3File_ = std::make_unique<sp3::SP3_FILE>();
   ctx.dm      = &dm;

   const auto ep  = QDateTime::fromString("2020-01-01T00:00:00", Qt::ISODate);
   const auto p   = makePointECEF(0, 0, 0);
   const auto sat = makeSat(SatelliteSystem::TYPE::GPS, 999);

   ctx.gridPoints         = { p };
   ctx.visibleSats[ep][p] = { sat };

   // Координаты одинаковые => геометрия даёт 0, остаются только часы
   dm.sp3File_->records[ep].insert(sat, makeSp3(ep, sat, 10000.0, 0.0, 0.0, 0.0));

   static rinex::RINEX_FILE navOrig{};
   navOrig.navRecords.clear();
   navOrig.navRecords[ep].insert(sat, makeNav(10000.0, 0.0, 0.0, 1.0e-6));  // clkNavRaw

   auto navCorr = std::make_unique<rinex::RINEX_FILE>();
   navCorr->navRecords[ep].insert(sat, makeNav(10000.0, 0.0, 0.0, 3.0e-6)); // clkNavCor

   ctx.navOrig      = &navOrig;
   ctx.navCorrected = std::move(navCorr);

   ErrorCalculator ec;
   ASSERT_TRUE(ec.execute(ctx));

   const auto& e = ctx.residualErrors[ep][p].front();

   const double expectedResidual = constants::PZ_9011::c * (3.0e-6 - 1.0e-6); // c*(clkNavCor - clkNavRaw)
   EXPECT_NEAR(e.deltaSp3,  constants::PZ_9011::c * (0.0 - 1.0e-6), 1e-3);
   EXPECT_NEAR(e.deltaSdcm, constants::PZ_9011::c * (0.0 - 3.0e-6), 1e-3);
   EXPECT_NEAR(e.residual,  expectedResidual,                       1e-3);
}
