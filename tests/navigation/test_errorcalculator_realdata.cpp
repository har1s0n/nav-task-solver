#include <gtest/gtest.h>
#include <QtCore/QFileInfo>
#include <QtCore/QDateTime>

#include "modules/IO/datamanager.h"

// Открываем private как в ваших тестах
#define private public
#include "errorcalculator.h"
#undef private

using io::DataManager;
using navsolver::ErrorCalculator;

static QString td(const QString& rel) {
   return QString::fromUtf8(TEST_DATA_DIR) + "/" + rel;
}

static void assertFileExists(const QString& path) {
   QFileInfo fi(path);

   ASSERT_TRUE(fi.exists() && fi.isFile()) << path.toStdString();
}

static bool findEpochAndSats(const sp3::SP3_FILE*     sp3,
                             const rinex::RINEX_FILE* navCorr,
                             QDateTime&               outEpoch,
                             QVector<Satellite>&      outSats,
                             int                      minSats = 6) {
   if (!sp3 || !navCorr) {
      return false;
   }

   const auto& sp3Recs  = sp3->records;
   const auto& corrRecs = navCorr->navRecords;

   for (auto it = corrRecs.cbegin(); it != corrRecs.cend(); ++it) {
      const QDateTime& epoch = it.key();

      if (!sp3Recs.contains(epoch)) {
         continue;
      }

      const auto& corrBySat = it.value();
      const auto& sp3BySat  = sp3Recs[epoch];

      if (corrBySat.isEmpty() || sp3BySat.isEmpty()) {
         continue;
      }

      QVector<Satellite> sats;

      for (auto itSat = corrBySat.cbegin(); itSat != corrBySat.cend(); ++itSat) {
         const Satellite& sat = itSat.key();

         if (sp3BySat.contains(sat)) {
            sats.push_back(sat);
         }

         if (sats.size() >= minSats) {
            break;
         }
      }

      if (sats.size() >= minSats) {
         outEpoch = epoch;
         outSats  = std::move(sats);
         return true;
      }
   }
   return false;
}

TEST(ErrorCalculator_RealData, Pipeline_SMOKE_WithDcbCheck) {
   const QString sp3Path  = td("sp3/Sta23634.sp3.glo");
   const QString navPath  = td("rinex/GNSS00CMB_U_20251261311_15M_RN.rnx");
   const QString sbasPath = td("sbas_msg_csv/test_sbas_msg.csv");
   const QString dcbPath  = td("bsx/IPG_20251140000_01D_01D_DCB.BSX");

   assertFileExists(sp3Path);
   assertFileExists(navPath);
   assertFileExists(sbasPath);
   assertFileExists(dcbPath);

   // 1) DataManager
   DataManager::Config cfg{};
   cfg.sp3Path      = sp3Path;
   cfg.rinexNavPath = navPath;
   cfg.sbasPath     = sbasPath;
   cfg.dcbPath      = dcbPath;
   // cfg.sbasSourceType оставляем по умолчанию FILE_CSV

   pipeline::Context ctx{};
   DataManager dm(cfg);

   ASSERT_TRUE(dm.execute(ctx));
   ASSERT_NE(ctx.dm,                 nullptr);
   ASSERT_NE(ctx.dm->getSP3File(),   nullptr);
   ASSERT_NE(ctx.dm->getRinexFile(), nullptr);

   // 2) CorrectionApplier
   CorrectionApplier ap;
   ASSERT_TRUE(ap.execute(ctx));
   ASSERT_NE(ctx.navOrig, nullptr);
   ASSERT_TRUE(static_cast<bool> (ctx.navCorrected));

   // 3) Подготовим одну точку и найдём эпоху+спутники
   GRID_POINT gp{};
   gp.ecef        = COORD_XYZ{ 0.0, 0.0, 0.0 }; // допустимо для LOS-проекций
   ctx.gridPoints = { gp };

   QDateTime epoch;
   QVector<Satellite> sats;

   const auto* sp3 = ctx.dm->getSP3File();

   // Берём minSats=6, но если хотите мягче — поставьте 4
   ASSERT_TRUE(findEpochAndSats(sp3, ctx.navCorrected.get(), epoch, sats, 6))
      << "Не найдено эпохи с пересечением SP3 ∩ navCorrected по >=6 спутникам";

   // 4) visibleSats только для выбранной эпохи/точки
   ctx.visibleSats.clear();
   ctx.visibleSats[epoch][gp] = sats;

   // sanity: на этой эпохе corrected не больше orig
   const int nOrig = ctx.navOrig->navRecords.value(epoch).size();
   const int nCor  = ctx.navCorrected->navRecords.value(epoch).size();
   ASSERT_GT(nOrig, 0);
   ASSERT_GT(nCor,  0);
   ASSERT_LE(nCor, nOrig);

   // 5) ErrorCalculator
   ErrorCalculator ec;
   ASSERT_TRUE(ec.execute(ctx));

   ASSERT_TRUE(ctx.residualErrors.contains(epoch));
   ASSERT_TRUE(ctx.residualErrors[epoch].contains(gp));

   const auto& res = ctx.residualErrors[epoch][gp];
   ASSERT_EQ(res.size(), sats.size());

   for (const auto& e : res) {
      ASSERT_TRUE(std::isfinite(e.deltaSp3));
      ASSERT_TRUE(std::isfinite(e.deltaSdcm));
      ASSERT_TRUE(std::isfinite(e.residual));
      EXPECT_NEAR(e.residual, e.deltaSp3 - e.deltaSdcm, 1e-9);
   }

   // 6) Проверка: DCB реально подхватывается и добавляется к часам SP3 (для 1 спутника)
   const Satellite sat0 = sats.front();
   auto dcb_ns          = ctx.dm->getGlonassL3MinusL1Bias(sat0);
   ASSERT_TRUE(dcb_ns.has_value()) << "Нет DCB L3-L1 для выбранного спутника из пересечения";

   const auto&  sp3Rec  = sp3->records[epoch].value(sat0);
   const double clockL3 = sp3Rec.clock;

   const double base_sec = (std::fabs(clockL3) > 1e-3) ? (clockL3 * 1e-6) : clockL3;
   const double got_sec  = ec.correctSp3ClockL3toL1(ctx, sat0, epoch, clockL3);
   const double exp_sec  = base_sec + (*dcb_ns) * 1e-9;

   EXPECT_NEAR(got_sec, exp_sec, 1e-15);
}
