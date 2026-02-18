#include <gtest/gtest.h>
#include <QtCore/QDir>
#include <QtCore/QDateTime>

#include "modules/IO/datamanager.h"
#include "modules/CorrectionApplier/correctionapplier.h"
#include "errorcalculator.h"

using io::DataManager;
using corrections::CorrectionApplier;
using navsolver::ErrorCalculator;

static QString p(const QString& rel) {
   return QStringLiteral(TEST_DATA_DIR) + "/" + rel;
}

TEST(ErrorCalculator_RealData, Smoke_LoadApplyCompute) {
   // 1) Пути к данным (заполните под вашу структуру tests/data)
   DataManager::Config cfg{};

   cfg.sp3Path        = p("sp3/Sta23634.sp3.glo");
   cfg.rinexNavPath   = p("rinex/GNSS00CMB_U_20251261311_15M_RN.rnx");
   cfg.sbasPath       = p("sbas_msg_csv/test_sbas_msg.csv");
   cfg.dcbPath        = p("bsx/IPG_20251140000_01D_01D_DCB.BSX");
   cfg.sbasSourceType = io::SourceType::FILE_CSV;

   pipeline::Context ctx{};

   // 2) Загрузка данных
   DataManager dm(cfg);
   ASSERT_TRUE(dm.execute(ctx));

   ASSERT_NE(ctx.dm,                 nullptr);
   ASSERT_NE(ctx.dm->getSP3File(),   nullptr);
   ASSERT_NE(ctx.dm->getRinexFile(), nullptr);

   // 3) Применение поправок (формирует navOrig + navCorrected)
   CorrectionApplier ap;
   ASSERT_TRUE(ap.execute(ctx));
   ASSERT_NE(ctx.navOrig, nullptr);
   ASSERT_TRUE(static_cast<bool> (ctx.navCorrected));

   // 4) Выбираем одну точку наблюдения (любая разумная ECEF)
   GRID_POINT gp{};
   gp.ecef        = COORD_XYZ{ constants::PZ_9011::a, 0.0, 0.0 }; // экватор, поверхность
   ctx.gridPoints = { gp };

   // 5) Выбираем эпоху: первая общая между SP3 и navCorrected
   const auto* sp3         = ctx.dm->getSP3File();
   const auto& sp3Recs     = sp3->records;
   const auto& navCorrRecs = ctx.navCorrected->navRecords;

   QDateTime chosenEpoch;

   for (auto it = navCorrRecs.cbegin(); it != navCorrRecs.cend(); ++it) {
      if (sp3Recs.contains(it.key())) {
         chosenEpoch = it.key();
         break;
      }
   }
   ASSERT_TRUE(chosenEpoch.isValid()) << "Нет общей эпохи между SP3 и navCorrected";

   // 6) Выбираем несколько спутников на этой эпохе (пересечение navCorrected ∩ SP3)
   QVector<Satellite> sats;
   const auto& navBySat = navCorrRecs[chosenEpoch];
   const auto& sp3BySat = sp3Recs[chosenEpoch];

   for (auto itSat = navBySat.cbegin(); itSat != navBySat.cend(); ++itSat) {
      const Satellite& sat = itSat.key();

      if (sp3BySat.contains(sat)) {
         sats.push_back(sat);
      }

      if (sats.size() >= 8) {
         break; // ограничим для скорости/детерминизма
      }
   }
   ASSERT_GE(sats.size(), 4) << "Слишком мало спутников в пересечении navCorrected ∩ SP3";

   ctx.visibleSats.clear();
   ctx.visibleSats[chosenEpoch][gp] = sats;

   // 7) Считаем residuals
   ErrorCalculator ec;
   ASSERT_TRUE(ec.execute(ctx));

   ASSERT_TRUE(ctx.residualErrors.contains(chosenEpoch));
   ASSERT_TRUE(ctx.residualErrors[chosenEpoch].contains(gp));
   const auto& resVec = ctx.residualErrors[chosenEpoch][gp];

   ASSERT_EQ(resVec.size(), sats.size()) << "Ожидали residual для каждого выбранного спутника";

   // 8) Инварианты
   for (const auto& e : resVec) {
      ASSERT_TRUE(std::isfinite(e.deltaSp3));
      ASSERT_TRUE(std::isfinite(e.deltaSdcm));
      ASSERT_TRUE(std::isfinite(e.residual));
      EXPECT_NEAR(e.residual, e.deltaSp3 - e.deltaSdcm, 1e-9);
   }
}
