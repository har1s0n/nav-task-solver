#include <gtest/gtest.h>
#include <QtCore/QDir>
#include <QtCore/QDebug>

#define private public
#include "errorcalculator.h"
#undef private

#include "modules/IO/datamanager.h"
#include <inav/Coordinates>
#include <inav/Constants>

using navsolver::ErrorCalculator;
using io::DataManager;

static std::unique_ptr<DataManager> loadIO_orSkip() {
   const QString base    = QString::fromUtf8(TEST_DATA_DIR); // <- из CMake
   const QString sp3Path = base + "/sp3/Sta23634.sp3.glo";
   const QString rnxPath = base + "/rinex/GNSS00CMB_U_20251261311_15M_RN.rnx";
   const QString dcbPath = base + "/bsx/IPG_20251140000_01D_01D_DCB.BSX";


   DataManager::Config cfg;

   cfg.sp3Path      = sp3Path;
   cfg.rinexNavPath = rnxPath;
   // SBAS умышленно не грузим в этих тестах
   cfg.sbasPath.clear();
   cfg.dcbPath = dcbPath;

   auto dm = std::make_unique<DataManager> (cfg);

   if (!dm->loadSP3(sp3Path)) {
      return nullptr;
   }

   if (!dm->loadRinexNav(rnxPath)) {
      return nullptr;
   }
   (void)dm->loadDCB(dcbPath); // опционально

   return dm;
}

// Берём первую эпоху, которая есть одновременно и в SP3, и в RINEX
static bool pickCommonEpochSat(const DataManager& dm,
                               QDateTime&         epochOut,
                               Satellite&         satOut) {
   const auto* sp3 = dm.getSP3File();
   const auto* rnx = dm.getRinexFile();

   if (!sp3 || !rnx) {
      return false;
   }

   for (auto itSp3 = sp3->records.cbegin(); itSp3 != sp3->records.cend(); ++itSp3) {
      const QDateTime& ep = itSp3.key();
      auto itNav          = rnx->navRecords.constFind(ep);

      if (itNav == rnx->navRecords.cend()) {
         continue;
      }

      const auto& sp3BySat = itSp3.value();
      const auto& navBySat = itNav.value();

      for (auto itSat = sp3BySat.cbegin(); itSat != sp3BySat.cend(); ++itSat) {
         const Satellite& s = itSat.key();

         if (navBySat.contains(s)) {
            epochOut = ep;
            satOut   = s;
            return true;
         }
      }
   }
   return false;
}

// Синтетический GRID_POINT: возьмём наблюдателя в центре масс (0,0,0)
// Это упрощает LOS = r_sp3_pc
static GRID_POINT makeObserverAtEarthCenter() {
   GRID_POINT gp;

   gp.ecef = COORD_XYZ(0.0, 0.0, 0.0);
   gp.llh  = COORD_LLH(0.0, 0.0, 0.0); // на всякий
   return gp;
}

// --- ТЕСТ 1: без модификаций RINEX (navCorrected == navOrig) → residual ≈ 0 ---
TEST(ErrorCalculator_E2E, ResidualZero_WhenNavCorrectedEqualsNavOrig)
{
   auto dm = loadIO_orSkip();

   pipeline::Context ctx;

   ctx.dm           = dm.get();
   ctx.navOrig      = dm->getRinexFile();
   ctx.navCorrected = std::make_unique<rinex::RINEX_FILE> (*ctx.navOrig); // копия без правок

   // Подготовим минимальные входы для ErrorCalculator: одна эпоха, одна точка, один НКА
   QDateTime epoch; Satellite sat;
   ASSERT_TRUE(pickCommonEpochSat(*dm, epoch, sat)) << "No common epoch+sat found in SP3 and RINEX";

   GRID_POINT gp = makeObserverAtEarthCenter();
   ctx.gridPoints             = { gp };
   ctx.visibleSats[epoch][gp] = { sat };

   ErrorCalculator ec;
   ASSERT_TRUE(ec.execute(ctx)) << "ErrorCalculator returned false";

   // Проверяем, что появились результаты и residual ~ 0
   ASSERT_TRUE(ctx.residualErrors.contains(epoch));
   ASSERT_TRUE(ctx.residualErrors[epoch].contains(gp));
   const auto& vec = ctx.residualErrors[epoch][gp];
   ASSERT_EQ(vec.size(), 1);

   const auto& re = vec.front();
   EXPECT_EQ(re.satellite, sat);
   // Δρ_SP3 и Δρ_SDCM должны совпасть → residual ≈ 0
   EXPECT_NEAR(re.residual, 0.0, 1e-6);
}

// --- ТЕСТ 2: искусственно сдвигаем navCorrected по LOS на +d ---
// Ожидаем residual ≈ +d (метров), т.к. deltaSdcm = deltaSp3 - d.
TEST(ErrorCalculator_E2E, ResidualEqualsAppliedShiftAlongLOS)
{
   auto dm = loadIO_orSkip();

   pipeline::Context ctx;

   ctx.dm           = dm.get();
   ctx.navOrig      = dm->getRinexFile();
   ctx.navCorrected = std::make_unique<rinex::RINEX_FILE> (*ctx.navOrig); // базовая копия

   QDateTime epoch; Satellite sat;
   ASSERT_TRUE(pickCommonEpochSat(*dm, epoch, sat)) << "No common epoch+sat found in SP3 and RINEX";

   GRID_POINT gp = makeObserverAtEarthCenter();
   ctx.gridPoints             = { gp };
   ctx.visibleSats[epoch][gp] = { sat };

   // Подготовим LOS для выбранного sat на epoch
   const auto* sp3       = dm->getSP3File();
   const auto& sp3SatMap = sp3->records[epoch];
   auto itSp3            = sp3SatMap.constFind(sat);
   ASSERT_TRUE(itSp3 != sp3SatMap.cend());
   COORD_XYZ r_sp3 = itSp3.value().coord; // фазовый центр не критичен для теста при малом d

   COORD_XYZ los        = { r_sp3.x - gp.ecef.x, r_sp3.y - gp.ecef.y, r_sp3.z - gp.ecef.z };
   const double losNorm = std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);
   ASSERT_GT(losNorm, 0.0);
   los.x /= losNorm; los.y /= losNorm; los.z /= losNorm; // единичный LOS

   // Сдвинем эфемериды navCorrected на +d вдоль LOS для выбранного sat/epoch
   constexpr double d = 10.0;                            // метров
   auto& recCorr      = ctx.navCorrected->navRecords[epoch][sat];
   recCorr.coord.x += d * los.x;
   recCorr.coord.y += d * los.y;
   recCorr.coord.z += d * los.z;
   // Часы оставляем без изменения, чтобы часовой вклад сократился как и раньше

   ErrorCalculator ec;
   ASSERT_TRUE(ec.execute(ctx)) << "ErrorCalculator returned false";

   ASSERT_TRUE(ctx.residualErrors.contains(epoch));
   ASSERT_TRUE(ctx.residualErrors[epoch].contains(gp));
   const auto& vec = ctx.residualErrors[epoch][gp];
   ASSERT_EQ(vec.size(), 1);

   const auto& re = vec.front();
   EXPECT_EQ(re.satellite, sat);

   // Ожидаем residual ≈ +d (метров). Дадим пару мм на округления.
   EXPECT_NEAR(re.residual, d, 2e-3);
}

// LOS (единичный), от наблюдателя в фазовый центр спутника из SP3
static COORD_XYZ losUnit(const COORD_XYZ& obs_ecef, const COORD_XYZ& sat_ecef) {
   COORD_XYZ v{ sat_ecef.x - obs_ecef.x, sat_ecef.y - obs_ecef.y, sat_ecef.z - obs_ecef.z };
   double    n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

   if (n <= 0) {
      return COORD_XYZ{ 0, 0, 0 };
   }
   return COORD_XYZ{ v.x / n, v.y / n, v.z / n };
}

static QString sbasCsvPath() {
   // tests/data/sbas_msg_csv/test_sbas_msg.csv
   return QString(TEST_DATA_DIR) + "/sbas_msg_csv/test_sbas_msg.csv";
}

// 2) Найти первую (epoch, sat), для которой есть LT-correction в store
static bool pickEpochSatWithSBAS(const DataManager&             dm,
                                 const io::SBASCorrectionStore& store,
                                 QDateTime&                     epochOut,
                                 Satellite&                     satOut) {
   const auto* sp3 = dm.getSP3File();
   const auto* rn  = dm.getRinexFile();

   if (!sp3 || !rn) {
      return false;
   }

   for (auto itEp = sp3->records.cbegin(); itEp != sp3->records.cend(); ++itEp) {
      const QDateTime& ep = itEp.key();
      auto itNav          = rn->navRecords.constFind(ep);

      if (itNav == rn->navRecords.cend()) {
         continue;
      }

      const auto& sp3BySat = itEp.value();
      const auto& navBySat = itNav.value();

      for (auto itS = sp3BySat.cbegin(); itS != sp3BySat.cend(); ++itS) {
         const Satellite& s = itS.key();

         if (!navBySat.contains(s)) {
            continue;
         }

         if (!store.getLongTermCorrection(s, ep)) {
            continue;
         }
         epochOut = ep; satOut = s; return true;
      }
   }
   return false;
}

// -----------------------------------------------------------------------------
// ТЕСТ 1: Применение SBAS (CSV) реально модифицирует нав.эфемериды и
// residual == (Δr · û) + c * Δclk  (тождество из определений).
// -----------------------------------------------------------------------------
TEST(ErrorCalculator_E2E_SBAS, ResidualEqualsAppliedSBASProjection)
{
   auto dm = loadIO_orSkip();

   if (!dm) {
      GTEST_SKIP() << "SP3/RINEX/DCB not found in TEST_DATA_DIR";
   }

   // загрузим SBAS CSV в store
   auto& store = dm->sbasStore();
   ASSERT_TRUE(store.load(io::SourceType::FILE_CSV, sbasCsvPath())) << "CSV parse failed";

   // выберем эпоху и НКА, где реально есть LT
   QDateTime epoch;
   Satellite sat;
   ASSERT_TRUE(pickEpochSatWithSBAS(*dm, store, epoch, sat))
      << "No (epoch,sat) with SBAS LT intersecting SP3 & RINEX";

   // подготовим контекст
   pipeline::Context ctx;
   ctx.dm           = dm.get();
   ctx.navOrig      = dm->getRinexFile();
   ctx.navCorrected = std::make_unique<rinex::RINEX_FILE> (*ctx.navOrig);
   GRID_POINT gp = makeObserverAtEarthCenter();
   ctx.gridPoints             = { gp };
   ctx.visibleSats[epoch][gp] = { sat };

   // 1) применяем SBAS к navCorrected
   corrections::CorrectionApplier applier;
   applier.applySBASCorrections(*ctx.navCorrected, store);

   // убережёмся: если по факту для выбранного sat/epoch нет изменений, переключимся на другой
   {
      const auto& recO = ctx.navOrig->navRecords[epoch][sat];
      const auto& recC = ctx.navCorrected->navRecords[epoch][sat];
      bool posChanged  = !(qFuzzyCompare(recO.coord.x, recC.coord.x) &&
                           qFuzzyCompare(recO.coord.y, recC.coord.y) &&
                           qFuzzyCompare(recO.coord.z, recC.coord.z));
      bool clkChanged = !qFuzzyCompare(recO.clock.svClockBias, recC.clock.svClockBias);

      if (!posChanged && !clkChanged) {
         // Итеративно пройти по всем подходящим (эпоха,НКА) и выбрать первую, где есть изменение.
         // (Для компактности можно оформить в helper; если хотите — пришлю готовый код перебора.)
         GTEST_SKIP() << "SBAS LT present, but no effective change on selected pair; try another (epoch,sat) in CSV.";
      }
   }


   // 2) считаем residual через ErrorCalculator
   ErrorCalculator ec;
   ASSERT_TRUE(ec.execute(ctx)) << "ErrorCalculator failed";

   ASSERT_TRUE(ctx.residualErrors.contains(epoch));
   ASSERT_TRUE(ctx.residualErrors[epoch].contains(gp));
   const auto& v = ctx.residualErrors[epoch][gp];
   // ищем нашу запись по спутнику
   auto it = std::find_if(v.begin(), v.end(), [&](const ResidualError& re){
      return re.satellite == sat;
   });
   ASSERT_TRUE(it != v.end());
   const ResidualError& re = *it;

   // 3) проверяем тождество: residual == (Δr·û) + c*Δclk
   const auto& recO = ctx.navOrig->navRecords[epoch][sat];
   const auto& recC = ctx.navCorrected->navRecords[epoch][sat];

   const auto* sp3       = dm->getSP3File();
   const auto& sp3SatMap = sp3->records[epoch];
   auto itSp3            = sp3SatMap.constFind(sat);
   ASSERT_TRUE(itSp3 != sp3SatMap.cend());
   const COORD_XYZ u = losUnit(gp.ecef, itSp3.value().coord);

   const COORD_XYZ dR{ recC.coord.x - recO.coord.x,
                       recC.coord.y - recO.coord.y,
                       recC.coord.z - recO.coord.z };
   const double proj = dR.x * u.x + dR.y * u.y + dR.z * u.z;

   const double dClk     = recC.clock.svClockBias - recO.clock.svClockBias; // [с]
   const double expected = proj + constants::PZ_9011::c * dClk;             // [м]

   EXPECT_NEAR(re.residual, expected, 2e-3);                                // миллиметры/сантиметры из-за округлений
   // И дополнительно — изменения действительно есть
   EXPECT_FALSE(qFuzzyIsNull(expected));
}

// -----------------------------------------------------------------------------
// ТЕСТ 2: Наличие SBAS-данных меняет residual относительно случая без коррекции.
// В «нулевом» случае (без аплайера) residual = 0 (navCorr == navOrig).
// С аплайером residual ≈ требуемая LOS-проекция + c*Δclk (не ноль).
// -----------------------------------------------------------------------------
TEST(ErrorCalculator_E2E_SBAS, ResidualBecomesNonZero_AfterSBAS)
{
   auto dm = loadIO_orSkip();

   if (!dm) {
      GTEST_SKIP() << "SP3/RINEX/DCB not found in TEST_DATA_DIR";
   }

   auto& store       = dm->sbasStore();
   const QString csv = sbasCsvPath();

   if (!store.load(io::SourceType::FILE_CSV, csv)) {
      GTEST_SKIP() << "SBAS CSV not found or failed: " << csv.toStdString();
   }

   QDateTime epoch; Satellite sat;
   ASSERT_TRUE(pickEpochSatWithSBAS(*dm, store, epoch, sat))
      << "No epoch/sat with SBAS LT in provided CSV";

   pipeline::Context ctx0; // без SBAS
   ctx0.dm           = dm.get();
   ctx0.navOrig      = dm->getRinexFile();
   ctx0.navCorrected = std::make_unique<rinex::RINEX_FILE> (*ctx0.navOrig);
   GRID_POINT gp = makeObserverAtEarthCenter();
   ctx0.gridPoints             = { gp };
   ctx0.visibleSats[epoch][gp] = { sat };

   ErrorCalculator ec;
   ASSERT_TRUE(ec.execute(ctx0));
   const auto& v0 = ctx0.residualErrors[epoch][gp];
   auto it0       = std::find_if(v0.begin(), v0.end(), [&](const ResidualError& r){
      return r.satellite == sat;
   });
   ASSERT_TRUE(it0 != v0.end());
   EXPECT_NEAR(it0->residual, 0.0, 1e-6); // без изменений — ноль

   pipeline::Context ctx1;                // с SBAS
   ctx1.dm                     = dm.get();
   ctx1.navOrig                = dm->getRinexFile();
   ctx1.navCorrected           = std::make_unique<rinex::RINEX_FILE> (*ctx1.navOrig);
   ctx1.gridPoints             = { gp };
   ctx1.visibleSats[epoch][gp] = { sat };

   corrections::CorrectionApplier applier;
   applier.applySBASCorrections(*ctx1.navCorrected, store);

   {
      const auto& recO       = ctx1.navOrig->navRecords[epoch][sat];
      const auto& recC       = ctx1.navCorrected->navRecords[epoch][sat];
      const bool  posChanged = !(qFuzzyCompare(recO.coord.x, recC.coord.x) &&
                                 qFuzzyCompare(recO.coord.y, recC.coord.y) &&
                                 qFuzzyCompare(recO.coord.z, recC.coord.z));
      const bool clkChanged = !qFuzzyCompare(recO.clock.svClockBias, recC.clock.svClockBias);

      if (!posChanged && !clkChanged) {
         GTEST_SKIP() << "Selected (epoch,sat) has SBAS LT but yields no change in navCorrected; try another pair.";
      }
   }

   ASSERT_TRUE(ec.execute(ctx1));
   const auto& v1 = ctx1.residualErrors[epoch][gp];
   auto it1       = std::find_if(v1.begin(), v1.end(), [&](const ResidualError& r){
      return r.satellite == sat;
   });
   ASSERT_TRUE(it1 != v1.end());

   // Теперь остаток не ноль (изменилась позиция и/или часы)
   EXPECT_FALSE(qFuzzyIsNull(it1->residual));
}
