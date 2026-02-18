// gtest
#include <gtest/gtest.h>

#define private public
#include "sbascorrectionstore.h"
#undef private

#include <inav/SBAS>
#include <memory>

using namespace io;
using namespace sbas;

static QDateTime T(const char* s) {
   return QDateTime::fromString(QString::fromUtf8(s), "yyyy-MM-dd hh:mm:ss");
}

template<typename TMsg>
static std::shared_ptr<TMsg> makeMsg(const char* t) {
   auto m = std::make_shared<TMsg>();

   m->recvTime = T(t);
   return m;
}

// ---------- ЭТАП 1: PRN mask → resolveLocalIndex ----------
TEST(SBASCorrectionStore, PrnMaskResolve) {
   SBASCorrectionStore st;

   // Синтетическая PRN_MASK на 2 спутника: локальные индексы 10 (GPS, PRN=10) и 42 (GLO, PRN=5)
   auto m1 = makeMsg<MSG_PRN_MASK> ("2025-01-01 00:00:00");
   {
      m1->activePsps = { 10, 42 };
      MSG_PRN_MASK::Satelite_PRN sGPS;
      sGPS.satId = 10; sGPS.systemSat = PURPOSE_SYSTEM::GPS;
      MSG_PRN_MASK::Satelite_PRN sGLO;
      sGLO.satId    = 5;  sGLO.systemSat = PURPOSE_SYSTEM::GLONASS;
      m1->satelites = { sGPS, sGLO };
   }

   st.parsed_[MESSAGE_TYPE::PRN_MASK].push_back(m1);

   st.buildCorrectionIndex();

   auto satGPS = st.resolveByLocalIndex(10, T("2025-01-01 00:05:00"));
   ASSERT_TRUE(satGPS.has_value());
   EXPECT_EQ(satGPS->getSystem(), SatelliteSystem::TYPE::GPS);
   EXPECT_EQ(satGPS->m_number,    10);

   auto satGLO = st.resolveByLocalIndex(42, T("2025-01-01 00:05:00"));
   ASSERT_TRUE(satGLO.has_value());
   EXPECT_EQ(satGLO->getSystem(), SatelliteSystem::TYPE::GLONASS);
   EXPECT_EQ(satGLO->getNumber(), 5);

   auto none = st.resolveByLocalIndex(99, T("2025-01-01 00:10:00"));
   EXPECT_FALSE(none.has_value());
}

// ---------- ЭТАП 2: FAST/UDRE/DEG и udreSigmaEff ----------
TEST(SBASCorrectionStore, FastUdreDegradation) {
   SBASCorrectionStore st;

   // PRN mask (локальный 10 -> GPS PRN=10)
   auto m1 = makeMsg<MSG_PRN_MASK> ("2025-01-01 00:00:00");

   m1->activePsps = { 10 };
   { MSG_PRN_MASK::Satelite_PRN s; s.satId = 10; s.systemSat = PURPOSE_SYSTEM::GPS; m1->satelites = { s }; }
   st.parsed_[MESSAGE_TYPE::PRN_MASK].push_back(m1);

   // FAST type2 с UDREI и iodf/iodp
   auto f2 = makeMsg<MSG_FAST_CORRECTIONS> ("2025-01-01 00:00:10");
   f2->iodf = 1; f2->iodp = 2;
   { MSG_FAST_CORRECTIONS::Satelite_PRN_FAST_CORRECTION s;
     s.satNum      = 10; s.UDREI = 4; s.UDREI_meters = 4.0; s.sigma_UDREI = 1.0; s.doNotUse = false;
     f2->satelites = { s }; }
   st.parsed_[MESSAGE_TYPE::FAST_CORRECTIONS_2].push_back(f2);

   // DEG (Type 7): a=0.02 м/с, updateInterval не принципиален
   auto d7 = makeMsg<MSG_FAST_CORRECTION_DEGRADATION_FACTOR> ("2025-01-01 00:00:15");
   { MSG_FAST_CORRECTION_DEGRADATION_FACTOR::Satelite_PRN_FCD s;
     s.satNum      = 10; s.a = 0.02; s.updateInterval = 120; s.doNotUse = false;
     d7->satelites = { s }; }
   st.parsed_[MESSAGE_TYPE::FAST_CORRECTION_DEGRADATION_FACTOR].push_back(d7);

   st.buildCorrectionIndex();

   Satellite sat(10, SatelliteSystem::TYPE::GPS);

   // Базовая σ^2 из UDRE (1.0 м^2)
   auto s2 = st.udreSigma_m2(sat, T("2025-01-01 00:02:00"));
   ASSERT_TRUE(s2.has_value());
   EXPECT_NEAR(*s2, 1.0, 1e-9);

   // Эффективная с деградацией: dt ≈ 110 сек от fastLastUpdate(00:00:10) до 00:02:00
   // var_eff = 1.0 + (a*dt)^2 = 1 + (0.02*110)^2 = 1 + 4.84 ≈ 5.84
   auto s2eff = st.udreSigmaEff_m2(sat, T("2025-01-01 00:02:00"));
   ASSERT_TRUE(s2eff.has_value());
   EXPECT_NEAR(*s2eff, 5.84, 1e-2);

   // fastIodf/iodp
   auto iodf = st.fastIodf(sat, T("2025-01-01 00:02:00"));
   auto iodp = st.fastIodp(sat, T("2025-01-01 00:02:00"));
   ASSERT_TRUE(iodf.has_value()); ASSERT_TRUE(iodp.has_value());
   EXPECT_EQ(*iodf, 1); EXPECT_EQ(*iodp, 2);
}

// ---------- ЭТАП 3: LONG-TERM 24/25 и выбор по IODP и наличию скорости ----------
TEST(SBASCorrectionStore, LongTermSelectionIODPAndVelocity) {
   SBASCorrectionStore st;

   // PRN mask (локальный 10 -> GPS PRN=10)
   auto m1 = makeMsg<MSG_PRN_MASK> ("2025-01-01 00:00:00");

   m1->activePsps = { 10 };
   { MSG_PRN_MASK::Satelite_PRN s; s.satId = 10; s.systemSat = PURPOSE_SYSTEM::GPS; m1->satelites = { s }; }
   st.parsed_[MESSAGE_TYPE::PRN_MASK].push_back(m1);

   // FAST для актуального iodp=7 (чтобы приоритет сработал)
   auto f2 = makeMsg<MSG_FAST_CORRECTIONS> ("2025-01-01 00:00:20");
   f2->iodf = 3; f2->iodp = 7;
   { MSG_FAST_CORRECTIONS::Satelite_PRN_FAST_CORRECTION s;
     s.satNum      = 10; s.UDREI = 2; s.UDREI_meters = 2.0; s.sigma_UDREI = 0.5; s.doNotUse = false;
     f2->satelites = { s }; }
   st.parsed_[MESSAGE_TYPE::FAST_CORRECTIONS_2].push_back(f2);

   // LONG-TERM Type 25 (без IODP в структуре VELOCITY_CODE_* — используем s.iodp, если у вас есть; если нет — будет -1)
   auto l25 = makeMsg<MSG_LONG_TERM_SATELLITE_ERROR_CORRECTIONS> ("2025-01-01 00:01:00");
   {
      // Code 1 — имеет скорость и t0
      VELOCITY_CODE_1 c1;
      c1.prn        = 10; c1.t0 = 3600; c1.deltaEcef = { 1.0, 2.0, 3.0 }; c1.deltaRoc = { 0.1, 0.2, 0.3 };
      c1.delta_a_f0 = 2.5e-6; c1.delta_a_f1 = 1.0e-12; c1.iodp = 8; // не совпадает с fast
      l25->satellites_code_1.push_back(c1);
   }
   st.parsed_[MESSAGE_TYPE::LONG_TERM_SATELLITE_ERROR_CORRECTIONS].push_back(l25);

   // LONG-TERM Type 24 (имеет m->iodp)
   auto l24 = makeMsg<MSG_MIXED_CORRECTIONS_SATELLITE_ERROR> ("2025-01-01 00:01:30");
   {
      l24->iodp = 7; // совпадает с fast → приоритет
      // Вставим long-term Code 1 в поле satellites (как base VELOCITY_CODE)
      auto c1 = std::make_shared<VELOCITY_CODE_1>();
      c1->prn        = 10; c1->t0 = 3700; c1->deltaEcef = { 4.0, 5.0, 6.0 }; c1->deltaRoc = { 0.4, 0.5, 0.6 };
      c1->delta_a_f0 = 3.5e-6; c1->delta_a_f1 = 2.0e-12;
      l24->satellites.push_back(c1);
      // Добавим fast часть, чтобы store заполнил таймлайн fast (не критично для этого теста)
      l24->fast_correction.iodf = 3; l24->fast_correction.iodp = 7;
      MSG_FAST_CORRECTIONS::Satelite_PRN_FAST_CORRECTION fs; fs.satNum = 10; fs.UDREI = 2; fs.UDREI_meters = 2.0; fs.sigma_UDREI = 0.5;
      fs.doNotUse                    = false;
      l24->fast_correction.satelites = { fs };
   }
   st.parsed_[MESSAGE_TYPE::MIXED_CORRECTIONS_SATELLITE_ERROR].push_back(l24);

   st.buildCorrectionIndex();

   Satellite sat(10, SatelliteSystem::TYPE::GPS);
   auto e = st.getLongTermCorrection(sat, T("2025-01-01 00:02:00"));
   ASSERT_TRUE(e.has_value());
   // Должен выбрать Type 24 (iodp=7 совпадает с fastIodp)
   EXPECT_EQ(e->iodp, 7);
   EXPECT_TRUE(e->hasVelocity);
   EXPECT_NEAR(e->deltaPos.x, 4.0,     1e-12);
   EXPECT_NEAR(e->deltaVel.x, 0.4,     1e-12);
   EXPECT_NEAR(e->deltaAf0,   3.5e-6,  1e-15);
   EXPECT_NEAR(e->deltaAf1,   2.0e-12, 1e-18);
}

// ---------- ЭТАП 4: IONO 18/26 — билинейная интерполяция ----------
TEST(SBASCorrectionStore, IonoBilinearInterpolation) {
   SBASCorrectionStore st;

   // Type 18: снимок Band=0, IODI=1, 4 узла квадратом:
   // (lat,lon) в градусах: (10,20), (10,21), (11,20), (11,21)
   // idPoint условные: 1,2,3,4
   auto igp18 = makeMsg<MSG_IONOSPHERIC_GRID_POINT_MASK> ("2025-01-01 00:00:00");

   igp18->idBand = 0; igp18->iod = 1;
   {
      MSG_IONOSPHERIC_GRID_POINT_MASK::NODE_GRID n;
      n.idPoint = 1; n.lat = 10; n.lon = 20; igp18->coordinatesRange.push_back(n);
      n.idPoint = 2; n.lat = 10; n.lon = 21; igp18->coordinatesRange.push_back(n);
      n.idPoint = 3; n.lat = 11; n.lon = 20; igp18->coordinatesRange.push_back(n);
      n.idPoint = 4; n.lat = 11; n.lon = 21; igp18->coordinatesRange.push_back(n);
   }
   st.parsed_[MESSAGE_TYPE::IONOSPHERIC_GRID_POINT_MASK].push_back(igp18);

   // Type 26: те же узлы, IODI=1, значения задержки: z11=2, z21=4, z12=6, z22=8 м; сигма — 0.25 м^2 везде
   auto igp26 = makeMsg<MSG_IONOSPHERIC_DELAY_CORRECTIONS> ("2025-01-01 00:00:30");
   igp26->numberBand = 0; igp26->iod = 1;
   {
      MSG_IONOSPHERIC_DELAY_CORRECTIONS::BlockPoint b;
      b.idPoint = 1; b.igp = 2.0; b.sigma_give = 0.25; b.doNotUse = false; igp26->points.push_back(b);
      b.idPoint = 2; b.igp = 4.0; b.sigma_give = 0.25; b.doNotUse = false; igp26->points.push_back(b);
      b.idPoint = 3; b.igp = 6.0; b.sigma_give = 0.25; b.doNotUse = false; igp26->points.push_back(b);
      b.idPoint = 4; b.igp = 8.0; b.sigma_give = 0.25; b.doNotUse = false; igp26->points.push_back(b);
   }
   st.parsed_[MESSAGE_TYPE::IONOSPHERIC_DELAY_CORRECTIONS].push_back(igp26);

   st.buildCorrectionIndex();

   // IPP в центре клетки: (10.5°, 20.5°)
   double vdelay = 0.0, varv = 0.0;
   bool   ok = st.ionoVerticalAt(qDegreesToRadians(10.5), qDegreesToRadians(20.5),
                                 T("2025-01-01 00:01:00"), vdelay, varv);
   ASSERT_TRUE(ok);
   // Билинейно в центре — среднее: (2+4+6+8)/4 = 5
   EXPECT_NEAR(vdelay, 5.0,    1e-9);
   // Вар по весам (в центре все w=0.25) → 0.25^2 * 0.25 * 4 = 0.25 (та же дисперсия, т.к. одинаковая)
   // но по коду var складывает w^2*var_i, где var_i=0.25 → var=4*(0.25^2*0.25)=4*0.015625=0.0625
   EXPECT_NEAR(varv,   0.0625, 1e-12);
}

// ---------- ЭТАП 5: GPS–GLONASS offset ----------
TEST(SBASCorrectionStore, GpsGlonassOffset) {
   SBASCorrectionStore st;

   st.gpsGloOffsets_.push_back({ T("2025-01-01 00:00:00"), T("2025-01-01 12:00:00"), +3.0 });
   st.gpsGloOffsets_.push_back({ T("2025-01-01 12:00:00"), T("2025-01-02 00:00:00"), +3.1 });

   auto a = st.gpsGlonassOffset(T("2025-01-01 08:00:00"));
   auto b = st.gpsGlonassOffset(T("2025-01-01 16:00:00"));
   ASSERT_TRUE(a.has_value()); ASSERT_TRUE(b.has_value());
   EXPECT_DOUBLE_EQ(*a, 3.0);
   EXPECT_DOUBLE_EQ(*b, 3.1);
}
