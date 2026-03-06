#include <gtest/gtest.h>

// Даём тестовый доступ к приватным полям (только в этом TU)
#define private public
#include "modules/IO/sbascorrectionstore.h"
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

// ---------- PRN: правая граница исключается ----------
TEST(SBASCorrectionStore_Edge, PRNMaskBoundaryExclusiveRight) {
   SBASCorrectionStore st;

   auto m = makeMsg<MSG_PRN_MASK> ("2025-01-01 00:00:00");

   m->activePsps = { 10 };
   MSG_PRN_MASK::Satelite_PRN s; s.satId = 10; s.systemSat = PURPOSE_SYSTEM::GPS;
   m->satelites = { s };
   st.parsed_[MESSAGE_TYPE::PRN_MASK].push_back(m);

   st.buildCorrectionIndex();

   // Внутри окна [00:00:00, 00:10:00) — на 00:05:00 доступно
   auto ok_in = st.resolveByLocalIndex(10, T("2025-01-01 00:05:00"));
   ASSERT_TRUE(ok_in.has_value());

   // Ровно по правой границе — НЕ входит
   auto ok_out = st.resolveByLocalIndex(10, T("2025-01-01 00:10:00"));
   ASSERT_FALSE(ok_out.has_value());
}

// ---------- PRN: ремап локального индекса по времени ----------
TEST(SBASCorrectionStore_Edge, PRNMaskRemapLocalIndex) {
   SBASCorrectionStore st;

   // Маска #1: локальный 10 -> GPS PRN=10 в [00:00,00:10)
   auto m1 = makeMsg<MSG_PRN_MASK> ("2025-01-01 00:00:00");

   m1->activePsps = { 10 };
   { MSG_PRN_MASK::Satelite_PRN s; s.satId = 10; s.systemSat = PURPOSE_SYSTEM::GPS; m1->satelites = { s }; }
   st.parsed_[MESSAGE_TYPE::PRN_MASK].push_back(m1);

   // Маска #2: локальный 10 -> GPS PRN=11 в [00:10,00:20)
   auto m2 = makeMsg<MSG_PRN_MASK> ("2025-01-01 00:10:00");
   m2->activePsps = { 10 };
   { MSG_PRN_MASK::Satelite_PRN s; s.satId = 11; s.systemSat = PURPOSE_SYSTEM::GPS; m2->satelites = { s }; }
   st.parsed_[MESSAGE_TYPE::PRN_MASK].push_back(m2);

   st.buildCorrectionIndex();

   auto s1 = st.resolveByLocalIndex(10, T("2025-01-01 00:05:00"));
   ASSERT_TRUE(s1.has_value());
   EXPECT_EQ(s1->getNumber(), 10);

   auto s2 = st.resolveByLocalIndex(10, T("2025-01-01 00:15:00"));
   ASSERT_TRUE(s2.has_value());
   EXPECT_EQ(s2->getNumber(), 11);
}

// ---------- UDRE: doNotUse -> игнорируется ----------
TEST(SBASCorrectionStore_Edge, UDRE_DoNotUse_Ignored) {
   SBASCorrectionStore st;

   // PRN (локальный 10 -> GPS:10)
   auto m1 = makeMsg<MSG_PRN_MASK> ("2025-01-01 00:00:00");

   m1->activePsps = { 10 };
   { MSG_PRN_MASK::Satelite_PRN s; s.satId = 10; s.systemSat = PURPOSE_SYSTEM::GPS; m1->satelites = { s }; }
   st.parsed_[MESSAGE_TYPE::PRN_MASK].push_back(m1);

   // FAST Type2: единственная запись с doNotUse=true
   auto f2 = makeMsg<MSG_FAST_CORRECTIONS> ("2025-01-01 00:00:10");
   f2->iodf = 1; f2->iodp = 2;
   MSG_FAST_CORRECTIONS::Satelite_PRN_FAST_CORRECTION sat{};
   sat.satNum       = 10;
   sat.UDREI        = 4;
   sat.UDREI_meters = 4.0;
   sat.sigma_UDREI  = 1.0;
   sat.doNotUse     = true;
   f2->satelites    = { sat };
   st.parsed_[MESSAGE_TYPE::FAST_CORRECTIONS_2].push_back(f2);

   st.buildCorrectionIndex();

   Satellite s(10, SatelliteSystem::TYPE::GPS);
   auto base = st.udreSigma_m2(s, T("2025-01-01 00:02:00"));
   // Ожидаем, что doNotUse ведёт к отсутствию валидной оценки
   EXPECT_FALSE(base.has_value());
}

// ---------- INTEGRITY only: нет FAST -> var_eff == var_base ----------
TEST(SBASCorrectionStore_Edge, IntegrityOnly_BaseEqualsEff) {
   SBASCorrectionStore st;

   // PRN
   auto m1 = makeMsg<MSG_PRN_MASK> ("2025-01-01 00:00:00");

   m1->activePsps = { 10 };
   { MSG_PRN_MASK::Satelite_PRN s; s.satId = 10; s.systemSat = PURPOSE_SYSTEM::GPS; m1->satelites = { s }; }
   st.parsed_[MESSAGE_TYPE::PRN_MASK].push_back(m1);

   // INTEGRITY Type 6: UDRE без FAST
   auto t6 = makeMsg<MSG_INTEGRITY_INFORMATION> ("2025-01-01 00:00:05");
   t6->iodfi = { 0, 0, 0, 0 };
   MSG_INTEGRITY_INFORMATION::Satelite_PRN_UDREI u{};
   u.satNum      = 10; u.UDREI = 3; u.UDREI_meters = 3.0; u.sigma_UDREI = 0.75; u.doNotUse = false;
   t6->satelites = { u };
   st.parsed_[MESSAGE_TYPE::INTEGRITY_INFORMATION].push_back(t6);

   st.buildCorrectionIndex();

   Satellite s(10, SatelliteSystem::TYPE::GPS);
   auto base = st.udreSigma_m2(s, T("2025-01-01 00:01:00"));
   auto eff  = st.udreSigmaEff_m2(s, T("2025-01-01 00:01:00"));
   ASSERT_TRUE(base.has_value());
   ASSERT_TRUE(eff.has_value());
   EXPECT_NEAR(*base, *eff, 1e-12);
}

// ---------- DEG: a=0 -> var_eff == var_base (даже при наличии FAST) ----------
TEST(SBASCorrectionStore_Edge, DegradationZero_NoGrowth) {
   SBASCorrectionStore st;

   // PRN
   auto m1 = makeMsg<MSG_PRN_MASK> ("2025-01-01 00:00:00");

   m1->activePsps = { 10 };
   { MSG_PRN_MASK::Satelite_PRN s; s.satId = 10; s.systemSat = PURPOSE_SYSTEM::GPS; m1->satelites = { s }; }
   st.parsed_[MESSAGE_TYPE::PRN_MASK].push_back(m1);

   // FAST (обновление в 00:00:10)
   auto f2 = makeMsg<MSG_FAST_CORRECTIONS> ("2025-01-01 00:00:10");
   f2->iodf = 1; f2->iodp = 2;
   MSG_FAST_CORRECTIONS::Satelite_PRN_FAST_CORRECTION sat{};
   sat.satNum    = 10; sat.UDREI = 4; sat.UDREI_meters = 4.0; sat.sigma_UDREI = 1.0; sat.doNotUse = false;
   f2->satelites = { sat };
   st.parsed_[MESSAGE_TYPE::FAST_CORRECTIONS_2].push_back(f2);

   // DEG: a=0
   auto d7 = makeMsg<MSG_FAST_CORRECTION_DEGRADATION_FACTOR> ("2025-01-01 00:00:15");
   MSG_FAST_CORRECTION_DEGRADATION_FACTOR::Satelite_PRN_FCD d{};
   d.satNum      = 10; d.a = 0.0; d.updateInterval = 120; d.doNotUse = false;
   d7->satelites = { d };
   st.parsed_[MESSAGE_TYPE::FAST_CORRECTION_DEGRADATION_FACTOR].push_back(d7);

   st.buildCorrectionIndex();

   Satellite s(10, SatelliteSystem::TYPE::GPS);
   auto base = st.udreSigma_m2(s,  T("2025-01-01 00:02:00"));
   auto eff  = st.udreSigmaEff_m2(s, T("2025-01-01 00:02:00"));
   ASSERT_TRUE(base.has_value());
   ASSERT_TRUE(eff.has_value());
   EXPECT_NEAR(*base, *eff, 1e-12);
}

// ---------- IONO: IODI mismatch -> отказ ----------
TEST(SBASCorrectionStore_Edge, Iono_IODI_Mismatch_Fails) {
   SBASCorrectionStore st;

   // Type 18: band=0, IODI=1, один узел
   auto igp18 = makeMsg<MSG_IONOSPHERIC_GRID_POINT_MASK> ("2025-01-01 00:00:00");

   igp18->idBand = 0; igp18->iod = 1;
   { MSG_IONOSPHERIC_GRID_POINT_MASK::NODE_GRID n; n.idPoint = 1; n.lat = 10; n.lon = 20; igp18->coordinatesRange.push_back(n); }
   st.parsed_[MESSAGE_TYPE::IONOSPHERIC_GRID_POINT_MASK].push_back(igp18);

   // Type 26: тот же узел, но IODI=2
   auto igp26 = makeMsg<MSG_IONOSPHERIC_DELAY_CORRECTIONS> ("2025-01-01 00:00:30");
   igp26->numberBand = 0; igp26->iod = 2;
   { MSG_IONOSPHERIC_DELAY_CORRECTIONS::BlockPoint b; b.idPoint = 1; b.igp = 5.0; b.sigma_give = 0.25; b.doNotUse = false;
     igp26->points.push_back(b); }
   st.parsed_[MESSAGE_TYPE::IONOSPHERIC_DELAY_CORRECTIONS].push_back(igp26);

   st.buildCorrectionIndex();

   double vd = 0.0, var = 0.0;
   bool   ok = st.ionoVerticalAt(qDegreesToRadians(10.0), qDegreesToRadians(20.0),
                                 T("2025-01-01 00:01:00"), vd, var);
   EXPECT_FALSE(ok);
}

// ---------- IONO: IPP точно в узле -> вернуть значение узла ----------
TEST(SBASCorrectionStore_Edge, Iono_OnNode_ReturnsNodeValue) {
   SBASCorrectionStore st;

   // 18: квадрат из 4-х узлов (IODI=1)
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

   // 26: значения, узел (10,20) = 7.0 м
   auto igp26 = makeMsg<MSG_IONOSPHERIC_DELAY_CORRECTIONS> ("2025-01-01 00:00:30");
   igp26->numberBand = 0; igp26->iod = 1;
   {
      MSG_IONOSPHERIC_DELAY_CORRECTIONS::BlockPoint b;
      b.sigma_give = 0.25; b.doNotUse = false;
      b.idPoint    = 1; b.igp = 7.0; igp26->points.push_back(b);
      b.idPoint    = 2; b.igp = 4.0; igp26->points.push_back(b);
      b.idPoint    = 3; b.igp = 6.0; igp26->points.push_back(b);
      b.idPoint    = 4; b.igp = 8.0; igp26->points.push_back(b);
   }
   st.parsed_[MESSAGE_TYPE::IONOSPHERIC_DELAY_CORRECTIONS].push_back(igp26);

   st.buildCorrectionIndex();

   double vd = 0.0, var = 0.0;
   bool   ok = st.ionoVerticalAt(qDegreesToRadians(10.0), qDegreesToRadians(20.0),
                                 T("2025-01-01 00:01:00"), vd, var);
   ASSERT_TRUE(ok);
   EXPECT_NEAR(vd, 7.0, 1e-12);
}

// ---------- IONO: отсутствует один из узлов клетки -> отказ (нет интерполяции) ----------
TEST(SBASCorrectionStore_Edge, Iono_MissingNode_Fails) {
   SBASCorrectionStore st;

   // 18: три узла (нет 4-го)
   auto igp18 = makeMsg<MSG_IONOSPHERIC_GRID_POINT_MASK> ("2025-01-01 00:00:00");

   igp18->idBand = 0; igp18->iod = 1;
   {
      MSG_IONOSPHERIC_GRID_POINT_MASK::NODE_GRID n;
      n.idPoint = 1; n.lat = 10; n.lon = 20; igp18->coordinatesRange.push_back(n);
      n.idPoint = 2; n.lat = 10; n.lon = 21; igp18->coordinatesRange.push_back(n);
      n.idPoint = 3; n.lat = 11; n.lon = 20; igp18->coordinatesRange.push_back(n);
      // n.idPoint=4 отсутствует
   }
   st.parsed_[MESSAGE_TYPE::IONOSPHERIC_GRID_POINT_MASK].push_back(igp18);

   // 26: значения для имеющихся узлов
   auto igp26 = makeMsg<MSG_IONOSPHERIC_DELAY_CORRECTIONS> ("2025-01-01 00:00:30");
   igp26->numberBand = 0; igp26->iod = 1;
   {
      MSG_IONOSPHERIC_DELAY_CORRECTIONS::BlockPoint b;
      b.sigma_give = 0.25; b.doNotUse = false;
      b.idPoint    = 1; b.igp = 2.0; igp26->points.push_back(b);
      b.idPoint    = 2; b.igp = 4.0; igp26->points.push_back(b);
      b.idPoint    = 3; b.igp = 6.0; igp26->points.push_back(b);
   }
   st.parsed_[MESSAGE_TYPE::IONOSPHERIC_DELAY_CORRECTIONS].push_back(igp26);

   st.buildCorrectionIndex();

   double vd = 0.0, var = 0.0;
   bool   ok = st.ionoVerticalAt(qDegreesToRadians(10.5), qDegreesToRadians(20.5),
                                 T("2025-01-01 00:01:00"), vd, var);
   ASSERT_TRUE(ok);
   EXPECT_NEAR(vd,  5.0,   1e-12);
   EXPECT_NEAR(var, 0.125, 1e-12);
}

// ---------- LongTerm: tie → предпочесть hasVelocity=true ----------
TEST(SBASCorrectionStore_Edge, LongTerm_TiePreferVelocity) {
   SBASCorrectionStore st;

   // PRN
   auto m1 = makeMsg<MSG_PRN_MASK> ("2025-01-01 00:00:00");

   m1->activePsps = { 10 };
   { MSG_PRN_MASK::Satelite_PRN s; s.satId = 10; s.systemSat = PURPOSE_SYSTEM::GPS; m1->satelites = { s }; }
   st.parsed_[MESSAGE_TYPE::PRN_MASK].push_back(m1);

   // Нет FAST → score будет одинаковым (iodp=-1)
   // Два кандидата c одинаковым start (сделаем 00:01:00)
   auto l24 = makeMsg<MSG_MIXED_CORRECTIONS_SATELLITE_ERROR> ("2025-01-01 00:01:00");
   {
      l24->iodp = -1;
      auto c0 = std::make_shared<VELOCITY_CODE_0>();
      c0->prn = 10; c0->deltaEcef = { 1, 2, 3 }; c0->delta_a_f0 = 1e-6;
      l24->satellites.push_back(c0);
   }
   st.parsed_[MESSAGE_TYPE::MIXED_CORRECTIONS_SATELLITE_ERROR].push_back(l24);

   auto l25 = makeMsg<MSG_LONG_TERM_SATELLITE_ERROR_CORRECTIONS> ("2025-01-01 00:01:00");
   {
      VELOCITY_CODE_1 c1;
      c1.prn        = 10; c1.t0 = 3000; c1.deltaEcef = { 4, 5, 6 }; c1.deltaRoc = { 0.4, 0.5, 0.6 };
      c1.delta_a_f0 = 2e-6; c1.delta_a_f1 = 1e-12;
      l25->satellites_code_1.push_back(c1);
   }
   st.parsed_[MESSAGE_TYPE::LONG_TERM_SATELLITE_ERROR_CORRECTIONS].push_back(l25);

   st.buildCorrectionIndex();

   Satellite sat(10, SatelliteSystem::TYPE::GPS);
   auto e = st.getLongTermCorrection(sat, T("2025-01-01 00:02:00"));
   ASSERT_TRUE(e.has_value());
   // Предпочесть hasVelocity=true (здесь Type 25 c Code1)
   EXPECT_TRUE(e->hasVelocity);
   EXPECT_NEAR(e->deltaVel.x, 0.4, 1e-12);
}

// ---------- LongTerm: полное равенство → предпочесть Type 24 ----------
TEST(SBASCorrectionStore_Edge, LongTerm_TiePrefer24) {
   SBASCorrectionStore st;

   // PRN
   auto m1 = makeMsg<MSG_PRN_MASK> ("2025-01-01 00:00:00");

   m1->activePsps = { 10 };
   { MSG_PRN_MASK::Satelite_PRN s; s.satId = 10; s.systemSat = PURPOSE_SYSTEM::GPS; m1->satelites = { s }; }
   st.parsed_[MESSAGE_TYPE::PRN_MASK].push_back(m1);

   // FAST с IODP=5, но оба кандидата с iodp=-1 → score=0
   auto f2 = makeMsg<MSG_FAST_CORRECTIONS> ("2025-01-01 00:00:10");
   f2->iodf = 1; f2->iodp = 5;
   MSG_FAST_CORRECTIONS::Satelite_PRN_FAST_CORRECTION qq{};
   qq.satNum     = 10; qq.UDREI = 2; qq.UDREI_meters = 2.0; qq.sigma_UDREI = 0.5; qq.doNotUse = false;
   f2->satelites = { qq };
   st.parsed_[MESSAGE_TYPE::FAST_CORRECTIONS_2].push_back(f2);

   // Два кандидата в одну секунду, оба без скорости, одинаковые поля
   auto l24 = makeMsg<MSG_MIXED_CORRECTIONS_SATELLITE_ERROR> ("2025-01-01 00:01:00");
   {
      l24->iodp = -1;
      auto c0 = std::make_shared<VELOCITY_CODE_0>();
      c0->prn = 10; c0->deltaEcef = { 3, 3, 3 }; c0->delta_a_f0 = 1e-6;
      l24->satellites.push_back(c0);
   }
   st.parsed_[MESSAGE_TYPE::MIXED_CORRECTIONS_SATELLITE_ERROR].push_back(l24);

   auto l25 = makeMsg<MSG_LONG_TERM_SATELLITE_ERROR_CORRECTIONS> ("2025-01-01 00:01:00");
   {
      VELOCITY_CODE_0 c0;
      c0.prn = 10; c0.deltaEcef = { 3, 3, 3 }; c0.delta_a_f0 = 1e-6;
      l25->satellites_code_0.push_back(c0);
   }
   st.parsed_[MESSAGE_TYPE::LONG_TERM_SATELLITE_ERROR_CORRECTIONS].push_back(l25);

   st.buildCorrectionIndex();

   Satellite sat(10, SatelliteSystem::TYPE::GPS);
   auto e = st.getLongTermCorrection(sat, T("2025-01-01 00:02:00"));
   ASSERT_TRUE(e.has_value());
   // Полное равенство → tie-breaker должен отдать From24
   // (проверь, что у тебя включён приоритет From24 в компараторе)
   // Явный признак — deltaPos из l24 (3)
   EXPECT_NEAR(e->deltaPos.x, 3.0, 1e-12);
   EXPECT_FALSE(e->hasVelocity);
}

// ---------- LongTerm: выбор по смене FAST IODP ----------
TEST(SBASCorrectionStore_Edge, LongTerm_SelectByFastIodpChange) {
   SBASCorrectionStore st;

   // PRN
   auto m1 = makeMsg<MSG_PRN_MASK> ("2025-01-01 00:00:00");

   m1->activePsps = { 10 };
   { MSG_PRN_MASK::Satelite_PRN s; s.satId = 10; s.systemSat = PURPOSE_SYSTEM::GPS; m1->satelites = { s }; }
   st.parsed_[MESSAGE_TYPE::PRN_MASK].push_back(m1);

   // Два FAST с разными iodp
   auto fA = makeMsg<MSG_FAST_CORRECTIONS> ("2025-01-01 00:00:10");
   fA->iodf = 1; fA->iodp = 5;
   { MSG_FAST_CORRECTIONS::Satelite_PRN_FAST_CORRECTION a{}; a.satNum = 10; a.UDREI = 2; a.UDREI_meters = 2; a.sigma_UDREI = 0.5;
     a.doNotUse = false; fA->satelites = { a }; }
   st.parsed_[MESSAGE_TYPE::FAST_CORRECTIONS_2].push_back(fA);

   auto fB = makeMsg<MSG_FAST_CORRECTIONS> ("2025-01-01 00:05:00");
   fB->iodf = 1; fB->iodp = 7;
   { MSG_FAST_CORRECTIONS::Satelite_PRN_FAST_CORRECTION a{}; a.satNum = 10; a.UDREI = 2; a.UDREI_meters = 2; a.sigma_UDREI = 0.5;
     a.doNotUse = false; fB->satelites = { a }; }
   st.parsed_[MESSAGE_TYPE::FAST_CORRECTIONS_2].push_back(fB);

   // Два long-term 24 c разными iodp
   auto l24a = makeMsg<MSG_MIXED_CORRECTIONS_SATELLITE_ERROR> ("2025-01-01 00:01:00");
   l24a->iodp = 5;
   { auto c1 = std::make_shared<VELOCITY_CODE_1>(); c1->prn = 10; c1->t0 = 3600; c1->deltaEcef = { 1, 0, 0 }; c1->deltaRoc = { 0.1, 0, 0 };
     c1->delta_a_f0 = 1e-6; c1->delta_a_f1 = 1e-12; l24a->satellites.push_back(c1); }
   st.parsed_[MESSAGE_TYPE::MIXED_CORRECTIONS_SATELLITE_ERROR].push_back(l24a);

   auto l24b = makeMsg<MSG_MIXED_CORRECTIONS_SATELLITE_ERROR> ("2025-01-01 00:05:10");
   l24b->iodp = 7;
   { auto c1 = std::make_shared<VELOCITY_CODE_1>(); c1->prn = 10; c1->t0 = 3660; c1->deltaEcef = { 2, 0, 0 }; c1->deltaRoc = { 0.2, 0, 0 };
     c1->delta_a_f0 = 2e-6; c1->delta_a_f1 = 2e-12; l24b->satellites.push_back(c1); }
   st.parsed_[MESSAGE_TYPE::MIXED_CORRECTIONS_SATELLITE_ERROR].push_back(l24b);

   st.buildCorrectionIndex();

   Satellite sat(10, SatelliteSystem::TYPE::GPS);

   // До смены FAST IODP → выбирается iodp=5
   auto e1 = st.getLongTermCorrection(sat, T("2025-01-01 00:04:00"));
   ASSERT_TRUE(e1.has_value());
   EXPECT_EQ(e1->iodp, 5);
   EXPECT_NEAR(e1->deltaPos.x, 1.0, 1e-12);

   // После смены FAST IODP → выбирается iodp=7
   auto e2 = st.getLongTermCorrection(sat, T("2025-01-01 00:06:00"));
   ASSERT_TRUE(e2.has_value());
   EXPECT_EQ(e2->iodp, 7);
   EXPECT_NEAR(e2->deltaPos.x, 2.0, 1e-12);
}

// ---------- GPS–GLONASS offset: нет интервала ----------
TEST(SBASCorrectionStore_Edge, GpsGlonassOffset_NoMatch) {
   SBASCorrectionStore st;

   st.gpsGloOffsets_.push_back({ T("2025-01-01 00:00:00"), T("2025-01-01 01:00:00"), +3.0 });

   auto v = st.gpsGlonassOffset(T("2025-01-01 02:00:00"));
   EXPECT_FALSE(v.has_value());
}
