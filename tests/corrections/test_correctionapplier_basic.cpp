#include <gtest/gtest.h>
#include <QtCore/QDateTime>

#define private public
#include "correctionapplier.h"
#include "sbascorrectionstore.h"
#undef private

using corrections::CorrectionApplier;
using io::SBASCorrectionStore;
using io::LongTermCorrectionEntry;

static Satellite makeSat(SatelliteSystem::TYPE sys, int prn) {
   Satellite s;

   s.setSystem(sys);
   s.setNumber(prn);

   return s;
}

static rinex::NAV_RECORD makeNav(double x, double y, double z, double clkBias = 0.0) {
   rinex::NAV_RECORD r{};

   r.coord.x           = x; r.coord.y = y; r.coord.z = z;
   r.clock.svClockBias = clkBias;
   return r;
}

TEST(CorrectionApplier_Basic, PositionOnly_LongTerm25_NoVelocity) {
   SBASCorrectionStore store;

   // прямой доступ для теста
    #define private public
    #include "modules/IO/sbascorrectionstore.h"
    #undef private

   const QDateTime ep = QDateTime::fromString("2020-01-01T00:00:10", Qt::ISODate);
   const Satellite sat = makeSat(SatelliteSystem::TYPE::GPS, 5);

   LongTermCorrectionEntry e{};
   e.start       = ep.addSecs(-60); e.end = ep.addSecs(3600);
   e.iodp        = 7; e.t0 = 0;
   e.deltaPos    = { 1.0, 2.0, 3.0 }; // м
   e.hasVelocity = false; e.deltaAf0 = 0.0; e.deltaAf1 = 0.0;
   store.longTermBySat_[sat].push_back(e);

   rinex::RINEX_FILE nav{};
   nav.navRecords[ep].insert(sat, makeNav(1000, 2000, 3000, 0.0));

   CorrectionApplier ap;
   // применяем через приватный метод
   ASSERT_TRUE(ap.applyCorrectionToNavRecord(nav.navRecords[ep][sat], sat, ep, store));

   EXPECT_DOUBLE_EQ(nav.navRecords[ep][sat].coord.x,           1001.0);
   EXPECT_DOUBLE_EQ(nav.navRecords[ep][sat].coord.y,           2002.0);
   EXPECT_DOUBLE_EQ(nav.navRecords[ep][sat].coord.z,           3003.0);
   EXPECT_DOUBLE_EQ(nav.navRecords[ep][sat].clock.svClockBias, 0.0);
}

TEST(CorrectionApplier_Basic, PositionPlusVelocity_LongTerm24_WithClock) {
   SBASCorrectionStore store;

    #define private public
    #include "modules/IO/sbascorrectionstore.h"
    #undef private

   const QDateTime ep = QDateTime::fromString("2020-01-01T00:02:10", Qt::ISODate);
   const int t_epoch   = QTime(0, 0).secsTo(ep.time()); // 130 c
   const Satellite sat = makeSat(SatelliteSystem::TYPE::GPS, 7);

   LongTermCorrectionEntry e{};
   e.start       = ep.addSecs(-60); e.end = ep.addSecs(3600);
   e.iodp        = 5; e.t0 = 100;     // опорное время
   e.deltaPos    = { 1.0, 0.0, 0.0 }; // м
   e.deltaVel    = { 0.1, 0.0, 0.0 }; // м/с
   e.hasVelocity = true;
   e.deltaAf0    = 2.0e-6;            // сек
   e.deltaAf1    = 1.0e-12;           // сек/сек
   store.longTermBySat_[sat].push_back(e);

   rinex::RINEX_FILE nav{};
   nav.navRecords[ep].insert(sat, makeNav(0, 0, 0, 0.0));

   CorrectionApplier ap;
   ASSERT_TRUE(ap.applyCorrectionToNavRecord(nav.navRecords[ep][sat], sat, ep, store));

   // Δr = 1 + 0.1*(130-100) = 1 + 3 = 4 м по X
   EXPECT_NEAR(nav.navRecords[ep][sat].coord.x, 4.0, 1e-12);
   EXPECT_NEAR(nav.navRecords[ep][sat].coord.y, 0.0, 1e-12);
   EXPECT_NEAR(nav.navRecords[ep][sat].coord.z, 0.0, 1e-12);

   // часы: δa_f0 + δa_f1*(t - t0) = 2e-6 + 1e-12*30 = 2.00003e-6
   const double expected_clk = 2.0e-6 + 1.0e-12 * (t_epoch - e.t0); // секунды!
   EXPECT_NEAR(nav.navRecords[ep][sat].clock.svClockBias, expected_clk, 1e-15);
}

TEST(CorrectionApplier_Basic, GlonassClock_WithGpsMinusGloOffset) {
   SBASCorrectionStore store;

    #define private public
    #include "modules/IO/sbascorrectionstore.h"
    #undef private

   const QDateTime ep = QDateTime::fromString("2020-01-01T00:00:00", Qt::ISODate);
   const Satellite sat = makeSat(SatelliteSystem::TYPE::GLONASS, 3);

   LongTermCorrectionEntry e{};
   e.start       = ep.addSecs(-10); e.end = ep.addSecs(3600);
   e.iodp        = 2; e.t0 = 0;
   e.deltaPos    = { 0, 0, 0 };
   e.hasVelocity = false;
   e.deltaAf0    = 0.0;
   e.deltaAf1    = 2.0e-12; // сек/сек
   store.longTermBySat_[sat].push_back(e);

   // смещение шкал GPS−GLONASS = 0.5 c → добавляется в Δt (для ГЛОНАСС)
   io::TimeOffsetInterval off{ ep.addSecs(-100), ep.addSecs(100), 0.5 };
   store.setGpsGlonassOffsets({ off });

   rinex::RINEX_FILE nav{};
   nav.navRecords[ep].insert(sat, makeNav(0, 0, 0, 0.0));

   CorrectionApplier ap;
   ASSERT_TRUE(ap.applyCorrectionToNavRecord(nav.navRecords[ep][sat], sat, ep, store));

   // часы: δa_f0 + δa_f1*(t - t0 + 0.5) = 0 + 2e-12*0.5 = 1e-12
   EXPECT_NEAR(nav.navRecords[ep][sat].clock.svClockBias, 1.0e-12, 1e-18);
}

TEST(CorrectionApplier_Basic, RemoveSatIfNoLongTerm) {
   SBASCorrectionStore store; // пустой — LT нет
   const QDateTime     ep = QDateTime::fromString("2020-01-01T00:00:10", Qt::ISODate);
   const Satellite     s1 = makeSat(SatelliteSystem::TYPE::GPS, 1);
   const Satellite     s2 = makeSat(SatelliteSystem::TYPE::GPS, 2);

   rinex::RINEX_FILE nav{};

   nav.navRecords[ep].insert(s1, makeNav(1, 2, 3));
   nav.navRecords[ep].insert(s2, makeNav(4, 5, 6));

   CorrectionApplier ap;
   ap.applySBASCorrections(nav, store); // приватный метод — открыт в тесте

   // оба НКА без LT → оба удаляются
   ASSERT_TRUE(nav.navRecords.contains(ep));
   EXPECT_TRUE(nav.navRecords[ep].isEmpty());
}
