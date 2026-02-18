#include <gtest/gtest.h>
#include <QtCore/QDateTime>

#define private public
#include "errorcalculator.h"
#undef private

using navsolver::ErrorCalculator;

// Вспомогательный конструктор QDateTime (UTC)
static QDateTime T(const char* iso) {
   QDateTime dt = QDateTime::fromString(QString::fromUtf8(iso), Qt::ISODate);

   dt.setTimeSpec(Qt::UTC);
   return dt;
}

static QDateTime gpsTow(int weekOffset, int towSec) {
   QDateTime gps0(QDate(1980, 1, 6), QTime(0, 0, 0), Qt::UTC); // GPS epoch (вс)

   return gps0.addSecs(weekOffset * 604800 + towSec);
}

TEST(ErrorCalculator_Clock, WrapWeek_PositiveLarge) {
   ErrorCalculator ec;
   // +8e5 c  -> должен "завернуть" в диапазон ±302400
   double w = ec.wrapWeek(+800000.0);

   EXPECT_GT(w, -302400.0);
   EXPECT_LT(w, +302400.0);
}

TEST(ErrorCalculator_Clock, WrapWeek_NegativeLarge) {
   ErrorCalculator ec;
   // -9e5 c -> вернёт в пределы ±302400
   double w = ec.wrapWeek(-900000.0);

   EXPECT_GT(w, -302400.0);
   EXPECT_LT(w, +302400.0);
}

TEST(ErrorCalculator_Clock, SecondsOfWeek_MonotonicWithinWeek) {
   ErrorCalculator ec;
   // Две близкие эпохи в пределах одной недели
   auto   t1 = T("2025-01-05T00:00:10Z");
   auto   t2 = T("2025-01-05T00:01:20Z");
   double s1 = ec.secondsOfWeek(t1);
   double s2 = ec.secondsOfWeek(t2);

   EXPECT_LT(s1, s2);
   EXPECT_NEAR(s2 - s1, 70.0, 1e-9);
}

TEST(ErrorCalculator_Clock, GLO_Polynomial_WeekWrap) {
   ErrorCalculator ec;

   // Эпоха: конец недели + чуть-чуть
   // Пусть опорное время эфемерид tr.week_time около конца недели,
   // а наблюдение — уже в начале следующей; wrapWeek должен дать малую разницу.
   rinex::R_TIME tr{};

   tr.week_time        = 604799.0; // [с] ≈ конец недели
   tr.tk               = -1.0;     // не используем
   tr.svClockBias      =  1.0e-5;  // сек
   tr.svClockDrift     =  2.0e-12; // сек/сек
   tr.svClockDriftRate = -5.0e-18; // сек/сек^2

   // наблюдение на 2 секунды позже "по реальному времени недели"
   // secondsOfWeek вернёт маленькое TOW в начале новой недели,
   // wrapWeek(-604797) → +3 с (в зависимости от HALF_WEEK логики).
   auto epoch = T("2025-01-05T00:00:02Z"); // начало недели (+2 с)

   // Вызов закрытого ГЛОНАСС-пути
   double clk = ec.computeBroadcastClockL1_GLO(tr, epoch);

   // Ожидаем: bias + drift*dt + 0.5*driftRate*dt^2, dt — малое (несколько секунд)
   // Формально dt посчитает wrapWeek(secondsOfWeek(epoch) - tr.week_time).
   // Проверим лишь разумные рамки: ≈ 1e-5 ± O(1e-12*сек)
   EXPECT_NEAR(clk, 1.0e-5 + 2.0e-12 * 3.0 + 0.5 * (-5.0e-18) * 9.0, 5e-12);
}

TEST(ErrorCalculator_Clock, NonGLO_Fallback_ReturnsBias) {
   ErrorCalculator ec;
   rinex::R_TIME   tr{};

   tr.week_time        = 1000.0;
   tr.tk               = -1.0;
   tr.svClockBias      = 3.2e-6;
   tr.svClockDrift     = 0.0;
   tr.svClockDriftRate = 0.0;

   Satellite sat(7, SatelliteSystem::TYPE::GPS); // любой не-GLONASS
   auto epoch = T("2025-01-02T03:04:05Z");

   double clk = ec.computeBroadcastClockL1(sat, tr, epoch);
   EXPECT_DOUBLE_EQ(clk, tr.svClockBias);
}


// --- Тест 1: SP3 clock автоскейл (микросекунды → секунды) для не-ГЛОНАСС ---
TEST(ErrorCalculator_MoreClock, Sp3Clock_Microseconds_AutoScale_NoGlo) {
   ErrorCalculator   ec;
   pipeline::Context ctx; // dm не требуется для не-GLONАСС
   const QDateTime   ep = QDateTime::fromString("2025-01-05T12:00:00Z", Qt::ISODate);

   Satellite gps(1, SatelliteSystem::TYPE::GPS);

   // По реализации: |clockL3| > 1e-3 → трактуется как мкс и умножается на 1e-6
   // см. correctSp3ClockL3toL1(...) в errorcalculator.cpp
   const double sp3_clk_L3_microsec = 2500.0; // 2500 мкс
   const double got_sec             = ec.correctSp3ClockL3toL1(ctx, gps, ep, sp3_clk_L3_microsec);

   EXPECT_NEAR(got_sec, 2.5e-3, 1e-15);
}

// --- Тест 2: computeBroadcastClockL1_GLO устойчиво через границу недели ---
TEST(ErrorCalculator_MoreClock, Glo_Polynomial_WeekBoundary_Stable) {
   ErrorCalculator ec;

   // Формируем R_TIME так, чтобы ref-время (tk/week_time) отличалось
   // от эпохи на ±N недель: wrapWeek() должен сложиться, см. реализацию.
   rinex::R_TIME tr{};

   tr.svClockBias      = 1.0e-5;   // сек
   tr.svClockDrift     = 2.0e-12;  // сек/сек
   tr.svClockDriftRate = -5.0e-18; // сек/сек^2

   // Пусть в RINEX записан week_time = 100.0 c
   tr.week_time = 100.0;           // сек от начала недели
   tr.tk        = -1.0;            // чтобы использовался week_time

   // Эпоха: следующая неделя + 3 секунды от начала недели
   const QDateTime ep = gpsTow(+1, 3);

   const double clk = ec.computeBroadcastClockL1(Satellite(5, SatelliteSystem::TYPE::GLONASS), tr, ep);

   // Полином: bias + drift*dt + 0.5*driftRate*dt^2 при dt = -97
   const double dt       = -97.0;
   const double expected = 1.0e-5 + 2.0e-12 * dt + 0.5 * (-5.0e-18) * dt * dt;

   EXPECT_NEAR(clk, expected, 5e-12);
}
