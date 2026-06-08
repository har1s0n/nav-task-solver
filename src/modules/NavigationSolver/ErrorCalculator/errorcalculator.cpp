#include "errorcalculator.h"
#include "modules/IO/datamanager.h"

#include <QTimeZone>
#include <cmath>


#include <QDateTime>
#include <QString>
#include <QVector>
#include <QtDebug>
#include "sbascorrectionstore.h"


// ============================================================================
// Статистика — только необходимые метрики для отладки
// ============================================================================
struct Stats {
   qint64 epochsTotal{ 0 };
   qint64 epochsSkipNoSp3{ 0 };
   qint64 epochsSkipNoNav{ 0 };

   qint64 satsVisible{ 0 };
   qint64 satsProcessed{ 0 };
   qint64 satsMissingNav{ 0 };

   qint64 satsFastLt{ 0 };
   qint64 satsFastOnly{ 0 };
   qint64 satsLtOnly{ 0 };
   qint64 satsNoSbas{ 0 };

   qint64 gpsLtMatched{ 0 };
   qint64 gpsLtMissed{ 0 };
   qint64 gloLtMatched{ 0 };
   qint64 gloLtMissed{ 0 };

   qint64 masterClkApplied{ 0 };
   qint64 masterClkSkipped{ 0 };

   qint64 residualsFinite{ 0 };

   qint64 improvedCount{ 0 };
   qint64 worsenedCount{ 0 };
   qint64 sameCount{ 0 };

   qint64 improvedFastLt{ 0 };
   qint64 worsenedFastLt{ 0 };

   qint64 improvedFastOnly{ 0 };
   qint64 worsenedFastOnly{ 0 };

   qint64 improvedLtOnly{ 0 };
   qint64 worsenedLtOnly{ 0 };

   qint64 masterClkImproved{ 0 };
   qint64 masterClkWorsened{ 0 };

   double sumAbsDeltaSp3{ 0.0 };
   double sumAbsResidualFinal{ 0.0 };
   qint64 sbasFiniteCases{ 0 };
};

// ============================================================================
// Вспомогательные функции
// ============================================================================

enum class SbasMode {
   None,
   FastOnly,
   LtOnly,
   FastLt
};

static inline SbasMode detectSbasMode(bool hasFast, bool hasLt) noexcept {
   if (hasFast && hasLt) {
      return SbasMode::FastLt;
   }

   if (hasFast) {
      return SbasMode::FastOnly;
   }

   if (hasLt) {
      return SbasMode::LtOnly;
   }
   return SbasMode::None;
}

static inline const char*sbasModeToString(SbasMode mode) noexcept {
   switch (mode) {
     case SbasMode::FastLt:   return "FAST_LT";
     case SbasMode::FastOnly: return "FAST_ONLY";
     case SbasMode::LtOnly:   return "LT_ONLY";
     default:                 return "NONE";
   }
}

static inline void accumulateOutcome(Stats&   st,
                                     SbasMode mode,
                                     double   deltaSp3,
                                     double   residualFinal) noexcept {
   if (!qIsFinite(deltaSp3) || !qIsFinite(residualFinal)) {
      return;
   }

   const double a = std::abs(deltaSp3);
   const double b = std::abs(residualFinal);

   st.sumAbsDeltaSp3      += a;
   st.sumAbsResidualFinal += b;

   constexpr double EPS = 1e-6;

   if (b + EPS < a) {
      ++st.improvedCount;

      if (mode == SbasMode::FastLt) {
         ++st.improvedFastLt;
      } else if (mode == SbasMode::FastOnly) {
         ++st.improvedFastOnly;
      } else if (mode == SbasMode::LtOnly) {
         ++st.improvedLtOnly;
      }
   } else if (b > a + EPS) {
      ++st.worsenedCount;

      if (mode == SbasMode::FastLt) {
         ++st.worsenedFastLt;
      } else if (mode == SbasMode::FastOnly) {
         ++st.worsenedFastOnly;
      } else if (mode == SbasMode::LtOnly) {
         ++st.worsenedLtOnly;
      }
   } else {
      ++st.sameCount;
   }
}

static inline void accumulateMasterClkEffect(Stats& st,
                                             double residualRaw,
                                             double residualFinal,
                                             bool   clkApplied) noexcept {
   if (!clkApplied || !qIsFinite(residualRaw) || !qIsFinite(residualFinal)) {
      return;
   }

   const double a = std::abs(residualRaw);
   const double b = std::abs(residualFinal);

   constexpr double EPS = 1e-6;

   if (b + EPS < a) {
      ++st.masterClkImproved;
   } else if (b > a + EPS) {
      ++st.masterClkWorsened;
   }
}

static inline double secondsOfWeekUtc(const QDateTime& tUtc) noexcept {
   static const QDateTime gpsEpoch(QDate(1980, 1, 6), QTime(0, 0, 0), Qt::UTC);
   const qint64 secs = gpsEpoch.secsTo(tUtc.toUTC());
   const qint64 mod  = ((secs % 604800) + 604800) % 604800;

   return static_cast<double> (mod);
}

static inline double wrapWeek(double dt) noexcept {
   constexpr double HALF_WEEK = 302400.0;

   if (dt >  HALF_WEEK) {
      dt -= 2.0 * HALF_WEEK;
   }

   if (dt < -HALF_WEEK) {
      dt += 2.0 * HALF_WEEK;
   }
   return dt;
}

static inline double wrapGpsWeekSeconds(double dt) noexcept {
   constexpr double WEEK = 604800.0;
   constexpr double HALF = WEEK / 2.0;

   if (dt > HALF) {
      dt -= WEEK;
   } else if (dt < -HALF) {
      dt += WEEK;
   }

   return dt;
}

static inline double secondsOfWeekWall(const QDateTime& t) noexcept {
   const QDate d  = t.date();
   const QTime tm = t.time();

   const int dow = d.dayOfWeek() % 7;

   return static_cast<double> (dow) * 86400.0 +
          static_cast<double> (tm.hour()) * 3600.0 +
          static_cast<double> (tm.minute()) * 60.0 +
          static_cast<double> (tm.second()) +
          static_cast<double> (tm.msec()) * 1e-3;
}

static inline double norm3(const COORD_XYZ& p) noexcept {
   return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

static inline COORD_XYZ scaleKmToM_ifNeeded(const COORD_XYZ& p) noexcept {
   const double n      = norm3(p);
   const bool   kmLike = std::isfinite(n) && (n > 1e3) && (n < 1e6);

   if (kmLike) {
      return COORD_XYZ{ p.x * 1000.0, p.y * 1000.0, p.z * 1000.0 };
   }

   return p;
}

template<typename TNavRecords>
static inline auto findNavEpochLE(const TNavRecords& navRecs,
                                  const QDateTime&   epoch) -> typename TNavRecords::const_iterator {
   auto it = navRecs.upperBound(epoch);

   if (it == navRecs.cbegin()) {
      return navRecs.cend();
   }

   --it;
   return it;
}

// ============================================================================
// GPS Kepler orbit propagation (IS-GPS-200)
// ============================================================================

static inline COORD_XYZ evaluateGpsBroadcastCoordKmAtTow(const rinex::NAV_RECORD& nd,
                                                         double                   towSec) noexcept {
   const long double tk = static_cast<long double> (wrapGpsWeekSeconds(towSec - nd.Toe));
   const long double A  = static_cast<long double> (nd.sqrt_A) * static_cast<long double> (nd.sqrt_A);

   const long double n0 = std::sqrt(constants::WGS84::GMe / std::pow(A, 3));
   const long double n  = n0 + static_cast<long double> (nd.delta_n);
   const long double Mk = static_cast<long double> (nd.M0) + n * tk;

   long double Ek   = Mk;
   long double prev = 0.0;

   while (std::fabs(Ek - prev) > 1.0e-13L) {
      prev = Ek;
      Ek   = Mk + static_cast<long double> (nd.e) * std::sin(prev);
   }

   long double vk = std::atan2(std::sqrt(1.0L - std::pow(static_cast<long double> (nd.e), 2)) * std::sin(Ek),
                               std::cos(Ek) - static_cast<long double> (nd.e));

   if (vk < 0.0L) {
      vk += 2.0L * M_PI;
   }

   const long double PHk = vk + static_cast<long double> (nd.omega);

   const long double delta_u_k =
      static_cast<long double> (nd.Cuc) * std::cos(2.0L * PHk) +
      static_cast<long double> (nd.Cus) * std::sin(2.0L * PHk);

   const long double delta_r_k =
      static_cast<long double> (nd.Crc) * std::cos(2.0L * PHk) +
      static_cast<long double> (nd.Crs) * std::sin(2.0L * PHk);

   const long double delta_i_k =
      static_cast<long double> (nd.Cic) * std::cos(2.0L * PHk) +
      static_cast<long double> (nd.Cis) * std::sin(2.0L * PHk);

   const long double uk = PHk + delta_u_k;
   const long double rk = A * (1.0L - static_cast<long double> (nd.e) * std::cos(Ek)) + delta_r_k;
   const long double ik = static_cast<long double> (nd.i0) + static_cast<long double> (nd.IDOT) * tk + delta_i_k;

   const long double xk = rk * std::cos(uk);
   const long double yk = rk * std::sin(uk);

   const long double OMEGA_k =
      static_cast<long double> (nd.OMEGA0) +
      (static_cast<long double> (nd.OMEGA_DOT) - static_cast<long double> (constants::WGS84::omega)) * tk -
      static_cast<long double> (constants::WGS84::omega) * static_cast<long double> (nd.Toe);

   const long double X = xk * std::cos(OMEGA_k) - yk * std::sin(OMEGA_k) * std::cos(ik);
   const long double Y = xk * std::sin(OMEGA_k) + yk * std::cos(OMEGA_k) * std::cos(ik);
   const long double Z = yk * std::sin(ik);

   return COORD_XYZ(static_cast<double> (X / 1000.0L),
                    static_cast<double> (Y / 1000.0L),
                    static_cast<double> (Z / 1000.0L));
}

// ============================================================================
// GLONASS state propagation
// ============================================================================

struct GloState {
   double x;
   double y;
   double z;
   double vx;
   double vy;
   double vz;
};

static inline GloState gloDerivatives(const GloState&  s,
                                      const COORD_XYZ& luniSolarAcc) noexcept {
   // Единицы: км, км/с, км/с^2
   constexpr double MU    = 398600.44;   // км^3/с^2
   constexpr double AE    = 6378.136;    // км
   constexpr double J2    = 1.08262575e-3;
   constexpr double OMEGA = 7.292115e-5; // рад/с

   const double r2 = s.x * s.x + s.y * s.y + s.z * s.z;
   const double r  = std::sqrt(r2);
   const double r3 = r2 * r;
   const double r5 = r3 * r2;
   const double z2 = s.z * s.z;

   const double kJ2 = 1.5 * J2 * MU * AE * AE / r5;
   const double c   = 5.0 * z2 / r2;

   const double ax =
      -MU * s.x / r3
      - kJ2 * s.x * (1.0 - c)
      + OMEGA * OMEGA * s.x
      + 2.0 * OMEGA * s.vy
      + luniSolarAcc.x;

   const double ay =
      -MU * s.y / r3
      - kJ2 * s.y * (1.0 - c)
      + OMEGA * OMEGA * s.y
      - 2.0 * OMEGA * s.vx
      + luniSolarAcc.y;

   const double az =
      -MU * s.z / r3
      - kJ2 * s.z * (3.0 - c)
      + luniSolarAcc.z;

   return GloState{
      s.vx, s.vy, s.vz,
      ax,   ay,   az
   };
}

static inline GloState addScaled(const GloState& a,
                                 const GloState& b,
                                 double          scale) noexcept {
   return GloState{
      a.x  + b.x  * scale,
      a.y  + b.y  * scale,
      a.z  + b.z  * scale,
      a.vx + b.vx * scale,
      a.vy + b.vy * scale,
      a.vz + b.vz * scale
   };
}

static inline GloState rk4Step(const GloState&  s,
                               const COORD_XYZ& luniSolarAcc,
                               double           h) noexcept {
   const GloState k1 = gloDerivatives(s, luniSolarAcc);
   const GloState k2 = gloDerivatives(addScaled(s, k1, 0.5 * h), luniSolarAcc);
   const GloState k3 = gloDerivatives(addScaled(s, k2, 0.5 * h), luniSolarAcc);
   const GloState k4 = gloDerivatives(addScaled(s, k3, h),       luniSolarAcc);

   GloState out = s;

   out.x  += (h / 6.0) * (k1.x  + 2.0 * k2.x  + 2.0 * k3.x  + k4.x);
   out.y  += (h / 6.0) * (k1.y  + 2.0 * k2.y  + 2.0 * k3.y  + k4.y);
   out.z  += (h / 6.0) * (k1.z  + 2.0 * k2.z  + 2.0 * k3.z  + k4.z);
   out.vx += (h / 6.0) * (k1.vx + 2.0 * k2.vx + 2.0 * k3.vx + k4.vx);
   out.vy += (h / 6.0) * (k1.vy + 2.0 * k2.vy + 2.0 * k3.vy + k4.vy);
   out.vz += (h / 6.0) * (k1.vz + 2.0 * k2.vz + 2.0 * k3.vz + k4.vz);
   return out;
}

static inline COORD_XYZ evaluateGlonassBroadcastCoordAtEpoch(
   const rinex::NAV_RECORD& navRec,
   const QDateTime&         navEpoch,
   const QDateTime&         epoch) noexcept {
   const double dtTotal = static_cast<double> (navEpoch.secsTo(epoch));

   if (std::abs(dtTotal) < 1e-9) {
      return navRec.coord;
   }

   GloState s{
      navRec.coord.x,
      navRec.coord.y,
      navRec.coord.z,
      navRec.velocity.x,
      navRec.velocity.y,
      navRec.velocity.z
   };

   const COORD_XYZ luniSolarAcc{
      navRec.acceleration.x,
      navRec.acceleration.y,
      navRec.acceleration.z
   };

   double remaining = dtTotal;

   while (std::abs(remaining) > 1e-9) {
      const double h = (std::abs(remaining) > 60.0)
        ? (remaining > 0.0 ? 60.0 : -60.0)
        : remaining;

      s          = rk4Step(s, luniSolarAcc, h);
      remaining -= h;
   }

   return COORD_XYZ{ s.x, s.y, s.z };
}

// ============================================================================
// ГЛОНАСС LT applicability (временной gate)
// ============================================================================

static inline QDateTime glonassTrFromNavRecord(const rinex::NAV_RECORD& rec) {
   return rec.clock.epoch;
}

struct GlonassLtIodWindow {
   int  V_sec{ 0 };
   int  L_sec{ 0 };
   bool valid{ false };
};

static inline GlonassLtIodWindow decodeGlonassLtIod(const int iod) noexcept {
   if ((iod < 0) || (iod > 255)) {
      return {};
   }

   const int vBits = (iod & 0x1F);
   const int lBits = ((iod >> 5) & 0x07);

   GlonassLtIodWindow out;
   out.V_sec = vBits * 30;
   out.L_sec = lBits * 30;
   out.valid = (out.V_sec >= 30) && (out.V_sec <= 960) &&
               (out.L_sec >= 0)  && (out.L_sec <= 210);

   return out;
}

static inline bool passesGlonassLtApplicability(const rinex::NAV_RECORD&           navRec,
                                                const io::LongTermCorrectionEntry& lt) {
   const auto w = decodeGlonassLtIod(lt.iod);

   if (!w.valid || !lt.recvTime.isValid()) {
      return false;
   }
   const auto tr = glonassTrFromNavRecord(navRec);

   if (!tr.isValid()) {
      return false;
   }
   const int  dtSec = static_cast<int> (tr.toUTC().secsTo(lt.recvTime.toUTC()));
   const bool ok    = (dtSec >= w.L_sec) && (dtSec <= (w.L_sec + w.V_sec));

   return ok;
}

// ============================================================================
// LT correction helpers
// ============================================================================

static inline int secondsOfDaySnt(const QDateTime& t) noexcept {
   return (t.time().msecsSinceStartOfDay() / 1000) + 18;
}

static inline int wrapDaySeconds(int dt) noexcept {
   if (dt > 43200) {
      dt -= 86400;
   }

   if (dt < -43200) {
      dt += 86400;
   }
   return dt;
}

static inline int computeLtDeltaSecSnt(const io::LongTermCorrectionEntry& lt,
                                       const QDateTime&                   epoch) noexcept {
   return wrapDaySeconds(secondsOfDaySnt(epoch) - lt.t0);
}

static inline COORD_XYZ evaluateLtDeltaR(const io::LongTermCorrectionEntry& lt,
                                         int                                deltaSec) noexcept {
   COORD_XYZ dR = lt.deltaPos;

   if (lt.hasVelocity) {
      dR.x += lt.deltaVel.x * static_cast<double> (deltaSec);
      dR.y += lt.deltaVel.y * static_cast<double> (deltaSec);
      dR.z += lt.deltaVel.z * static_cast<double> (deltaSec);
   }

   return dR;
}

static inline double evaluateLtClockMeters(const io::LongTermCorrectionEntry& lt,
                                           int                                deltaSec) noexcept {
   const double clockCorr_s = lt.deltaAf0 + lt.deltaAf1 * static_cast<double> (deltaSec);

   return constants::PZ_9011::c * clockCorr_s;
}

// ============================================================================
// GPS NAV поиск по IODE (в обе стороны по времени)
// ============================================================================

/**
 * @brief Поиск GPS NAV записи по совпадению IODE с LT correction.
 *        Ищет как назад, так и вперёд по времени (±maxAgeSec).
 */
static inline std::optional<rinex::NAV_RECORD> findGpsNavByIode(const QMap<QDateTime, QMap<Satellite, rinex::NAV_RECORD> >& navRecs,
                                                                const Satellite&                                            sat,
                                                                int                                                         targetIode,
                                                                const QDateTime&                                            epoch,
                                                                int                                                         maxAgeSec =
                                                                14400) {
   // Поиск назад
   auto itBack = navRecs.upperBound(epoch);

   while (itBack != navRecs.cbegin()) {
      --itBack;

      const QDateTime& navEpoch = itBack.key();
      const qint64     ageSec   = navEpoch.secsTo(epoch);

      if (ageSec > maxAgeSec) {
         break;
      }

      const auto& satMap = itBack.value();
      auto satIt         = satMap.constFind(sat);

      if (satIt == satMap.cend()) {
         continue;
      }

      const rinex::NAV_RECORD& navRec = satIt.value();
      const int iode                  = qRound(navRec.IODE);
      const int iodc8                 = qRound(navRec.IODC) & 0xFF;

      if ((targetIode == iode) && (targetIode == iodc8)) {
         return navRec;
      }
   }

   // Поиск вперёд
   auto itFwd = navRecs.upperBound(epoch);

   while (itFwd != navRecs.cend()) {
      const QDateTime& navEpoch = itFwd.key();
      const qint64     ageSec   = epoch.secsTo(navEpoch);

      if (ageSec > maxAgeSec) {
         break;
      }

      const auto& satMap = itFwd.value();
      auto satIt         = satMap.constFind(sat);

      if (satIt != satMap.cend()) {
         const rinex::NAV_RECORD& navRec = satIt.value();
         const int iode                  = qRound(navRec.IODE);
         const int iodc8                 = qRound(navRec.IODC) & 0xFF;

         if ((targetIode == iode) && (targetIode == iodc8)) {
            return navRec;
         }
      }

      ++itFwd;
   }

   return std::nullopt;
}

bool navsolver::ErrorCalculator::execute(pipeline::Context& ctx) {
   if (!ctx.dm || ctx.gridPoints.isEmpty() || ctx.visibleSats.isEmpty()) {
      qWarning() << "[ErrorCalculator] Недостаточно данных для расчета.";
      return false;
   }
   computeResidualErrors(ctx);

   return true;
}

void navsolver::ErrorCalculator::computeResidualErrors(pipeline::Context& ctx) {
   if (!ctx.dm) {
      qWarning() << "[ErrorCalculator] Некорректный Context (ctx.dm=nullptr)";
      return;
   }

   const auto* rinexNavGlo = ctx.dm->getRinexGlonassFile();
   const auto* rinexNavGps = ctx.dm->getRinexGpsFile();

   const bool noGloNav = (!rinexNavGlo || rinexNavGlo->navRecords.isEmpty());
   const bool noGpsNav = (!rinexNavGps || rinexNavGps->navRecords.isEmpty());

   if (noGloNav && noGpsNav) {
      qWarning() << "[ErrorCalculator] NAV отсутствует или не загружен (GLONASS и GPS)";
      return;
   }

   const auto* sp3 = ctx.dm->getSP3File();

   if (!sp3 || sp3->records.isEmpty()) {
      qWarning() << "[ErrorCalculator] SP3 отсутствует или пуст";
      return;
   }

   ctx.residualErrors.clear();

   Stats st;

   const auto& sp3Recs = sp3->records;
   const auto& sbas    = ctx.dm->getSBASStore();

   qint64 dcbMissing = 0;

   struct TempResidual {
      Satellite sat;
      SbasMode  mode{ SbasMode::None };

      double deltaSp3{ qQNaN() };
      double deltaSdcm{ qQNaN() };

      double prc{ 0.0 };
      double rrc{ 0.0 };
      double drLos{ 0.0 };
      double dtLt{ 0.0 };

      double    residualRaw{ qQNaN() };
      COORD_XYZ satEcef{};
   };

   for (auto itEpoch = ctx.visibleSats.cbegin(); itEpoch != ctx.visibleSats.cend(); ++itEpoch) {
      ++st.epochsTotal;

      const QDateTime& epoch = itEpoch.key();
      const auto& pointMap   = itEpoch.value();

      const auto itSp3Epoch = sp3Recs.constFind(epoch);

      if (itSp3Epoch == sp3Recs.cend()) {
         ++st.epochsSkipNoSp3;
         continue;
      }

      const auto* navRecsGlo = noGloNav ? nullptr : &rinexNavGlo->navRecords;
      const auto* navRecsGps = noGpsNav ? nullptr : &rinexNavGps->navRecords;

      decltype(rinexNavGlo->navRecords.cbegin()) itNavEpochGlo;
      decltype(rinexNavGps->navRecords.cbegin()) itNavEpochGps;

      const QMap<Satellite, rinex::NAV_RECORD>* navBySatGlo = nullptr;
      const QMap<Satellite, rinex::NAV_RECORD>* navBySatGps = nullptr;

      if (navRecsGlo) {
         itNavEpochGlo = findNavEpochLE(*navRecsGlo, epoch);

         if (itNavEpochGlo != navRecsGlo->cend()) {
            navBySatGlo = &itNavEpochGlo.value();
         }
      }

      if (navRecsGps) {
         itNavEpochGps = findNavEpochLE(*navRecsGps, epoch);

         if (itNavEpochGps != navRecsGps->cend()) {
            navBySatGps = &itNavEpochGps.value();
         }
      }

      if (!navBySatGlo && !navBySatGps) {
         ++st.epochsSkipNoNav;
         continue;
      }

      const auto& sp3BySat = itSp3Epoch.value();

      for (auto itPoint = pointMap.cbegin(); itPoint != pointMap.cend(); ++itPoint) {
         const auto& point = itPoint.key();
         const auto& sats  = itPoint.value();

         if (sats.isEmpty()) {
            continue;
         }

         st.satsVisible += sats.size();

         QVector<TempResidual> temp;
         temp.reserve(sats.size());

         const COORD_XYZ r_obs = point.ecef;

         for (const auto& sat : sats) {
            const auto itSp3Sat = sp3BySat.constFind(sat);

            if (itSp3Sat == sp3BySat.cend()) {
               continue;
            }

            const QMap<Satellite, rinex::NAV_RECORD>* navBySat = nullptr;

            if (sat.getSystem() == SatelliteSystem::TYPE::GLONASS) {
               navBySat = navBySatGlo;
            } else if (sat.getSystem() == SatelliteSystem::TYPE::GPS) {
               navBySat = navBySatGps;
            } else {
               continue;
            }

            if (!navBySat) {
               ++st.satsMissingNav;
               continue;
            }

            const auto itNavSat = navBySat->constFind(sat);

            if (itNavSat == navBySat->cend()) {
               ++st.satsMissingNav;
               continue;
            }

            const auto& sp3Rec       = itSp3Sat.value();
            rinex::NAV_RECORD navRec = itNavSat.value();

            // ###
            const QDateTime navEpochPicked =
               (sat.getSystem() == SatelliteSystem::TYPE::GLONASS &&
                navRecsGlo &&
                itNavEpochGlo != navRecsGlo->cend())
                    ? itNavEpochGlo.key()
                    : QDateTime{};

            auto prc = sbas.fastPrc_m(sat, epoch);

            // ========== Получение LT с учётом системы ==========
            std::optional<io::LongTermCorrectionEntry> lt;

            if (sat.getSystem() == SatelliteSystem::TYPE::GPS) {
               auto allLt = sbas.getAllActiveLongTermCorrections(sat, epoch);

               for (const auto& candidate : allLt) {
                  auto matchingNav = findGpsNavByIode(*navRecsGps, sat, candidate.iod, epoch, 14400);

                  if (matchingNav.has_value()) {
                     lt     = candidate;
                     navRec = *matchingNav;
                     ++st.gpsLtMatched;
                     break;
                  }
               }

               if (!lt && !allLt.isEmpty()) {
                  ++st.gpsLtMissed;
               }
            } else if (sat.getSystem() == SatelliteSystem::TYPE::GLONASS) {
               lt = sbas.getLongTermCorrection(sat, epoch);

               if (lt) {
                  const rinex::NAV_RECORD* navForLtGate = nullptr;

                  if (navRecsGlo && lt->recvTime.isValid()) {
                     auto itGate = findNavEpochLE(*navRecsGlo, lt->recvTime);

                     if (itGate != navRecsGlo->cend()) {
                        auto itSatGate = itGate.value().constFind(sat);

                        if (itSatGate != itGate.value().cend()) {
                           navForLtGate = &itSatGate.value();
                        }
                     }
                  }

                  const bool gateOk = navForLtGate
                                            ? passesGlonassLtApplicability(*navForLtGate, *lt)
                                            : false;

                  if (gateOk) {
                     ++st.gloLtMatched;
                  } else {
                     ++st.gloLtMissed;
                     lt.reset();
                  }
               }
            }

            // ========== Координаты SP3 (phase center) ==========
            const COORD_XYZ sp3_cm_m = scaleKmToM_ifNeeded(sp3Rec.coord);
            COORD_XYZ r_sp3_pc       = sp3_cm_m;

            const auto* antennaModel = ctx.dm->getAntennaModel();

            if (antennaModel && antennaModel->isReady()) {
               auto band   = antex::SatelliteAntennaModel::defaultBandForSystem(sat.getSystem());
               auto result = antennaModel->convertMassCenter2PhaseCenter(sat, epoch, band, sp3_cm_m, r_obs);

               if (!result.isValid) {
                  if (itPoint == pointMap.cbegin()) {
                     qWarning().noquote()
                        << QString("[EC][CM2PC_FAIL] epoch=%1 sat=%2 code=%3 error=%4")
                        .arg(epoch.toString(Qt::ISODate))
                        .arg(sat.toString())
                        .arg(result.errorCode)
                        .arg(result.errorMessage);
                  }
                  continue;
               }

               r_sp3_pc = result.phaseCenter;
            }

            // ========== Координаты NAV (broadcast) ==========
            COORD_XYZ navCoordKm = navRec.coord;

            if (sat.getSystem() == SatelliteSystem::TYPE::GPS) {
               const double gpsTowOrbit = secondsOfWeekWall(epoch);
               navCoordKm = evaluateGpsBroadcastCoordKmAtTow(navRec, gpsTowOrbit);
            } else if (sat.getSystem() == SatelliteSystem::TYPE::GLONASS) {
               navCoordKm = evaluateGlonassBroadcastCoordAtEpoch(navRec, navEpochPicked, epoch);
            }

            const COORD_XYZ nav_m = scaleKmToM_ifNeeded(navCoordKm);

            // ========== LOS вектор ==========
            COORD_XYZ los = {
               r_sp3_pc.x - r_obs.x,
               r_sp3_pc.y - r_obs.y,
               r_sp3_pc.z - r_obs.z
            };

            const double losNorm = std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);

            if (!(losNorm > 0.0) || !std::isfinite(losNorm)) {
               continue;
            }

            los.x /= losNorm;
            los.y /= losNorm;
            los.z /= losNorm;

            // ========== ΔSP3 (orbit + clock) ==========
            const COORD_XYZ d_sp3_nav = {
               r_sp3_pc.x - nav_m.x,
               r_sp3_pc.y - nav_m.y,
               r_sp3_pc.z - nav_m.z
            };

            const double clkSp3L1 = correctSp3ClockL3toL1(ctx, sat, epoch, sp3Rec.clock, &dcbMissing);
            const double clkNavL1 = computeBroadcastClockL1(sat, navRec, epoch);

            const double orbitProj_m =
               d_sp3_nav.x * los.x +
               d_sp3_nav.y * los.y +
               d_sp3_nav.z * los.z;

            const double clkDiff_s  = clkSp3L1 - clkNavL1;
            const double clkTerm_m  = constants::PZ_9011::c * clkDiff_s;
            const double deltaSp3_m = orbitProj_m - clkTerm_m;

            // ========== SBAS коррекции ==========
            const bool hasFast    = prc.has_value();
            const bool hasLt      = lt.has_value();
            const bool hasAnySbas = hasFast || hasLt;
            // const double   prc_m      = hasFast ? *prc : 0.0;
            const double   prc_m    = hasFast ? -(*prc) : 0.0;
            const SbasMode sbasMode = detectSbasMode(hasFast, hasLt);

            if (hasFast && hasLt) {
               ++st.satsFastLt;
            } else if (hasFast) {
               ++st.satsFastOnly;
            } else if (hasLt) {
               ++st.satsLtOnly;
            } else {
               ++st.satsNoSbas;
            }

            double dr_los_m = 0.0;
            double dt_lt_m  = 0.0;

            if (hasLt) {
               const int ltDeltaSec   = computeLtDeltaSecSnt(*lt, epoch);
               const COORD_XYZ deltaR = evaluateLtDeltaR(*lt, ltDeltaSec);

               dr_los_m = deltaR.x * los.x + deltaR.y * los.y + deltaR.z * los.z;
               dt_lt_m  = evaluateLtClockMeters(*lt, ltDeltaSec);
            }

            double rrc_m = 0.0;

            if (hasFast) {
               auto fp = sbas.fastCurrentPrevious(sat, epoch);

               if (fp && qIsFinite(fp->current.prc_m) && qIsFinite(fp->previous.prc_m)) {
                  const double dt_fast = fp->previous.recvTime.secsTo(fp->current.recvTime);

                  if (std::abs(dt_fast) > 0.5) {
                     const double rrc      = (fp->current.prc_m - fp->previous.prc_m) / dt_fast;
                     const double dt_apply = fp->current.recvTime.secsTo(epoch);

                     // rrc_m = rrc * dt_apply;
                     rrc_m = 0.0;
                  }
               }
            }

            // ========== Residual формула ==========
            const double deltaSdcm_m =
               hasAnySbas ? ((hasLt ? (dr_los_m - dt_lt_m) : 0.0) - prc_m - rrc_m)
                               : qQNaN();

            const double residualRaw_m =
               hasAnySbas ? (deltaSp3_m - deltaSdcm_m) : qQNaN();

            // ###
            if (hasAnySbas && (itPoint == pointMap.cbegin())) {
               qDebug().noquote()
                  << QString("[EC][SBAS_DIAG] ep=%1 sat=%2 mode=%3 orbitProj=%4 clkTerm=%5 "
                             "dr_los=%6 dt_lt=%7 prc=%8 dSp3=%9 dSdcm=%10 resid=%11 gap=%12")
                  .arg(epoch.toString(Qt::ISODate)).arg(sat.toString()).arg(int(sbasMode))
                  .arg(orbitProj_m,       0, 'f', 3).arg(clkTerm_m, 0, 'f', 3)
                  .arg(dr_los_m,          0, 'f', 3).arg(dt_lt_m, 0, 'f', 3).arg(prc_m, 0, 'f', 3)
                  .arg(deltaSp3_m,        0, 'f', 3).arg(deltaSdcm_m, 0, 'f', 3).arg(residualRaw_m, 0, 'f', 3)
                  .arg(prc_m - clkTerm_m, 0, 'f', 3);
            }

            if (hasAnySbas && (itPoint == pointMap.cbegin())) {
               const int navIode = std::isfinite(navRec.IODE) ? qRound(navRec.IODE) : -1; // IODE записи deltaSp3 (для GPS — после
                                                                                          // IODE-replace)
               const auto fIodp = sbas.fastIodp(sat, epoch);
               const auto fLast = sbas.fastLastUpdate(sat, epoch);
               qInfo().noquote()
                  << QString("[EC][BIND] ep=%1 sat=%2 sys=%3 | dSp3.navIODE=%4 navEpoch=%5 "
                             "| LT iod=%6 iodp=%7 t0=%8 recv=%9 daf0=%10 daf1=%11 dPos=(%12,%13,%14) "
                             "| FAST iodp=%15 recv=%16")
                  .arg(epoch.toString(Qt::ISODate)).arg(sat.toString()).arg(int(sat.getSystem()))
                  .arg(navIode)
                  .arg(navEpochPicked.isValid() ? navEpochPicked.toString(Qt::ISODate) : QStringLiteral("—"))
                  .arg(lt ? lt->iod  : -1).arg(lt ? lt->iodp : -1).arg(lt ? lt->t0 : -1)
                  .arg(lt ? lt->recvTime.toString(Qt::ISODateWithMs) : QStringLiteral("—"))
                  .arg(lt ? lt->deltaAf0 : qQNaN(),   0, 'e', 3)
                  .arg(lt ? lt->deltaAf1 : qQNaN(),   0, 'e', 3)
                  .arg(lt ? lt->deltaPos.x : qQNaN(), 0, 'f', 3)
                  .arg(lt ? lt->deltaPos.y : qQNaN(), 0, 'f', 3)
                  .arg(lt ? lt->deltaPos.z : qQNaN(), 0, 'f', 3)
                  .arg(fIodp ? *fIodp : -1)
                  .arg(fLast ? fLast->toString(Qt::ISODateWithMs) : QStringLiteral("—"));
            }
            // ###

            temp.push_back(TempResidual{
               .sat         = sat,
               .mode        = sbasMode,
               .deltaSp3    = deltaSp3_m,
               .deltaSdcm   = deltaSdcm_m,
               .prc         = prc_m,
               .rrc         = rrc_m,
               .drLos       = dr_los_m,
               .dtLt        = dt_lt_m,
               .residualRaw = residualRaw_m,
               .satEcef     = r_sp3_pc
            });
         }

         if (temp.isEmpty()) {
            continue;
         }

         // ========== Шаг 8 методики: master-clock correction по GPS ==========
         double gpsResidualSum   = 0.0;
         qint64 gpsResidualCount = 0;

         for (const auto& tr : temp) {
            if (!qIsFinite(tr.residualRaw)) {
               continue;
            }

            if (tr.sat.getSystem() == SatelliteSystem::TYPE::GPS) {
               gpsResidualSum += tr.residualRaw;
               ++gpsResidualCount;
            }
         }

         double deltaClk_m = qQNaN();

         if (gpsResidualCount > 0) {
            deltaClk_m = gpsResidualSum / static_cast<double> (gpsResidualCount);
            ++st.masterClkApplied;
         } else {
            ++st.masterClkSkipped;
         }

         // Лог master clock (только первая точка в эпохе)
         if ((gpsResidualCount > 0) && (itPoint == pointMap.cbegin())) {
            qDebug().noquote()
               << QString("[EC][MASTER_CLK] epoch=%1 deltaClk_m=%2 gpsCount=%3")
               .arg(epoch.toString(Qt::ISODate))
               .arg(deltaClk_m, 0, 'f', 4)
               .arg(gpsResidualCount);
         }

         // ========== Формирование результата ==========
         QVector<ResidualError> result;
         result.reserve(temp.size());

         for (const auto& tr : temp) {
            double residualFinal_m = tr.residualRaw;

            if (qIsFinite(deltaClk_m) && qIsFinite(residualFinal_m)) {
               residualFinal_m -= deltaClk_m;
            }

            const bool clkApplied = qIsFinite(deltaClk_m);
            accumulateMasterClkEffect(st, tr.residualRaw, residualFinal_m, clkApplied);

            if (qIsFinite(residualFinal_m)) {
               ++st.residualsFinite;

               if (tr.mode != SbasMode::None) {
                  ++st.sbasFiniteCases;
               }
               accumulateOutcome(st, tr.mode, tr.deltaSp3, residualFinal_m);
            }

            result.append(ResidualError{
               .satellite = tr.sat,
               .deltaSp3  = tr.deltaSp3,
               .deltaSdcm = tr.deltaSdcm,
               .residual  = residualFinal_m,
               .satEcef   = tr.satEcef
            });

            ++st.satsProcessed;
         }

         if (!result.isEmpty()) {
            ctx.residualErrors[epoch][point] = std::move(result);
         }
      }
   }

   // ========== Итоговая статистика ==========
   qInfo().noquote()
      << QString("[ErrorCalculator][STATS] epochs=%1 skipSp3=%2 skipNav=%3 "
                 "visible=%4 processed=%5 missingNav=%6 dcbMissing=%7 masterClkApplied=%8 masterClkSkipped=%9")
      .arg(st.epochsTotal)
      .arg(st.epochsSkipNoSp3)
      .arg(st.epochsSkipNoNav)
      .arg(st.satsVisible)
      .arg(st.satsProcessed)
      .arg(st.satsMissingNav)
      .arg(dcbMissing)
      .arg(st.masterClkApplied)
      .arg(st.masterClkSkipped);

   qInfo().noquote()
      << QString("[ErrorCalculator][SBAS] fast+lt=%1 fastOnly=%2 ltOnly=%3 noSbas=%4 "
                 "gpsLtOk=%5 gpsLtMiss=%6 gloLtOk=%7 gloLtMiss=%8")
      .arg(st.satsFastLt)
      .arg(st.satsFastOnly)
      .arg(st.satsLtOnly)
      .arg(st.satsNoSbas)
      .arg(st.gpsLtMatched)
      .arg(st.gpsLtMissed)
      .arg(st.gloLtMatched)
      .arg(st.gloLtMissed);

   qInfo().noquote()
      << QString("[ErrorCalculator][SURROGATE] finite=%1 sbasFinite=%2 improved=%3 worsened=%4 same=%5 "
                 "mean|SP3-NAV|=%6 mean|residualFinal|=%7 "
                 "impr(F+LT)=%8 impr(F)=%9 impr(LT)=%10 "
                 "worse(F+LT)=%11 worse(F)=%12 worse(LT)=%13 "
                 "clkImproved=%14 clkWorsened=%15")
      .arg(st.residualsFinite)
      .arg(st.sbasFiniteCases)
      .arg(st.improvedCount)
      .arg(st.worsenedCount)
      .arg(st.sameCount)
      .arg(st.residualsFinite > 0 ? st.sumAbsDeltaSp3 / st.residualsFinite : 0.0,      0, 'f', 4)
      .arg(st.residualsFinite > 0 ? st.sumAbsResidualFinal / st.residualsFinite : 0.0, 0, 'f', 4)
      .arg(st.improvedFastLt)
      .arg(st.improvedFastOnly)
      .arg(st.improvedLtOnly)
      .arg(st.worsenedFastLt)
      .arg(st.worsenedFastOnly)
      .arg(st.worsenedLtOnly)
      .arg(st.masterClkImproved)
      .arg(st.masterClkWorsened);
}

// ============================================================================
// Clock correction methods
// ============================================================================

double navsolver::ErrorCalculator::correctSp3ClockL3toL1(pipeline::Context& ctx,
                                                         const Satellite&   sat,
                                                         const QDateTime&   epoch,
                                                         double             clockL3) const {
   return correctSp3ClockL3toL1(ctx, sat, epoch, clockL3, nullptr);
}

double navsolver::ErrorCalculator::correctSp3ClockL3toL1(pipeline::Context& ctx,
                                                         const Satellite&   sat,
                                                         const QDateTime&   epoch,
                                                         double             clockL3,
                                                         qint64*            dcbMissingCounter) const {
   double sp3ClockSec = (std::fabs(clockL3) > 1e-3) ? (clockL3 * 1e-6) : clockL3;

   if (sat.getSystem() == SatelliteSystem::TYPE::GLONASS) {
      const auto dcb_ns = ctx.dm->getGlonassL3MinusL1Bias(sat);

      if (dcb_ns.has_value()) {
         sp3ClockSec += dcb_ns.value() * 1e-9;
      } else if (dcbMissingCounter) {
         ++(*dcbMissingCounter);
      }
   }

   return sp3ClockSec;
}

double navsolver::ErrorCalculator::computeBroadcastClockL1_GLO(const rinex::R_TIME& tr,
                                                               const QDateTime&     epoch) {
   const double tref = (tr.week_time >= 0.0) ? tr.week_time : tr.tk;
   const double tow  = secondsOfWeekUtc(epoch);
   const double dt   = wrapWeek(tow - tref);

   return tr.svClockBias +
          tr.svClockDrift * dt +
          0.5 * tr.svClockDriftRate * dt * dt;
}

double navsolver::ErrorCalculator::computeBroadcastClockL1_GPS(const rinex::NAV_RECORD& navRec, const QDateTime& epoch) {
   const auto& tr = navRec.clock;

   const double toc = secondsOfWeekUtc(tr.epoch);
   const double tow = secondsOfWeekUtc(epoch);
   const double dt  = wrapWeek(tow - toc);

   const double clkL1 =
      tr.svClockBias +
      tr.svClockDrift     * dt +
      tr.svClockDriftRate * dt * dt;

   // IS-GPS-200: Δt_sv = af0 + af1·dt + af2·dt² + Δt_rel − TGD
   //   Δt_rel = F · e · sqrt(A) · sin(Ek),  F = −2·sqrt(mu)/c² = −4.442807633e-10 с/sqrt(м)
   //   Ek — эксцентрическая аномалия на epoch (решение уравнения Кеплера Ek = Mk + e·sin Ek).
   // Раньше член отсутствовал → clkNavL1 завышен до ±13 м (через e·sin Ek), что раздувало clkTerm.
   constexpr double kMu = 3.986005e14;                              // м³/с² (GPS / WGS-84)
   constexpr double kF  = -4.442807633e-10;                         // с/sqrt(м)

   const double A  = navRec.sqrt_A * navRec.sqrt_A;                 // A = (sqrt_A)²
   const double tk = wrapWeek(tow - navRec.Toe);                    // время от опорного момента эфемерид
   const double n  = std::sqrt(kMu / (A * A * A)) + navRec.delta_n; // n = n0 + Δn
   const double Mk = navRec.M0 + n * tk;                            // средняя аномалия

   double Ek = Mk;                                                  // Kepler: Ek = Mk + e·sin Ek

   for (int i = 0; i < 15; ++i) {
      const double prev = Ek;
      Ek = Mk + navRec.e * std::sin(Ek);

      if (std::abs(Ek - prev) < 1e-12) {
         break;
      }
   }

   const double dt_rel = kF * navRec.e * navRec.sqrt_A * std::sin(Ek);

   return clkL1 + dt_rel - navRec.TGD;
}

double navsolver::ErrorCalculator::computeBroadcastClockL1(const Satellite&         sat,
                                                           const rinex::NAV_RECORD& navRec,
                                                           const QDateTime&         epoch) {
   switch (sat.getSystem()) {
     case SatelliteSystem::TYPE::GLONASS:
        return computeBroadcastClockL1_GLO(navRec.clock, epoch);

     case SatelliteSystem::TYPE::GPS:
        return computeBroadcastClockL1_GPS(navRec, epoch);

     default:
        return navRec.clock.svClockBias;
   }
}
