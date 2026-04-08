#include "errorcalculator.h"
#include "modules/IO/datamanager.h"

#include <QTimeZone>
#include <cmath>

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

   // SBAS coverage
   qint64 satsFastLt{ 0 };   // Fast + LT (лучшее качество)
   qint64 satsFastOnly{ 0 }; // Только Fast
   qint64 satsLtOnly{ 0 };   // Только LT
   qint64 satsNoSbas{ 0 };   // Без SBAS

   // LT matching по системам
   qint64 gpsLtMatched{ 0 };
   qint64 gpsLtMissed{ 0 };
   qint64 gloLtMatched{ 0 };
   qint64 gloLtMissed{ 0 };

   // Master clock
   qint64 masterClkApplied{ 0 };
   qint64 masterClkSkipped{ 0 };

   // Residuals
   qint64 residualsFinite{ 0 };
};

// ============================================================================
// Вспомогательные функции
// ============================================================================

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
// ГЛОНАСС LT applicability (временной gate)
// ============================================================================

static inline QDateTime glonassTrFromNavRecord(const rinex::NAV_RECORD& rec) {
   if (!rec.clock.epoch.isValid()) {
      return {};
   }

   int sod = -1;

   if (rec.clock.tk >= 0.0) {
      sod = static_cast<int> (std::llround(rec.clock.tk));
   } else if (rec.clock.week_time >= 0.0) {
      sod = static_cast<int> (std::llround(rec.clock.week_time)) % 86400;
   } else {
      return {};
   }

   sod %= 86400;

   if (sod < 0) {
      sod += 86400;
   }

   return QDateTime(rec.clock.epoch.date(),
                    QTime(0, 0),
                    rec.clock.epoch.timeRepresentation()).addSecs(sod);
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

   const int dtSec = static_cast<int> (tr.toUTC().secsTo(lt.recvTime.toUTC()));

   return (dtSec >= w.L_sec) && (dtSec <= (w.L_sec + w.V_sec));
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
static inline std::optional<rinex::NAV_RECORD> findGpsNavByIode(
   const QMap<QDateTime, QMap<Satellite, rinex::NAV_RECORD> >& navRecs,
   const Satellite&                                            sat,
   int                                                         targetIode,
   const QDateTime&                                            epoch,
   int                                                         maxAgeSec = 14400) {
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
      double    deltaSp3{ qQNaN() };
      double    deltaSdcm{ qQNaN() };
      double    residualRaw{ qQNaN() };
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
                  if (passesGlonassLtApplicability(navRec, *lt)) {
                     ++st.gloLtMatched;
                  } else {
                     ++st.gloLtMissed;
                     lt.reset();
                  }
               }
            }

            // ========== Координаты SP3 (phase center) ==========
            const COORD_XYZ sp3_cm_m = scaleKmToM_ifNeeded(sp3Rec.coord);
            const COORD_XYZ r_sp3_pc =
               Coordinates::convertMassCenter2PhaseCenter(sp3_cm_m, sat.getNumber());

            // ========== Координаты NAV (broadcast) ==========
            COORD_XYZ navCoordKm = navRec.coord;

            if (sat.getSystem() == SatelliteSystem::TYPE::GPS) {
               const double gpsTowOrbit = secondsOfWeekWall(epoch);
               navCoordKm = evaluateGpsBroadcastCoordKmAtTow(navRec, gpsTowOrbit);
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
            const bool   hasFast    = prc.has_value();
            const bool   hasLt      = lt.has_value();
            const bool   hasAnySbas = hasFast || hasLt;
            const double prc_m      = hasFast ? *prc : 0.0;

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
                     rrc_m = rrc * dt_apply;
                  }
               }
            }

            // ========== Residual формула ==========
            const double deltaSdcm_m =
               hasAnySbas ? ((hasLt ? (dr_los_m - dt_lt_m) : 0.0) - prc_m - rrc_m)
                               : qQNaN();

            const double residualRaw_m =
               hasAnySbas ? (deltaSp3_m - deltaSdcm_m) : qQNaN();

            temp.push_back(TempResidual{
               .sat         = sat,
               .deltaSp3    = deltaSp3_m,
               .deltaSdcm   = deltaSdcm_m,
               .residualRaw = residualRaw_m
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

            if (qIsFinite(residualFinal_m)) {
               ++st.residualsFinite;
            }

            // Лог residualFinal для ГЛОНАСС (первая точка в эпохе, первые 5 спутников)
            if ((tr.sat.getSystem() == SatelliteSystem::TYPE::GLONASS) &&
                qIsFinite(residualFinal_m) &&
                (itPoint == pointMap.cbegin())) {
               qDebug().noquote()
                  << QString("[EC][GLO_RES] epoch=%1 sat=R%2 deltaSp3=%3 deltaSdcm=%4 residualFinal=%5")
                  .arg(epoch.toString(Qt::ISODate))
                  .arg(tr.sat.getNumber(), 2, 10,  QChar('0'))
                  .arg(tr.deltaSp3,        0, 'f', 4)
                  .arg(tr.deltaSdcm,       0, 'f', 4)
                  .arg(residualFinal_m,    0, 'f', 4);
            }

            result.append(ResidualError{
               .satellite = tr.sat,
               .deltaSp3  = tr.deltaSp3,
               .deltaSdcm = tr.deltaSdcm,
               .residual  = residualFinal_m
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
      << QString("[ErrorCalculator][STATS] epochs=%1 (skipSp3=%2 skipNav=%3) | "
                 "sats: visible=%4 processed=%5 missingNav=%6 | "
                 "sbas: fast+lt=%7 fastOnly=%8 ltOnly=%9 noSbas=%10 | "
                 "lt: gpsOk=%11 gpsMiss=%12 gloOk=%13 gloMiss=%14 | "
                 "masterClk: applied=%15 skipped=%16 | "
                 "residualsFinite=%17 dcbMissing=%18")
      .arg(st.epochsTotal)
      .arg(st.epochsSkipNoSp3)
      .arg(st.epochsSkipNoNav)
      .arg(st.satsVisible)
      .arg(st.satsProcessed)
      .arg(st.satsMissingNav)
      .arg(st.satsFastLt)
      .arg(st.satsFastOnly)
      .arg(st.satsLtOnly)
      .arg(st.satsNoSbas)
      .arg(st.gpsLtMatched)
      .arg(st.gpsLtMissed)
      .arg(st.gloLtMatched)
      .arg(st.gloLtMissed)
      .arg(st.masterClkApplied)
      .arg(st.masterClkSkipped)
      .arg(st.residualsFinite)
      .arg(dcbMissing);
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
      tr.svClockDrift * dt +
      tr.svClockDriftRate * dt * dt;

   return clkL1 - navRec.TGD;
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
