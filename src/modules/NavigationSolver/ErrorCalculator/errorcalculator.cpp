#include "errorcalculator.h"
#include "modules/IO/datamanager.h"

#include <QTimeZone>
#include <cmath>

struct Stats {
   qint64 epochsTotal{ 0 };
   qint64 epochsSkipNoSp3{ 0 };
   qint64 epochsSkipNoNav{ 0 };

   qint64 tripletsTotal{ 0 };
   qint64 tripletsWithVis{ 0 };
   qint64 tripletsStored{ 0 };
   qint64 tripletsEmpty{ 0 };

   qint64 satsVisibleTotal{ 0 };
   qint64 satsOutputTotal{ 0 };

   qint64 satsMissingSp3{ 0 };
   qint64 satsMissingNav{ 0 };
   qint64 satsBadLos{ 0 };

   qint64 satsNoFast{ 0 };
   qint64 satsNoLongTerm{ 0 };
   qint64 satsNoSbasAny{ 0 };
   qint64 satsLtSuppressed{ 0 };

   qint64 satsFastOnly{ 0 };
   qint64 satsLtOnly{ 0 };
   qint64 satsFastLt{ 0 };

   qint64 ltIodInvalid{ 0 };
   qint64 ltNavCandidates{ 0 };
   qint64 ltNavRejected{ 0 };
   qint64 ltNavCompatible{ 0 };
   qint64 ltNavSelected{ 0 };
   qint64 ltNavFallback{ 0 };

   qint64 dcbMissing{ 0 };
   qint64 sp3ScaledKm2m{ 0 };
   qint64 navScaledKm2m{ 0 };

   qint64 residualRawFinite{ 0 };
   qint64 residualFinalFinite{ 0 };
   qint64 masterClkAppliedEpochs{ 0 };
   qint64 masterClkSkippedEpochs{ 0 };
};

static inline double norm3(const COORD_XYZ& p) noexcept {
   return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

static inline COORD_XYZ scaleKmToM_ifNeeded(const COORD_XYZ& p,
                                            bool*            didScale = nullptr) noexcept {
   const double n = norm3(p);

   // Координаты спутника:
   // - в км  -> ||r|| порядка 1e4..1e5
   // - в м   -> ||r|| порядка 1e7..1e8
   const bool kmLike = std::isfinite(n) && (n > 1e3) && (n < 1e6);

   if (didScale) {
      *didScale = kmLike;
   }

   if (kmLike) {
      return COORD_XYZ{ p.x * 1000.0, p.y * 1000.0, p.z * 1000.0 };
   }

   return p;
}

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

static inline bool passesGlonassLtApplicability(const rinex::NAV_RECORD&           navRec,
                                                const io::LongTermCorrectionEntry& lt,
                                                Stats&                             st) {
   ++st.ltNavCandidates;

   const auto w = decodeGlonassLtIod(lt.iod);

   if (!w.valid || !lt.recvTime.isValid()) {
      ++st.ltIodInvalid;
      ++st.ltNavRejected;
      ++st.ltNavFallback;
      ++st.satsLtSuppressed;
      return false;
   }

   const auto tr = glonassTrFromNavRecord(navRec);

   if (!tr.isValid()) {
      ++st.ltNavRejected;
      ++st.ltNavFallback;
      ++st.satsLtSuppressed;
      return false;
   }

   const int  dtSec = static_cast<int> (tr.toUTC().secsTo(lt.recvTime.toUTC()));
   const bool pass  = (dtSec >= w.L_sec) && (dtSec <= (w.L_sec + w.V_sec));

   if (pass) {
      ++st.ltNavCompatible;
      ++st.ltNavSelected;
   } else {
      ++st.ltNavRejected;
      ++st.ltNavFallback;
      ++st.satsLtSuppressed;
   }

   return pass;
}

static inline bool passesGpsLtApplicability(const rinex::NAV_RECORD&           navRec,
                                            const io::LongTermCorrectionEntry& lt,
                                            Stats&                             st) {
   ++st.ltNavCandidates;

   const int iode  = qRound(navRec.IODE);
   const int iodc8 = qRound(navRec.IODC) & 0xFF;

   const bool pass = (lt.iod == iode) && (lt.iod == iodc8);

   if (pass) {
      ++st.ltNavCompatible;
      ++st.ltNavSelected;
   } else {
      ++st.ltNavRejected;
      ++st.ltNavFallback;
      ++st.satsLtSuppressed;
   }

   return pass;
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

   const auto* rinexNav = ctx.dm->getRinexFile();

   if (!rinexNav || rinexNav->navRecords.isEmpty()) {
      qWarning() << "[ErrorCalculator] NAV отсутствует или не загружен";
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
   const auto& navRecs = rinexNav->navRecords;
   const auto& sbas    = ctx.dm->getSBASStore();

   struct TempResidual {
      Satellite sat;
      double    deltaSp3{ qQNaN() };    // ΔSP3corr_k
      double    deltaSdcm{ qQNaN() };   // ΔSDCMcorr_k
      double    residualRaw{ qQNaN() }; // ΔSDCMerr_k
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

      const auto itNavEpoch = navRecs.constFind(epoch);

      if (itNavEpoch == navRecs.cend()) {
         ++st.epochsSkipNoNav;
         continue;
      }

      const auto& sp3BySat = itSp3Epoch.value();
      const auto& navBySat = itNavEpoch.value();

      for (auto itPoint = pointMap.cbegin(); itPoint != pointMap.cend(); ++itPoint) {
         ++st.tripletsTotal;

         const auto& point = itPoint.key();
         const auto& sats  = itPoint.value();

         if (sats.isEmpty()) {
            continue;
         }

         ++st.tripletsWithVis;
         st.satsVisibleTotal += sats.size();

         QVector<TempResidual> temp;
         temp.reserve(sats.size());

         const COORD_XYZ r_obs = point.ecef;

         for (const auto& sat : sats) {
            const auto itSp3Sat = sp3BySat.constFind(sat);

            if (itSp3Sat == sp3BySat.cend()) {
               ++st.satsMissingSp3;
               continue;
            }

            const auto itNavSat = navBySat.constFind(sat);

            if (itNavSat == navBySat.cend()) {
               ++st.satsMissingNav;
               continue;
            }

            const auto& sp3Rec       = itSp3Sat.value();
            rinex::NAV_RECORD navRec = itNavSat.value();

            auto prc = sbas.fastPrc_m(sat, epoch);
            auto lt  = sbas.getLongTermCorrection(sat, epoch);

            if (lt) {
               if (sat.getSystem() == SatelliteSystem::TYPE::GLONASS) {
                  if (!passesGlonassLtApplicability(navRec, *lt, st)) {
                     lt.reset();
                  }
               } else if (sat.getSystem() == SatelliteSystem::TYPE::GPS) {
                  if (!passesGpsLtApplicability(navRec, *lt, st)) {
                     lt.reset();
                  }
               }
            }

            bool sp3Scaled           = false;
            const COORD_XYZ sp3_cm_m = scaleKmToM_ifNeeded(sp3Rec.coord, &sp3Scaled);

            if (sp3Scaled) {
               ++st.sp3ScaledKm2m;
            }

            const COORD_XYZ r_sp3_pc =
               Coordinates::convertMassCenter2PhaseCenter(sp3_cm_m, sat.getNumber());

            bool navScaled        = false;
            const COORD_XYZ nav_m = scaleKmToM_ifNeeded(navRec.coord, &navScaled);

            if (navScaled) {
               ++st.navScaledKm2m;
            }

            COORD_XYZ los = {
               r_sp3_pc.x - r_obs.x,
               r_sp3_pc.y - r_obs.y,
               r_sp3_pc.z - r_obs.z
            };

            const double losNorm = std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);

            if (!(losNorm > 0.0) || !std::isfinite(losNorm)) {
               ++st.satsBadLos;
               continue;
            }

            los.x /= losNorm;
            los.y /= losNorm;
            los.z /= losNorm;

            const COORD_XYZ d_sp3_nav = {
               r_sp3_pc.x - nav_m.x,
               r_sp3_pc.y - nav_m.y,
               r_sp3_pc.z - nav_m.z
            };

            double deltaSp3_m =
               d_sp3_nav.x * los.x +
               d_sp3_nav.y * los.y +
               d_sp3_nav.z * los.z;

            const double clkSp3L1 = correctSp3ClockL3toL1(ctx, sat, epoch, sp3Rec.clock, &st.dcbMissing);
            const double clkNavL1 = computeBroadcastClockL1(sat, navRec, epoch);

            // ####
            if (sat.getSystem() == SatelliteSystem::TYPE::GPS) {
               const double toc = secondsOfWeek(navRec.clock.epoch);
               const double tow = secondsOfWeek(epoch);
               const double dt  = wrapWeek(tow - toc);

               qDebug().noquote()
                  << QString("[EC][GPS_CLK] epoch=%1 sat=G%2 toc=%3 tow=%4 dt=%5 "
                             "af0=%6 af1=%7 af2=%8 TGD=%9 clkNav=%10")
                  .arg(epoch.toString(Qt::ISODate))
                  .arg(sat.getNumber(),               2, 10,  QChar('0'))
                  .arg(toc,                           0, 'f', 3)
                  .arg(tow,                           0, 'f', 3)
                  .arg(dt,                            0, 'f', 3)
                  .arg(navRec.clock.svClockBias,      0, 'e', 12)
                  .arg(navRec.clock.svClockDrift,     0, 'e', 12)
                  .arg(navRec.clock.svClockDriftRate, 0, 'e', 12)
                  .arg(navRec.TGD,                    0, 'e', 12)
                  .arg(clkNavL1,                      0, 'e', 12);
            }

            if ((sat.getSystem() == SatelliteSystem::TYPE::GPS) && lt) {
               const int iode  = qRound(navRec.IODE);
               const int iodc8 = qRound(navRec.IODC) & 0xFF;

               qDebug().noquote()
                  << QString("[EC][GPS_LT_CHECK] epoch=%1 sat=G%2 ltIod=%3 iode=%4 iodc8=%5")
                  .arg(epoch.toString(Qt::ISODate))
                  .arg(sat.getNumber(), 2, 10, QChar('0'))
                  .arg(lt->iod)
                  .arg(iode)
                  .arg(iodc8);
            }
            // ###

            deltaSp3_m -= constants::PZ_9011::c * (clkSp3L1 - clkNavL1);

            const bool   hasFast    = prc.has_value();
            const bool   hasLt      = lt.has_value();
            const bool   hasAnySbas = hasFast || hasLt;
            const double prc_m      = hasFast ? *prc : 0.0;

            if (!hasFast) {
               ++st.satsNoFast;
            }

            if (!hasLt) {
               ++st.satsNoLongTerm;
            }

            if (hasFast && hasLt) {
               ++st.satsFastLt;
            } else if (hasFast) {
               ++st.satsFastOnly;
            } else if (hasLt) {
               ++st.satsLtOnly;
            } else {
               ++st.satsNoSbasAny;
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
            ++st.tripletsEmpty;
            continue;
         }

         // Шаг 8 методики: master-clock correction по GPS.
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
            ++st.masterClkAppliedEpochs;
         } else {
            ++st.masterClkSkippedEpochs;
         }

         QVector<ResidualError> result;
         result.reserve(temp.size());

         if (gpsResidualCount > 0) {
            qDebug().noquote()
               << QString("[EC][MASTER_CLK] epoch=%1 deltaClk_m=%2 gpsCount=%3")
               .arg(epoch.toString(Qt::ISODate))
               .arg(deltaClk_m, 0, 'f', 4)
               .arg(gpsResidualCount);
         }

         for (const auto& tr : temp) {
            if (qIsFinite(tr.residualRaw)) {
               ++st.residualRawFinite;
            }

            double residualFinal_m = tr.residualRaw;

            if (qIsFinite(deltaClk_m) && qIsFinite(residualFinal_m)) {
               residualFinal_m -= deltaClk_m;
            }

            if (qIsFinite(residualFinal_m)) {
               ++st.residualFinalFinite;
            }

            if ((tr.sat.getSystem() == SatelliteSystem::TYPE::GLONASS) &&
                ((tr.sat.getNumber() == 7) || (tr.sat.getNumber() == 13) ||
                 (tr.sat.getNumber() == 14) || (tr.sat.getNumber() == 17))) {
               qDebug().noquote()
                  << QString("[EC][FINAL] epoch=%1 sat=R%2 "
                             "residualRaw=%3 deltaClk=%4 residualFinal=%5")
                  .arg(epoch.toString(Qt::ISODate))
                  .arg(tr.sat.getNumber(), 2, 10,  QChar('0'))
                  .arg(tr.residualRaw,     0, 'f', 4)
                  .arg(deltaClk_m,         0, 'f', 4)
                  .arg(residualFinal_m,    0, 'f', 4);
            }

            result.append(ResidualError{
               .satellite = tr.sat,
               .deltaSp3  = tr.deltaSp3,
               .deltaSdcm = tr.deltaSdcm,
               .residual  = residualFinal_m
            });

            ++st.satsOutputTotal;
         }

         if (!result.isEmpty()) {
            ctx.residualErrors[epoch][point] = std::move(result);
            ++st.tripletsStored;
         } else {
            ++st.tripletsEmpty;
         }
      }
   }

   qInfo().noquote()
      << QString("[ErrorCalculator][STATS] "
                 "epochs=%1 (skip: noSp3=%2, noNav=%3); "
                 "triplets=%4 (withVisible=%5, stored=%6, empty=%7); "
                 "satVisible=%8 satResidual=%9; "
                 "satSkip: noSp3=%10 noNav=%11 badLOS=%12; "
                 "sbasCoverage: fastOnly=%13 ltOnly=%14 fastLt=%15 noSbas=%16; "
                 "ltNav: cand=%17 rej=%18 ok=%19 sel=%20 fallback=%21; "
                 "dcbMissing=%22; scaled: sp3Km2m=%23 navKm2m=%24")
      .arg(st.epochsTotal)
      .arg(st.epochsSkipNoSp3)
      .arg(st.epochsSkipNoNav)
      .arg(st.tripletsTotal)
      .arg(st.tripletsWithVis)
      .arg(st.tripletsStored)
      .arg(st.tripletsEmpty)
      .arg(st.satsVisibleTotal)
      .arg(st.satsOutputTotal)
      .arg(st.satsMissingSp3)
      .arg(st.satsMissingNav)
      .arg(st.satsBadLos)
      .arg(st.satsFastOnly)
      .arg(st.satsLtOnly)
      .arg(st.satsFastLt)
      .arg(st.satsNoSbasAny)
      .arg(st.ltNavCandidates)
      .arg(st.ltNavRejected)
      .arg(st.ltNavCompatible)
      .arg(st.ltNavSelected)
      .arg(st.ltNavFallback)
      .arg(st.dcbMissing)
      .arg(st.sp3ScaledKm2m)
      .arg(st.navScaledKm2m);

   qInfo().noquote()
      << QString("[ErrorCalculator][OUTPUT] "
                 "residualRawFinite=%1 residualFinalFinite=%2 "
                 "masterClkAppliedEpochs=%3 masterClkSkippedEpochs=%4")
      .arg(st.residualRawFinite)
      .arg(st.residualFinalFinite)
      .arg(st.masterClkAppliedEpochs)
      .arg(st.masterClkSkippedEpochs);
}

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
      } else {
         if (dcbMissingCounter) {
            const qint64 before = *dcbMissingCounter;
            ++(*dcbMissingCounter);

            if (before < 10) {
               qWarning() << "[ErrorCalculator] Нет DCB для" << sat.toString()
                          << "в эпоху" << epoch
                          << "(далее предупреждения будут подавлены; счётчик продолжит расти)";
            }
         } else {
            qWarning() << "[ErrorCalculator] Нет DCB для" << sat.toString()
                       << "в эпоху" << epoch;
         }
      }
   }

   return sp3ClockSec;
}

double navsolver::ErrorCalculator::computeBroadcastClockL1_GLO(const rinex::R_TIME& tr,
                                                               const QDateTime&     epoch) {
   const double tref = (tr.week_time >= 0.0) ? tr.week_time : tr.tk;
   const double tow  = secondsOfWeek(epoch);
   const double dt   = wrapWeek(tow - tref);

   return tr.svClockBias +
          tr.svClockDrift * dt +
          0.5 * tr.svClockDriftRate * dt * dt;
}

double navsolver::ErrorCalculator::computeBroadcastClockL1_GPS(const rinex::NAV_RECORD& navRec, const QDateTime& epoch) {
   const auto& tr = navRec.clock;

   const double toc = secondsOfWeek(tr.epoch);
   const double tow = secondsOfWeek(epoch);
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
