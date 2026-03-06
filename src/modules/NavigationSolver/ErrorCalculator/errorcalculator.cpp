#include "errorcalculator.h"
#include "modules/IO/datamanager.h"

#include <cmath>

static inline double norm3(const COORD_XYZ& p) noexcept {
   return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

static inline COORD_XYZ scaleKmToM_ifNeeded(const COORD_XYZ& p, bool* didScale = nullptr) noexcept {
   const double n = norm3(p);

   // Если координаты в км, то ||r|| порядка 1e4..1e5.
   // Если в м, то ||r|| порядка 1e7..1e8.
   const bool kmLike = std::isfinite(n) && (n > 1e3) && (n < 1e6);

   if (didScale) {
      *didScale = kmLike;
   }

   if (kmLike) {
      return COORD_XYZ{ p.x * 1000.0, p.y * 1000.0, p.z * 1000.0 };
   }
   return p;
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

   // NAV (RINEX)
   const auto* rinexNav = ctx.dm->getRinexFile();

   if (!rinexNav || rinexNav->navRecords.isEmpty()) {
      qWarning() << "[ErrorCalculator] NAV отсутствует или не загружен";
      return;
   }

   // SP3
   const auto* sp3 = ctx.dm->getSP3File();

   if (!sp3 || sp3->records.isEmpty()) {
      qWarning() << "[ErrorCalculator] SP3 отсутствует или пуст";
      return;
   }

   // Важно: пересчёт должен быть "с нуля"
   ctx.residualErrors.clear();

   struct Stats {
      qint64 epochsTotal     = 0;
      qint64 epochsSkipNoSp3 = 0;
      qint64 epochsSkipNoNav = 0;

      qint64 tripletsTotal   = 0; // (epoch, point)
      qint64 tripletsWithVis = 0;
      qint64 tripletsStored  = 0;
      qint64 tripletsEmpty   = 0;

      qint64 satsVisibleTotal = 0;
      qint64 satsResidualMade = 0;

      qint64 satsMissingSp3 = 0;
      qint64 satsMissingNav = 0;
      qint64 satsBadLos     = 0;

      qint64 satsNoFast     = 0;
      qint64 satsNoLongTerm = 0;
      qint64 satsNoSbasAny  = 0;

      qint64 dcbMissing = 0;

      qint64 sp3ScaledKm2m = 0;
      qint64 navScaledKm2m = 0;
   } st;

   const auto& sp3Recs     = sp3->records;
   const auto& navOrigRecs = rinexNav->navRecords;
   const auto& store       = ctx.dm->getSBASStore();

   for (auto itEpoch = ctx.visibleSats.cbegin(); itEpoch != ctx.visibleSats.cend(); ++itEpoch) {
      ++st.epochsTotal;

      const QDateTime& epoch = itEpoch.key();
      const auto& pointMap   = itEpoch.value();

      const auto itSp3Epoch = sp3Recs.constFind(epoch);

      if (itSp3Epoch == sp3Recs.cend()) {
         ++st.epochsSkipNoSp3;
         continue;
      }

      const auto itNavEpoch = navOrigRecs.constFind(epoch);

      if (itNavEpoch == navOrigRecs.cend()) {
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

         QVector<ResidualError> result;
         result.reserve(sats.size());

         const COORD_XYZ r_obs = point.ecef;

         for (const auto& sat : sats) {
            // --- SP3 запись ---
            const auto itSp3Sat = sp3BySat.constFind(sat);

            if (itSp3Sat == sp3BySat.cend()) {
               ++st.satsMissingSp3;
               continue;
            }

            // --- NAV запись ---
            const auto itNavSat = navBySat.constFind(sat);

            if (itNavSat == navBySat.cend()) {
               ++st.satsMissingNav;
               continue;
            }

            const auto& sp3Rec      = itSp3Sat.value();
            const auto& rinexNavRec = itNavSat.value();

            // --- SP3: км->м (если нужно) + ЦМ->ФЦА ---
            bool sp3Scaled           = false;
            const COORD_XYZ sp3_cm_m = scaleKmToM_ifNeeded(sp3Rec.coord, &sp3Scaled);

            if (sp3Scaled) {
               ++st.sp3ScaledKm2m;
            }
            const COORD_XYZ r_sp3_pc = Coordinates::convertMassCenter2PhaseCenter(sp3_cm_m, sat.getNumber());

            // --- NAV: км->м (если нужно) ---
            bool navScaled        = false;
            const COORD_XYZ nav_m = scaleKmToM_ifNeeded(rinexNavRec.coord, &navScaled);

            if (navScaled) {
               ++st.navScaledKm2m;
            }

            // --- LOS по SP3 (от наблюдателя к спутнику) ---
            COORD_XYZ los        = { r_sp3_pc.x - r_obs.x, r_sp3_pc.y - r_obs.y, r_sp3_pc.z - r_obs.z };
            const double losNorm = std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);

            if (!(losNorm > 0.0) || !std::isfinite(losNorm)) {
               ++st.satsBadLos;
               continue;
            }
            los.x /= losNorm; los.y /= losNorm; los.z /= losNorm;

            // ==========================================================
            // 1) ABS: Δρ_abs = (SP3 - NAV)·LOS + c( dt_sp3 - dt_nav )
            // ==========================================================
            const COORD_XYZ d_sp3_nav = { r_sp3_pc.x - nav_m.x,
                                          r_sp3_pc.y - nav_m.y,
                                          r_sp3_pc.z - nav_m.z };

            double deltaSp3        = d_sp3_nav.x * los.x + d_sp3_nav.y * los.y + d_sp3_nav.z * los.z;
            const double clkSp3_L1 = correctSp3ClockL3toL1(ctx, sat, epoch, sp3Rec.clock, &st.dcbMissing); // [sec]
            const double clkNavRaw = computeBroadcastClockL1(sat, rinexNavRec.clock, epoch);               // [sec]
            deltaSp3 += constants::PZ_9011::c * (clkSp3_L1 - clkNavRaw);
            // ==========================================================
            // 2) SBAS correction in range:
            //    corr_sbas = PRC + (Δr_LT·LOS) + c·Δt_LT
            // ==========================================================
            bool   hasFast    = false;
            bool   hasLt      = false;
            double corrSbas_m = 0.0;

            if (auto prc = store.fastPrc_m(sat, epoch)) {
               corrSbas_m += *prc; // PRC в метрах
               hasFast     = true;
            } else {
               ++st.satsNoFast;
            }

            if (auto lt = store.getLongTermCorrection(sat, epoch)) {
               hasLt = true;

               // LT position part (м)
               const int deltaSec = QTime(0, 0).secsTo(epoch.time()) - lt->t0;

               COORD_XYZ deltaR = lt->deltaPos;

               if (lt->hasVelocity) {
                  deltaR.x += lt->deltaVel.x * deltaSec;
                  deltaR.y += lt->deltaVel.y * deltaSec;
                  deltaR.z += lt->deltaVel.z * deltaSec;
               }
               corrSbas_m += (deltaR.x * los.x + deltaR.y * los.y + deltaR.z * los.z);

               // LT clock part (сек -> м)
               double t_epoch_s  = double(QTime(0, 0).secsTo(epoch.time()));
               double deltaT_sec = t_epoch_s - double(lt->t0);

               if (sat.getSystem() == SatelliteSystem::TYPE::GLONASS) {
                  if (auto gpsOffset = store.gpsGlonassOffset(epoch)) {
                     deltaT_sec += *gpsOffset;
                  }
               }

               const double clockCorr_s = lt->deltaAf0 + lt->deltaAf1 * deltaT_sec;
               corrSbas_m += constants::PZ_9011::c * clockCorr_s;
            } else {
               ++st.satsNoLongTerm;
            }

            double sbasCorrection_m = 0.0;
            double residualSdcm_m   = qQNaN();

            if (hasFast || hasLt) {
               sbasCorrection_m = corrSbas_m;
               residualSdcm_m   = deltaSp3 - sbasCorrection_m;
            } else {
               ++st.satsNoSbasAny;
            }

            result.append(ResidualError{
               .satellite = sat,
               .deltaSp3  = deltaSp3,         // Истинная ошибка
               .deltaSdcm = sbasCorrection_m, // Поправка СДКМ
               .residual  = residualSdcm_m    // Невязка после СДКМ
            });

            ++st.satsResidualMade;
         }

         if (!result.isEmpty()) {
            ctx.residualErrors[epoch][point] = std::move(result);
            ++st.tripletsStored;
         } else {
            ++st.tripletsEmpty;
         }
      }
   }

   const double avgVis = (st.tripletsWithVis > 0)
                              ? (double(st.satsVisibleTotal) / double(st.tripletsWithVis))
                              : 0.0;
   const double avgRes = (st.tripletsWithVis > 0)
                              ? (double(st.satsResidualMade) / double(st.tripletsWithVis))
                              : 0.0;

   qInfo().noquote()
      << QString("[ErrorCalculator][STATS] "
                 "epochs=%1 (skip: noSp3=%2 noNav=%3); "
                 "triplets=%4 (withVisible=%5 stored=%6 empty=%7); "
                 "satVisible=%8 satResidual=%9 (avgVis=%10 avgRes=%11); "
                 "satSkip: noSp3=%12 noNav=%13 badLOS=%14; "
                 "sbasMissing: noFAST=%15 noLT=%16 noAny=%17; "
                 "dcbMissing=%18; scaled: sp3Km2m=%19 navKm2m=%20")
      .arg(st.epochsTotal)
      .arg(st.epochsSkipNoSp3)
      .arg(st.epochsSkipNoNav)
      .arg(st.tripletsTotal)
      .arg(st.tripletsWithVis)
      .arg(st.tripletsStored)
      .arg(st.tripletsEmpty)
      .arg(st.satsVisibleTotal)
      .arg(st.satsResidualMade)
      .arg(avgVis, 0, 'f', 2)
      .arg(avgRes, 0, 'f', 2)
      .arg(st.satsMissingSp3)
      .arg(st.satsMissingNav)
      .arg(st.satsBadLos)
      .arg(st.satsNoFast)
      .arg(st.satsNoLongTerm)
      .arg(st.satsNoSbasAny)
      .arg(st.dcbMissing)
      .arg(st.sp3ScaledKm2m)
      .arg(st.navScaledKm2m);
}

double navsolver::ErrorCalculator::correctSp3ClockL3toL1(pipeline::Context& ctx,
                                                         const Satellite&   sat,
                                                         const QDateTime&   epoch,
                                                         double             clockL3) const {
   // В зависимости от парсера SP3 clock может быть уже в секундах или в микросекундах.
   // Эвристика: если по модулю больше 1e-3, считаем что это микросекунды (типично для SP3).
   double sp3Clock_sec = (std::fabs(clockL3) > 1e-3) ? (clockL3 * 1e-6) : clockL3;

   if (sat.getSystem() == SatelliteSystem::TYPE::GLONASS) {
      auto dcb_ns = ctx.dm->getGlonassL3MinusL1Bias(sat);

      if (dcb_ns.has_value()) {
         sp3Clock_sec += dcb_ns.value() * 1e-9; // нс → сек
      } else {
         qWarning() << "[ErrorCalculator] Нет DCB для" << sat.toString() << "в эпоху" << epoch;
      }
   }
   return sp3Clock_sec;
}

double navsolver::ErrorCalculator::correctSp3ClockL3toL1(pipeline::Context& ctx,
                                                         const Satellite&   sat,
                                                         const QDateTime&   epoch,
                                                         double             clockL3,
                                                         qint64*            dcbMissingCounter) const {
   double sp3Clock_sec = (std::fabs(clockL3) > 1e-3) ? (clockL3 * 1e-6) : clockL3;

   if (sat.getSystem() == SatelliteSystem::TYPE::GLONASS) {
      auto dcb_ns = ctx.dm->getGlonassL3MinusL1Bias(sat);

      if (dcb_ns.has_value()) {
         sp3Clock_sec += dcb_ns.value() * 1e-9; // нс → сек
      } else {
         // считаем пропуски, но не спамим лог (предупредим только первые 10 раз)
         if (dcbMissingCounter) {
            const qint64 before = *dcbMissingCounter;
            ++(*dcbMissingCounter);

            if (before < 10) {
               qWarning() << "[ErrorCalculator] Нет DCB для" << sat.toString() << "в эпоху" << epoch
                          << "(далее предупреждения будут подавлены; счётчик продолжит расти)";
            }
         } else {
            qWarning() << "[ErrorCalculator] Нет DCB для" << sat.toString() << "в эпоху" << epoch;
         }
      }
   }
   return sp3Clock_sec;
}

double navsolver::ErrorCalculator::computeBroadcastClockL1_GLO(const rinex::R_TIME& tr, const QDateTime& epoch) {
   // Выбор опорного времени: tk или week_time (как в вашем коде)
   const double tref = (tr.week_time >= 0.0 ? tr.week_time : tr.tk); // [s]
   const double tow  = secondsOfWeek(epoch);                         // [s]
   const double dt   = wrapWeek(tow - tref);                         // [s]

   // Полином (сек)
   return tr.svClockBias + tr.svClockDrift * dt + 0.5 * tr.svClockDriftRate * dt * dt;
}

double navsolver::ErrorCalculator::computeBroadcastClockL1(const Satellite& sat, const rinex::R_TIME& tr, const QDateTime& epoch) {
   switch (sat.getSystem()) {
     case SatelliteSystem::TYPE::GLONASS:
        return computeBroadcastClockL1_GLO(tr, epoch);
     default:
        return tr.svClockBias;
   }
}
