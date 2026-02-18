#include "errorcalculator.h"
#include "modules/IO/datamanager.h"

bool navsolver::ErrorCalculator::execute(pipeline::Context& ctx) {
   if (!ctx.dm || ctx.gridPoints.isEmpty() || ctx.visibleSats.isEmpty()) {
      qWarning() << "[ErrorCalculator] Недостаточно данных для расчета.";
      return false;
   }
   computeResidualErrors(ctx);
   return true;
}

void navsolver::ErrorCalculator::computeResidualErrors(pipeline::Context& ctx) {
   if (!ctx.dm || !ctx.navOrig || !ctx.navCorrected) {
      qWarning() << "[ErrorCalculator] Некорректный Context";
      return;
   }

   const auto* sp3 = ctx.dm->getSP3File();

   if (!sp3 || sp3->records.isEmpty()) {
      qWarning() << "[ErrorCalculator] SP3 отсутствует или пуст.";
      return;
   }

   const auto& sp3Recs     = sp3->records;
   const auto& navOrigRecs = ctx.navOrig->navRecords;
   const auto& navCorrRecs = ctx.navCorrected->navRecords;

   for (auto itEpoch = ctx.visibleSats.cbegin(); itEpoch != ctx.visibleSats.cend(); ++itEpoch) {
      const auto& epoch    = itEpoch.key();
      const auto& pointMap = itEpoch.value();

      // --- Проверяем наличие данных по эпохе один раз ---
      const auto itSp3Epoch  = sp3Recs.constFind(epoch);
      const auto itNavEpoch  = navOrigRecs.constFind(epoch);
      const auto itCorrEpoch = navCorrRecs.constFind(epoch);

      if ((itSp3Epoch == sp3Recs.cend()) ||
          (itNavEpoch == navOrigRecs.cend()) ||
          (itCorrEpoch == navCorrRecs.cend())) {
         continue;
      }

      const auto& sp3BySat  = itSp3Epoch.value();
      const auto& navBySat  = itNavEpoch.value();
      const auto& corrBySat = itCorrEpoch.value();

      for (auto itPoint = pointMap.cbegin(); itPoint != pointMap.cend(); ++itPoint) {
         const auto& point = itPoint.key();
         const auto& sats  = itPoint.value();

         QVector<ResidualError> result;
         result.reserve(sats.size());

         const COORD_XYZ r_obs = point.ecef;

         for (const auto& sat : sats) {
            // --- Поиск по спутнику без value(), без копий и без "дефолтов" ---
            const auto itSp3Sat  = sp3BySat.constFind(sat);
            const auto itNavSat  = navBySat.constFind(sat);
            const auto itCorrSat = corrBySat.constFind(sat);

            if ((itSp3Sat == sp3BySat.cend()) ||
                (itNavSat == navBySat.cend()) ||
                (itCorrSat == corrBySat.cend())) {
               continue;
            }

            const auto& sp3Rec          = itSp3Sat.value();
            const auto& rinexNavRec     = itNavSat.value();
            const auto& rinexCorrNavRec = itCorrSat.value();

            // SP3: ЦМ->ФЦА
            const COORD_XYZ r_sp3_pc = Coordinates::convertMassCenter2PhaseCenter(sp3Rec.coord, sat.getNumber());
            // LOS по SP3 (от наблюдателя к спутнику)
            COORD_XYZ los = {
               r_sp3_pc.x - r_obs.x,
               r_sp3_pc.y - r_obs.y,
               r_sp3_pc.z - r_obs.z
            };
            const double losNorm = std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);

            if (losNorm <= 0.0) {
               continue;
            }
            // единичный вектор линии визирования
            los.x /= losNorm;
            los.y /= losNorm;
            los.z /= losNorm;

            // Проекции координатных разностей на LOS ---
            const COORD_XYZ d_sp3_nav = { r_sp3_pc.x - rinexNavRec.coord.x, r_sp3_pc.y - rinexNavRec.coord.y,
                                          r_sp3_pc.z - rinexNavRec.coord.z };
            const COORD_XYZ d_sp3_cor = { r_sp3_pc.x - rinexCorrNavRec.coord.x, r_sp3_pc.y - rinexCorrNavRec.coord.y,
                                          r_sp3_pc.z - rinexCorrNavRec.coord.z };

            double deltaSp3  = d_sp3_nav.x * los.x + d_sp3_nav.y * los.y + d_sp3_nav.z * los.z;
            double deltaSdcm = d_sp3_cor.x * los.x + d_sp3_cor.y * los.y + d_sp3_cor.z * los.z;

            double clkSp3_L1 = correctSp3ClockL3toL1(ctx, sat, epoch, sp3Rec.clock);

            // Добавляем часовую составляющую в метрах (общая формула из методики)
            double clkNavRaw = computeBroadcastClockL1(sat, rinexNavRec.clock, epoch);     // [sec]
            double clkNavCor = computeBroadcastClockL1(sat, rinexCorrNavRec.clock, epoch); // [sec]

            // Скалярный часовой вклад в метрах:
            deltaSp3  += constants::PZ_9011::c * (clkSp3_L1 - clkNavRaw);
            deltaSdcm += constants::PZ_9011::c * (clkSp3_L1 - clkNavCor);

            // Остаточная ошибка
            const double residual = deltaSp3 - deltaSdcm;

            result.append(ResidualError{
               .satellite = sat,
               .deltaSp3  = deltaSp3,
               .deltaSdcm = deltaSdcm,
               .residual  = residual
            });
         }

         if (!result.isEmpty()) {
            ctx.residualErrors[epoch][point] = std::move(result);
         }
      }
   }
}

double navsolver::ErrorCalculator::correctSp3ClockL3toL1(pipeline::Context& ctx,
                                                         const Satellite&   sat,
                                                         const QDateTime&   epoch,
                                                         double             clockL3) const {
   double sp3Clock_sec = (std::fabs(clockL3) > 1e-3) ? (clockL3 * 1e-6)
                                                          :  clockL3;

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

double navsolver::ErrorCalculator::computeBroadcastClockL1_GLO(const rinex::R_TIME& tr, const QDateTime& epoch) {
   // Выбераем привязку: tk или week_time (зависит от парсинга rinex-nav).
   double tref = (tr.week_time >= 0.0 ? tr.week_time : tr.tk); // [s]
   double tow  = secondsOfWeek(epoch);                         // [s]
   double dt   = wrapWeek(tow - tref);                         // [s]

   // Полином (сек)
   return tr.svClockBias + tr.svClockDrift * dt + 0.5 * tr.svClockDriftRate * dt * dt;
}

double navsolver::ErrorCalculator::computeBroadcastClockL1(const Satellite& sat, const rinex::R_TIME& tr, const QDateTime& epoch) {
   switch (sat.getSystem()) {
     case SatelliteSystem::TYPE::GLONASS:
        return computeBroadcastClockL1_GLO(tr, epoch);
     // case SatelliteSystem::TYPE::GPS:
     // case SatelliteSystem::TYPE::GALILEO:
     //   return computeBroadcastClockL1_GPSGAL(tr, orbitParams, epoch); // когда
     default:
        return tr.svClockBias;
   }
}
