#include "correctionapplier.h"

corrections::CorrectionApplier::CorrectionApplier(const io::SBASCorrectionStore& correctionStore) :
   correctionStore_(correctionStore) {}

void corrections::CorrectionApplier::applySBASCorrections(rinex::RINEX_FILE& rinexNavFiles) {
   for (auto itEpoch = rinexNavFiles.navRecords.begin(); itEpoch != rinexNavFiles.navRecords.end(); ++itEpoch) {
      const QDateTime& epoch = itEpoch.key();
      auto& satMap           = itEpoch.value();

      for (auto itSat = satMap.begin(); itSat != satMap.end();) {
         if (!applyCorrectionToNavRecord(itSat.value(), itSat.key(), epoch)) {
            qDebug() << "[CorrectionApplier] Removed satellite" << itSat.key().toString()
                     << "at epoch" << epoch.toString(Qt::ISODate) << "(no applicable correction)";
            itSat = satMap.erase(itSat);
         } else {
            ++itSat;
         }
      }
   }
}

bool corrections::CorrectionApplier::applyCorrectionToNavRecord(rinex::NAV_RECORD& navRecord, const Satellite& sat,
                                                                const QDateTime& epoch) {
   auto optCorr = correctionStore_.getLongTermCorrection(sat, epoch);

   if (!optCorr.has_value()) {
      return false;
   }

   const auto& corr = *optCorr;
   int deltaSec     = QTime(0, 0).secsTo(epoch.time()) - corr.t0;
   COORD_XYZ deltaR = corr.deltaPos;

   if (corr.hasVelocity) {
      deltaR.x += corr.deltaVel.x * deltaSec;
      deltaR.y += corr.deltaVel.y * deltaSec;
      deltaR.z += corr.deltaVel.z * deltaSec;
   }
   navRecord.coord.x += deltaR.x;
   navRecord.coord.y += deltaR.y;
   navRecord.coord.z += deltaR.z;

   const bool isGlonass = (sat.getSystem() == SatelliteSystem::TYPE::GLONASS);
   double     clockCorr = computeClockCorrection(corr, epoch, isGlonass);
   navRecord.clock.svClockBias += clockCorr;

   return true;
}

double corrections::CorrectionApplier::computeClockCorrection(const io::LongTermCorrectionEntry& correction,
                                                              const QDateTime&                   epoch,
                                                              bool                               isGlonass) {
   int t_epoch       = QTime(0, 0).secsTo(epoch.time());
   double deltaT_sec = t_epoch - correction.t0;

   if (isGlonass) {
      auto gpsOffset = correctionStore_.getGpsMinusGlonassOffset(epoch);

      if (gpsOffset.has_value()) {
         qDebug() << "[SBAS] Δt_GPS−GLONASS =" << gpsOffset.value();
      } else {
         qDebug() << "[SBAS] Нет действующего смещения GPS−GLONASS на" << epoch;
      }

      if (gpsOffset.has_value()) {
         deltaT_sec += (gpsOffset.value());
      }
   }

   return correction.deltaAf0 + correction.deltaAf1 * deltaT_sec;
}
