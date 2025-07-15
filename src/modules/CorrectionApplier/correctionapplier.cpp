#include "correctionapplier.h"
#include "modules/IO/datamanager.h"

bool corrections::CorrectionApplier::execute(pipeline::Context& ctx) {
   auto src = ctx.dm->getRinexFile();

   if (!src) {
      qWarning() << "[" << name() << "] NAV-файл не загружен";
      return false;
   }
   ctx.navOrig      = src;
   ctx.navCorrected = std::make_unique<rinex::RINEX_FILE> (*src);
   applySBASCorrections(*ctx.navCorrected, ctx.dm->getSBASStore());

   return true;
}

void corrections::CorrectionApplier::applySBASCorrections(rinex::RINEX_FILE& rinexNavFiles, const io::SBASCorrectionStore& store) {
   for (auto itEpoch = rinexNavFiles.navRecords.begin(); itEpoch != rinexNavFiles.navRecords.end(); ++itEpoch) {
      const QDateTime& epoch = itEpoch.key();
      auto& satMap           = itEpoch.value();

      for (auto itSat = satMap.begin(); itSat != satMap.end();) {
         if (!applyCorrectionToNavRecord(itSat.value(), itSat.key(), epoch, store)) {
            qDebug() << "[" << name() << "] Удаляем"
                     << itSat.key().toString()
                     << "на" << epoch.toString(Qt::ISODate)
                     << "(нет поправок)";
            itSat = satMap.erase(itSat);
         } else {
            ++itSat;
         }
      }
   }
}

bool corrections::CorrectionApplier::applyCorrectionToNavRecord(rinex::NAV_RECORD& navRecord, const Satellite& sat,
                                                                const QDateTime& epoch, const io::SBASCorrectionStore& store) {
   auto optCorr = store.getLongTermCorrection(sat, epoch);

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
   double     clockCorr = computeClockCorrection(corr, epoch, store, isGlonass);
   navRecord.clock.svClockBias += clockCorr;

   return true;
}

double corrections::CorrectionApplier::computeClockCorrection(const io::LongTermCorrectionEntry& correction,
                                                              const QDateTime& epoch, const io::SBASCorrectionStore& store,
                                                              bool                               isGlonass) {
   double t_epoch    = QTime(0, 0).secsTo(epoch.time());
   double deltaT_sec = t_epoch - static_cast<double> (correction.t0);

   if (isGlonass) {
      auto gpsOffset = store.gpsGlonassOffset(epoch);

      if (gpsOffset) {
         deltaT_sec += *gpsOffset;
      }
   }

   return correction.deltaAf0 + correction.deltaAf1 * deltaT_sec;
}
