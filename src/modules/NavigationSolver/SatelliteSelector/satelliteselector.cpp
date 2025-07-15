#include "satelliteselector.h"
#include "modules/IO/datamanager.h"

bool navsolver::SatelliteSelector::execute(pipeline::Context& ctx) {
   ctx.visibleSats = std::move(selectVisibleSatellites(ctx));
   return true;
}

VisibleSatellites navsolver::SatelliteSelector::selectVisibleSatellites(
   pipeline::Context& ctx) const {
   constexpr double  elevationMaskDeg = 5.0;
   VisibleSatellites result;
   ScopedTimer timer("selectVisibleSatellites");

   const auto sp3 = ctx.dm->getSP3File();

   if (sp3 == nullptr) {
      qWarning() << "[SatelliteSelector] SP3-файл отсутствует (nullptr)";
      return result;
   }

   if (sp3->records.isEmpty()) {
      return result;
   }
   QSet<QDateTime> allowedEpochSet(ctx.allowedEpochs.begin(), ctx.allowedEpochs.end());

   for (auto itEpoch = sp3->records.constBegin(); itEpoch != sp3->records.constEnd(); ++itEpoch) {
      const QDateTime& epoch = itEpoch.key();

      if (!allowedEpochSet.contains(epoch)) {
         continue;
      }

      const auto& epochRecords = itEpoch.value();

      for (const auto& point : ctx.gridPoints) {
         QVector<Satellite> visibleSats;
         visibleSats.reserve(epochRecords.size());

         for (auto itSat = epochRecords.constBegin(); itSat != epochRecords.constEnd(); ++itSat) {
            const Satellite& sat      = itSat.key();
            const COORD_XYZ& satCoord = itSat.value().coord;
            COORD_XYZ sp3CoordM;
            sp3CoordM.x = satCoord.x * 1000.0;
            sp3CoordM.y = satCoord.y * 1000.0;
            sp3CoordM.z = satCoord.z * 1000.0;

            double elevation = Coordinates::getElevation(point.ecef, sp3CoordM);

            if (elevation >= elevationMaskDeg) {
               visibleSats.append(sat);
            }
         }

         result[epoch].insert(point, std::move(visibleSats));
      }
   }

   return result;
}
