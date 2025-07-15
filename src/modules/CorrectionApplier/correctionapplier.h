#ifndef CORRECTIONAPPLIER_H
#define CORRECTIONAPPLIER_H

#include <inav/RINEX>

#include "modules/IModule/imodule.h"
#include "modules/IO/sbascorrectionstore.h"

namespace corrections {
class CorrectionApplier : public pipeline::IModule {
public:

   CorrectionApplier() = default;
   bool    execute(pipeline::Context& ctx) override;
   QString name()    const override {
      return QStringLiteral("Применение SBAS-поправок");
   }

private:

   void applySBASCorrections(rinex::RINEX_FILE&             rinexNavFiles,
                             const io::SBASCorrectionStore& store);
   bool applyCorrectionToNavRecord(rinex::NAV_RECORD&             navRecord,
                                   const Satellite&               sat,
                                   const QDateTime&               epoch,
                                   const io::SBASCorrectionStore& store);
   double computeClockCorrection(const io::LongTermCorrectionEntry& correction,
                                 const QDateTime&                   epoch,
                                 const io::SBASCorrectionStore&     store,
                                 bool                               isGlonass = true);
};
}
#endif // CORRECTIONAPPLIER_H
