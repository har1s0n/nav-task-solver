#ifndef CORRECTIONAPPLIER_H
#define CORRECTIONAPPLIER_H

#include <inav/RINEX>

#include "modules/IO/sbascorrectionstore.h"

namespace corrections {
class CorrectionApplier {
public:

   explicit CorrectionApplier(const io::SBASCorrectionStore& correctionStore);
   void applySBASCorrections(rinex::RINEX_FILE& rinexNavFiles);

private:

   bool   applyCorrectionToNavRecord(rinex::NAV_RECORD& navRecord,
                                     const Satellite&   sat,
                                     const QDateTime&   epoch);
   double computeClockCorrection(const io::LongTermCorrectionEntry& correction,
                                 const QDateTime&                   epoch,
                                 bool                               isGlonass = true);

private:

   const io::SBASCorrectionStore& correctionStore_;
};
}
#endif // CORRECTIONAPPLIER_H
