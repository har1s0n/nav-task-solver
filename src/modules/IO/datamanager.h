#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <QString>
#include <memory>

#include <inav/Sp3>
#include <inav/RINEX>

#include "sbascorrectionstore.h"

namespace io {
class DataManager {
public:

   bool                       loadSP3(const QString& fullPath);
   const sp3::SP3_FILE*       getSP3File() const;
   bool                       loadRinexNav(const QString& filePath);
   const rinex::RINEX_FILE*   getRinexFile()const;
   bool                       loadSBASCorrections(const QString& path,
                                                  SourceType     sourceType);
   const SBASCorrectionStore &getSBASStore() const;

private:

   std::unique_ptr<sp3::SP3_FILE> sp3File_;
   std::unique_ptr<rinex::RINEX_FILE> rinexFile_;
   SBASCorrectionStore sbasStore_;
};
}

#endif // DATAMANAGER_H
