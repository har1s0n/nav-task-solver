#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <QString>
#include <memory>

#include <inav/Sp3>
#include <inav/RINEX>

#include "modules/IModule/imodule.h"
#include "sbascorrectionstore.h"

namespace io {
class DataManager : public pipeline::IModule {
public:

   struct Config {
      QString    sp3Path;
      QString    rinexNavPath;
      QString    sbasPath;
      SourceType sbasSourceType = SourceType::FILE_CSV;
      QString    dcbPath;
   };
   explicit DataManager(const Config& cfg);
   bool    execute(pipeline::Context& ctx) override;
   QString name() const override {
      return QStringLiteral("Загрузка IO-данных");
   }

   bool                       loadSP3(const QString& path);
   bool                       loadRinexNav(const QString& path);
   bool                       loadSBASCorrections(const QString& path,
                                                  SourceType     sourceType);
   bool                       loadDCB(const QString& path);

   const sp3::SP3_FILE*       getSP3File()const noexcept;
   const rinex::RINEX_FILE*   getRinexFile()const noexcept;
   const SBASCorrectionStore &getSBASStore() const noexcept;
   io::SBASCorrectionStore &  sbasStore() noexcept;
   std::optional<double>      getGlonassL3MinusL1Bias(const Satellite& satId) const noexcept;

private:

   bool fileExists(const QString& path) const noexcept;

private:

   Config cfg_;
   std::unique_ptr<sp3::SP3_FILE> sp3File_;
   std::unique_ptr<rinex::RINEX_FILE> rinexFile_;
   SBASCorrectionStore sbasStore_;
   std::unordered_map<Satellite, double> glonassDcbL3L1_;
};
}

#endif // DATAMANAGER_H
