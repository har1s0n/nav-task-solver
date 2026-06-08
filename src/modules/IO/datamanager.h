#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <QString>
#include <memory>

#include <inav/Sp3>
#include <inav/RINEX>
#include <inav/Antex>

#include "modules/IModule/imodule.h"
#include "sbascorrectionstore.h"

namespace io {
class DataManager : public pipeline::IModule {
public:

   struct Config {
      QString    sp3Path;
      QString    rinexNavGlonassPath;
      QString    rinexNavGpsPath;
      QString    sbasPath;
      SourceType sbasSourceType = SourceType::FILE_CSV;
      QString    dcbPath;
      QString    antexPath;
      QString    satMetadataPath;
   };
   explicit DataManager(const Config& cfg);
   bool    execute(pipeline::Context& ctx) override;
   QString name() const override {
      return QStringLiteral("Загрузка IO-данных");
   }

   const sp3::SP3_FILE*               getSP3File()const noexcept;
   const rinex::RINEX_FILE*           getRinexGlonassFile() const noexcept;
   const rinex::RINEX_FILE*           getRinexGpsFile() const noexcept;
   const rinex::RINEX_FILE*           getRinexFile()const noexcept;
   const SBASCorrectionStore &        getSBASStore() const noexcept;
   io::SBASCorrectionStore &          sbasStore() noexcept;
   std::optional<double>              getGlonassL3MinusL1Bias(const Satellite& satId) const noexcept;
   bool                               loadAntennaModel(const QString& antexPath,
                                                       const QString& metadataPath);
   const antex::SatelliteAntennaModel*getAntennaModel() const noexcept;

private:

   bool loadSP3(const QString& path);
   bool loadRinexNav(const QString&                      path,
                     std::unique_ptr<rinex::RINEX_FILE>& dst,
                     const char*                         tag);
   bool loadSBASCorrections(const QString& path,
                            SourceType     sourceType);
   bool loadDCB(const QString& path);
   bool fileExists(const QString& path) const noexcept;

   void applyLeapSecondShiftToSP3(sp3::SP3_FILE& sp3,
                                  int            leapSeconds = 18);

private:

   Config cfg_;
   std::unique_ptr<sp3::SP3_FILE> sp3File_;
   std::unique_ptr<rinex::RINEX_FILE> rinexGlonassFile_;
   std::unique_ptr<rinex::RINEX_FILE> rinexGpsFile_;
   SBASCorrectionStore sbasStore_;
   std::unordered_map<Satellite, double> glonassDcbL3L1_;
   std::unique_ptr<antex::SatelliteAntennaModel> antennaModel_;
};
}

#endif // DATAMANAGER_H
