#include "datamanager.h"
#include "dcbparser.h"

#include <QFileInfo>

using namespace io;

DataManager::DataManager(const Config& cfg) : cfg_(std::move(cfg)) {}

bool DataManager::execute(pipeline::Context& ctx) {
   ctx.dm = this;

   if (!loadSP3(cfg_.sp3Path)) {
      qCritical() << name() << ": не удалось загрузить SP3:" << cfg_.sp3Path;
      return false;
   }

   if (!loadRinexNav(cfg_.rinexNavPath)) {
      qCritical() << name() << ": не удалось загрузить RINEX NAV:" << cfg_.rinexNavPath;
      return false;
   }

   // if (!loadSBASCorrections(cfg_.sbasPath, cfg_.sbasSourceType)) {
   //    qCritical() << name() << ": не удалось загрузить SBAS:" << cfg_.sbasPath;
   //    return false;
   // }

   if (!loadDCB(cfg_.dcbPath)) {
      qCritical() << name() << ": не удалось загрузить DCB:" << cfg_.dcbPath;
      return false;
   }

   return true;
}

bool DataManager::fileExists(const QString& path) const noexcept {
   QFileInfo info(path.trimmed());

   return info.exists() && info.isFile();
}

bool DataManager::loadSP3(const QString& path) {
   if (!fileExists(path)) {
      qWarning() << "SP3 file not found:" << path;
      sp3File_.reset();
      return false;
   }

   QFileInfo info(path.trimmed());
   auto sp3File = std::make_unique<sp3::SP3_FILE>();

   if (!sp3::Sp3Reader::parse(info.path(), info.fileName(), *sp3File)) {
      qWarning() << "Failed to parse SP3:" << path;
      return false;
   }
   sp3File_ = std::move(sp3File);

   return true;
}

const sp3::SP3_FILE*DataManager::getSP3File() const noexcept {
   return sp3File_.get();
}

bool DataManager::loadRinexNav(const QString& path) {
   if (!fileExists(path)) {
      qWarning() << "RINEX NAV file not found:" << path;
      rinexFile_.reset();
      return false;
   }
   QFileInfo info(path);
   auto rinexFile = std::make_unique<rinex::RINEX_FILE>();

   if (rinex::RinexReader::parse(info.path(), info.fileName(), *rinexFile) != rinex::PARSE_RESULT::SUCCESS) {
      qWarning() << "Failed to parse RINEX NAV:" << path;
      return false;
   }
   rinexFile_ = std::move(rinexFile);

   return true;
}

const rinex::RINEX_FILE*DataManager::getRinexFile() const noexcept {
   return rinexFile_.get();
}

bool DataManager::loadSBASCorrections(const QString& path, SourceType sourceType) {
   if (!fileExists(path)) {
      qWarning() << "SBAS corrections file not found:" << path;
      return false;
   }

   if (!sbasStore_.load(sourceType, path.trimmed())) {
      qWarning() << "Failed to load SBAS corrections:" << path;
      return false;
   }
   return true;
}

const SBASCorrectionStore &DataManager::getSBASStore() const noexcept {
   return sbasStore_;
}

bool DataManager::loadDCB(const QString& path) {
   if (!fileExists(path)) {
      qWarning() << "DCB file not found:" << path;
      glonassDcbL3L1_.clear();
      return false;
   }
   auto result = DCBParser::parseGlonassL3L1Bias(path.trimmed());

   if (result.empty()) {
      qWarning() << "Failed to parse DCB file:" << path;
      return false;
   }
   glonassDcbL3L1_ = std::move(result);

   return true;
}

std::optional<double> DataManager::getGlonassL3MinusL1Bias(const Satellite& satId) const noexcept {
   if (satId.getSystem() != SatelliteSystem::TYPE::GLONASS) {
      return std::nullopt;
   }
   auto it = glonassDcbL3L1_.find(satId);

   return it != glonassDcbL3L1_.end()
              ? std::make_optional(it->second)
              : std::nullopt;
}
