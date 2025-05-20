#include "datamanager.h"

#include <QFileInfo>

using namespace io;


bool DataManager::loadSP3(const QString& fullPath) {
   QFileInfo sp3FileInfo(fullPath);

   if (!sp3FileInfo.exists()) {
      return false;
   }
   auto sp3File = std::make_unique<sp3::SP3_FILE>();

   if (!sp3::Sp3Reader::parse(sp3FileInfo.path(), sp3FileInfo.fileName(), *sp3File)) {
      return false;
   }

   sp3File_ = std::move(sp3File);

   return true;
}

const sp3::SP3_FILE*DataManager::getSP3File() const {
   return sp3File_.get();
}

bool DataManager::loadRinexNav(const QString& filePath) {
   QFileInfo rinexFileInfo(filePath);

   if (!rinexFileInfo.exists()) {
      return false;
   }
   auto rinexFile = std::make_unique<rinex::RINEX_FILE>();

   if (rinex::RinexReader::parse(rinexFileInfo.path(), rinexFileInfo.fileName(), *rinexFile) != rinex::PARSE_RESULT::SUCCESS) {
      return false;
   }

   rinexFile_ = std::move(rinexFile);

   return true;
}

const rinex::RINEX_FILE*DataManager::getRinexFile() const {
   return rinexFile_.get();
}

bool DataManager::loadSBASCorrections(const QString& path, SourceType sourceType) {
   return sbasStore_.load(sourceType, path);
}

const SBASCorrectionStore &DataManager::getSBASStore() const {
   return sbasStore_;
}
