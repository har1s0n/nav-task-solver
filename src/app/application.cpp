#include "application.h"


bool Application::initialize(const ApplicationConfig& config) {
   config_ = config;

   if (config_.sp3Path.isEmpty() || !dataManager_.loadSP3(config_.sp3Path.trimmed())) {
      qCritical() << "Ошибка загрузки SP3";
      return false;
   }

   if (config_.rinexNavPath.isEmpty() || !dataManager_.loadRinexNav(config_.rinexNavPath.trimmed())) {
      qCritical() << "Ошибка загрузки RINEX NAV";
      return false;
   }

   if (config_.sbasPath.isEmpty() || !dataManager_.loadSBASCorrections(config_.sbasPath.trimmed(), io::SourceType::FILE_CSV)) {
      qCritical() << "SBAS (CSV) не загружен";
      return false;
   }

   const auto& sbas_store = dataManager_.getSBASStore();
   auto test_sbas         = sbas_store.messages();


   return true;
}

int Application::run() {
   if (!applyCorrections()) {
      qWarning() << "Поправки не применены";
   }
   return 0;
}

bool Application::applyCorrections() {
   // применение поправок к RINEX
   return true;
}
