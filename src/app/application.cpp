#include "application.h"


bool Application::initialize(const ApplicationConfig& config) {
   config_ = config;

   if (config_.sp3Path.isEmpty() || !dataManager_.loadSP3(config_.sp3Path)) {
      qCritical() << "Ошибка загрузки SP3";
      return false;
   }
   auto test_1 = dataManager_.getSP3File();

   if (config_.rinexNavPath.isEmpty() || !dataManager_.loadRinexNav(config_.rinexNavPath)) {
      qCritical() << "Ошибка загрузки RINEX NAV";
      return false;
   }
   auto test_2 = dataManager_.getRinexFile();

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
