#ifndef APPLICATION_H
#define APPLICATION_H

#include "modules/io/datamanager.h"

struct ApplicationConfig {
   QString sp3Path;
   QString rinexNavPath;
   QString sbasPath;

   std::optional<QString> igsClockPath;
   std::optional<QString> rawObsPath;
   std::optional<QString> rinexObsPath;

   // флаги, режимы, параметры
   bool applyCorrections = true;
   bool refineEphemeris  = true;
   bool solveNavigation  = true;
};

class Application {
public:

   bool initialize(const ApplicationConfig& config);

   int  run();

private:

   bool applyCorrections();

private:

   io::DataManager dataManager_;
   ApplicationConfig config_;
};

#endif // APPLICATION_H
