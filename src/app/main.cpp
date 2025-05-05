#include <QCoreApplication>
#include <QDebug>

#include "application.h"

int main(int argc, char* argv[]) {
   QCoreApplication a(argc, argv);

   ApplicationConfig config;

   config.sp3Path      = "/Users/tarasovevgenij/Documents/Sta23601.sp3.glo";
   config.rinexNavPath = "data/test.nav";
   config.sisnetPath   = "data/sbas.dat";

   Application navApp;

   if (!navApp.initialize(config)) {
      return 1;
   }

   return navApp.run();
}
