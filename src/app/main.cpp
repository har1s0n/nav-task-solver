#include <QCoreApplication>
#include <QDebug>

#include "application.h"

int main(int argc, char* argv[]) {
   QCoreApplication a(argc, argv);

   ApplicationConfig config;

   config.sp3Path      = "/Users/tarasovevgenij/Documents/24.04.25_test/Sta23634.sp3.glo";
   config.rinexNavPath = "/Users/tarasovevgenij/Documents/24.04.25_test/GNSS00CMB_U_20251261311_15M_RN.rnx";
   config.sbasPath     = "/Users/tarasovevgenij/Documents/24.04.25_test/data-1746531729399.csv";
   // config.sbasPath = "/Users/tarasovevgenij/Documents/h01.ems";

   Application navApp;

   if (!navApp.initialize(config)) {
      return 1;
   }

   return navApp.run();
}
