#include <QCoreApplication>
#include <QDebug>

#include "application.h"

int main(int argc, char* argv[]) {
   QCoreApplication a(argc, argv);

   ApplicationConfig config;

   config.sp3Path             = "/Users/tarasovevgenij/Documents/25.04.2024_test/Sta23114.sp3";
   config.rinexNavGlonassPath = "/Users/tarasovevgenij/Documents/25.04.2024_test/Brdc1160.24g";
   config.rinexNavGpsPath     = "/Users/tarasovevgenij/Documents/25.04.2024_test/Brdc1160.24n";
   config.sbasPath            = "/Users/tarasovevgenij/Documents/25.04.2024_test/data-1773399945589.csv";
   // config.sbasPath = "/Users/tarasovevgenij/Documents/24.04.25_test/test_sbas/test_sbas_msg.csv";
   config.dcbPath = "/Users/tarasovevgenij/Documents/25.04.2024_test/IPG_20241160000_01D_01D_DCB.BSX";
   // config.sbasPath = "/Users/tarasovevgenij/Documents/h01.ems";

   Application navApp;

   if (!navApp.initialize(config)) {
      return 1;
   }

   return navApp.run();
}
