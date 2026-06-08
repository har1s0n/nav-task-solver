#include <QCoreApplication>
#include <QDebug>

#include "application.h"

int main(int argc, char* argv[]) {
   QCoreApplication a(argc, argv);

   ApplicationConfig config;

   config.sp3Path             = "/Users/tarasovevgenij/Documents/2026_sisnet/2026-04-09/Sta24134.sp3";
   config.rinexNavGlonassPath = "/Users/tarasovevgenij/Documents/2026_sisnet/2026-04-09/Brdc0990.26g";
   config.rinexNavGpsPath     = "/Users/tarasovevgenij/Documents/2026_sisnet/2026-04-09/Brdc0990.26n";
   config.sbasPath            = "/Users/tarasovevgenij/Documents/2026_sisnet/2026-04-09/2026-04-09_backup_sisnet.csv";
   config.dcbPath             = "/Users/tarasovevgenij/Documents/2026_sisnet/2026-04-09/IPG_20260990000_01D_01D_DCB.BSX";
   config.antexPath           = "/Users/tarasovevgenij/Documents/2026_sisnet/2026-04-09/igs20.atx";
   config.satMetadataPath     = "/Users/tarasovevgenij/Documents/2026_sisnet/2026-04-09/igs_satellite_metadata.snx";

   Application navApp;

   if (!navApp.initialize(config)) {
      return 1;
   }

   return navApp.run();
}
