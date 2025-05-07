#include <QCoreApplication>
#include <QDebug>

#include "application.h"

int main(int argc, char* argv[]) {
   QCoreApplication a(argc, argv);

   ApplicationConfig config;

   config.sp3Path      = "/home/user/IAC23506.sp3.glo";
   config.rinexNavPath = "/home/user/mdvj0760.25g";
   config.sbasPath     = "data/sbas.dat";

   Application navApp;

   if (!navApp.initialize(config)) {
      return 1;
   }

   return navApp.run();
}
