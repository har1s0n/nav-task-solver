#include <QCoreApplication>
#include <QDebug>

#include "modules/io/datamanager.h"

int main(int argc, char* argv[]) {
   QCoreApplication a(argc, argv);

   qDebug() << "test" << '\n';

   return a.exec();
}
