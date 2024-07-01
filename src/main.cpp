#include <QCoreApplication>
#include <QDebug>

int main(int argc, char* argv[]) {
   QCoreApplication a(argc, argv);

   qDebug() << "test" << endl;

   return a.exec();
}
