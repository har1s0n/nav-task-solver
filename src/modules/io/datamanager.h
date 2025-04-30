#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <QString>

class DataManager {
public:

   DataManager();

   bool loadSp3(const QString& filePath);
   bool loadRinexNav(const QString& filePath);
   bool loadSBASCorrections(const QString& filePath);
};

#endif // DATAMANAGER_H
