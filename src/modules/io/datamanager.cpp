#include "datamanager.h"
#include "sp3reader.h"
#include "rinexnavreader.h"
#include "sbascorrectionreader.h"

DataManager::DataManager() {}

bool DataManager::loadSp3(const QString& filePath) {
   SP3Reader sp3Reader;

   return sp3Reader.read();
}

bool DataManager::loadRinexNav(const QString& filePath) {
   RinexNavReader rinexReader;

   return rinexReader.read();
}

bool DataManager::loadSBASCorrections(const QString& filePath) {
   SBASCorrectionReader sbasReader;

   return sbasReader.read();
}
