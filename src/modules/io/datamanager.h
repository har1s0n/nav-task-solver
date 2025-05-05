#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <QString>

#include <inav/SP3>

class DataManager {
public:

   bool                loadSP3(const QString& fullPath);
   const sp3::SP3_FILE*getSP3File() const;

   bool                loadRinexNav(const QString& filePath);
   bool                loadSBASCorrections(const QString& filePath);

private:

   std::unique_ptr<sp3::SP3_FILE> sp3File_;
};

#endif // DATAMANAGER_H
