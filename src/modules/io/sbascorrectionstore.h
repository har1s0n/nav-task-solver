#ifndef SBASCORRECTIONSTORE_H
#define SBASCORRECTIONSTORE_H

#include <QString>
#include <QMap>

#include <inav/SBAS>

namespace io {
enum class SourceType {
   FILE_HEX,
   FILE_CSV,
   DATABASE
};

class SBASCorrectionStore {
public:

   SBASCorrectionStore();
   bool                                                                   load(SourceType     type,
                                                                               const QString& sourcePath);
   const QMap<sbas::MESSAGE_TYPE, QVector<std::shared_ptr<sbas::MSG> > > &messages() const;
   QVector<std::shared_ptr<sbas::MSG> >                                   getByType(sbas::MESSAGE_TYPE type) const;

private:

   bool loadFromFileHex(const QString& path);
   bool loadFromCsvFile(const QString& path);
   bool loadFromDatabase();

private:

   QMap<sbas::MESSAGE_TYPE, QVector<std::shared_ptr<sbas::MSG> > > parsedMessages_;
   sbas::SbasParser parser_;
};
}

#endif // SBASCORRECTIONSTORE_H
