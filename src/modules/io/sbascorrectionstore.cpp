#include "sbascorrectionstore.h"

using namespace io;

SBASCorrectionStore::SBASCorrectionStore() {}

bool SBASCorrectionStore::load(SourceType type, const QString& sourcePath) {
   switch (type) {
     case SourceType::FILE_HEX:
        return loadFromFileHex(sourcePath);
     case SourceType::FILE_CSV:
        return loadFromCsvFile(sourcePath);
     case SourceType::DATABASE:
        return loadFromDatabase();
     default:
        return false;
   }
}

const QMap<sbas::MESSAGE_TYPE, QVector<std::shared_ptr<sbas::MSG> > > &SBASCorrectionStore::messages() const {
   return parsedMessages_;
}

QVector<std::shared_ptr<sbas::MSG> > SBASCorrectionStore::getByType(sbas::MESSAGE_TYPE type) const {
   return parsedMessages_.value(type);
}

bool SBASCorrectionStore::loadFromFileHex(const QString& path) {
   QFile file(path);

   if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      qWarning() << "Не удалось открыть файл:" << path;
      return false;
   }

   QTextStream in(&file);
   int total  = 0;
   int parsed = 0;

   while (!in.atEnd()) {
      const QString line = in.readLine().trimmed();

      if (line.isEmpty()) {
         continue;
      }

      ++total;
      auto result = parser_.parse(line);

      if ((result.msgStatus == sbas::PARSE_STATUS::OK) && result.msg) {
         parsedMessages_[result.msg->getTypeMsg()].append(result.msg);
         ++parsed;
      }
   }

   qDebug() << "[Hex] Прочитано:" << total << ", успешно:" << parsed;
   return parsed > 0;
}

bool SBASCorrectionStore::loadFromCsvFile(const QString& path) {
   QFile file(path);

   if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      qWarning() << "Не удалось открыть CSV:" << path;
      return false;
   }

   QTextStream in(&file);
   QString     header = in.readLine(); // пропускаем заголовок
   int total = 0, parsed = 0;

   while (!in.atEnd()) {
      const QString line   = in.readLine().trimmed();
      const auto    tokens = line.split(',');

      if (tokens.size() < 3) {
         continue;
      }

      const QString rawHex = tokens[2].trimmed(); // поле "data"
      ++total;
      auto result = parser_.parse(rawHex);

      if ((result.msgStatus == sbas::PARSE_STATUS::OK) && result.msg) {
         parsedMessages_[result.msg->getTypeMsg()].append(result.msg);
         ++parsed;
      }
   }

   qDebug() << "[CSV] Прочитано:" << total << ", успешно:" << parsed;
   return parsed > 0;
}

bool SBASCorrectionStore::loadFromDatabase() {
   qInfo() << "[Заглушка] SBASCorrectionStore::loadFromDatabaseStub";
   parsedMessages_.clear();
   auto testMsg = std::make_shared<sbas::MSG_TESTING>();
   parsedMessages_[sbas::MESSAGE_TYPE::TESTING].append(testMsg);
   return true;
}
