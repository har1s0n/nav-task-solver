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

std::optional<LongTermCorrectionEntry> SBASCorrectionStore::getLongTermCorrection(const Satellite& sat, const QDateTime& epoch) const {
   constexpr int MAX_TIME_DIFF_SECONDS = 360;
   auto it                             = correctionsBySat_.find(sat);

   if (it == correctionsBySat_.end()) {
      return std::nullopt;
   }

   const auto& entries                 = it.value();
   const LongTermCorrectionEntry* best = nullptr;
   int minTimeDiff                     = std::numeric_limits<int>::max();

   int epochSec = QTime(0, 0).secsTo(epoch.time());

   for (const auto& entry : entries) {
      int dt = std::abs(entry.t0 - epochSec);

      if ((dt <= MAX_TIME_DIFF_SECONDS) && (dt < minTimeDiff)) {
         minTimeDiff = dt;
         best        = &entry;
      }
   }

   if (best) {
      return *best;
   }
   return std::nullopt;
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

      const auto dtStr     = tokens[1].trimmed().remove('"');
      const QString rawHex = tokens[4].trimmed(); // поле data

      QDateTime recvTime = QDateTime::fromString(dtStr, "yyyy-MM-dd hh:mm:ss");

      if (!recvTime.isValid()) {
         continue;
      }
      ++total;
      auto result = parser_.parse(rawHex);

      if ((result.msgStatus == sbas::PARSE_STATUS::OK) && result.msg) {
         result.msg->recvTime = recvTime;
         parsedMessages_[result.msg->getTypeMsg()].append(result.msg);
         ++parsed;
      }
   }
   buildCorrectionIndex();

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

void SBASCorrectionStore::buildCorrectionIndex() {
   correctionsBySat_.clear();
   prnMaskTimeline_.clear();

   const auto prnMaskMessages = getByType(sbas::MESSAGE_TYPE::PRN_MASK);

   for (int i = 0; i < prnMaskMessages.size(); ++i) {
      auto msg = std::dynamic_pointer_cast<sbas::MSG_PRN_MASK> (prnMaskMessages[i]);

      if (!msg || msg->satelites.isEmpty()) {
         continue;
      }

      QDateTime startTime = msg->recvTime;
      QDateTime endTime   = (i + 1 < prnMaskMessages.size())
                                ? prnMaskMessages[i + 1]->recvTime
                                : startTime.addSecs(600);

      QVector<Satellite> sats;

      for (const auto& s : msg->satelites) {
         if (s.systemSat == sbas::PURPOSE_SYSTEM::GLONASS) {
            sats.append(Satellite(s.satId, SatelliteSystem::TYPE::GLONASS));
         }
      }

      if (!sats.isEmpty()) {
         prnMaskTimeline_.append({ startTime, endTime, msg->iodp, sats });
      }
   }

   const auto longtermMsgs = getByType(sbas::MESSAGE_TYPE::LONG_TERM_SATELLITE_ERROR_CORRECTIONS);

   for (const auto& msgRaw : longtermMsgs) {
      auto msg = std::dynamic_pointer_cast<sbas::MSG_LONG_TERM_SATELLITE_ERROR_CORRECTIONS> (msgRaw);

      if (!msg || msg->recvTime.isNull()) {
         continue;
      }

      QDateTime msgTime = msg->recvTime;
      // int iodp          = msg->iodp; // предполагается, что это поле присутствует

      for (int i = 0; i < msg->satellites_code_1.size(); ++i) {
         const auto& code1               = msg->satellites_code_1[i];
         std::optional<Satellite> optSat = resolveSatellite(code1.iodp, i + 1, msgTime);

         if (!optSat) {
            continue;
         }

         const Satellite& sat = *optSat;
         LongTermCorrectionEntry entry{ sat, code1.iode, code1.t0,
                                        code1.deltaEcef, code1.deltaRoc,
                                        code1.delta_a_f0, code1.delta_a_f1, true };
         correctionsBySat_[sat].append(entry);
      }

      for (int i = 0; i < msg->satellites_code_0.size(); ++i) {
         const auto& code0               = msg->satellites_code_0[i];
         std::optional<Satellite> optSat = resolveSatellite(code0.iode, i + 1, msgTime);

         if (!optSat) {
            continue;
         }

         const Satellite& sat = *optSat;
         LongTermCorrectionEntry entry{ sat, code0.iode,
                                        msgTime.time().secsTo(QTime(0, 0)),
                                        code0.deltaEcef, { 0.0, 0.0, 0.0 },
                                        code0.delta_a_f0, 0.0, false };
         correctionsBySat_[sat].append(entry);
      }
   }
}

bool SBASCorrectionStore::isPrnAllowed(const Satellite& sat, const QDateTime& time) const {
   for (const auto& interval : prnMaskTimeline_) {
      if ((time >= interval.startDt) && (time < interval.endDt) && interval.satList.contains(sat)) {
         return true;
      }
   }
   return false;
}

std::optional<Satellite> SBASCorrectionStore::resolveSatellite(int iodp, int prnMaskNumber, const QDateTime& recvTime) const {
   for (const auto& interval : prnMaskTimeline_) {
      if ((interval.iodp == iodp) &&
          (recvTime >= interval.startDt) && (recvTime < interval.endDt) &&
          (prnMaskNumber >= 1) && (prnMaskNumber <= interval.satList.size())) {
         return interval.satList[prnMaskNumber - 1];
      }
   }
   return std::nullopt;
}
