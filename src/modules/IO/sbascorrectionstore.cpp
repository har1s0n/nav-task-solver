#include "sbascorrectionstore.h"

#include <QDir>

using namespace io;

bool SBASCorrectionStore::load(SourceType type, const QString& path) {
   parsedMessages_.clear();
   correctionsBySat_.clear();
   timeOffsets_.clear();

   bool ok = false;

   switch (type) {
     case SourceType::FILE_HEX: ok = loadHex(path); break;
     case SourceType::FILE_CSV: ok = loadCsv(path); break;
     case SourceType::DATABASE: ok = loadDb();     break;
   }

   if (ok) {
      buildCorrectionIndex();
   }
   return ok;
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

   for (const auto& entry : entries) {
      QDateTime entryTime = QDateTime(epoch.date(), QTime(0, 0)).addSecs(entry.t0);

      if (entryTime > epoch) {
         continue;
      }
      int dt = std::abs(epoch.secsTo(entryTime));

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

std::optional<double> SBASCorrectionStore::gpsGlonassOffset(const QDateTime& epoch) const {
   for (const auto& m:timeOffsets_) {
      if ((epoch >= m.start) && (epoch < m.end)) {
         return m.timeCorrectionOffset;
      }
   }
   return std::nullopt;
}

bool SBASCorrectionStore::loadHex(const QString& path) {
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
      auto tokens        = line.split(' ');

      if (tokens.size() < 9) {
         continue;
      }
      auto testDt = QString("%1-%2-%3 %4:%5:%6")
                    .arg("20" + tokens[1])
                    .arg(tokens[2])
                    .arg(tokens[3])
                    .arg(tokens[4])
                    .arg(tokens[5])
                    .arg(tokens[6]);
      auto testRawHex    = tokens[8].trimmed();
      QDateTime recvTime = QDateTime::fromString(testDt, "yyyy-MM-dd hh:mm:ss");

      if (!recvTime.isValid()) {
         continue;
      }

      ++total;
      auto result = parser_.parse(testRawHex);

      if ((result.msgStatus == sbas::PARSE_STATUS::OK) && result.msg) {
         result.msg->recvTime = recvTime;
         parsedMessages_[result.msg->getTypeMsg()].append(result.msg);
         ++parsed;
      }
   }
   qDebug() << "[Hex] Прочитано:" << total << ", успешно:" << parsed;

   return parsed > 0;
}

bool SBASCorrectionStore::loadCsv(const QString& path) {
   QFile file(path);

   if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      qWarning() << "Failed to open:" << path << ", error:" << file.errorString();
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

   qDebug() << "[CSV] Прочитано:" << total << ", успешно:" << parsed;

   return parsed > 0;
}

bool SBASCorrectionStore::loadDb() {
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

      PrnMaskInterval interval;
      interval.startDt = startTime;
      interval.endDt   = endTime;
      interval.prns    = msg->satelites;

      prnMaskTimeline_.append(std::move(interval));
   }

   const auto longtermMsgs = getByType(sbas::MESSAGE_TYPE::LONG_TERM_SATELLITE_ERROR_CORRECTIONS);

   for (const auto& msgRaw : longtermMsgs) {
      auto msg = std::dynamic_pointer_cast<sbas::MSG_LONG_TERM_SATELLITE_ERROR_CORRECTIONS> (msgRaw);

      if (!msg || msg->recvTime.isNull()) {
         continue;
      }

      QDateTime msgTime = msg->recvTime;

      for (int i = 0; i < msg->satellites_code_1.size(); ++i) {
         const auto& code1               = msg->satellites_code_1[i];
         std::optional<Satellite> optSat = resolveSatellite(code1.prn, msgTime);

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
         std::optional<Satellite> optSat = resolveSatellite(code0.prn, msgTime);

         if (!optSat) {
            continue;
         }

         const Satellite& sat = *optSat;
         LongTermCorrectionEntry entry{ sat, code0.iode,
                                        QTime(0, 0).secsTo(msgTime.time()),
                                        code0.deltaEcef, { 0.0, 0.0, 0.0 },
                                        code0.delta_a_f0, 0.0, false };
         correctionsBySat_[sat].append(entry);
      }
   }

   const auto messages = getByType(sbas::MESSAGE_TYPE::NETWORK_OFFSET);

   for (int i = 0; i < messages.size(); ++i) {
      auto msg = std::dynamic_pointer_cast<sbas::MSG_NETWORK_OFFSET> (messages[i]);

      if (!msg) {
         continue;
      }

      QDateTime start = msg->recvTime;
      QDateTime end   = (i + 1 < messages.size())
                           ? messages[i + 1]->recvTime
                           : start.addSecs(86400);

      timeOffsets_.append({ start, end, msg->correctionOffset });
   }
}

std::optional<Satellite> SBASCorrectionStore::resolveSatellite(int prnMaskNumber, const QDateTime& recvTime) const {
   for (const auto& interval : prnMaskTimeline_) {
      if ((recvTime >= interval.startDt) && (recvTime < interval.endDt) &&
          (prnMaskNumber >= 1) /*&& (prnMaskNumber <= interval.prnList.size())*/) {
         const auto& prn = interval.prns[prnMaskNumber - 1];
         auto typeGNSS   = prn.systemSat == sbas::PURPOSE_SYSTEM::GLONASS ? SatelliteSystem::TYPE::GLONASS : SatelliteSystem::TYPE::GPS;
         return Satellite(prn.satId, typeGNSS);
      }
   }
   return std::nullopt;
}
