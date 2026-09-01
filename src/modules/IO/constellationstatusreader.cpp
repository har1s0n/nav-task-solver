#include "constellationstatusreader.h"

#include <QDate>
#include <QDebug>
#include <QFile>

namespace io {
namespace {
// Позиции фиксированных колонок (0-based), раскладка описана в заголовке
constexpr int kPosPrn        = 1;
constexpr int kLenPrn        = 3;
constexpr int kPosSvn        = 7;
constexpr int kLenSvn        = 5;
constexpr int kPosNorad      = 13;
constexpr int kLenNorad      = 5;
constexpr int kPosBlock      = 19;
constexpr int kLenBlock      = 6;
constexpr int kPosFileDate   = 26;
constexpr int kLenDate       = 8;
constexpr int kPosCommission = 44;

// Строка должна дотягивать хотя бы до конца колонки даты ввода в строй
constexpr int kMinLineLength = kPosCommission + kLenDate;
} // namespace

QDateTime ConstellationStatusReader::parseDate(const QString& ddmmyy) {
   const QString s = ddmmyy.trimmed();

   if ((s.size() != 8) || (s.at(2) != '.') || (s.at(5) != '.')) {
      return QDateTime();
   }

   bool      okDay = false, okMonth = false, okYear = false;
   const int day   = s.mid(0, 2).toInt(&okDay);
   const int month = s.mid(3, 2).toInt(&okMonth);
   int       year  = s.mid(6, 2).toInt(&okYear);

   if (!okDay || !okMonth || !okYear) {
      return QDateTime();
   }

   // Двузначный год: yy < 50 → 20yy, иначе 19yy
   // (то же соглашение, что в SatelliteMetadataReader::parseSinexDateTime)
   year += (year < 50) ? 2000 : 1900;

   const QDate date(year, month, day);

   if (!date.isValid()) {
      return QDateTime();
   }

   return QDateTime(date, QTime(0, 0, 0), Qt::UTC);
}

bool ConstellationStatusReader::loadSvnMap(const QString& path, QMap<int, QString>& out) {
   QFile file(path);

   if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      qCritical() << "[ConstStatus] Не удалось открыть таблицу SVN:" << path;
      return false;
   }

   int malformed = 0;

   while (!file.atEnd()) {
      const QString line = QString::fromLatin1(file.readLine()).trimmed();

      if (line.isEmpty() || line.startsWith('#')) {
         continue;
      }

      const QStringList parts = line.split(' ', Qt::SkipEmptyParts);

      if (parts.size() < 2) {
         ++malformed;
         continue;
      }

      bool      ok  = false;
      const int num = parts.at(0).toInt(&ok);
      const QString svn = parts.at(1);

      // SVN формата ANTEX: буква системы + три цифры
      const bool svnOk = (svn.size() == 4) && svn.at(0).isLetter()
                         && svn.at(1).isDigit() && svn.at(2).isDigit() && svn.at(3).isDigit();

      if (!ok || (num <= 0) || !svnOk) {
         ++malformed;
         qWarning().noquote() << QString("[ConstStatus] Битая строка таблицы SVN: '%1'").arg(line);
         continue;
      }

      out.insert(num, svn);
   }

   file.close();

   qInfo().noquote()
      << QString("[ConstStatus] Таблица SVN: %1 соответствий, битых строк %2")
      .arg(out.size()).arg(malformed);

   return !out.isEmpty();
}

bool ConstellationStatusReader::readOne(const QString&                    path,
                                        SatelliteSystem::TYPE             system,
                                        const QMap<int, QString>&         svnMap,
                                        satmeta::SATELLITE_METADATA_FILE& out,
                                        Statistics&                       stats) {
   QFile file(path);

   if (!file.open(QIODevice::ReadOnly)) {
      qCritical() << "[ConstStatus] Не удалось открыть файл состава ОГ:" << path;
      return false;
   }

   const bool  isGps  = (system == SatelliteSystem::TYPE::GPS);
   const QChar prefix = isGps ? QChar('G') : QChar('R');
   int&        total  = isGps ? stats.gpsTotal : stats.gloTotal;
   int&        inSvc  = isGps ? stats.gpsInService : stats.gloInService;

   while (!file.atEnd()) {
      QByteArray raw = file.readLine();

      while (raw.endsWith('\n') || raw.endsWith('\r')) {
         raw.chop(1);
      }

      if (raw.trimmed().isEmpty()) {
         continue;
      }

      // Latin-1, а не cp1251: используются только ASCII-колонки,
      // кириллица встречается лишь в типе НКА и комментарии
      const QString line = QString::fromLatin1(raw);

      if (line.size() < kMinLineLength) {
         ++stats.malformed;
         continue;
      }

      const QString prnRaw = line.mid(kPosPrn, kLenPrn).trimmed();

      if ((prnRaw.size() != kLenPrn) || (prnRaw.at(0) != prefix)) {
         ++stats.malformed;
         continue;
      }

      bool      ok  = false;
      const int prn = prnRaw.mid(1).toInt(&ok);

      if (!ok || (prn <= 0) || (prn > 99)) {
         ++stats.malformed;
         continue;
      }

      const int svnNum = line.mid(kPosSvn, kLenSvn).trimmed().toInt(&ok);

      if (!ok || (svnNum <= 0)) {
         ++stats.malformed;
         continue;
      }

      ++total;

      if (!stats.snapshotDate.isValid()) {
         stats.snapshotDate = parseDate(line.mid(kPosFileDate, kLenDate));
      }

      // Состояние НКА: пустая дата ввода в строй → в метаданные не попадает,
      // SvnResolver для такого PRN вернёт пустой SVN и НКА выпадет из расчёта
      const QDateTime commissioned = parseDate(line.mid(kPosCommission, kLenDate));

      if (!commissioned.isValid()) {
         qInfo().noquote()
            << QString("[ConstStatus] %1%2 (SVN %3) не введён в строй — пропущен")
            .arg(prefix)
            .arg(prn, 2, 10, QChar('0'))
            .arg(svnNum);
         continue;
      }

      ++inSvc;

      // SVN в формате ANTEX. Для GPS номер НКА совпадает с IGS SVN и приводится
      // напрямую; для ГЛОНАСС пространства идентификаторов разные, поэтому SVN
      // берётся только из таблицы — прямое приведение дало бы PCO чужого НКА
      QString svn;

      if (isGps) {
         svn = QString("%1%2").arg(prefix).arg(svnNum, 3, 10, QChar('0'));
      } else {
         svn = svnMap.value(svnNum);

         if (svn.isEmpty()) {
            ++stats.svnUnmapped;
            qWarning().noquote()
               << QString("[ConstStatus] %1%2: номера НКА %3 нет в таблице SVN — исключён")
               .arg(prefix)
               .arg(prn, 2, 10, QChar('0'))
               .arg(svnNum);
            continue;
         }
      }

      satmeta::SatelliteMetadataRecord rec;

      rec.svn       = svn;
      rec.system    = system;
      rec.prn       = prn;
      rec.validFrom = commissioned;
      rec.validTo   = QDateTime(); // в строю на дату файла
      rec.satCat    = line.mid(kPosNorad, kLenNorad).trimmed();

      // Тип НКА берём только из чисто ASCII-колонки (GPS: "II-F", "IIR-M", "III-A");
      // для ГЛОНАСС там кириллица в cp1251, а тип антенны всё равно даёт ANTEX
      const QString block     = line.mid(kPosBlock, kLenBlock).trimmed();
      bool          asciiOnly = true;

      for (const QChar c : block) {
         if (c.unicode() > 0x7F) {
            asciiOnly = false;
            break;
         }
      }

      if (asciiOnly) {
         rec.block = block;
      }

      out.records.insert(satmeta::SatelliteMetadataReader::makeKey(system, prn), rec);
      out.bySvn.insert(rec.svn, rec);
   }

   file.close();

   return true;
}

bool ConstellationStatusReader::read(const QString&                    gpsPath,
                                     const QString&                    gloPath,
                                     const QString&                    svnMapPath,
                                     satmeta::SATELLITE_METADATA_FILE& out,
                                     Statistics*                       stats) {
   Statistics         local;
   QMap<int, QString> svnMap;

   out = satmeta::SATELLITE_METADATA_FILE{};

   if (!loadSvnMap(svnMapPath, svnMap)) {
      return false;
   }

   const bool okGps = readOne(gpsPath, SatelliteSystem::TYPE::GPS,     svnMap, out, local);
   const bool okGlo = readOne(gloPath, SatelliteSystem::TYPE::GLONASS, svnMap, out, local);

   out.agency       = QStringLiteral("Const");
   out.creationDate = local.snapshotDate;

   if (stats) {
      *stats = local;
   }

   qInfo().noquote()
      << QString("[ConstStatus] Состав ОГ на %1: GPS %2/%3 в строю, ГЛОНАСС %4/%5 в строю, "
                 "битых строк %6, без записи в таблице SVN %7, записей PRN→SVN %8")
      .arg(local.snapshotDate.isValid() ? local.snapshotDate.toString("yyyy-MM-dd")
                                        : QStringLiteral("—"))
      .arg(local.gpsInService).arg(local.gpsTotal)
      .arg(local.gloInService).arg(local.gloTotal)
      .arg(local.malformed)
      .arg(local.svnUnmapped)
      .arg(out.records.size());

   return okGps && okGlo && !out.records.isEmpty();
}
} // namespace io
