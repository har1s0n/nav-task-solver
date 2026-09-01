#include "datamanager.h"
#include "constellationstatusreader.h"
#include "dcbparser.h"

#include <QFileInfo>

using namespace io;

DataManager::DataManager(const Config& cfg) : cfg_(std::move(cfg)) {}

bool DataManager::execute(pipeline::Context& ctx) {
   ctx.dm = this;

   if (!loadSP3(cfg_.sp3Path)) {
      qCritical() << name() << ": не удалось загрузить SP3:" << cfg_.sp3Path;
      return false;
   }

   if (!loadRinexNav(cfg_.rinexNavGlonassPath, rinexGlonassFile_, "GLONASS")) {
      qCritical() << name() << ": не удалось загрузить RINEX NAV (GLONASS):"
                  << cfg_.rinexNavGlonassPath;
      return false;
   }

   if (!loadRinexNav(cfg_.rinexNavGpsPath, rinexGpsFile_, "GPS")) {
      qCritical() << name() << ": не удалось загрузить RINEX NAV (GPS):"
                  << cfg_.rinexNavGpsPath;
      return false;
   }

   if (!loadSBASCorrections(cfg_.sbasPath, cfg_.sbasSourceType)) {
      qCritical() << name() << ": не удалось загрузить SBAS:" << cfg_.sbasPath;
      return false;
   }

   if (!loadDCB(cfg_.dcbPath)) {
      qCritical() << name() << ": не удалось загрузить DCB:" << cfg_.dcbPath;
      return false;
   }

   if (!loadAntennaModel(cfg_.antexPath, cfg_.constGpsPath, cfg_.constGloPath,
                         cfg_.constSvnMapPath)) {
      qCritical() << name() << ": не удалось загрузить ANTEX модель:" << cfg_.antexPath
                  << " | " << cfg_.constGpsPath << " | " << cfg_.constGloPath;
      return false;
   }


   return true;
}

bool DataManager::fileExists(const QString& path) const noexcept {
   QFileInfo info(path.trimmed());

   return info.exists() && info.isFile();
}

static double lagrangeInterpolate(const QVector<double>& x, const QVector<double>& y, double xi) {
   double result = 0.0;
   int    n      = x.size();

   for (int i = 0; i < n; i++) {
      double term = y[i];

      for (int j = 0; j < n; j++) {
         if (j != i) {
            term = term * (xi - x[j]) / (x[i] - x[j]);
         }
      }
      result += term;
   }
   return result;
}

void DataManager::applyLeapSecondShiftToSP3(sp3::SP3_FILE& sp3, int leapSeconds) {
   if (sp3.records.isEmpty()) {
      return;
   }

   sp3::SP3_FILE shiftedSp3 = sp3;
   shiftedSp3.records.clear();

   auto epochs = sp3.records.keys();
   std::sort(epochs.begin(), epochs.end());

   // Используем окно в 10 точек для полинома 9-й степени (стандарт для SP3)
   const int N_POINTS = 10;

   for (const QDateTime& currentEpoch : epochs) {
      // Искомое время в шкале GPS (на leapSeconds впереди UTC)
      QDateTime targetGpsTime = currentEpoch.addSecs(leapSeconds);

      // Ищем окно из N_POINTS ближайших эпох
      int bestIdx    = 0;
      double minDiff = std::numeric_limits<double>::max();

      for (int i = 0; i <= epochs.size() - N_POINTS; ++i) {
         double diff = std::abs(epochs[i + N_POINTS / 2].secsTo(targetGpsTime));

         if (diff < minDiff) {
            minDiff = diff;
            bestIdx = i;
         }
      }

      int startIdx = std::max(0, bestIdx);
      int pts      = std::min(N_POINTS, (int)epochs.size());

      QMap<Satellite, sp3::SP3_RECORD> newEpochRecords;

      // Проходим по всем спутникам в текущую эпоху
      for (const Satellite& sat : sp3.records[currentEpoch].keys()) {
         if (sat.getSystem() != SatelliteSystem::TYPE::GLONASS) {
            // SP3 для GPS уже в GPS Time — копируем без интерполяции
            newEpochRecords.insert(sat, sp3.records[currentEpoch][sat]);
            continue;
         }
         QVector<QDateTime> satTimes;
         QVector<COORD_XYZ> satCoords;

         // Собираем валидные узлы интерполяции для конкретного НКА
         for (int i = startIdx; i < startIdx + pts; ++i) {
            if (sp3.records[epochs[i]].contains(sat)) {
               satTimes.push_back(epochs[i]);
               satCoords.push_back(sp3.records[epochs[i]][sat].coord);
            }
         }

         // Если недостаточно точек для интерполяции, оставляем как есть
         if (satTimes.size() < 2) {
            newEpochRecords.insert(sat, sp3.records[currentEpoch][sat]);
            continue;
         }

         int n = satTimes.size();
         QVector<double> oldIntervals;
         QVector<double> coordX, coordY, coordZ;
         oldIntervals.reserve(n);
         coordX.reserve(n);
         coordY.reserve(n);
         coordZ.reserve(n);

         double t0         = satTimes.first().toMSecsSinceEpoch() / 1000.0;
         double targetTime = (targetGpsTime.toMSecsSinceEpoch() / 1000.0) - t0;

         for (int i = 0; i < n; ++i) {
            oldIntervals.push_back((satTimes[i].toMSecsSinceEpoch() / 1000.0) - t0);
            coordX.push_back(satCoords[i].x);
            coordY.push_back(satCoords[i].y);
            coordZ.push_back(satCoords[i].z);
         }

         sp3::SP3_RECORD newRec = sp3.records[currentEpoch][sat];

         // Интерполируем координаты напрямую
         newRec.coord.x = lagrangeInterpolate(oldIntervals, coordX, targetTime);
         newRec.coord.y = lagrangeInterpolate(oldIntervals, coordY, targetTime);
         newRec.coord.z = lagrangeInterpolate(oldIntervals, coordZ, targetTime);

         // ===================================================================
         // ВРЕМЕННАЯ МЕТРИКА: ДЕТЕКТОР ОСЦИЛЛЯЦИЙ (Феномен Рунге)
         // ===================================================================
         const COORD_XYZ& origCoord = sp3.records[currentEpoch][sat].coord;
         double dx                  = newRec.coord.x - origCoord.x;
         double dy                  = newRec.coord.y - origCoord.y;
         double dz                  = newRec.coord.z - origCoord.z;

         // SP3 координаты обычно хранятся в километрах.
         double shiftDistanceKm = std::sqrt(dx * dx + dy * dy + dz * dz);

         // Ожидаемый сдвиг: ~3.5 км/с * 18 сек = ~63 км.
         // Жесткий допуск: от 40 км до 90 км (с учетом проекций и эллиптичности).
         if ((shiftDistanceKm > 90.0) || (shiftDistanceKm < 40.0)) {
            qWarning().noquote() << QString("[LeapSecond] ВНИМАНИЕ: Возможная осцилляция! %1 эпоха %2. "
                                            "Сдвиг: %3 км (ожидалось ~63 км).")
               .arg(sat.toString())
               .arg(currentEpoch.toString(Qt::ISODate))
               .arg(shiftDistanceKm, 0, 'f', 2);
         }
         // ===================================================================

         newEpochRecords.insert(sat, newRec);
      }

      shiftedSp3.records.insert(currentEpoch, newEpochRecords);
   }

   sp3 = shiftedSp3;
}

bool DataManager::loadSP3(const QString& path) {
   if (!fileExists(path)) {
      qWarning() << "SP3 file not found:" << path;
      sp3File_.reset();
      return false;
   }

   QFileInfo info(path.trimmed());
   auto sp3File = std::make_unique<sp3::SP3_FILE>();

   if (!sp3::Sp3Reader::parse(info.path(), info.fileName(), *sp3File)) {
      qWarning() << "Failed to parse SP3:" << path;
      return false;
   }
   // приводим sp3 эфемериды к UTS(SU) (сдвигая на leapSecond)
   applyLeapSecondShiftToSP3(*sp3File, 18);
   qInfo() << "[DataManager] SP3-координаты успешно сдвинуты на +18 сек (Leap Second) через полином Лагранжа";

   sp3File_ = std::move(sp3File);

   return true;
}

const sp3::SP3_FILE*DataManager::getSP3File() const noexcept {
   return sp3File_.get();
}

bool DataManager::loadRinexNav(const QString&                      path,
                               std::unique_ptr<rinex::RINEX_FILE>& dst,
                               const char*                         tag) {
   if (!fileExists(path)) {
      qWarning() << "RINEX NAV file not found:" << tag << path;
      dst.reset();
      return false;
   }

   QFileInfo info(path.trimmed());
   auto rinexFile = std::make_unique<rinex::RINEX_FILE>();

   if (rinex::RinexReader::parse(info.path(), info.fileName(), *rinexFile)
       != rinex::PARSE_RESULT::SUCCESS) {
      qWarning() << "Failed to parse RINEX NAV:" << tag << path;
      dst.reset();
      return false;
   }

   qInfo().noquote()
      << QString("[DataManager][RINEX_LOAD] sys=%1 epochs=%2")
      .arg(tag)
      .arg(rinexFile->navRecords.size());

   dst = std::move(rinexFile);
   return true;
}

const rinex::RINEX_FILE*DataManager::getRinexGlonassFile() const noexcept {
   return rinexGlonassFile_.get();
}

const rinex::RINEX_FILE*DataManager::getRinexGpsFile() const noexcept {
   return rinexGpsFile_.get();
}

// временная совместимость со старым кодом:
const rinex::RINEX_FILE*DataManager::getRinexFile() const noexcept {
   return rinexGlonassFile_.get();
}

bool DataManager::loadSBASCorrections(const QString& path, SourceType sourceType) {
   if (!fileExists(path)) {
      qWarning() << "SBAS corrections file not found:" << path;
      return false;
   }

   if (!sbasStore_.load(sourceType, path.trimmed())) {
      qWarning() << "Failed to load SBAS corrections:" << path;
      return false;
   }
   return true;
}

const SBASCorrectionStore &DataManager::getSBASStore() const noexcept {
   return sbasStore_;
}

SBASCorrectionStore &DataManager::sbasStore() noexcept{
   return sbasStore_;
}

bool DataManager::loadDCB(const QString& path) {
   if (!fileExists(path)) {
      qWarning() << "DCB file not found:" << path;
      glonassDcbL3L1_.clear();
      return false;
   }
   auto result = DCBParser::parseGlonassL3L1Bias(path.trimmed());

   if (result.empty()) {
      qWarning() << "Failed to parse DCB file:" << path;
      return false;
   }
   glonassDcbL3L1_ = std::move(result);

   return true;
}

std::optional<double> DataManager::getGlonassL3MinusL1Bias(const Satellite& satId) const noexcept {
   if (satId.getSystem() != SatelliteSystem::TYPE::GLONASS) {
      return std::nullopt;
   }
   auto it = glonassDcbL3L1_.find(satId);

   return it != glonassDcbL3L1_.end()
              ? std::make_optional(it->second)
              : std::nullopt;
}

bool DataManager::loadAntennaModel(const QString& antexPath,
                                   const QString& constGpsPath,
                                   const QString& constGloPath,
                                   const QString& constSvnMapPath) {
   antennaModel_ = std::make_unique<antex::SatelliteAntennaModel>();

   if (!antennaModel_->loadAntex(antexPath)) {
      qCritical() << "[DataManager] Не удалось загрузить ANTEX:" << antexPath;
      antennaModel_.reset();
      return false;
   }

   // Связь PRN → SVN и состояние НКА берутся из файлов состава ОГ Const_*.gps/.glo
   // (ранее — igs_satellite_metadata.snx)
   satmeta::SATELLITE_METADATA_FILE          metadata;
   ConstellationStatusReader::Statistics     constStats;

   if (!ConstellationStatusReader::read(constGpsPath, constGloPath, constSvnMapPath,
                                        metadata, &constStats)) {
      qCritical() << "[DataManager] Не удалось прочитать состав ОГ:"
                  << constGpsPath << "|" << constGloPath;
      antennaModel_.reset();
      return false;
   }

   if (!antennaModel_->loadMetadata(metadata)) {
      qCritical() << "[DataManager] Модель антенны отвергла метаданные состава ОГ";
      antennaModel_.reset();
      return false;
   }

   qInfo() << "[DataManager] ANTEX antenna model успешно загружена";

   return true;
}

const antex::SatelliteAntennaModel*DataManager::getAntennaModel() const noexcept{
   return antennaModel_.get();
}
