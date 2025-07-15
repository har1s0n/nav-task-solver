#include "application.h"
#include "modules/CorrectionApplier/correctionapplier.h"
#include "modules/NavigationSolver/GridGenerator/gridgenerator.h"
#include "modules/NavigationSolver/SatelliteSelector/satelliteselector.h"


bool Application::initialize(const ApplicationConfig& config) {
   cfg_ = config;

   // 1) Модуль загрузки всех IO-данных
   io::DataManager::Config ioCfg{
      cfg_.sp3Path,
      cfg_.rinexNavPath,
      cfg_.sbasPath,
      io::SourceType::FILE_CSV,
      cfg_.dcbPath
   };
   modules_.push_back(std::make_unique<io::DataManager> (ioCfg));

   // Шаг 2: применение SBAS-поправок (если включено)
   if (cfg_.applyCorrections) {
      modules_.push_back(std::make_unique<corrections::CorrectionApplier>());
   }

   // Шаг 3: модуль генерации сетки
   modules_.push_back(std::make_unique<navsolver::GridGenerator>());

   // Шаг 4: модуль отбора видимых спутников
   modules_.push_back(std::make_unique<navsolver::SatelliteSelector>());

   return true;
}

int Application::run() {
   pipeline::Context ctx;

   for (auto& module : modules_) {
      qDebug() << "=== Запускаем модуль:" << module->name();

      if (!module->execute(ctx)) {
         qWarning() << "Модуль" << module->name() << "завершился с ошибкой";
         return -1;
      }

      // После генерации сетки и перед SatelliteSelector заполняем ctx.epochs
      if (module->name() == QStringLiteral("Генерация сетки")) {
         // Берём RINEX из ctx: скорректированный (если есть) или оригинал
         const auto navPtr =
            ctx.navCorrected ? ctx.navCorrected.get() : ctx.navOrig;

         if (!navPtr) {
            qWarning() << "[Application] NAV-файл отсутствует";
            return -1;
         }
         ctx.allowedEpochs = extractEpochs(navPtr);

         if (ctx.allowedEpochs.isEmpty()) {
            qDebug() << "[Application] Нет эпох для фильтрации (RINEX пуст)";
         } else {
            qDebug() << "[Application] Извлечено эпох для фильтрации:" << ctx.allowedEpochs.size();
         }
      }
   }

   qDebug() << "[Application] Пайплайн завершён.";
   qDebug() << "[Application] Общее число видимых спутников:" << ctx.visibleSats.size();

   // qDebug() << "Пайплайн завершён, найдено спутников:" << ctx.visibleSats.size();
   // return 0;
   // 1. Генерация сетки
   // navsolver::GridGenerator gridGen;
   // auto gridPoints = gridGen.generateGrid();

   // 2. Извлечение эпох из скорректированного RINEX
   // auto epochs = extractEpochs(corrected_.get());

   // 3. Определение видимых спутников
   // navsolver::SatelliteSelector selector(dataManager_);
   // auto visibleSats = selector.selectVisibleSatellites(gridPoints, epochs, 5.0);

   // 4. [Следующий шаг] Расчёт проекционных ошибок в ErrorCalculator
   // navsolver::ErrorCalculator calc;
   // auto residuals = calc.computeResidualErrors(dataManager_, correctedRinex_, gridPoints, visibleSats);


   return 0;
}

QVector<QDateTime> Application::extractEpochs(const sp3::SP3_FILE* f) noexcept{
   QVector<QDateTime> res;

   if (!f) {
      return res;
   }
   res.reserve(f->records.size());

   for (auto it = f->records.constBegin(); it != f->records.constEnd(); ++it) {
      res.push_back(it.key());
   }
   return res;
}

QVector<QDateTime> Application::extractEpochs(const rinex::RINEX_FILE* f) noexcept{
   QVector<QDateTime> res;

   if (!f) {
      return res;
   }
   res.reserve(f->navRecords.size());

   for (auto it = f->navRecords.constBegin(); it != f->navRecords.constEnd(); ++it) {
      res.push_back(it.key());
   }
   return res;
}
