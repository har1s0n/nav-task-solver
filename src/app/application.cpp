#include "application.h"
#include "modules/NavigationSolver/ErrorCalculator/errorcalculator.h"
#include "modules/NavigationSolver/GridGenerator/gridgenerator.h"
#include "modules/NavigationSolver/SatelliteSelector/satelliteselector.h"
#include "modules/NavigationSolver/NavigationTaskSolver/navigationtasksolver.h"

#include <QJsonArray>
#include <QJsonObject>


bool Application::initialize(const ApplicationConfig& config) {
   cfg_ = config;

   // 1) Модуль загрузки всех IO-данных
   io::DataManager::Config ioCfg{
      cfg_.sp3Path,
      cfg_.rinexNavGlonassPath,
      cfg_.rinexNavGpsPath,
      cfg_.sbasPath,
      io::SourceType::FILE_CSV,
      cfg_.dcbPath,
      cfg_.antexPath,
      cfg_.constGpsPath,
      cfg_.constGloPath,
      cfg_.constSvnMapPath
   };

   navsolver::NavigationTaskSolver::Config navSolverCfg{};
   navSolverCfg.weightCfg.sigmaReceiver_m = 0.75;
   navSolverCfg.weightCfg.multipathA_m    = 0.0;
   navSolverCfg.weightCfg.multipathB_m    = 0.0;
   navSolverCfg.weightCfg.requireGive     = false;

   modules_.clear();

   modules_.push_back(std::make_unique<io::DataManager> (ioCfg));

   // 2) Генерация сетки
   modules_.push_back(std::make_unique<navsolver::GridGenerator>());

   // 3) Отбор видимых спутников (по allowedEpochs)
   modules_.push_back(std::make_unique<navsolver::SatelliteSelector>());

   // 4) Расчёт Δρ_abs (deltaSp3) и Δρ_sbas (deltaSdcm)
   modules_.push_back(std::make_unique<navsolver::ErrorCalculator>());

   // 5) Решение навигационной задачи и метрики
   modules_.push_back(std::make_unique<navsolver::NavigationTaskSolver> (navSolverCfg));

   return true;
}

int Application::run() {
   bool epochsPrepared = false;

   for (auto& module : modules_) {
      qDebug() << "=== Запускаем модуль:" << module->name();

      if (!module->execute(ctx_)) {
         qWarning() << "Модуль" << module->name() << "завершился с ошибкой";
         return -1;
      }

      if (!epochsPrepared) {
         const auto* sp3Ptr = (ctx_.dm ? ctx_.dm->getSP3File() : nullptr);
         const auto* navGlo = (ctx_.dm ? ctx_.dm->getRinexGlonassFile() : nullptr);
         const auto* navGps = (ctx_.dm ? ctx_.dm->getRinexGpsFile() : nullptr);

         if (sp3Ptr) {
            const auto epochsSp3 = extractEpochs(sp3Ptr);
            const auto epochsGlo = extractEpochs(navGlo);
            const auto epochsGps = extractEpochs(navGps);

            QVector<QDateTime> epochsFiltered;
            epochsFiltered.reserve(epochsSp3.size());

            if (!epochsSp3.isEmpty()) {
               const QDate baseDate = epochsSp3.first().date();

               for (const auto& t : epochsSp3) {
                  if (t.date() == baseDate) {
                     epochsFiltered.push_back(t);
                  }
               }
            }

            ctx_.allowedEpochs = epochsFiltered;
            epochsPrepared     = true;

            qDebug() << "[Application] epochs prepared:"
                     << "sp3="    << epochsSp3.size()
                     << "navGlo=" << epochsGlo.size()
                     << "navGps=" << epochsGps.size()
                     << "allowed=" << ctx_.allowedEpochs.size();
         }
      }

      qDebug() << "=== Завершение работы модуля:" << module->name();
   }

   qDebug() << "[Application] Пайплайн завершён";
   return 0;
}

QJsonObject Application::getResultsAsJson() const {
   QJsonObject res;

   // 1. Базовая статистика
   res.insert("processed_epochs_count", ctx_.allowedEpochs.size());
   res.insert("grid_points_count",      ctx_.gridPoints.size());

   // 2. Сериализация навигационных решений
   QJsonArray epochsArray;

   // Проходим по всем эпохам, для которых есть решения
   for (auto epochIt = ctx_.solutions.constBegin(); epochIt != ctx_.solutions.constEnd(); ++epochIt) {
      QJsonObject epochObj;
      // Сохраняем время в стандартном ISO 8601 формате
      epochObj.insert("time", epochIt.key().toString(Qt::ISODate));

      QJsonArray  pointsArray;
      const auto& pointsMap = epochIt.value();

      for (auto pointIt = pointsMap.constBegin(); pointIt != pointsMap.constEnd(); ++pointIt) {
         const auto& gridPoint = pointIt.key();
         const auto& navSol    = pointIt.value();

         QJsonObject pointObj;

         // координаты виртуальной точки наблюдления
         pointObj.insert("lat",       gridPoint.llh.latitude);
         pointObj.insert("lon",       gridPoint.llh.longitude);
         pointObj.insert("height",    gridPoint.llh.height);

         pointObj.insert("converged", navSol.converged);
         pointObj.insert("num_sats",  navSol.num_sats);

         // Если решение сошлось, добавляем метрики
         if (navSol.converged) {
            pointObj.insert("err3d",       navSol.err3d);
            pointObj.insert("horiz_err",   navSol.horiz_err);
            pointObj.insert("vert_err",    navSol.vert_err);
            pointObj.insert("postfit_rms", navSol.postfit_rms);
            pointObj.insert("delta_clk_s", navSol.delta_clk_s);

            // Вложенный объект DOP
            QJsonObject dopObj;
            dopObj.insert("pdop", navSol.dop.PDOP);
            dopObj.insert("hdop", navSol.dop.HDOP);
            dopObj.insert("vdop", navSol.dop.VDOP);
            dopObj.insert("gdop", navSol.dop.GDOP);
            pointObj.insert("dop", dopObj);

            // Вложенный объект отклонения позиции
            QJsonObject deltaPosObj;
            deltaPosObj.insert("x", navSol.delta_pos_ecef.x);
            deltaPosObj.insert("y", navSol.delta_pos_ecef.y);
            deltaPosObj.insert("z", navSol.delta_pos_ecef.z);
            pointObj.insert("delta_pos_ecef", deltaPosObj);
         }

         pointsArray.append(pointObj);
      }

      epochObj.insert("points", pointsArray);
      epochsArray.append(epochObj);
   }

   res.insert("solutions", epochsArray);

   // Примечание: Массив ctx_.residualErrors здесь не сериализуется,
   // чтобы не раздувать JSON до гигантских размеров. Если ошибки по каждому
   // спутнику критичны для выдачи в ПОЭХ, нужно добавить аналогичный цикл

   return res;
}

QVector<QDateTime> Application::extractEpochs(const sp3::SP3_FILE* f) noexcept {
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

QVector<QDateTime> Application::extractEpochs(const rinex::RINEX_FILE* f) noexcept {
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

QVector<QDateTime> Application::intersectEpochs(const QVector<QDateTime>& a,
                                                const QVector<QDateTime>& b) noexcept {
   QSet<QDateTime> setB;

   setB.reserve(b.size());

   for (const auto& t : b) {
      setB.insert(t);
   }

   QVector<QDateTime> out;
   out.reserve(std::min(a.size(), b.size()));

   for (const auto& t : a) {
      if (setB.contains(t)) {
         out.push_back(t);
      }
   }
   return out;
}
