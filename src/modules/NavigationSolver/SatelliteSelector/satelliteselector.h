#ifndef SATELLITESELECTOR_H
#define SATELLITESELECTOR_H

#include "modules/IModule/imodule.h"

class ScopedTimer {
public:

   explicit ScopedTimer(const std::string& name)
      : name_(name),
      start_(std::chrono::high_resolution_clock::now()) {}

   ~ScopedTimer() {
      using namespace std::chrono;
      auto end  = high_resolution_clock::now();
      auto diff = duration_cast<milliseconds> (end - start_).count();
      std::cout << name_ << ": выполнено за "
                << diff << " мс\n";
   }

private:

   std::string startMessage_;
   std::string name_;
   std::chrono::high_resolution_clock::time_point start_;
};

namespace navsolver {
class SatelliteSelector : public pipeline::IModule {
public:

   SatelliteSelector() = default;

   /**
    * Основной метод: перебирает все записи SP3, фильтрует по allowedEpochs
    * и для каждой точки подсчитывает, какие спутники «выше» elevationMaskDeg.
    *
    * @param points           Список точек сетки (каждая – GRID_POINT)
    * @param allowedEpochs    Список временных меток (эпох), которые нас интересуют
    * @param elevationMaskDeg Минимальный угол возвышения (градусы), по умолчанию 5.0°
    *
    * @return visibleMap      Для каждой эпохи – карта «точка сетки → список спутников»
    */
   bool    execute(pipeline::Context& ctx) override;
   QString name() const override {
      return QStringLiteral("Отбор видимых спутников");
   }

private:

   VisibleSatellites selectVisibleSatellites(pipeline::Context& ctx) const;
};
}

#endif // SATELLITESELECTOR_H
