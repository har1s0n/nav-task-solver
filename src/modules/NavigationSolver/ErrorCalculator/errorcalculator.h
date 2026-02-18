#ifndef ERRORCALCULATOR_H
#define ERRORCALCULATOR_H

#include "modules/IModule/imodule.h"
#include "src/RINEX/rinex.h"

namespace navsolver {
class ErrorCalculator : public pipeline::IModule {
public:

   ErrorCalculator() = default;

   bool    execute(pipeline::Context& ctx) override;
   QString name() const override {
      return QStringLiteral("Расчёт ошибок в проекции на LOS");
   }

private:

   void   computeResidualErrors(pipeline::Context& ctx);
   double correctSp3ClockL3toL1(pipeline::Context& ctx,
                                const Satellite&   sat,
                                const QDateTime&   epoch,
                                double             clockL3) const;

   double computeBroadcastClockL1_GLO(const  rinex::R_TIME& tr,
                                      const QDateTime&      epoch);

   double computeBroadcastClockL1(const Satellite&      sat,
                                  const  rinex::R_TIME& tr,
                                  const QDateTime&      epoch);

   // Удобная обёртка для вычитания времени (секунды от начала недели)
   inline double secondsOfWeek(const QDateTime& tUtc) {
      static const QDateTime gpsEpoch(QDate(1980, 1, 6), QTime(0, 0, 0), Qt::UTC);
      const qint64 secs = gpsEpoch.secsTo(tUtc.toUTC());
      const qint64 mod  = ((secs % 604800) + 604800) % 604800;

      return static_cast<double> (mod);
   }

   inline double wrapWeek(double dt) {
      // аккуратный анти-ролловер ±302400 c (половина недели)
      const double HALF_WEEK = 302400.0;

      if (dt >  HALF_WEEK) {
         dt -= 2.0 * HALF_WEEK;
      }

      if (dt < -HALF_WEEK) {
         dt += 2.0 * HALF_WEEK;
      }
      return dt;
   }
};
}

#endif // ERRORCALCULATOR_H
