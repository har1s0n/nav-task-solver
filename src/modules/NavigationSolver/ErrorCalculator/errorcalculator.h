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

   double correctSp3ClockL3toL1(pipeline::Context& ctx,
                                const Satellite&   sat,
                                const QDateTime&   epoch,
                                double             clockL3,
                                qint64*            dcbMissingCounter) const;

   double computeBroadcastClockL1_GLO(const  rinex::R_TIME& tr,
                                      const QDateTime&      epoch);

   double computeBroadcastClockL1_GPS(const rinex::NAV_RECORD& navRec,
                                      const QDateTime&         epoch);

   double computeBroadcastClockL1(const Satellite&         sat,
                                  const rinex::NAV_RECORD& navRec,
                                  const QDateTime&         epoch);
};
}

#endif // ERRORCALCULATOR_H
