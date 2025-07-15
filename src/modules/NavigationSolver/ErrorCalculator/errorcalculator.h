#ifndef ERRORCALCULATOR_H
#define ERRORCALCULATOR_H

#include "modules/IModule/imodule.h"

namespace navsolver {
class ErrorCalculator : public pipeline::IModule {
public:

   ErrorCalculator();

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
};
}

#endif // ERRORCALCULATOR_H
