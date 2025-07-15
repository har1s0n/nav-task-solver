#include "errorcalculator.h"
#include "modules/IO/datamanager.h"

bool navsolver::ErrorCalculator::execute(pipeline::Context& ctx) {
   if (!ctx.dm || ctx.gridPoints.isEmpty() || ctx.visibleSats.isEmpty()) {
      qWarning() << "[ErrorCalculator] Недостаточно данных для расчета.";
      return false;
   }
   computeResidualErrors(ctx);
   return true;
}

void   navsolver::ErrorCalculator::computeResidualErrors(pipeline::Context& ctx) {
}

double navsolver::ErrorCalculator::correctSp3ClockL3toL1(pipeline::Context& ctx,
                                                         const Satellite&   sat,
                                                         const QDateTime&   epoch,
                                                         double             clockL3) const {
   if (sat.getSystem() != SatelliteSystem::TYPE::GLONASS) {
      return clockL3;
   }

   auto dcb_ns = ctx.dm->getGlonassL3MinusL1Bias(sat);

   if (!dcb_ns.has_value()) {
      qWarning() << "[ErrorCalculator] Нет DCB для" << sat.toString() << "в эпоху" << epoch;
      return clockL3;
   }

   return clockL3 + dcb_ns.value() * 1e-9;
}
