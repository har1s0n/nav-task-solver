#ifndef IMODULE_H
#define IMODULE_H

#include <QString>
#include <memory>

#include <inav/Coordinates>

namespace io   { class DataManager; }
namespace rinex { class RINEX_FILE; }

using VisibleSatellites = QMap<QDateTime, QMap<GRID_POINT, QVector<Satellite> > >;

struct ResidualError {
   Satellite satellite;       ///< НКА
   double    deltaSp3  = 0.0; ///< ошибка SP3 - NAV
   double    deltaSdcm = 0.0; ///< ошибка SP3 - NAV+SDCM
   double    residual  = 0.0; ///< остаточная ошибка СДКМ
};

namespace pipeline {
struct Context {
   io::DataManager*                                            dm      = nullptr;
   const rinex::RINEX_FILE*                                    navOrig = nullptr;
   std::unique_ptr<rinex::RINEX_FILE>                          navCorrected;
   QVector<GRID_POINT>                                         gridPoints;
   QVector<QDateTime>                                          allowedEpochs;
   VisibleSatellites                                           visibleSats;
   QMap<QDateTime, QMap<GRID_POINT, QVector<ResidualError> > > residualErrors;
};

class IModule {
public:

   virtual ~IModule() = default;

   /**
    * Выполнить свою логику, заполнять/корректировать Context.
    * @return true — успешно, false — остановить пайплайн.
    */
   virtual bool    execute(Context& ctx) = 0;
   virtual QString name() const          = 0;
};
} // namespace pipeline

#endif // IMODULE_H
