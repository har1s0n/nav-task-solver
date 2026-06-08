#ifndef IMODULE_H
#define IMODULE_H

#include <QString>

#include <inav/Coordinates>

namespace io   { class DataManager; }
namespace rinex { struct RINEX_FILE; }

using VisibleSatellites = QMap<QDateTime, QMap<GRID_POINT, QVector<Satellite> > >;

struct ResidualError {
   Satellite satellite;       ///< НКА
   double    deltaSp3  = 0.0; ///< ошибка SP3 - NAV
   double    deltaSdcm = 0.0; ///< ошибка SP3 - NAV+SDCM
   double    residual  = 0.0; ///< остаточная ошибка СДКМ
   COORD_XYZ satEcef   = {};
};

struct DOP {
   double PDOP = qQNaN();
   double HDOP = qQNaN();
   double VDOP = qQNaN();
   double GDOP = qQNaN();
};

struct NavSolution {
   COORD_XYZ delta_pos_ecef{ 0.0, 0.0, 0.0 }; // δX,δY,δZ в той же линейной шкале, что и H/y
   double    delta_clk_s = qQNaN();           // δt (сек), если cδt в метрах

   DOP    dop{};
   int    num_sats    = 0;
   double postfit_rms = qQNaN();
   bool   converged   = false;

   // требуемые метрики (по δpos)
   double err3d     = qQNaN();
   double horiz_err = qQNaN();
   double vert_err  = qQNaN();
};

namespace pipeline {
struct Context {
   io::DataManager*                                            dm = nullptr;
   QVector<GRID_POINT>                                         gridPoints;
   QVector<QDateTime>                                          allowedEpochs;
   VisibleSatellites                                           visibleSats;
   QMap<QDateTime, QMap<GRID_POINT, QVector<ResidualError> > > residualErrors;
   QMap<QDateTime, QMap<GRID_POINT, NavSolution> >             solutions;
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
