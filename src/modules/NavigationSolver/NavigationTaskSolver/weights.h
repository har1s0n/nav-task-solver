#ifndef WEIGHTS_H
#define WEIGHTS_H

#include <QVector>
#include <QDateTime>
#include <QtMath>

#include "geometry.h"
#include "modules/IO/sbascorrectionstore.h"

namespace navsolver {
class WeightModel final {
public:

   struct Config {
      double earthRadius_m     = 6378136.3;
      double ionoShellHeight_m = 350000.0;

      double sigmaTropoZenith_m = 0.12;

      double sigmaReceiver_m    = 0.72;
      double multipathA_m       = 0.13;
      double multipathB_m       = 0.53;
      double multipathScale_deg = 10.0;

      bool requireUdre = true;
      bool requireGive = false;

      double minSigma2_m2 = 1e-6;
   };

   struct SatWeight {
      int       geomIndex = -1;
      Satellite sat;

      double elevation_deg = qQNaN();
      double sigma2_m2     = qQNaN();

      double sigma2_udre_m2  = qQNaN();
      double sigma2_iono_m2  = qQNaN();
      double sigma2_tropo_m2 = qQNaN();
      double sigma2_air_m2   = qQNaN();
   };

   struct RejectStats {
      int badElevation = 0;
      int noAzimuth    = 0;
      int badIpp       = 0;
      int noUdre       = 0;
      int noIono       = 0;
      int badSigma2    = 0;
   };

   struct BuildResult {
      QVector<SatWeight> weights;
      QVector<Satellite> rejected;
      RejectStats        rej;
   };

   explicit WeightModel(const io::SBASCorrectionStore& store,
                        const Config&                  cfg);
   explicit WeightModel(const io::SBASCorrectionStore& store);

   BuildResult buildRDiagonal(const QDateTime&                             epoch,
                              const GRID_POINT&                            user,
                              const QVector<GeometryBuilder::SatGeometry>& geoms) const;

private:

   const io::SBASCorrectionStore& store_;
   Config cfg_;

   static inline QDateTime normalizeClockAsUtc(const QDateTime& t) {
      // Фиксируем timeSpec=UTC без сдвига “по часам”
      return QDateTime(t.date(), t.time(), Qt::UTC);
   }

   static inline double clamp(double v, double lo, double hi) noexcept {
      return (v < lo) ? lo : (v > hi ? hi : v);
   }

   static inline double wrapLonRad(double lon) noexcept {
      while (lon > M_PI) {
         lon -= 2.0 * M_PI;
      }

      while (lon <= -M_PI) {
         lon += 2.0 * M_PI;
      }
      return lon;
   }

   bool computeIppAndFpp(const GRID_POINT& user,
                         double            az_rad,
                         double            el_rad,
                         double&           ippLat_rad,
                         double&           ippLon_rad,
                         double&           fpp) const noexcept;

   static double tropoMapping(double el_rad) noexcept;
   double        sigmaMultipath(double el_deg) const noexcept;
};
} // namespace navsolver
#endif // WEIGHTS_H
