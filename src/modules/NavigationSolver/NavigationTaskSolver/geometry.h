#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <QVector>
#include <QtMath>

#include <inav/Coordinates>
#include <inav/Satellite>

namespace navsolver {
// Геометрия "точка -> НКА" для построения матрицы H, DOP и метрик.
class GeometryBuilder final {
public:

   struct Config {
      double minRange = 1e-12;

      // ВАЖНО: для расчета ионосферы (IPP) в WeightModel азимут обязателен (true).
      bool computeAzimuth = true;
   };

   struct SatInput {
      Satellite sat;
      COORD_XYZ satEcef;
   };

   struct SatGeometry {
      Satellite sat;

      double    range = qQNaN(); // Дальность |sat - obs|
      COORD_XYZ los   = {};      // Единичный вектор (obs -> sat)

      double elevation_deg = qQNaN();
      double azimuth_deg   = qQNaN();
      double elevation_rad = qQNaN();
      double azimuth_rad   = qQNaN();
   };

   explicit GeometryBuilder(const Config& cfg) : cfg_(cfg) {}

   GeometryBuilder() : cfg_() {}

   // Возвращает массив геометрий только для НКА, прошедших фильтрацию
   [[nodiscard]] QVector<SatGeometry> compute(const GRID_POINT&        user,
                                              const QVector<SatInput>& sats) const;

   struct RotEcefToEnu {
      double t[3][3];
   };

   [[nodiscard]] static RotEcefToEnu rotationEcefToEnu(const COORD_LLH& llh) noexcept;
   [[nodiscard]] static COORD_ENU    apply(const RotEcefToEnu& R,
                                           const COORD_XYZ&    v) noexcept;

private:

   [[nodiscard]] static bool computeLosRange(const COORD_XYZ& obs,
                                             const COORD_XYZ& sat,
                                             double           minRange,
                                             COORD_XYZ&       losUnit,
                                             double&          range) noexcept;
   [[nodiscard]] static bool computeAzElFromEnu(const COORD_ENU& enu,
                                                double           range,
                                                bool             computeAzimuth,
                                                double&          az_rad,
                                                double&          el_rad,
                                                double&          az_deg,
                                                double&          el_deg) noexcept;

private:

   Config cfg_;
};
} // namespace navsolver
#endif // GEOMETRY_H
