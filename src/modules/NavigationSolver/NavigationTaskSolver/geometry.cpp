#include "geometry.h"

namespace navsolver {
QVector<GeometryBuilder::SatGeometry> GeometryBuilder::compute(const GRID_POINT&        user,
                                                               const QVector<SatInput>& sats) const {
   QVector<SatGeometry> result;

   result.reserve(sats.size());

   const RotEcefToEnu R = rotationEcefToEnu(user.llh);

   for (const auto& s : sats) {
      COORD_XYZ losUnit{};
      double    range = qQNaN();

      if (!computeLosRange(user.ecef, s.satEcef, cfg_.minRange, losUnit, range)) {
         continue;
      }

      // Вектор разности (dx, dy, dz).
      const COORD_XYZ d{
         s.satEcef.x - user.ecef.x,
         s.satEcef.y - user.ecef.y,
         s.satEcef.z - user.ecef.z
      };

      const COORD_ENU enu = apply(R, d);

      double az_rad = qQNaN();
      double el_rad = qQNaN();
      double az_deg = qQNaN();
      double el_deg = qQNaN();

      if (!computeAzElFromEnu(enu, range, cfg_.computeAzimuth,
                              az_rad, el_rad, az_deg, el_deg)) {
         continue;
      }

      SatGeometry g;
      g.sat           = s.sat;
      g.range         = range;
      g.los           = losUnit;
      g.elevation_deg = el_deg;
      g.azimuth_deg   = az_deg;
      g.elevation_rad = el_rad;
      g.azimuth_rad   = az_rad;

      result.push_back(g);
   }

   return result;
}

GeometryBuilder::RotEcefToEnu GeometryBuilder::rotationEcefToEnu(const COORD_LLH& llh) noexcept {
   const double lat = qDegreesToRadians(llh.latitude);
   const double lon = qDegreesToRadians(llh.longitude);

   const double sinLat = qSin(lat), cosLat = qCos(lat);
   const double sinLon = qSin(lon), cosLon = qCos(lon);

   RotEcefToEnu R{};

   R.t[0][0] = -sinLon;           R.t[0][1] =  cosLon;           R.t[0][2] = 0.0;
   R.t[1][0] = -sinLat * cosLon;  R.t[1][1] = -sinLat * sinLon;  R.t[1][2] = cosLat;
   R.t[2][0] =  cosLat * cosLon;  R.t[2][1] =  cosLat * sinLon;  R.t[2][2] = sinLat;

   return R;
}

COORD_ENU GeometryBuilder::apply(const RotEcefToEnu& R, const COORD_XYZ& v) noexcept {
   COORD_ENU enu;

   enu.east  = R.t[0][0] * v.x + R.t[0][1] * v.y + R.t[0][2] * v.z;
   enu.north = R.t[1][0] * v.x + R.t[1][1] * v.y + R.t[1][2] * v.z;
   enu.up    = R.t[2][0] * v.x + R.t[2][1] * v.y + R.t[2][2] * v.z;
   return enu;
}

bool GeometryBuilder::computeLosRange(const COORD_XYZ& obs,
                                      const COORD_XYZ& sat,
                                      double           minRange,
                                      COORD_XYZ&       losUnit,
                                      double&          range) noexcept {
   const double dx = sat.x - obs.x;
   const double dy = sat.y - obs.y;
   const double dz = sat.z - obs.z;

   const double r2 = dx * dx + dy * dy + dz * dz;

   if (!(r2 > 0.0) || !qIsFinite(r2)) {
      return false;
   }

   range = qSqrt(r2);

   if (!(range > minRange) || !qIsFinite(range)) {
      return false;
   }

   losUnit.x = dx / range;
   losUnit.y = dy / range;
   losUnit.z = dz / range;

   return qIsFinite(losUnit.x) && qIsFinite(losUnit.y) && qIsFinite(losUnit.z);
}

bool GeometryBuilder::computeAzElFromEnu(const COORD_ENU& enu,
                                         double           range,
                                         bool             computeAzimuth,
                                         double&          az_rad,
                                         double&          el_rad,
                                         double&          az_deg,
                                         double&          el_deg) noexcept {
   const double e = enu.east;
   const double n = enu.north;
   const double u = enu.up;

   if (!(range > 0.0) || !qIsFinite(range)) {
      return false;
   }

   // Физически horiz2 всегда положительный, но защита от погрешностей FPU необходима
   double horiz2 = range * range - u * u;

   if (horiz2 < 0.0) {
      horiz2 = 0.0;
   }
   const double horiz = qSqrt(horiz2);

   el_rad = qAtan2(u, horiz);
   el_deg = qRadiansToDegrees(el_rad);

   if (computeAzimuth) {
      az_rad = qAtan2(e, n); // Диапазон [-pi; pi]

      if (az_rad < 0.0) {
         az_rad += 2.0 * M_PI;
      }
      az_deg = qRadiansToDegrees(az_rad);
   } else {
      az_rad = qQNaN();
      az_deg = qQNaN();
   }

   return qIsFinite(el_deg) && qIsFinite(el_rad);
}
} // namespace navsolver
