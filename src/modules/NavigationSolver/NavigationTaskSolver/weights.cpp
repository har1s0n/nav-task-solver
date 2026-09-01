#include "weights.h"

namespace navsolver {
WeightModel::WeightModel(const io::SBASCorrectionStore& store, const Config& cfg)
   : store_(store), cfg_(cfg) {}

WeightModel::WeightModel(const io::SBASCorrectionStore& store)
   : WeightModel(store, Config{}) {}

bool WeightModel::computeIppAndFpp(const GRID_POINT& user, double az_rad, double el_rad, double& ippLat_rad, double& ippLon_rad,
                                   double& fpp) const noexcept {
   const double lat_u = qDegreesToRadians(user.llh.latitude);
   const double lon_u = qDegreesToRadians(user.llh.longitude);
   const double Re    = cfg_.earthRadius_m;
   const double hI    = cfg_.ionoShellHeight_m;
   const double denom = Re + hI;

   const double cosEl  = qCos(el_rad);
   const double arg    = clamp((Re / denom) * cosEl, -1.0, 1.0);
   const double chi    = qAsin(arg);
   const double psi    = M_PI_2 - el_rad - chi;
   const double inside = qMax(1.0 - arg * arg, 1e-15);

   fpp = 1.0 / qSqrt(inside);

   const double sinLatU = qSin(lat_u), cosLatU = qCos(lat_u);
   const double sinPsi  = qSin(psi), cosPsi  = qCos(psi);

   const double sinLatPp = sinLatU * cosPsi + cosLatU * sinPsi * qCos(az_rad);

   ippLat_rad = qAsin(clamp(sinLatPp, -1.0, 1.0));
   const double y = sinPsi * qSin(az_rad);
   const double x = cosLatU * cosPsi - sinLatU * sinPsi * qCos(az_rad);
   ippLon_rad = wrapLonRad(lon_u + qAtan2(y, x));

   return qIsFinite(ippLat_rad) && qIsFinite(ippLon_rad) && qIsFinite(fpp);
}

double WeightModel::tropoMapping(double el_rad) noexcept {
   const double s     = qSin(el_rad);
   const double denom = qSqrt(0.002001 + s * s);

   return 1.001 / denom;
}

double WeightModel::sigmaMultipath(double el_deg) const noexcept {
   return cfg_.multipathA_m + cfg_.multipathB_m * qExp(-el_deg / cfg_.multipathScale_deg);
}

WeightModel::BuildResult WeightModel::buildRDiagonal(const QDateTime& epoch, const GRID_POINT& user,
                                                     const QVector<GeometryBuilder::SatGeometry>& geoms) const {
   BuildResult out;

   out.weights.reserve(geoms.size());
   out.rejected.reserve(geoms.size());

   // qDebug().noquote() << QString("=== Запуск WeightModel | Эпоха: %1 | НКА всего: %2 ===")
   //    .arg(epoch.toString(Qt::ISODate))
   //    .arg(geoms.size());

   for (int i = 0; i < geoms.size(); ++i) {
      const auto& g = geoms[i];

      // QString satName = QString("%1%2").arg(g.sat.getSystem() ==
      //                                       SatelliteSystem::TYPE::GPS ? "G" : (g.sat.getSystem() ==
      //                                                                           SatelliteSystem::TYPE::GLONASS ? "R" :
      //                                                                           "S")).arg(g.sat.getNumber());

      if (!qIsFinite(g.elevation_rad) || !qIsFinite(g.elevation_deg)) {
         out.rej.badElevation += 1; out.rejected.push_back(g.sat);

         // qDebug().noquote() << QString("  [REJECT] %1 | Причина: Некорректный угол места").arg(satName);

         continue;
      }

      if (!qIsFinite(g.azimuth_rad) || !qIsFinite(g.azimuth_deg)) {
         out.rej.noAzimuth += 1; out.rejected.push_back(g.sat);

         // qDebug().noquote() << QString("  [REJECT] %1 | Причина: Некорректный азимут").arg(satName);

         continue;
      }

      // --- UDRE ---
      const auto udreOpt     = store_.udreSigmaEff_m2(g.sat, epoch);
      double     sigma2_udre = 0.0;

      if (cfg_.requireUdre) {
         if (!udreOpt.has_value() || !qIsFinite(*udreOpt) || (*udreOpt <= 0.0)) {
            out.rej.noUdre += 1; out.rejected.push_back(g.sat);

            // qDebug().noquote() << QString("  [REJECT] %1 | Причина: Нет UDRE или udreSigmaEff_m2 <= 0").arg(satName);

            continue;
         }
         sigma2_udre = *udreOpt;
      } else if (udreOpt.has_value() && qIsFinite(*udreOpt) && (*udreOpt > 0.0)) {
         sigma2_udre = *udreOpt;
      }

      // --- IONO ---
      double sigma2_iono = 0.0;
      double ippLat_deg = qQNaN(), ippLon_deg = qQNaN(), fpp = qQNaN();
      const bool okIpp = computeIppAndFpp(user, g.azimuth_rad, g.elevation_rad, ippLat_deg, ippLon_deg, fpp);

      if (!okIpp) {
         if (cfg_.requireGive) {
            out.rej.badIpp += 1; out.rejected.push_back(g.sat);

            // qDebug().noquote() << QString("  [REJECT] %1 | Причина: Ошибка расчета точки протыкания ионосферы (IPP)").arg(satName);

            continue;
         }
      } else {
         double vdelay_m = 0.0, var_v_m2 = qQNaN();
         const bool okIono = store_.ionoVerticalAt(ippLat_deg, ippLon_deg, epoch, vdelay_m, var_v_m2);

         if (cfg_.requireGive) {
            if (!okIono || !qIsFinite(var_v_m2) || (var_v_m2 < 0.0) || !qIsFinite(fpp) || (fpp <= 0.0)) {
               out.rej.noIono += 1; out.rejected.push_back(g.sat);

               // qDebug().noquote() <<
               //    QString("  [REJECT] %1 | Причина: Нет данных ионосферы (GIVE) для IPP [%2, %3]").arg(satName).arg(ippLat_deg).arg(
               //    ippLon_deg);

               continue;
            }
            sigma2_iono = (fpp * fpp) * var_v_m2;
         } else if (okIono && qIsFinite(var_v_m2) && (var_v_m2 >= 0.0) && qIsFinite(fpp) && (fpp > 0.0)) {
            sigma2_iono = (fpp * fpp) * var_v_m2;
         } else if (qIsFinite(fpp) && (fpp > 0.0)) {
            //   sigma2_iono = F_pp^2 * sigma2_VIVE_fallback
            // F_pp^2 (велик на малых углах) даёт корректное угловое подавление.
            constexpr double kSigma2VIVEFallback_m2 = 25.0; // (5 м)^2, калибруется по [NTS][DUAL]
            sigma2_iono = (fpp * fpp) * kSigma2VIVEFallback_m2;
         }
      }

      // --- TROPO & AIR ---
      const double sigma2_tropo = qPow(cfg_.sigmaTropoZenith_m * tropoMapping(g.elevation_rad), 2);
      const double sigma2_air   = qPow(cfg_.sigmaReceiver_m, 2) + qPow(sigmaMultipath(g.elevation_deg), 2);

      // --- Total ---
      double sigma2 = sigma2_udre + sigma2_air; // Базовая дисперсия эфемерид и шума приемника

      if (!cfg_.assessSISREonly) {
         sigma2 += (sigma2_iono + sigma2_tropo);
      }

      if (!qIsFinite(sigma2) || (sigma2 <= 0.0)) {
         out.rej.badSigma2 += 1; out.rejected.push_back(g.sat);

         // qDebug().noquote() << QString("  [REJECT] %1 | Причина: Итоговая дисперсия (sigma2) некорректна: %2").arg(satName).arg(sigma2);

         continue;
      }

      SatWeight w;
      w.geomIndex       = i; w.sat = g.sat; w.elevation_deg = g.elevation_deg;
      w.sigma2_m2       = qMax(sigma2, cfg_.minSigma2_m2);
      w.sigma2_udre_m2  = sigma2_udre; w.sigma2_iono_m2 = sigma2_iono;
      w.sigma2_tropo_m2 = sigma2_tropo; w.sigma2_air_m2 = sigma2_air;
      out.weights.push_back(w);

      qDebug().noquote() << QString("  [ACCEPT] %1 | el:%2 sigma2_udre:%3 sigma2_total:%4")
         .arg(g.sat.toString(), -4)
         .arg(w.elevation_deg,  0, 'f', 1) // угол места
         .arg(w.sigma2_udre_m2, 0, 'f', 4) // σ_UDRE = sqrt(этого)
         .arg(w.sigma2_m2,      0, 'f', 4);

      // qDebug().noquote() << QString("  [ACCEPT] %1 | Вес сформирован. sigma2_total: %2 | udre: %3, iono: %4, tropo: %5, air: %6")
      //    .arg(satName,           -4)
      //    .arg(w.sigma2_m2,       0, 'f', 4)
      //    .arg(w.sigma2_udre_m2,  0, 'f', 4)
      //    .arg(w.sigma2_iono_m2,  0, 'f', 4)
      //    .arg(w.sigma2_tropo_m2, 0, 'f', 4)
      //    .arg(w.sigma2_air_m2,   0, 'f', 4);
   }

   // qDebug().noquote() <<
   //    QString(
   //    "=== Итоги эпохи | Принято: %1 | Отклонено: %2 (noUdre: %3, noIono: %4, badIpp: %5, badElev: %6, noAzim: %7, badSig2: %8) ===\n")
   //    .arg(out.weights.size())
   //    .arg(out.rejected.size())
   //    .arg(out.rej.noUdre)
   //    .arg(out.rej.noIono)
   //    .arg(out.rej.badIpp)
   //    .arg(out.rej.badElevation)
   //    .arg(out.rej.noAzimuth)
   //    .arg(out.rej.badSigma2);

   return out;
}
} // namespace navsolver
