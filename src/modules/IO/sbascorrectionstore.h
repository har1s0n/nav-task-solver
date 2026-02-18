#ifndef SBASCORRECTIONSTORE_H
#define SBASCORRECTIONSTORE_H

#include <QString>
#include <QMap>
#include <QHash>
#include <QVector>
#include <QDateTime>
#include <optional>

#include <inav/SBAS>

namespace io {
// Источник для загрузки SBAS
enum class SourceType { FILE_HEX, FILE_CSV, DATABASE };

// ---------------- PRN MASK интервал ----------------
struct PrnMaskInterval {
   QDateTime start;
   QDateTime end; // [start,end)
   // соответствие локального индекса из маски (1..210) → Satellite
   QHash<int, Satellite> mapLocalIdxToSat;
};

// ---------------- UDRE/FAST/DEG ----------------
struct UdreEntry {
   QDateTime          start, end; // [start,end)
   int                udreIndex { -1 };
   double             bound_m   { qQNaN() };
   double             sigma_m2  { qQNaN() };
   int                iodf { -1 };
   int                iodp { -1 };
   sbas::MESSAGE_TYPE source { sbas::MESSAGE_TYPE::ZERO_MESSAGE };
};

struct FastEntry {
   QDateTime start, end;
   int       iodf{ -1 };
   int       iodp{ -1 };
};
struct DegrParams { QDateTime start, end; double a{ 0.0 }; int updateInterval_s{ 0 }; };

// ---------------- IONO (18/26) ----------------
struct IonoBandSnapshot {
   QDateTime start, end;
   int       band{ -1 };
   int       iodi{ -1 };

   // активные узлы: idPoint -> (lat_rad,lon_rad)
   QHash<int, QPair<double, double> > nodesRad;
};

struct IonoObs {
   QDateTime start, end;
   int       band{ -1 };
   int       iodi{ -1 };
   int       idPoint{ -1 };
   double    lat_rad{ 0 }, lon_rad{ 0 };
   double    vdelay_m{ 0 }, sigma_v_m2{ qQNaN() };
   bool      doNotUse{ false };
};

// ---------------- LONG-TERM (тип 25 и long-term часть типа 24) ----------------
// Структура — совместима с ожиданиями CorrectionApplier
struct LongTermCorrectionEntry {
   QDateTime start, end;          // интервал актуальности [start,end)
   int       iodp{ -1 };
   int       t0{ 0 };             // опорное время в секундах суток (секунды от 00:00)
   COORD_XYZ deltaPos{ 0, 0, 0 }; // ΔX,ΔY,ΔZ [м]
   COORD_XYZ deltaVel{ 0, 0, 0 }; // dΔX/dt ... [м/с]
   bool      hasVelocity{ false };
   double    deltaAf0{ 0.0 };     // поправка часов (сек)
   double    deltaAf1{ 0.0 };     // дрейф поправки, сек/сек
   enum class Source { From24, From25 };
   Source source{ Source::From24 };
};

// Временные смещения систем (для часов ГЛОНАСС)
struct TimeOffsetInterval {
   QDateTime start, end;
   double    gpsMinusGlonass_s{ 0.0 };
};

class SBASCorrectionStore {
public:

   SBASCorrectionStore() = default;
   bool        load(SourceType     type,
                    const QString& path);

   inline void buildCorrectionIndex() {
      buildTimelines();
   }

   const QMap<sbas::MESSAGE_TYPE, QVector<std::shared_ptr<sbas::MSG> > > &messages() const {
      return parsed_;
   }

   QVector<std::shared_ptr<sbas::MSG> > getByType(sbas::MESSAGE_TYPE t) const {
      return parsed_.value(t);
   }

   // ---- PRN resolve ----
   std::optional<Satellite> resolveByLocalIndex(int              localIdx,
                                                const QDateTime& t) const;

   // ---- UDRE ----
   std::optional<double> udreSigma_m2(const Satellite& sat,
                                      const QDateTime& t) const;
   std::optional<double> udreBound_m(const Satellite& sat,
                                     const QDateTime& t) const;
   std::optional<int>    udreIndex(const Satellite& sat,
                                   const QDateTime& t) const;

   // Эффективная дисперсия UDRE c учётом деградации (тип 7) и задержки обновления fast
   std::optional<double> udreSigmaEff_m2(const Satellite& sat,
                                         const QDateTime& t) const;

   // ---- FAST ----
   std::optional<int>       fastIodf(const Satellite& sat,
                                     const QDateTime& t) const;
   std::optional<int>       fastIodp(const Satellite& sat,
                                     const QDateTime& t) const;
   std::optional<QDateTime> fastLastUpdate(const Satellite& sat,
                                           const QDateTime& t) const;

   // ---- DEG (Type 7) ----
   std::optional<DegrParams> degradation(const Satellite& sat,
                                         const QDateTime& t) const;

   // ---- GIVE (18/26) ----
   bool ionoVerticalAt(double           ippLat_rad,
                       double           ippLon_rad,
                       const QDateTime& t,
                       double&          vdelay_m,
                       double&          var_v_m2) const;

   // NN-геттеры (оставлены для совместимости)
   std::optional<double> giveVar_m2(double           ippLat_rad,
                                    double           ippLon_rad,
                                    const QDateTime& t) const;
   std::optional<double> giveVDelay_m(double           ippLat_rad,
                                      double           ippLon_rad,
                                      const QDateTime& t) const;

   // ---- LONG-TERM (для CorrectionApplier) ----
   std::optional<LongTermCorrectionEntry> getLongTermCorrection(const Satellite& sat,
                                                                const QDateTime& t) const;

   // ---- Time offsets ----
   std::optional<double> gpsGlonassOffset(const QDateTime& t) const; // сек, если есть интервал
   void                  setGpsGlonassOffsets(const QVector<TimeOffsetInterval>& v) {
      gpsGloOffsets_ = v;
   }

private:

   void                 buildTimelines();
   bool                 loadHex(const QString& path);
   bool                 loadCsv(const QString& path);
   bool                 loadDb();

   // helpers
   static inline double haversine(double la1, double lo1, double la2, double lo2) {
      const double dlat = la2 - la1, dlon = lo2 - lo1;
      const double a = qSin(dlat / 2) * qSin(dlat / 2) + qCos(la1) * qCos(la2) * qSin(dlon / 2) * qSin(dlon / 2);

      return 2 * atan2(qSqrt(a), qSqrt(qMax(1.0 - a, 0.0)));
   }

   static inline double wrapLon(double lon) {
      while (lon >= M_PI) {
         lon -= 2 * M_PI;
      }

      while (lon <  -M_PI) {
         lon += 2 * M_PI;
      }
      return lon;
   }

   static inline double normLonRad(double lon) {
      // нормализация в (-pi, pi]
      double x = std::fmod(lon + M_PI, 2.0 * M_PI);

      if (x < 0) {
         x += 2.0 * M_PI;
      }
      return x - M_PI;
   }

   static inline double angDiffRad(double a, double b) {
      double d = normLonRad(a) - normLonRad(b);

      d = normLonRad(d);
      return std::fabs(d);
   }

   const IonoBandSnapshot*pickBandSnapshot(double           ippLat,
                                           double           ippLon,
                                           const QDateTime& t) const;
   int                    findCellCorners(const IonoBandSnapshot& snap,
                                          const QDateTime&        t,
                                          double                  ippLat,
                                          double                  ippLon,
                                          QVector<IonoObs>&       outCorners) const;

private:

   // Сырые сообщения
   QMap<sbas::MESSAGE_TYPE, QVector<std::shared_ptr<sbas::MSG> > > parsed_;
   QMap<sbas::MESSAGE_TYPE, QVector<std::shared_ptr<sbas::MSG> > > parsedMessages_;
   sbas::SbasParser parser_;

   // Индексы
   QVector<PrnMaskInterval> prnMasks_;
   QMap<Satellite, QVector<UdreEntry> > udre_;
   QMap<Satellite, QVector<FastEntry> > fast_;
   QMap<Satellite, QVector<DegrParams> > degr_;
   QMap<int, QVector<IonoBandSnapshot> > ionoBands_; // key=band
   QVector<IonoObs> ionoObs_;
   QMap<Satellite, QVector<LongTermCorrectionEntry> > longTermBySat_;
   QVector<TimeOffsetInterval> gpsGloOffsets_;
};
} // namespace io

#endif // SBASCORRECTIONSTORE_H
