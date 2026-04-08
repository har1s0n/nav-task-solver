#ifndef SBASCORRECTIONSTORE_H
#define SBASCORRECTIONSTORE_H

#include <QString>
#include <QMap>
#include <QHash>
#include <QVector>
#include <QDateTime>
#include <QtMath>

#include <optional>
#include <cmath>
#include <memory>

#include <inav/SBAS>

namespace io {
enum class SourceType { FILE_HEX, FILE_CSV, DATABASE };

// ---------------- PRN MASK интервал ----------------
struct PrnMaskInterval {
   QDateTime start;
   QDateTime end;
   int       iodp{ -1 };

   // соответствие локального индекса из маски (1..210) -> Satellite
   QHash<int, Satellite> mapLocalIdxToSat;

   // Упорядоченный список номеров ПСП (1..210), для которых в данной PRN mask стоит «1».
   // Порядок соответствует перечислению в сообщениях, где используется PRN Mask Number (1..51).
   QVector<int> activePspsOrdered;
};

// ---------------- UDRE / FAST / DEG ----------------
struct UdreEntry {
   QDateTime          start, end;
   QDateTime          recvTime;
   int                udreIndex{ -1 };
   double             bound_m{ qQNaN() };
   double             sigma_m2{ qQNaN() };
   int                iodf{ -1 };
   int                iodp{ -1 };
   bool               doNotUse{ false };
   sbas::MESSAGE_TYPE source{ sbas::MESSAGE_TYPE::ZERO_MESSAGE };
};

struct FastEntry {
   QDateTime          start, end;
   QDateTime          recvTime;
   int                iodf{ -1 };
   int                iodp{ -1 };
   double             prc_m{ qQNaN() }; // FAST PRC (м) из типов 2..5/24
   bool               doNotUse{ false };
   sbas::MESSAGE_TYPE source{ sbas::MESSAGE_TYPE::ZERO_MESSAGE };
};

struct FastPair {
   FastEntry current;
   FastEntry previous;
};

struct DegrParams {
   QDateTime start, end;
   double    a{ 0.0 };
   int       updateInterval_s{ 0 };
};

// ---------------- IONO (18 / 26) ----------------
struct IonoBandSnapshot {
   QDateTime start, end;
   int       band{ -1 };
   int       iodi{ -1 };

   // активные узлы: idPoint -> (lat_rad, lon_rad)
   QHash<int, QPair<double, double> > nodesRad;
};

struct IonoObs {
   QDateTime start, end;
   int       band{ -1 };
   int       iodi{ -1 };
   int       idPoint{ -1 };
   double    lat_rad{ 0.0 };
   double    lon_rad{ 0.0 };
   double    vdelay_m{ 0.0 };
   double    sigma_v_m2{ qQNaN() };
   bool      doNotUse{ false };
};

// ---------------- LONG-TERM (тип 25 и long-term часть типа 24) ----------------
struct LongTermCorrectionEntry {
   QDateTime recvTime; // время приема LT сообщения пользователем
   QDateTime start, end;

   // 8-битный LT IOD (НЕ путать с IODP)
   int iod{ -1 };
   int iodp{ -1 };

   // опорное время в секундах суток (sec-of-day)
   int       t0{ 0 };
   COORD_XYZ deltaPos{ 0, 0, 0 };
   COORD_XYZ deltaVel{ 0, 0, 0 };
   bool      hasVelocity{ false };

   // поправка часов [с]
   double deltaAf0{ 0.0 };

   // дрейф поправки [с/с]
   double deltaAf1{ 0.0 };

   enum class Source { From24, From25 };
   Source source{ Source::From24 };
};

// ---------------- Time offsets ----------------
struct TimeOffsetInterval {
   QDateTime start, end; // [start,end)
   double    rawDeltaAGlonass_s = 0.0;
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

   // PRN Mask Number (1..51) -> Satellite через активную PRN Mask.
   // В ИКД СДКМ поля satNum/prn в сообщениях типов 2–7, 24, 25 — это именно PRN Mask Number,
   // а не номер ПСП (1..210) и не PRN в смысле GPS.
   std::optional<Satellite> resolveByMaskNumber(int              prnMaskNumber,
                                                const QDateTime& t) const;

   // ---- UDRE ----
   std::optional<double> udreSigma_m2(const Satellite& sat,
                                      const QDateTime& t) const;

   std::optional<double> udreBound_m(const Satellite& sat,
                                     const QDateTime& t) const;

   std::optional<int>    udreIndex(const Satellite& sat,
                                   const QDateTime& t) const;

   // Эффективная дисперсия UDRE с учетом деградации (тип 7) и задержки обновления FAST
   std::optional<double> udreSigmaEff_m2(const Satellite& sat,
                                         const QDateTime& t) const;

   // ---- FAST ----
   std::optional<int>       fastIodf(const Satellite& sat,
                                     const QDateTime& t) const;

   std::optional<int>       fastIodp(const Satellite& sat,
                                     const QDateTime& t) const;

   std::optional<double>    fastPrc_m(const Satellite& sat,
                                      const QDateTime& t) const;

   std::optional<QDateTime> fastLastUpdate(const Satellite& sat,
                                           const QDateTime& t) const;

   std::optional<FastPair>  fastCurrentPrevious(const Satellite& sat,
                                                const QDateTime& t) const;

   // ---- DEG (Type 7) ----
   std::optional<DegrParams> degradation(const Satellite& sat,
                                         const QDateTime& t) const;

   // ---- GIVE (18 / 26) ----
   bool ionoVerticalAt(double           ippLat_rad,
                       double           ippLon_rad,
                       const QDateTime& t,
                       double&          vdelay_m,
                       double&          var_v_m2) const;

   // NN-геттеры оставлены для совместимости
   std::optional<double> giveVar_m2(double           ippLat_rad,
                                    double           ippLon_rad,
                                    const QDateTime& t) const;

   std::optional<double> giveVDelay_m(double           ippLat_rad,
                                      double           ippLon_rad,
                                      const QDateTime& t) const;

   // ---- LONG-TERM ----
   std::optional<LongTermCorrectionEntry> getLongTermCorrection(const Satellite& sat,
                                                                const QDateTime& t) const;
   QVector<LongTermCorrectionEntry>       getAllActiveLongTermCorrections(const Satellite& sat,
                                                                          const QDateTime& t) const;

   // ---- Time offsets ----
   std::optional<double> gpsGlonassOffset(const QDateTime& t) const;

   void                  setGpsGlonassOffsets(const QVector<TimeOffsetInterval>& v) {
      gpsGloOffsets_ = v;
   }

   // public:
   void logLt25GlonassStoreDaf0Summary() const;

private:

   void buildTimelines();

   bool loadHex(const QString& path);
   bool loadCsv(const QString& path);
   bool loadDb();

   // ---- helpers ----

private:

   std::optional<int>            activePrnMaskIodp(const QDateTime& t) const;
   const PrnMaskInterval*        activePrnMask(const QDateTime& t) const;
   const FastEntry*              selectedFastEntry(const Satellite& sat,
                                                   const QDateTime& t) const;
   const UdreEntry*              selectedUdreEntry(const Satellite& sat,
                                                   const QDateTime& t) const;
   const LongTermCorrectionEntry*selectedLongTermEntry(const Satellite& sat,
                                                       const QDateTime& t) const;

   static inline double          haversine(double la1, double lo1, double la2, double lo2) {
      const double dlat = la2 - la1;
      const double dlon = lo2 - lo1;
      const double a    =
         qSin(dlat / 2.0) * qSin(dlat / 2.0) +
         qCos(la1) * qCos(la2) * qSin(dlon / 2.0) * qSin(dlon / 2.0);

      return 2.0 * std::atan2(qSqrt(a), qSqrt(qMax(1.0 - a, 0.0)));
   }

   static inline double wrapLon(double lon) {
      while (lon >= M_PI) {
         lon -= 2.0 * M_PI;
      }

      while (lon < -M_PI) {
         lon += 2.0 * M_PI;
      }
      return lon;
   }

   static inline double normLonRad(double lon) {
      // нормализация в (-pi, pi]
      double x = std::fmod(lon + M_PI, 2.0 * M_PI);

      if (x < 0.0) {
         x += 2.0 * M_PI;
      }
      return x - M_PI;
   }

   static inline double angDiffRad(double a, double b) {
      double d = normLonRad(a) - normLonRad(b);

      d = normLonRad(d);
      return std::fabs(d);
   }

   struct ActiveNode {
      double lat_rad{ 0.0 };
      double lon_rad{ 0.0 };
      int    band{ -1 };
      int    iodi{ -1 };
      int    idPoint{ -1 };
   };

   // Формирует единую глобальную сетку активных узлов на заданную эпоху
   QVector<ActiveNode> getActiveGrid(const QDateTime& t) const;

   // Ищет окружающие узлы по глобальной сетке
   int                 findCellCorners(const QVector<ActiveNode>& grid,
                                       const QDateTime&           t,
                                       double                     ippLat,
                                       double                     ippLon,
                                       QVector<IonoObs>&          outCorners) const;

   const IonoBandSnapshot*pickBandSnapshot(double           ippLat,
                                           double           ippLon,
                                           const QDateTime& t) const;

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
   QMap<int, QVector<IonoBandSnapshot> > ionoBands_;
   QVector<IonoObs> ionoObs_;
   QMap<Satellite, QVector<LongTermCorrectionEntry> > longTermBySat_;
   QVector<TimeOffsetInterval> gpsGloOffsets_;
};
} // namespace io

#endif // SBASCORRECTIONSTORE_H
