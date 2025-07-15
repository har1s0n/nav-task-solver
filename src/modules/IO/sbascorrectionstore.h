#ifndef SBASCORRECTIONSTORE_H
#define SBASCORRECTIONSTORE_H

#include <QString>
#include <QMap>

#include <inav/SBAS>

namespace io {
enum class SourceType {
   FILE_HEX,
   FILE_CSV,
   DATABASE
};

struct LongTermCorrectionEntry {
   Satellite satId;               // eg. "R01"
   int       iode;
   int       t0;                  // reference time [sec of day]
   COORD_XYZ deltaPos;            // dx, dy, dz [m]
   COORD_XYZ deltaVel;            // vx, vy, vz [m/s], may be zero
   double    deltaAf0    = 0.0;
   double    deltaAf1    = 0.0;
   bool      hasVelocity = false; // true if derived from VELOCITY_CODE_1
};

struct PrnMaskInterval {
   QDateTime                                 startDt;
   QDateTime                                 endDt;
   QVector<sbas::MSG_PRN_MASK::Satelite_PRN> prns;
};

struct TimeOffsetInterval {
   QDateTime start;
   QDateTime end;
   double    timeCorrectionOffset = 0.0;
};

class SBASCorrectionStore {
public:

   SBASCorrectionStore() = default;
   bool                                                                   load(SourceType     type,
                                                                               const QString& path);
   const QMap<sbas::MESSAGE_TYPE, QVector<std::shared_ptr<sbas::MSG> > > &messages() const;
   QVector<std::shared_ptr<sbas::MSG> >                                   getByType(sbas::MESSAGE_TYPE type) const;
   std::optional<LongTermCorrectionEntry>                                 getLongTermCorrection(const Satellite& sat,
                                                                                                const QDateTime& epoch) const;
   std::optional<double>                                                  gpsGlonassOffset(const QDateTime& epoch)const;

private:

   bool                     loadHex(const QString& path);
   bool                     loadCsv(const QString& path);
   bool                     loadDb();
   void                     buildCorrectionIndex();
   std::optional<Satellite> resolveSatellite(int              prnMaskNumber,
                                             const QDateTime& recvTime) const;

private:

   QMap<sbas::MESSAGE_TYPE, QVector<std::shared_ptr<sbas::MSG> > > parsedMessages_;
   QMap<Satellite, QVector<LongTermCorrectionEntry> > correctionsBySat_;
   QVector<PrnMaskInterval> prnMaskTimeline_;
   QVector<TimeOffsetInterval> timeOffsets_;
   sbas::SbasParser parser_;
};
}

#endif // SBASCORRECTIONSTORE_H
