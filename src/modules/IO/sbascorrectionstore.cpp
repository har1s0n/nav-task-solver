#include "sbascorrectionstore.h"
#include <QFile>
#include <QTextStream>
#include <QtMath>
#include <limits>
#include <algorithm>
#include <QDebug>

using namespace io;
using namespace sbas;

constexpr int kPrnMaskHoldSeconds     = 600;
constexpr int kFastTimeoutSeconds     = 18;
constexpr int kUdreTimeoutSeconds     = 600;
constexpr int kDegrTimeoutSeconds     = 600;
constexpr int kLongTermTimeoutSeconds = 360;
constexpr int kIonoHoldSeconds        = 600;

template<typename T>
int extractLtIod(const T& s) {
   return static_cast<int> (s.iode);
}

namespace {
struct Daf0Summary {
   qint64 count{ 0 };
   qint64 pos{ 0 };
   qint64 neg{ 0 };
   qint64 zero{ 0 };

   double sumAbsPos{ 0.0 };
   double sumAbsNeg{ 0.0 };

   double maxAbsPos{ 0.0 };
   double maxAbsNeg{ 0.0 };
};

static inline void accumulateDaf0Summary(Daf0Summary& s, double v) {
   ++s.count;

   if (v > 0.0) {
      ++s.pos;
      const double a = std::abs(v);
      s.sumAbsPos += a;

      if (a > s.maxAbsPos) {
         s.maxAbsPos = a;
      }
   } else if (v < 0.0) {
      ++s.neg;
      const double a = std::abs(v);
      s.sumAbsNeg += a;

      if (a > s.maxAbsNeg) {
         s.maxAbsNeg = a;
      }
   } else {
      ++s.zero;
   }
}

static inline void printDaf0Summary(const QString& tag, const Daf0Summary& s) {
   const double meanAbsPos = (s.pos > 0) ? (s.sumAbsPos / static_cast<double> (s.pos)) : 0.0;
   const double meanAbsNeg = (s.neg > 0) ? (s.sumAbsNeg / static_cast<double> (s.neg)) : 0.0;

   qInfo().noquote()
      << QString("[%1] count=%2 | pos=%3 neg=%4 zero=%5 | "
                 "mean|pos|=%6 mean|neg|=%7 | max|pos|=%8 max|neg|=%9")
      .arg(tag)
      .arg(s.count)
      .arg(s.pos)
      .arg(s.neg)
      .arg(s.zero)
      .arg(meanAbsPos,  0, 'e', 6)
      .arg(meanAbsNeg,  0, 'e', 6)
      .arg(s.maxAbsPos, 0, 'e', 6)
      .arg(s.maxAbsNeg, 0, 'e', 6);
}
} // namespace

void SBASCorrectionStore::logLt25GlonassStoreDaf0Summary() const {
   Daf0Summary s;

   for (auto it = longTermBySat_.cbegin(); it != longTermBySat_.cend(); ++it) {
      const Satellite& sat = it.key();

      if (sat.getSystem() != SatelliteSystem::TYPE::GLONASS) {
         continue;
      }

      for (const auto& e : it.value()) {
         if ((e.source == LongTermCorrectionEntry::Source::From25) &&
             !e.hasVelocity) {
            accumulateDaf0Summary(s, e.deltaAf0);
         }
      }
   }

   printDaf0Summary(QStringLiteral("LT25_GLO_STORE_DAF0_SUMMARY"), s);
}

bool SBASCorrectionStore::load(SourceType type, const QString& path) {
   parsed_.clear();
   parsedMessages_.clear();
   bool ok = false;

   switch (type) {
     case SourceType::FILE_HEX: ok = loadHex(path); break;
     case SourceType::FILE_CSV: ok = loadCsv(path); break;
     case SourceType::DATABASE: ok = loadDb();      break;
   }

   if (ok) {
      parsed_ = parsedMessages_;
      buildCorrectionIndex();
   }
   return ok;
}

bool SBASCorrectionStore::loadHex(const QString& path) {
   QFile file(path);

   if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      return false;
   }
   QTextStream in(&file);
   int total = 0, parsed = 0;

   while (!in.atEnd()) {
      const QString line = in.readLine().trimmed();

      if (line.isEmpty()) {
         continue;
      }
      const QStringList tokens = line.split(' ', Qt::SkipEmptyParts);

      if (tokens.size() < 9) {
         continue;
      }

      const QString testDt = QString("%1-%2-%3 %4:%5:%6")
                             .arg("20" + tokens[1]).arg(tokens[2]).arg(tokens[3])
                             .arg(tokens[4]).arg(tokens[5]).arg(tokens[6]);
      const QString testRawHex = tokens[8].trimmed();
      QDateTime     recvTime   = QDateTime::fromString(testDt, "yyyy-MM-dd hh:mm:ss");

      if (!recvTime.isValid()) {
         continue;
      }

      ++total;
      auto result = parser_.parse(testRawHex);

      if ((result.msgStatus == sbas::PARSE_STATUS::OK) && result.msg) {
         result.msg->recvTime = recvTime;
         parsedMessages_[result.msg->getTypeMsg()].append(result.msg);
         ++parsed;
      }
   }
   return parsed > 0;
}

bool SBASCorrectionStore::loadCsv(const QString& path) {
   QFile file(path);

   if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      return false;
   }
   QTextStream in(&file);
   QString     header = in.readLine(); Q_UNUSED(header);
   int total = 0, parsed = 0;

   while (!in.atEnd()) {
      const QString line = in.readLine().trimmed();

      if (line.isEmpty()) {
         continue;
      }
      const QStringList tokens = line.split(',', Qt::KeepEmptyParts);

      if (tokens.size() < 5) {
         continue;
      }

      const QString dtStr    = tokens[1].trimmed().remove('"');
      const QString rawHex   = tokens[4].trimmed();
      QDateTime     recvTime = QDateTime::fromString(dtStr, "yyyy-MM-dd hh:mm:ss");

      if (!recvTime.isValid()) {
         continue;
      }

      ++total;
      auto result = parser_.parse(rawHex);

      if ((result.msgStatus == sbas::PARSE_STATUS::OK) && result.msg) {
         result.msg->recvTime = recvTime;
         parsedMessages_[result.msg->getTypeMsg()].append(result.msg);
         ++parsed;
      }
   }
   return parsed > 0;
}

bool SBASCorrectionStore::loadDb() {
   return false;
}

std::optional<Satellite> SBASCorrectionStore::resolveByLocalIndex(int localIdx, const QDateTime& t) const {
   const auto* iv = activePrnMask(t);

   if (!iv) {
      return std::nullopt;
   }

   auto it = iv->mapLocalIdxToSat.find(localIdx);

   if (it != iv->mapLocalIdxToSat.end()) {
      return it.value();
   }
   return std::nullopt;
}

std::optional<Satellite> SBASCorrectionStore::resolveByMaskNumber(int prnMaskNumber, const QDateTime& t) const {
   if (prnMaskNumber <= 0) {
      return std::nullopt;
   }

   const auto* iv = activePrnMask(t);

   if (!iv) {
      return std::nullopt;
   }

   const int idx0 = prnMaskNumber - 1;

   if ((idx0 < 0) || (idx0 >= iv->activePspsOrdered.size())) {
      return std::nullopt;
   }

   auto it = iv->mapLocalIdxToSat.find(iv->activePspsOrdered[idx0]);

   if (it != iv->mapLocalIdxToSat.end()) {
      return it.value();
   }
   return std::nullopt;
}

std::optional<int> SBASCorrectionStore::activePrnMaskIodp(const QDateTime& t) const {
   const auto* iv = activePrnMask(t);

   if (!iv) {
      return std::nullopt;
   }
   return iv->iodp;
}

const PrnMaskInterval*SBASCorrectionStore::activePrnMask(const QDateTime& t) const {
   for (auto it = prnMasks_.crbegin(); it != prnMasks_.crend(); ++it) {
      if ((t >= it->start) && (t < it->end)) {
         return &(*it);
      }
   }
   return nullptr;
}

const FastEntry*SBASCorrectionStore::selectedFastEntry(const Satellite& sat, const QDateTime& t) const {
   const auto maskIodp = activePrnMaskIodp(t);

   auto it = fast_.find(sat);

   if (it == fast_.end()) {
      return nullptr;
   }

   for (auto rit = it.value().crbegin(); rit != it.value().crend(); ++rit) {
      const auto& e = *rit;

      if (!((t >= e.start) && (t < e.end))) {
         continue;
      }

      if (maskIodp && (e.iodp >= 0) && (e.iodp != *maskIodp)) {
         continue;
      }

      if (e.doNotUse || !qIsFinite(e.prc_m)) {
         return nullptr;
      }

      return &e;
   }

   return nullptr;
}

const UdreEntry*SBASCorrectionStore::selectedUdreEntry(const Satellite& sat,
                                                       const QDateTime& t) const {
   const auto maskIodp    = activePrnMaskIodp(t);
   const auto curFastIodf = fastIodf(sat, t);

   auto it = udre_.find(sat);

   if (it == udre_.end()) {
      return nullptr;
   }

   const auto& vec = it.value();

   // 1) Пытаемся найти запись, согласованную с текущим FAST IODF
   if (curFastIodf) {
      for (auto rit = vec.crbegin(); rit != vec.crend(); ++rit) {
         const auto& e = *rit;

         if (!((t >= e.start) && (t < e.end))) {
            continue;
         }

         if (maskIodp && (e.iodp >= 0) && (e.iodp != *maskIodp)) {
            continue;
         }

         if (e.iodf == *curFastIodf) {
            return &e;
         }
      }
   }

   // 2) Временный fallback для type 6 без привязки по IODF
   for (auto rit = vec.crbegin(); rit != vec.crend(); ++rit) {
      const auto& e = *rit;

      if (!((t >= e.start) && (t < e.end))) {
         continue;
      }

      if (maskIodp && (e.iodp >= 0) && (e.iodp != *maskIodp)) {
         continue;
      }

      if (e.iodf < 0) {
         return &e;
      }
   }

   return nullptr;
}

const LongTermCorrectionEntry*SBASCorrectionStore::selectedLongTermEntry(const Satellite& sat, const QDateTime& t) const {
   const auto maskIodp = activePrnMaskIodp(t);

   auto it = longTermBySat_.find(sat);

   if (it == longTermBySat_.end()) {
      return nullptr;
   }

   for (auto rit = it.value().crbegin(); rit != it.value().crend(); ++rit) {
      const auto& e = *rit;

      if (!((t >= e.start) && (t < e.end))) {
         continue;
      }

      if (maskIodp && (e.iodp >= 0) && (e.iodp != *maskIodp)) {
         continue;
      }

      return &e;
   }

   return nullptr;
}

void SBASCorrectionStore::buildTimelines() {
   prnMasks_.clear();
   udre_.clear();
   fast_.clear();
   degr_.clear();
   ionoBands_.clear();
   ionoObs_.clear();
   longTermBySat_.clear();
   gpsGloOffsets_.clear();

   for (auto it = parsed_.begin(); it != parsed_.end(); ++it) {
      std::sort(it.value().begin(), it.value().end(), [](const auto& a, const auto& b) {
         if (a && b) {
            return a->recvTime < b->recvTime;
         }
         return a != nullptr;
      });
   }

   QDateTime maxRecv, tMax;
   bool tMaxInit = false;

   for (auto it = parsed_.begin(); it != parsed_.end(); ++it) {
      for (const auto& msg : it.value()) {
         if (!msg) {
            continue;
         }

         if (!tMaxInit || (msg->recvTime > tMax)) {
            tMax = msg->recvTime; tMaxInit = true;
         }

         if (!maxRecv.isValid() || (msg->recvTime > maxRecv)) {
            maxRecv = msg->recvTime;
         }
      }
   }

   if (!tMaxInit) {
      tMax = QDateTime();
   }

   // --------- NETWORK OFFSET / TIME OFFSET (type 12) ---------
   const auto netMsgs = parsed_.value(MESSAGE_TYPE::NETWORK_OFFSET);

   for (int i = 0; i < netMsgs.size(); ++i) {
      auto m = std::dynamic_pointer_cast<MSG_NETWORK_OFFSET> (netMsgs[i]);

      if (!m) {
         continue;
      }

      TimeOffsetInterval iv;
      iv.start = m->recvTime;
      iv.end   = (i + 1 < netMsgs.size() && netMsgs[i + 1])
                        ? netMsgs[i + 1]->recvTime
                        : (maxRecv.isValid()
                               ? maxRecv.addSecs(1)
                               : m->recvTime.addSecs(1));
      iv.rawDeltaAGlonass_s = m->deltaAGlonassCandidate_s;

      gpsGloOffsets_.push_back(iv);
   }

   // --------- PRN MASK (1) ---------
   const auto prnMsgs = parsed_.value(MESSAGE_TYPE::PRN_MASK);

   for (int i = 0; i < prnMsgs.size(); ++i) {
      auto m = std::dynamic_pointer_cast<MSG_PRN_MASK> (prnMsgs[i]);

      if (!m) {
         continue;
      }
      PrnMaskInterval iv;
      iv.start = m->recvTime;
      iv.end   = (i + 1 < prnMsgs.size() &&
                  prnMsgs[i + 1]) ? prnMsgs[i + 1]->recvTime
                     : (maxRecv.isValid() ? maxRecv.addSecs(kPrnMaskHoldSeconds)
                                          : m->recvTime.addSecs(kPrnMaskHoldSeconds));
      iv.iodp              = m->iodp;
      iv.activePspsOrdered = m->activePsps;

      int N = qMin(m->activePsps.size(), m->satelites.size());

      for (int k = 0; k < N; ++k) {
         SatelliteSystem::TYPE sys = SatelliteSystem::TYPE::UNKNOWN;

         switch (m->satelites[k].systemSat) {
           case PURPOSE_SYSTEM::GPS:          sys = SatelliteSystem::TYPE::GPS; break;
           case PURPOSE_SYSTEM::GLONASS:      sys = SatelliteSystem::TYPE::GLONASS; break;
           case PURPOSE_SYSTEM::GEO_WAAS_PRN: sys = SatelliteSystem::TYPE::SBAS; break;
           default: break;
         }

         iv.mapLocalIdxToSat.insert(
            m->activePsps[k],
            Satellite(m->satelites[k].satId, sys)
            );
      }

      prnMasks_.push_back(std::move(iv));
   }

   // --------- FAST / MIXED (2..5, 24) ---------
   auto ingestFast = [&](MESSAGE_TYPE ty) {
                        for (const auto& r : parsed_.value(ty)) {
                           auto m = std::dynamic_pointer_cast<MSG_FAST_CORRECTIONS> (r);

                           if (!m) {
                              continue;
                           }

                           for (const auto& s : m->satelites) {
                              if (auto sat = resolveByMaskNumber((int)s.satNum, m->recvTime)) {
                                 fast_[*sat].push_back({
                  m->recvTime,
                  m->recvTime.addSecs(kFastTimeoutSeconds),
                  m->recvTime,
                  m->iodf,
                  m->iodp,
                  s.doNotUse ? qQNaN() : s.fc,
                  s.doNotUse,
                  ty
               });

                                 udre_[*sat].push_back({
                  m->recvTime,
                  m->recvTime.addSecs(kUdreTimeoutSeconds),
                  m->recvTime,
                  s.UDREI,
                  s.doNotUse ? qInf() : s.UDREI_meters,
                  s.doNotUse ? qInf() : s.sigma_UDREI,
                  m->iodf,
                  m->iodp,
                  s.doNotUse,
                  ty
               });
                              }
                           }
                        }
                     };

   ingestFast(MESSAGE_TYPE::FAST_CORRECTIONS_1);
   ingestFast(MESSAGE_TYPE::FAST_CORRECTIONS_2);
   ingestFast(MESSAGE_TYPE::FAST_CORRECTIONS_3);
   ingestFast(MESSAGE_TYPE::FAST_CORRECTIONS_4);

   for (const auto& r : parsed_.value(MESSAGE_TYPE::MIXED_CORRECTIONS_SATELLITE_ERROR)) {
      auto m = std::dynamic_pointer_cast<MSG_MIXED_CORRECTIONS_SATELLITE_ERROR> (r);

      if (!m) {
         continue;
      }

      for (const auto& s : m->fast_correction.satelites) {
         if (auto sat = resolveByMaskNumber((int)s.satNum, m->recvTime)) {
            fast_[*sat].push_back({
               m->recvTime,
               m->recvTime.addSecs(kFastTimeoutSeconds),
               m->recvTime,
               m->fast_correction.iodf,
               m->fast_correction.iodp,
               s.doNotUse ? qQNaN() : s.fc,
               s.doNotUse,
               MESSAGE_TYPE::MIXED_CORRECTIONS_SATELLITE_ERROR
            });

            udre_[*sat].push_back({
               m->recvTime,
               m->recvTime.addSecs(kUdreTimeoutSeconds),
               m->recvTime,
               s.UDREI,
               s.doNotUse ? qInf() : s.UDREI_meters,
               s.doNotUse ? qInf() : s.sigma_UDREI,
               m->fast_correction.iodf,
               m->fast_correction.iodp,
               s.doNotUse,
               MESSAGE_TYPE::MIXED_CORRECTIONS_SATELLITE_ERROR
            });
         }
      }
   }

   // --------- INTEGRITY (6) ---------
   for (const auto& r : parsed_.value(MESSAGE_TYPE::INTEGRITY_INFORMATION)) {
      auto m = std::dynamic_pointer_cast<MSG_INTEGRITY_INFORMATION> (r);

      if (!m) {
         continue;
      }

      for (const auto& s : m->satelites) {
         if (auto sat = resolveByMaskNumber((int)s.satNum, m->recvTime)) {
            udre_[*sat].push_back({
               m->recvTime,
               m->recvTime.addSecs(kUdreTimeoutSeconds),
               m->recvTime,
               s.UDREI,
               s.doNotUse ? qInf() : s.UDREI_meters,
               s.doNotUse ? qInf() : s.sigma_UDREI,
               -1,
               -1,
               s.doNotUse,
               MESSAGE_TYPE::INTEGRITY_INFORMATION
            });
         }
      }
   }

   // --------- DEG (7) ---------
   for (const auto& r : parsed_.value(MESSAGE_TYPE::FAST_CORRECTION_DEGRADATION_FACTOR)) {
      auto m = std::dynamic_pointer_cast<MSG_FAST_CORRECTION_DEGRADATION_FACTOR> (r);

      if (!m) {
         continue;
      }

      for (const auto& s : m->satelites) {
         if (auto sat = resolveByMaskNumber((int)s.satNum, m->recvTime); sat && !s.doNotUse) {
            degr_[*sat].push_back({
               m->recvTime,
               m->recvTime.addSecs(kDegrTimeoutSeconds),
               s.a,
               s.updateInterval
            });
         }
      }
   }

   // --------- LONG-TERM (24, 25) ---------
   for (const auto& r : parsed_.value(MESSAGE_TYPE::LONG_TERM_SATELLITE_ERROR_CORRECTIONS)) {
      auto m = std::dynamic_pointer_cast<MSG_LONG_TERM_SATELLITE_ERROR_CORRECTIONS> (r);

      if (!m) {
         continue;
      }

      for (const auto& s : m->satellites_code_1) {
         const auto satResolved = resolveByMaskNumber(s.prn, m->recvTime);

         if (auto sat = resolveByMaskNumber(s.prn, m->recvTime)) {
            longTermBySat_[*sat].push_back({
               m->recvTime,
               m->recvTime,
               m->recvTime.addSecs(kLongTermTimeoutSeconds),
               s.iode,
               s.iodp,
               s.t0,
               { s.deltaEcef.x, s.deltaEcef.y, s.deltaEcef.z },
               { s.deltaRoc.x,  s.deltaRoc.y,  s.deltaRoc.z  },
               true,
               s.delta_a_f0,
               s.delta_a_f1,
               LongTermCorrectionEntry::Source::From25
            });
         }
      }

      for (const auto& s : m->satellites_code_0) {
         const auto satResolved = resolveByMaskNumber(s.prn, m->recvTime);

         if (auto sat = resolveByMaskNumber(s.prn, m->recvTime)) {
            longTermBySat_[*sat].push_back({
               m->recvTime,
               m->recvTime,
               m->recvTime.addSecs(kLongTermTimeoutSeconds),
               s.iode,
               s.iodp,
               QTime(0, 0).secsTo(m->recvTime.time()),
               { s.deltaEcef.x, s.deltaEcef.y, s.deltaEcef.z },
               { 0, 0, 0 },
               false,
               s.delta_a_f0,
               0.0,
               LongTermCorrectionEntry::Source::From25
            });
         }
      }
   }

   for (const auto& r : parsed_.value(MESSAGE_TYPE::MIXED_CORRECTIONS_SATELLITE_ERROR)) {
      auto m = std::dynamic_pointer_cast<MSG_MIXED_CORRECTIONS_SATELLITE_ERROR> (r);

      if (!m) {
         continue;
      }

      for (const auto& basePtr : m->satellites) {
         if (auto c1 = std::dynamic_pointer_cast<VELOCITY_CODE_1> (basePtr)) {
            if (auto sat = resolveByMaskNumber(c1->prn, m->recvTime)) {
               longTermBySat_[*sat].push_back({
                  m->recvTime,
                  m->recvTime,
                  m->recvTime.addSecs(kLongTermTimeoutSeconds),
                  c1->iode,
                  m->iodp,
                  c1->t0,
                  { c1->deltaEcef.x, c1->deltaEcef.y, c1->deltaEcef.z },
                  { c1->deltaRoc.x,  c1->deltaRoc.y,  c1->deltaRoc.z  },
                  true,
                  c1->delta_a_f0,
                  c1->delta_a_f1,
                  LongTermCorrectionEntry::Source::From24
               });
            }
         } else if (auto c0 = std::dynamic_pointer_cast<VELOCITY_CODE_0> (basePtr)) {
            if (auto sat = resolveByMaskNumber(c0->prn, m->recvTime)) {
               longTermBySat_[*sat].push_back({
                  m->recvTime,
                  m->recvTime,
                  m->recvTime.addSecs(kLongTermTimeoutSeconds),
                  extractLtIod(*c0),
                  m->iodp,
                  QTime(0, 0).secsTo(m->recvTime.time()),
                  { c0->deltaEcef.x, c0->deltaEcef.y, c0->deltaEcef.z },
                  { 0, 0, 0 },
                  false,
                  c0->delta_a_f0,
                  0.0,
                  LongTermCorrectionEntry::Source::From24
               });
            }
         }
      }
   }

   auto normalizeUdreSameStart = [](auto& mapOfVecs) {
                                    for (auto it = mapOfVecs.begin(); it != mapOfVecs.end(); ++it) {
                                       auto& vec = it.value();

                                       std::sort(vec.begin(), vec.end(), [](const auto& a, const auto& b) {
            if (a.start != b.start) {
               return a.start < b.start;
            }

            if (a.recvTime != b.recvTime) {
               return a.recvTime < b.recvTime;
            }

            if (a.source != b.source) {
               return static_cast<int> (a.source) < static_cast<int> (b.source);
            }
            return a.iodf < b.iodf;
         });

                                       for (int i = 0; i < vec.size() - 1; ++i) {
                                          if (vec[i].start < vec[i + 1].start) {
                                             vec[i].end = qMin(vec[i].end, vec[i + 1].start);
                                          }
                                       }

                                       vec.erase(
                                          std::remove_if(vec.begin(), vec.end(), [](const auto& e) {
            return e.start > e.end;
         }),
                                          vec.end()
                                          );
                                    }
                                 };

   auto fixOverlapsSoftSameStart = [](auto& mapOfVecs, const char* tag) {
                                      for (auto it = mapOfVecs.begin(); it != mapOfVecs.end(); ++it) {
                                         auto& vec = it.value();

                                         std::sort(vec.begin(), vec.end(), [](const auto& a, const auto& b) {
            if (a.start != b.start) {
               return a.start < b.start;
            }

            if (a.recvTime != b.recvTime) {
               return a.recvTime < b.recvTime;
            }
            return static_cast<int> (a.source) < static_cast<int> (b.source);
         });

                                         for (int i = 0; i < vec.size() - 1; ++i) {
                                            if (vec[i].start < vec[i + 1].start) {
                                               vec[i].end = qMin(vec[i].end, vec[i + 1].start);
                                            } else if (vec[i].start == vec[i + 1].start) {}
                                         }

                                         vec.erase(
                                            std::remove_if(vec.begin(), vec.end(), [](const auto& e) {
            return e.start > e.end;
         }),
                                            vec.end()
                                            );
                                      }
                                   };

   normalizeUdreSameStart(udre_);
   // normalizeUdreSameStart(degr_);

   fixOverlapsSoftSameStart(fast_,          "FAST");
   fixOverlapsSoftSameStart(longTermBySat_, "LT");

   // --------- IONO MASKS (18) ---------
   QHash<int, QVector<std::shared_ptr<MSG_IONOSPHERIC_GRID_POINT_MASK> > > masksByBand;

   for (const auto& r : parsed_.value(MESSAGE_TYPE::IONOSPHERIC_GRID_POINT_MASK)) {
      if (auto m = std::dynamic_pointer_cast<MSG_IONOSPHERIC_GRID_POINT_MASK> (r)) {
         masksByBand[m->idBand].push_back(std::move(m));
      }
   }

   for (auto it = masksByBand.begin(); it != masksByBand.end(); ++it) {
      auto& vec = it.value();

      for (int i = 0; i < vec.size(); ++i) {
         if (!vec[i]) {
            continue;
         }

         IonoBandSnapshot snap{
            vec[i]->recvTime,
            (i + 1 < vec.size() && vec[i + 1])
                    ? vec[i + 1]->recvTime
                    : (tMaxInit ? tMax.addSecs(kIonoHoldSeconds)
                                : vec[i]->recvTime.addSecs(kIonoHoldSeconds)),
            it.key(),
            static_cast<int> (vec[i]->iod)
         };

         if (!(snap.end > snap.start)) {
            snap.end = snap.start.addSecs(1);
         }

         for (const auto& nd : vec[i]->coordinatesRange) {
            if ((nd.idPoint >= 1) && (nd.idPoint <= 201)) {
               snap.nodesRad.insert(
                  nd.idPoint,
                  { qDegreesToRadians(double(nd.lat)), wrapLon(qDegreesToRadians(double(nd.lon))) }
                  );
            }
         }

         ionoBands_[snap.band].push_back(std::move(snap));
      }
   }

   // --------- IONO OBS (26) ---------
   for (const auto& r : parsed_.value(MESSAGE_TYPE::IONOSPHERIC_DELAY_CORRECTIONS)) {
      if (auto m = std::dynamic_pointer_cast<MSG_IONOSPHERIC_DELAY_CORRECTIONS> (r)) {
         const IonoBandSnapshot* snap = nullptr;

         // Ослаблена привязка ко времени: ищем узлы по совпадению IODI в нужной зоне.
         for (const auto& s : ionoBands_.value(m->numberBand)) {
            if (s.iodi == m->iod) {
               snap = &s;

               if ((m->recvTime >= s.start) && (m->recvTime < s.end)) {
                  break;
               }
            }
         }

         if (!snap) {
            continue;
         }

         for (const auto& bp : m->points) {
            if (auto itNode = snap->nodesRad.find(bp.idPoint); (itNode != snap->nodesRad.end())) {
               ionoObs_.push_back({
                  m->recvTime,
                  m->recvTime.addSecs(kIonoHoldSeconds),
                  static_cast<int> (m->numberBand),
                  static_cast<int> (m->iod),
                  static_cast<int> (bp.idPoint),
                  itNode.value().first,
                  itNode.value().second,
                  bp.igp,
                  bp.sigma_give,
                  bp.doNotUse
               });
            }
         }
      }
   }

   std::sort(ionoObs_.begin(), ionoObs_.end(), [](const IonoObs& a, const IonoObs& b) {
      if (a.band != b.band) {
         return a.band < b.band;
      }

      if (a.iodi != b.iodi) {
         return a.iodi < b.iodi;
      }

      if (a.idPoint != b.idPoint) {
         return a.idPoint < b.idPoint;
      }
      return a.start < b.start;
   });

   for (int i = 0; i < ionoObs_.size() - 1; ++i) {
      if ((ionoObs_[i].band == ionoObs_[i + 1].band) &&
          (ionoObs_[i].iodi == ionoObs_[i + 1].iodi) &&
          (ionoObs_[i].idPoint == ionoObs_[i + 1].idPoint)) {
         ionoObs_[i].end = qMin(ionoObs_[i].end, ionoObs_[i + 1].start);
      }
   }

   ionoObs_.erase(
      std::remove_if(ionoObs_.begin(), ionoObs_.end(), [](const IonoObs& e) {
      return e.start >= e.end;
   }),
      ionoObs_.end()
      );
}

// ---------------- Getters ----------------
std::optional<double> SBASCorrectionStore::udreSigma_m2(const Satellite& sat, const QDateTime& t) const {
   if (const auto* e = selectedUdreEntry(sat, t)) {
      if (e->doNotUse || !qIsFinite(e->sigma_m2)) {
         return std::nullopt;
      }
      return e->sigma_m2;
   }
   return std::nullopt;
}

std::optional<double> SBASCorrectionStore::udreBound_m(const Satellite& sat, const QDateTime& t) const {
   if (const auto* e = selectedUdreEntry(sat, t)) {
      if (e->doNotUse || !qIsFinite(e->bound_m)) {
         return std::nullopt;
      }
      return e->bound_m;
   }
   return std::nullopt;
}

std::optional<int> SBASCorrectionStore::udreIndex(const Satellite& sat, const QDateTime& t) const {
   if (const auto* e = selectedUdreEntry(sat, t)) {
      if (e->doNotUse) {
         return std::nullopt;
      }
      return e->udreIndex;
   }
   return std::nullopt;
}

std::optional<double> SBASCorrectionStore::udreSigmaEff_m2(const Satellite& sat, const QDateTime& t) const {
   auto s2 = udreSigma_m2(sat, t);

   if (!s2) {
      return std::nullopt;
   }

   double var = *s2;
   auto   d   = degradation(sat, t);
   auto   lu  = fastLastUpdate(sat, t);

   if (d && lu) {
      double dt = std::max(0, (int)lu->secsTo(t));
      var += (d->a * dt) * (d->a * dt);
   }

   return var;
}

std::optional<int> SBASCorrectionStore::fastIodf(const Satellite& sat, const QDateTime& t) const {
   if (const auto* e = selectedFastEntry(sat, t)) {
      return e->iodf;
   }
   return std::nullopt;
}

std::optional<int> SBASCorrectionStore::fastIodp(const Satellite& sat, const QDateTime& t) const {
   if (const auto* e = selectedFastEntry(sat, t)) {
      return e->iodp;
   }
   return std::nullopt;
}

std::optional<double> SBASCorrectionStore::fastPrc_m(const Satellite& sat, const QDateTime& t) const {
   if (const auto* e = selectedFastEntry(sat, t)) {
      return e->prc_m;
   }
   return std::nullopt;
}

std::optional<QDateTime> SBASCorrectionStore::fastLastUpdate(const Satellite& sat, const QDateTime& t) const {
   if (const auto* e = selectedFastEntry(sat, t)) {
      return e->recvTime;
   }
   return std::nullopt;
}

std::optional<io::FastPair> SBASCorrectionStore::fastCurrentPrevious(const Satellite& sat,
                                                                     const QDateTime& t) const {
   const auto* cur = selectedFastEntry(sat, t);

   if (!cur) {
      return std::nullopt;
   }

   auto it = fast_.find(sat);

   if (it == fast_.end()) {
      return std::nullopt;
   }

   const auto  maskIodp = activePrnMaskIodp(t);
   const auto& vec      = it.value();

   const FastEntry* prev = nullptr;

   for (auto rit = vec.crbegin(); rit != vec.crend(); ++rit) {
      const auto& e = *rit;

      // строго предыдущая по времени запись
      const QDateTime curRecv = cur->recvTime.isValid() ? cur->recvTime : cur->start;
      const QDateTime eRecv   = e.recvTime.isValid()   ? e.recvTime   : e.start;

      if (eRecv >= curRecv) {
         continue;
      }

      if (e.doNotUse || !qIsFinite(e.prc_m)) {
         continue;
      }

      if (maskIodp && (e.iodp >= 0) && (e.iodp != *maskIodp)) {
         continue;
      }

      prev = &e;
      break;
   }

   if (!prev) {
      return std::nullopt;
   }

   return FastPair{ *cur, *prev };
}

std::optional<DegrParams> SBASCorrectionStore::degradation(const Satellite& sat, const QDateTime& t) const {
   if (auto it = degr_.find(sat); (it != degr_.end())) {
      for (const auto& e : it.value()) {
         if ((t >= e.start) && (t < e.end)) {
            return e;
         }
      }
   }
   return std::nullopt;
}

const IonoBandSnapshot*SBASCorrectionStore::pickBandSnapshot(double ippLat, double ippLon, const QDateTime& t) const {
   Q_UNUSED(ippLat);
   Q_UNUSED(ippLon);
   Q_UNUSED(t);
   return nullptr; // больше не используется, заменено на getActiveGrid
}

// ---------------- IONO helpers/interp ----------------
QVector<SBASCorrectionStore::ActiveNode> SBASCorrectionStore::getActiveGrid(const QDateTime& t) const {
   QVector<ActiveNode> grid;

   // Собираем все активные узлы со всех зон, чтобы преодолеть границы Bands
   for (const auto& snaps : ionoBands_) {
      for (const auto& s : snaps) {
         if ((t >= s.start) && (t < s.end)) {
            for (auto it = s.nodesRad.begin(); it != s.nodesRad.end(); ++it) {
               grid.push_back({ it.value().first, it.value().second, s.band, s.iodi, it.key() });
            }
            break;
         }
      }
   }

   if (grid.isEmpty()) {
      qDebug() << "CRITICAL: Grid is empty for time" << t << "! Check parsed Type 18 masks.";
   } else {
      static bool once = false;

      if (!once) {
         qDebug() << "Grid loaded with" << grid.size() << "nodes for time" << t;
         once = true;
      }
   }

   return grid;
}

int SBASCorrectionStore::findCellCorners(const QVector<ActiveNode>& grid,
                                         const QDateTime&           t,
                                         double                     ippLat,
                                         double                     ippLon,
                                         QVector<IonoObs>&          outCorners) const {
   outCorners.clear();

   if (grid.isEmpty()) {
      return 0;
   }

   QSet<double> lats;

   for (const auto& node : grid) {
      lats.insert(node.lat_rad);
   }

   QList<double> slats = lats.values();
   std::sort(slats.begin(), slats.end());

   double lat_floor, lat_ceil;
   int    idx = std::upper_bound(slats.begin(), slats.end(), ippLat) - slats.begin();
   int    iu  = qBound(0, idx, (int)slats.size() - 1);
   int    il  = qMax(0, iu - 1);

   if (il == iu) {
      if (iu < slats.size() - 1) {
         iu++;
      } else if (il > 0) {
         il--;
      }
   }

   lat_floor = slats[il];
   lat_ceil  = slats[iu];

   auto getLonBoundsForLat = [&](double targetLat, double& lo_floor, double& lo_ceil) -> bool {
                                QList<double> lons;

                                for (const auto& node : grid) {
                                   if (std::fabs(node.lat_rad - targetLat) < 1e-9) {
                                      lons.push_back(node.lon_rad);
                                   }
                                }

                                if (lons.isEmpty()) {
                                   return false;
                                }

                                std::sort(lons.begin(), lons.end());
                                lons.erase(
                                   std::unique(lons.begin(), lons.end(), [](double a, double b) {
         return std::fabs(a - b) < 1e-9;
      }),
                                   lons.end()
                                   );

                                if (lons.size() == 1) {
                                   lo_floor = lons[0];
                                   lo_ceil  = lons[0];
                                   return true;
                                }

                                double best_floor     = lons.last();
                                double best_ceil      = lons.first();
                                double min_floor_diff = 1e9;
                                double min_ceil_diff  = 1e9;

                                for (double lon : lons) {
                                   double diff = normLonRad(ippLon - lon);

                                   if ((diff >= 0) && (diff < min_floor_diff)) {
                                      min_floor_diff = diff;
                                      best_floor     = lon;
                                   }

                                   if ((diff <= 0) && (std::fabs(diff) < min_ceil_diff)) {
                                      min_ceil_diff = std::fabs(diff);
                                      best_ceil     = lon;
                                   }
                                }

                                // Если IPP ровно на линии сетки — формируем коробку из 4 узлов
                                if (std::fabs(best_floor - best_ceil) < 1e-9) {
                                   for (int i = 0; i < lons.size(); ++i) {
                                      if (std::fabs(lons[i] - best_floor) < 1e-9) {
                                         best_ceil = (i < lons.size() - 1) ? lons[i + 1] : lons.first();
                                         break;
                                      }
                                   }
                                }

                                lo_floor = best_floor;
                                lo_ceil  = best_ceil;
                                return true;
                             };

   double lo1, lo2, lo3, lo4;

   if (!getLonBoundsForLat(lat_floor, lo1, lo2)) {
      return 0;
   }

   if (!getLonBoundsForLat(lat_ceil, lo3, lo4)) {
      return 0;
   }

   QVector<QPair<double, double> > candidates = {
      { lat_floor, lo1                                                                                  },
      { lat_floor, lo2                                                                                  },
      { lat_ceil,  lo3                                                                                  },
      { lat_ceil,  lo4                                                                                  }
   };

   for (const auto& cand : candidates) {
      bool foundInGrid = false;
      bool foundInObs  = false;
      bool isExpired   = false;
      bool isDoNotUse  = false;

      for (const auto& node : grid) {
         if ((std::fabs(node.lat_rad - cand.first) < 1e-9) &&
             (std::fabs(node.lon_rad - cand.second) < 1e-9)) {
            foundInGrid = true;

            IonoObs dummy;
            dummy.band    = node.band;
            dummy.iodi    = node.iodi;
            dummy.idPoint = node.idPoint;
            dummy.start   = t;

            auto it = std::upper_bound(
               ionoObs_.begin(),
               ionoObs_.end(),
               dummy,
               [](const IonoObs& a, const IonoObs& b) {
               if (a.band != b.band) {
                  return a.band < b.band;
               }

               if (a.iodi != b.iodi) {
                  return a.iodi < b.iodi;
               }

               if (a.idPoint != b.idPoint) {
                  return a.idPoint < b.idPoint;
               }
               return a.start < b.start;
            }
               );

            if (it != ionoObs_.begin()) {
               auto prevIt = it - 1;

               if ((prevIt->band == node.band) &&
                   (prevIt->iodi == node.iodi) &&
                   (prevIt->idPoint == node.idPoint)) {
                  foundInObs = true;

                  if ((t >= prevIt->start) && (t < prevIt->end)) {
                     if (prevIt->doNotUse) {
                        isDoNotUse = true;
                     } else {
                        bool alreadyAdded = false;

                        for (const auto& added : outCorners) {
                           if ((std::fabs(added.lat_rad - cand.first) < 1e-9) &&
                               (std::fabs(added.lon_rad - cand.second) < 1e-9)) {
                              alreadyAdded = true;
                              break;
                           }
                        }

                        if (!alreadyAdded) {
                           IonoObs kept = *prevIt;
                           kept.lat_rad = cand.first;
                           kept.lon_rad = cand.second;
                           outCorners.push_back(kept);
                        }

                        break;
                     }
                  } else {
                     isExpired = true;
                  }
               }
            }
         }
      }

      Q_UNUSED(foundInGrid);
      Q_UNUSED(foundInObs);
      Q_UNUSED(isExpired);
      Q_UNUSED(isDoNotUse);
   }

   return outCorners.size();
}

bool SBASCorrectionStore::ionoVerticalAt(double           ippLat,
                                         double           ippLon,
                                         const QDateTime& t,
                                         double&          vdelay_m,
                                         double&          var_v_m2) const {
   vdelay_m = 0.0;
   var_v_m2 = qQNaN();

   auto grid = getActiveGrid(t);

   if (grid.isEmpty()) {
      return false;
   }

   QVector<IonoObs> c;
   int n = findCellCorners(grid, t, ippLat, ippLon, c);

   if (n >= 1) {
      for (const auto& e : c) {
         if ((std::fabs(e.lat_rad - ippLat) <= 1e-10) &&
             (std::fabs(angDiffRad(e.lon_rad, ippLon)) <= 1e-10)) {
            vdelay_m = e.vdelay_m;
            var_v_m2 = e.sigma_v_m2;
            return true;
         }
      }
   }

   if (n < 3) {
      return false;
   }

   if (n == 4) {
      double lat_min = 1e9, lat_max = -1e9;

      for (const auto& e : c) {
         lat_min = qMin(lat_min, e.lat_rad);
         lat_max = qMax(lat_max, e.lat_rad);
      }

      QVector<IonoObs> bottom, top;

      for (const auto& e : c) {
         if (std::fabs(e.lat_rad - lat_min) < 1e-9) {
            bottom.push_back(e);
         } else {
            top.push_back(e);
         }
      }

      if ((bottom.size() != 2) || (top.size() != 2)) {
         return false;
      }

      auto sortByLon = [&](QVector<IonoObs>& row) {
                          if (normLonRad(row[1].lon_rad - row[0].lon_rad) < 0) {
                             std::swap(row[0], row[1]);
                          }
                       };

      sortByLon(bottom);
      sortByLon(top);

      IonoObs c11 = bottom[0], c21 = bottom[1], c12 = top[0], c22 = top[1];

      double dLon = angDiffRad(c11.lon_rad, c21.lon_rad);
      double x    = (dLon > 1e-9) ? (angDiffRad(c11.lon_rad, ippLon) / dLon) : 0.0;

      double dLat = c12.lat_rad - c11.lat_rad;
      double y    = (dLat > 1e-9) ? ((ippLat - c11.lat_rad) / dLat) : 0.0;

      double w11 = (1.0 - x) * (1.0 - y);
      double w21 = x * (1.0 - y);
      double w12 = (1.0 - x) * y;
      double w22 = x * y;

      vdelay_m = w11 * c11.vdelay_m + w21 * c21.vdelay_m + w12 * c12.vdelay_m + w22 * c22.vdelay_m;
      var_v_m2 = w11 * c11.sigma_v_m2 + w21 * c21.sigma_v_m2 + w12 * c12.sigma_v_m2 + w22 * c22.sigma_v_m2;
      return true;
   }

   if (n == 3) {
      auto area = [](double x1, double y1, double x2, double y2, double x3, double y3) {
                     return std::fabs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
                  };

      double x1 = 0, y1 = 0;
      double x2 = normLonRad(c[1].lon_rad - c[0].lon_rad), y2 = c[1].lat_rad - c[0].lat_rad;
      double x3 = normLonRad(c[2].lon_rad - c[0].lon_rad), y3 = c[2].lat_rad - c[0].lat_rad;
      double xp = normLonRad(ippLon - c[0].lon_rad), yp = ippLat - c[0].lat_rad;

      double A = area(x1, y1, x2, y2, x3, y3);

      if (A < 1e-12) {
         return false;
      }

      double w1 = area(x2, y2, x3, y3, xp, yp) / A;
      double w2 = area(x1, y1, x3, y3, xp, yp) / A;
      double w3 = area(x1, y1, x2, y2, xp, yp) / A;

      vdelay_m = w1 * c[0].vdelay_m + w2 * c[1].vdelay_m + w3 * c[2].vdelay_m;
      var_v_m2 = w1 * c[0].sigma_v_m2 + w2 * c[1].sigma_v_m2 + w3 * c[2].sigma_v_m2;
      return true;
   }

   return false;
}

std::optional<double> SBASCorrectionStore::giveVar_m2(double ippLat, double ippLon, const QDateTime& t) const {
   double bestD = std::numeric_limits<double>::max();
   double best  = -1;

   for (const auto& o : ionoObs_) {
      if ((t >= o.start) && (t < o.end)) {
         if (double d = haversine(ippLat, ippLon, o.lat_rad, o.lon_rad); (d < bestD)) {
            bestD = d;
            best  = o.sigma_v_m2;
         }
      }
   }

   return best >= 0 ? std::optional<double> (best) : std::nullopt;
}

std::optional<double> SBASCorrectionStore::giveVDelay_m(double ippLat, double ippLon, const QDateTime& t) const {
   double bestD = std::numeric_limits<double>::max();
   double best  = -1;

   for (const auto& o : ionoObs_) {
      if ((t >= o.start) && (t < o.end)) {
         if (double d = haversine(ippLat, ippLon, o.lat_rad, o.lon_rad); (d < bestD)) {
            bestD = d;
            best  = o.vdelay_m;
         }
      }
   }

   return best >= 0 ? std::optional<double> (best) : std::nullopt;
}

std::optional<LongTermCorrectionEntry> SBASCorrectionStore::getLongTermCorrection(const Satellite& sat, const QDateTime& t) const {
   if (const auto* e = selectedLongTermEntry(sat, t)) {
      return *e;
   }
   return std::nullopt;
}

std::optional<double> SBASCorrectionStore::gpsGlonassOffset(const QDateTime& t) const {
   for (const auto& iv : gpsGloOffsets_) {
      if ((t >= iv.start) && (t < iv.end)) {
         return iv.rawDeltaAGlonass_s;
      }
   }
   return std::nullopt;
}
