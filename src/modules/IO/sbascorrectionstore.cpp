#include "sbascorrectionstore.h"
#include <QFile>
#include <QTextStream>
#include <QtMath>
#include <limits>
#include <algorithm>

using namespace io;
using namespace sbas;


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
      qWarning() << "Не удалось открыть файл:" << path;
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
         result.msg->recvTime = recvTime; // при необходимости позже привести к GPST
         parsedMessages_[result.msg->getTypeMsg()].append(result.msg);
         ++parsed;
      }
   }
   qDebug() << "[Hex] Прочитано:" << total << ", успешно:" << parsed;
   return parsed > 0;
}

bool SBASCorrectionStore::loadCsv(const QString& path) {
   QFile file(path);

   if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      qWarning() << "Failed to open:" << path << ", error:" << file.errorString();
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
   qDebug() << "[CSV] Прочитано:" << total << ", успешно:" << parsed;
   return parsed > 0;
}

bool SBASCorrectionStore::loadDb() {
   qWarning() << "SBASCorrectionStore::loadDb(): нет встроенной реализации; используйте setMessages(...)";
   return false;
}

// ---------------- helpers: PRN->Satellite ----------------
std::optional<Satellite> SBASCorrectionStore::resolveByLocalIndex(int localIdx, const QDateTime& t) const {
   for (const auto& iv : prnMasks_) {
      if ((t < iv.start) || (t >= iv.end)) {
         continue;
      }
      auto it = iv.mapLocalIdxToSat.find(localIdx);

      if (it == iv.mapLocalIdxToSat.end()) {
         return std::nullopt;
      }
      return it.value();
   }
   return std::nullopt;
}

// ---------------- Build timelines ----------------
void SBASCorrectionStore::buildTimelines() {
   prnMasks_.clear();
   udre_.clear();
   fast_.clear();
   degr_.clear();
   ionoBands_.clear();
   ionoObs_.clear();
   longTermBySat_.clear();

   // --------- PRN MASK (Type 1) ---------
   const auto prnMsgs = parsed_.value(MESSAGE_TYPE::PRN_MASK);

   for (int i = 0; i < prnMsgs.size(); ++i) {
      auto m = std::dynamic_pointer_cast<MSG_PRN_MASK> (prnMsgs[i]);

      if (!m) {
         continue;
      }

      PrnMaskInterval iv;
      iv.start = m->recvTime;
      iv.end   = (i + 1 < prnMsgs.size() ? prnMsgs[i + 1]->recvTime : m->recvTime.addSecs(600));

      const int N = qMin(m->activePsps.size(), m->satelites.size());

      for (int k = 0; k < N; ++k) {
         const int   idx = m->activePsps[k];
         const auto& s   = m->satelites[k];
         SatelliteSystem::TYPE sys;

         switch (s.systemSat) {
           case PURPOSE_SYSTEM::GPS: {
              sys = SatelliteSystem::TYPE::GPS;
              break;
           }
           case PURPOSE_SYSTEM::GLONASS: {
              sys = SatelliteSystem::TYPE::GLONASS;
              break;
           }
           case PURPOSE_SYSTEM::GEO_WAAS_PRN: {
              sys = SatelliteSystem::TYPE::SBAS;
              break;
           }
           default: {
              sys = SatelliteSystem::TYPE::UNKNOWN;
              break;
           }
         }
         iv.mapLocalIdxToSat.insert(idx, Satellite(s.satId, sys));
      }
      prnMasks_.push_back(std::move(iv));
   }

   // --------- FAST (2..5) и MIXED(24) → fast timeline + UDRE события ---------
   auto ingestFast = [&](MESSAGE_TYPE ty){
                        const auto v = parsed_.value(ty);

                        for (const auto& r : v) {
                           auto m = std::dynamic_pointer_cast<MSG_FAST_CORRECTIONS> (r);

                           if (!m) {
                              continue;
                           }
                           const auto t = m->recvTime;

                           for (const auto& s : m->satelites) {
                              auto sat = resolveByLocalIndex((int)s.satNum, t);

                              if (!sat || s.doNotUse) {
                                 continue;
                              }

                              fast_[*sat].push_back({ t, t.addSecs(300), m->iodf, m->iodp });

                              UdreEntry ue;
                              ue.start     = t; ue.end = t.addSecs(600);
                              ue.source    = ty; ue.iodf = m->iodf; ue.iodp = m->iodp;
                              ue.udreIndex = s.UDREI;
                              ue.bound_m   = s.UDREI_meters;
                              ue.sigma_m2  = s.doNotUse ? qInf() : s.sigma_UDREI;
                              udre_[*sat].push_back(std::move(ue));
                           }
                        }
                     };
   ingestFast(MESSAGE_TYPE::FAST_CORRECTIONS_1);
   ingestFast(MESSAGE_TYPE::FAST_CORRECTIONS_2);
   ingestFast(MESSAGE_TYPE::FAST_CORRECTIONS_3);
   ingestFast(MESSAGE_TYPE::FAST_CORRECTIONS_4);

   // MIXED 24 — fast часть
   for (const auto& r : parsed_.value(MESSAGE_TYPE::MIXED_CORRECTIONS_SATELLITE_ERROR)) {
      auto m = std::dynamic_pointer_cast<MSG_MIXED_CORRECTIONS_SATELLITE_ERROR> (r);

      if (!m) {
         continue;
      }
      const auto  t  = m->recvTime;
      const auto& fc = m->fast_correction;

      for (const auto& s : fc.satelites) {
         auto sat = resolveByLocalIndex((int)s.satNum, t);

         if (!sat || s.doNotUse) {
            continue;
         }

         fast_[*sat].push_back({ t, t.addSecs(300), fc.iodf, fc.iodp });

         UdreEntry ue;
         ue.start     = t; ue.end = t.addSecs(600);
         ue.source    = MESSAGE_TYPE::MIXED_CORRECTIONS_SATELLITE_ERROR;
         ue.iodf      = fc.iodf; ue.iodp = fc.iodp;
         ue.udreIndex = s.UDREI;
         ue.bound_m   = s.UDREI_meters;
         ue.sigma_m2  = s.doNotUse ? qInf() : s.sigma_UDREI;
         udre_[*sat].push_back(std::move(ue));
      }
   }

   // --------- INTEGRITY (Type 6) → UDRE ---------
   for (const auto& r : parsed_.value(MESSAGE_TYPE::INTEGRITY_INFORMATION)) {
      auto m = std::dynamic_pointer_cast<MSG_INTEGRITY_INFORMATION> (r);

      if (!m) {
         continue;
      }
      const auto t = m->recvTime;

      for (const auto& s : m->satelites) {
         auto sat = resolveByLocalIndex((int)s.satNum, t);

         if (!sat || s.doNotUse) {
            continue;
         }

         UdreEntry ue;
         ue.start     = t; ue.end = t.addSecs(600);
         ue.source    = MESSAGE_TYPE::INTEGRITY_INFORMATION;
         ue.iodf      = -1; ue.iodp = -1;
         ue.udreIndex = s.UDREI;
         ue.bound_m   = s.UDREI_meters;
         ue.sigma_m2  = s.sigma_UDREI;
         udre_[*sat].push_back(std::move(ue));
      }
   }

   // --------- DEG (Type 7) ---------
   for (const auto& r : parsed_.value(MESSAGE_TYPE::FAST_CORRECTION_DEGRADATION_FACTOR)) {
      auto m = std::dynamic_pointer_cast<MSG_FAST_CORRECTION_DEGRADATION_FACTOR> (r);

      if (!m) {
         continue;
      }
      const auto t = m->recvTime;

      for (const auto& s : m->satelites) {
         auto sat = resolveByLocalIndex((int)s.satNum, t);

         if (!sat) {
            continue;
         }

         if (s.doNotUse) {
            continue;
         }

         DegrParams d;
         d.start            = t; d.end = t.addSecs(600);
         d.a                = s.a;
         d.updateInterval_s = s.updateInterval;
         degr_[*sat].push_back(std::move(d));
      }
   }

   // --------- LONG-TERM (Type 25) ---------
   for (const auto& r : parsed_.value(MESSAGE_TYPE::LONG_TERM_SATELLITE_ERROR_CORRECTIONS)) {
      auto m = std::dynamic_pointer_cast<MSG_LONG_TERM_SATELLITE_ERROR_CORRECTIONS> (r);

      if (!m) {
         continue;
      }
      const auto t = m->recvTime;

      // ---- Code 1 (со скоростью, t0 есть) ----
      if (!m->satellites_code_1.empty()) {
         for (const auto& s : m->satellites_code_1) {
            auto sat = resolveByLocalIndex(s.prn, t);

            if (!sat) {
               continue;
            }

            LongTermCorrectionEntry e;
            e.start = t;
            e.end   = t.addSecs(7200);
            e.iodp  = (s.iodp >= 0 ? s.iodp : -1);
            e.t0    = (s.t0 >= 0 ? s.t0 : QTime(0, 0).secsTo(t.time()));

            e.deltaPos    = { s.deltaEcef.x, s.deltaEcef.y, s.deltaEcef.z };
            e.hasVelocity = true;
            e.deltaVel    = { s.deltaRoc.x, s.deltaRoc.y, s.deltaRoc.z };

            // часы уже в секундах
            e.deltaAf0 = s.delta_a_f0;
            e.deltaAf1 = s.delta_a_f1;
            e.source   =  LongTermCorrectionEntry::Source::From25;

            longTermBySat_[*sat].push_back(std::move(e));
         }
      }

      // ---- Code 0 (без скорости, t0 может отсутствовать) ----
      if (!m->satellites_code_0.empty()) {
         for (const auto& s : m->satellites_code_0) {
            auto sat = resolveByLocalIndex(s.prn, t);

            if (!sat) {
               continue;
            }

            LongTermCorrectionEntry e;
            e.start = t;
            e.end   = t.addSecs(7200);
            e.iodp  = (s.iodp >= 0 ? s.iodp : -1);
            e.t0    = QTime(0, 0).secsTo(t.time()); // t0 в VELOCITY_CODE_0 нет — берём по recvTime

            e.deltaPos    = { s.deltaEcef.x, s.deltaEcef.y, s.deltaEcef.z };
            e.hasVelocity = false;
            e.deltaVel    = { 0.0, 0.0, 0.0 };

            e.deltaAf0 = s.delta_a_f0;
            e.deltaAf1 = 0.0; // в Code 0 нет дрейфа
            e.source   =  LongTermCorrectionEntry::Source::From25;

            longTermBySat_[*sat].push_back(std::move(e));
         }
      }
   }

   // --------- LONG-TERM из Type 24 (mixed) ---------
   for (const auto& r : parsed_.value(MESSAGE_TYPE::MIXED_CORRECTIONS_SATELLITE_ERROR)) {
      auto m = std::dynamic_pointer_cast<MSG_MIXED_CORRECTIONS_SATELLITE_ERROR> (r);

      if (!m) {
         continue;
      }
      const auto t = m->recvTime;

      // Унифицированная обработка массива m->satellites (база VELOCITY_CODE)
      for (const auto& basePtr : m->satellites) {
         // попробуем Code1
         if (auto c1 = std::dynamic_pointer_cast<VELOCITY_CODE_1> (basePtr)) {
            auto sat = resolveByLocalIndex(c1->prn, t);

            if (!sat) {
               continue;
            }

            LongTermCorrectionEntry e;
            e.start       = t; e.end = t.addSecs(7200);
            e.iodp        = m->iodp;
            e.t0          = (c1->t0 >= 0 ? c1->t0 : QTime(0, 0).secsTo(t.time()));
            e.deltaPos    = { c1->deltaEcef.x, c1->deltaEcef.y, c1->deltaEcef.z };
            e.hasVelocity = true;
            e.deltaVel    = { c1->deltaRoc.x, c1->deltaRoc.y, c1->deltaRoc.z };
            e.deltaAf0    = c1->delta_a_f0; // сек
            e.deltaAf1    = c1->delta_a_f1; // сек/сек
            e.source      =  LongTermCorrectionEntry::Source::From24;
            longTermBySat_[*sat].push_back(std::move(e));
            continue;
         }

         // иначе Code0
         if (auto c0 = std::dynamic_pointer_cast<VELOCITY_CODE_0> (basePtr)) {
            auto sat = resolveByLocalIndex(c0->prn, t);

            if (!sat) {
               continue;
            }

            LongTermCorrectionEntry e;
            e.start       = t; e.end = t.addSecs(7200);
            e.iodp        = m->iodp;
            e.t0          = QTime(0, 0).secsTo(t.time());
            e.deltaPos    = { c0->deltaEcef.x, c0->deltaEcef.y, c0->deltaEcef.z };
            e.hasVelocity = false;
            e.deltaAf0    = c0->delta_a_f0; // сек
            e.deltaAf1    = 0.0;            // нет в Code0
            e.source      =  LongTermCorrectionEntry::Source::From24;
            longTermBySat_[*sat].push_back(std::move(e));
         }
      }
   }

   // --------- IONO (Type 18) маски по band/IODI ---------
   const auto igp18 = parsed_.value(MESSAGE_TYPE::IONOSPHERIC_GRID_POINT_MASK);
   QMultiMap<int, int> orderByBand; // band -> index в igp18

   for (int i = 0; i < igp18.size(); ++i) {
      auto m = std::dynamic_pointer_cast<MSG_IONOSPHERIC_GRID_POINT_MASK> (igp18[i]);

      if (!m) {
         continue;
      }
      orderByBand.insert((int)m->idBand, i);
   }

   for (auto it = orderByBand.begin(); it != orderByBand.end();) {
      int band = it.key(); auto range = orderByBand.equal_range(band);

      for (auto jt = range.first; jt != range.second; ++jt) {
         int  idx = jt.value();
         auto m   = std::dynamic_pointer_cast<MSG_IONOSPHERIC_GRID_POINT_MASK> (igp18[idx]);

         if (!m) {
            continue;
         }

         IonoBandSnapshot snap; snap.band = band; snap.iodi = (int)m->iod; snap.start = m->recvTime;
         auto next                        = std::next(jt);
         snap.end = (next != range.second) ? igp18[next.value()]->recvTime : m->recvTime.addSecs(600);

         for (const auto& nd : m->coordinatesRange) {
            if ((nd.idPoint < 1) || (nd.idPoint > 201)) {
               continue;
            }
            double lat_rad = qDegreesToRadians((double)nd.lat);
            double lon_rad = qDegreesToRadians((double)nd.lon);
            snap.nodesRad.insert(nd.idPoint, { lat_rad, wrapLon(lon_rad) });
         }
         ionoBands_[band].push_back(std::move(snap));
      }
      it = range.second;
   }

   // --------- IONO (Type 26) наблюдения по узлам ---------
   const auto igp26 = parsed_.value(MESSAGE_TYPE::IONOSPHERIC_DELAY_CORRECTIONS);

   for (const auto& r : igp26) {
      auto m = std::dynamic_pointer_cast<MSG_IONOSPHERIC_DELAY_CORRECTIONS> (r);

      if (!m) {
         continue;
      }
      const auto t      = m->recvTime;
      const int  band   = (int)m->numberBand;
      const int  iodi26 = (int)m->iod;

      const auto snaps             = ionoBands_.value(band);
      const IonoBandSnapshot* snap = nullptr;

      for (const auto& s : snaps) {
         if ((t >= s.start) && (t < s.end)) {
            snap = &s; break;
         }
      }

      if (!snap || (snap->iodi != iodi26)) {
         continue; // строгая проверка IODI
      }

      for (const auto& bp : m->points) {
         auto itNode = snap->nodesRad.find((int)bp.idPoint);

         if (itNode == snap->nodesRad.end()) {
            continue;
         }

         IonoObs obs; obs.start = t; obs.end = t.addSecs(300);
         obs.band     = band; obs.iodi = iodi26; obs.idPoint = (int)bp.idPoint;
         obs.lat_rad  = itNode.value().first; obs.lon_rad = itNode.value().second;
         obs.vdelay_m = bp.igp; obs.sigma_v_m2 = bp.sigma_give; obs.doNotUse = bp.doNotUse;
         ionoObs_.push_back(std::move(obs));
      }
   }
}

// ---------------- UDRE getters ----------------
std::optional<double> SBASCorrectionStore::udreSigma_m2(const Satellite& sat, const QDateTime& t) const {
   auto it = udre_.find(sat);

   if (it == udre_.end()) {
      return std::nullopt;
   }
   const auto& v = it.value();

   for (const auto& e : v) {
      if ((t >= e.start) && (t < e.end)) {
         return e.sigma_m2;
      }
   }
   return std::nullopt;
}

std::optional<double> SBASCorrectionStore::udreBound_m(const Satellite& sat, const QDateTime& t) const {
   auto it = udre_.find(sat);

   if (it == udre_.end()) {
      return std::nullopt;
   }
   const auto& v = it.value();

   for (const auto& e : v) {
      if ((t >= e.start) && (t < e.end)) {
         return e.bound_m;
      }
   }
   return std::nullopt;
}

std::optional<int> SBASCorrectionStore::udreIndex(const Satellite& sat, const QDateTime& t) const {
   auto it = udre_.find(sat);

   if (it == udre_.end()) {
      return std::nullopt;
   }
   const auto& v = it.value();

   for (const auto& e : v) {
      if ((t >= e.start) && (t < e.end)) {
         return e.udreIndex;
      }
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
      const double dt = std::max(0, (int)lu->secsTo(t));
      var += (d->a * dt) * (d->a * dt); // σ_eff^2 = σ^2 + (a*dt)^2
   }
   return var;
}

// ---------------- FAST getters ----------------
std::optional<int> SBASCorrectionStore::fastIodf(const Satellite& sat, const QDateTime& t) const {
   auto it = fast_.find(sat);

   if (it == fast_.end()) {
      return std::nullopt;
   }
   const auto& v = it.value();

   for (const auto& e: v) {
      if ((t >= e.start) && (t < e.end)) {
         return e.iodf;
      }
   }
   return std::nullopt;
}

std::optional<int> SBASCorrectionStore::fastIodp(const Satellite& sat, const QDateTime& t) const {
   auto it = fast_.find(sat);

   if (it == fast_.end()) {
      return std::nullopt;
   }
   const auto& v = it.value();

   for (const auto& e: v) {
      if ((t >= e.start) && (t < e.end)) {
         return e.iodp;
      }
   }
   return std::nullopt;
}

std::optional<QDateTime> SBASCorrectionStore::fastLastUpdate(const Satellite& sat, const QDateTime& t) const {
   auto it = fast_.find(sat);

   if (it == fast_.end()) {
      return std::nullopt;
   }
   const auto& v = it.value();
   QDateTime   best;

   for (const auto& e: v) {
      if ((t >= e.start) && (t < e.end)) {
         if (!best.isValid() || (e.start > best)) {
            best = e.start;
         }
      }
   }

   if (best.isValid()) {
      return best;
   }
   return std::nullopt;
}

// ---------------- DEG getter ----------------
std::optional<DegrParams> SBASCorrectionStore::degradation(const Satellite& sat, const QDateTime& t) const {
   auto it = degr_.find(sat);

   if (it == degr_.end()) {
      return std::nullopt;
   }
   const auto& v = it.value();

   for (const auto& e: v) {
      if ((t >= e.start) && (t < e.end)) {
         return e;
      }
   }
   return std::nullopt;
}

// ---------------- IONO helpers/interp ----------------
const IonoBandSnapshot*SBASCorrectionStore::pickBandSnapshot(double ippLat, double ippLon, const QDateTime& t) const {
   const IonoBandSnapshot* best = nullptr; double bestD = 1e9;

   for (auto it = ionoBands_.begin(); it != ionoBands_.end(); ++it) {
      const auto& snaps = it.value();

      for (const auto& s : snaps) {
         if ((t < s.start) || (t >= s.end)) {
            continue;
         }

         for (auto jt = s.nodesRad.begin(); jt != s.nodesRad.end(); ++jt) {
            double d = haversine(ippLat, ippLon, jt.value().first, jt.value().second);

            if (d < bestD) {
               bestD = d; best = &s;
            }
         }
      }
   }
   return best;
}

int SBASCorrectionStore::findCellCorners(const IonoBandSnapshot& snap, const QDateTime& t,
                                         double ippLat, double ippLon, QVector<IonoObs>& outCorners) const {
   outCorners.clear();

   QSet<double> lats, lons;
   lats.reserve(snap.nodesRad.size());
   lons.reserve(snap.nodesRad.size());

   for (auto it = snap.nodesRad.begin(); it != snap.nodesRad.end(); ++it) {
      lats.insert(it.value().first);
      lons.insert(it.value().second);
   }
   QList<double> slats = lats.values();
   QList<double> slons = lons.values();
   std::sort(slats.begin(), slats.end());
   std::sort(slons.begin(), slons.end());

   auto lower_or_equal = [](const QList<double>& v, double x){
                            int i = std::lower_bound(v.begin(), v.end(), x) - v.begin();
                            return i == v.size() ? i - 1 : (v[i] == x ? i : i - 1);
                         };
   auto upper_or_equal = [](const QList<double>& v, double x){
                            int i = std::lower_bound(v.begin(), v.end(), x) - v.begin(); return i < 0 ? 0 : i;
                         };

   int il = qMax(0, lower_or_equal(slats, ippLat));
   int iu = qMin(slats.size() - 1, qMax(il, upper_or_equal(slats, ippLat)));
   int jl = qMax(0, lower_or_equal(slons, ippLon));
   int ju = qMin(slons.size() - 1, qMax(jl, upper_or_equal(slons, ippLon)));

   QVector<IonoObs> corners;

   for (double la : { slats[il], slats[iu] }) {
      for (double lo : { slons[jl], slons[ju] }) {
         int foundId = -1;

         for (auto it = snap.nodesRad.begin(); it != snap.nodesRad.end(); ++it) {
            if (qFuzzyCompare(it.value().first, la) && qFuzzyCompare(it.value().second, lo)) {
               foundId = it.key(); break;
            }
         }

         if (foundId < 0) {
            continue;
         }

         for (const auto& o : ionoObs_) {
            if ((o.band != snap.band) || (o.idPoint != foundId)) {
               continue;
            }

            if ((t < o.start) || (t >= o.end)) {
               continue;
            }

            if (o.doNotUse) {
               continue;
            }

            IonoObs kept = o;
            kept.lat_rad = la;
            kept.lon_rad = lo;
            corners.push_back(kept);

            break;
         }
      }
   }

   QSet<int> used;
   QVector<IonoObs> uniq;

   for (const auto& c : corners) {
      if (used.contains(c.idPoint)) {
         continue;
      }
      used.insert(c.idPoint); uniq.push_back(c);
   }
   outCorners.swap(uniq);

   return outCorners.size();
}

bool SBASCorrectionStore::ionoVerticalAt(double ippLat, double ippLon, const QDateTime& t,
                                         double& vdelay_m, double& var_v_m2) const {
   vdelay_m = 0.0;
   var_v_m2 = qQNaN();

   const IonoBandSnapshot* snap = pickBandSnapshot(ippLat, ippLon, t);

   if (!snap) {
      return false;
   }

   QVector<IonoObs> c;
   int n = findCellCorners(*snap, t, ippLat, ippLon, c);

   if (n >= 1) {
      constexpr double EPS_LAT = 1e-10; // ~1e-10 рад ~ 6e-9 град
      constexpr double EPS_LON = 1e-10;

      for (const auto& e : c) {
         if ((std::fabs(e.lat_rad - ippLat) <= EPS_LAT) &&
             (angDiffRad(e.lon_rad, ippLon) <= EPS_LON)) {
            vdelay_m = e.vdelay_m;
            var_v_m2 = e.sigma_v_m2;
            return true;
         }
      }
   }

   if (n < 3) {
      return false;
   }

   auto calc_xy = [&](double la, double lo){
                     double la_min = c[0].lat_rad;
                     double la_max = c[0].lat_rad;
                     double lo_min = c[0].lon_rad;
                     double lo_max = c[0].lon_rad;

                     for (const auto& e : c) {
                        la_min = qMin(la_min, e.lat_rad);
                        la_max = qMax(la_max, e.lat_rad);
                        lo_min = qMin(lo_min, e.lon_rad);
                        lo_max = qMax(lo_max, e.lon_rad);
                     }
                     double x = (lo_max == lo_min) ? 0.5 : (lo - lo_min) / (lo_max - lo_min);
                     double y = (la_max == la_min) ? 0.5 : (la - la_min) / (la_max - la_min);

                     return QPair<double, double> (x, y);
                  };
   auto   xy = calc_xy(ippLat, ippLon);
   double x = xy.first, y = xy.second;

   if (n == 4) {
      double la_min = 1e9;
      double la_max = -1e9;
      double lo_min = 1e9;
      double lo_max = -1e9;

      for (const auto& e : c) {
         la_min = qMin(la_min, e.lat_rad);
         la_max = qMax(la_max, e.lat_rad);
         lo_min = qMin(lo_min, e.lon_rad);
         lo_max = qMax(lo_max, e.lon_rad);
      }
      auto pick = [&](double la, double lo){
                     for (const auto& e : c) {
                        if (qFuzzyCompare(e.lat_rad, la) && qFuzzyCompare(e.lon_rad, lo)) {
                           return e;
                        }
                     }
                     return c[0];
                  };
      auto   c11 = pick(la_min, lo_min);
      auto   c21 = pick(la_min, lo_max);
      auto   c12 = pick(la_max, lo_min);
      auto   c22 = pick(la_max, lo_max);
      double W11 = (1 - x) * (1 - y), W21 = x * (1 - y), W12 = (1 - x) * y, W22 = x * y;

      vdelay_m = W11 * c11.vdelay_m + W21 * c21.vdelay_m + W12 * c12.vdelay_m + W22 * c22.vdelay_m;
      var_v_m2 = W11 * W11 * c11.sigma_v_m2 + W21 * W21 * c21.sigma_v_m2 + W12 * W12 * c12.sigma_v_m2 + W22 * W22 * c22.sigma_v_m2;

      return true;
   }

   // n==3 → барицентрические веса
   auto area = [](double x1, double y1, double x2, double y2, double x3, double y3){
                  return fabs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
               };
   QVector<QPair<double, double> > xyc;

   for (const auto& e : c) {
      xyc.push_back(calc_xy(e.lat_rad, e.lon_rad));
   }

   double A  = area(0, 0, 1, 0, 0, 1);
   double w1 = area(xyc[1].first, xyc[1].second, xyc[2].first, xyc[2].second, x, y) / A;
   double w2 = area(xyc[0].first, xyc[0].second, xyc[2].first, xyc[2].second, x, y) / A;
   double w3 = area(xyc[0].first, xyc[0].second, xyc[1].first, xyc[1].second, x, y) / A;
   double S  = w1 + w2 + w3;

   if (S <= 0) {
      return false;
   }

   w1      /= S; w2 /= S; w3 /= S;
   vdelay_m = w1 * c[0].vdelay_m + w2 * c[1].vdelay_m + w3 * c[2].vdelay_m;
   var_v_m2 = w1 * w1 * c[0].sigma_v_m2 + w2 * w2 * c[1].sigma_v_m2 + w3 * w3 * c[2].sigma_v_m2;

   return true;
}

// ---------------- GIVE (NN) ----------------
std::optional<double> SBASCorrectionStore::giveVar_m2(double ippLat, double ippLon, const QDateTime& t) const {
   double bestD = std::numeric_limits<double>::max();
   double best  = -1;

   for (const auto& o : ionoObs_) {
      if ((t < o.start) || (t >= o.end)) {
         continue;
      }
      double d = haversine(ippLat, ippLon, o.lat_rad, o.lon_rad);

      if (d < bestD) {
         bestD = d; best = o.sigma_v_m2;
      }
   }
   return (best >= 0) ? std::optional<double> (best) : std::nullopt;
}

std::optional<double> SBASCorrectionStore::giveVDelay_m(double ippLat, double ippLon, const QDateTime& t) const {
   double bestD = std::numeric_limits<double>::max();
   double best  = -1;

   for (const auto& o : ionoObs_) {
      if ((t < o.start) || (t >= o.end)) {
         continue;
      }
      double d = haversine(ippLat, ippLon, o.lat_rad, o.lon_rad);

      if (d < bestD) {
         bestD = d; best = o.vdelay_m;
      }
   }
   return (best >= 0) ? std::optional<double> (best) : std::nullopt;
}

std::optional<LongTermCorrectionEntry>
SBASCorrectionStore::getLongTermCorrection(const Satellite& sat, const QDateTime& t) const {
   struct Cand {
      LongTermCorrectionEntry e;
      int                     score;
      qint64                  ts;
      bool                    hasVel;
      bool                    from24;
   };
   QVector<Cand> cs;
   auto fast = fastIodp(sat, t);

   auto it = longTermBySat_.find(sat);

   if (it != longTermBySat_.end()) {
      for (const auto& e : it.value()) {
         if ((t < e.start) || (t >= e.end)) {
            continue;
         }

         int sc = 0;

         if (fast && (e.iodp >= 0)) {
            sc = (*fast == e.iodp) ? 2 : 0;
         }           else if (e.iodp >= 0) {
            sc = 1;
         }

         cs.push_back({ e, sc, e.start.toSecsSinceEpoch(), e.hasVelocity,
                        e.source == LongTermCorrectionEntry::Source::From24 });
      }
   }

   if (cs.isEmpty()) {
      return std::nullopt;
   }

   std::sort(cs.begin(), cs.end(), [](const Cand& a, const Cand& b){
      if (a.score  != b.score) {
         return a.score  > b.score; // 1) IODP match
      }

      if (a.ts     != b.ts) {
         return a.ts     > b.ts; // 2) свежее
      }

      if (a.hasVel != b.hasVel) {
         return a.hasVel; // 3) есть скорость
      }

      if (a.from24 != b.from24) {
         return a.from24; // 4) 24 лучше 25
      }
      return false;
   });

   return cs.front().e;
}

// ---------------- Time offset getter ----------------
std::optional<double> SBASCorrectionStore::gpsGlonassOffset(const QDateTime& t) const {
   for (const auto& iv : gpsGloOffsets_) {
      if ((t >= iv.start) && (t < iv.end)) {
         return iv.gpsMinusGlonass_s;
      }
   }
   return std::nullopt;
}
