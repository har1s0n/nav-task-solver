#ifndef APPLICATION_H
#define APPLICATION_H

#include "modules/IO/datamanager.h"

struct ApplicationConfig {
   QString sp3Path;
   QString rinexNavPath;
   QString sbasPath;
   QString dcbPath;

   // Диапозон сетки в заданном диапозоне
   // Например: широта от 35° до 75°, долгота от 10° до 150°, шаг по долготе 0.5°
   double lonStepDeg = 0.0;
   double minLat     = 0.0;
   double maxLat     = 0.0;
   double minLon     = 0.0;
   double maxLon     = 0.0;

   std::optional<QString> igsClockPath;
   std::optional<QString> rawObsPath;
   std::optional<QString> rinexObsPath;

   // флаги, режимы, параметры
   bool applyCorrections          = true;
   bool refineEphemeris           = true;
   bool solveNavigation           = true;
   bool redefiningGridCoordinates = false;
};

class Application {
public:

   bool        initialize(const ApplicationConfig& config);
   int         run();
   QJsonObject getResultsAsJson() const;

private:

   static QVector<QDateTime> extractEpochs(const sp3::SP3_FILE* f) noexcept;
   static QVector<QDateTime> extractEpochs(const rinex::RINEX_FILE* f) noexcept;
   static QVector<QDateTime> intersectEpochs(const QVector<QDateTime>& a,
                                             const QVector<QDateTime>& b) noexcept;

private:

   ApplicationConfig cfg_;
   std::vector<std::unique_ptr<pipeline::IModule> > modules_;
   pipeline::Context ctx_;
};

#endif // APPLICATION_H
