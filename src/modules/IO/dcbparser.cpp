#include "dcbparser.h"

#include <QFile>
#include <QRegularExpression>

std::unordered_map<Satellite, double> io::DCBParser::parseGlonassL3L1Bias(const QString& filePath) {
   std::unordered_map<Satellite, double> result;

   QFile file(filePath);

   if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      qWarning() << "[DCBParser] Не удалось открыть файл:" << filePath;
      return result;
   }

   QTextStream in(&file);
   bool inBiasBlock = false;

   while (!in.atEnd()) {
      const QString line = in.readLine();

      if (line.startsWith("+BIAS/SOLUTION")) {
         inBiasBlock = true;
         continue;
      }

      if (line.startsWith("-BIAS/SOLUTION")) {
         break;
      }

      if (!inBiasBlock || line.startsWith("*") || line.trimmed().isEmpty()) {
         continue;
      }

      // Извлечение полей по фиксированным позициям
      QString sys      = line.mid(6, 1);             // буква системы: "R"
      QString svNumStr = line.mid(12, 2);            // номер НКА: "01", "10" и т.д.
      QString obs1     = line.mid(25, 3).trimmed();  // "C2P"
      QString obs2     = line.mid(30, 3).trimmed();  // "C1C"
      QString valueStr = line.mid(85, 12).trimmed(); // Значение смещения в наносекундах

      if ((sys != "R") || (obs1 != "C1C") || (obs2 != "C2P")) {
         continue;
      }

      bool   ok1   = false;
      ushort svNum = svNumStr.toUShort(&ok1);

      if (!ok1) {
         continue;
      }

      bool   ok2    = false;
      double biasNs = valueStr.toDouble(&ok2);

      if (!ok2) {
         continue;
      }

      Satellite sat(svNum, SatelliteSystem::TYPE::GLONASS);
      result[sat] = biasNs;
   }

   qDebug() << "[DCBParser] Прочитано смещений L3-L1 для" << result.size() << "ГЛОНАСС спутников.";
   return result;
}
