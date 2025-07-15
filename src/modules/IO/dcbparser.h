#ifndef DCBPARSER_H
#define DCBPARSER_H

#include <QString>

#include <inav/Satellite>

namespace io {
class DCBParser {
public:

   /**
    * Парсит секцию +BIAS/SOLUTION → -BIAS/SOLUTION,
    * возвращает map<Satellite, bias_ns>
    */
   static std::unordered_map<Satellite, double> parseGlonassL3L1Bias(const QString& filePath);
};
}

#endif // DCBPARSER_H
