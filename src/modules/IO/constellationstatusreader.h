#ifndef CONSTELLATIONSTATUSREADER_H
#define CONSTELLATIONSTATUSREADER_H

#include <QMap>
#include <QString>

#include <inav/Antex>
#include <inav/Satellite>

namespace io {
/*!
 * \brief Читатель файлов состава орбитальной группировки Const_YYMMDD.gps / .glo
 *
 * Источник связи PRN ↔ SVN и состояния НКА для перехода ЦМ → ФЦА по ANTEX.
 * Заменяет igs_satellite_metadata.snx: на выходе тот же
 * satmeta::SATELLITE_METADATA_FILE, который принимает
 * antex::SatelliteAntennaModel::loadMetadata().
 *
 * Формат — фиксированные колонки, содержательная часть строки 57 символов
 * (с 58-й позиции — необязательный комментарий), 0-based позиции:
 *
 *      1..3   PRN             "G24"  / "R01"
 *      5..6   слот / литера   "A1"   / "-4"
 *      7..11  SVN             "   65" / "  730"
 *     13..17  NORAD           "38833"
 *     19..24  тип НКА         "II-F" / "ГЛО-М" (cp1251)
 *     26..33  дата файла      "30.07.26"
 *     35..42  дата запуска    "04.10.12"
 *     44..51  дата ввода      "14.11.12"  — пусто, если НКА не в строю
 *     53..56  суток в строю   "4994"
 *
 * НКА без даты ввода в строй в метаданные не попадает — это и есть критерий
 * состояния.
 *
 * SVN приводится к формату ANTEX ("G065", "R854"):
 *  - GPS: номер НКА совпадает с IGS SVN, приводится напрямую;
 *  - ГЛОНАСС: номер НКА и IGS SVN — разные пространства идентификаторов
 *    (часть номеров совпадает, часть смещена на +100), поэтому соответствие
 *    задаётся таблицей svn_map.txt. Таблица авторитетна: НКА, номера которого
 *    в ней нет, из расчёта исключается. Прямое приведение "R" + номер
 *    запрещено — оно молча даёт PCO чужого НКА (758 → R758 в igs20.atx есть,
 *    но это другой аппарат, нужен R854).
 *
 * Кириллица в колонках типа и комментария не используется: тип антенны
 * берётся из ANTEX, поэтому файл читается как Latin-1, а поле block
 * заполняется только когда колонка чисто ASCII (GPS).
 */
class ConstellationStatusReader {
public:

   struct Statistics {
      int       gpsTotal     = 0; ///< строк GPS в файле
      int       gpsInService = 0; ///< из них с датой ввода в строй
      int       gloTotal     = 0; ///< строк ГЛОНАСС в файле
      int       gloInService = 0; ///< из них с датой ввода в строй
      int       malformed    = 0; ///< строк, не разобранных по раскладке
      int       svnUnmapped  = 0; ///< НКА ГЛОНАСС, которых нет в таблице SVN
      QDateTime snapshotDate;     ///< дата файла (колонка 26..33)
   };

   ConstellationStatusReader() = delete;

   /*! Разбор пары файлов состава ОГ
    * \param [in]  gpsPath    Путь к Const_YYMMDD.gps
    * \param [in]  gloPath    Путь к Const_YYMMDD.glo
    * \param [in]  svnMapPath Путь к svn_map.txt (номер НКА ГЛОНАСС → IGS SVN)
    * \param [out] out        Метаданные PRN ↔ SVN
    * \param [out] stats      Статистика разбора (может быть nullptr)
    * \return true — файлы прочитаны и записи получены
    */
   static bool read(const QString&                    gpsPath,
                    const QString&                    gloPath,
                    const QString&                    svnMapPath,
                    satmeta::SATELLITE_METADATA_FILE& out,
                    Statistics*                       stats = nullptr);

private:

   /// Таблица «номер НКА → IGS SVN»; строки "# ..." и пустые пропускаются
   static bool      loadSvnMap(const QString& path, QMap<int, QString>& out);

   static bool      readOne(const QString&                    path,
                            SatelliteSystem::TYPE             system,
                            const QMap<int, QString>&         svnMap,
                            satmeta::SATELLITE_METADATA_FILE& out,
                            Statistics&                       stats);

   /// Разбор даты "dd.MM.yy" в UTC-полночь; пустая/битая → невалидный QDateTime
   static QDateTime parseDate(const QString& ddmmyy);
};
} // namespace io

#endif // CONSTELLATIONSTATUSREADER_H
