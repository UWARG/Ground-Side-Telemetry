//#include "Json_Functions.h"
//#include <QFile>
//#include <QIODevice>
//#include <QJsonParseError>
//#include <QJsonObject>
//#include <QString>
//#include <QDebug>
//#include <QApplication>

//int write_to_POGI_JSON(QString type, QJsonValue pogi_data, QString POGI_FILEPATH="/INPUT/ABS/PATH/HERE")
//{

//    QFile file(POGI_FILEPATH);

//    if (!file.exists()) {   // make sure file exists
//        qWarning() << "JSON File Write Error: " << POGI_FILEPATH << " doesn't exist";
//        return -1;
//    }
//    if (!file.open(QIODevice::ReadOnly)) {      // open file for reading
//        qWarning() << "JSON File Write Error: " << "couldn't open for reading " << POGI_FILEPATH;
//        return -1;
//    }

//    QByteArray jsonData = file.readAll();
//    file.close();

//    QJsonDocument JsonDoc(QJsonDocument::fromJson(jsonData));
//    QJsonObject root = JsonDoc.object();

//    if (root.contains(type)){   // make sure key exists in pogi file
//        if (root[type] != pogi_data) {  // if different data is in the file, then rewrite
//            root[type] = pogi_data;
//        }
//    }
//    else {  // if key doesn't exist, then insert a new key:value pair
//        root.insert(type, pogi_data);
//    }

//    if (!file.open(QIODevice::WriteOnly)) { // open file for writing
//        qWarning() << "JSON File Write Error: " << "couldn't open for writing " << POGI_FILEPATH;
//        return -1;
//    }

//    /* write the modified data to the json file */
//    file.write(QJsonDocument(root).toJson());
//    file.close();

//    return 0;

//}
