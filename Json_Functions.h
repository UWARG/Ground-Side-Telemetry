#ifndef JSON_FUNCTIONS_H
#define JSON_FUNCTIONS_H

#include <QJsonValue>

/* POGI data to write to the CV JSON file */

enum POGI_Data_IDs_e{
    TIMESTAMP_OF_MEASUREMENTS = 0,
    GPS_COORDINATES,
    CURRENT_AIRSPEED,
    EULER_ANGLES_OF_PLANE,
    EULER_ANGLES_OF_CAMERA,
    IS_LANDED,
};

int write_to_POGI_JSON(QString type, QJsonValue pogi_data);

#endif // JSON_FUNCTIONS_H
