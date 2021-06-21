#ifndef PILOTMANAGER_H
#define PILOTMANAGER_H

#include <QMainWindow>
#include <QObject>
#include "Mavlink2/Groundside_Functions.hpp"
#include "Mavlink2/Airside_Functions.hpp"
#include "Mavlink2/Mavlink2_lib/common/common.h"


QT_BEGIN_NAMESPACE
namespace Ui { class PilotManager; }
QT_END_NAMESPACE

class PilotManager : public QMainWindow
{
    Q_OBJECT

public:
    PilotManager(QWidget *parent = nullptr);
    ~PilotManager();

private:
    Ui::PilotManager *ui;
    void writeToJSON(char* jsonIndex);

    mavlink_decoding_status_t decoderStatus;
    char decoded_message_buffer[50];

public slots:
    void decodeNewSerialData(QByteArray new_serial_data);
    void updateWidget(char* decoded_message, POGI_Message_IDs_e message_type);

signals:
    void newDecodedData(char* decoded_message, POGI_Message_IDs_e message_type);

};
#endif // PILOTMANAGER_H
