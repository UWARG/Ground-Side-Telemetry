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

public slots:
    void updateWidget(mavlink_message_t encoded_msg);

};
#endif // PILOTMANAGER_H
