#ifndef PILOTMANAGER_H
#define PILOTMANAGER_H

#include <QMainWindow>

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
};
#endif // PILOTMANAGER_H
