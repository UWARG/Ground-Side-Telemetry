#include "pilotmanager.h"
#include "ui_pilotmanager.h"

PilotManager::PilotManager(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::PilotManager)
{
    ui->setupUi(this);
    ui->data_errorCode->setText("ooga booga. this text was set from the backend. booga ooga");
}

PilotManager::~PilotManager()
{
    delete ui;
}

