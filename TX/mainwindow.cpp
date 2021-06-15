#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    watcher = new QFileSystemWatcher(this);
    connect(watcher, SIGNAL(fileChanged(const QString &)), this, SLOT(fileChanged(const QString &)));
    watcher->addPath("C:\\Users\\basel\\OneDrive\\Desktop\\Waterloo\\Club Work\\UWARG\\Repos\\Files to Test\\BaselTest.txt");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::addWaypoint(int num, QFormLayout *layout, int maxNum)
{

    layout->addRow(new QLabel("Waypoint"), new QLabel(QString::number(num)));
    layout->addItem(new QSpacerItem(0, 3, QSizePolicy::Fixed));

    layout->addRow("Latitude", new QLineEdit);
    layout->addRow("Longitude", new QLineEdit);
    layout->addRow("Altitude", new QLineEdit);
    layout->addRow("Turn Radius", new QLineEdit);

    QComboBox *waypointType = new QComboBox();
    waypointType->addItems({"", "0 - Path Follow", "1 - Orbit", "2 - Hold"});

    layout->addRow("Waypoint Type", waypointType);

    if (num != maxNum)
    {
        layout->addItem(new QSpacerItem(0, 25, QSizePolicy::Fixed));
    }
}

void MainWindow::remove(QLayout *layout)
{
    QLayoutItem *child;
    while (layout->count() != 0)
    {
        child = layout->takeAt(0);
        if (child->layout() != 0)
        {
            remove(child->layout());
        }
        else if (child->widget() != 0)
        {
            delete child->widget();
        }
        delete child;
    }
}

void MainWindow::fileChanged(const QString & path)
{
   if (QFile::exists(path)) {
        watcher->addPath(path);
    }
  qDebug() << "FILE CHANGED";
}

QString MainWindow::enumSelection(QComboBox* field){
    QString text = field->currentText();
    if (text == ""){
        return "";
    }
    else{
        return (QString)(text[0]);
    }
}

void MainWindow::on_setWaypointNumberButton_clicked()
{
    QFormLayout *layout = new QFormLayout;

    if (ui->waypointScrollAreaWidgetContents->layout() == nullptr)
    {
        ui->waypointScrollAreaWidgetContents->setLayout(layout);
    }
    else
    {
        delete layout;
        layout = qobject_cast<QFormLayout *>(ui->waypointScrollAreaWidgetContents->layout());
        remove(layout);
    }

    int numWaypoints = ui->setWaypointNumberEdit->value();

    for (int i{1}; i <= numWaypoints; i++)
    {
        addWaypoint(i, layout, numWaypoints);
    }
}

void MainWindow::on_sendInfoButton_clicked()
{
    QString waypointModifyFlightPathCommand = enumSelection(ui->waypointModifyFlightPathCommandBox);
    convertMessage(waypointModifyFlightPathCommand, MESSAGE_ID_WAYPOINT_MODIFY_PATH_CMD);

    QString waypointNextDirectionCommand = ui->waypointNextDirectionsCommandBox->currentText();
    convertMessage(waypointNextDirectionCommand, MESSAGE_ID_WAYPOINT_NEXT_DIRECTIONS_CMD);

    QString initalizingHomeBase = enumSelection(ui->initializingHomeBaseBox);
    convertMessage(initalizingHomeBase, MESSAGE_ID_INITIALIZING_HOMEBASE);

    QString holdingAltitude = ui->holdingAltitudeEdit->text();
    convertMessage(holdingAltitude, MESSAGE_ID_HOLDING_ALTITUDE);

    QString holdingTurnRadius = ui->holdingTurnRadiusEdit->text();
    convertMessage(holdingTurnRadius, MESSAGE_ID_HOLDING_TURN_RADIUS);

    QString holdingTurnDirection = enumSelection(ui->holdingTurnDirectionBox);
    convertMessage(holdingTurnDirection, MESSAGE_ID_HOLDING_TURN_DIRECTION);

    QString flightPathModifyNextId = ui->flightPathModifyNextIdEdit->text();
    convertMessage(flightPathModifyNextId, MESSAGE_ID_PATH_MODIFY_NEXT_LD);

    QString flightPathModifyPrevId = ui->flightPathModifyPrevIdEdit->text();
    convertMessage(flightPathModifyPrevId, MESSAGE_ID_PATH_MODIFY_PREV_LD);

    QString flightPathModifyId = ui->flightPathModifyIdEdit->text();
    convertMessage(flightPathModifyId, MESSAGE_ID_PATH_MODIFY_LD);

    QString homeBaseLatitude = ui->homeBaseLatitudeEdit->text();
    QString homeBaseLongitude = ui->homeBaseLongitudeEdit->text();
    QString homeBaseAltitude = ui->homeBaseAltitudeEdit->text();
    QString homeBaseTurnRadius = ui->homeBaseTurnRadiusdit->text();
    QString homeBaseWaypointType = enumSelection(ui->homeBaseWaypointTypeBox);

    QList<QString> homeBaseInfo = {homeBaseLatitude,
                                   homeBaseLongitude,
                                   homeBaseAltitude,
                                   homeBaseTurnRadius,
                                   homeBaseWaypointType};
    convertMessage(homeBaseInfo, MESSAGE_ID_HOMEBASE);


    int numWaypoints = ui->setWaypointNumberEdit->value();
    convertMessage(numWaypoints == 0 ? "" : QString::number(numWaypoints),
                   MESSAGE_ID_NUM_WAYPOINTS);

    if (!(ui->waypointScrollAreaWidgetContents->layout() == nullptr || numWaypoints == 0))
    {
        QFormLayout *layout = qobject_cast<QFormLayout *>(ui->waypointScrollAreaWidgetContents->layout());

        for (int i{}; i < numWaypoints; i++)
        {
            QString waypointLatitude = qobject_cast<QLineEdit *>(layout->itemAt(i * 14 + 4)->widget())->text();
            QString waypointLongitude = qobject_cast<QLineEdit *>(layout->itemAt(i * 14 + 6)->widget())->text();
            QString waypointAltitude = qobject_cast<QLineEdit *>(layout->itemAt(i * 14 + 8)->widget())->text();
            QString waypointTurnRadius = qobject_cast<QLineEdit *>(layout->itemAt(i * 14 + 10)->widget())->text();
            QString waypointType = enumSelection(qobject_cast<QComboBox *>(layout->itemAt(i * 14 + 12)->widget()));

            QList<QString> waypointInfo = {waypointLatitude,
                                           waypointLongitude,
                                           waypointAltitude,
                                           waypointTurnRadius,
                                           waypointType};

            convertMessage(waypointInfo, MESSAGE_ID_WAYPOINTS);
        }
    }
}

void MainWindow::convertMessage(QString data, PIGO_Message_IDs_e msg_id){
    if (data == ""){
        return;
    }

    int data_int = data.toInt();

    mavlink_message_t encoded_msg;
    memset(&encoded_msg, 0x00, sizeof(mavlink_message_t));

    uint8_t encoderStatus = Mavlink_airside_encoder(msg_id, &encoded_msg, (const uint8_t*) &data_int);
}

void MainWindow::convertMessage(QList<QString> data, PIGO_Message_IDs_e msg_id){
    for (int i{}; i < data.length(); i++){
        if (data[i] == ""){
            return;
        }
    }

    mavlink_message_t encoded_msg;
    memset(&encoded_msg, 0x00, sizeof(mavlink_message_t));

    uint8_t encoderStatus{};

    if (msg_id == MESSAGE_ID_HOMEBASE || msg_id == MESSAGE_ID_WAYPOINTS){
        PIGO_WAYPOINTS_t data_transferred{};
        data_transferred.latitude = data[0].toInt();
        data_transferred.longitude = data[1].toInt();
        data_transferred.altitude = data[2].toInt();
        float turnRadius = data[3].toFloat();
        uint32_t* turnRadiusInt32 = reinterpret_cast<uint32_t*>(&turnRadius);
        data_transferred.turnRadius = *turnRadiusInt32;
        data_transferred.waypointType = data[4].toInt();

        encoderStatus = Mavlink_airside_encoder(msg_id, &encoded_msg, (const uint8_t*) &data_transferred);
    }
    else if (msg_id == MESSAGE_ID_GPS_LANDING_SPOT){
        PIGO_GPS_LANDING_SPOT_t data_transferred{};
        data_transferred.latitude = data[0].toInt();
        data_transferred.longitude = data[1].toInt();
        data_transferred.altitude = data[2].toInt();
        data_transferred.landingDirection = data[3].toInt();

       encoderStatus = Mavlink_airside_encoder(msg_id, &encoded_msg, (const uint8_t*) &data_transferred);
    }
    else if (msg_id == MESSAGE_ID_GROUND_CMD || msg_id == MESSAGE_ID_GIMBAL_CMD){
        PIGO_GIMBAL_t data_transferred{};

        data_transferred.pitch = data[0].toInt();
        data_transferred.yaw = data[1].toInt();

        encoderStatus = Mavlink_airside_encoder(msg_id, &encoded_msg, (const uint8_t*) &data_transferred);
    }
    return;
}
