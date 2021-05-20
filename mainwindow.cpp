#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
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
    waypointType->addItems({"0 - Path Follow", "1 - Orbit", "2 - Hold"});

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
    QString waypointModifyFlightPathCommand = ui->waypointModifyFlightPathCommandBox->currentText();
    QString waypointNextDirectionCommand = ui->waypointNextDirectionsCommandBox->currentText();
    QString initalizingHomeBase = ui->initializingHomeBaseBox->currentText();
    QString holdingAltitude = ui->holdingAltitudeEdit->text();
    QString holdingTurnRadius = ui->holdingTurnRadiusEdit->text();
    QString holdingTurnDirection = ui->holdingTurnDirectionBox->currentText();
    QString flightPathModifyNextId = ui->flightPathModifyNextIdEdit->text();
    QString flightPathModifyPrevId = ui->flightPathModifyPrevIdEdit->text();
    QString flightPathModifyId = ui->flightPathModifyIdEdit->text();
    QString homeBaseLatitude = ui->homeBaseLatitudeEdit->text();
    QString homeBaseLongitude = ui->homeBaseLongitudeEdit->text();
    QString homeBaseAltitude = ui->homeBaseAltitudeEdit->text();
    QString homeBaseTurnRadius = ui->homeBaseTurnRadiusdit->text();
    QString homeBaseWaypointType = ui->homeBaseWaypointTypeBox->currentText();

    int numWaypoints = ui->setWaypointNumberEdit->value();

    if (!(ui->waypointScrollAreaWidgetContents->layout() == nullptr || numWaypoints == 0))
    {
        QFormLayout *layout = qobject_cast<QFormLayout *>(ui->waypointScrollAreaWidgetContents->layout());

        for (int i{}; i < numWaypoints; i++)
        {

            qDebug() << "Waypoint " << i + 1;
            qDebug() << "Latitude: " << qobject_cast<QLineEdit *>(layout->itemAt(i * 14 + 4)->widget())->text();
            qDebug() << "Longitude: " << qobject_cast<QLineEdit *>(layout->itemAt(i * 14 + 6)->widget())->text();
            qDebug() << "Altitude: " << qobject_cast<QLineEdit *>(layout->itemAt(i * 14 + 8)->widget())->text();
            qDebug() << "Turn Radius: " << qobject_cast<QLineEdit *>(layout->itemAt(i * 14 + 10)->widget())->text();
            qDebug() << "Waypoint Type: " << qobject_cast<QComboBox *>(layout->itemAt(i * 14 + 12)->widget())->currentText() << "\n\n";
        }
    }
}
