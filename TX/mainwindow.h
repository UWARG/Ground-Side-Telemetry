#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QLineEdit>
#include <QFormLayout>
#include <QSpacerItem>
#include <QSizePolicy>
#import <QDebug>
#include <QComboBox>
#include <Mavlink/Airside_Functions.hpp>
#include <iostream>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void addWaypoint(int num, QFormLayout* layout, int maxNum);

    void remove(QLayout* layout);
    QString enumSelection(QComboBox* field);

    void convertMessage(QString data, PIGO_Message_IDs_e msg_id);
    void convertMessage(QList<QString> data, PIGO_Message_IDs_e msg_id);
private slots:

    void on_setWaypointNumberButton_clicked();

    void on_sendInfoButton_clicked();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
