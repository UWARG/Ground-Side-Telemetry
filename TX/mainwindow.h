#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QLineEdit>
#include <QFormLayout>
#include <QSpacerItem>
#include <QSizePolicy>
#import <QDebug>

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

private slots:

    void on_setWaypointNumberButton_clicked();

    void on_sendInfoButton_clicked();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
