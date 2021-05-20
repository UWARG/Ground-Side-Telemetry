#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QString>
#include <QLineEdit>
#include <json.hpp>
#include <QFormLayout>
#include <QSpacerItem>
#include <QSizePolicy>

using json = nlohmann::json;

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

    QFile File;

private slots:
    void on_fileBrowseButton_clicked();

    void on_setWaypointNumberButton_clicked();

    void on_sendInfoButton_clicked();

    void on_fileLoadButton_clicked();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
