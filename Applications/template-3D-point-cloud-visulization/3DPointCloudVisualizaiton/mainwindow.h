#ifndef     MAINWINDOW_H
#define     MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QColorDialog>
#include <QInputDialog>
#include <QLineEdit>

#include "vst3d_camera.h"
#include "qviewerwidget.h"

#define POINTCLOUD "POINTCLOUD"

namespace Ui
{
    class MainWindow;
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:

    Ui::MainWindow *ui;

    QViewerWidget *qviewer;

    camera_3d_com * vst3d_camera;

    QTimer timer;

    void paintEvent(QPaintEvent *);

    cloud_io ci;

    void write_log(QString text);

private slots:

    void update();

    void open();

    void clean();

    void quit();
    void on_actionSet_Points_Color_triggered();
    void on_actionSet_Background_Color_triggered();
    void on_actionConnect_to_Camera_triggered();
    void on_actionSet_Points_Size_triggered();
    void on_pushButton_clicked();
};

#endif // MAINWINDOW_H
