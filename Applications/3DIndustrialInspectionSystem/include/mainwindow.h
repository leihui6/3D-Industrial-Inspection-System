#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QProcess>
#include <QColorDialog>
#include <QPlainTextEdit>
#include <QTextBlock>
#include <QDebug>
#include <QFuture>
#include <QtConcurrent/QtConcurrent>
#include <QThread>

#include <dialog_settings.h>


#include "LabelVisualCom.h"
#include "LabelVisualExp.h"

#include "BackProcessCom.h"
#include "BackProcessExp.h"


#include "common_use.h"
#include "cloud_io.h"
#include "qviewerwidget.h"

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

    dialog_settings * dialog_settings_ui;

    cloud_io ci;
    QTimer m_timer;
    QViewerWidget * m_viewer_widget;
    void paintEvent(QPaintEvent *);

    std::vector<point_3d> m_target_cloud_point;

    size_t write_log(std::string log);

    bool initialize_parameters();

    void initialize_dll();

    void visual_thread(const std::string & file1, const std::string & file2, int flag);

    int all_process_thread();

private slots:

    void update();

    void clean();

    void quit();

    void on_btn_start_labeling_clicked();

    void on_btn_cancel_labeling_clicked();

	void on_btn_point_labeling_clicked();

    void on_btn_line_labeling_clicked();

	void on_btn_plane_labeling_clicked();

	void on_btn_cylinder_labeling_clicked();

    void on_btn_export_label_infor_clicked();

    void on_pushButton_load_data_clicked();

    void on_checkBox_show_axis_stateChanged(int arg1);

    void on_doubleSpinBox_show_axis_valueChanged(const QString &arg1);

    void on_spinBox_point_size_valueChanged(const QString &arg1);

    void on_pushButton_back_color_clicked();

    void on_pushButton_front_color_clicked();

    void on_pushButton_check_list_clicked();

    void on_pushButton_hover_color_clicked();

    void on_pushButton_picked_color_clicked();

    void on_pushButton_fitting_color_clicked();

    void on_tabWidget_2_currentChanged(int index);

    void on_tabWidget_currentChanged(int index);

    //void on_visual_labeling_clicked();

    //void on_btn_all_process_start_clicked();

    void on_btn_measurement_read_clicked();

    void on_btn_measurement_write_clicked();

    void on_btn_measurement_check_clicked();

    void on_actionStart_triggered();

    void on_actionShow_Label_Result_triggered();

    void on_actionShow_Measurement_Result_triggered();

    void on_actionClean_triggered();

    void on_actionQuit_triggered();

    void on_actionCheck_triggered();

private:
    // configuration parameters
    std::map<std::string, std::string> m_str_str_map;
    void initialize_widget();
	void initialize_folder();
    QString m_export_folder;
    QString m_export_labeled_info_path;

    std::string m_configuration_file;

    std::map<std::string, HINSTANCE> m_handle_vec;
};

#endif // MAINWINDOW_H
