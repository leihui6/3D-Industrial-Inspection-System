#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QGridLayout>
#include <QFileDialog>
#include <QDir>

#include <osg/Group>
#include <osgDB/ReadFile>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_viewer_widget(Q_NULLPTR)
{
    ui->setupUi(this);
    this->setWindowTitle("3D Industrial Workpiece Measurement System V1.0");

    m_viewer_widget = new QViewerWidget(QRect(0, 0, ui->frame->widthMM(), ui->frame->heightMM()));

    QGridLayout *layout = new QGridLayout;

    if(layout != Q_NULLPTR)
    {
        layout->setSpacing(1);
        layout->setMargin(1);

        layout->addWidget(m_viewer_widget);
        ui->frame->setLayout(layout);

        connect(&m_timer, &QTimer::timeout, this, &MainWindow::update);
        m_timer.start(1000/60);
    }

    initialize_folder();
    initialize_widget();
    initialize_parameters();
    initialize_dll();
}

MainWindow::~MainWindow()
{
    for (auto & item:m_handle_vec)
    {
        FreeLibrary(item.second);
    }

    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *)
{
    ui->frame->update();
}

size_t MainWindow::write_log(std::string log)
{
    std::string before_log = ">", postfix = "\n";

    if(log[0] >= 97 && log[0] <= 97+26)
        log[0] = log[0]-32;

    ui->textEdit->moveCursor(QTextCursor::End);
    ui->textEdit->insertPlainText(QString::fromStdString(before_log + log + postfix));
    ui->textEdit->moveCursor(QTextCursor::End);

    return size_t(ui->textEdit->toPlainText().size());
}

bool MainWindow::initialize_parameters()
{
    m_configuration_file = "data/3d_measurement_configuration.txt";

    if (!read_file_as_map(m_configuration_file, m_str_str_map))
    {
        return false;
    }

    m_str_str_map["configuration"] = m_configuration_file;

    std::string output_log =
            "reading " + std::to_string(m_str_str_map.size()) +" parameters from " + m_configuration_file;

    //ui->label_config_name->setText(QString(m_configuration_file.data()));

    std::string measurement_pairs_filename = m_str_str_map["measurement_pairs"];
    ui->label_measurement_info->setText(QString(measurement_pairs_filename.data()));

    write_log(output_log);

    return true;
}

void MainWindow::initialize_dll()
{
    // add more dll files
    std::vector<std::string> handle_filename_vec;
    handle_filename_vec.push_back("label_visual.dll");
    handle_filename_vec.push_back("back_process.dll");




    // end adding



    for(size_t i=0;i<handle_filename_vec.size();++i)
    {
        std::wstring this_filename(handle_filename_vec.at(i).begin(), handle_filename_vec.at(i).end());
        HINSTANCE this_dll = LoadLibrary(this_filename.data());

        if (!this_dll)
        {
            show_warning("Wrong","No "+ (handle_filename_vec.at(i)));
            write_log("Loading " + handle_filename_vec.at(i) + " failed");
        }
        else
        {
            if(i == 0)
                m_handle_vec.insert(std::pair<std::string, HINSTANCE>("label_visual", this_dll));
            if(i == 1)
                m_handle_vec.insert(std::pair<std::string, HINSTANCE>("back_process", this_dll));
        }
    }
}

void MainWindow::visual_thread(const std::string & filename)
{
    typedef LabelVisualCom*(*LabelVisualComfunc)();

    HINSTANCE hDll = m_handle_vec["label_visual"];

    LabelVisualComfunc dllFunc = (LabelVisualComfunc)GetProcAddress(hDll, "getLabelVisualCom");

    // const std::string filename = "visual_config_labeling.txt";
    LabelVisualCom * label_visual_p;
    label_visual_p = (LabelVisualCom*)(dllFunc());

    label_visual_p->initial_label_info(filename);

    label_visual_p->visual_label();

    delete label_visual_p;
}

void MainWindow::update()
{
    QMainWindow::update(this->geometry());
}

void MainWindow::clean()
{
    //osg::Group *scene = qviewer->getScene();

    //    if (scene == nullptr)
    //        return;

    //    scene->removeChildren(0, scene->getNumChildren());
}

void MainWindow::quit()
{
    QApplication::quit();
}

void MainWindow::initialize_folder()
{
    m_export_labeled_info_path = QString((m_str_str_map["output_folder"] + m_str_str_map["marked_points_result"]).data());


    if (!QDir(m_export_folder).exists(m_export_folder))
    {
        if(QDir().mkdir(m_export_folder))
        {
            // TODO
        }
    }
}

void MainWindow::on_btn_start_labeling_clicked()
{
    m_viewer_widget->add_pick_handler();

    ui->btn_start_labeling->setDisabled(true);
    ui->btn_cancel_labeling->setDisabled(false);


    ui->btn_point_labeling->setEnabled(true);
    ui->btn_line_labeling->setEnabled(true);
    ui->btn_plane_labeling->setEnabled(true);
    ui->btn_cylinder_labeling->setEnabled(true);
}

void MainWindow::on_btn_cancel_labeling_clicked()
{
    m_viewer_widget->remove_pick_handler();
    m_viewer_widget->record_labeled_points();

    DETECT_TYPE dt;
    dt = m_viewer_widget->get_current_detection_type();

    std::string text = "Recorded as ";
    if (dt == DETECT_TYPE::DT_POINT)
    {
        text += "Point";
    }
    else if (dt == DETECT_TYPE::DT_LINE)
    {
        text += "Line";
    }
    else if (dt == DETECT_TYPE::DT_PLANE)
    {
        text += "Plane";
    }
    else if (dt == DETECT_TYPE::DT_CYLINDER)
    {
        text += "Cylinder";
    }
    else
    {
        text += "Unknown";
    }
    write_log(text);

    m_viewer_widget->clear_labeled_fitting();
    m_viewer_widget->clear_point_cloud();

    ui->btn_start_labeling->setEnabled(true);
    ui->btn_cancel_labeling->setDisabled(true);


    ui->btn_point_labeling->setEnabled(false);
    ui->btn_line_labeling->setEnabled(false);
    ui->btn_plane_labeling->setEnabled(false);
    ui->btn_cylinder_labeling->setEnabled(false);
}

void MainWindow::on_btn_point_labeling_clicked()
{
    write_log("Selected: point");

    m_viewer_widget->fit_picked_point_to_point();
}

void MainWindow::on_btn_line_labeling_clicked()
{
    write_log("selected: line");

    m_viewer_widget->fit_picked_point_to_line();
}

void MainWindow::on_btn_plane_labeling_clicked()
{
    write_log("selected: plane");

    m_viewer_widget->fit_picked_point_to_plane();
}

void MainWindow::on_btn_cylinder_labeling_clicked()
{
    write_log("selected: cylinder");

    m_viewer_widget->fit_picked_point_cylinder();
}


void MainWindow::initialize_widget()
{
    ui->textEdit->setReadOnly(true);

    ui->btn_start_labeling->setEnabled(false);
    ui->btn_point_labeling->setEnabled(false);
    ui->btn_line_labeling->setEnabled(false);
    ui->btn_plane_labeling->setEnabled(false);
    ui->btn_cylinder_labeling->setEnabled(false);

    ui->btn_cancel_labeling->setDisabled(true);

    // initial 'show_axis' spindouble
    ui->doubleSpinBox_show_axis->setValue(1.0);
    ui->doubleSpinBox_show_axis->setRange(0.1, 20.0);
    ui->doubleSpinBox_show_axis->setSingleStep(0.1);
    if (ui->checkBox_show_axis->checkState() != Qt::Checked)
    {
        ui->doubleSpinBox_show_axis->setEnabled(false);
    }

    // initial 'point size' spindouble
    ui->spinBox_point_size->setValue(1);
    ui->spinBox_point_size->setMinimum(1);
    ui->spinBox_point_size->setSingleStep(1);

    // initial back and front color
    ui->pushButton_back_color->setStyleSheet("background: rgb(135,206,235)");
    ui->pushButton_front_color->setStyleSheet("background: rgb(0,0,0)");
    ui->pushButton_hover_color->setStyleSheet("background: rgb(255,255,255)");
    ui->pushButton_picked_color->setStyleSheet("background: rgb(255,0,0)");
    ui->pushButton_fitting_color->setStyleSheet("background: rgb(0,255,0)");

    m_viewer_widget->set_background_color(135, 206, 235, 1);
}

void MainWindow::on_btn_export_label_infor_clicked()
{
    std::map<std::string,std::vector<point_3d>> labeled_points_map;

    this->m_viewer_widget->get_labeled_points_map(labeled_points_map);

    std::map<std::string,std::vector<point_3d>>::iterator it;

    std::ofstream ofile(m_export_labeled_info_path.toStdString());
    if(!ofile.is_open()) return;

    for(it = labeled_points_map.begin();it != labeled_points_map.end();++it)
    {
        std::vector<point_3d> & points = it->second;
        for(size_t i=0; i<points.size(); ++i)
        {
            ofile << points[i].x << " "<< points[i].y << " "<< points[i].z ;
            ofile <<"\n";
        }
        ofile<<"#"<<it->first<<"\n";
    }
    ofile.close();

    write_log("writing into: " + m_export_labeled_info_path.toStdString());
}

void MainWindow::on_pushButton_check_list_clicked()
{
    std::map<std::string, std::vector<point_3d>> labeled_points_map;

    this->m_viewer_widget->get_labeled_points_map(labeled_points_map);

    if (labeled_points_map.empty())
    {
        write_log("no labeled points");
    }

    std::map<std::string, std::vector<point_3d>>::iterator it;

    write_log("all labeling points are shown as below:");

    for (it = labeled_points_map.begin(); it != labeled_points_map.end(); ++it)
    {
        write_log(it->first + " : " + std::to_string(it->second.size()));
    }

    write_log("done");
}

void MainWindow::on_pushButton_load_data_clicked()
{
    if (m_str_str_map.find("reference_data") == m_str_str_map.end())
    {
        show_warning("Warnning", "No reference data");
        return ;
    }

    std::string reference_data_path =m_str_str_map["input_folder"]+ m_str_str_map["reference_data"];

    if (ci.load_point_cloud_txt(reference_data_path, m_target_cloud_point))
    {
        write_log("Reading " + reference_data_path + " (size: " + std::to_string(m_target_cloud_point.size()) + ")");
    }
    else
    {
        write_log("Reading " + reference_data_path + " failed");
        return ;
    }
    m_viewer_widget->add_point_cloud(m_target_cloud_point, POINT_CLOUD, 1.0);
    m_viewer_widget->set_target_point_cloud(m_target_cloud_point);

    ui->btn_start_labeling->setEnabled(true);
}

void MainWindow::on_checkBox_show_axis_stateChanged(int arg1)
{
    if (this->ui->checkBox_show_axis->checkState() == Qt::Checked)
    {
        ui->doubleSpinBox_show_axis->setEnabled(true);
        this->m_viewer_widget->show_axis(float(ui->doubleSpinBox_show_axis->value()));
        write_log("opening show axis " + std::to_string(arg1));
    }
    else
    {
        ui->doubleSpinBox_show_axis->setEnabled(false);
        this->m_viewer_widget->hide_axis();
        write_log("closing show axis " + std::to_string(arg1));
    }
}

void MainWindow::on_doubleSpinBox_show_axis_valueChanged(const QString &arg1)
{
    on_checkBox_show_axis_stateChanged(arg1.toInt());
}

void MainWindow::on_spinBox_point_size_valueChanged(const QString &arg1)
{
    float point_size = float(ui->spinBox_point_size->value());
    this->m_viewer_widget->set_point_size(POINT_CLOUD, point_size);
    write_log("set point size: " + arg1.toStdString());
}

void MainWindow::on_pushButton_back_color_clicked()
{
    QColor color = QColorDialog::getColor(Qt::yellow, this);
    if (color.isValid())
    {
        float r, g, b;
        r = color.red();
        g = color.green();
        b = color.blue();
        this->m_viewer_widget->set_background_color(r, g, b, 1);

        QString html_content = "background: rgb(" + QString::number(int(r)) + "," + QString::number(int(g)) + "," + QString::number(int(b)) + ")";
        ui->pushButton_back_color->setStyleSheet(html_content);

        write_log("set background color: " + html_content.toStdString());
    }

}

void MainWindow::on_pushButton_front_color_clicked()
{
    QColor color = QColorDialog::getColor(Qt::yellow, this);
    if (color.isValid())
    {
        float r, g, b;
        r = color.red();
        g = color.green();
        b = color.blue();
        this->m_viewer_widget->set_color(POINT_CLOUD, r, g, b, 1);

        QString html_content = "background: rgb(" + QString::number(int(r)) + "," + QString::number(int(g)) + "," + QString::number(int(b)) + ")";
        ui->pushButton_front_color->setStyleSheet(html_content);

        write_log("set point cloud color: " + html_content.toStdString());
    }
    else
    {
        write_log("picked color is invalid");
    }
}

void MainWindow::on_pushButton_hover_color_clicked()
{
    QColor color = QColorDialog::getColor(Qt::yellow, this);
    if (color.isValid())
    {
        float r, g, b;
        r = color.red();
        g = color.green();
        b = color.blue();
        this->m_viewer_widget->set_color(HOVER_POINT, r, g, b, 1);

        QString html_content = "background: rgb(" + QString::number(int(r)) + "," + QString::number(int(g)) + "," + QString::number(int(b)) + ")";
        ui->pushButton_hover_color->setStyleSheet(html_content);

        write_log("set hover color: " + html_content.toStdString());
    }
    else
    {
        write_log("picked color is invalid");
    }
}

void MainWindow::on_pushButton_picked_color_clicked()
{
    QColor color = QColorDialog::getColor(Qt::yellow, this);
    if (color.isValid())
    {
        float r, g, b;
        r = color.red();
        g = color.green();
        b = color.blue();
        this->m_viewer_widget->set_color(PICKED_POINTS, r, g, b, 1);

        QString html_content = "background: rgb(" + QString::number(int(r)) + "," + QString::number(int(g)) + "," + QString::number(int(b)) + ")";
        ui->pushButton_picked_color->setStyleSheet(html_content);

        write_log("set picked color: " + html_content.toStdString());
    }
    else
    {
        write_log("picked color is invalid");
    }
}

void MainWindow::on_pushButton_fitting_color_clicked()
{
    QColor color = QColorDialog::getColor(Qt::yellow, this);
    if (color.isValid())
    {
        float r, g, b;
        r = color.red();
        g = color.green();
        b = color.blue();
        this->m_viewer_widget->set_color(FITTING_CLOUD, r, g, b, 1);

        QString html_content = "background: rgb(" + QString::number(int(r)) + "," + QString::number(int(g)) + "," + QString::number(int(b)) + ")";
        ui->pushButton_fitting_color->setStyleSheet(html_content);

        write_log("set fitting color: " + html_content.toStdString());
    }
    else
    {
        write_log("picked color is invalid");
    }
}


void MainWindow::on_tabWidget_2_currentChanged(int index)
{
    std::cout <<index <<std::endl;
    ui->tabWidget->setCurrentIndex(index);
}

void MainWindow::on_tabWidget_currentChanged(int index)
{
    ui->tabWidget_2->setCurrentIndex(index);
}

int MainWindow::all_process_thread()
{
    typedef BackProcessCom*(*BackProcessComfunc)();

    HINSTANCE hDll = m_handle_vec["back_process"];

    BackProcessComfunc dllFunc = (BackProcessComfunc)GetProcAddress(hDll, "getBackProcessCom");

    BackProcessCom * back_process_p;
    back_process_p = (BackProcessCom*)dllFunc();
    back_process_p->initial_parameter(m_configuration_file);

    clock_t beg_t = clock();
    double this_time = 0, total_time = 0;

    write_log("starting registration...");
    // registration
    back_process_p->registration();
    this_time = (clock() - beg_t)/CLOCKS_PER_SECOND;
    total_time += this_time;
    write_log("registration done! time elapsed:"+ std::to_string(this_time).substr(0,6) + "s");
    beg_t = clock();

    // searching all labeled points
    write_log("starting searching...");
    back_process_p->searching();
    this_time = (clock() - beg_t)/CLOCKS_PER_SECOND;
    total_time += this_time;
    write_log("searching done! time elapsed:"+ std::to_string(this_time).substr(0,6) + "s");
    beg_t = clock();

    // measurement
    write_log("starting measurement...");
    back_process_p->measurement();
    this_time = (clock() - beg_t)/CLOCKS_PER_SECOND;
    total_time += this_time;
    write_log("measurement done! time elapsed:"+ std::to_string(this_time).substr(0,6) + "s");

    write_log("total time elapsed:"+ std::to_string(total_time).substr(0,6) + "s");

    // Evaluation
    write_log("starting evaluation...");
    back_process_p->evaluation();
    write_log("evaluation done!");

    delete back_process_p;

    return 1;
}

void MainWindow::on_btn_measurement_read_clicked()
{
    std::string measurement_pairs_filename = m_str_str_map["input_folder"]+m_str_str_map["measurement_pairs"];

    write_log("reading "+measurement_pairs_filename);

    std::ifstream ifile(measurement_pairs_filename);
    if(!ifile.is_open())
    {
        return ;
    }

    ui->text_measurement_pairs->clear();
    std::string line;
    while(std::getline(ifile, line))
    {
        ui->text_measurement_pairs->appendPlainText(QString(line.data()));
    }

    ifile.close();
}

void MainWindow::on_btn_measurement_write_clicked()
{
    QMessageBox::StandardButton result;
    QMessageBox::StandardButton  defaultBtn=QMessageBox::NoButton;
    result=QMessageBox::question(this, "Warning", "Are you sure, it will clear the local file",
                                 QMessageBox::Yes|QMessageBox::No |QMessageBox::Cancel,
                                 defaultBtn);
    if(result==QMessageBox::No)
        return;
    else if(result==QMessageBox::Cancel)
        return;

    QString tmp_str = ui->text_measurement_pairs->toPlainText();
    if(tmp_str.isEmpty())
    {
        return;
    }

    // read QTextEditPlain
    QTextDocument* doc=ui->text_measurement_pairs->document();

    std::vector<std::string> text_context;
    for (int i=0; i<doc->blockCount();i++)
    {
        QTextBlock textLine = doc->findBlockByNumber(i);

        text_context.push_back(textLine.text().toStdString());
    }


    // write into the local file
    std::string measurement_pairs_filename = m_str_str_map["output_folder"] + m_str_str_map["measurement_pairs"];

    std::ofstream ofile(measurement_pairs_filename);
    if(!ofile.is_open())
    {
        return;
    }

    for(auto & line: text_context)
    {
        ofile << line<<"\n";
    }
    write_log("wrote "+ std::to_string(text_context.size())+" iteam(s) for measurement");
    ofile.close();
}

void MainWindow::on_btn_measurement_check_clicked()
{
    // read context, collecting all items
    QTextDocument* doc = ui->text_measurement_pairs->document();
    std::vector<std::string> ui_items_vec;

    for (int i=0; i<doc->blockCount();i++)
    {
        QTextBlock textLine = doc->findBlockByNumber(i);

        if(textLine.text()[0] == '#') continue;

        QStringList strlist = textLine.text().split(":");

        for (auto &s:strlist)
        {
            //qDebug() << s << "\n";
            ui_items_vec.push_back(s.toStdString());
        }
    }

    // read the local file, collecting all items
    qDebug() << m_export_labeled_info_path;
    std::ifstream ifile(m_export_labeled_info_path.toStdString());
    if(!ifile.is_open())
    {
        return;
    }

    std::string line;
    std::vector<std::string> local_items_vec;
    while (std::getline(ifile, line))
    {
        if (line.empty()) break;

        if (line[0] == '#')
        {
            local_items_vec.push_back(line.substr(1,line.size()-1));
        }
    }

    // compare
    std::vector<std::string> not_found_items;
    for(auto & i: ui_items_vec)
    {
        bool if_found = false;

        for(auto & j:local_items_vec)
        {
            if(i == j)
            {
                if_found = true;
                break;
            }
        }

        if(!if_found)
        {
            not_found_items.push_back(i);
        }
    }

    if(not_found_items.empty())
    {
        write_log("Check done! All items are recorded in the label file");
    }
    else
    {
        std::string text = std::to_string(not_found_items.size())+" item(s) are not recorded";
        QString qtext = QString(text.data());
        QMessageBox::information(this, "information", qtext,
                                 QMessageBox::Ok,QMessageBox::NoButton);

        write_log("these items are: ");
        for(auto & i:not_found_items)
        {
            write_log(i);
        }
    }

}

void MainWindow::on_actionStart_triggered()
{
    QFuture<int> future = QtConcurrent::run(this, &MainWindow::all_process_thread);
}

void MainWindow::on_actionShow_Label_Result_triggered()
{
    std::string labeling_filename = "data/visual_config_labeling.txt";
    QFuture<void> future = QtConcurrent::run(this, &MainWindow::visual_thread,labeling_filename);
    //future.waitForFinished();
}

void MainWindow::on_actionShow_Measurement_Result_triggered()
{
    std::string measurement_filename = "data/visual_config_measurement.txt";
    QFuture<void> future = QtConcurrent::run(this, &MainWindow::visual_thread,measurement_filename);
}

void MainWindow::on_actionClean_triggered()
{
    MainWindow::clean();
}

void MainWindow::on_actionQuit_triggered()
{
    MainWindow::quit();
}

void MainWindow::on_actionCheck_triggered()
{
    dialog_settings_ui = new dialog_settings(this);
    dialog_settings_ui->set_configuration(m_str_str_map);

    if (dialog_settings_ui->exec() == QDialog::Accepted)
    {
        dialog_settings_ui = nullptr;
    }
}
