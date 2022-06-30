#include    "mainwindow.h"
#include    "ui_mainwindow.h"

#include    <QGridLayout>
#include    <QFileDialog>

#include    <osg/Group>
#include    <osgDB/ReadFile>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , qviewer(Q_NULLPTR)
{
    ui->setupUi(this);

    QGridLayout *layout = new QGridLayout;
    qviewer = new QViewerWidget(QRect(0, 0, ui->frame->width(), ui->frame->height()));
    layout->setSpacing(1);
    layout->setMargin(1);
    layout->addWidget(qviewer);
    ui->frame->setLayout(layout);

    //ui->verticalLayout->addWidget(ui->frame);

    connect(&timer, &QTimer::timeout, this, &MainWindow::update);
    timer.start(40);

    connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::quit);
    connect(ui->actionClean, &QAction::triggered, this, &MainWindow::clean);
    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::open);

    this->setWindowTitle("3D-Point-Cloud-Visualization");

    ui->pushButton->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *)
{
    ui->frame->update();
}

void MainWindow::write_log(QString text)
{
    ui->label->setText(text);
}

void MainWindow::update()
{
    QMainWindow::update(this->geometry());
}

void MainWindow::open()
{
    QString path = QFileDialog::getOpenFileName(Q_NULLPTR,
                                                tr("Open model file"),
                                                "./",
                                                tr("OpenSceneGraph (*.txt)"));

    if (path.isEmpty())
        return;

    clean();
    std::vector<point_3d> points;
    ci.load_point_cloud_txt(path.toStdString(),points);
    qviewer->add_point_cloud(POINTCLOUD, points);

    write_log(path);
}

void MainWindow::clean()
{
    osg::Group *scene = qviewer->getScene();

    if (scene == nullptr)
        return;

    scene->removeChildren(0, scene->getNumChildren());
}

void MainWindow::quit()
{
    QApplication::quit();
}

void MainWindow::on_actionSet_Points_Color_triggered()
{
    QColor color = QColorDialog::getColor(Qt::yellow, this);
    if (color.isValid())
    {
        float r, g, b;
        r = color.red();
        g = color.green();
        b = color.blue();
        qviewer->set_points_color(POINTCLOUD,r, g, b);
    }
}


void MainWindow::on_actionSet_Background_Color_triggered()
{
    QColor color = QColorDialog::getColor(Qt::yellow, this);
    if (color.isValid())
    {
        float r, g, b;
        r = color.red();
        g = color.green();
        b = color.blue();
        qviewer->set_background_color(r, g, b);
    }
}


void MainWindow::on_actionConnect_to_Camera_triggered()
{
    vst3d_camera = new VST3D_Camera;

    if (vst3d_camera->init("C:\\Program Files\\VST\\VisenTOP Studio\\VisenTOP Studio.exe") == VST3D_RESULT_OK)
    {
        write_log("3D Camera Initialized Successfully");
        ui->pushButton->setEnabled(true);
    }
    else
    {
        write_log("3D Camera Initialized Failed");
        ui->pushButton->setEnabled(false);
    }
}


void MainWindow::on_actionSet_Points_Size_triggered()
{
    bool ok;
    QString text = QInputDialog::getText(this, tr("QInputDialog::getInt()"),
                                        tr("New Points Size:"), QLineEdit::Normal,
                                        "1", &ok);
    if (ok && !text.isEmpty())
    {
        qviewer->set_points_size(POINTCLOUD, text.toInt());
    }
}


void MainWindow::on_pushButton_clicked()
{
    std::vector<point_3d> point_cloud;

    vst3d_camera->get_point_cloud(point_cloud);

    qviewer->add_point_cloud(POINTCLOUD, point_cloud);

    write_log("Retrieved| Number of points:" + QString::number(point_cloud.size()));
}

