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
    layout->addWidget(qviewer);
    ui->frame->setLayout(layout);
    //ui->verticalLayout->addWidget(ui->frame);

    connect(&timer, &QTimer::timeout, this, &MainWindow::update);
    timer.start(40);

    connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::quit);
    connect(ui->actionClean, &QAction::triggered, this, &MainWindow::clean);
    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::open);

    this->setWindowTitle("3D-Point-Cloud-Visualization");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *)
{
    ui->frame->update();
}

void MainWindow::update()
{
    QMainWindow::update(this->geometry());
}

void MainWindow::open()
{
    osg::Group *scene = qviewer->getScene();

    if (scene == nullptr)
        return;

    QString path = QFileDialog::getOpenFileName(Q_NULLPTR,
                                                tr("Open model file"),
                                                "./",
                                                tr("OpenSceneGraph (*.txt)"));

    if (path.isEmpty())
        return;

    scene->removeChildren(0, scene->getNumChildren());

    std::vector<point_3d> points;
    ci.load_point_cloud_txt(path.toStdString(),points);
    std::cout << points.size()<<std::endl;

    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

    points_to_geometry_node(points, geometry, 255,255,255,1);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, int(points.size())));

    osg::ref_ptr<osg::Node> model = geometry;//osgDB::readNodeFile(path.toStdString());
    scene->addChild(model.get());

    ui->label->setText(path);
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
        qviewer->set_points_color(r, g, b);
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

}

