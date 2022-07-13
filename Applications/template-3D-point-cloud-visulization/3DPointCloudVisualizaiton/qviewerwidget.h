#ifndef     QVIEWER_WIDGET_H
#define     QVIEWER_WIDGET_H

#include "cloud_io.h"

#include <QWidget>

#include <osg/Point>
#include <osgViewer/Viewer>
#include <osgQt/GraphicsWindowQt>


class QViewerWidget : public QWidget
{
public:

    QViewerWidget(const QRect &geometry);

    virtual ~QViewerWidget();

    osg::Group *getScene();

    osgViewer::Viewer *getViewer();

    // [0, 255]
    void set_background_color(int r, int g,int b);
    void set_points_color(const std::string &point_cloud_name, int r, int g,int b);
    void set_points_size(const std::string &point_cloud_name, int size_number);
    void add_point_cloud(const std::string & point_cloud_name, std::vector<point_3d> & points);

protected:

    osg::ref_ptr<osg::Group> scene;

    osgViewer::Viewer viewer;


    std::map<std::string, osg::ref_ptr<osg::Node>> node_map;

private:

    osgQt::GraphicsWindowQt *createGraphicsWindow(const QRect &geometry);

    void initCamera(const QRect &geometry);

    void paintEvent(QPaintEvent *);

    void initial_scene();
};

#endif // QVIEWER_WIDGET_H
