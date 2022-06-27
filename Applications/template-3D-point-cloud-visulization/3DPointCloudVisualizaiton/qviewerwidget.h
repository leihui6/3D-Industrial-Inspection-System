#ifndef     QVIEWER_WIDGET_H
#define     QVIEWER_WIDGET_H

#include    <QWidget>

#include    <osgViewer/Viewer>
#include    <osgQt/GraphicsWindowQt>


class QViewerWidget : public QWidget
{
public:

    QViewerWidget(const QRect &geometry);

    virtual ~QViewerWidget();

    osg::Group *getScene();

    osgViewer::Viewer *getViewer();

    // [0, 255]
    void set_background_color(int r, int g,int b);
    void set_points_color(int r, int g,int b);

protected:

    osg::ref_ptr<osg::Group> scene;

    osgViewer::Viewer   viewer;

private:

    osgQt::GraphicsWindowQt *createGraphicsWindow(const QRect &geometry);

    void initCamera(const QRect &geometry);

    void paintEvent(QPaintEvent *);

    void initial_scene();
};

#endif // QVIEWER_WIDGET_H
