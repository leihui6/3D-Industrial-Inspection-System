#include    "qviewerwidget.h"

#include    <osgViewer/ViewerEventHandlers>
#include    <osgGA/TrackballManipulator>

#include    <QGridLayout>

QViewerWidget::QViewerWidget(const QRect &geometry)
    : QWidget()
    , scene(new osg::Group)
{
    initCamera(geometry);

    viewer.setSceneData(scene);
    viewer.addEventHandler(new osgViewer::StatsHandler);
    viewer.setCameraManipulator(new osgGA::TrackballManipulator);
    viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);

    osgQt::GraphicsWindowQt *gw = static_cast<osgQt::GraphicsWindowQt *>(viewer.getCamera()->getGraphicsContext());

    QGridLayout *layout = new QGridLayout;

    if (layout != Q_NULLPTR)
    {
        layout->addWidget(gw->getGLWidget());
        this->setLayout(layout);
    }

    initial_scene();
}

QViewerWidget::~QViewerWidget()
{

}

osg::Group *QViewerWidget::getScene()
{
    return scene.get();
}

osgViewer::Viewer *QViewerWidget::getViewer()
{
    return &viewer;
}

void QViewerWidget::set_background_color(int r, int g, int b)
{
    osg::ref_ptr<osg::Camera> camera = this->getViewer()->getCamera();

    camera->setClearColor(osg::Vec4(r/255.0f,g/255.0f,b/255.0f,1.0));
}

void QViewerWidget::set_points_color(const std::string &point_cloud_name, int r, int g, int b)
{
    if (node_map.find(point_cloud_name) == node_map.end()) return;

    osg::ref_ptr<osg::Node> node = node_map[point_cloud_name];
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
    colors->push_back(osg::Vec4(r/255.0f,g/255.0f,b/255.0f,1));

    // set color
    node->asGeode()->getChild(0)->asGeometry()->setColorArray(colors.get());
    node->asGeode()->getChild(0)->asGeometry()->setColorBinding(osg::Geometry::BIND_OVERALL);
}

void QViewerWidget::set_points_size(const std::string &point_cloud_name, int size_number)
{
    auto it = node_map.find(point_cloud_name);
    if (it == node_map.end()) return;

    osg::StateSet* stateSet = it->second->asGeode()->getChild(0)->asGeometry()->getOrCreateStateSet();
    stateSet->setAttribute(new osg::Point(size_number));
}

void QViewerWidget::add_point_cloud(const std::string &point_cloud_name, std::vector<point_3d> & points)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

    points_to_geometry_node(points, geometry, 255,255,255,1);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, int(points.size())));
    geode->addDrawable(geometry);

    // add a new one
    if(node_map.find(point_cloud_name) == node_map.end())
    {
       scene->addChild(geode);
       node_map[point_cloud_name] = geode;
    }
    // replace the current one
    else
    {
       scene->replaceChild(node_map[point_cloud_name].get(),geode);
       node_map[point_cloud_name] = geode;
    }
}

osgQt::GraphicsWindowQt *QViewerWidget::createGraphicsWindow(const QRect &geometry)
{
    osg::DisplaySettings *ds = osg::DisplaySettings::instance().get();

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = "";
    traits->windowDecoration = false;
    traits->x = geometry.x();
    traits->y = geometry.y();
    traits->width = geometry.width();
    traits->height = geometry.height();

    if (traits->height == 0) traits->height = 1;

    traits->doubleBuffer = true;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = ds->getMultiSamples();
    traits->samples = ds->getNumMultiSamples();

    return new osgQt::GraphicsWindowQt(traits.get());
}

void QViewerWidget::initCamera(const QRect &geometry)
{
    osg::Camera *camera = viewer.getCamera();

    osg::ref_ptr<osgQt::GraphicsWindowQt> gw = createGraphicsWindow(geometry);
    gw->setTouchEventsEnabled(true);
    camera->setGraphicsContext(gw.get());

    const osg::GraphicsContext::Traits *traits = gw->getTraits();
    camera->setClearColor(osg::Vec4(0.7f, 0.7f, 0.7f, 1.0f));
    camera->setViewport(0, 0, traits->width, traits->height);

    double aspect = static_cast<double>(traits->width) / static_cast<double>(traits->height);
    camera->setProjectionMatrixAsPerspective(30.0, aspect, 1.0, 1000.0);
    GLenum buffer = (traits->doubleBuffer) ? GL_BACK : GL_FRONT;
    camera->setDrawBuffer(buffer);
    camera->setReadBuffer(buffer);

    // Solve the problem that one point in geode does not show
    osg::CullStack::CullingMode cullingMode = camera->getCullingMode();
    cullingMode &= ~(osg::CullStack::SMALL_FEATURE_CULLING);
    camera->setCullingMode(cullingMode);
}

void QViewerWidget::paintEvent(QPaintEvent *)
{
    viewer.frame();
}

void QViewerWidget::initial_scene()
{
    set_background_color(135, 206, 235);
}
