#include "QOSGWidget.h"
#include <QDebug>
#include <QMouseEvent>
#include <osgEarth/GLUtils>

// 该控件负责桥接 Qt OpenGL 窗口和 OSG 渲染循环。

// 构造嵌入式 OSG Viewer，并配置 osgEarth 所需的 OpenGL 状态。
QOSGWidget::QOSGWidget(QWidget* parent) : QOpenGLWidget(parent)
{
    _viewer = new osgViewer::Viewer();
    _gw = _viewer->setUpViewerAsEmbeddedInWindow(0, 0, width(), height());

    // 设置相机状态和 GL3 兼容参数。
    osg::StateSet* ss = _viewer->getCamera()->getOrCreateStateSet();

    if(_viewer->getCamera()->getGraphicsContext()) {
        _viewer->getCamera()->getGraphicsContext()->getState()->setUseVertexAttributeAliasing(true);
        _viewer->getCamera()->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(true);
    }
    _viewer->setRealizeOperation(new osgEarth::GL3RealizeOperation());

    osgEarth::GLUtils::setGlobalDefaults(ss);

    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    qInfo() << "[地图加载]" << "QOSGWidget 初始化完成";
}

void QOSGWidget::paintGL() {
    _viewer->frame();
    update(); // 持续重绘以保证动画连续。
}

void QOSGWidget::resizeGL(int w, int h) {
    // 根据设备像素比同步 Qt 控件尺寸与 OSG 视口尺寸。
    float ratio = this->devicePixelRatioF();
    int width = static_cast<int>(w * ratio);
    int height = static_cast<int>(h * ratio);

    _gw->getEventQueue()->windowResize(0, 0, width, height);
    _gw->resized(0, 0, width, height);

    _viewer->getCamera()->setViewport(0, 0, width, height);

    // 更新投影矩阵，保持场景比例稳定。
    _viewer->getCamera()->setProjectionMatrixAsPerspective(
        30.0f, static_cast<double>(width) / static_cast<double>(height), 1.0f, 10000.0f);
}

// 将 Qt 鼠标和滚轮事件转发给 OSG 漫游器。
void QOSGWidget::mousePressEvent(QMouseEvent* event) {
    _gw->getEventQueue()->mouseButtonPress(event->x(), event->y(), event->button());
}
void QOSGWidget::mouseReleaseEvent(QMouseEvent* event) {
    _gw->getEventQueue()->mouseButtonRelease(event->x(), event->y(), event->button());
}
void QOSGWidget::mouseMoveEvent(QMouseEvent* event) {
    _gw->getEventQueue()->mouseMotion(event->x(), event->y());
}
void QOSGWidget::wheelEvent(QWheelEvent* event) {
    _gw->getEventQueue()->mouseScroll(event->angleDelta().y() > 0 ? osgGA::GUIEventAdapter::SCROLL_UP : osgGA::GUIEventAdapter::SCROLL_DOWN);
}
