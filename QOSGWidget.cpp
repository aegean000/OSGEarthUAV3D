#include "QOSGWidget.h"
#include <QMouseEvent>
#include <osgEarth/GLUtils>

QOSGWidget::QOSGWidget(QWidget* parent) : QOpenGLWidget(parent)
{
    _viewer = new osgViewer::Viewer();
    _gw = _viewer->setUpViewerAsEmbeddedInWindow(0, 0, width(), height());

    // 获取相机状态集
    osg::StateSet* ss = _viewer->getCamera()->getOrCreateStateSet();

    //注意：getState() 之前最好判空，虽然 EmbeddedWindow 通常会自动创建 context
    if(_viewer->getCamera()->getGraphicsContext()) {
        // 1. 开启顶点属性别名（将 gl_Vertex 映射到属性指针）
        _viewer->getCamera()->getGraphicsContext()->getState()->setUseVertexAttributeAliasing(true);
        // 2. 开启矩阵统一变量（将投影/模型矩阵映射到 Uniform）
        _viewer->getCamera()->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(true);
    }
    // 3. 必须在 Viewer 启动前设置这个 RealizeOperation
    _viewer->setRealizeOperation(new osgEarth::GL3RealizeOperation());
    // --- 关键修复代码结束 ---

    osgEarth::GLUtils::setGlobalDefaults(ss);

    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
}

void QOSGWidget::paintGL() {
    _viewer->frame();
    update(); // 持续触发重绘，保证动画流畅
}

void QOSGWidget::resizeGL(int w, int h) {
    // 1. 算出当前的设备像素比（防止高分屏/缩放导致的错位）
    float ratio = this->devicePixelRatioF();
    int width = static_cast<int>(w * ratio);
    int height = static_cast<int>(h * ratio);

    // 2. 通知 OSG 窗口系统：窗口尺寸变了
    _gw->getEventQueue()->windowResize(0, 0, width, height);
    _gw->resized(0, 0, width, height);

    // 3. 极其重要：通知相机更新视口矩形
    // 如果没有这一行，地球就会缩在左下角的旧尺寸里
    _viewer->getCamera()->setViewport(0, 0, width, height);

    // 4. 更新投影矩阵（防止地球被拉扁）
    _viewer->getCamera()->setProjectionMatrixAsPerspective(
        30.0f, static_cast<double>(width) / static_cast<double>(height), 1.0f, 10000.0f);
}

// 示例：将 Qt 鼠标点击传递给 OSG 漫游器
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