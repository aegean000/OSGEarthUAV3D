#ifndef QOSGWIDGET_H
#define QOSGWIDGET_H

#include <QOpenGLWidget>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osgViewer/api/Win32/GraphicsWindowWin32>

// Qt OpenGL 控件封装类，负责在界面中嵌入 OSG Viewer。
class QOSGWidget : public QOpenGLWidget
{
    Q_OBJECT
public:
    QOSGWidget(QWidget* parent = nullptr);
    osgViewer::Viewer* getViewer() { return _viewer.get(); }

protected:
    virtual void paintGL() override;
    virtual void resizeGL(int w, int h) override;

    // 将 Qt 输入事件转发给 OSG 事件队列。
    virtual void mouseMoveEvent(QMouseEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void mouseReleaseEvent(QMouseEvent* event) override;
    virtual void wheelEvent(QWheelEvent* event) override;

private:
    osg::ref_ptr<osgViewer::Viewer> _viewer;
    osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> _gw;
};

#endif
