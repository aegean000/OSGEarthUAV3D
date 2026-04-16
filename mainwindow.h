#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer> // 如果你要做高度计，会用到定时器

// OSG 核心头文件
#include <osg/ref_ptr>
#include <osg/PositionAttitudeTransform>
#include <osgEarth/MapNode>


// 预留 QOSGWidget 的声明（如果不包含头文件，可以用前置声明）
class QOSGWidget;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    // 按钮槽函数
    void on_btnTogglePlane_clicked();

    void on_sliderProgress_valueChanged(int value);

private:
    Ui::MainWindow *ui;

    // 成员变量
    QOSGWidget* _osgWidget;
    osg::ref_ptr<osg::PositionAttitudeTransform> _planePat;
    osg::observer_ptr<osgEarth::MapNode> _mapNode;
};


#endif // MAINWINDOW_H