#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QElapsedTimer>
#include <QListWidgetItem>
#include <QMainWindow>
#include <QTimer>

#include <osg/PositionAttitudeTransform>
#include <osg/ref_ptr>
#include <osgEarth/MapNode>

#include "refactor_src/TrackManager.h"
#include "refactor_src/SystemConfig.h"

class QOSGWidget;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// 主窗口类负责界面交互、流程调度、视角控制和当前航迹状态维护。
// 数据读取、地图初始化、节点构建和多航迹管理等功能由独立模块完成。
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // 视角跟随回调需要访问主窗口中的当前航迹和相机状态。
    friend class CameraFollowCallback;

private slots:
    void on_sliderProgress_valueChanged(int value);              // 处理回放进度条变化。
    void on_btnResetView_clicked();                              // 处理视角复位操作。
    void on_btnImportData_clicked();                             // 处理航迹文件导入操作。
    void on_btnPlayPause_clicked();                              // 处理航迹播放与暂停切换。
    void on_comboBox_currentTextChanged(const QString &arg1);    // 处理播放倍率切换。
    void on_btnPitchUp_clicked();                                // 处理视角俯仰角上调。
    void on_btnPitchDown_clicked();                              // 处理视角俯仰角下调。
    void on_checkShowProjection_clicked(bool checked);           // 处理投影线显隐控制。
    void on_checkShowPoints_clicked(bool checked);               // 处理航迹点显隐控制。
    void on_checkShowBasePlane_clicked(bool checked);            // 处理高度参考面显隐控制。
    void on_listWidgetTracks_itemClicked(QListWidgetItem *item); // 处理航迹列表选中事件。
    void on_btnDeleteTrack_clicked();                            // 处理航迹删除操作。

private:
    Ui::MainWindow *ui;

    // 嵌入 Qt 界面的 OSG 渲染控件。
    QOSGWidget* _osgWidget;

    // 当前选中航迹在场景中的主要显示对象。
    osg::ref_ptr<osg::Geometry> _pathGeom;
    osg::ref_ptr<osg::PositionAttitudeTransform> _planePat;
    osg::observer_ptr<osgEarth::MapNode> _mapNode;

    // 当前选中航迹的投影线对象，用于显示无人机到地面的高度关系。
    osg::ref_ptr<osg::Geometry> _projLineGeom;
    osg::ref_ptr<osg::Vec3Array> _projLineVertices;
    osg::ref_ptr<osg::Geode> _projLineGeode;

    // 多航迹数据管理对象，负责航迹保存、查询和当前航迹维护。
    TrackManager _trackManager;

    // 视角跟随状态和复位动画保护参数。
    bool _isFollowing = false;
    double _lastRange = 0.0;
    QElapsedTimer _resetTimer;

    // 当前播放状态和视角跟随参数。
    bool _isPaused = false;
    double _followRange = SystemConfig::View::FollowRange;
    double _followPitch = SystemConfig::View::FollowPitch;
    double _followDuration = SystemConfig::View::FollowDuration;
    bool updateFollowCamera(bool instant);
    void focusCurrentAircraft(bool instant);

    // 刷新航迹信息面板中的点数、时长、速度和异常提示等内容。
    void refreshTrackInfoPanel();
};

#endif // MAINWINDOW_H
