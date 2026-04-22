#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer> // 如果你要做高度计，会用到定时器

// OSG 核心头文件
#include <osg/ref_ptr>
#include <osg/PositionAttitudeTransform>
#include <osgEarth/MapNode>
#include <QElapsedTimer>
#include <QListWidgetItem>

// 预留 QOSGWidget 的声明（如果不包含头文件，可以用前置声明）
class QOSGWidget;
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// 轨迹点结构：包含空间坐标与时间戳
struct TrackPoint {
    double timeOffset;    // 时间轴位置（单位：秒）
    osg::Vec3d worldPos;  // 转换后的世界坐标 (ECEF)
    osg::Quat rotation;   // 姿态（朝向）
    double velocity;      // 瞬时速度（用于 UI 显示）
};
// 单个飞机结构
struct TrackObject {
    QString id;

    osg::ref_ptr<osg::PositionAttitudeTransform> planePat; // 飞机

    osg::ref_ptr<osg::Geode> projLineGeode;                // 投影线容器
    osg::ref_ptr<osg::Geometry> projLineGeom;              // 投影线几何体
    osg::ref_ptr<osg::Vec3Array> projLineVertices;         // 投影线顶点

    osg::ref_ptr<osg::Geode> pathGeode;                    // 航迹线容器
    osg::ref_ptr<osg::Geometry> pathGeom;                  // 航迹线几何体

    std::vector<TrackPoint> points;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    friend class CameraFollowCallback;
    void initProjectionLine(osg::Group* root);

private slots:
    // 按钮槽函数
    void on_sliderProgress_valueChanged(int value);
    void on_btnResetView_clicked();

    void on_btnImportData_clicked();

    void on_btnPlayPause_clicked();

    void on_comboBox_currentTextChanged(const QString &arg1);

    void on_btnPitchUp_clicked();

    void on_btnPitchDown_clicked();

    void on_checkShowProjection_clicked(bool checked);

    void on_listWidgetTracks_itemClicked(QListWidgetItem *item);

    void on_btnDeleteTrack_clicked();

private:
    Ui::MainWindow *ui;

    // 成员变量
    QOSGWidget* _osgWidget;
    // --- 场景相关变量 ---
    osg::ref_ptr<osg::Geometry> _pathGeom; // 航迹线几何体
    osg::ref_ptr<osg::PositionAttitudeTransform> _planePat;
    osg::observer_ptr<osgEarth::MapNode> _mapNode;
    // 投影线相关成员变量
    osg::ref_ptr<osg::Geometry> _projLineGeom;
    osg::ref_ptr<osg::Vec3Array> _projLineVertices;
    osg::ref_ptr<osg::Geode> _projLineGeode;

    // 单元成员变量
    QMap<QString, TrackObject> _allTracks; // 存储所有航迹
    QString _currentTrackId;               // 当前选中的航迹 ID

    bool _isFollowing = false;
    QElapsedTimer _resetTimer;
    double _lastRange = 0.0; // ✅ 用于检测缩放
    void updatePathRender(osg::Vec3Array* va);
    // 记录当前是否处于暂停状态（初始化建议为 false）
    bool _isPaused = false;


};





#endif // MAINWINDOW_H