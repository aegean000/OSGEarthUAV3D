#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

// OSG 核心头文件
#include <osg/ref_ptr>
#include <osg/PositionAttitudeTransform>
#include <osgEarth/MapNode>
#include <QElapsedTimer>
#include <QListWidgetItem>

class QOSGWidget;
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// 轨迹点结构：包含空间坐标与时间戳
struct TrackPoint {
    double timeOffset = 0.0;   // 相对首点的秒偏移（程序内部使用）
    osg::Vec3d worldPos;       // 世界坐标
    osg::Quat rotation;        // 姿态（由 heading_deg 转换）
    double velocity = 0.0;     // 真实速度，单位 m/s

    double lon = 0.0;          // 方便信息面板和后续扩展
    double lat = 0.0;
    double alt = 0.0;

    double headingDeg = 0.0;   // CSV里直接提供
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

    // ===== 当前航迹信息面板所需 =====
    int pointCount = 0;
    double totalDuration = 0.0;
    double startLon = 0.0;
    double startLat = 0.0;
    double endLon = 0.0;
    double endLat = 0.0;
    QString startTimeText;
    QString endTimeText;
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

    QOSGWidget* _osgWidget;

    osg::ref_ptr<osg::Geometry> _pathGeom;
    osg::ref_ptr<osg::PositionAttitudeTransform> _planePat;
    osg::observer_ptr<osgEarth::MapNode> _mapNode;

    osg::ref_ptr<osg::Geometry> _projLineGeom;
    osg::ref_ptr<osg::Vec3Array> _projLineVertices;
    osg::ref_ptr<osg::Geode> _projLineGeode;

    QMap<QString, TrackObject> _allTracks;
    QString _currentTrackId;

    bool _isFollowing = false;
    QElapsedTimer _resetTimer;
    double _lastRange = 0.0;

    void updatePathRender(osg::Vec3Array* va);
    void refreshTrackInfoPanel();   // ✅ 新增

    bool _isPaused = false;
};

#endif // MAINWINDOW_H