#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "QOSGWidget.h"
#include "refactor_src/TrackLoader.h"
#include "refactor_src/MapSceneManager.h"
#include "refactor_src/TrackNodeBuilder.h"
#include "refactor_src/TrackAnimationCallback.h"
#include "refactor_src/TrackPlaybackController.h"
#include "refactor_src/SystemConfig.h"

// 主窗口实现界面初始化、用户交互响应和各功能模块的调用编排。

// Qt 相关
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>


// OSG 核心基础
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/PolygonOffset>

// osgEarth 核心
#include <osgEarth/MapNode>        // 地图核心节点
#include <osgEarth/EarthManipulator> // 漫游器
#include <osgEarth/GeoData>        // 坐标转换 (GeoPoint 等)
#include <osgEarth/Units>          // 单位转换
#include <osgEarth/Registry>

// 标准库
#include <vector>

using namespace osgEarth;
using namespace osgEarth::Util;

// ==============================================
// 配置项结构体
// ==============================================
struct FlightConfig {
    double speed = 0.002;
    bool showPlane = true;
    bool showPath = true;
    float planeScale = SystemConfig::Render::PlaneScale;
    osg::Vec4 pathColor = osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
    float lineWidth = SystemConfig::Render::PathLineWidth;
};
// ==============================================
// 视角跟随回调类
// ==============================================
class CameraFollowCallback : public osg::NodeCallback
{
public:
    MainWindow* _mainWin;
    CameraFollowCallback(MainWindow* win) : _mainWin(win) {}

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        if (_mainWin && _mainWin->_isFollowing && _mainWin->_planePat.valid())
        {
            osgEarth::Util::EarthManipulator* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(
                _mainWin->_osgWidget->getViewer()->getCameraManipulator());

            if (manip) {
                // 复位动画执行期间暂停自动跟随更新
                if (_mainWin->_resetTimer.isValid() && _mainWin->_resetTimer.elapsed() < SystemConfig::View::FollowCallbackGuardMs) {
                    this->traverse(node, nv);
                    return;
                }
                _mainWin->updateFollowCamera(true);
            }
        }
        this->traverse(node, nv);
    }
};
// ==============================================
// MainWindow 实现
// ==============================================
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // 初始关闭航迹点和 500m 参考面显示
    if (ui->checkShowPoints) {
        ui->checkShowPoints->setChecked(false);
    }

    if (ui->checkShowBasePlane) {
        ui->checkShowBasePlane->setChecked(false);
    }


    // 创建并嵌入 QOSGWidget
    QOSGWidget* osgWidget = new QOSGWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(ui->renderBase);
    layout->addWidget(osgWidget);
    layout->setContentsMargins(0, 0, 0, 0);
    this->_osgWidget = osgWidget;
    qInfo() << "[地图加载]" << "osgViewer 初始化完成";

    // 基础显示配置
    FlightConfig config;
    config.speed = 0.002;
    config.showPath = true;
    config.showPlane = true;
    // ==============================================
    // 构建地图节点
    // ==============================================
    MapSceneManager mapSceneManager;
    if (!mapSceneManager.initialize(SystemConfig::Resource::MapFile)) {
        if (mapSceneManager.errorType() == MapSceneManager::EarthFileLoadFailed) {
            QMessageBox::critical(this, "错误", "无法加载 mymap.earth 文件！请确保它在运行目录下。");
        } else {
            QMessageBox::critical(this, "错误", "文件加载成功但未发现有效的 MapNode。");
        }
        return;
    }

    this->_mapNode = mapSceneManager.mapNode();

    // ==============================================
    // 创建场景根节点
    // ==============================================
    osg::ref_ptr<osg::Group> root = new osg::Group();
    root->addChild(this->_mapNode.get());
    qInfo() << "[地图加载]" << "场景根节点初始化完成";
    // 初始化空航线
    if (config.showPath) {
        _pathGeom = new osg::Geometry(); // 赋值给成员变量

        // 初始化空顶点数组
        osg::ref_ptr<osg::Vec3Array> emptyVa = new osg::Vec3Array();
        _pathGeom->setVertexArray(emptyVa);

        // 无航迹点时不添加绘制指令

        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
        colors->push_back(config.pathColor);
        _pathGeom->setColorArray(colors, osg::Array::BIND_OVERALL);

        osg::ref_ptr<osg::Geode> lineGeode = new osg::Geode();
        lineGeode->addDrawable(_pathGeom.get());

        osg::StateSet* ss = lineGeode->getOrCreateStateSet();
        // 开启深度偏移，防止线嵌入地面
        ss->setAttributeAndModes(new osg::PolygonOffset(1.0f, 1.0f), osg::StateAttribute::ON);
        // 关闭光照
        ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
        // 设置线宽
        ss->setAttributeAndModes(new osg::LineWidth(config.lineWidth), osg::StateAttribute::ON);

        osgEarth::Registry::shaderGenerator().run(lineGeode);
        root->addChild(lineGeode);

        // 设置渲染队列，保证航迹显示在基础地图之上
        ss->setRenderBinDetails(1, "RenderBin");
    }
    osg::ref_ptr<osg::PositionAttitudeTransform> pat;
    _planePat = new osg::PositionAttitudeTransform();   // 将其赋值给 _planePat，方便点击事件控制
    // 初始化飞机模型
    if (config.showPlane) {
        // 导入 CSV 前隐藏飞机
        _planePat->setNodeMask(0x0);

        qInfo() << "[模型回放]" << "开始加载无人机模型：" << SystemConfig::Resource::DroneModelFile;
        osg::ref_ptr<osg::Node> model =
            osgDB::readNodeFile(SystemConfig::Resource::DroneModelFile);

        if (model.valid()) {
            qInfo() << "[模型回放]" << "无人机模型加载成功";

            // 缩放控制
            osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform();
            mt->setMatrix(osg::Matrix::scale(config.planeScale, config.planeScale, config.planeScale));
            mt->addChild(model);

            _planePat->addChild(mt);

            // 导入 CSV 后绑定 TrackAnimationCallback
            root->addChild(_planePat);
        } else {
            qWarning() << "[模型回放]" << "无人机模型预加载失败：" << SystemConfig::Resource::DroneModelFile;
        }
    }
    // 初始化按钮状态
    ui->btnResetView->setEnabled(false);
    ui->sliderProgress->setEnabled(false);
    ui->labelState->setText("状态：等待导入数据");
    if (ui->labelTrackInfo) ui->labelTrackInfo->setText("当前航迹信息：暂无");
    // 设置 Viewer 场景数据
    osgWidget->getViewer()->setSceneData(root);

    // 设置地图漫游器
    mapSceneManager.setupEarthManipulator(osgWidget->getViewer());
    qInfo() << "[地图加载]" << "三维地图场景初始化完成";

    // 设置进度条初始范围
    ui->sliderProgress->setRange(0, 1000); // 对应 coords 数组的索引 0, 1, 2

    // 定时刷新 UI 状态
    QTimer* uiTimer = new QTimer(this);
    connect(uiTimer, &QTimer::timeout, this, [this]() {
        // 基础指针检查
        if (!_osgWidget || !_planePat.valid() || !_mapNode.valid()) return;
        osgViewer::Viewer* viewer = _osgWidget->getViewer();
        osgEarth::Util::EarthManipulator* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(
            viewer->getCameraManipulator());

        if (manip) {
            // 用户交互检测
            const osgGA::GUIEventAdapter* ea = viewer->getEventQueue()->getCurrentEventState();
            double currentRange = 0.0;
            if (manip->getViewpoint().range().isSet()) {
                // 转换为米制距离
                currentRange = manip->getViewpoint().range().value().as(osgEarth::Units::METERS);
            }

            if (_isFollowing) {
                if (_resetTimer.isValid() && _resetTimer.elapsed() < SystemConfig::View::UserInteractionGuardMs) {
                    _lastRange = currentRange;
                } else {
                    bool mousePressed = (ea && ea->getButtonMask() != 0);
                    bool rangeChanged = (_lastRange > 0 && std::abs(currentRange - _lastRange) > SystemConfig::View::RangeChangeThreshold);

                    if (mousePressed || rangeChanged) {
                        _isFollowing = false;
                        qInfo() << "[视角控制]" << "自动跟随关闭，切换为手动观察";
                        ui->labelState->setText("状态：手动模式");
                    }
                }
            }
            // 记录当前视距用于下一次比较
            _lastRange = currentRange;
        }
        // 获取飞机当前位置
        osg::Vec3d worldPos = _planePat->getPosition();
        osgEarth::GeoPoint geoPos;
        if (!geoPos.fromWorld(_mapNode->getMapSRS(), worldPos)) return;

        // 同步位置显示
        ui->labelPos->setText(QString("位置: %1, %2")
                                  .arg(geoPos.x(), 0, 'f', 4)
                                  .arg(geoPos.y(), 0, 'f', 4));

        // 同步进度条状态
        TrackPlaybackController playbackController;
        playbackController.bind(_planePat.get());
        if (playbackController.hasCallback()) {
            ui->sliderProgress->blockSignals(true);
            ui->sliderProgress->setValue(static_cast<int>(playbackController.currentTime() * SystemConfig::Playback::SliderTimeScale));
            ui->sliderProgress->blockSignals(false);

            int curSec = (int)playbackController.currentTime();
            int totSec = (int)playbackController.totalDuration();

            // 同步播放倍率显示
            double currentSpeed = playbackController.timeScale();
            QString speedStr = (currentSpeed == 1.0) ? "" : QString(" [%1x]").arg(currentSpeed);

            // 同步播放状态
            QString stateStr = playbackController.isPaused() ? "已暂停" : "正在回放";

            // 显示播放进度和状态
            ui->labelState->setText(QString("进度: %1s / %2s | %3%4")
                                        .arg(curSec)
                                        .arg(totSec)
                                        .arg(stateStr)
                                        .arg(speedStr));
        }
        refreshTrackInfoPanel();

    });
    uiTimer->start(SystemConfig::Playback::UiRefreshIntervalMs); // 100 毫秒刷新一次
    // 通过 MapNode 更新回调实现自动跟随
    _mapNode->addUpdateCallback(new CameraFollowCallback(this));


}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::refreshTrackInfoPanel()
{
    if (!ui->labelTrackInfo) return;

    const TrackObject* trackPtr = _trackManager.currentTrack();
    if (!trackPtr) {
        ui->labelTrackInfo->setText("当前航迹信息：暂无");
        return;
    }

    const TrackObject& track = *trackPtr;

    double currentSpeed = 0.0;
    if (track.planePat.valid()) {
        TrackPlaybackController playbackController;
        playbackController.bind(track.planePat.get());
        currentSpeed = playbackController.currentVelocity();
    }

    QString speedCheckText;

    if (track.abnormalSpeedCount > 0) {
        speedCheckText = QString("速度校验：发现 %1 个异常航段，%2 个可疑航段\n"
                                 "最大计算速度：%3 m/s\n"
                                 "最大速度误差：%4 m/s\n")
                             .arg(track.abnormalSpeedCount)
                             .arg(track.warningSpeedCount)
                             .arg(track.maxActualSpeed, 0, 'f', 1)
                             .arg(track.maxSpeedError, 0, 'f', 1);
    }
    else if (track.warningSpeedCount > 0) {
        speedCheckText = QString("速度校验：未发现异常，存在 %1 个可疑航段\n"
                                 "最大计算速度：%2 m/s\n"
                                 "最大速度误差：%3 m/s\n")
                             .arg(track.warningSpeedCount)
                             .arg(track.maxActualSpeed, 0, 'f', 1)
                             .arg(track.maxSpeedError, 0, 'f', 1);
    }
    else {
        speedCheckText = QString("速度校验：正常\n"
                                 "最大计算速度：%1 m/s\n"
                                 "最大速度误差：%2 m/s\n")
                             .arg(track.maxActualSpeed, 0, 'f', 1)
                             .arg(track.maxSpeedError, 0, 'f', 1);
    }

    ui->labelTrackInfo->setText(
        QString("航迹ID：%1\n"
                "点数：%2\n"
                "真实时长：%3 秒\n"
                "当前速度：%4 m/s\n"
                "%5"
                "起点：(%6, %7)\n"
                "终点：(%8, %9)")
            .arg(track.id)
            .arg(track.pointCount)
            .arg(track.totalDuration, 0, 'f', 1)
            .arg(currentSpeed, 0, 'f', 1)
            .arg(speedCheckText)
            .arg(track.startLon, 0, 'f', 4)
            .arg(track.startLat, 0, 'f', 4)
            .arg(track.endLon, 0, 'f', 4)
            .arg(track.endLat, 0, 'f', 4)
        );
}

bool MainWindow::updateFollowCamera(bool instant)
{
    if (!_osgWidget || !_planePat.valid() || !_mapNode.valid()) return false;

    osgEarth::Util::EarthManipulator* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(
        _osgWidget->getViewer()->getCameraManipulator());
    if (!manip) return false;

    osgEarth::GeoPoint geoPos;
    if (!geoPos.fromWorld(_mapNode->getMapSRS(), _planePat->getPosition()) || !geoPos.isValid()) {
        return false;
    }

    osgEarth::Viewpoint vp("Follow",
                           geoPos.x(),
                           geoPos.y(),
                           geoPos.z(),
                           0.0,
                           _followPitch,
                           _followRange);
    manip->setViewpoint(vp, instant ? 0.0 : _followDuration);
    _lastRange = _followRange;
    return true;
}

void MainWindow::focusCurrentAircraft(bool instant)
{
    if (!updateFollowCamera(instant)) return;

    _resetTimer.start();
    _isFollowing = true;
    ui->labelState->setText("状态：自动跟随");
    qInfo() << "[视角控制]" << (instant ? "快速定位到当前无人机并开启自动跟随"
                                      : "定位到当前无人机并开启自动跟随");
}

// UI 交互槽函数

// 进度条跳转

void MainWindow::on_sliderProgress_valueChanged(int value)
{
    if (_planePat.valid()) {
        TrackPlaybackController playbackController;
        playbackController.bind(_planePat.get());
        double targetTime = value / static_cast<double>(SystemConfig::Playback::SliderTimeScale);
        qInfo() << "[播放控制]" << "进度条跳转，目标时间：" << targetTime << "s";
        playbackController.setTime(targetTime); // slider 的整数值转回秒
    }
}

// 复位视角
void MainWindow::on_btnResetView_clicked()
{
    qInfo() << "[视角控制]" << "用户点击视角重置";
    focusCurrentAircraft(true);
}

// 导入航迹数据
void MainWindow::on_btnImportData_clicked()
{
    qInfo() << "[航迹导入]" << "用户点击导入航迹按钮";
    QString path = QFileDialog::getOpenFileName(this, "导入航迹", "", "CSV文件 (*.csv)");
    if (path.isEmpty()) {
        qWarning() << "[航迹导入]" << "用户取消选择 CSV 文件";
        return;
    }
    qInfo() << "[航迹导入]" << "选择文件：" << path;

    TrackLoader loader;
    TrackLoadResult loadResult = loader.loadCsv(path, _mapNode.get());
    if (!loadResult.ok) {
        qWarning() << "[航迹导入]" << "导入失败：" << loadResult.errorMessage;
        if (loadResult.errorType == TrackLoadResult::OpenFailed) {
            QMessageBox::warning(this, "错误", "无法打开所选文件！");
        } else {
            QMessageBox::critical(this, "导入失败", "航迹数据为空，请检查CSV格式是否为：track_id,timestamp,longitude,latitude,altitude_m,speed_mps,heading_deg");
        }
        return;
    }

    std::vector<TrackPoint> points = loadResult.points;
    osg::ref_ptr<osg::Vec3Array> lineCoords = loadResult.lineCoords;
    QString csvTrackId = loadResult.csvTrackId;
    int warningSpeedCount = loadResult.warningSpeedCount;
    int abnormalSpeedCount = loadResult.abnormalSpeedCount;
    double maxActualSpeed = loadResult.maxActualSpeed;
    double maxSpeedError = loadResult.maxSpeedError;


    // 生成唯一 trackId
    QString trackId = _trackManager.makeUniqueTrackId(csvTrackId, path);

    osg::Group* root = _osgWidget->getViewer()->getSceneData()->asGroup();
    if (!root) return;

    TrackRenderConfig renderConfig;
    TrackNodeBuilder nodeBuilder;
    TrackObject newTrack = nodeBuilder.buildTrackObject(
        trackId,
        loadResult,
        root,
        _mapNode.get(),
        ui->checkShowProjection->isChecked(),
        renderConfig);

    osg::ref_ptr<osg::Geometry> pathGeom = newTrack.pathGeom;
    osg::ref_ptr<osg::Geode> pathGeode = newTrack.pathGeode;
    osg::ref_ptr<osg::Geode> pointGeode = newTrack.pointGeode;
    osg::ref_ptr<osg::Geode> basePlaneGeode = newTrack.basePlaneGeode;
    osg::ref_ptr<osg::Geometry> projLineGeom = newTrack.projLineGeom;
    osg::ref_ptr<osg::Vec3Array> projLineVertices = newTrack.projLineVertices;
    osg::ref_ptr<osg::Geode> projLineGeode = newTrack.projLineGeode;
    osg::ref_ptr<osg::PositionAttitudeTransform> planePat = newTrack.planePat;

    auto* newCb = new TrackAnimationCallback(points,
                                            projLineVertices.get(),
                                            projLineGeom.get(),
                                            _mapNode.get());

    QString curTxt = ui->comboBox->currentText();
    double curSpeed = curTxt.left(curTxt.indexOf('x')).toDouble();
    planePat->setUpdateCallback(newCb);
    qInfo() << "[模型回放]" << "回放回调初始化完成";

    TrackPlaybackController newPlaybackController;
    newPlaybackController.bind(planePat.get());
    newPlaybackController.setTimeScale((curSpeed > 0) ? curSpeed : 1.0);
    newPlaybackController.setPaused(true);

    // 设置飞机初始位置
    planePat->setPosition(points.front().worldPos);
    planePat->setAttitude(points.front().rotation);
    qInfo() << "[模型回放]" << "模型初始位置设置完成";

    // =========================
    // 写入 TrackObject
    // =========================
    newTrack.id = trackId;
    newTrack.points = points;
    newTrack.planePat = planePat;
    newTrack.pathGeode = pathGeode;
    newTrack.pathGeom = pathGeom;
    newTrack.pointGeode = pointGeode;
    newTrack.basePlaneGeode = basePlaneGeode;
    newTrack.projLineGeode = projLineGeode;
    newTrack.projLineGeom = projLineGeom;
    newTrack.projLineVertices = projLineVertices;

    newTrack.pointCount = static_cast<int>(points.size());
    newTrack.totalDuration = points.back().timeOffset;
    newTrack.warningSpeedCount = warningSpeedCount;
    newTrack.abnormalSpeedCount = abnormalSpeedCount;
    newTrack.maxActualSpeed = maxActualSpeed;
    newTrack.maxSpeedError = maxSpeedError;

    osgEarth::GeoPoint startGeo, endGeo;
    if (startGeo.fromWorld(_mapNode->getMapSRS(), points.front().worldPos)) {
        newTrack.startLon = startGeo.x();
        newTrack.startLat = startGeo.y();
    }
    if (endGeo.fromWorld(_mapNode->getMapSRS(), points.back().worldPos)) {
        newTrack.endLon = endGeo.x();
        newTrack.endLat = endGeo.y();
    }

    _trackManager.addTrack(newTrack);
    qInfo() << "[航迹导入]" << "航迹对象创建完成，航迹ID：" << trackId;
    qInfo() << "[速度检查]"
            << QString("导入完成，速度检查结果：异常 %1 个，可疑 %2 个")
                   .arg(abnormalSpeedCount)
                   .arg(warningSpeedCount);
    qInfo() << "[多航迹管理]" << "新航迹导入完成，当前航迹数量：" << _trackManager.trackIds().size();

    // =========================
    // 绑定当前选中航迹的快捷指针
    // =========================
    _planePat = planePat;
    _pathGeom = pathGeom;
    _projLineGeode = projLineGeode;
    _projLineGeom = projLineGeom;
    _projLineVertices = projLineVertices;

    // =========================
    // 更新 UI 状态
    // =========================
    ui->listWidgetTracks->addItem(trackId);
    ui->listWidgetTracks->setCurrentRow(ui->listWidgetTracks->count() - 1);

    ui->btnResetView->setEnabled(true);
    ui->sliderProgress->setEnabled(true);
    ui->sliderProgress->setRange(0, static_cast<int>(newPlaybackController.totalDuration() * SystemConfig::Playback::SliderTimeScale));
    ui->sliderProgress->setValue(0);

    ui->labelState->setText(QString("状态：已加载轨迹（%1 个点）").arg(points.size()));
    refreshTrackInfoPanel();

    focusCurrentAircraft(true);
    newPlaybackController.setPaused(false);
    _isPaused = false;
    ui->btnPlayPause->setText("暂停播放");
    qInfo() << "[模型回放]" << "开始按时间插值回放";

    on_checkShowPoints_clicked(ui->checkShowPoints->isChecked());
    on_checkShowBasePlane_clicked(ui->checkShowBasePlane->isChecked());
}
void MainWindow::on_btnPlayPause_clicked()
{
    if (!_planePat.valid()) return;

    TrackPlaybackController playbackController;
    playbackController.bind(_planePat.get());
    if (playbackController.hasCallback()) {
        qInfo() << "[播放控制]" << (playbackController.isPaused() ? "用户点击播放按钮" : "用户点击暂停按钮");
        // 切换播放状态
        _isPaused = playbackController.togglePaused();

        // 更新按钮和状态文字
        if (_isPaused) {
            ui->btnPlayPause->setText("继续播放");
            ui->labelState->setText("状态：已暂停");
            qInfo() << "[播放控制]" << "当前状态：暂停";
        } else {
            ui->btnPlayPause->setText("暂停播放");
            ui->labelState->setText("状态：正在回放");
            qInfo() << "[播放控制]" << "当前状态：播放";
            qInfo() << "[模型回放]" << "开始按时间插值回放";
        }
    }
}


void MainWindow::on_comboBox_currentTextChanged(const QString &arg1)
{
    if (!_planePat.valid()) return;
    TrackPlaybackController playbackController;
    playbackController.bind(_planePat.get());
    if (playbackController.hasCallback()) {
        double s = QStringView(arg1).left(arg1.indexOf('x')).toDouble();
        if (s > 0) {
            playbackController.setTimeScale(s);
            qInfo() << "[播放控制]" << "回放倍率切换为：" << QString("%1x").arg(s, 0, 'f', 1);
            // 状态文字由 UI 定时器刷新
        }
    }
}


void MainWindow::on_btnPitchUp_clicked()
{
    if (!_osgWidget) return;

    osgEarth::Util::EarthManipulator* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(
        _osgWidget->getViewer()->getCameraManipulator());

    if (manip) {
        osgEarth::Viewpoint vp = manip->getViewpoint();

        // pitch() 为可选值，读取前需检查有效性
        double currentPitch = 0.0;
        if (vp.pitch().isSet()) {
            currentPitch = vp.pitch()->as(osgEarth::Units::DEGREES);
        }

        // 增大俯仰角
        vp.pitch() = osgEarth::Angle(currentPitch + 5.0, osgEarth::Units::DEGREES);

        manip->setViewpoint(vp, 0.3); // 调快一点，0.3 秒更灵敏
    }
}


void MainWindow::on_btnPitchDown_clicked()
{
    if (!_osgWidget) return;

    osgEarth::Util::EarthManipulator* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(
        _osgWidget->getViewer()->getCameraManipulator());

    if (manip) {
        osgEarth::Viewpoint vp = manip->getViewpoint();

        double currentPitch = 0.0;
        if (vp.pitch().isSet()) {
            currentPitch = vp.pitch()->as(osgEarth::Units::DEGREES);
        }

        double nextPitch = currentPitch - 5.0;
        if (nextPitch < -90.0) nextPitch = -90.0;

        vp.pitch() = osgEarth::Angle(nextPitch, osgEarth::Units::DEGREES);

        manip->setViewpoint(vp, 0.3);
    }
}


void MainWindow::on_checkShowProjection_clicked(bool checked)
{
    for (const QString& trackId : _trackManager.trackIds()) {
        TrackObject* track = _trackManager.track(trackId);
        if (track && track->projLineGeode.valid()) {
            track->projLineGeode->setNodeMask(checked ? 0xffffffff : 0x0);
        }
    }
    qInfo() << "[辅助可视化]" << "投影线显示开关：" << (checked ? "开启" : "关闭");
}

// 航迹列表切换
void MainWindow::on_listWidgetTracks_itemClicked(QListWidgetItem *item)
{
    if (!item) return;

    QString trackId = item->text();
    qInfo() << "[多航迹管理]" << "用户选择航迹：" << trackId;
    if (!_trackManager.selectTrack(trackId)) return;
    qInfo() << "[多航迹管理]" << "当前操作航迹已切换为：" << trackId;

    TrackObject* trackPtr = _trackManager.track(trackId);
    if (!trackPtr) return;
    TrackObject& track = *trackPtr;

    // 重新绑定当前航迹快捷指针
    _planePat = track.planePat;
    _projLineGeode = track.projLineGeode;
    _projLineGeom = track.projLineGeom;
    _projLineVertices = track.projLineVertices;
    _pathGeom = track.pathGeom;
    refreshTrackInfoPanel();

    if (_planePat.valid()) {
        TrackPlaybackController playbackController;
        playbackController.bind(_planePat.get());
        if (playbackController.hasCallback()) {
            ui->sliderProgress->blockSignals(true);
            ui->sliderProgress->setRange(0, static_cast<int>(playbackController.totalDuration() * SystemConfig::Playback::SliderTimeScale));
            ui->sliderProgress->setValue(static_cast<int>(playbackController.currentTime() * SystemConfig::Playback::SliderTimeScale));
            ui->sliderProgress->blockSignals(false);

            _isPaused = playbackController.isPaused();
            ui->btnPlayPause->setText(_isPaused ? "继续播放" : "暂停播放");
        }
    }

    focusCurrentAircraft(true);
    qInfo() << "[视角控制]" << "航迹切换完成，镜头定位到航迹：" << trackId;
    on_checkShowPoints_clicked(ui->checkShowPoints->isChecked());
    on_checkShowBasePlane_clicked(ui->checkShowBasePlane->isChecked());
}

// 删除航迹
void MainWindow::on_btnDeleteTrack_clicked()
{
    QListWidgetItem* item = ui->listWidgetTracks->currentItem();
    if (!item) {
        qWarning() << "[多航迹管理]" << "删除航迹失败：未选择航迹";
        QMessageBox::warning(this, "提示", "请先在列表中选择要删除的航迹！");
        return;
    }

    QString trackId = item->text();
    qInfo() << "[多航迹管理]" << "删除航迹：" << trackId;

    // 从 OSG 场景图中移除渲染节点
    osg::Group* root = _osgWidget->getViewer()->getSceneData()->asGroup();
    TrackObject* track = _trackManager.track(trackId);
    if (root && track) {
        if (track->planePat.valid()) root->removeChild(track->planePat);
        if (track->projLineGeode.valid()) root->removeChild(track->projLineGeode);
        if (track->pathGeode.valid()) root->removeChild(track->pathGeode);
        if (track->pointGeode.valid()) root->removeChild(track->pointGeode);
        if (track->basePlaneGeode.valid()) root->removeChild(track->basePlaneGeode);
    }

    // 从数据结构和 UI 列表中移除
    bool wasCurrentTrack = (_trackManager.currentTrackId() == trackId);
    _trackManager.removeTrack(trackId);
    delete item; // 释放 UI item 内存
    qInfo() << "[多航迹管理]" << "删除完成，剩余航迹数量：" << _trackManager.trackIds().size();

    // 删除当前航迹后的状态处理
    if (wasCurrentTrack) {
        if (_trackManager.isEmpty()) {
            // 清空当前航迹状态
            _planePat = nullptr;
            _pathGeom = nullptr;
            _projLineGeode = nullptr;
            _projLineGeom = nullptr;
            _projLineVertices = nullptr;

            ui->sliderProgress->setEnabled(false);
            ui->btnResetView->setEnabled(false);
            ui->labelState->setText("状态：等待导入数据");
        } else {
            // 选中 UI 列表中的第一条剩余航迹
            ui->listWidgetTracks->setCurrentRow(0);
            on_listWidgetTracks_itemClicked(ui->listWidgetTracks->currentItem());
        }
    }
}

// 航迹点显隐
void MainWindow::on_checkShowPoints_clicked(bool checked)
{
    // 先隐藏全部航迹点
    for (const QString& trackId : _trackManager.trackIds()) {
        TrackObject* track = _trackManager.track(trackId);
        if (track && track->pointGeode.valid()) {
            track->pointGeode->setNodeMask(0x0);
        }
    }

    // 显示当前航迹点
    TrackObject* currentTrack = _trackManager.currentTrack();
    if (checked && currentTrack && currentTrack->pointGeode.valid()) {
        currentTrack->pointGeode->setNodeMask(0xffffffff);
    }
    qInfo() << "[航迹显示]" << "航迹点显示开关：" << (checked ? "开启" : "关闭");
    qInfo() << "[辅助可视化]" << "航迹点显示开关：" << (checked ? "开启" : "关闭");
}

// 航迹参考面显隐
void MainWindow::on_checkShowBasePlane_clicked(bool checked)
{
    // 先隐藏全部参考面
    for (const QString& trackId : _trackManager.trackIds()) {
        TrackObject* track = _trackManager.track(trackId);
        if (track && track->basePlaneGeode.valid()) {
            track->basePlaneGeode->setNodeMask(0x0);
        }
    }

    // 显示当前航迹参考面
    TrackObject* currentTrack = _trackManager.currentTrack();
    if (checked && currentTrack && currentTrack->basePlaneGeode.valid()) {
        currentTrack->basePlaneGeode->setNodeMask(0xffffffff);
    }
    qInfo() << "[辅助可视化]" << "高度参考面显示开关：" << (checked ? "开启" : "关闭");
}

