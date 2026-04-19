#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "QOSGWidget.h"

// Qt 相关
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QTimer>

// OSG 核心基础
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/PolygonOffset>
#include <osg/LineStipple>

// osgEarth 核心
#include <osgEarth/MapNode>        // 地图核心节点
#include <osgEarth/EarthManipulator> // 漫游器
#include <osgEarth/GeoData>        // 坐标转换 (GeoPoint 等)
#include <osgEarth/Units>          // 单位转换

// 标准库
#include <vector>
#include <algorithm>

using namespace osgEarth;
using namespace osgEarth::Util;

// ==============================================
// 配置项结构体
// ==============================================
struct FlightConfig {
    double speed = 0.002;
    bool showPlane = true;
    bool showPath = true;
    float planeScale = 50.0f;
    osg::Vec4 pathColor = osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
    float lineWidth = 4.0f;
};

// ==============================================
// 飞行动画回调类
// ==============================================
class MoveCallback : public osg::NodeCallback
{
public:
    std::vector<osg::Vec3d> path;
    int index = 0;
    double speed;
    double t = 0.0;
    bool isPaused = false; // ✅ 新增：用于任务复盘的暂停控制

    MoveCallback(const std::vector<osg::Vec3d>& pts, double s) : path(pts), speed(s) {}

    // ✅ 新增：供滑动条调用的跳转函数
    void setProgress(int idx, double ratio) {
        if (idx >= 0 && idx < (int)path.size() - 1) {
            index = idx;
            t = ratio;
        }
    }

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        osg::PositionAttitudeTransform* pat = dynamic_cast<osg::PositionAttitudeTransform*>(node);
        // ✅ 增加 isPaused 判断
        if (!isPaused && pat && path.size() >= 2)
        {
            osg::Vec3d p1 = path[index];
            osg::Vec3d p2 = path[(index + 1) % path.size()];
            osg::Vec3d pos = p1 + (p2 - p1) * t;
            pat->setPosition(pos);

            osg::Vec3d dir = p2 - p1;
            dir.normalize();
            osg::Quat q;
            q.makeRotate(osg::Vec3d(0, -1, 0), dir);
            pat->setAttitude(q);

            t += speed;
            if (t >= 1.0) { t = 0.0; index = (index + 1) % path.size(); }
        }
        traverse(node, nv);
    }
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
                // 动画保护：复位动画执行期间，回调不干预
                if (_mainWin->_resetTimer.isValid() && _mainWin->_resetTimer.elapsed() < 2200) {
                    this->traverse(node, nv);
                    return;
                }

                osg::Vec3d worldPos = _mainWin->_planePat->getPosition();
                osgEarth::GeoPoint geoPos;
                if (geoPos.fromWorld(_mainWin->_mapNode->getMapSRS(), worldPos) && geoPos.isValid()) {
                    osgEarth::Viewpoint vp = manip->getViewpoint();
                    vp.focalPoint() = geoPos; // 🎯 只更新中心点
                    // ❌ 删掉 vp.range() = ... 这一行，不再强制锁定距离
                    manip->setViewpoint(vp);
                }
            }
        }
        this->traverse(node, nv);
    }
};
void MainWindow::initProjectionLine(osg::Group* root) {
    if (!root) return;

    _projLineGeom = new osg::Geometry();
    _projLineVertices = new osg::Vec3Array(2);
    (*_projLineVertices)[0] = osg::Vec3(0,0,0);
    (*_projLineVertices)[1] = osg::Vec3(0,0,0);

    _projLineGeom->setVertexArray(_projLineVertices);

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
    colors->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 0.8f)); // 换成黄色，地面更醒目
    _projLineGeom->setColorArray(colors, osg::Array::BIND_OVERALL);

    _projLineGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(_projLineGeom);

    osg::StateSet* ss = geode->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);

    // 开启虚线模式
    ss->setMode(GL_LINE_STIPPLE, osg::StateAttribute::ON);
    // osg::LineStipple* stipple = new osg::LineStipple(2, 0xAAAA); // 2倍宽度，0xAAAA是均匀虚线
    // ss->setAttributeAndModes(stipple, osg::StateAttribute::ON);
    ss->setAttributeAndModes(new osg::LineWidth(5.0f), osg::StateAttribute::ON); //实线

    // 解决 Z-fighting：让线“浮”在地图表面，防止闪烁
    ss->setAttributeAndModes(new osg::PolygonOffset(-1.0f, -1.0f), osg::StateAttribute::ON);
    ss->setRenderBinDetails(11, "RenderBin"); // 确保在地图(Bin 0)和航迹(Bin 1)之后渲染

    root->addChild(geode);
}
class TimeBasedMoveCallback : public osg::NodeCallback {
public:
    std::vector<TrackPoint> _track;
    double _currentTime = 0.0;
    double _totalDuration = 0.0;
    double _timeScale = 1.0;
    double _lastFrameTime = -1.0;
    bool isPaused = true;

    // --- ✅ 新增：用于操作投影线的成员变量 ---
    osg::observer_ptr<osg::Vec3Array> _lineVerts; // 投影线的顶点数组
    osg::observer_ptr<osg::Geometry> _lineGeom;   // 投影线的几何体
    osg::observer_ptr<osgEarth::MapNode> _mapNode;                  // 用于地形高度采样

    // ✅ 修改构造函数：接收投影线相关的指针
    TimeBasedMoveCallback(const std::vector<TrackPoint>& data,
                          osg::Vec3Array* v,
                          osg::Geometry* g,
                          osgEarth::MapNode* mn)
        : _track(data), _lineVerts(v), _lineGeom(g), _mapNode(mn)
    {
        if (!_track.empty()) _totalDuration = _track.back().timeOffset;
    }

    void setTime(double t) {
        _currentTime = std::max(0.0, std::min(t, _totalDuration));
        _lastFrameTime = -1.0;
    }

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) {
        osg::PositionAttitudeTransform* pat = dynamic_cast<osg::PositionAttitudeTransform*>(node);

        if (pat && _track.size() >= 2) {
            // ... (这里保留你原来的 dt 计算逻辑) ...
            double now = nv->getFrameStamp()->getSimulationTime();
            double dt = (_lastFrameTime > 0) ? (now - _lastFrameTime) : 0.0;
            _lastFrameTime = now;

            if (!isPaused) {
                if (dt > 0.2) dt = 0.0166;
                _currentTime += dt * _timeScale;
                if (_currentTime > _totalDuration) _currentTime = 0.0;
            }

            // --- 1. 更新飞机位置 ---
            auto it = std::lower_bound(_track.begin(), _track.end(), _currentTime,
                                       [](const TrackPoint& a, double t) { return a.timeOffset < t; });
            int idx = std::distance(_track.begin(), it);

            if (idx > 0 && idx < _track.size()) {
                const TrackPoint& p0 = _track[idx - 1];
                const TrackPoint& p1 = _track[idx];
                double t_ratio = (_currentTime - p0.timeOffset) / (p1.timeOffset - p0.timeOffset);

                osg::Vec3d currentPos = p0.worldPos + (p1.worldPos - p0.worldPos) * t_ratio;
                pat->setPosition(currentPos);
                osg::Quat rot;
                rot.slerp(t_ratio, p0.rotation, p1.rotation);
                pat->setAttitude(rot);

                // --- 2. ✅ 新增：动态更新投影线（严谨处理地形） ---
                if (_lineVerts.valid() && _lineGeom.valid() && _mapNode.valid()) {
                    // 获取飞机当前的经纬度
                    osgEarth::GeoPoint geoPos;
                    geoPos.fromWorld(_mapNode->getMapSRS(), currentPos);

                    // 核心：采样当前经纬度下的地形高度
                    double terrainH = 0.0;
                    _mapNode->getTerrain()->getHeight(geoPos.getSRS(), geoPos.x(), geoPos.y(), &terrainH);

                    // 构造地面接触点（高度设为采样的地形高度）
                    osgEarth::GeoPoint groundGeo(geoPos.getSRS(), geoPos.x(), geoPos.y(), terrainH);
                    osg::Vec3d groundWorldPos;
                    groundGeo.toWorld(groundWorldPos);

                    // 更新顶点数组：[0]是飞机，[1]是地面
                    (*_lineVerts)[0] = currentPos;
                    (*_lineVerts)[1] = groundWorldPos;

                    // 标记数据已更新，强制重绘
                    _lineVerts->dirty();
                    _lineGeom->dirtyBound();
                }
            }
        }
        traverse(node, nv);
    }
};
// ==============================================
// MainWindow 实现
// ==============================================
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 1. 初始化 osgEarth 资源
    osgEarth::initialize();

    // 2. 创建并嵌入 QOSGWidget
    QOSGWidget* osgWidget = new QOSGWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(ui->renderBase);
    layout->addWidget(osgWidget);
    layout->setContentsMargins(0, 0, 0, 0);
    this->_osgWidget = osgWidget;

    // 3. 配置参数
    FlightConfig config;
    config.speed = 0.002;
    config.showPath = true;
    config.showPlane = true;

    // ==============================================
    // 4. 构建地图节点 (已修正：加入缓存配置并解决重定义)
    // ==============================================
    // 核心：直接读取 .earth 配置文件
    osg::ref_ptr<osg::Node> loadedNode = osgDB::readNodeFile("mymap.earth");

    if (!loadedNode.valid()) {
        QMessageBox::critical(this, "错误", "无法加载 mymap.earth 文件！请确保它在运行目录下。");
        return;
    }

    // 找到 MapNode (这是 osgEarth 操作的核心入口)
    this->_mapNode = osgEarth::MapNode::findMapNode(loadedNode.get());

    if (!this->_mapNode.valid()) {
        QMessageBox::critical(this, "错误", "文件加载成功但未发现有效的 MapNode。");
        return;
    }
    // ==============================================
    // 5. 创建场景根节点
    // ==============================================
    osg::ref_ptr<osg::Group> root = new osg::Group();
    root->addChild(this->_mapNode.get());
    // 6. 绘制航线 (初始化空航线)
    if (config.showPath) {
        _pathGeom = new osg::Geometry(); // ✅ 赋值给成员变量

        // 初始化一个空的顶点数组
        osg::ref_ptr<osg::Vec3Array> emptyVa = new osg::Vec3Array();
        _pathGeom->setVertexArray(emptyVa);

        // 初始时不添加任何绘制指令 (DrawArrays)，因为现在没有点

        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
        colors->push_back(config.pathColor);
        _pathGeom->setColorArray(colors, osg::Array::BIND_OVERALL);

        osg::ref_ptr<osg::Geode> lineGeode = new osg::Geode();
        lineGeode->addDrawable(_pathGeom.get());

        osg::StateSet* ss = lineGeode->getOrCreateStateSet();
        // 开启深度偏移，防止线嵌入地图
        ss->setAttributeAndModes(new osg::PolygonOffset(1.0f, 1.0f), osg::StateAttribute::ON);
        // 关闭光照
        ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
        // 设置线宽
        ss->setAttributeAndModes(new osg::LineWidth(config.lineWidth), osg::StateAttribute::ON);

        osgEarth::Registry::shaderGenerator().run(lineGeode);
        root->addChild(lineGeode);

        // 设置渲染队列，确保航迹在基础地图之上绘制
        ss->setRenderBinDetails(1, "RenderBin");
    }
    osg::ref_ptr<osg::PositionAttitudeTransform> pat;
    _planePat = new osg::PositionAttitudeTransform();   //将其赋值给_planePat，方便点击事件控制
    // 7. 绘制飞机 (初始化：加载模型但不设置初始位置和回调)
    if (config.showPlane) {
        // 初始状态下隐藏飞机，直到 CSV 导入数据
        _planePat->setNodeMask(0x0);

        osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("D:/OSG/QT-osgearth/osgearth/data/cessna.osgb");
        if (model.valid()) {
            osg::StateSet* ss = model->getOrCreateStateSet();

            // 修改材质：添加自发光 (Emission)
            osg::ref_ptr<osg::Material> redMat = new osg::Material;
            redMat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 0, 0, 1));
            redMat->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0.5, 0, 0, 1));
            redMat->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0.8, 0, 0, 1));

            ss->setAttributeAndModes(redMat.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);

            // 关闭光照，确保红色纯正
            ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);

            // 确保渲染顺序在航迹线之后 (Bin 10)
            ss->setRenderBinDetails(10, "RenderBin");

            // 缩放控制
            osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform();
            mt->setMatrix(osg::Matrix::scale(config.planeScale, config.planeScale, config.planeScale));
            mt->addChild(model);

            _planePat->addChild(mt);

            // 现在的逻辑是：导入 CSV 时才 setUpdateCallback 为 TimeBasedMoveCallback
            root->addChild(_planePat);
        }
    }
    // 8. [重要] 初始禁用相关按钮
    ui->btnResetView->setEnabled(false);
    ui->sliderProgress->setEnabled(false);
    ui->labelState->setText("状态: 等待导入数据");
    // 9. 将场景设置到 Viewer 中
    osgWidget->getViewer()->setSceneData(root);

    // 10. 设置漫游器并开启跟随
    osgEarth::Util::EarthManipulator* manip = new osgEarth::Util::EarthManipulator();
    osgWidget->getViewer()->setCameraManipulator(manip);

    // 初始化进度条范围
    ui->sliderProgress->setRange(0, 1000); // 对应你 coords 数组的索引 0, 1, 2

    // 创建定时器刷新 UI 仪表盘
    QTimer* uiTimer = new QTimer(this);
    connect(uiTimer, &QTimer::timeout, this, [this]() {
        // 1. 基础指针检查
        if (!_osgWidget || !_planePat.valid() || !_mapNode.valid()) return;
        osgViewer::Viewer* viewer = _osgWidget->getViewer();
        osgEarth::Util::EarthManipulator* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(
            viewer->getCameraManipulator());

        if (manip) {
            // --- ✅ 核心：用户交互检测 ---
            const osgGA::GUIEventAdapter* ea = viewer->getEventQueue()->getCurrentEventState();
            double currentRange = 0.0;
            if (manip->getViewpoint().range().isSet()) {
                // 强制转换为双精度浮点数
                currentRange = manip->getViewpoint().range().value().as(osgEarth::Units::METERS);
            }

            if (_isFollowing) {
                // 1. 检测鼠标按键：如果用户按下左键/右键（尝试拖拽或旋转）
                bool mousePressed = (ea && ea->getButtonMask() != 0);

                // 2. 检测滚轮缩放：如果当前距离和上次记录的距离差异较大
                bool rangeChanged = (_lastRange > 0 && std::abs(currentRange - _lastRange) > 1.0);

                if (mousePressed || rangeChanged) {
                    _isFollowing = false; // 🛑 立即断开跟随
                    ui->labelState->setText("状态: 手动模式");
                }
            }
            // 更新距离记录，供下一帧对比
            _lastRange = currentRange;
        }
        // 2. 获取飞机当前位置
        osg::Vec3d worldPos = _planePat->getPosition();
        osgEarth::GeoPoint geoPos;
        if (!geoPos.fromWorld(_mapNode->getMapSRS(), worldPos)) return;

        // --- UI 仪表盘同步 (始终运行) ---
        ui->labelPos->setText(QString("位置: %1, %2")
                                  .arg(geoPos.x(), 0, 'f', 4)
                                  .arg(geoPos.y(), 0, 'f', 4));
        ui->labelAlt->setText(QString("%1 m").arg(geoPos.z(), 0, 'f', 1));

        // 进度条逻辑同步
        TimeBasedMoveCallback* tbc = dynamic_cast<TimeBasedMoveCallback*>(_planePat->getUpdateCallback());
        if (tbc) {
            ui->sliderProgress->blockSignals(true);
            ui->sliderProgress->setValue(static_cast<int>(tbc->_currentTime * 10));
            ui->sliderProgress->blockSignals(false);

            int curSec = (int)tbc->_currentTime;
            int totSec = (int)tbc->_totalDuration;

            // ✅ 改进：获取当前的倍速，并一起显示出来
            double currentSpeed = tbc->_timeScale;
            QString speedStr = (currentSpeed == 1.0) ? "" : QString(" [%1x]").arg(currentSpeed);

            // 状态判断
            QString stateStr = tbc->isPaused ? "已暂停" : "正在回放";

            // 综合显示：进度: 10s / 60s | 正在回放 [2.0x]
            ui->labelState->setText(QString("进度: %1s / %2s | %3%4")
                                        .arg(curSec)
                                        .arg(totSec)
                                        .arg(stateStr)
                                        .arg(speedStr));
        }


    });
    uiTimer->start(100); // 100毫秒刷新一次
    // 利用 MapNode 的更新回调实现丝滑跟随（每帧执行）
    _mapNode->addUpdateCallback(new CameraFollowCallback(this));


}

MainWindow::~MainWindow()
{
    delete ui;
}
// 辅助函数：计算两个地理点之间的旋转姿态
osg::Quat computeRotation(const osg::Vec3d& current, const osg::Vec3d& next) {
    osg::Vec3d dir = next - current;
    dir.normalize();
    osg::Quat q;
    // 假设 Cessna 模型原始朝向是负 Y 轴 (0,-1,0)
    q.makeRotate(osg::Vec3d(0, -1, 0), dir);
    return q;
}


//后续的为UI点击事件等交互逻辑函数

// 控制无人机飞行进度的点击函数

void MainWindow::on_sliderProgress_valueChanged(int value)
{
    if (_planePat.valid()) {
        auto* cb = dynamic_cast<TimeBasedMoveCallback*>(_planePat->getUpdateCallback());
        if (cb) {
            cb->setTime(value / 10.0); // 将 slider 的整数值转回秒
        }
    }
}

// 复位视角的点击函数
void MainWindow::on_btnResetView_clicked()
{
    if (_osgWidget && _planePat.valid() && _mapNode.valid()) {
        osgEarth::Util::EarthManipulator* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(
            _osgWidget->getViewer()->getCameraManipulator());

        if (manip) {
            osg::Vec3d worldPos = _planePat->getPosition();
            osgEarth::GeoPoint geoPos;
            geoPos.fromWorld(_mapNode->getMapSRS(), worldPos);

            // 1. 设置复位视角：高度 100000
            osgEarth::Viewpoint vp("Follow", geoPos.x(), geoPos.y(), geoPos.z(), 0, -35, 100000);

            _isFollowing = false;
            _resetTimer.start();
            manip->setViewpoint(vp, 2.0); // 2秒转场动画

            QTimer::singleShot(2100, [this, currentRange = 100000.0](){
                this->_lastRange = currentRange; // ✅ 初始化距离记录
                this->_isFollowing = true;       // ✅ 开启自动跟随
                ui->labelState->setText("状态: 自动跟随");
            });
        }
    }
}

// 导入文件的点击函数
void MainWindow::on_btnImportData_clicked()
{
    QString path = QFileDialog::getOpenFileName(this, "导入航迹", "", "CSV文件 (*.csv)");
    if (path.isEmpty()) return;

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(this, "错误", "无法打开所选文件！");
        return;
    }

    std::vector<TrackPoint> points;
    osg::ref_ptr<osg::Vec3Array> lineCoords = new osg::Vec3Array();

    QTextStream in(&file);
    bool isFirstLine = true;

    while (!in.atEnd()) {
        QString line = in.readLine();
        // 健壮性：跳过空行
        if (line.trimmed().isEmpty()) continue;
        if (isFirstLine) { isFirstLine = false; continue; }

        QStringList cols = line.split(',');
        if (cols.size() < 4) continue;

        TrackPoint tp;
        tp.timeOffset = cols[0].toDouble();
        double lon = cols[1].toDouble();
        double lat = cols[2].toDouble();
        double alt = cols[3].toDouble();

        // 经纬度转世界坐标
        osgEarth::GeoPoint(this->_mapNode->getMapSRS(), lon, lat, alt).toWorld(tp.worldPos);
        points.push_back(tp);
        lineCoords->push_back(tp.worldPos);
    }

    // 严谨性校验：如果文件是空的或非法格式
    if (points.empty()) {
        QMessageBox::critical(this, "导入失败", "航迹数据为空，请检查CSV格式是否为：时间,经,纬,高");
        return;
    }

    // 预计算姿态
    for (size_t i = 0; i < points.size(); ++i) {
        if (i < points.size() - 1)
            points[i].rotation = computeRotation(points[i].worldPos, points[i+1].worldPos);
        else
            points[i].rotation = (i > 0) ? points[i-1].rotation : osg::Quat();
    }

    // --- 核心：更新场景逻辑 ---
    auto* newCb = new TimeBasedMoveCallback(points,
                                            _projLineVertices.get(),
                                            _projLineGeom.get(),
                                            _mapNode.get());
    QString curTxt = ui->comboBox->currentText();
    double curSpeed = curTxt.left(curTxt.indexOf('x')).toDouble();
    newCb->_timeScale = (curSpeed > 0) ? curSpeed : 1.0;
    newCb->isPaused = false;
    _planePat->setUpdateCallback(newCb);

    // 更新渲染线
    updatePathRender(lineCoords);

    // --- ✨ 严谨性增强：激活 UI 状态 ---
    _planePat->setNodeMask(0xffffffff); // 让原本隐藏的飞机显影

    ui->btnResetView->setEnabled(true);
    ui->sliderProgress->setEnabled(true);

    // 设置进度条范围 (0.1秒精度)
    ui->sliderProgress->setRange(0, static_cast<int>(newCb->_totalDuration * 10));
    ui->sliderProgress->setValue(0); // 重置进度条到起点

    ui->labelState->setText(QString("状态: 已加载轨迹 (%1个点)").arg(points.size()));

    // --- 视角定位逻辑 ---
    osgEarth::Util::EarthManipulator* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(
        _osgWidget->getViewer()->getCameraManipulator());
    if (manip) {
        osgEarth::GeoPoint firstPt;
        if (firstPt.fromWorld(_mapNode->getMapSRS(), points[0].worldPos)) {
            // 定位到第一个点上方，给一个稍微倾斜的视角（-45度），这样看起来更立体
            manip->setViewpoint(osgEarth::Viewpoint("Import", firstPt.x(), firstPt.y(), 0, 0, -45, 20000), 2.0);
        }
    }

}
//动态刷新航迹
void MainWindow::updatePathRender(osg::Vec3Array* va) {
    if (_pathGeom.valid()) {
        _pathGeom->setVertexArray(va);
        // 清除旧的指令并添加新的
        _pathGeom->removePrimitiveSet(0, _pathGeom->getNumPrimitiveSets());
        _pathGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, va->size()));

        _pathGeom->dirtyDisplayList(); // 刷新显示列表
        _pathGeom->dirtyBound();       // 刷新包围盒
    }
}

void MainWindow::on_btnPlayPause_clicked()
{
    if (!_planePat.valid()) return;

    auto* cb = dynamic_cast<TimeBasedMoveCallback*>(_planePat->getUpdateCallback());
    if (cb) {
        // 1. 切换逻辑状态
        cb->isPaused = !cb->isPaused;
        _isPaused = cb->isPaused;

        // 2. 更新 UI 文字
        if (_isPaused) {
            ui->btnPlayPause->setText("继续播放");
            ui->labelState->setText("状态: 已暂停");
        } else {
            ui->btnPlayPause->setText("暂停播放");
            ui->labelState->setText("状态: 正在回放");
        }
    }
}


void MainWindow::on_comboBox_currentTextChanged(const QString &arg1)
{
    if (!_planePat.valid()) return;
    auto* cb = dynamic_cast<TimeBasedMoveCallback*>(_planePat->getUpdateCallback());
    if (cb) {
        double s = QStringView(arg1).left(arg1.indexOf('x')).toDouble();
        if (s > 0) {
            cb->_timeScale = s;
            // 这里不需要再 setText 了，uiTimer 下一秒就会自动刷新它
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

        // 因为 pitch() 返回的是 optional，所以用 -> 来访问内部的 Angle 对象
        // 同时要检查它是否有值 (isSet)
        double currentPitch = 0.0;
        if (vp.pitch().isSet()) {
            currentPitch = vp.pitch()->as(osgEarth::Units::DEGREES);
        }

        // 抬头：角度增加
        vp.pitch() = osgEarth::Angle(currentPitch + 5.0, osgEarth::Units::DEGREES);

        manip->setViewpoint(vp, 0.3); // 调快一点，0.3秒更灵敏
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

