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
#include <osgEarth/XYZ>
#include <osgEarth/URI>
#include <osgEarth/Registry>

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
    float planeScale = 2.0f;
    osg::Vec4 pathColor = osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
    float lineWidth = 4.0f;
};
// 天地图加载对应函数
osgEarth::XYZImageLayer* createTianDiTuLayer(
    const std::string& key,
    const std::string& type,
    const std::string& name)
{
    osgEarth::URIContext context;

    context.addHeader("User-Agent",
                      "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
                      "AppleWebKit/537.36 Chrome/99.0 Safari/537.36");

    context.addHeader("Referer", "http://localhost/");
    context.addHeader("Accept", "image/avif,image/webp,image/apng,image/*,*/*;q=0.8");
    context.addHeader("Accept-Language", "zh-CN,zh;q=0.9");

    // 先不要用 t[01234567]，有些版本 osgEarth 不展开这个写法
    std::string url =
        "http://t0.tianditu.gov.cn/DataServer?"
        "T=" + type +
        "&x={x}&y={y}&l={z}&tk=" + key;

    osgEarth::URI uri(url, context);

    osgEarth::XYZImageLayer* layer = new osgEarth::XYZImageLayer();
    layer->setURL(uri);
    layer->setProfile(osgEarth::Profile::create("spherical-mercator"));
    layer->setName(name);
    layer->setOpacity(1.0f);

    return layer;
}

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

    // ✅ 修正这里：使用下标访问，这是最通用的写法
    (*_projLineVertices)[0] = osg::Vec3(0,0,0);
    (*_projLineVertices)[1] = osg::Vec3(0,0,0);

    _projLineGeom->setUseDisplayList(false);
    _projLineGeom->setUseVertexBufferObjects(true);
    _projLineGeom->setVertexArray(_projLineVertices.get());

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
    colors->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
    _projLineGeom->setColorArray(colors.get(), osg::Array::BIND_OVERALL);
    _projLineGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));

    // ✅ 关键：一定要把这个 Geode 加入场景，并且只用这一个 Geode
    _projLineGeode = new osg::Geode();
    _projLineGeode->addDrawable(_projLineGeom.get());
    _projLineGeode->setName("ProjectionLineNode");

    osg::StateSet* ss = _projLineGeode->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    ss->setAttributeAndModes(new osg::LineWidth(3.0f), osg::StateAttribute::ON);
    ss->setRenderBinDetails(100, "RenderBin");
    ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

    // ✅ 确保 root 添加的是成员变量 _projLineGeode
    root->addChild(_projLineGeode.get());

    if (ui->checkShowProjection) {
        bool isDefaultChecked = ui->checkShowProjection->isChecked();
        _projLineGeode->setNodeMask(isDefaultChecked ? 0xffffffff : 0x0);
    }
}
class TimeBasedMoveCallback : public osg::NodeCallback {
public:
    std::vector<TrackPoint> _track;
    double _currentTime = 0.0;
    double _totalDuration = 0.0;
    double _timeScale = 1.0;
    double _lastFrameTime = -1.0;
    bool isPaused = true;
    double _currentVelocity = 0.0;

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
                double segDt = p1.timeOffset - p0.timeOffset;
                if (segDt <= 0.0) segDt = 1.0;

                double t_ratio = (_currentTime - p0.timeOffset) / segDt;

                osg::Vec3d currentPos = p0.worldPos + (p1.worldPos - p0.worldPos) * t_ratio;
                pat->setPosition(currentPos);

                osg::Quat rot;
                rot.slerp(t_ratio, p0.rotation, p1.rotation);
                pat->setAttitude(rot);

                // ✅ 当前速度插值
                _currentVelocity = p0.velocity + (p1.velocity - p0.velocity) * t_ratio;

                // --- 2. ✅ 新增：动态更新投影线（严谨处理地形） ---
                if (_lineVerts.valid() && _lineGeom.valid() && _mapNode.valid()) {
                    // 获取飞机当前的经纬度
                    osgEarth::GeoPoint geoPos;
                    geoPos.fromWorld(_mapNode->getMapSRS(), currentPos);

                    // 核心：采样当前经纬度下的地形高度
                    double terrainH = 0.0;
                    // _mapNode->getTerrain()->getHeight(geoPos.getSRS(), geoPos.x(), geoPos.y(), &terrainH);  //（111）

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
    // 加载天地图内容
    // ==============================================
    osgEarth::Map* map = this->_mapNode->getMap();

    std::string key = "80f4e2cd107652a761bb8683f5c8f93e";

    osgEarth::XYZImageLayer* imgLayer =
        createTianDiTuLayer(key, "img_w", "TianDiTu_Image");

    osgEarth::XYZImageLayer* labelLayer =
        createTianDiTuLayer(key, "cia_w", "TianDiTu_Label");

    map->addLayer(imgLayer);
    map->addLayer(labelLayer);

    // ==============================================
    // 5. 创建场景根节点
    // ==============================================
    osg::ref_ptr<osg::Group> root = new osg::Group();
    root->addChild(this->_mapNode.get());
    this->initProjectionLine(root.get());   // 初始化垂直投影线
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
        // osgDB::Registry::instance()->getDataFilePathList().push_back(
        //     "D:/OSG_QT_Project/models/uav/source/"
        //     );
        // osgDB::Registry::instance()->getDataFilePathList().push_back(
        //     "D:/OSG_QT_Project/models/uav/textures/"
        //     );

        // osg::ref_ptr<osg::Node> model =
        //     osgDB::readNodeFile("D:/OSG_QT_Project/models/uav/source/uAV.obj");
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
    if (ui->labelTrackInfo) ui->labelTrackInfo->setText("当前航迹信息：暂无");
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
        refreshTrackInfoPanel();

    });
    uiTimer->start(100); // 100毫秒刷新一次
    // 利用 MapNode 的更新回调实现丝滑跟随（每帧执行）
    _mapNode->addUpdateCallback(new CameraFollowCallback(this));


}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::refreshTrackInfoPanel()
{
    if (!ui->labelTrackInfo) return;

    if (_currentTrackId.isEmpty() || !_allTracks.contains(_currentTrackId)) {
        ui->labelTrackInfo->setText("当前航迹信息：暂无");
        return;
    }

    const TrackObject& track = _allTracks[_currentTrackId];

    double currentSpeed = 0.0;
    if (track.planePat.valid()) {
        auto* cb = dynamic_cast<TimeBasedMoveCallback*>(track.planePat->getUpdateCallback());
        if (cb) {
            currentSpeed = cb->_currentVelocity;
        }
    }

    ui->labelTrackInfo->setText(
        QString("航迹ID：%1\n"
                "点数：%2\n"
                "真实时长：%3 秒\n"
                "当前速度：%4 m/s\n"
                "起点：(%5, %6)\n"
                "终点：(%7, %8)")
            .arg(track.id)
            .arg(track.pointCount)
            .arg(track.totalDuration, 0, 'f', 1)
            .arg(currentSpeed, 0, 'f', 1)
            .arg(track.startLon, 0, 'f', 4)
            .arg(track.startLat, 0, 'f', 4)
            .arg(track.endLon, 0, 'f', 4)
            .arg(track.endLat, 0, 'f', 4)
        );
}

// 辅助函数：计算两个地理点之间的旋转姿态
// osg::Quat computeRotation(const osg::Vec3d& current, const osg::Vec3d& next) {
//     osg::Vec3d dir = next - current;
//     dir.normalize();
//     osg::Quat q;
//     // 假设 Cessna 模型原始朝向是负 Y 轴 (0,-1,0)
//     q.makeRotate(osg::Vec3d(0, -1, 0), dir);
//     return q;
// }

osg::Quat computeRotationFromHeading(double headingDeg)
{
    double rad = osg::DegreesToRadians(headingDeg);

    osg::Vec3d dir(std::sin(rad), std::cos(rad), 0.0);
    dir.normalize();

    osg::Quat q;
    q.makeRotate(osg::Vec3d(0, -1, 0), dir); // 保持和你模型原始朝向一致
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
            double followRange = 3000.0;

            osgEarth::Viewpoint vp("Follow", geoPos.x(), geoPos.y(), geoPos.z(), 0, -45, followRange);

            _isFollowing = false;
            _resetTimer.start();
            manip->setViewpoint(vp, 2.0);

            QTimer::singleShot(2100, [this, followRange]() {
                this->_lastRange = followRange;
                this->_isFollowing = true;
                ui->labelState->setText("状态: 自动跟随");
            });
        }
    }
}

// 导入文件的点击函数
void MainWindow::on_btnImportData_clicked()
{
    QDateTime firstTime;
    bool firstPoint = true;
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
    QString csvTrackId;
    while (!in.atEnd()) {
        QString line = in.readLine();
        if (line.trimmed().isEmpty()) continue;
        if (isFirstLine) { isFirstLine = false; continue; }

        QStringList cols = line.split(',');
        if (cols.size() < 7) continue;

        TrackPoint tp;

        QString fileTrackId = cols[0].trimmed();
        QString timeStr = cols[1].trimmed();
        if (csvTrackId.isEmpty()) {
            csvTrackId = fileTrackId;
        }
        QDateTime currentTime = QDateTime::fromString(timeStr, Qt::ISODate);
        if (!currentTime.isValid()) continue;

        if (firstPoint) {
            firstTime = currentTime;
            firstPoint = false;
        }

        tp.timeOffset = firstTime.secsTo(currentTime);

        tp.lon = cols[2].toDouble();
        tp.lat = cols[3].toDouble();
        tp.alt = cols[4].toDouble();
        tp.velocity = cols[5].toDouble();      // speed_mps
        tp.headingDeg = cols[6].toDouble();    // heading_deg
        double baseGroundHeight = 500.0;
        osgEarth::GeoPoint(this->_mapNode->getMapSRS(), tp.lon, tp.lat, tp.alt + baseGroundHeight).toWorld(tp.worldPos);
        tp.rotation = computeRotationFromHeading(tp.headingDeg);

        points.push_back(tp);
        lineCoords->push_back(tp.worldPos);
    }

    if (points.empty()) {
        QMessageBox::critical(this, "导入失败", "航迹数据为空，请检查CSV格式是否为：track_id,timestamp,longitude,latitude,altitude_m,speed_mps,heading_deg");
        return;
    }


    // 1. 自动生成唯一 trackId
    QString trackId = csvTrackId.isEmpty() ? QFileInfo(path).fileName() : csvTrackId;
    int counter = 1;
    while (_allTracks.contains(trackId)) {
        trackId = QFileInfo(path).fileName() + QString("_%1").arg(counter++);
    }

    osg::Group* root = _osgWidget->getViewer()->getSceneData()->asGroup();
    if (!root) return;

    FlightConfig config;

    // =========================
    // A. 创建当前航迹自己的航迹线
    // =========================
    osg::ref_ptr<osg::Geometry> pathGeom = new osg::Geometry();
    pathGeom->setVertexArray(lineCoords.get());

    osg::ref_ptr<osg::Vec4Array> pathColors = new osg::Vec4Array();
    pathColors->push_back(config.pathColor);
    pathGeom->setColorArray(pathColors.get(), osg::Array::BIND_OVERALL);
    pathGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, lineCoords->size()));

    osg::ref_ptr<osg::Geode> pathGeode = new osg::Geode();
    pathGeode->addDrawable(pathGeom.get());

    osg::StateSet* pathSS = pathGeode->getOrCreateStateSet();
    pathSS->setAttributeAndModes(new osg::PolygonOffset(1.0f, 1.0f), osg::StateAttribute::ON);
    pathSS->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    pathSS->setAttributeAndModes(new osg::LineWidth(config.lineWidth), osg::StateAttribute::ON);
    pathSS->setRenderBinDetails(1, "RenderBin");

    osgEarth::Registry::shaderGenerator().run(pathGeode.get());
    root->addChild(pathGeode.get());

    // =========================
    // B. 创建当前航迹自己的投影线
    // =========================
    osg::ref_ptr<osg::Geometry> projLineGeom = new osg::Geometry();
    osg::ref_ptr<osg::Vec3Array> projLineVertices = new osg::Vec3Array(2);
    (*projLineVertices)[0] = points.front().worldPos;
    (*projLineVertices)[1] = points.front().worldPos;

    projLineGeom->setUseDisplayList(false);
    projLineGeom->setUseVertexBufferObjects(true);
    projLineGeom->setVertexArray(projLineVertices.get());

    osg::ref_ptr<osg::Vec4Array> projColors = new osg::Vec4Array();
    projColors->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
    projLineGeom->setColorArray(projColors.get(), osg::Array::BIND_OVERALL);
    projLineGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));

    osg::ref_ptr<osg::Geode> projLineGeode = new osg::Geode();
    projLineGeode->addDrawable(projLineGeom.get());
    projLineGeode->setName("ProjectionLineNode_" + trackId.toStdString());

    osg::StateSet* projSS = projLineGeode->getOrCreateStateSet();
    projSS->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    projSS->setAttributeAndModes(new osg::LineWidth(3.0f), osg::StateAttribute::ON);
    projSS->setRenderBinDetails(100, "RenderBin");
    projSS->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

    projLineGeode->setNodeMask(ui->checkShowProjection->isChecked() ? 0xffffffff : 0x0);
    root->addChild(projLineGeode.get());

    // =========================
    // C. 创建当前航迹自己的飞机
    // =========================
    osg::ref_ptr<osg::PositionAttitudeTransform> planePat = new osg::PositionAttitudeTransform();
    planePat->setNodeMask(0xffffffff);

    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("D:/OSG/QT-osgearth/osgearth/data/cessna.osgb");
    // osgDB::Registry::instance()->getDataFilePathList().push_back(
    //     "D:/OSG_QT_Project/models/uav/source/"
    //     );
    // osgDB::Registry::instance()->getDataFilePathList().push_back(
    //     "D:/OSG_QT_Project/models/uav/textures/"
    //     );

    // osg::ref_ptr<osg::Node> model =
    //     osgDB::readNodeFile("D:/OSG_QT_Project/models/uav/source/uAV.obj");
    if (model.valid()) {
        osg::StateSet* ss = model->getOrCreateStateSet();

        osg::ref_ptr<osg::Material> redMat = new osg::Material;
        redMat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 0, 0, 1));
        redMat->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0.5, 0, 0, 1));
        redMat->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0.8, 0, 0, 1));

        ss->setAttributeAndModes(redMat.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
        ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
        ss->setRenderBinDetails(10, "RenderBin");

        osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform();
        mt->setMatrix(osg::Matrix::scale(config.planeScale, config.planeScale, config.planeScale));
        mt->addChild(model.get());

        planePat->addChild(mt.get());
        root->addChild(planePat.get());
    }

    auto* newCb = new TimeBasedMoveCallback(points,
                                            projLineVertices.get(),
                                            projLineGeom.get(),
                                            _mapNode.get());

    QString curTxt = ui->comboBox->currentText();
    double curSpeed = curTxt.left(curTxt.indexOf('x')).toDouble();
    newCb->_timeScale = (curSpeed > 0) ? curSpeed : 1.0;
    newCb->isPaused = false;
    planePat->setUpdateCallback(newCb);

    // 初始化飞机位置
    planePat->setPosition(points.front().worldPos);
    planePat->setAttitude(points.front().rotation);

    // =========================
    // D. 存入 TrackObject
    // =========================
    TrackObject newTrack;
    newTrack.id = trackId;
    newTrack.points = points;
    newTrack.planePat = planePat;
    newTrack.pathGeode = pathGeode;
    newTrack.pathGeom = pathGeom;
    newTrack.projLineGeode = projLineGeode;
    newTrack.projLineGeom = projLineGeom;
    newTrack.projLineVertices = projLineVertices;

    newTrack.pointCount = static_cast<int>(points.size());
    newTrack.totalDuration = points.back().timeOffset;

    osgEarth::GeoPoint startGeo, endGeo;
    if (startGeo.fromWorld(_mapNode->getMapSRS(), points.front().worldPos)) {
        newTrack.startLon = startGeo.x();
        newTrack.startLat = startGeo.y();
    }
    if (endGeo.fromWorld(_mapNode->getMapSRS(), points.back().worldPos)) {
        newTrack.endLon = endGeo.x();
        newTrack.endLat = endGeo.y();
    }

    _allTracks.insert(trackId, newTrack);
    _currentTrackId = trackId;

    // =========================
    // E. 绑定“当前选中轨迹”的主指针
    // =========================
    _planePat = planePat;
    _pathGeom = pathGeom;
    _projLineGeode = projLineGeode;
    _projLineGeom = projLineGeom;
    _projLineVertices = projLineVertices;

    // =========================
    // F. UI 更新
    // =========================
    ui->listWidgetTracks->addItem(trackId);
    ui->listWidgetTracks->setCurrentRow(ui->listWidgetTracks->count() - 1);

    ui->btnResetView->setEnabled(true);
    ui->sliderProgress->setEnabled(true);
    ui->sliderProgress->setRange(0, static_cast<int>(newCb->_totalDuration * 10));
    ui->sliderProgress->setValue(0);

    ui->labelState->setText(QString("状态: 已加载轨迹 (%1个点)").arg(points.size()));
    refreshTrackInfoPanel();

    osgEarth::Util::EarthManipulator* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(
        _osgWidget->getViewer()->getCameraManipulator());
    if (manip) {
        osgEarth::GeoPoint firstPt;
        if (firstPt.fromWorld(_mapNode->getMapSRS(), points[0].worldPos)) {
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


void MainWindow::on_checkShowProjection_clicked(bool checked)
{
    for (auto it = _allTracks.begin(); it != _allTracks.end(); ++it) {
        if (it.value().projLineGeode.valid()) {
            it.value().projLineGeode->setNodeMask(checked ? 0xffffffff : 0x0);
        }
    }
}

// 列表切换逻辑
void MainWindow::on_listWidgetTracks_itemClicked(QListWidgetItem *item)
{
    if (!item) return;

    QString trackId = item->text();
    if (!_allTracks.contains(trackId)) return;

    _currentTrackId = trackId;
    TrackObject& track = _allTracks[trackId];

    // 主指针重新绑定到当前选中的航迹
    _planePat = track.planePat;
    _projLineGeode = track.projLineGeode;
    _projLineGeom = track.projLineGeom;
    _projLineVertices = track.projLineVertices;
    _pathGeom = track.pathGeom;
    refreshTrackInfoPanel();

    if (_planePat.valid()) {
        auto* cb = dynamic_cast<TimeBasedMoveCallback*>(_planePat->getUpdateCallback());
        if (cb) {
            ui->sliderProgress->blockSignals(true);
            ui->sliderProgress->setRange(0, static_cast<int>(cb->_totalDuration * 10));
            ui->sliderProgress->setValue(static_cast<int>(cb->_currentTime * 10));
            ui->sliderProgress->blockSignals(false);

            _isPaused = cb->isPaused;
            ui->btnPlayPause->setText(_isPaused ? "继续播放" : "暂停播放");
        }
    }

    // 点击后视角聚焦到当前选中飞机
    osgEarth::Util::EarthManipulator* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(
        _osgWidget->getViewer()->getCameraManipulator());

    if (manip && _planePat.valid() && _mapNode.valid()) {
        osg::Vec3d worldPos = _planePat->getPosition();
        osgEarth::GeoPoint geoPos;
        if (geoPos.fromWorld(_mapNode->getMapSRS(), worldPos)) {
            double followRange = 3000.0;   //复位后的高度

            osgEarth::Viewpoint vp("Focus", geoPos.x(), geoPos.y(), geoPos.z(), 0, -45, followRange);
            _isFollowing = false;
            _resetTimer.start();
            manip->setViewpoint(vp, 2.0);

            QTimer::singleShot(2100, [this,followRange]() {
                this->_lastRange = followRange;
                this->_isFollowing = true;  //(111)
                ui->labelState->setText("状态: 自动跟随");
            });
        }
    }
}

// 删除航迹逻辑
void MainWindow::on_btnDeleteTrack_clicked()
{
    QListWidgetItem* item = ui->listWidgetTracks->currentItem();
    if (!item) {
        QMessageBox::warning(this, "提示", "请先在列表中选择要删除的航迹！");
        return;
    }

    QString trackId = item->text();

    // 1. 从 OSG 场景图中安全移除渲染节点
    osg::Group* root = _osgWidget->getViewer()->getSceneData()->asGroup();
    if (root && _allTracks.contains(trackId)) {
        TrackObject& track = _allTracks[trackId];
        if (track.planePat.valid()) root->removeChild(track.planePat);
        if (track.projLineGeode.valid()) root->removeChild(track.projLineGeode);
        if (track.pathGeode.valid()) root->removeChild(track.pathGeode);
    }

    // 2. 从数据结构和 UI 列表中移除
    _allTracks.remove(trackId);
    delete item; // 释放 UI item 内存

    // 3. 处理善后：如果删除的是当前正在播放的航迹，需要重置状态
    if (_currentTrackId == trackId) {
        if (_allTracks.isEmpty()) {
            // 全部删空了，彻底清空状态
            _currentTrackId = "";
            _planePat = nullptr;
            _projLineGeode = nullptr;

            ui->sliderProgress->setEnabled(false);
            ui->btnResetView->setEnabled(false);
            ui->labelState->setText("状态: 等待导入数据");
        } else {
            // 还有别的航迹，默认选中剩下的第一个
            ui->listWidgetTracks->setCurrentRow(0);
            on_listWidgetTracks_itemClicked(ui->listWidgetTracks->currentItem());
        }
    }
}

