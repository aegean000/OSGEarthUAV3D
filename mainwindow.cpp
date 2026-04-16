#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "QOSGWidget.h"

// OSG & osgEarth 头文件
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Material>

#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/TMS>
#include <osgEarth/EarthManipulator>
#include <osgEarth/GLUtils>
#include <osg/PolygonOffset>
#include <osgEarth/EarthManipulator>


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
    QVBoxLayout* layout = new QVBoxLayout(ui->renderBase); // 关联到 UI 里的渲染区域
    layout->addWidget(osgWidget);
    layout->setContentsMargins(0, 0, 0, 0); // 让地图紧贴边框

    // 为了以后能用，把 osgWidget 存起来（建议在 mainwindow.h 声明为成员变量）
    // this->_osgWidget = osgWidget;

    // 3. 配置参数
    FlightConfig config;
    config.speed = 0.002;
    config.showPath = true;
    config.showPlane = true;

    // 4. 构建地图节点 (MapNode)
    osg::ref_ptr<Map> map = new Map();
    TMSImageLayer::Options tms;
    tms.url() = "https://readymap.org/readymap/tiles/1.0.0/7/";
    map->addLayer(new TMSImageLayer(tms));

    osg::ref_ptr<MapNode> mapNode = new MapNode(map);
    const SpatialReference* geoSRS = mapNode->getMapSRS();

    // 5. 准备航迹坐标
    std::vector<osg::Vec3d> worldPath;
    osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array();
    double coords[][3] = { {139.7, 35.6, 3000}, {140.0, 35.8, 3000}, {140.5, 35.7, 3000} };

    for (int i = 0; i < 3; ++i) {
        osg::Vec3d v;
        GeoPoint(geoSRS, coords[i][0], coords[i][1], coords[i][2], ALTMODE_ABSOLUTE).toWorld(v);
        worldPath.push_back(v);
        va->push_back(v);
    }

    // 6. 创建场景根节点
    osg::ref_ptr<osg::Group> root = new osg::Group();
    root->addChild(mapNode);

    // 7. 绘制航线
    if (config.showPath) {
        osg::ref_ptr<osg::Geometry> lineGeom = new osg::Geometry();
        lineGeom->setVertexArray(va);
        lineGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP, 0, va->size()));

        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
        colors->push_back(config.pathColor);
        lineGeom->setColorArray(colors, osg::Array::BIND_OVERALL);

        osg::ref_ptr<osg::Geode> lineGeode = new osg::Geode();
        lineGeode->addDrawable(lineGeom);

        osg::StateSet* ss = lineGeode->getOrCreateStateSet();
        // 开启深度偏移，让线稍微“浮”在地球表面一点点
        ss->setAttributeAndModes(new osg::PolygonOffset(1.0f, 1.0f), osg::StateAttribute::ON);
        ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
        ss->setAttributeAndModes(new osg::LineWidth(config.lineWidth), osg::StateAttribute::ON);

        osgEarth::Registry::shaderGenerator().run(lineGeode);
        root->addChild(lineGeode);
        ss->setRenderBinDetails(1, "RenderBin"); // 确保航迹先画
    }
    osg::ref_ptr<osg::PositionAttitudeTransform> pat;
    _planePat = new osg::PositionAttitudeTransform();   //将其赋值给_planePat，方便点击事件控制
    // 8. 绘制飞机
    if (config.showPlane) {
        osg::ref_ptr<osg::Node> model = osgDB::readNodeFile("D:/OSG/QT-osgearth/osgearth/data/cessna.osgb");
        if (model.valid()) {
            // 1. 强制关闭该模型的所有 Shader 影响，先看看裸模型颜色
            osg::StateSet* ss = model->getOrCreateStateSet();

            // 3. 修改材质：添加自发光 (Emission)
            osg::ref_ptr<osg::Material> redMat = new osg::Material;
            redMat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 0, 0, 1)); // 红色散射光
            redMat->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0.5, 0, 0, 1)); // 深红环境光
            // ✅ 关键：设置自发光颜色，这样即便没有灯光，它自己也会呈现红色
            redMat->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0.8, 0, 0, 1));

            ss->setAttributeAndModes(redMat.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);

            // ✅ 4. 重点：确保关闭光照，因为我们已经用了自发光，关了光照它反而会显示纯正的红色
            ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
            // 5. 重要：把飞机的渲染顺序往后排，避免被先画的航迹线污染
            ss->setRenderBinDetails(10, "RenderBin");

            osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform();
            mt->setMatrix(osg::Matrix::scale(config.planeScale, config.planeScale, config.planeScale));
            mt->addChild(model);
            _planePat->addChild(mt);
            _planePat->setUpdateCallback(new MoveCallback(worldPath, config.speed));
            root->addChild(_planePat);
        }
    }

    // 9. 将场景设置到 Viewer 中
    osgWidget->getViewer()->setSceneData(root);

    // 10. 设置漫游器并开启跟随
    osgEarth::Util::EarthManipulator* manip = new osgEarth::Util::EarthManipulator();
    osgWidget->getViewer()->setCameraManipulator(manip);
    // 记录 mapNode，方便坐标转换
    this->_mapNode = mapNode.get();

    // 初始化进度条范围
    ui->sliderProgress->setRange(0, 1000); // 对应你 coords 数组的索引 0, 1, 2

    // 创建定时器刷新 UI 仪表盘
    QTimer* uiTimer = new QTimer(this);
    connect(uiTimer, &QTimer::timeout, this, [this]() {
        if (_planePat.valid() && _mapNode.valid()) {
            // 1. 获取世界坐标并转为地理坐标 (经纬高)
            osg::Vec3d worldPos = _planePat->getPosition();
            osgEarth::GeoPoint geoPos;
            geoPos.fromWorld(_mapNode->getMapSRS(), worldPos);

            // 2. 更新仪表盘文字 (实现综述中的状态监控 )
            ui->labelPos->setText(QString("位置: %1, %2").arg(geoPos.x(), 0, 'f', 4).arg(geoPos.y(), 0, 'f', 4));
            ui->labelHeight->setText(QString("当前高度: %1 m").arg(geoPos.z(), 0, 'f', 1));
            ui->labelAlt->setText(QString("%1 m").arg(geoPos.z(), 0, 'f', 1));

            // 3. 同步进度条位置
            MoveCallback* cb = dynamic_cast<MoveCallback*>(_planePat->getUpdateCallback());
            if (cb && cb->path.size() >= 2) {
                // 计算总进度比例：(当前点索引 + 当前段内的偏移t) / 总段数
                double totalSegments = cb->path.size() - 1;
                double currentGlobalT = (cb->index + cb->t) / totalSegments;

                // 同步到进度条 (0~1000)
                ui->sliderProgress->blockSignals(true);
                ui->sliderProgress->setValue(static_cast<int>(currentGlobalT * 1000));
                ui->sliderProgress->blockSignals(false);

                // 更新百分比文字
                ui->labelState->setText(QString("进度: %1%").arg(currentGlobalT * 100.0, 0, 'f', 1));
            }
        }
    });
    uiTimer->start(100); // 100毫秒刷新一次


}

MainWindow::~MainWindow()
{
    delete ui;
}

// 控制显示隐藏的点击函数
void MainWindow::on_btnTogglePlane_clicked()
{
    // 1. 检查飞机对象是否存在（防止程序崩溃）
    if (_planePat.valid()) {
        // 2. 获取当前的掩码 (NodeMask)
        // OSG 中，掩码为 0x0 表示隐藏，0xffffffff 表示显示
        bool isCurrentlyHidden = (_planePat->getNodeMask() == 0x0);

        if (isCurrentlyHidden) {
            // 如果当前是隐藏的，就改为显示
            _planePat->setNodeMask(0xffffffff);
            // 可选：如果想更人性化，可以改一下按钮文字
            ui->btnTogglePlane->setText("隐藏飞机");
        }
        else {
            // 如果当前是显示的，就改为隐藏
            _planePat->setNodeMask(0x0);
            ui->btnTogglePlane->setText("显示飞机");
        }
    }

}


// 控制无人机飞行进度的点击函数

void MainWindow::on_sliderProgress_valueChanged(int value)
{
    if (_planePat.valid()) {
        MoveCallback* cb = dynamic_cast<MoveCallback*>(_planePat->getUpdateCallback());
        if (cb) {
            double totalSegments = cb->path.size() - 1;
            double globalT = value / 1000.0; // 0.0 ~ 1.0

            double exactIndex = globalT * totalSegments;
            int idx = static_cast<int>(exactIndex);
            double remainderT = exactIndex - idx;

            // 边界处理：如果是最后一点
            if (idx >= totalSegments) {
                idx = totalSegments - 1;
                remainderT = 0.999;
            }

            cb->setProgress(idx, remainderT);
        }
    }
}

