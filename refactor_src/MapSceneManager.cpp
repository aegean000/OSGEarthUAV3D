#include "MapSceneManager.h"

#include <QDebug>
#include <osgDB/ReadFile>
#include <osgEarth/EarthManipulator>
#include <osgEarth/GeoData>
#include <osgEarth/Map>
#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osgEarth/XYZ>
#include <osgViewer/Viewer>

// 初始化 osgEarth、加载 .earth 文件，并追加天地图图层。
bool MapSceneManager::initialize(const QString& earthFile)
{
    osgEarth::initialize();

    qInfo() << "[地图加载]" << "开始读取 earth 文件：" << earthFile;
    _loadedNode = osgDB::readNodeFile(earthFile.toStdString());
    if (!_loadedNode.valid()) {
        _errorType = EarthFileLoadFailed;
        _lastError = "Cannot load earth file: " + earthFile;
        qWarning() << "[地图加载]" << "地图加载失败：earth 文件读取失败" << earthFile;
        return false;
    }
    qInfo() << "[地图加载]" << "earth 文件读取成功";

    _mapNode = osgEarth::MapNode::findMapNode(_loadedNode.get());
    if (!_mapNode.valid()) {
        _errorType = MapNodeNotFound;
        _lastError = "Earth file loaded, but MapNode was not found.";
        qWarning() << "[地图加载]" << "地图加载失败：未获取到 MapNode";
        return false;
    }
    qInfo() << "[地图加载]" << "MapNode 获取成功";

    osgEarth::Map* map = _mapNode->getMap();
    std::string key = SystemConfig::TianDiTu::Key;

    osgEarth::XYZImageLayer* imgLayer =
        createTianDiTuLayer(key, SystemConfig::TianDiTu::ImageType, SystemConfig::TianDiTu::ImageLayerName);
    osgEarth::XYZImageLayer* labelLayer =
        createTianDiTuLayer(key, SystemConfig::TianDiTu::LabelType, SystemConfig::TianDiTu::LabelLayerName);

    map->addLayer(imgLayer);
    map->addLayer(labelLayer);

    _errorType = NoError;
    _lastError.clear();
    qInfo() << "[地图加载]" << "地图图层初始化完成";
    return true;
}

// 为 Viewer 设置 osgEarth 地图漫游器。
void MapSceneManager::setupEarthManipulator(osgViewer::Viewer* viewer) const
{
    if (!viewer) {
        qWarning() << "[地图加载]" << "地图浏览器初始化失败：Viewer 为空";
        return;
    }
    osgEarth::Util::EarthManipulator* manip = new osgEarth::Util::EarthManipulator();
    viewer->setCameraManipulator(manip);
    qInfo() << "[地图加载]" << "鼠标地图浏览器初始化完成";
}

// 将地理坐标转换为 OSG 世界坐标。
osg::Vec3d MapSceneManager::geoToWorld(double lon, double lat, double alt) const
{
    osg::Vec3d world;
    if (!_mapNode.valid()) return world;

    const osgEarth::SpatialReference* geoSRS =
        _mapNode->getMapSRS()->getGeographicSRS();

    osgEarth::GeoPoint(
        geoSRS,
        lon,
        lat,
        alt,
        osgEarth::ALTMODE_ABSOLUTE
        ).toWorld(world);

    return world;
}

// 将 OSG 世界坐标转换为地理坐标。
bool MapSceneManager::worldToGeo(const osg::Vec3d& world, double& lon, double& lat, double& alt) const
{
    if (!_mapNode.valid()) return false;

    osgEarth::GeoPoint geoPos;
    if (!geoPos.fromWorld(_mapNode->getMapSRS(), world)) return false;

    lon = geoPos.x();
    lat = geoPos.y();
    alt = geoPos.z();
    return true;
}

// 采样指定经纬度处的地形高度。
double MapSceneManager::sampleTerrainHeight(double lon, double lat) const
{
    if (!_mapNode.valid()) return 0.0;

    double terrainH = 0.0;
    const osgEarth::SpatialReference* geoSRS =
        _mapNode->getMapSRS()->getGeographicSRS();
    // 投影线地形采样需使用与当前地理坐标一致的 SRS。
    _mapNode->getTerrain()->getHeight(geoSRS, lon, lat, &terrainH);
    return terrainH;
}

// 创建天地图 XYZ 影像或注记图层。
osgEarth::XYZImageLayer* MapSceneManager::createTianDiTuLayer(
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

    std::string url =
        std::string(SystemConfig::TianDiTu::UrlPrefix) +
        "T=" + type +
        "&x={x}&y={y}&l={z}&tk=" + key;

    osgEarth::URI uri(url, context);

    osgEarth::XYZImageLayer* layer = new osgEarth::XYZImageLayer();
    layer->setURL(uri);
    layer->setProfile(osgEarth::Profile::create("spherical-mercator"));
    layer->setName(name);
    layer->setOpacity(SystemConfig::TianDiTu::LayerOpacity);

    return layer;
}
