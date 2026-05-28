#include "TrackNodeBuilder.h"
#include "SystemConfig.h"

#include <QDebug>
#include <algorithm>
#include <osg/Depth>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/MatrixTransform>
#include <osg/Point>
#include <osg/PolygonOffset>
#include <osgDB/ReadFile>
#include <osgEarth/GeoData>
#include <osgEarth/Registry>

// 创建一条航迹对应的全部场景节点，并写入 TrackObject 节点引用。
TrackObject TrackNodeBuilder::buildTrackObject(const QString& trackId,
                                               const TrackLoadResult& loadResult,
                                               osg::Group* root,
                                               osgEarth::MapNode* mapNode,
                                               bool projectionVisible,
                                               const TrackRenderConfig& config) const
{
    TrackObject newTrack;
    newTrack.id = trackId;
    newTrack.points = loadResult.points;

    if (!root || !mapNode || loadResult.points.empty()) {
        qWarning() << "[航迹显示]" << "航迹显示对象创建失败：输入数据无效";
        // 调用方负责处理无效输入对应的界面提示。
        return newTrack;
    }

    osg::Geometry* pathGeom = nullptr;
    qInfo() << "[航迹显示]" << "开始生成航迹线";
    newTrack.pathGeode = createPathGeode(loadResult.lineCoords.get(), &pathGeom, config);
    newTrack.pathGeom = pathGeom;
    root->addChild(newTrack.pathGeode.get());
    qInfo() << "[航迹显示]" << "航迹线加入三维场景";

    newTrack.pointGeode = createPointGeode(trackId, loadResult.lineCoords.get());
    root->addChild(newTrack.pointGeode.get());

    newTrack.basePlaneGeode = createBasePlaneGeode(trackId, loadResult.points, mapNode);
    root->addChild(newTrack.basePlaneGeode.get());

    osg::Geometry* projLineGeom = nullptr;
    osg::Vec3Array* projLineVertices = nullptr;
    newTrack.projLineGeode = createProjectionLineGeode(
        trackId,
        loadResult.points,
        &projLineGeom,
        &projLineVertices,
        projectionVisible);
    newTrack.projLineGeom = projLineGeom;
    newTrack.projLineVertices = projLineVertices;
    root->addChild(newTrack.projLineGeode.get());

    newTrack.planePat = createPlanePat(loadResult.points, config);
    if (newTrack.planePat.valid() && newTrack.planePat->getNumChildren() > 0) {
        root->addChild(newTrack.planePat.get());
    }
    qInfo() << "[航迹显示]" << "航迹对象已加入三维场景";

    newTrack.planePat->setPosition(loadResult.points.front().worldPos);
    newTrack.planePat->setAttitude(loadResult.points.front().rotation);
    qInfo() << "[模型回放]" << "模型初始位置设置完成";
    qInfo() << "[航迹显示]" << "航迹显示对象创建完成，航迹ID：" << trackId;

    return newTrack;
}

// 创建航迹线节点。
osg::ref_ptr<osg::Geode> TrackNodeBuilder::createPathGeode(osg::Vec3Array* lineCoords,
                                                           osg::Geometry** outPathGeom,
                                                           const TrackRenderConfig& config) const
{
    osg::ref_ptr<osg::Geometry> pathGeom = new osg::Geometry();
    pathGeom->setVertexArray(lineCoords);

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

    if (outPathGeom) *outPathGeom = pathGeom.get();
    qInfo() << "[航迹显示]" << "航迹线节点创建完成，点数：" << static_cast<int>(lineCoords->size());
    return pathGeode;
}

// 创建航迹采样点节点，默认保持隐藏。
osg::ref_ptr<osg::Geode> TrackNodeBuilder::createPointGeode(const QString& trackId,
                                                            osg::Vec3Array* lineCoords) const
{
    osg::ref_ptr<osg::Geometry> pointGeom = new osg::Geometry();
    pointGeom->setVertexArray(lineCoords);

    osg::ref_ptr<osg::Vec4Array> pointColors = new osg::Vec4Array();
    pointColors->push_back(osg::Vec4(1.0f, 0.2f, 0.2f, 1.0f));
    pointGeom->setColorArray(pointColors.get(), osg::Array::BIND_OVERALL);
    pointGeom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, lineCoords->size()));

    osg::ref_ptr<osg::Geode> pointGeode = new osg::Geode();
    pointGeode->addDrawable(pointGeom.get());
    pointGeode->setName("TrackPoints_" + trackId.toStdString());

    osg::StateSet* pointSS = pointGeode->getOrCreateStateSet();
    pointSS->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    pointSS->setAttributeAndModes(new osg::Point(SystemConfig::Render::TrackPointSize), osg::StateAttribute::ON);
    pointSS->setRenderBinDetails(10, "RenderBin");

    osgEarth::Registry::shaderGenerator().run(pointGeode.get());
    pointGeode->setNodeMask(0x0);

    qInfo() << "[航迹显示]" << "航迹点节点创建完成";
    return pointGeode;
}

// 创建 500m 高度参考面节点，默认保持隐藏。
osg::ref_ptr<osg::Geode> TrackNodeBuilder::createBasePlaneGeode(const QString& trackId,
                                                                const std::vector<TrackPoint>& points,
                                                                osgEarth::MapNode* mapNode) const
{
    double minLon = points.front().lon;
    double maxLon = points.front().lon;
    double minLat = points.front().lat;
    double maxLat = points.front().lat;

    for (const TrackPoint& p : points) {
        minLon = std::min(minLon, p.lon);
        maxLon = std::max(maxLon, p.lon);
        minLat = std::min(minLat, p.lat);
        maxLat = std::max(maxLat, p.lat);
    }

    double marginLon = SystemConfig::Render::BasePlaneMarginLon;
    double marginLat = SystemConfig::Render::BasePlaneMarginLat;

    minLon -= marginLon;
    maxLon += marginLon;
    minLat -= marginLat;
    maxLat += marginLat;

    double baseHeight = SystemConfig::Render::BasePlaneHeight;

    const osgEarth::SpatialReference* geoSRS =
        mapNode->getMapSRS()->getGeographicSRS();

    osg::ref_ptr<osg::Vec3Array> planeVerts = new osg::Vec3Array();

    auto addPlanePoint = [&](double lon, double lat) {
        osg::Vec3d world;
        osgEarth::GeoPoint(
            geoSRS,
            lon,
            lat,
            baseHeight,
            osgEarth::ALTMODE_ABSOLUTE
            ).toWorld(world);

        planeVerts->push_back(world);
    };

    addPlanePoint(minLon, minLat);
    addPlanePoint(maxLon, minLat);
    addPlanePoint(maxLon, maxLat);
    addPlanePoint(minLon, maxLat);

    osg::ref_ptr<osg::Geometry> basePlaneGeom = new osg::Geometry();
    basePlaneGeom->setVertexArray(planeVerts.get());
    basePlaneGeom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

    osg::ref_ptr<osg::Vec4Array> planeColors = new osg::Vec4Array();
    planeColors->push_back(osg::Vec4(0.2f, 0.7f, 1.0f, 0.18f));
    basePlaneGeom->setColorArray(planeColors.get(), osg::Array::BIND_OVERALL);

    osg::ref_ptr<osg::Geode> basePlaneGeode = new osg::Geode();
    basePlaneGeode->addDrawable(basePlaneGeom.get());
    basePlaneGeode->setName("BaseHeightPlane_" + trackId.toStdString());

    osg::StateSet* planeSS = basePlaneGeode->getOrCreateStateSet();
    planeSS->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
    planeSS->setMode(GL_BLEND, osg::StateAttribute::ON);
    planeSS->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    planeSS->setRenderBinDetails(20, "DepthSortedBin");

    osg::Depth* depth = new osg::Depth;
    depth->setWriteMask(false);
    planeSS->setAttributeAndModes(depth, osg::StateAttribute::ON);

    osgEarth::Registry::shaderGenerator().run(basePlaneGeode.get());
    basePlaneGeode->setNodeMask(0x0);

    qInfo() << "[辅助可视化]" << "高度参考面创建完成，高度：" << baseHeight << "m";
    return basePlaneGeode;
}

// 创建无人机到地面的动态投影线节点。
osg::ref_ptr<osg::Geode> TrackNodeBuilder::createProjectionLineGeode(const QString& trackId,
                                                                     const std::vector<TrackPoint>& points,
                                                                     osg::Geometry** outGeom,
                                                                     osg::Vec3Array** outVertices,
                                                                     bool projectionVisible) const
{
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
    projSS->setAttributeAndModes(new osg::LineWidth(SystemConfig::Render::ProjectionLineWidth), osg::StateAttribute::ON);
    projSS->setRenderBinDetails(100, "RenderBin");
    projSS->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

    projLineGeode->setNodeMask(projectionVisible ? 0xffffffff : 0x0);

    if (outGeom) *outGeom = projLineGeom.get();
    if (outVertices) *outVertices = projLineVertices.get();
    qInfo() << "[辅助可视化]" << "投影线节点创建完成";
    return projLineGeode;
}

// 创建无人机模型变换节点。
osg::ref_ptr<osg::PositionAttitudeTransform> TrackNodeBuilder::createPlanePat(
    const std::vector<TrackPoint>& points,
    const TrackRenderConfig& config) const
{
    osg::ref_ptr<osg::PositionAttitudeTransform> planePat = new osg::PositionAttitudeTransform();
    planePat->setNodeMask(0xffffffff);
    qInfo() << "[模型回放]" << "PositionAttitudeTransform 节点创建完成";

    qInfo() << "[模型回放]" << "开始加载无人机模型：" << config.modelPath;
    osg::ref_ptr<osg::Node> model =
        osgDB::readNodeFile(config.modelPath.toStdString());
    if (model.valid()) {
        qInfo() << "[模型回放]" << "无人机模型加载成功";
        osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform();
        mt->setMatrix(osg::Matrix::scale(config.planeScale, config.planeScale, config.planeScale));
        mt->addChild(model.get());

        planePat->addChild(mt.get());
    } else {
        qWarning() << "[模型回放]" << "无人机模型加载失败：" << config.modelPath;
    }

    if (!points.empty()) {
        planePat->setPosition(points.front().worldPos);
        planePat->setAttitude(points.front().rotation);
    }

    return planePat;
}
