#ifndef REFACTOR_MAPSCENEMANAGER_H
#define REFACTOR_MAPSCENEMANAGER_H

#include <QString>
#include <osg/Node>
#include <osg/Vec3d>
#include <osg/observer_ptr>
#include <osg/ref_ptr>
#include <osgEarth/MapNode>

#include <string>

#include "SystemConfig.h"

namespace osgEarth {
class XYZImageLayer;
}

namespace osgViewer {
class Viewer;
}

// 管理 osgEarth 地图场景初始化、地图节点访问和基础坐标转换。
class MapSceneManager {
public:
    enum ErrorType {
        NoError,
        EarthFileLoadFailed,
        MapNodeNotFound
    };

    bool initialize(const QString& earthFile = SystemConfig::Resource::MapFile);
    void setupEarthManipulator(osgViewer::Viewer* viewer) const;

    osg::Node* loadedNode() const { return _loadedNode.get(); }
    osgEarth::MapNode* mapNode() const { return _mapNode.get(); }
    ErrorType errorType() const { return _errorType; }
    QString lastError() const { return _lastError; }

    osg::Vec3d geoToWorld(double lon, double lat, double alt) const;
    bool worldToGeo(const osg::Vec3d& world, double& lon, double& lat, double& alt) const;
    double sampleTerrainHeight(double lon, double lat) const;

    static osgEarth::XYZImageLayer* createTianDiTuLayer(
        const std::string& key,
        const std::string& type,
        const std::string& name);

private:
    osg::ref_ptr<osg::Node> _loadedNode;
    osg::observer_ptr<osgEarth::MapNode> _mapNode;
    ErrorType _errorType = NoError;
    QString _lastError;
};

#endif // REFACTOR_MAPSCENEMANAGER_H
