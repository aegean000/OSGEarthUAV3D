#ifndef REFACTOR_TRACKDATA_H
#define REFACTOR_TRACKDATA_H

#include <QString>
#include <osg/Array>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/Quat>
#include <osg/Vec3d>
#include <osg/Vec4>
#include <osg/ref_ptr>

#include <vector>

#include "SystemConfig.h"

// 航迹点数据结构，字段名称与业务含义保持稳定。
struct TrackPoint {
    double timeOffset = 0.0;
    osg::Vec3d worldPos;
    osg::Quat rotation;
    double velocity = 0.0;

    double lon = 0.0;
    double lat = 0.0;
    double alt = 0.0;

    double headingDeg = 0.0;
    double actualSpeed = 0.0;
    double speedError = 0.0;
    bool speedAbnormal = false;
    bool speedWarning = false;
};

// 航迹对象数据结构，保存数据统计和场景节点引用。
struct TrackObject {
    QString id;

    osg::ref_ptr<osg::PositionAttitudeTransform> planePat;

    osg::ref_ptr<osg::Geode> projLineGeode;
    osg::ref_ptr<osg::Geometry> projLineGeom;
    osg::ref_ptr<osg::Vec3Array> projLineVertices;

    osg::ref_ptr<osg::Geode> pathGeode;
    osg::ref_ptr<osg::Geometry> pathGeom;

    osg::ref_ptr<osg::Geode> pointGeode;
    osg::ref_ptr<osg::Geode> basePlaneGeode;
    std::vector<TrackPoint> points;

    int pointCount = 0;
    double totalDuration = 0.0;
    double startLon = 0.0;
    double startLat = 0.0;
    double endLon = 0.0;
    double endLat = 0.0;
    QString startTimeText;
    QString endTimeText;
    int warningSpeedCount = 0;
    int abnormalSpeedCount = 0;
    double maxActualSpeed = 0.0;
    double maxSpeedError = 0.0;
};

// CSV 加载结果，包含航迹数据、错误状态和速度校验统计。
struct TrackLoadResult {
    enum ErrorType {
        NoError,
        EmptyPath,
        NullMapNode,
        OpenFailed,
        EmptyData
    };

    bool ok = false;
    ErrorType errorType = NoError;
    QString errorMessage;
    QString csvTrackId;
    std::vector<TrackPoint> points;
    osg::ref_ptr<osg::Vec3Array> lineCoords;

    int warningSpeedCount = 0;
    int abnormalSpeedCount = 0;
    double maxActualSpeed = 0.0;
    double maxSpeedError = 0.0;
};

// 航迹显示参数，控制模型路径、缩放和基础渲染样式。
struct TrackRenderConfig {
    double speed = 0.002;
    bool showPlane = true;
    bool showPath = true;
    float planeScale = SystemConfig::Render::PlaneScale;
    osg::Vec4 pathColor = osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
    float lineWidth = SystemConfig::Render::PathLineWidth;
    QString modelPath = SystemConfig::Resource::DroneModelFile;
};

#endif // REFACTOR_TRACKDATA_H
