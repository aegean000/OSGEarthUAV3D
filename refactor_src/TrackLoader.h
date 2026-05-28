#ifndef REFACTOR_TRACKLOADER_H
#define REFACTOR_TRACKLOADER_H

#include "TrackData.h"

#include <QString>
#include <osgEarth/MapNode>

// 负责读取 CSV 航迹文件，并转换为系统内部航迹数据。
// 解析结果包含时间偏移、世界坐标、姿态和速度校验统计。
class TrackLoader {
public:
    TrackLoadResult loadCsv(const QString& path, osgEarth::MapNode* mapNode) const;

    static double calcGeoDistanceMeters(double lon1, double lat1, double lon2, double lat2);
    static osg::Quat computeRotationFromHeading(double headingDeg);
    static osg::Quat computeRotationFromHeading(double headingDeg, double lon, double lat);
    static osg::Quat computeRotationFromDirection(const osg::Vec3d& directionWorld, double lon, double lat);
    static osg::Quat applyModelAttitudeOffset(const osg::Quat& baseAttitude);

private:
    void checkSpeed(std::vector<TrackPoint>& points, TrackLoadResult& result) const;
};

#endif // REFACTOR_TRACKLOADER_H
