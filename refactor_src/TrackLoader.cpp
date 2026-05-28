#include "TrackLoader.h"

#include <QDateTime>
#include <QDebug>
#include <QFile>
#include <QStringList>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <osg/Math>
#include <osgEarth/GeoData>

// 按约定字段读取 CSV，并生成航迹点序列与航迹线顶点数组。
TrackLoadResult TrackLoader::loadCsv(const QString& path, osgEarth::MapNode* mapNode) const
{
    TrackLoadResult result;
    result.lineCoords = new osg::Vec3Array();

    if (path.isEmpty()) {
        result.errorType = TrackLoadResult::EmptyPath;
        result.errorMessage = "CSV path is empty.";
        qWarning() << "[航迹导入]" << "导入失败：CSV 文件路径为空";
        return result;
    }
    if (!mapNode) {
        result.errorType = TrackLoadResult::NullMapNode;
        result.errorMessage = "MapNode is null.";
        qWarning() << "[航迹导入]" << "导入失败：MapNode 为空";
        return result;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        result.errorType = TrackLoadResult::OpenFailed;
        result.errorMessage = "Cannot open selected CSV file.";
        qWarning() << "[航迹导入]" << "CSV 文件打开失败：" << path;
        return result;
    }
    qInfo() << "[航迹导入]" << "CSV 文件打开成功";

    QDateTime firstTime;
    bool firstPoint = true;
    bool isFirstLine = true;
    bool hasHeader = false;
    int lineNumber = 0;

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine();
        lineNumber++;
        if (line.trimmed().isEmpty()) continue;
        if (isFirstLine) {
            isFirstLine = false;
            hasHeader = true;
            QStringList headers = line.split(',');
            if (headers.size() >= 7) {
                qInfo() << "[航迹导入]" << "表头字段检查通过";
            } else {
                qWarning() << "[航迹导入]" << "表头字段缺失，字段数量：" << headers.size();
            }
            continue;
        }

        QStringList cols = line.split(',');
        if (cols.size() < 7) {
            qWarning() << "[航迹导入]" << QString("第 %1 行数据无效，已跳过").arg(lineNumber);
            continue;
        }

        TrackPoint tp;

        QString fileTrackId = cols[0].trimmed();
        QString timeStr = cols[1].trimmed();
        if (result.csvTrackId.isEmpty()) {
            result.csvTrackId = fileTrackId;
        }

        QDateTime currentTime = QDateTime::fromString(timeStr, Qt::ISODate);
        if (!currentTime.isValid()) {
            qWarning() << "[航迹导入]" << QString("第 %1 行时间字段无效，已跳过").arg(lineNumber);
            continue;
        }

        if (firstPoint) {
            firstTime = currentTime;
            firstPoint = false;
        }

        tp.timeOffset = firstTime.secsTo(currentTime);

        tp.lon = cols[2].toDouble();
        tp.lat = cols[3].toDouble();
        tp.alt = cols[4].toDouble();
        tp.velocity = cols[5].toDouble();
        tp.headingDeg = cols[6].toDouble();

        const osgEarth::SpatialReference* geoSRS =
            mapNode->getMapSRS()->getGeographicSRS();

        double baseGroundHeight = 500.0;

        osgEarth::GeoPoint(
            geoSRS,
            tp.lon,
            tp.lat,
            tp.alt + baseGroundHeight,
            osgEarth::ALTMODE_ABSOLUTE
            ).toWorld(tp.worldPos);

        tp.rotation = computeRotationFromHeading(tp.headingDeg, tp.lon, tp.lat);

        result.points.push_back(tp);
        result.lineCoords->push_back(tp.worldPos);
    }

    if (result.points.empty()) {
        result.errorType = TrackLoadResult::EmptyData;
        result.errorMessage = "Track data is empty. Expected fields: track_id,timestamp,longitude,latitude,altitude_m,speed_mps,heading_deg";
        if (!hasHeader) {
            qWarning() << "[航迹导入]" << "导入失败：CSV 文件为空";
        } else {
            qWarning() << "[航迹导入]" << "导入失败：未解析到有效航迹点";
        }
        return result;
    }

    if (result.points.size() >= 2) {
        for (int i = 0; i < static_cast<int>(result.points.size()); ++i) {
            int nextIndex = (i < static_cast<int>(result.points.size()) - 1) ? i + 1 : i;
            int prevIndex = (i > 0) ? i - 1 : i;
            osg::Vec3d dir = result.points[nextIndex].worldPos - result.points[prevIndex].worldPos;
            result.points[i].rotation = computeRotationFromDirection(
                dir,
                result.points[i].lon,
                result.points[i].lat);
        }
    }

    qInfo() << "[航迹导入]" << "有效航迹点数量：" << static_cast<int>(result.points.size());
    qInfo() << "[速度检查]" << "开始进行速度一致性检查";
    checkSpeed(result.points, result);
    if (result.abnormalSpeedCount > 0) {
        qWarning() << "[速度检查]" << "速度异常数量：" << result.abnormalSpeedCount;
    } else {
        qInfo() << "[速度检查]" << "速度检查完成，未发现明显异常";
    }
    result.errorType = TrackLoadResult::NoError;
    result.ok = true;
    return result;
}

// 根据相邻航迹点的空间距离和时间差统计速度异常。
void TrackLoader::checkSpeed(std::vector<TrackPoint>& points, TrackLoadResult& result) const
{
    for (int i = 1; i < static_cast<int>(points.size()); ++i) {
        TrackPoint& prev = points[i - 1];
        TrackPoint& cur = points[i];

        double dt = cur.timeOffset - prev.timeOffset;

        if (dt <= 0.0) {
            cur.speedAbnormal = true;
            result.abnormalSpeedCount++;
            qWarning() << "[速度检查]" << QString("第 %1 段速度异常，时间间隔无效").arg(i);
            continue;
        }

        double horizontalDist = calcGeoDistanceMeters(
            prev.lon, prev.lat,
            cur.lon, cur.lat
            );

        double verticalDist = cur.alt - prev.alt;
        double totalDist = std::sqrt(horizontalDist * horizontalDist +
                                     verticalDist * verticalDist);
        double actualSpeed = totalDist / dt;

        cur.actualSpeed = actualSpeed;
        cur.speedError = std::abs(actualSpeed - cur.velocity);

        result.maxActualSpeed = std::max(result.maxActualSpeed, actualSpeed);
        result.maxSpeedError = std::max(result.maxSpeedError, cur.speedError);

        double relativeError = 0.0;
        if (cur.velocity > 0.1) {
            relativeError = cur.speedError / cur.velocity;
        }

        double climbRate = verticalDist / dt;
        double accel = (cur.velocity - prev.velocity) / dt;

        bool isTakeoffOrLanding =
            cur.alt < 80.0 ||
            std::abs(climbRate) > 2.5 ||
            cur.velocity < 10.0;

        if (isTakeoffOrLanding) {
            if (actualSpeed > 35.0 || std::abs(accel) > 6.0) {
                cur.speedAbnormal = true;
                result.abnormalSpeedCount++;
                qWarning() << "[速度检查]"
                           << QString("第 %1 段速度异常，文件速度：%2 m/s，计算速度：%3 m/s")
                                  .arg(i)
                                  .arg(cur.velocity, 0, 'f', 1)
                                  .arg(actualSpeed, 0, 'f', 1);
            }
            else if (actualSpeed > 28.0 || std::abs(accel) > 4.0) {
                cur.speedWarning = true;
                result.warningSpeedCount++;
            }
        } else {
            if (actualSpeed > 45.0 ||
                (cur.speedError > 10.0 && relativeError > 0.5)) {
                cur.speedAbnormal = true;
                result.abnormalSpeedCount++;
                qWarning() << "[速度检查]"
                           << QString("第 %1 段速度异常，文件速度：%2 m/s，计算速度：%3 m/s")
                                  .arg(i)
                                  .arg(cur.velocity, 0, 'f', 1)
                                  .arg(actualSpeed, 0, 'f', 1);
            }
            else if (cur.speedError > 6.0 || relativeError > 0.35) {
                cur.speedWarning = true;
                result.warningSpeedCount++;
            }
        }
    }
}

// 使用球面距离公式估算两组经纬度之间的水平距离。
double TrackLoader::calcGeoDistanceMeters(double lon1, double lat1, double lon2, double lat2)
{
    static const double R = 6371000.0;

    double radLat1 = osg::DegreesToRadians(lat1);
    double radLat2 = osg::DegreesToRadians(lat2);
    double dLat = osg::DegreesToRadians(lat2 - lat1);
    double dLon = osg::DegreesToRadians(lon2 - lon1);

    double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0) +
               std::cos(radLat1) * std::cos(radLat2) *
                   std::sin(dLon / 2.0) * std::sin(dLon / 2.0);

    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    return R * c;
}

// 根据航向角生成无人机模型姿态四元数。
osg::Quat TrackLoader::computeRotationFromHeading(double headingDeg)
{
    return computeRotationFromHeading(headingDeg, 0.0, 0.0);
}

osg::Quat TrackLoader::computeRotationFromHeading(double headingDeg, double lon, double lat)
{
    double rad = osg::DegreesToRadians(headingDeg);
    double lonRad = osg::DegreesToRadians(lon);
    double latRad = osg::DegreesToRadians(lat);

    osg::Vec3d east(-std::sin(lonRad), std::cos(lonRad), 0.0);
    osg::Vec3d north(-std::sin(latRad) * std::cos(lonRad),
                     -std::sin(latRad) * std::sin(lonRad),
                     std::cos(latRad));
    osg::Vec3d up(std::cos(latRad) * std::cos(lonRad),
                  std::cos(latRad) * std::sin(lonRad),
                  std::sin(latRad));

    osg::Vec3d dir = east * std::sin(rad) + north * std::cos(rad);
    return computeRotationFromDirection(dir, lon, lat);
}

osg::Quat TrackLoader::computeRotationFromDirection(const osg::Vec3d& directionWorld, double lon, double lat)
{
    osg::Vec3d forward = directionWorld;
    if (forward.length2() < 1e-6) {
        return computeRotationFromHeading(0.0, lon, lat);
    }
    forward.normalize();

    double lonRad = osg::DegreesToRadians(lon);
    double latRad = osg::DegreesToRadians(lat);
    osg::Vec3d up(std::cos(latRad) * std::cos(lonRad),
                  std::cos(latRad) * std::sin(lonRad),
                  std::sin(latRad));
    up.normalize();

    double offsetRad = osg::DegreesToRadians(SystemConfig::Render::ModelHeadingOffsetDeg);
    osg::Vec3d modelForward(-std::sin(offsetRad), -std::cos(offsetRad), 0.0);

    osg::Quat forwardRotation;
    forwardRotation.makeRotate(modelForward, forward);

    osg::Vec3d currentUp = forwardRotation * osg::Vec3d(0, 0, 1);
    osg::Vec3d desiredUp = up - forward * (up * forward);
    osg::Vec3d projectedCurrentUp = currentUp - forward * (currentUp * forward);

    if (desiredUp.length2() > 1e-6 && projectedCurrentUp.length2() > 1e-6) {
        desiredUp.normalize();
        projectedCurrentUp.normalize();

        double dot = std::max(-1.0, std::min(1.0, projectedCurrentUp * desiredUp));
        double angle = std::acos(dot);
        if ((projectedCurrentUp ^ desiredUp) * forward < 0.0) {
            angle = -angle;
        }

        osg::Quat rollCorrection;
        rollCorrection.makeRotate(angle, forward);
        return applyModelAttitudeOffset(rollCorrection * forwardRotation);
    }

    return applyModelAttitudeOffset(forwardRotation);
}

osg::Quat TrackLoader::applyModelAttitudeOffset(const osg::Quat& baseAttitude)
{
    const double headingRad = osg::DegreesToRadians(SystemConfig::Render::ModelHeadingOffsetDeg);
    const osg::Vec3d localUp(0.0, 0.0, 1.0);
    const osg::Vec3d localForward(-std::sin(headingRad), -std::cos(headingRad), 0.0);
    osg::Vec3d localRight = localUp ^ localForward;
    if (localRight.length2() < 1e-6) {
        localRight.set(1.0, 0.0, 0.0);
    } else {
        localRight.normalize();
    }

    osg::Quat pitchOffset;
    pitchOffset.makeRotate(osg::DegreesToRadians(SystemConfig::Render::ModelPitchOffsetDeg),
                           localRight);

    osg::Quat rollOffset;
    rollOffset.makeRotate(osg::DegreesToRadians(SystemConfig::Render::ModelRollOffsetDeg),
                          localForward);

    osg::Quat finalAttitude = baseAttitude * pitchOffset * rollOffset;

    static int logCount = 0;
    if (logCount < 12) {
        qInfo() << "[姿态调试]"
                << "headingOffset =" << SystemConfig::Render::ModelHeadingOffsetDeg
                << "pitchOffset =" << SystemConfig::Render::ModelPitchOffsetDeg
                << "rollOffset =" << SystemConfig::Render::ModelRollOffsetDeg
                << "localForward =" << localForward.x() << localForward.y() << localForward.z()
                << "localRight =" << localRight.x() << localRight.y() << localRight.z()
                << "finalQuat =" << finalAttitude.x() << finalAttitude.y()
                << finalAttitude.z() << finalAttitude.w();
        ++logCount;
    }

    return finalAttitude;
}
