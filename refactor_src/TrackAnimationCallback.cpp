#include "TrackAnimationCallback.h"
#include "TrackLoader.h"

#include <QDebug>
#include <algorithm>
#include <osg/NodeVisitor>
#include <osg/PositionAttitudeTransform>
#include <osgEarth/GeoData>
#include <osgEarth/Terrain>

// 保存航迹序列和投影线节点引用，计算回放总时长。
TrackAnimationCallback::TrackAnimationCallback(const std::vector<TrackPoint>& data,
                                               osg::Vec3Array* v,
                                               osg::Geometry* g,
                                               osgEarth::MapNode* mn)
    : _track(data), _lineVerts(v), _lineGeom(g), _mapNode(mn)
{
    if (!_track.empty()) _totalDuration = _track.back().timeOffset;
    qInfo() << "[模型回放]" << "回放回调创建完成";
}

// 将回放时间定位到指定秒数，并重置帧间隔计算。
void TrackAnimationCallback::setTime(double t)
{
    _currentTime = std::max(0.0, std::min(t, _totalDuration));
    _lastFrameTime = -1.0;
    qInfo() << "[模型回放]" << "当前回放时间跳转到：" << _currentTime << "s";
}

// 在相邻航迹点之间插值，更新模型位置、姿态、速度和投影线。
void TrackAnimationCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    osg::PositionAttitudeTransform* pat = dynamic_cast<osg::PositionAttitudeTransform*>(node);

    if (pat && _track.size() >= 2) {
        double now = nv->getFrameStamp()->getSimulationTime();
        double dt = (_lastFrameTime > 0) ? (now - _lastFrameTime) : 0.0;
        _lastFrameTime = now;

        if (!isPaused) {
            if (dt > 0.2) dt = 0.0166;
            _currentTime += dt * _timeScale;
            if (_currentTime > _totalDuration) {
                qInfo() << "[模型回放]" << "航迹回放结束";
                _currentTime = 0.0;
            }
        }

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

            osgEarth::GeoPoint aircraftGeo;
            if (_mapNode.valid() &&
                aircraftGeo.fromWorld(_mapNode->getMapSRS(), currentPos) &&
                aircraftGeo.isValid()) {
                osg::Quat attitude = TrackLoader::computeRotationFromDirection(
                    p1.worldPos - p0.worldPos,
                    aircraftGeo.x(),
                    aircraftGeo.y());
                pat->setAttitude(attitude);

                static int attitudeLogCount = 0;
                if (attitudeLogCount < 12) {
                    qInfo() << "[姿态调试]"
                            << "TrackAnimationCallback setAttitude"
                            << "time =" << _currentTime
                            << "quat =" << attitude.x() << attitude.y()
                            << attitude.z() << attitude.w();
                    ++attitudeLogCount;
                }
            } else {
                osg::Quat rot;
                rot.slerp(t_ratio, p0.rotation, p1.rotation);
                pat->setAttitude(rot);
            }

            _currentVelocity = p0.velocity + (p1.velocity - p0.velocity) * t_ratio;

            if (_lineVerts.valid() && _lineGeom.valid() && _mapNode.valid()) {
                osgEarth::GeoPoint geoPos;
                geoPos.fromWorld(_mapNode->getMapSRS(), currentPos);

                double terrainH = 0.0;
                _mapNode->getTerrain()->getHeight(geoPos.getSRS(), geoPos.x(), geoPos.y(), &terrainH);

                osgEarth::GeoPoint groundGeo(geoPos.getSRS(), geoPos.x(), geoPos.y(), terrainH);
                osg::Vec3d groundWorldPos;
                groundGeo.toWorld(groundWorldPos);

                (*_lineVerts)[0] = currentPos;
                (*_lineVerts)[1] = groundWorldPos;

                _lineVerts->dirty();
                _lineGeom->dirtyBound();
            }
        }
    }

    traverse(node, nv);
}
