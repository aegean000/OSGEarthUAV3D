#ifndef REFACTOR_TRACKANIMATIONCALLBACK_H
#define REFACTOR_TRACKANIMATIONCALLBACK_H

#include "TrackData.h"

#include <osg/NodeCallback>
#include <osg/observer_ptr>
#include <osgEarth/MapNode>

// 基于播放时间驱动无人机模型沿航迹运动，并同步更新投影线。
class TrackAnimationCallback : public osg::NodeCallback {
public:
    std::vector<TrackPoint> _track;
    double _currentTime = 0.0;
    double _totalDuration = 0.0;
    double _timeScale = 1.0;
    double _lastFrameTime = -1.0;
    bool isPaused = true;
    double _currentVelocity = 0.0;

    osg::observer_ptr<osg::Vec3Array> _lineVerts;
    osg::observer_ptr<osg::Geometry> _lineGeom;
    osg::observer_ptr<osgEarth::MapNode> _mapNode;

    TrackAnimationCallback(const std::vector<TrackPoint>& data,
                           osg::Vec3Array* v,
                           osg::Geometry* g,
                           osgEarth::MapNode* mn);

    void setTime(double t);
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) override;
};

#endif // REFACTOR_TRACKANIMATIONCALLBACK_H
