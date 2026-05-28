#ifndef REFACTOR_TRACKPLAYBACKCONTROLLER_H
#define REFACTOR_TRACKPLAYBACKCONTROLLER_H

#include "TrackAnimationCallback.h"

#include <osg/observer_ptr>
#include <osg/PositionAttitudeTransform>
#include <osg/ref_ptr>

// 封装对 TrackAnimationCallback 的播放控制和状态读取。
class TrackPlaybackController {
public:
    void bind(osg::PositionAttitudeTransform* planePat);

    bool togglePaused();
    void setPaused(bool paused);
    void setTime(double seconds);
    void setTimeScale(double scale);

    bool isPaused() const;
    double currentTime() const;
    double totalDuration() const;
    double currentVelocity() const;
    double timeScale() const;
    bool hasCallback() const;

    TrackAnimationCallback* callback() const;

private:
    osg::observer_ptr<osg::PositionAttitudeTransform> _planePat;
};

#endif // REFACTOR_TRACKPLAYBACKCONTROLLER_H
