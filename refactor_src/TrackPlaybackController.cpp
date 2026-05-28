#include "TrackPlaybackController.h"

// 绑定当前航迹的模型节点，用于访问其回放回调。
void TrackPlaybackController::bind(osg::PositionAttitudeTransform* planePat)
{
    _planePat = planePat;
}

// 切换播放暂停状态。
bool TrackPlaybackController::togglePaused()
{
    TrackAnimationCallback* cb = callback();
    if (!cb) return true;

    cb->isPaused = !cb->isPaused;
    return cb->isPaused;
}

// 设置暂停状态。
void TrackPlaybackController::setPaused(bool paused)
{
    TrackAnimationCallback* cb = callback();
    if (cb) cb->isPaused = paused;
}

// 设置当前回放时间。
void TrackPlaybackController::setTime(double seconds)
{
    TrackAnimationCallback* cb = callback();
    if (cb) cb->setTime(seconds);
}

// 设置回放倍率。
void TrackPlaybackController::setTimeScale(double scale)
{
    TrackAnimationCallback* cb = callback();
    if (cb && scale > 0) cb->_timeScale = scale;
}

// 读取暂停状态。
bool TrackPlaybackController::isPaused() const
{
    TrackAnimationCallback* cb = callback();
    return cb ? cb->isPaused : true;
}

// 读取当前回放时间。
double TrackPlaybackController::currentTime() const
{
    TrackAnimationCallback* cb = callback();
    return cb ? cb->_currentTime : 0.0;
}

// 读取回放总时长。
double TrackPlaybackController::totalDuration() const
{
    TrackAnimationCallback* cb = callback();
    return cb ? cb->_totalDuration : 0.0;
}

// 读取当前插值速度。
double TrackPlaybackController::currentVelocity() const
{
    TrackAnimationCallback* cb = callback();
    return cb ? cb->_currentVelocity : 0.0;
}

// 读取当前播放倍率。
double TrackPlaybackController::timeScale() const
{
    TrackAnimationCallback* cb = callback();
    return cb ? cb->_timeScale : 1.0;
}

// 判断当前模型节点是否绑定回放回调。
bool TrackPlaybackController::hasCallback() const
{
    return callback() != nullptr;
}

// 获取当前模型节点上的回放回调。
TrackAnimationCallback* TrackPlaybackController::callback() const
{
    if (!_planePat.valid()) return nullptr;
    return dynamic_cast<TrackAnimationCallback*>(_planePat->getUpdateCallback());
}
