#include "TrackManager.h"

#include <QFileInfo>

// 根据 CSV 中的航迹编号或文件名生成不重复的航迹 ID。
QString TrackManager::makeUniqueTrackId(const QString& csvTrackId, const QString& fileName) const
{
    QString trackId = csvTrackId.isEmpty() ? QFileInfo(fileName).fileName() : csvTrackId;
    int counter = 1;
    while (_allTracks.contains(trackId)) {
        trackId = QFileInfo(fileName).fileName() + QString("_%1").arg(counter++);
    }
    return trackId;
}

// 添加航迹并设为当前航迹。
void TrackManager::addTrack(const TrackObject& track)
{
    _allTracks.insert(track.id, track);
    _currentTrackId = track.id;
}

// 判断指定航迹是否存在。
bool TrackManager::contains(const QString& trackId) const
{
    return _allTracks.contains(trackId);
}

// 选择指定航迹作为当前航迹。
bool TrackManager::selectTrack(const QString& trackId)
{
    if (!_allTracks.contains(trackId)) return false;
    _currentTrackId = trackId;
    return true;
}

// 设置当前航迹 ID。
void TrackManager::setCurrentTrackId(const QString& trackId)
{
    _currentTrackId = trackId;
}

// 从数据容器中移除指定航迹。
bool TrackManager::removeTrack(const QString& trackId)
{
    if (!_allTracks.contains(trackId)) return false;

    _allTracks.remove(trackId);

    if (_currentTrackId == trackId) {
        // 当前航迹的 UI 选择顺序由 MainWindow 管理。
        _currentTrackId.clear();
    }

    return true;
}

// 获取当前航迹对象。
TrackObject* TrackManager::currentTrack()
{
    if (_currentTrackId.isEmpty() || !_allTracks.contains(_currentTrackId)) return nullptr;
    return &_allTracks[_currentTrackId];
}

// 获取当前航迹对象。
const TrackObject* TrackManager::currentTrack() const
{
    if (_currentTrackId.isEmpty()) return nullptr;
    auto it = _allTracks.constFind(_currentTrackId);
    if (it == _allTracks.constEnd()) return nullptr;
    return &it.value();
}

// 按 ID 获取航迹对象。
TrackObject* TrackManager::track(const QString& trackId)
{
    if (!_allTracks.contains(trackId)) return nullptr;
    return &_allTracks[trackId];
}

// 按 ID 获取航迹对象。
const TrackObject* TrackManager::track(const QString& trackId) const
{
    auto it = _allTracks.constFind(trackId);
    if (it == _allTracks.constEnd()) return nullptr;
    return &it.value();
}
