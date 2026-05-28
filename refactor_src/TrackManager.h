#ifndef REFACTOR_TRACKMANAGER_H
#define REFACTOR_TRACKMANAGER_H

#include "TrackData.h"

#include <QMap>
#include <QStringList>

// 管理多条航迹对象及当前航迹标识。
class TrackManager {
public:
    QString makeUniqueTrackId(const QString& csvTrackId, const QString& fileName) const;

    void addTrack(const TrackObject& track);
    bool contains(const QString& trackId) const;
    bool selectTrack(const QString& trackId);
    void setCurrentTrackId(const QString& trackId);
    bool removeTrack(const QString& trackId);

    TrackObject* currentTrack();
    const TrackObject* currentTrack() const;
    TrackObject* track(const QString& trackId);
    const TrackObject* track(const QString& trackId) const;

    QString currentTrackId() const { return _currentTrackId; }
    QStringList trackIds() const { return _allTracks.keys(); }
    bool isEmpty() const { return _allTracks.isEmpty(); }

private:
    QMap<QString, TrackObject> _allTracks;
    QString _currentTrackId;
};

#endif // REFACTOR_TRACKMANAGER_H
