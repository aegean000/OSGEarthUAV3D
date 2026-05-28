#ifndef REFACTOR_TRACKNODEBUILDER_H
#define REFACTOR_TRACKNODEBUILDER_H

#include "TrackData.h"

#include <QString>
#include <osg/Group>
#include <osgEarth/MapNode>

// 根据航迹数据创建三维显示节点，包括航迹线、采样点、投影线、参考面和无人机模型。
class TrackNodeBuilder {
public:
    TrackObject buildTrackObject(const QString& trackId,
                                 const TrackLoadResult& loadResult,
                                 osg::Group* root,
                                 osgEarth::MapNode* mapNode,
                                 bool projectionVisible,
                                 const TrackRenderConfig& config = TrackRenderConfig()) const;

private:
    osg::ref_ptr<osg::Geode> createPathGeode(osg::Vec3Array* lineCoords,
                                             osg::Geometry** outPathGeom,
                                             const TrackRenderConfig& config) const;
    osg::ref_ptr<osg::Geode> createPointGeode(const QString& trackId,
                                              osg::Vec3Array* lineCoords) const;
    osg::ref_ptr<osg::Geode> createBasePlaneGeode(const QString& trackId,
                                                  const std::vector<TrackPoint>& points,
                                                  osgEarth::MapNode* mapNode) const;
    osg::ref_ptr<osg::Geode> createProjectionLineGeode(const QString& trackId,
                                                       const std::vector<TrackPoint>& points,
                                                       osg::Geometry** outGeom,
                                                       osg::Vec3Array** outVertices,
                                                       bool projectionVisible) const;
    osg::ref_ptr<osg::PositionAttitudeTransform> createPlanePat(const std::vector<TrackPoint>& points,
                                                                const TrackRenderConfig& config) const;
};

#endif // REFACTOR_TRACKNODEBUILDER_H
