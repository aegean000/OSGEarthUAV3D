#ifndef REFACTOR_SYSTEMCONFIG_H
#define REFACTOR_SYSTEMCONFIG_H

namespace SystemConfig {

namespace Resource {
constexpr const char* MapFile = "mymap.earth";                                  // 地图配置文件
constexpr const char* DroneModelFile = "D:/OSG_QT_Project/models/drone/Drone.osgb"; // 无人机模型
}

namespace TianDiTu {
constexpr const char* Key = "80f4e2cd107652a761bb8683f5c8f93e"; // 天地图密钥
constexpr const char* ImageType = "img_w";                     // 影像图层类型
constexpr const char* LabelType = "cia_w";                     // 注记图层类型
constexpr const char* ImageLayerName = "TianDiTu_Image";       // 影像图层名
constexpr const char* LabelLayerName = "TianDiTu_Label";       // 注记图层名
constexpr const char* UrlPrefix = "http://t0.tianditu.gov.cn/DataServer?"; // 服务地址
constexpr float LayerOpacity = 1.0f;                            // 图层透明度
}

namespace Render {
constexpr float PlaneScale = 18.0f;              // 模型缩放比例
constexpr float PathLineWidth = 4.0f;            // 航迹线宽
constexpr float TrackPointSize = 8.0f;           // 航迹点大小
constexpr float ProjectionLineWidth = 3.0f;      // 投影线宽
constexpr double BasePlaneHeight = 500.0;        // 参考面高度
constexpr double BasePlaneMarginLon = 0.002;     // 参考面经度边距
constexpr double BasePlaneMarginLat = 0.002;     // 参考面纬度边距
constexpr double ModelHeadingOffsetDeg = 0.0;    // 模型航向修正角
constexpr double ModelPitchOffsetDeg = 0.0;      // 模型俯仰修正角
constexpr double ModelRollOffsetDeg = 30.0;       // 模型横滚修正角
}

namespace Playback {
constexpr int SliderTimeScale = 10;        // 进度条时间倍率
constexpr int UiRefreshIntervalMs = 100;   // 界面刷新间隔
}

namespace View {
constexpr double FollowRange = 1000.0;             // 跟随距离
constexpr double FollowPitch = -45.0;              // 跟随俯仰角
constexpr double FollowDuration = 0.15;            // 跟随动画时长
constexpr int FollowCallbackGuardMs = 250;         // 跟随保护时间
constexpr int UserInteractionGuardMs = 350;        // 用户操作保护时间
constexpr double RangeChangeThreshold = 1.0;       // 距离变化阈值
}

} // namespace SystemConfig

#endif // REFACTOR_SYSTEMCONFIG_H