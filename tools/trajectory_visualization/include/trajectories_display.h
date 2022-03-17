#ifndef ROS_SRC_TOOLS_TRAJECTORIES_VISUALIZATION_TRAJECTORY_DISPLAY_H
#define ROS_SRC_TOOLS_TRAJECTORIES_VISUALIZATION_TRAJECTORY_DISPLAY_H

#include "trajectory_display.h"

#include <custom_messages/msg/trajectories.hpp>

namespace trajectory_visualization {

class TrajectoriesDisplay
    : public rviz_common::RosTopicDisplay<custom_messages::msg::Trajectories> {
  Q_OBJECT

public:
  TrajectoriesDisplay();
  void onInitialize() override;
  void load(const rviz_common::Config &config) override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

private Q_SLOTS:
  void updateProperty();

private:
  using Trajectories = custom_messages::msg::Trajectories;

  // Convert boxes into markers, push them to the display queue
  void processMessage(Trajectories::ConstSharedPtr msg) override;

  std::unique_ptr<MarkerCommon> mMarkerCommon;
  std::shared_ptr<MarkerArray> mTrajectoriesMarkerArray;
  Trajectories::ConstSharedPtr mMsgCache{};
  rviz_common::properties::ColorProperty *mColorProperty;
  rviz_common::properties::FloatProperty *mAlphaProperty;
  rviz_common::properties::FloatProperty *mScaleProperty;
  rviz_common::properties::FloatProperty *mTextAlphaProperty;
  rviz_common::properties::FloatProperty *mTextScaleProperty;
};
} // namespace trajectory_visualization
#endif // ROS_SRC_TOOLS_TRAJECTORIES_VISUALIZATION_TRAJECTORY_DISPLAY_H
