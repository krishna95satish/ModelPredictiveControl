#ifndef ROS_SRC_TOOLS_TRAJECTORY_VISUALIZATION_TRAJECTORY_DISPLAY_H
#define ROS_SRC_TOOLS_TRAJECTORY_VISUALIZATION_TRAJECTORY_DISPLAY_H

#include <rviz_common/display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>

#include <custom_messages/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace trajectory_visualization {

using MarkerCommon = rviz_default_plugins::displays::MarkerCommon;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Trajectory = custom_messages::msg::Trajectory;
using TrajectoryPoint = custom_messages::msg::TrajectoryPoint;

void quaternionFromEuler(float roll, float pitch, float yaw,
                         geometry_msgs::msg::Quaternion &quaternion);

/*
 * convert trajectory msg into a LineStrip marker
 */
Marker createLineStripMarker( const trajectory_visualization::Trajectory & trajectory, QColor & color, float alpha,
                              float scale );

/*
 * generate text marker message at a trajectory point
 */
Marker createTextMarker( const TrajectoryPoint & point, QColor & color, float textAlpha, float textScale );

class TrajectoryDisplay
    : public rviz_common::RosTopicDisplay<custom_messages::msg::Trajectory> {
  Q_OBJECT

public:
  TrajectoryDisplay();
  void onInitialize() override;
  void load(const rviz_common::Config &config) override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

private Q_SLOTS:
  void updateProperty();

private:

  // Convert boxes into markers, push them to the display queue
  void processMessage(Trajectory::ConstSharedPtr msg) override;

  std::unique_ptr<MarkerCommon> mMarkerCommon;
  Trajectory::ConstSharedPtr mMsgCache{};
  rviz_common::properties::ColorProperty *mColorProperty;
  rviz_common::properties::FloatProperty *mAlphaProperty;
  rviz_common::properties::FloatProperty *mScaleProperty;
  rviz_common::properties::FloatProperty *mTextAlphaProperty;
  rviz_common::properties::FloatProperty *mTextScaleProperty;
};
} // namespace trajectory_visualization
#endif // ROS_SRC_TOOLS_TRAJECTORY_VISUALIZATION_TRAJECTORY_DISPLAY_H
