#include "trajectory_display.h"

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <math.h>

/**
 * Converts euler roll, pitch, yaw to quaternion
 * @param[out] quaternion
 */
void trajectory_visualization::quaternionFromEuler(
    float roll, float pitch, float yaw,
    geometry_msgs::msg::Quaternion &quaternion) {

  tf2::Quaternion quatTf;
  quatTf.setRPY(roll, pitch, yaw);
  quaternion.w = quatTf.getW();
  quaternion.x = quatTf.getX();
  quaternion.y = quatTf.getY();
  quaternion.z = quatTf.getZ();
}

visualization_msgs::msg::Marker
trajectory_visualization::createLineStripMarker( const trajectory_visualization::Trajectory & trajectory,
                                                 QColor & color, float alpha, float scale )
{
    static constexpr char ns[] = "trajectory_line_trip";

    auto marker = Marker();
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = ns;
    marker.pose.orientation.w = 1.0;

    // scale.x represents the line width
    marker.scale.x = 0.1f * scale;
    marker.color.a = alpha;
    marker.color.r = static_cast<float>( color.redF() );
    marker.color.g = static_cast<float>( color.greenF() );
    marker.color.b = static_cast<float>( color.blueF() );

    marker.points.reserve( trajectory.points.size() );
    for( const auto & point : trajectory.points ) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0;
        marker.points.push_back( p );
    }

    return marker;
}

visualization_msgs::msg::Marker
trajectory_visualization::createTextMarker( const trajectory_visualization::TrajectoryPoint & point, QColor & color,
                                            float textAlpha, float textScale )
{
    auto marker = Marker();
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = static_cast<float>( point.x );
    marker.pose.position.y = static_cast<float>( point.y );
    marker.pose.position.z = 0.1;
    marker.scale.z = 0.2f * textScale;
    marker.color.a = textAlpha;
    marker.color.r = static_cast<float>( color.redF() );
    marker.color.g = static_cast<float>( color.greenF() );
    marker.color.b = static_cast<float>( color.blueF() );

    return marker;
}

trajectory_visualization::TrajectoryDisplay::TrajectoryDisplay()
    : rviz_common::RosTopicDisplay<custom_messages::msg::Trajectory>(),
      mMarkerCommon(std::make_unique<MarkerCommon>(this)) {
  mColorProperty = new rviz_common::properties::ColorProperty(
      "Color", QColor(0, 255, 0), "Color to draw the arrow.", this,
      SLOT(updateProperty()));

  mAlphaProperty = new rviz_common::properties::FloatProperty(
      "Alpha", 1.0, "Amount of transparency to apply to the arrow.", this,
      SLOT(updateProperty()));
  mAlphaProperty->setMin(0);
  mAlphaProperty->setMax(1);

  mScaleProperty = new rviz_common::properties::FloatProperty(
      "Scale", 1, "Scale of the arrow.", this, SLOT(updateProperty()));

  mTextScaleProperty = new rviz_common::properties::FloatProperty(
      "Text Scale", 1, "Scale of the text showing velocity.", this,
      SLOT(updateProperty()));

  mTextAlphaProperty = new rviz_common::properties::FloatProperty(
      "Text Alpha", 1.0,
      "Amount of transparency to apply to the velocity text.", this,
      SLOT(updateProperty()));
  mTextAlphaProperty->setMin(0);
  mTextAlphaProperty->setMax(1);
}

void trajectory_visualization::TrajectoryDisplay::onInitialize() {
  RTDClass::onInitialize();
  mMarkerCommon->initialize(context_, scene_node_);

  topic_property_->setValue("trajectory");
  topic_property_->setDescription("Trajectory topic to subscribe to.");
}

void trajectory_visualization::TrajectoryDisplay::load(
    const rviz_common::Config &config) {
  Display::load(config);
  mMarkerCommon->load(config);
}

void trajectory_visualization::TrajectoryDisplay::update(float wall_dt,
                                                         float ros_dt) {
  mMarkerCommon->update(wall_dt, ros_dt);
}

void trajectory_visualization::TrajectoryDisplay::reset() {
  RosTopicDisplay::reset();
  mMarkerCommon->clearMarkers();
}

void trajectory_visualization::TrajectoryDisplay::updateProperty() {
  if (mMsgCache != nullptr) {
    processMessage(mMsgCache);
  }
}

void trajectory_visualization::TrajectoryDisplay::processMessage(
    const Trajectory::ConstSharedPtr msg) {
  mMsgCache = msg;
  mMarkerCommon->clearMarkers();
  auto count = 0;
  const auto update = [&count, this,
                       header = msg->header](auto marker) -> void {
    marker->header = header;
    marker->id = ++count;
    mMarkerCommon->addMessage(marker);
  };

  auto color = mColorProperty->getColor();
  auto alpha = mAlphaProperty->getFloat();
  auto scale = mScaleProperty->getFloat();
  auto textAlpha = mTextAlphaProperty->getFloat();
  auto textScale = mTextScaleProperty->getFloat();

  const auto trajMarker = std::make_shared<Marker>( createLineStripMarker( *msg, color, alpha, scale ) );
  update( trajMarker );

  static constexpr char ns[] = "trajectory_velocity";
  static constexpr char mps[] = " m/s";
  for( const auto & point : msg->points ) {
      {
          const auto velMarker = std::make_shared<Marker>( createTextMarker( point, color, textAlpha, textScale ) );
          velMarker->ns = ns;
          velMarker->text = std::to_string( point.velocity ) + mps;
          update( velMarker );
      }
  }
}

// Export the plugin
#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(trajectory_visualization::TrajectoryDisplay,
                       rviz_common::Display)
