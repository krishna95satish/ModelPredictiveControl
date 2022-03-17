#include "trajectories_display.h"

#include <math.h>

trajectory_visualization::TrajectoriesDisplay::TrajectoriesDisplay()
    : rviz_common::RosTopicDisplay<custom_messages::msg::Trajectories>()
    , mMarkerCommon( std::make_unique<MarkerCommon>( this ) )
    , mTrajectoriesMarkerArray( std::make_shared<MarkerArray>() )
{
    mColorProperty = new rviz_common::properties::ColorProperty(
        "Color", QColor( 0, 255, 0 ), "Color to draw the arrow.", this, SLOT( updateProperty() ) );

    mAlphaProperty = new rviz_common::properties::FloatProperty(
        "Alpha", 1.0, "Amount of transparency to apply to the arrow.", this, SLOT( updateProperty() ) );
    mAlphaProperty->setMin( 0 );
    mAlphaProperty->setMax( 1 );

    mScaleProperty =
        new rviz_common::properties::FloatProperty( "Scale", 1, "Scale of the arrow.", this, SLOT( updateProperty() ) );

    mTextScaleProperty = new rviz_common::properties::FloatProperty(
        "Text Scale", 1, "Scale of the text showing velocity.", this, SLOT( updateProperty() ) );

    mTextAlphaProperty = new rviz_common::properties::FloatProperty(
        "Text Alpha", 1.0, "Amount of transparency to apply to the velocity text.", this, SLOT( updateProperty() ) );
    mTextAlphaProperty->setMin( 0 );
    mTextAlphaProperty->setMax( 1 );
}

void
trajectory_visualization::TrajectoriesDisplay::onInitialize()
{
    RTDClass::onInitialize();
    mMarkerCommon->initialize( context_, scene_node_ );

    topic_property_->setValue( "trajectory" );
    topic_property_->setDescription( "Trajectory topic to subscribe to." );
}

void
trajectory_visualization::TrajectoriesDisplay::load( const rviz_common::Config & config )
{
    Display::load( config );
    mMarkerCommon->load( config );
}

void
trajectory_visualization::TrajectoriesDisplay::update( float wall_dt, float ros_dt )
{
    mMarkerCommon->update( wall_dt, ros_dt );
}

void
trajectory_visualization::TrajectoriesDisplay::reset()
{
    RosTopicDisplay::reset();
    mMarkerCommon->clearMarkers();
}

void
trajectory_visualization::TrajectoriesDisplay::updateProperty()
{
    if( mMsgCache != nullptr ) {
        processMessage( mMsgCache );
    }
}

void
trajectory_visualization::TrajectoriesDisplay::processMessage( const Trajectories::ConstSharedPtr msg )
{
    mMsgCache = msg;
    mMarkerCommon->clearMarkers();
    auto count = 0;
    const auto update = [this]( const auto & marker ) -> void { mMarkerCommon->addMessage( marker ); };

    auto color = mColorProperty->getColor();
    auto alpha = mAlphaProperty->getFloat();
    auto scale = mScaleProperty->getFloat();
    auto textAlpha = mTextAlphaProperty->getFloat();
    auto textScale = mTextScaleProperty->getFloat();

    mTrajectoriesMarkerArray->markers.clear();
    //auto costMarkerArray = std::make_shared<MarkerArray>();
    mTrajectoriesMarkerArray->markers.reserve( msg->trajectories.size() );
    //costMarkerArray->markers.reserve( msg->trajectories.size() );

    static constexpr char ns[] = "trajectory_cost";
    for( const auto & trajectory : msg->trajectories ) {
        auto trajMarker = createLineStripMarker( trajectory, color, alpha, scale );
        trajMarker.header = msg->header;
        trajMarker.id = ++count;
        trajMarker.text = std::to_string( trajectory.cost );
        mTrajectoriesMarkerArray->markers.emplace_back( std::move( trajMarker ) );

        //auto costMarker = createTextMarker( trajectory.points.back(), color, textAlpha, textScale );
        //costMarker.header = msg->header;
        //costMarker.ns = ns;
        //costMarker.text = std::to_string( trajectory.cost );
        //costMarkerArray->markers.emplace_back( std::move( costMarker ) );
    }
    update( mTrajectoriesMarkerArray );
    //update( costMarkerArray );
}

// Export the plugin
#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS( trajectory_visualization::TrajectoriesDisplay, rviz_common::Display )
