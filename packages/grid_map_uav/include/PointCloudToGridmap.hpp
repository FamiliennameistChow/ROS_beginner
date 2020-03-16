#include <ros/ros.h>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"
#include "drone_flight_modes.hpp"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

namespace gm = ::grid_map::grid_map_pcl;

namespace grid_map_uav {

/*!
 * Loads an image and saves it as layer 'elevation' of a grid map.
 * The grid map is published and can be viewed in Rviz.
 */
class PointCloudToGridmap
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  PointCloudToGridmap(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~PointCloudToGridmap();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  void pointCloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg);

 private:

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Grid map publisher.
  ros::Publisher gridMapPublisher_;

  //!real-time Grid map data.
  grid_map::GridMap map_;

  //!global Grid map data.
  grid_map::GridMap globalMap_;

  //! Image subscriber
  ros::Subscriber pointCloudSubscriber_;

  //! Name of the grid map topic.
  std::string pointCloudTopic_;

  //! Resolution of the grid map.
  double resolution_;

  //! Range of the height values.
  double minHeight_;
  double maxHeight_;

  double heightRange_;

  //! Frame id of the grid map.
  std::string mapFrameId_;

  AeroDrone myDrone_;
  // 无人机当前高度
  double uavHight_;
  
  // 无人机当前位置坐标
  double uavX_;
  double uavY_;

  grid_map::GridMapPclLoader gridMapPclLoader_;
};


PointCloudToGridmap::PointCloudToGridmap(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(grid_map::GridMap({"elevation"})),
      globalMap_(grid_map::GridMap({"elevation"})),
      myDrone_(AeroDrone("NOCTRL"))
{
  readParameters();
  gm::setVerbosityLevelToDebugIfFlagSet(nodeHandle_);

  map_.setBasicLayers({"elevation"});
  map_.setFrameId(mapFrameId_);

  globalMap_.setGeometry(grid_map::Length(1.0, 1.0), resolution_, grid_map::Position(0.0, 0.0)); // bufferSize(6, 6)
  globalMap_.setBasicLayers({"elevation"});
  globalMap_.setFrameId(mapFrameId_);

  pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &PointCloudToGridmap::pointCloudCallback, this);
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
}

PointCloudToGridmap::~PointCloudToGridmap()
{
}

bool PointCloudToGridmap::readParameters()
{
  nodeHandle_.param<std::string>("pointcloud_topic", pointCloudTopic_, std::string("/kinect/depth/points"));
  nodeHandle_.param<double>("resolution", resolution_, 0.3);
  nodeHandle_.param<double>("min_height", minHeight_, 0.0);
  nodeHandle_.param<double>("max_height", maxHeight_, 10.0);
  nodeHandle_.param<std::string>("map_frame_id", mapFrameId_, std::string("map"));
  heightRange_ = maxHeight_ - minHeight_;
  return true;
}

void PointCloudToGridmap::pointCloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg)
{
    gridMapPclLoader_.loadParameters(gm::getParameterPath());
    gridMapPclLoader_.loadCloudFromROSMsg(msg);

    uavHight_ = myDrone_.localPosition().pose.position.z;
    uavX_ = myDrone_.localPosition().pose.position.x;
    uavY_ = myDrone_.localPosition().pose.position.y;

    gm::processPointcloud(&gridMapPclLoader_, nodeHandle_);

    map_ = gridMapPclLoader_.getGridMap();
    map_.setPosition(grid_map::Position(uavX_, -(uavY_)));

    grid_map::GridMap unifiedResMap_;
    grid_map::GridMapCvProcessing::changeResolution(map_, unifiedResMap_, resolution_);

    globalMap_.addDataFrom(unifiedResMap_, true, true, true);
    globalMap_.setTimestamp(ros::Time::now().toNSec());

    // Publish as grid map.
    grid_map_msgs::GridMap mapMessage;
    grid_map::GridMapRosConverter::toMessage(globalMap_, mapMessage);
    gridMapPublisher_.publish(mapMessage);
}

} /* namespace */
