#include <ros/ros.h>

#include "filters/filter_chain.h"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

namespace gm = ::grid_map::grid_map_pcl;

namespace gridmap_server {

/*!
 * Loads an image and saves it as layer 'elevation' of a grid map.
 * The grid map is published and can be viewed in Rviz.
 */
class PointCloudToGridmap {
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  PointCloudToGridmap(ros::NodeHandle &nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~PointCloudToGridmap();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  void pointCloudCallback(
      const boost::shared_ptr<const sensor_msgs::PointCloud2> &msg);

private:
  //! ROS nodehandle.
  ros::NodeHandle &nodeHandle_;

  //! Local grid map publisher.
  ros::Publisher localMapPublisher_;

  //! Global grid map publisher.
  ros::Publisher globalMapPublisher_;

  //! real-time Grid map data.
  grid_map::GridMap map_;

  //! global Grid map data.
  grid_map::GridMap localMap_;

  //! global Grid map data.
  grid_map::GridMap globalMap_;

  //! Image subscriber
  ros::Subscriber pointCloudSubscriber_;

  //! Name of the input point cloud topic.
  std::string pointCloudTopic_;

  //! Frame id of the input pointcloud.
  std::string pointCloudFrameId_;

  //! Name of the output local grid_map topic.
  std::string outputLocalMapTopic_;

  //! Name of the output global grid_map topic.
  std::string outputGlobalMapTopic_;

  //! Resolution of the grid map.
  double resolution_;

  //! Fix frame id of the grid map.
  std::string mapFrameId_;

  tf::TransformListener listener_;

  // 无人机当前高度
  double uavHight_;

  // 无人机当前位置坐标
  double uavX_;
  double uavY_;

  grid_map::GridMapPclLoader gridMapPclLoader_;

  //! Filter chain.
  filters::FilterChain<grid_map::GridMap> filterChain_;

  //! Filter chain parameters name.
  std::string filterChainParametersName_;
};

PointCloudToGridmap::PointCloudToGridmap(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle), map_(grid_map::GridMap({"elevation"})),
      globalMap_(grid_map::GridMap({"elevation"})),
      filterChain_("grid_map::GridMap") {
  readParameters();
  gm::setVerbosityLevelToDebugIfFlagSet(nodeHandle_);

  map_.setBasicLayers({"elevation"});
  map_.setFrameId(mapFrameId_);

  localMap_.setBasicLayers({"elevation"});
  localMap_.setFrameId(mapFrameId_);

  globalMap_.setGeometry(grid_map::Length(1.0, 1.0), resolution_, grid_map::Position(0.0, 0.0)); // bufferSize(6, 6)
  globalMap_.setBasicLayers({"elevation"});
  globalMap_.setFrameId(mapFrameId_);

  pointCloudSubscriber_ = nodeHandle_.subscribe(
      pointCloudTopic_, 10, &PointCloudToGridmap::pointCloudCallback, this);
  localMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(
      outputLocalMapTopic_, 1, true);
  globalMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(
      outputGlobalMapTopic_, 1, true);

  // Setup filter chain.
  if (!filterChain_.configure(filterChainParametersName_, nodeHandle)) {
    ROS_ERROR("Could not configure the filter chain!");
    return;
  }
}

PointCloudToGridmap::~PointCloudToGridmap() {}

bool PointCloudToGridmap::readParameters() {
  nodeHandle_.param<std::string>("pointcloud_topic", pointCloudTopic_,
                                 std::string("/kinect/depth/points"));
  nodeHandle_.param<double>("resolution", resolution_, 0.3);
  nodeHandle_.param<std::string>("map_frame", mapFrameId_, std::string("map"));
  nodeHandle_.param<std::string>("pointcloud_frame", pointCloudFrameId_, std::string("camera_link"));
  nodeHandle_.param<std::string>(
      "output_local_map_topic", outputLocalMapTopic_,
      std::string("/pointcloud_to_gridmap/local_grid_map"));
  nodeHandle_.param<std::string>(
      "output_global_map_topic", outputGlobalMapTopic_,
      std::string("/pointcloud_to_gridmap/global_grid_map"));
  nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_,
                    std::string("grid_map_filters"));
  return true;
}

void PointCloudToGridmap::pointCloudCallback(
    const boost::shared_ptr<const sensor_msgs::PointCloud2> &msg) {
  sensor_msgs::PointCloud2 pointcloud_sub, pointcloud;
  pointcloud_sub = *msg;
  pointcloud_sub.header.frame_id = pointCloudFrameId_;

  //tf::StampedTransform transform;
  //try {
    //listener_.waitForTransform(mapFrameId_, pointCloudFrameId_, msg->header.stamp,
     //                          ros::Duration(5.0));
   // listener_.lookupTransform(mapFrameId_, pointCloudFrameId_, msg->header.stamp,
   //                           transform);

 // } catch (tf::TransformException &ex) {
  //  ROS_ERROR("fram transform error: %s", ex.what());
 //   return;
 // }

  //pcl_ros::transformPointCloud(mapFrameId_, pointcloud_sub, pointcloud, listener_);

  gridMapPclLoader_.loadParameters(gm::getParameterPath());
  gridMapPclLoader_.loadCloudFromROSMsg(pointcloud_sub);
  //gridMapPclLoader_.loadCloudFromROSMsg(pointcloud);

  gm::processPointcloud(&gridMapPclLoader_, nodeHandle_);

  localMap_ = gridMapPclLoader_.getGridMap();
  localMap_.setTimestamp(msg->header.stamp.toNSec());
  localMap_.setFrameId(pointCloudFrameId_);



  // grid_map::GridMap unifiedResMap_;
  // grid_map::GridMapCvProcessing::changeResolution(map_, unifiedResMap_,
  // resolution_);

 /*!
   * Adds data from an other grid map to this grid map
   * @param other the grid map to take data from.
   * @param extendMap if true the grid map is resized that the other map fits within.
   * @param overwriteData if true the new data replaces the old values, else only invalid cells are updated.
   * @param copyAllLayer if true all layers are used to add data.
   * @param layers the layers that are copied if not all layers are used.
   * @return true if successful.
   */
  // globalMap_.addDataFrom(unifiedResMap_, true, true, true);

  //std::vector<std::string> stringVector;
 // stringVector.push_back("elevation");
  //globalMap_.addDataFrom(localMap_, true, true, false, stringVector);
 // globalMap_.setTimestamp(msg->header.stamp.toNSec());

    // Apply filter chain.
  if (!filterChain_.update(localMap_, map_)) {
    ROS_ERROR("Could not update the grid map filter chain!");
    return;
  }
  map_.setTimestamp(msg->header.stamp.toNSec());
  map_.setFrameId(pointCloudFrameId_);

  // Publish as local grid map.
  //grid_map_msgs::GridMap localMapMessage;
  //grid_map::GridMapRosConverter::toMessage(localMap_, localMapMessage);
  //localMapPublisher_.publish(localMapMessage);

  // Publish as global grid map.
  grid_map_msgs::GridMap globalMapMessage;
  grid_map::GridMapRosConverter::toMessage(map_, globalMapMessage);
  globalMapPublisher_.publish(globalMapMessage);
}

} /* namespace */
