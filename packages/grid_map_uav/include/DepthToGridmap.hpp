/*
 * ImageToGridmapDemo.hpp
 *
 *  Created on: May 4, 2015
 *      Author: Martin Wermelinger
 *	 Institute: ETH Zurich, ANYbotics
 *
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include "opencv2/core/core.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include "drone_flight_modes.hpp"

namespace grid_map_uav {

/*!
 * Loads an image and saves it as layer 'elevation' of a grid map.
 * The grid map is published and can be viewed in Rviz.
 */
class DepthToGridmap
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  DepthToGridmap(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~DepthToGridmap();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  void depthCallback(const sensor_msgs::Image& msg);
  // void uavLocalPosCB(const geometry_msgs::PoseStamped::ConstPtr &msg);

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
  ros::Subscriber imageSubscriber_;

  //! Name of the grid map topic.
  std::string imageTopic_;

  //! Length of the grid map in x direction.
  double mapLengthX_;

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
};


DepthToGridmap::DepthToGridmap(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(grid_map::GridMap({"elevation"})),
      globalMap_(grid_map::GridMap({"elevation"})),
      myDrone_(AeroDrone(0, "NOCTRL"))
{
  readParameters();

  map_.setBasicLayers({"elevation"});
  map_.setFrameId(mapFrameId_);

  globalMap_.setGeometry(grid_map::Length(1.0, 1.0), resolution_, grid_map::Position(0.0, 0.0)); // bufferSize(6, 6)
  globalMap_.setBasicLayers({"elevation"});
  globalMap_.setFrameId(mapFrameId_);

  imageSubscriber_ = nodeHandle_.subscribe(imageTopic_, 1, &DepthToGridmap::depthCallback, this);
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
}

DepthToGridmap::~DepthToGridmap()
{
}

bool DepthToGridmap::readParameters()
{
  nodeHandle_.param<std::string>("image_topic", imageTopic_, std::string("/kinect/depth/image_raw"));
  nodeHandle_.param<double>("resolution", resolution_, 0.3);
  nodeHandle_.param<double>("min_height", minHeight_, 0.0);
  nodeHandle_.param<double>("max_height", maxHeight_, 10.0);
  nodeHandle_.param<std::string>("map_frame_id", mapFrameId_, std::string("map"));
  heightRange_ = maxHeight_ - minHeight_;
  return true;
}

void DepthToGridmap::depthCallback(const sensor_msgs::Image& msg)
{
    uavHight_ = myDrone_.localPosition().pose.position.z;
    uavX_ = myDrone_.localPosition().pose.position.x;
    uavY_ = myDrone_.localPosition().pose.position.y;

    double initResolution = uavHight_/122.0;
    grid_map::GridMapRosConverter::initializeFromImage(msg, initResolution, map_, grid_map::Position(-(uavX_), -(uavY_)));
    map_.setTimestamp(ros::Time::now().toNSec());
    // ROS_INFO("Initialized map with size %f x %f m (%i x %i cells) with resolution %f.", map_.getLength().x(),
    //          map_.getLength().y(), map_.getSize()(0), map_.getSize()(1), map_.getResolution());

  cv_bridge::CvImageConstPtr cvImage;
  try {
    // TODO Use `toCvShared()`?
    cvImage = cv_bridge::toCvCopy(msg, msg.encoding);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  double minv, maxv;
  cv::Mat image;
  cv::minMaxLoc(cvImage->image, &minv, &maxv);
  cvImage->image.convertTo(image, CV_16UC1, -65535 / heightRange_, maxv * 65535 / heightRange_);
  // grid_map::GridMapRosConverter::addLayerFromImage(msg, "elevation", map_, minHeight_, maxHeight_);
  // grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "color", map_);
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(image, "elevation", map_, minHeight_, maxHeight_);
  
  //!real-time Grid map data with unified resolution.
  grid_map::GridMap unifiedResMap_;
  grid_map::GridMapCvProcessing::changeResolution(map_, unifiedResMap_, resolution_);
  // ROS_INFO("unified map with size %f x %f m (%i x %i cells) with resolution %f.", unifiedResMap_.getLength().x(),
  //            unifiedResMap_.getLength().y(), unifiedResMap_.getSize()(0), unifiedResMap_.getSize()(1), unifiedResMap_.getResolution());
  
  // add real-time map data to global map
  // std::vector<std::string> stringVector;
  // stringVector.push_back("elevation");
  globalMap_.addDataFrom(unifiedResMap_, true, true, true);
  globalMap_.setTimestamp(ros::Time::now().toNSec());

  // Publish as grid map.
  grid_map_msgs::GridMap mapMessage;
  grid_map::GridMapRosConverter::toMessage(globalMap_, mapMessage);
  gridMapPublisher_.publish(mapMessage);
}

} /* namespace */
