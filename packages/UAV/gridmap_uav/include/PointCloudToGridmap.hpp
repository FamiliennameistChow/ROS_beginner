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
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <geometry_msgs/PointStamped.h>

namespace gm = ::grid_map::grid_map_pcl;
using namespace cv;
using namespace std;

namespace gridmap_uav {

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

  //! landing site publisher.
  ros::Publisher landingSitePublisher_;

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

  // file name of pcl_grid_map_extraction parameters
  std::string parameters_file_;

  //! Resolution of the grid map.
  double resolution_;

  // safe distance from UAV landing site to obstacles
  double safe_distance_;

  // size of UAV radius
  double UAV_radius_;

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

  int OtsuThreshold(Mat srcImage);
};

PointCloudToGridmap::PointCloudToGridmap(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle), map_(grid_map::GridMap({"elevation"})),
      globalMap_(grid_map::GridMap({"elevation"})),
      filterChain_("grid_map::GridMap") {
  readParameters();
  gm::setVerbosityLevelToDebugIfFlagSet(nodeHandle_);

  map_.setBasicLayers({"elevation"});
  map_.setFrameId(mapFrameId_);

  localMap_.setFrameId(mapFrameId_);

  globalMap_.setGeometry(grid_map::Length(1.0, 1.0), resolution_,
                         grid_map::Position(0.0, 0.0)); // bufferSize(6, 6)
  globalMap_.setBasicLayers({"elevation"});
  globalMap_.setFrameId(mapFrameId_);

  pointCloudSubscriber_ = nodeHandle_.subscribe(
      pointCloudTopic_, 10, &PointCloudToGridmap::pointCloudCallback, this);
  localMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(
      outputLocalMapTopic_, 1, true);
  globalMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(
      outputGlobalMapTopic_, 1, true);
  landingSitePublisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped> ("/landing_site", 1);

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
  nodeHandle_.param<double>("safe_distance", safe_distance_, 0.5);
  nodeHandle_.param<double>("UAV_radius", UAV_radius_, 0.5);
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
  nodeHandle_.param<std::string>(
      "parameters_file", parameters_file_,
      std::string("pointcloud_to_gridmap.yaml"));
  return true;
}

void PointCloudToGridmap::pointCloudCallback(
    const boost::shared_ptr<const sensor_msgs::PointCloud2> &msg) {
  sensor_msgs::PointCloud2 pointcloud_sub, pointcloud;
  pointcloud_sub = *msg;
  pointcloud_sub.header.frame_id = pointCloudFrameId_;

  tf::StampedTransform transform;
  try {
    listener_.waitForTransform(mapFrameId_, pointCloudFrameId_, msg->header.stamp,
                               ros::Duration(5.0));
    listener_.lookupTransform(mapFrameId_, pointCloudFrameId_, msg->header.stamp,
                              transform);

  } catch (tf::TransformException &ex) {
    ROS_ERROR("fram transform error: %s", ex.what());
    return;
  }

  pcl_ros::transformPointCloud(mapFrameId_, pointcloud_sub, pointcloud, listener_);

  //gridMapPclLoader_.loadParameters(gm::getParameterPath());
  gridMapPclLoader_.loadParameters(ros::package::getPath("gridmap_uav") + "/config/" + parameters_file_);
  gridMapPclLoader_.loadCloudFromROSMsg(pointcloud);

  gm::processPointcloud(&gridMapPclLoader_, nodeHandle_);

  map_ = gridMapPclLoader_.getGridMap();

  // Apply filter chain.
  if (!filterChain_.update(map_, localMap_)) {
    ROS_ERROR("Could not update the grid map filter chain!");
    return;
  }
  localMap_.setFrameId(mapFrameId_);

  // grid_map::GridMap unifiedResMap_;
  // grid_map::GridMapCvProcessing::changeResolution(map_, unifiedResMap_,
  // resolution_);

  // globalMap_.addDataFrom(unifiedResMap_, true, true, true);
  globalMap_.addDataFrom(localMap_, true, true, true);
  globalMap_.setTimestamp(ros::Time::now().toNSec());

  // Publish as local grid map.
  grid_map_msgs::GridMap localMapMessage;
  grid_map::GridMapRosConverter::toMessage(localMap_, localMapMessage);
  localMapPublisher_.publish(localMapMessage);

  // Publish as global grid map.
  grid_map_msgs::GridMap globalMapMessage;
  grid_map::GridMapRosConverter::toMessage(globalMap_, globalMapMessage);
  globalMapPublisher_.publish(globalMapMessage);


  // 将traversability层转成图片并处理
  cv_bridge::CvImage local_image;
  grid_map::GridMapRosConverter::toCvImage(localMap_,"traversability", sensor_msgs::image_encodings::MONO8, local_image);
  namedWindow("local image", CV_WINDOW_NORMAL);
  imshow("local image", local_image.image);

  cv_bridge::CvImage global_image;
  grid_map::GridMapRosConverter::toCvImage(globalMap_,"traversability", sensor_msgs::image_encodings::MONO8, global_image);
  namedWindow("global image", CV_WINDOW_NORMAL);
  imshow("global image", global_image.image);
  
  Mat risk_map = local_image.image;
  Mat binary_risk, dilated;
  int threshold = OtsuThreshold(risk_map);
  cv::threshold(risk_map, binary_risk, threshold, 255, THRESH_BINARY);
  namedWindow("binary risk", CV_WINDOW_NORMAL);
	imshow("binary risk", binary_risk);
  int kernel_size = (ceil(safe_distance_ / resolution_))*2 +1;
  Mat kernel = getStructuringElement(MORPH_RECT, Size(kernel_size, kernel_size));
	dilate(binary_risk, dilated, kernel);
  namedWindow("dilated risk", CV_WINDOW_NORMAL);
	imshow("dilated risk", dilated);

  vector<vector<Point>> contours;
	findContours(dilated, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  Mat drawing = Mat::zeros(binary_risk.size(), CV_8U);
	drawContours(drawing, contours, -1, 255, -1);
  namedWindow("drawing", CV_WINDOW_NORMAL);
	imshow("drawing", drawing);

	int kernel2_size = (ceil(UAV_radius_ / resolution_))*2 +1;
  int UAV_size = kernel2_size;
	Mat kernel2 = Mat::ones(kernel2_size, kernel2_size, CV_8U);
  Mat statistical_risk;
	filter2D(drawing, statistical_risk, CV_32F, kernel2);
	statistical_risk = statistical_risk/255;
  namedWindow("statistical risk", CV_WINDOW_NORMAL);
	imshow("statistical risk", statistical_risk);

	int height = statistical_risk.rows;
	int width = statistical_risk.cols;
	//int channels = statistical_risk.channels();
	// for row in range(height):  # 遍历高
	// 	for col in range(width):  # 遍历宽
	// 		for c in range(channels):  # 遍历通道
	// 		print('{},  '.format(statistical_risk[row, col]), end = "")
	// 	print(" ")


	Mat cost_map = Mat::zeros(statistical_risk.size(), CV_32F); // create a black image
	float risk_weight = 0.7;
	float distance_weight = 0.3;
	for(int row = 0; row < height; row++) // 遍历高
  {
    float* cost_data = cost_map.ptr<float>(row);
    float* risk_data = statistical_risk.ptr<float>(row);
		for(int col = 0; col < width; col++) // 遍历宽
			//for(int c = 0; c < channels; c++) // 遍历通道
			cost_data[col] = risk_weight * risk_data[col] / (UAV_size * UAV_size)
                       + distance_weight * sqrt(pow((row - height/2),2) + pow((col - width/2),2)) / sqrt(pow((height/2),2) + pow((width/2),2));
  }
    


  double min_val, max_val;
  Point min_loc, max_loc;
	minMaxLoc(cost_map, &min_val, &max_val, &min_loc, &max_loc);
	Point ptLeftTop = Point(min_loc.x - (UAV_size - 1)/2, min_loc.y - (UAV_size - 1)/2);
	Point ptRightBottom = Point(min_loc.x + (UAV_size - 1)/2, min_loc.y + (UAV_size - 1)/2);
	rectangle(cost_map, ptLeftTop, ptRightBottom, 255, 1);
	rectangle(binary_risk, ptLeftTop, ptRightBottom, 255, 1);
	circle(binary_risk, min_loc, 3, 255, -1);
	circle(binary_risk, Point(width/2, height/2), 4, 255, -1);

  namedWindow("cost map", CV_WINDOW_NORMAL);
	imshow("cost map", cost_map);
  waitKey(5);

  geometry_msgs::PointStamped landing_site;
  // frame id
  landing_site.header.frame_id = mapFrameId_;

  grid_map::Index index(min_loc.y, min_loc.x);
  grid_map::Position position;
  localMap_.getPosition(index, position);

  // 降落点的坐标 
  landing_site.point.x = position.x();
  landing_site.point.y = position.y();
  landing_site.point.z = localMap_.at("elevation_smooth", index);
  landingSitePublisher_.publish(landing_site);


}


/******************************************************************************************
Function:       OtsuThreshold
Description:	图片二值化最佳阈值确定（大津法,OTSU算法)
Input:          src:原图片
Return:         阈值
******************************************************************************************/
int PointCloudToGridmap::OtsuThreshold(Mat srcImage) {
    int nCols = srcImage.cols;
    int nRows = srcImage.rows;
    int threshold = 0;
    //init the parameters
    int nSumPix[256];
    float nProDis[256];
    for (int i = 0; i < 256; i++)
    {
        nSumPix[i] = 0;
        nProDis[i] = 0;
    }

    //统计灰度集中每个像素在整幅图像中的个数
    for (int i = 0; i < nRows; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            nSumPix[(int)srcImage.at<uchar>(i, j)]++;
        }
    }

    //计算每个灰度级占图像中的概率分布
    for (int i = 0; i < 256; i++)
    {
        nProDis[i] = (float)nSumPix[i] / (nCols*nRows);
    }

    //遍历灰度级【0，255】，计算出最大类间方差下的阈值

    float w0, w1, u0_temp, u1_temp, u0, u1, delta_temp;
    double delta_max = 0.0;
    for (int i = 0; i < 256; i++)
    {
        //初始化相关参数
        w0 = w1 = u0 = u1 = u0_temp = u1_temp = delta_temp = 0;
        for (int j = 0; j < 256; j++)
        {
            //背景部分
            if (j <= i)
            {
                w0 += nProDis[j];
                u0_temp += j*nProDis[j];
            }
            //前景部分
            else
            {
                w1 += nProDis[j];
                u1_temp += j*nProDis[j];
            }
        }
        //计算两个分类的平均灰度
        u0 = u0_temp / w0;
        u1 = u1_temp / w1;
        //依次找到最大类间方差下的阈值
        delta_temp = (float)(w0*w1*pow((u0 - u1), 2)); //前景与背景之间的方差(类间方差)
        if (delta_temp > delta_max)
        {
            delta_max = delta_temp;
            threshold = i;
        }
   }
    return threshold;
}

} /* namespace */
