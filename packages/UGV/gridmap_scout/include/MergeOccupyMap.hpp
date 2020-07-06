#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

namespace gridmap_scout {

class MergeOccupyMap {
public:
  MergeOccupyMap(ros::NodeHandle &nodeHandle);
  virtual ~MergeOccupyMap();

  void globalMapCB(const nav_msgs::OccupancyGridConstPtr &globalMap);
  void updateMapCB(const nav_msgs::OccupancyGridConstPtr &updateMap);

private:
  float ALPHA = 0.8;
  float BETA = 0.8;
  ros::NodeHandle &nodeHandle_;

  ros::Rate rate_ = ros::Rate(20.0);

  bool globalmap_get_ = false;

  ros::Subscriber updateMapSubscriber_;

  ros::Publisher mergedMapPublisher_;

  nav_msgs::OccupancyGrid mergedMap_;

  nav_msgs::OccupancyGrid globalMap_;

  void getGlobalMap();

  void mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy,
                  const nav_msgs::OccupancyGrid &map);
  bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my,
                  unsigned int &index, const nav_msgs::OccupancyGrid &map);
  void worldToMapNoBounds(double wx, double wy, int &mx, int &my,
                          nav_msgs::OccupancyGrid &map);
};

MergeOccupyMap::MergeOccupyMap(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle) {

  getGlobalMap();
  updateMapSubscriber_ =
      nodeHandle_.subscribe("/grid_map_visualization/traversability_grid", 10,
                            &MergeOccupyMap::updateMapCB, this);
  mergedMapPublisher_ =
      nodeHandle_.advertise<nav_msgs::OccupancyGrid>("/mergedMap", 10, true);
}

MergeOccupyMap::~MergeOccupyMap() {}

void MergeOccupyMap::getGlobalMap() {
  auto globalMapSubscriber =
      nodeHandle_.subscribe("/map", 10, &MergeOccupyMap::globalMapCB, this);

  ROS_INFO("Waiting for global map to be received...");

  while (ros::ok() && !globalmap_get_) {
    ros::spinOnce();
    rate_.sleep();
  }

  globalMapSubscriber.shutdown();
}

void MergeOccupyMap::globalMapCB(
    const nav_msgs::OccupancyGridConstPtr &globalMap) {
  globalMap_ = *globalMap;
  globalMap_.header.frame_id = "map";
  globalmap_get_ = true;
  ROS_INFO("Received global map!");
}

void MergeOccupyMap::updateMapCB(
    const nav_msgs::OccupancyGridConstPtr &updateMap) {
  mergedMap_ = globalMap_;
  double x_coord, y_coord;
  unsigned int x_map, y_map, index_global, index_update;
  for (unsigned int x = 0; x < updateMap->info.width; x++) {
    for (unsigned int y = 0; y < updateMap->info.height; y++) {
      mapToWorld(x, y, x_coord, y_coord, *updateMap);
      worldToMap(x_coord, y_coord, x_map, y_map, index_global, globalMap_);
      index_update = updateMap->info.width * y + x;
      mergedMap_.data[index_global] = globalMap_.data[index_global] * ALPHA +
                                      updateMap->data[index_update] * BETA;
      if (mergedMap_.data[index_global] > 100)
        mergedMap_.data[index_global] = 100;
      if (mergedMap_.data[index_global] < 0)
        mergedMap_.data[index_global] = -1;
    }
  }
  mergedMapPublisher_.publish(mergedMap_);
}

/**
 * @brief  Convert from map coordinates to world coordinates
 * @param  mx The x map coordinate
 * @param  my The y map coordinate
 * @param  wx Will be set to the associated world x coordinate
 * @param  wy Will be set to the associated world y coordinate
 */
void MergeOccupyMap::mapToWorld(unsigned int mx, unsigned int my, double &wx,
                                double &wy,
                                const nav_msgs::OccupancyGrid &map) {
  /**
   *  【example】:
   *  data为一维排列，假设其宽度为 width = 5。其index为
   *  [0 1 2 3 4 5 6 7 8 9 10 11 12 13 14]
   *
   *   转换为二维形式为:
   *     4  3  2  1  0 x O原点
   *   <---------------   y
   *  [[ 4  3  2  1  0] | 0
   *   [ 9  8  7  6  5] | 1
   *   [14 13 12 11 10] | 2
   *
   *   其中inex 14 的map坐标应该为 (4, 2)  即是: 14 % 5 = 4； 14 / 5 = 2
   * */
  float origin_x =
      map.info.origin.position
          .x; // 这里的origin.position是以起点珊格的右上角世界坐标表示
  float origin_y = map.info.origin.position.y;
  float resolution = map.info.resolution;
  int width = map.info.width;
  wx = origin_x + (mx + 0.5) * resolution;
  wy = origin_y + (my + 0.5) * resolution;
}

/**
 * @brief  Convert from world coordinates to map coordinates
 * @param  wx The x world coordinate
 * @param  wy The y world coordinate
 * @param  mx Will be set to the associated map x coordinate
 * @param  my Will be set to the associated map y coordinate
 * @return True if the conversion was successful (legal bounds) false otherwise
 */
bool MergeOccupyMap::worldToMap(double wx, double wy, unsigned int &mx,
                                unsigned int &my, unsigned int &index,
                                const nav_msgs::OccupancyGrid &map) {

  float origin_x =
      map.info.origin.position
          .x; // 这里的origin.position是以起点珊格的右上角世界坐标表示
  float origin_y = map.info.origin.position.y;
  float resolution = map.info.resolution;
  int width = map.info.width;

  if (wx < origin_x || wy < origin_y)
    return false;

  mx = (int)((wx - origin_x) / resolution);
  my = (int)((wy - origin_y) / resolution);
  index = my * width + mx;

  //   if (mx < size_x_ && my < size_y_)
  //     return true;

  //   return false;
}

/**
 * @brief  Convert from world coordinates to map coordinates without checking
 * for legal bounds
 * @param  wx The x world coordinate
 * @param  wy The y world coordinate
 * @param  mx Will be set to the associated map x coordinate
 * @param  my Will be set to the associated map y coordinate
 * @note   The returned map coordinates <b>are not guaranteed to lie within the
 * map.</b>
 */
void MergeOccupyMap::worldToMapNoBounds(double wx, double wy, int &mx, int &my,
                                        nav_msgs::OccupancyGrid &map) {
  float origin_x =
      map.info.origin.position
          .x; // 这里的origin.position是以起点珊格的右上角世界坐标表示
  float origin_y = map.info.origin.position.y;
  float resolution = map.info.resolution;
  int width = map.info.width;

  mx = (int)((wx - origin_x) / resolution);
  my = (int)((wy - origin_y) / resolution);
}

} // namespace
