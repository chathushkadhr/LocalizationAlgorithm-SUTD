#include <chrono>
#include <memory>
#include <string>
#include <functional>
#include <iomanip>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tuw_multi_robot_msgs/msg/graph.hpp"
#include "eigen3/Eigen/Eigen"

#include "tuw_voronoi_graph/segment.h"
#include "tuw_serialization/serializer.h"
#include "tuw_voronoi_map/voronoi_path_generator.h"
#include "tuw_voronoi_graph/voronoi_graph_generator.h"

using std::placeholders::_1;

namespace tuw_graph{

  class VoronoiGeneratorNode : public rclcpp::Node , public Serializer, public voronoi_map::VoronoiPathGenerator, public VoronoiGraphGenerator
  {
    public:
      VoronoiGeneratorNode();

    private:
      
      void timer_callback();
      void globalMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
      void publishSegments();
      bool loadGraph(std::size_t _hash);
      void createGraph(const nav_msgs::msg::OccupancyGrid::ConstPtr &_map, size_t _map_hash);
      bool loadCustomGraph(std::string _path);

      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subMap_;
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubVoronoiMapImage_; 
      rclcpp::Publisher<tuw_multi_robot_msgs::msg::Graph>::SharedPtr pubSegments_;

      double loop_rate;
      bool publishVoronoiMapImage_;
      float inflation_;
      float segment_length_;
      float crossingOptimization_;
      float endSegmentOptimization_;
      std::string graphCachePath_;
      std::string customGraphPath_;
      size_t current_map_hash_;
      Eigen::Vector2d origin_;
      std::vector<Segment> segments_;
      float resolution_;
      cv::Mat map_;
      cv::Mat distField_;
      cv::Mat voronoiMap_;
      std::unique_ptr<float[]> potential;

      nav_msgs::msg::OccupancyGrid voronoiMapImage_;
  };
}
