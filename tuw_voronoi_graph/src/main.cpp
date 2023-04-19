#include "tuw_voronoi_graph/voronoi_graph_node.hpp"

using namespace tuw_graph;

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<VoronoiGeneratorNode>());
  rclcpp::shutdown();
  
  return EXIT_SUCCESS;
}