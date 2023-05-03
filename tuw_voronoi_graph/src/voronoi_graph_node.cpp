#include "tuw_voronoi_graph/voronoi_graph_node.hpp"

using namespace tuw_graph;


VoronoiGeneratorNode::VoronoiGeneratorNode() : 
    Node("voronoi_graph_node"), Serializer(), voronoi_map::VoronoiPathGenerator(), VoronoiGraphGenerator()
{ 

  this->declare_parameter("voronoi_map_topic",  "/robot1/voronoi_map");
  voronoi_map_topic=this->get_parameter("voronoi_map_topic").get_parameter_value().get<std::string>();
  
  this->declare_parameter("segment_topic",  "/robot1/segments");
  segment_topic=this->get_parameter("segment_topic").get_parameter_value().get<std::string>();  

  this->declare_parameter("loop_rate",  0.1);
  loop_rate=this->get_parameter("loop_rate").get_parameter_value().get<double>(); 
  
  this->declare_parameter("publish_voronoi_map_image",  false);
  publishVoronoiMapImage_=this->get_parameter("publish_voronoi_map_image").get_parameter_value().get<bool>(); 

  this->declare_parameter("map_inflation", 0.1);
  inflation_=this->get_parameter("map_inflation").get_parameter_value().get<float>(); 

  this->declare_parameter("segment_length",  1.0);
  segment_length_=this->get_parameter("segment_length").get_parameter_value().get<float>(); 

  this->declare_parameter("opt_crossings",  0.2);
  crossingOptimization_=this->get_parameter("opt_crossings").get_parameter_value().get<float>(); 


  this->declare_parameter("opt_end_segments", 0.2);
  endSegmentOptimization_=this->get_parameter("opt_end_segments").get_parameter_value().get<float>(); 
  
  this->declare_parameter("graph_cache_path",  "/tmp");
  graphCachePath_=this->get_parameter("graph_cache_path").get_parameter_value().get<std::string>();       

  if (graphCachePath_.back() != '/'){
            graphCachePath_ += "/";
        }
  
  this->declare_parameter("custom_graph_path",  "");
  customGraphPath_=this->get_parameter("custom_graph_path").get_parameter_value().get<std::string>(); 
  
  if (customGraphPath_.back() != '/' && customGraphPath_.size() != 0){
            customGraphPath_ += "/";
        }
  
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();

  subMap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(voronoi_map_topic, qos_profile, std::bind(&VoronoiGeneratorNode::globalMapCallback, this, _1));

  if(publishVoronoiMapImage_){
            pubVoronoiMapImage_=this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_eroded", 1);
        }

  pubSegments_ = this->create_publisher<tuw_multi_robot_msgs::msg::Graph>(segment_topic, 1);
  

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&VoronoiGeneratorNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(),"Initialization done!");

}

void VoronoiGeneratorNode::timer_callback()
{
  //RCLCPP_INFO(this->get_logger(),"Hello World");
  publishSegments();  
}

void VoronoiGeneratorNode::globalMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr _map)
{
    RCLCPP_INFO(this->get_logger(),"Map Recieved");

    std::vector<signed char> map = _map->data;

    std::vector<double> parameters;
    parameters.push_back(_map->info.origin.position.x);
    parameters.push_back(_map->info.origin.position.y);
    parameters.push_back(_map->info.resolution);
    parameters.push_back(inflation_);
    parameters.push_back(segment_length_);
    parameters.push_back(endSegmentOptimization_);
    parameters.push_back(crossingOptimization_);

    size_t new_hash = getHash(map, parameters);


    if (customGraphPath_.size() == 0)
    {
        if (new_hash != current_map_hash_ )
        {
            if (!loadGraph(new_hash) )
            {
                RCLCPP_INFO(this->get_logger(),"Graph generator: Graph not found! Generating new one!");
                createGraph(_map, new_hash);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),"Graph generator: Graph loaded from memory");
            }

            current_map_hash_ = new_hash;
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"loading custom graph from: %s", customGraphPath_.c_str());
        if (loadCustomGraph(customGraphPath_))
           RCLCPP_INFO(this->get_logger(),"graph loaded");
        else
            RCLCPP_INFO(this->get_logger(),"failed to load graph");
    }

    if (publishVoronoiMapImage_ && (*map_.size > 0))
    {
        voronoiMapImage_.header = _map->header;
        voronoiMapImage_.info = _map->info;
        voronoiMapImage_.data.resize(_map->data.size());

        for (unsigned int i = 0; i < voronoiMapImage_.data.size(); i++)
        {
            voronoiMapImage_.data[i] = map_.data[i];
        }
        pubVoronoiMapImage_->publish(voronoiMapImage_);
    }

}

void VoronoiGeneratorNode::createGraph(const nav_msgs::msg::OccupancyGrid::ConstPtr &_map, size_t _map_hash)
{
    std::vector<signed char> map = _map->data;

    segments_.clear();
    Segment::resetId();

    RCLCPP_INFO(this->get_logger(),"Graph generator: Computing distance field ...");
    origin_[0] = _map->info.origin.position.x;
    origin_[1] = _map->info.origin.position.y;
    resolution_ = _map->info.resolution;

    cv::Mat m(_map->info.height, _map->info.width, CV_8SC1, map.data());
    prepareMap(m, map_, inflation_ / _map->info.resolution);
    computeDistanceField(map_, distField_);

    RCLCPP_INFO(this->get_logger(),"Graph generator: Computing voronoi graph ...");
    computeVoronoiMap(distField_, voronoiMap_);

    RCLCPP_INFO(this->get_logger(),"Graph generator: Generating graph ...");
    potential.reset(new float[m.cols * m.rows]);
    float pixel_path_length = segment_length_ / resolution_;
    segments_ = calcSegments(m, distField_, voronoiMap_, potential.get(), pixel_path_length, crossingOptimization_ / resolution_, endSegmentOptimization_ / resolution_);

    //Check Directroy
    save(graphCachePath_ + std::to_string(_map_hash) + "/", segments_, origin_, resolution_, map_);
    RCLCPP_INFO(this->get_logger(),"Graph generator: Created new Graph %lu", _map_hash);
}



bool VoronoiGeneratorNode::loadGraph(std::size_t _hash)
{
    segments_.clear();
    Segment::resetId();
    return load(graphCachePath_ + std::to_string(_hash) + "/", segments_, origin_, resolution_, map_);
}

bool VoronoiGeneratorNode::loadCustomGraph(std::string _path)
{
    segments_.clear();
    Segment::resetId();
    return load(_path, segments_, origin_, resolution_);
}

void VoronoiGeneratorNode::publishSegments()
  {
      tuw_multi_robot_msgs::msg::Graph graph;
      graph.header.frame_id = "map";
      //graph.header.seq = 0;
      graph.header.stamp = this->get_clock()->now();

      graph.origin.position.x = origin_[0]; //TODO test
      graph.origin.position.y = origin_[1]; //TODO test

      for (auto it = segments_.begin(); it != segments_.end(); ++it)
      {
          tuw_multi_robot_msgs::msg::Vertex seg;

          seg.id = (*it).getId();
          seg.weight = (*it).getLength();
          seg.width = (*it).getMinPathSpace() * resolution_;
          seg.valid = true;
          std::vector<Eigen::Vector2d> path = (*it).getPath();

          for (uint32_t i = 0; i < path.size(); i++)
          {
              geometry_msgs::msg::Point pos;
              pos.x = path[i][0] * resolution_;
              pos.y = path[i][1] * resolution_;
              pos.z = 0;

              seg.path.push_back(pos);
          }

          //ROS_INFO("distORIG: %i/%i", (*it)->GetPredecessors().size(), (*it)->GetSuccessors().size());
          std::vector<uint32_t> predecessors = (*it).getPredecessors();

          for (uint32_t i = 0; i < predecessors.size(); i++)
          {
              seg.predecessors.push_back(predecessors[i]);
          }

          std::vector<uint32_t> successors = (*it).getSuccessors();

          for (uint32_t i = 0; i < successors.size(); i++)
          {
              seg.successors.push_back(successors[i]);
          }

          graph.vertices.push_back(seg);
      }

      pubSegments_->publish(graph);
  }

