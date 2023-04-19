
#ifndef _GENERATOR
#define _GENERATOR

#include "nav_msgs/msg/occupancy_grid.hpp"
#include <memory>

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"

#include <queue>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include <tuw_voronoi_map/thinning.h>

#define DEFAULT_MAP_NAME "voronoi_map"

namespace voronoi_map
{
    class VoronoiPathGenerator
    {
        public:
            VoronoiPathGenerator();
            /**
             * @brief prepares the map by smoothing it
             * @param _map the map 
             * @param _smoothedMap the smoothed map 
             * @param _erodeSize the erode map [pix]
             */
            void prepareMap(const cv::Mat& _map, cv::Mat& _smoothedMap, int _erodeSize);
            /** 
             * @brief  computes the distance field of a map 
             * @param _map the _map
             * @param _distField the resulting distance fieldW
             */
            void computeDistanceField(const cv::Mat& _map, cv::Mat& _distField) ;
            /**
             * @brief computes the voronoi _map
             * @param _distField the dist Field
             * @param _voronoiMap the resulting voronoi map  
             */
            void computeVoronoiMap(const cv::Mat& _distField, cv::Mat& _voronoiMap);
    };

}

#endif
