//
// Created by nagy on 15/04/23.
//

#ifndef FIND_SOURCE_UTILS_H
#define FIND_SOURCE_UTILS_H

#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#define WORLD_BOUNDARY_MIN_X    -80.
#define WORLD_BOUNDARY_MIN_Y    -80.
#define WORLD_BOUNDARY_MIN_Z    -80.
#define WORLD_BOUNDARY_MAX_X    80.
#define WORLD_BOUNDARY_MAX_Y    80.
#define WORLD_BOUNDARY_MAX_Z    80.
#define DRONE_TAKEOFF_ALTITUDE  6.
#define SENSOR_RANGE            1.
#define SUCCESS_RUN             0
#define FAILED_RUN              -1

class Utils
{
private:
    struct HEAT_MAP_COLOR_SCHEME
    {
        static const cv::Scalar_<std::uint8_t> COLOR_RED;
        static const cv::Scalar_<std::uint8_t> COLOR_YELLOW;
        static const cv::Scalar_<std::uint8_t> COLOR_GREEN;
        static const cv::Scalar_<std::uint8_t> COLOR_CYAN;
        static const cv::Scalar_<std::uint8_t> COLOR_BLUE;
        static const cv::Scalar_<std::uint8_t> COLOR_WHITE;

    };



public:
    static void setProbabilityColor(float probability, pcl::PointXYZRGB& point)
    {
        cv::Scalar_<std::uint8_t> color;


        if(probability <= 0)
        {
            color = HEAT_MAP_COLOR_SCHEME::COLOR_WHITE;
        } else if(probability < 0.10) // 0.25
        {
            float_t w   = probability/0.10;
            color       = (1 - w) * HEAT_MAP_COLOR_SCHEME::COLOR_BLUE + w * HEAT_MAP_COLOR_SCHEME::COLOR_CYAN;
        }
        else if(probability < 0.25) // 0.5
        {
            float_t w   = (probability - 0.10)/0.10;
            color       = (1 - w) * HEAT_MAP_COLOR_SCHEME::COLOR_CYAN + w * HEAT_MAP_COLOR_SCHEME::COLOR_GREEN;
        }
        else if(probability < 0.45) // 0.75
        {
            float_t w   = (probability - 0.25)/ 0.10;
            color       = (1 - w) * HEAT_MAP_COLOR_SCHEME::COLOR_GREEN + w * HEAT_MAP_COLOR_SCHEME::COLOR_YELLOW;
        }
        else if(probability < 0.70) // 1.
        {
            float_t w   = (probability - 0.45)/0.10;
            color       = (1 - w) * HEAT_MAP_COLOR_SCHEME::COLOR_YELLOW + w * HEAT_MAP_COLOR_SCHEME::COLOR_RED;
        }
        else
        {
            color = HEAT_MAP_COLOR_SCHEME::COLOR_RED;
        }

        point.r = color.val[0];
        point.g = color.val[1];
        point.b = color.val[2];
    }

};

const cv::Scalar_<std::uint8_t> Utils::HEAT_MAP_COLOR_SCHEME::COLOR_RED    = cv::Scalar(255, 0, 0);
const cv::Scalar_<std::uint8_t> Utils::HEAT_MAP_COLOR_SCHEME::COLOR_YELLOW = cv::Scalar(255, 255, 0);
const cv::Scalar_<std::uint8_t> Utils::HEAT_MAP_COLOR_SCHEME::COLOR_GREEN  = cv::Scalar(0, 255, 0);
const cv::Scalar_<std::uint8_t> Utils::HEAT_MAP_COLOR_SCHEME::COLOR_CYAN   = cv::Scalar(0, 255, 255);
const cv::Scalar_<std::uint8_t> Utils::HEAT_MAP_COLOR_SCHEME::COLOR_BLUE   = cv::Scalar(0, 0, 255);
const cv::Scalar_<std::uint8_t> Utils::HEAT_MAP_COLOR_SCHEME::COLOR_WHITE   = cv::Scalar(255, 255, 255);

#endif //FIND_SOURCE_UTILS_H
