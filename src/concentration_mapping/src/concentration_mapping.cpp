//
// Created by nagy on 11/04/23.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define WORLD_BOUNDARY_MIN_X        -80.
#define WORLD_BOUNDARY_MIN_Y        -80.
#define WORLD_BOUNDARY_MAX_X        80.
#define WORLD_BOUNDARY_MAX_Y        80.
#define MIN_CONCENTRATION           0.00000001
#define MAX_CONCENTRATION           1.
#define OFFSET                      10.
#define MAP_WINDOW_SIZE            800


static const char* NODE_NAME = "concentration_mapping";
static const char* MAP_WINDOW_NAME  = "Gas Concentration Mapping";

struct Topic
{
    static constexpr const char* DRONE_POSE                 = "/mavros/local_position/pose";
    static constexpr const char* DRONE_POSE_CONCENTRATION   = "/drone_pose_concentration";
};

struct HEAT_MAP_COLOR_SCHEME
{
    static const cv::Scalar COLOR_RED;
    static const cv::Scalar COLOR_YELLOW;
    static const cv::Scalar COLOR_GREEN;
    static const cv::Scalar COLOR_CYAN;
    static const cv::Scalar COLOR_BLUE;
    static const cv::Scalar COLOR_WHITE;
};

const cv::Scalar HEAT_MAP_COLOR_SCHEME::COLOR_RED    = cv::Scalar(0, 0, 255);
const cv::Scalar HEAT_MAP_COLOR_SCHEME::COLOR_YELLOW = cv::Scalar(0, 255, 255);
const cv::Scalar HEAT_MAP_COLOR_SCHEME::COLOR_GREEN  = cv::Scalar(0, 255, 0);
const cv::Scalar HEAT_MAP_COLOR_SCHEME::COLOR_CYAN   = cv::Scalar(255, 255, 0);
const cv::Scalar HEAT_MAP_COLOR_SCHEME::COLOR_BLUE   = cv::Scalar(255, 0, 0);
const cv::Scalar HEAT_MAP_COLOR_SCHEME::COLOR_WHITE   = cv::Scalar(255, 255, 255);


float_t currConcentration;
cv::Mat concentrationMap;
std::int32_t radarRadius;


void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr);
void dronePoseConcentrationCallback(const std_msgs::Float32);
void concentration2Color(cv::Scalar&);


int main(int argc, char** argv)
{
    currConcentration   = 0.;
    radarRadius         = 0.;
    concentrationMap    = cv::Mat(cv::Size(2*(WORLD_BOUNDARY_MAX_X + OFFSET),
                                           2*(WORLD_BOUNDARY_MAX_Y + OFFSET)),
                                  CV_8UC3, HEAT_MAP_COLOR_SCHEME::COLOR_WHITE);

    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nodeHandle;

    ros::Rate   rate(10);

    ros::Subscriber dronePoseSub                = nodeHandle.subscribe<geometry_msgs::PoseStamped::ConstPtr>(Topic::DRONE_POSE, 10, dronePoseCallback);
    ros::Subscriber dronePoseConcentrationSub   = nodeHandle.subscribe<std_msgs::Float32>(Topic::DRONE_POSE_CONCENTRATION, 10, dronePoseConcentrationCallback);

    cv::namedWindow(MAP_WINDOW_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(MAP_WINDOW_NAME, 2 * (WORLD_BOUNDARY_MAX_X + OFFSET), 2 * (WORLD_BOUNDARY_MAX_X + OFFSET));

    while(ros::ok())
    {

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void dronePoseConcentrationCallback(const std_msgs::Float32 msg)
{
    currConcentration   = (msg.data - MIN_CONCENTRATION)/(MAX_CONCENTRATION - MIN_CONCENTRATION);
}

void concentration2Color(cv::Scalar& color)
{
    if(currConcentration <= 0)
    {
        color = HEAT_MAP_COLOR_SCHEME::COLOR_WHITE;
    }
    else if(currConcentration < 0.10) // 0.25
    {
        float_t w   = currConcentration/0.10;
        color       = (1 - w) * HEAT_MAP_COLOR_SCHEME::COLOR_BLUE + w * HEAT_MAP_COLOR_SCHEME::COLOR_CYAN;
    }
    else if(currConcentration < 0.25) // 0.5
    {
        float_t w   = (currConcentration - 0.10)/0.10;
        color       = (1 - w) * HEAT_MAP_COLOR_SCHEME::COLOR_CYAN + w * HEAT_MAP_COLOR_SCHEME::COLOR_GREEN;
    }
    else if(currConcentration < 0.45) // 0.75
    {
        float_t w   = (currConcentration - 0.25)/ 0.10;
        color       = (1 - w) * HEAT_MAP_COLOR_SCHEME::COLOR_GREEN + w * HEAT_MAP_COLOR_SCHEME::COLOR_YELLOW;
    }
    else if(currConcentration < 0.70) // 1.
    {
        float_t w   = (currConcentration - 0.45)/0.10;
        color       = (1 - w) * HEAT_MAP_COLOR_SCHEME::COLOR_YELLOW + w * HEAT_MAP_COLOR_SCHEME::COLOR_RED;
    }
    else
    {
        color = HEAT_MAP_COLOR_SCHEME::COLOR_RED;
    }
}

void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    int32_t x = WORLD_BOUNDARY_MAX_X +  int32_t (msg->pose.position.x);
    int32_t y = WORLD_BOUNDARY_MAX_Y +   int32_t(msg->pose.position.y);

    x += (x < 0)? OFFSET : 0;
    y += (y < 0)? OFFSET : 0;

    cv::Scalar concentrationColor;

    concentration2Color(concentrationColor);

    concentrationMap.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b (concentrationColor[0], concentrationColor[1], concentrationColor[2]);
    concentrationMap.at<cv::Vec3b>(cv::Point(x, y+1)) = cv::Vec3b (concentrationColor[0], concentrationColor[1], concentrationColor[2]);
    concentrationMap.at<cv::Vec3b>(cv::Point(x+1, y)) = cv::Vec3b (concentrationColor[0], concentrationColor[1], concentrationColor[2]);
    concentrationMap.at<cv::Vec3b>(cv::Point(x+1, y+1)) = cv::Vec3b (concentrationColor[0], concentrationColor[1], concentrationColor[2]);
    concentrationMap.at<cv::Vec3b>(cv::Point(x, y-1)) = cv::Vec3b (concentrationColor[0], concentrationColor[1], concentrationColor[2]);
    concentrationMap.at<cv::Vec3b>(cv::Point(x-1, y)) = cv::Vec3b (concentrationColor[0], concentrationColor[1], concentrationColor[2]);
    concentrationMap.at<cv::Vec3b>(cv::Point(x-1, y-1)) = cv::Vec3b (concentrationColor[0], concentrationColor[1], concentrationColor[2]);

    cv::Mat concentrationMapImg = concentrationMap.clone();

    concentrationMapImg.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b (0, 0, 0);
    concentrationMapImg.at<cv::Vec3b>(cv::Point(x, y+1)) = cv::Vec3b (0, 0, 0);
    concentrationMapImg.at<cv::Vec3b>(cv::Point(x+1, y)) = cv::Vec3b (0, 0, 0);
    concentrationMapImg.at<cv::Vec3b>(cv::Point(x+1, y+1)) = cv::Vec3b (0, 0, 0);

    radarRadius = (radarRadius + 3) % 10;
    cv::circle(concentrationMapImg, cv::Point(x, y), radarRadius, cv::Scalar(0, 0, 255), 1);


    cv::imshow(MAP_WINDOW_NAME, concentrationMapImg);
    cv::waitKey(1);
}