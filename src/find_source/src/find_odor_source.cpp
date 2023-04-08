//
// Created by nagy on 08/04/23.
//
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <random>

#define WORLD_BOUNDARY_MIN_X    -100.
#define WORLD_BOUNDARY_MIN_Y    -100.
#define WORLD_BOUNDARY_MIN_Z    -100.
#define WORLD_BOUNDARY_MAX_X    100.
#define WORLD_BOUNDARY_MAX_Y    100.
#define WORLD_BOUNDARY_MAX_Z    100.
#define DRONE_ATTITUDE          5.

ros::Subscriber dronePoseSub;
ros::Subscriber dronePoseConcentrationSub;
ros::Publisher  dronePosePub;

bool isConcentrationStreamFound = false;
bool isOdorSourceFound          = false;
bool isReachedTargetPose        = true;

geometry_msgs::PoseStamped  targetPose  ;
geometry_msgs::PoseStamped  currPose;

std::default_random_engine randomGenerator;


static const char* NODE_NAME = "find_odor_source";

struct Topic
{
    static constexpr const char* DRONE_POSE                 = "/mavros/local_position/pose";
    static constexpr const char* SET_DRONE_POSE             = "/mavros/setpoint_position/local";
    static constexpr const char* DRONE_POSE_CONCENTRATION   = "/drone_pose_concentration";
};

void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr);

void dronePoseConcentrationCallback(const std_msgs::Float32::ConstPtr&);

void randomPose(double_t, double_t, double_t);

int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nodeHandle;

    /// Create subscribers & publishers.
    dronePoseSub                = nodeHandle.subscribe<geometry_msgs::PoseStamped::ConstPtr>(Topic::DRONE_POSE, 10, dronePoseCallback);
    dronePoseConcentrationSub   = nodeHandle.subscribe<std_msgs::Float32>(Topic::DRONE_POSE_CONCENTRATION, 10, dronePoseConcentrationCallback);
    dronePosePub                = nodeHandle.advertise<geometry_msgs::PoseStamped>(Topic::SET_DRONE_POSE, 10);

    ros::Rate rate(10);

    while(ros::ok())
    {
        if(!isConcentrationStreamFound)
        {
            // Search randomly
            if(isReachedTargetPose)
            {
                // Get a new pose.
                randomPose(WORLD_BOUNDARY_MIN_X, WORLD_BOUNDARY_MAX_X, targetPose.pose.position.x);
                randomPose(WORLD_BOUNDARY_MIN_Y, WORLD_BOUNDARY_MAX_Y, targetPose.pose.position.y);
                //randomPose(WORLD_BOUNDARY_MIN_Z, WORLD_BOUNDARY_MAX_Z, targetPose.pose.position.z);
                targetPose.pose.position.z  = DRONE_ATTITUDE;

                isReachedTargetPose = false;

                // Send the position to the drone.
                dronePosePub.publish(targetPose);
            }

        }
        else
        {
            if(currPose != targetPose)
            {
                targetPose  = currPose;

                // Send the position to the drone.
                dronePosePub.publish(targetPose);
            }

            // Hold the drone
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

/**
 * Detect concentration at the current drone location
 * @param msg
 */
void dronePoseConcentrationCallback(const std_msgs::Float32::ConstPtr& msg)
{
    isConcentrationStreamFound = (msg->data > 0.);
}

/**
 * Check whether the drone flied to the target pose
 * @param msg
 */
void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    currPose.pose.position.x    =   msg->pose.position.x;
    currPose.pose.position.y    =   msg->pose.position.y;
    currPose.pose.position.z    =   msg->pose.position.z;

    isReachedTargetPose =   (isnan(targetPose.pose.position.x) || isnan(targetPose.pose.position.x) || isnan(targetPose.pose.position.x)) ||
            (currPose.pose.position.x == targetPose.pose.position.x) &&
            (currPose.pose.position.y   ==  targetPose.pose.position.y) &&
            (currPose.pose.position.z   ==  targetPose.pose.position.z);

}

void randomPose(double_t& min, double_t & max, double& value)
{
    std::uniform_real_distribution<float_t>distribution(min, max);

    value   = distribution(randomGenerator);
}