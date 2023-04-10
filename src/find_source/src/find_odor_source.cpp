//
// Created by nagy on 08/04/23.
//


//
//ros::Publisher pub;
//geometry_msgs::PoseStamped pose;
//float requested = 0;
//
//#include <ros/ros.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/SetMode.h>
//#include <mavros_msgs/State.h>
//#include <mavros_msgs/ParamSet.h>
//#include <mavros_msgs/PositionTarget.h>
//
//
//mavros_msgs::State current_state;
//void state_cb(const mavros_msgs::State::ConstPtr& msg) {
//    current_state = *msg;
//}
//
//int main(int argc, char** argv) {
//    ros::init(argc, argv, "find_odor_source");
//    ros::NodeHandle nh;
//
//    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
//    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
//    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
//    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
//    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
//
//    // wait for FCU connection
//    while (ros::ok() && !current_state.connected) {
//        ros::spinOnce();
//        ros::Rate(20).sleep();
//    }
//
//    // arm the drone
//    mavros_msgs::CommandBool arm_cmd;
//    arm_cmd.request.value = true;
//    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
//        ROS_INFO("Vehicle armed");
//    } else {
//        ROS_ERROR("Failed to arm vehicle");
//        return -1;
//    }
//
//    // set mode to OFFBOARD
//    mavros_msgs::SetMode offboard_set_mode;
//    offboard_set_mode.request.custom_mode = "OFFBOARD";
//    if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
//        ROS_INFO("Offboard enabled");
//    } else {
//        ROS_ERROR("Failed to enable offboard");
//        return -1;
//    }
//
//
//
//
//    // set the target position
//    geometry_msgs::PoseStamped pose_cmd;
//    pose_cmd.header.stamp = ros::Time::now();
//    pose_cmd.header.frame_id = "map";
//    pose_cmd.pose.position.x = 5.0;  // move forward 5 meters
//    pose_cmd.pose.position.y = 0.0;  // maintain current y position
//    pose_cmd.pose.position.z = 5.0;  // maintain current z position
//
//    while (ros::ok()) {
//
//        local_pos_pub.publish(pose_cmd);
//
//        ros::spinOnce();
//        ros::Rate(20).sleep();
//    }
//    return 0;
//}


#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float32.h>

#include <gazebo_msgs/ModelStates.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <random>


#define WORLD_BOUNDARY_MIN_X    -100.
#define WORLD_BOUNDARY_MIN_Y    -100.
#define WORLD_BOUNDARY_MIN_Z    -100.
#define WORLD_BOUNDARY_MAX_X    100.
#define WORLD_BOUNDARY_MAX_Y    100.
#define WORLD_BOUNDARY_MAX_Z    100.
#define DRONE_TAKEOFF_ALTITUDE  6.
#define SUCCESS_RUN             0
#define FAILED_RUN               -1

ros::Subscriber dronePoseSub;
ros::Subscriber dronePoseConcentrationSub;
ros::Subscriber droneModeSub;
ros::Publisher  dronePosePub;

bool isConcentrationStreamFound = false;
bool isOdorSourceFound          = false;
bool isReachedTargetPose        = true;

geometry_msgs::PoseStamped  targetPose  ;
geometry_msgs::PoseStamped  currPose;
mavros_msgs::State          currDroneMode;

std::default_random_engine randomGenerator;


static const char* NODE_NAME = "find_odor_source";

struct Topic
{
    static constexpr const char* DRONE_POSE                 = "/mavros/local_position/pose";
    static constexpr const char* SET_DRONE_POSE             = "/mavros/setpoint_position/local";
    static constexpr const char* DRONE_POSE_CONCENTRATION   = "/drone_pose_concentration";
    static constexpr const char* DRONE_STATE                = "/mavros/state";
    static constexpr const char* ARMING                     = "/mavros/cmd/arming";
    static constexpr const char* SET_MODE                   = "/mavros/set_mode";
};

struct SetModeType
{
    static constexpr const char* OFF_BOARD                  = "OFFBOARD";
};

void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr);

void dronePoseConcentrationCallback(const std_msgs::Float32::ConstPtr&);

void droneStateCallback(const mavros_msgs::State);

void randomPose(double_t, double_t, double_t&);

void waitForStateConnection();

void makeDroneArmed(ros::ServiceClient&);

void setDroneModeToFullControl(ros::ServiceClient&);

int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nodeHandle;

    /// Create subscribers & publishers.
    // Subscribers
    dronePoseSub                = nodeHandle.subscribe<geometry_msgs::PoseStamped::ConstPtr>(Topic::DRONE_POSE, 10, dronePoseCallback);
    dronePoseConcentrationSub   = nodeHandle.subscribe<std_msgs::Float32>(Topic::DRONE_POSE_CONCENTRATION, 10, dronePoseConcentrationCallback);
    droneModeSub                = nodeHandle.subscribe<mavros_msgs::State>(Topic::DRONE_STATE, 10, droneStateCallback);

    // Services
    ros::ServiceClient armingServClient = nodeHandle.serviceClient<mavros_msgs::CommandBool>(Topic::ARMING);
    ros::ServiceClient setModeClient    = nodeHandle.serviceClient<mavros_msgs::SetMode>(Topic::SET_MODE);

    // Publishers
    dronePosePub                = nodeHandle.advertise<geometry_msgs::PoseStamped>(Topic::SET_DRONE_POSE, 10);

    waitForStateConnection();

    // Arming
    makeDroneArmed(armingServClient);

    // Have control on the drone
    setDroneModeToFullControl(setModeClient);


    // Takeoff the drone
    geometry_msgs::PoseStamped takeoffPose;
    takeoffPose.header.stamp    = ros::Time::now();
    takeoffPose.header.frame_id = "map";
    takeoffPose.pose.position.x = 0.0;
    takeoffPose.pose.position.y = 0.0;
    takeoffPose.pose.position.z = DRONE_TAKEOFF_ALTITUDE;

    while(ros::ok() && (currDroneMode.mode != SetModeType::OFF_BOARD || abs(currPose.pose.position.z - takeoffPose.pose.position.z) > 0.1 ))
    {
        dronePosePub.publish(takeoffPose);
        ros::spinOnce();
        ros::Rate(20).sleep();
    }

    ROS_INFO("Drone Successfully takeoff");


    // Keep drone position.
    geometry_msgs::PoseStamped  dronePose = takeoffPose;

    while(ros::ok())
    {
        dronePosePub.publish(takeoffPose);
        ros::spinOnce();
        ros::Rate(20).sleep();
    }
//
//    ros::Rate rate(10);
//
//    while(ros::ok())
//    {
//        if(!isConcentrationStreamFound)
//        {
//            // Search randomly
//            if(isReachedTargetPose)
//            {
//                // Get a new pose.
//                randomPose(WORLD_BOUNDARY_MIN_X, WORLD_BOUNDARY_MAX_X, targetPose.pose.position.x);
//                randomPose(WORLD_BOUNDARY_MIN_Y, WORLD_BOUNDARY_MAX_Y, targetPose.pose.position.y);
//                //randomPose(WORLD_BOUNDARY_MIN_Z, WORLD_BOUNDARY_MAX_Z, targetPose.pose.position.z);
//
//                targetPose.pose.position.z  = DRONE_ATTITUDE;
//                targetPose.pose.position.x  = 5.0;
//                targetPose.pose.position.y  = 5.0;
//
//                targetPose.header.stamp = ros::Time::now();
//                targetPose.header.frame_id = "map";
//                targetPose.pose.orientation.w = 1.0;
//
//                isReachedTargetPose = false;
//
//                // Send the position to the drone.
//                dronePosePub.publish(targetPose);
//
//                ROS_FATAL_STREAM("A new target is installed");
//            }
//
//        }
//        else
//        {
//            if(currPose != targetPose)
//            {
//                targetPose  = currPose;
//
//                // Send the position to the drone.
//                dronePosePub.publish(targetPose);
//
//                ROS_FATAL_STREAM("found molecule stream");
//
//            }
//
//            // Hold the drone
//        }
//
//        rate.sleep();
//        ros::spinOnce();
//    }

    return 0;
}

/**
 * Retrieve drone mode state
 * @param droneStateMsg
 */
void droneStateCallback(const mavros_msgs::State droneStateMsg)
{
    currDroneMode   = droneStateMsg;
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

void randomPose(double_t min, double_t  max, double_t& value)
{
    std::uniform_real_distribution<float_t>distribution(min, max);

    value   = distribution(randomGenerator);
}

/////////////////////////////////////////////
////////////// Utilities////////////////////
/////////////////////////////////////////////

void waitForStateConnection()
{
    while(ros::ok() && !currDroneMode.connected)
    {
        ros::spinOnce();
        ros::Rate(20).sleep();
    }
}

// Services
/**
 * Send an arming active request.
 * @param armingServClient
 */
void makeDroneArmed(ros::ServiceClient& armingServClient)
{
    mavros_msgs::CommandBool armedCmd;
    armedCmd.request.value  = true;

    if(armingServClient.call(armedCmd) && armedCmd.response.success)
    {
        ROS_INFO("Drone armed successfully");
    }
    else
    {
        ROS_ERROR("Failed to arm the drone");
        exit(FAILED_RUN);
    }
}

/**
 * Change drone mode to full control.
 * @param setModeServClient
 */
void setDroneModeToFullControl(ros::ServiceClient& setModeServClient)
{
    mavros_msgs::SetMode offboardSetModeCmd;
    offboardSetModeCmd.request.custom_mode  = SetModeType::OFF_BOARD;

    if(setModeServClient.call(offboardSetModeCmd) && offboardSetModeCmd.response.mode_sent)
    {
        ROS_INFO("OFFBOARD enabled");
    }
    else
    {
        ROS_ERROR("Failed to enable OFFBOARD");
        exit(FAILED_RUN);
    }
}
