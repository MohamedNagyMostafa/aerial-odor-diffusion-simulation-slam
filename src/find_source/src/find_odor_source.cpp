//
// Created by nagy on 08/04/23.
//
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <random>
#include <mavros_msgs/SetMode.h>

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
    static constexpr const char* SET_DRONE_POSE             = "/move_base_simple/goal";
    static constexpr const char* DRONE_POSE_CONCENTRATION   = "/drone_pose_concentration";
};
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>

ros::Publisher pub;
geometry_msgs::PoseStamped pose;
float requested = 0;

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/PositionTarget.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "find_odor_source");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        ros::Rate(20).sleep();
    }

    // arm the drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    } else {
        ROS_ERROR("Failed to arm vehicle");
        return -1;
    }

    // set mode to OFFBOARD
    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
    } else {
        ROS_ERROR("Failed to enable offboard");
        return -1;
    }



    // set the target position
    geometry_msgs::PoseStamped pose_cmd;
    pose_cmd.header.stamp = ros::Time::now();
    pose_cmd.header.frame_id = "map";
    pose_cmd.pose.position.x = 5.0;  // move forward 5 meters
    pose_cmd.pose.position.y = 0.0;  // maintain current y position
    pose_cmd.pose.position.z = 5.0;  // maintain current z position

    while (ros::ok()) {

        local_pos_pub.publish(pose_cmd);

        ros::spinOnce();
        ros::Rate(20).sleep();
    }
    return 0;
}
//int main(int argc, char** argv)
//{
//    // Initialize the node
//    ros::init(argc, argv, "drone_forward");
//    ros::NodeHandle nh;
//
//    // Create the arming client
//    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
//
//    // Create the mode client
//    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
//
//    // Create the setpoint position publisher
//    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_position/local", 10);
//
//    // Wait for the services to become available
//    arming_client.waitForExistence();
//    set_mode_client.waitForExistence();
//
//
//    // Arm the drone
//    mavros_msgs::CommandBool arm_cmd;
//    arm_cmd.request.value = true;
//    if (!arming_client.call(arm_cmd)) {
//        ROS_ERROR("Failed to arm the drone");
//        return 1;
//    }
//
//    // Switch to OFFBOARD mode
//    mavros_msgs::SetMode offboard_mode_cmd;
//    offboard_mode_cmd.request.custom_mode = "OFFBOARD";
//    if (!set_mode_client.call(offboard_mode_cmd)) {
//        ROS_ERROR("Failed to set the flight mode to OFFBOARD");
//        return 1;
//    }
//
//    // Move the drone to a position 5 meters forward from the current position
//    geometry_msgs::PoseStamped pose_cmd;
//    pose_cmd.header.stamp = ros::Time::now();
//    pose_cmd.header.frame_id = "map";
//    pose_cmd.pose.position.x = 5.0;
//    setpoint_pub.publish(pose_cmd);
//
//
//    return 0;
//}

//
//void modelStateCallback(const gazebo_msgs::ModelStates& msg)
//{
//    // Find the index of the drone model in the message
//    int index = -1;
//    for (int i = 0; i < msg.name.size(); i++) {
//        if (msg.name[i] == "iris") {
//            index = i;
//            break;
//        }
//    }
//
//    if (index != -1) {
//        // Extract the position and orientation of the drone
//        pose.pose.position = msg.pose[index].position;
//        pose.pose.orientation = msg.pose[index].orientation;
//
//        // Set the header information
//        pose.header.stamp = ros::Time::now();
//        pose.header.frame_id = "map"; // or any other Gazebo frame
//        pose.pose.position.z = 10.2;
//        // Publish the target position
//        if (requested == 0)
//        {
//            pub.publish(pose);
//            requested = 1;
//        }
//
//    }
//}
//
//int main(int argc, char** argv)
//{
//    // Initialize the node
//    ros::init(argc, argv, "gazebo_target_position");
//    ros::NodeHandle nh;
//
//    // Subscribe to the model_states topic
//    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, modelStateCallback);
//
//    // Advertise the target position topic
//    pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
//
//    // Set the loop rate
//    ros::Rate rate(10); // 10 Hz
//
//    // Enter the main loop
//    while (ros::ok()) {
//        ros::spinOnce();
//        rate.sleep();
//    }
//
//    return 0;
//}

//int main(int argc, char** argv) {
//    ros::init(argc, argv, "goal_publisher");
//    ros::NodeHandle nh;
//    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
//    ros::Rate rate(10);
//    ros::ServiceClient client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
//    mavros_msgs::SetMode srv_setMode;
//    srv_setMode.request.custom_mode = "GUIDED"; // set the custom mode to "GUIDED"
//    client.call(srv_setMode);
//    while (ros::ok()) {
//        geometry_msgs::PoseStamped goal_msg;
//        goal_msg.header.frame_id = "map";
//        goal_msg.header.stamp = ros::Time::now();
//        goal_msg.pose.position.x = 0.0;
//        goal_msg.pose.position.y = 0.0;
//        goal_msg.pose.position.z = 0.0;
//        goal_msg.pose.orientation.x = 0.0;
//        goal_msg.pose.orientation.y = 0.0;
//        goal_msg.pose.orientation.z = 0.0;
//        goal_msg.pose.orientation.w = 1.0;
//
//        goal_pub.publish(goal_msg);
//        rate.sleep();
//    }
//
//    return 0;
//}
/**
void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr);

void dronePoseConcentrationCallback(const std_msgs::Float32::ConstPtr&);

void randomPose(double_t, double_t, double_t&);

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
                targetPose.pose.position.x  = 5.0;
                targetPose.pose.position.y  = 5.0;

                targetPose.header.stamp = ros::Time::now();
                targetPose.header.frame_id = "map";
                targetPose.pose.orientation.w = 1.0;

                isReachedTargetPose = false;

                // Send the position to the drone.
                dronePosePub.publish(targetPose);

                ROS_FATAL_STREAM("A new target is installed");
            }

        }
        else
        {
            if(currPose != targetPose)
            {
                targetPose  = currPose;

                // Send the position to the drone.
                dronePosePub.publish(targetPose);

                ROS_FATAL_STREAM("found molecule stream");

            }

            // Hold the drone
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
**/
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