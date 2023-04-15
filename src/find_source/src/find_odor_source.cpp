//
// Created by nagy on 08/04/23.
//
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float32.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <random>
#include <ConcentrationPriorityQueue.h>
#include <thread>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Utils.h>

typedef std::vector<geometry_msgs::PoseStamped> DiscreteVector;

ros::Subscriber dronePoseSub;
ros::Subscriber dronePoseConcentrationSub;
ros::Subscriber droneModeSub;
ros::Publisher  dronePosePub;

bool isConcentrationStreamFound = false;
bool isReachedTargetPose        = true;

geometry_msgs::PoseStamped  targetPose  ;
geometry_msgs::PoseStamped  currPose;
geometry_msgs::PoseStamped  concentrationPose;
mavros_msgs::State          currDroneMode;
float_t                     currConcentration;

std::default_random_engine randomGenerator;
std::thread mappingThread;



static const char* NODE_NAME = "find_odor_source";

/**
 * Topics for publishers/subscribers/services
 */
struct Topic
{
    static constexpr const char* DRONE_POSE                 = "/mavros/local_position/pose";
    static constexpr const char* SET_DRONE_POSE             = "/mavros/setpoint_position/local";
    static constexpr const char* DRONE_POSE_CONCENTRATION   = "/drone_pose_concentration";
    static constexpr const char* DRONE_STATE                = "/mavros/state";
    static constexpr const char* ARMING                     = "/mavros/cmd/arming";
    static constexpr const char* SET_MODE                   = "/mavros/set_mode";
};

/**
 * Define drone modes
 */
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

void neighborsDiscretization(DiscreteVector&);

/**
 * Check whether the drone at certain location @param location or not.
 * @param location location to check
 * @return
 */
bool isDroneInLocation(const geometry_msgs::PoseStamped&);

bool isSameLocation(const geometry_msgs::PoseStamped&, const geometry_msgs::PoseStamped& );

void moveToLocation(geometry_msgs::PoseStamped&);

void jumpToNextGrid(const geometry_msgs::PoseStamped&);

void graphInitialization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& graph);

void pclViewerResetAndDraw(pcl::visualization::PCLVisualizer::Ptr& viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& graph);


void pcl_viewer_thread(pcl::visualization::PCLVisualizer::Ptr viewer) {
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nodeHandle;

    ros::Rate rate(20);

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
        rate.sleep();
    }

    ROS_INFO("Drone Successfully takeoff");

    // Initial target
    targetPose.pose.position.x  = NAN;
    targetPose.pose.position.y  = NAN;
    targetPose.pose.position.z  = NAN;

    isReachedTargetPose         = true;

    // Initialize visualization ..
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr gaussianGridProbabilityDensity( new pcl::PointCloud<pcl::PointXYZRGB>);

    graphInitialization(gaussianGridProbabilityDensity);

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Concentration Probability Density"));
    viewer->addPointCloud<pcl::PointXYZRGB>(gaussianGridProbabilityDensity, "grid");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10., "grid");
    viewer->addCoordinateSystem(0.5);
    viewer->initCameraParameters();

    std::thread viewer_thread(pcl_viewer_thread, viewer);

    while(ros::ok())
    {
        if(!isConcentrationStreamFound)
        {
            ROS_INFO("No concentration found");

            // Search randomly
            if(isReachedTargetPose)
            {
                targetPose.header.stamp = ros::Time::now();
                targetPose.header.frame_id = "map";

                // Get a new pose.
                randomPose(WORLD_BOUNDARY_MIN_X, WORLD_BOUNDARY_MAX_X, targetPose.pose.position.x);
                randomPose(WORLD_BOUNDARY_MIN_Y, WORLD_BOUNDARY_MAX_Y, targetPose.pose.position.y);
                //randomPose(WORLD_BOUNDARY_MIN_Z, WORLD_BOUNDARY_MAX_Z, targetPose.pose.position.z);

                targetPose.pose.position.z  = DRONE_TAKEOFF_ALTITUDE;

                isReachedTargetPose = false;

                // Send the position to the drone.
                dronePosePub.publish(targetPose);
                ROS_FATAL_STREAM("A new target is installed");
            }
            else
            {
                dronePosePub.publish(targetPose);
            }


        }
        else
        {
            ROS_INFO("Found concentration stream");
            targetPose  = concentrationPose;
            targetPose.pose.position.z  = DRONE_TAKEOFF_ALTITUDE;

            ROS_INFO("Localize the drone towards detected concentration");

            moveToLocation(targetPose);
            ROS_FATAL_STREAM("found molecule stream");

            OdorPriorityQueue queue(gaussianGridProbabilityDensity);

            if(isConcentrationStreamFound)
            {
                ROS_INFO("Begin searching for odor source");
                // Perform source finding algorithm

                while(ros::ok())
                {
                    viewer->spinOnce();
                    // Discretization.
                    DiscreteVector droneNeighbors;
                    neighborsDiscretization(droneNeighbors);

                    ROS_INFO_STREAM("Descritize into " << droneNeighbors.size());


                    for(geometry_msgs::PoseStamped& neighborPose: droneNeighbors)
                    {

                        moveToLocation(neighborPose);
                        ROS_INFO_STREAM("Concentration " << currConcentration);
                        queue.add(neighborPose, currConcentration);
                    }

                    // Pick max location
                    std::shared_ptr<ConcentrationZone> maxConcentrationNeighbor   = queue.top();
                    maxConcentrationNeighbor->pickIncrement();
                    queue.updateProbabilityGraph();

                    pclViewerResetAndDraw(viewer, gaussianGridProbabilityDensity);

                    ROS_INFO_STREAM("max Concentration " << maxConcentrationNeighbor->getConcentration());

                    ROS_INFO_STREAM("Probability " << maxConcentrationNeighbor->getProbability());



                    if(maxConcentrationNeighbor->getConcentration() == 0){
                        // Reset.
                        queue.clear();
                        pclViewerResetAndDraw(viewer, gaussianGridProbabilityDensity);
                        break;
                    }

                    jumpToNextGrid(maxConcentrationNeighbor->getPose());

                    moveToLocation(targetPose);

                }
            }
        }

        rate.sleep();
        ros::spinOnce();
    }
    viewer_thread.join();
    return SUCCESS_RUN;
}

void graphInitialization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& graph)
{

    graph->width   = 2 *    WORLD_BOUNDARY_MAX_X;
    graph->height  = 2 *    WORLD_BOUNDARY_MAX_Y;
    graph->points.resize(
            graph->width * graph->height
    );

    for(int32_t x = 0; x < graph->width; x++)
    {
        for(int32_t y = 0; y < graph->height; y++)
        {
            int32_t idx  = y + x * graph->height;

            pcl::PointXYZRGB& point     = graph->points[idx];
            point.x =   x;
            point.y =   y;
            point.z = 0.f;

            Utils::setProbabilityColor(point.z, point);
        }
    }
}


void pclViewerResetAndDraw(pcl::visualization::PCLVisualizer::Ptr& viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& graph)
{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    viewer->addPointCloud<pcl::PointXYZRGB> (graph, "grid");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10., "grid");
}
/**
 * Move to the middle of the next discretized grid given gradient result.
 * @param location
 */
void jumpToNextGrid(const geometry_msgs::PoseStamped& location)
{
//    targetPose.pose.position.x  = location.pose.position.x + 2 * (location.pose.position.x - targetPose.pose.position.x);
//    targetPose.pose.position.y  = location.pose.position.y + 2 * (location.pose.position.y - targetPose.pose.position.y);
//    targetPose.pose.position.z  = DRONE_TAKEOFF_ALTITUDE;

    targetPose.pose.position.x  = location.pose.position.x + (location.pose.position.x - targetPose.pose.position.x);
    targetPose.pose.position.y  = location.pose.position.y + (location.pose.position.y - targetPose.pose.position.y);
    targetPose.pose.position.z  = DRONE_TAKEOFF_ALTITUDE;
}

/**
 * Given certain location, move the drone to @param location, function terminates
 * once the drone reaches the desired location.
 * @param location
 */
void moveToLocation(geometry_msgs::PoseStamped& location)
{
    location.header.stamp    = ros::Time::now();
    location.header.frame_id = "map";

    while(ros::ok() && !isDroneInLocation(location))
    {
        dronePosePub.publish(location);
        ros::Rate(20).sleep();
        ros::spinOnce();
    }
}

/**
 * Check two locations are same under certain sensor's measurement error.
 * @param location1
 * @param location2
 * @return
 */
bool isSameLocation(const geometry_msgs::PoseStamped& location1, const geometry_msgs::PoseStamped& location2)
{
    return abs(location1.pose.position.x - location2.pose.position.x) <= 0.2 &&
           abs(location1.pose.position.y - location2.pose.position.y) <= 0.2 &&
           abs(location1.pose.position.z - location2.pose.position.z) <= 0.2;
}

/**
 * Check whether the drone at @param location or not.
 * @param location
 * @return
 */
bool isDroneInLocation(const geometry_msgs::PoseStamped& location)
{
    return isSameLocation(currPose, location);
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

    currConcentration   = msg->data;
    concentrationPose   = currPose;

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
            (abs(currPose.pose.position.x - targetPose.pose.position.x) < 0.2) &&
            (abs(currPose.pose.position.y   ==  targetPose.pose.position.y)  < 0.2) &&
            (abs(currPose.pose.position.z   ==  targetPose.pose.position.z)  < 0.2);

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

/**
 * Generate discretization for neighbor region next to the drone.
 * @param discreteVector
 */
void neighborsDiscretization(DiscreteVector& discreteVector)
{
    for(std::int16_t centriodX = -2 * SENSOR_RANGE; centriodX < 2 * SENSOR_RANGE + 1; centriodX+= 2*SENSOR_RANGE)
    {
        for(std::int16_t centriodY = -2 * SENSOR_RANGE; centriodY < 2 * SENSOR_RANGE + 1; centriodY+= 2*SENSOR_RANGE)
        {
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.pose.position.x = targetPose.pose.position.x + centriodX;
            poseStamped.pose.position.y = targetPose.pose.position.y + centriodY;
            poseStamped.pose.position.z = DRONE_TAKEOFF_ALTITUDE;

            discreteVector.push_back(poseStamped);
        }
    }
}

