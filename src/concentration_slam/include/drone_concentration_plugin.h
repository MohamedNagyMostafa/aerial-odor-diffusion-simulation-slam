//
// Created by nagy on 31/03/23.
//

#ifndef CONCENTRATION_SLAM_DRONE_CONCENTRATION_PLUGIN_H
#define CONCENTRATION_SLAM_DRONE_CONCENTRATION_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
    class DroneConcentrationPlugin : public gazebo::WorldPlugin
    {
    public:
        DroneConcentrationPlugin();
        ~DroneConcentrationPlugin();

        /**
         * Initialize the plugin settings
         * @param model
         * @param sdf
         */
        void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

        /**
         * Listener to retrieve drone pose during simulation.
         * @param msg message from the pose topic of the drone.
         */
        void OnDroneLocationListener(const geometry_msgs::PoseStamped::ConstPtr& msg);

        /**
         * Check whether a particle is sensed by a drone or not.
         * @param particlePose particle location in the world.
         * @param dronePose drone location in the world.
         */
        bool isSensedBySensor(ignition::math::Pose3<double>& particlePose, ignition::math::Pose3<double>& dronePose);

    private:
        physics::WorldPtr   _world;
        ros::NodeHandlePtr  _dronePoseNodeHandler;
        ros::Subscriber     _dronePoseSubscriber;
        ros::Publisher      _dronePoseConcentrationPublisher;
        double_t            _squareConcentrationRange;

        /**
         * Define all topics required for the plugin
         */
        struct Topic
        {
            static constexpr const char* DRONE_POSE = "/mavros/local_position/pose";
        };

        /**
         * Define plugin sdf elements
         */
        struct Elements
        {
            static constexpr const char* MODEL          = "model";
            static constexpr const char* POSE           = "pose";
            static constexpr const char* CONCENTRATION  = "concentration";

        };


        const std::string PLUGIN_NODE_NAME          = "drone_pose_concentration";
        const std::string PARTICLE_MODEL_NAME       = "particle_";

    };

    GZ_REGISTER_WORLD_PLUGIN(DroneConcentrationPlugin);
}


#endif //CONCENTRATION_SLAM_DRONE_CONCENTRATION_PLUGIN_H
