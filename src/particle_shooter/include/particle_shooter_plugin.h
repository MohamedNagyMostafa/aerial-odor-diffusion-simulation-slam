//
// Created by nagy on 30/03/23.
//

#ifndef TEST_PARTICLE_SHOOTER_PLUGIN_H
#define TEST_PARTICLE_SHOOTER_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/physics.hh>

#include <unistd.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>


#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace gazebo
{
    class ParticleShooterPlugin : public WorldPlugin
    {
    public:
        ParticleShooterPlugin();

        /**
         * To initialize the plugin parameters.
         * @param world Gazebo world
         * @param sdf SDF file of the plugin
         */
        void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

        /**
         * To initialize all plugin args from the sdf file.
         */
        void argsInitialization(sdf::ElementPtr& sdf);

        /**
         * To generate and intialize particle models in the world.
         */
        void particlesInitialization();

        /**
         * Listener for gazebo simulation to update the environment status.
         */
        void OnUpdate();

    };
}


#endif //TEST_PARTICLE_SHOOTER_PLUGIN_H

