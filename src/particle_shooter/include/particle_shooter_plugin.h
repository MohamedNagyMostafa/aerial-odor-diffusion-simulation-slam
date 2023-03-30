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
#include <random>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <Particle.h>

typedef std::vector<std::shared_ptr<Particle>> p_vector;

namespace gazebo
{
    class ParticleShooterPlugin : public WorldPlugin
    {
    public:
        ParticleShooterPlugin();
        ~ParticleShooterPlugin();
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
         * Generate a mode with @param modelName name, and insert it to the world.
         * @param modelName model's name to be inserted.
         */
        void generateModelByName_Add2World(std::string modelName);

        /**
         * Listener for gazebo simulation to update the environment status.
         */
        void OnUpdate();

        /**
         * Generate random numbers in range @param min and @param max
         * @param min minimum boundary
         * @param max max boundary
         * @return
         */
        double_t randomInRange(const float_t& min, const float_t& max);

    private:
        physics::WorldPtr       _world;
        int32_t                 _numParticles;
        ignition::math::Pose3d  _sourcePose;
        double_t                _sourcePoseOffsetRadius;
        double_t                _particleVelocity;
        double_t                _particleLifeTime;
        p_vector                _worldParticles;

        /**
         * Define plugin arguments.
         */
        struct Args
        {
            static constexpr const char*  NUM_PARTICLES             = "number_of_particles";
            static constexpr const char* SOURCE_POSE               = "source_position";
            static constexpr const char* SOURCE_POSE_OFFSET_RADIUS = "source_position_offsets_radius";
            static constexpr const char* PARTICLE_VELOCITY         = "particles_velocity";
            static constexpr const char* PARTICLE_LIFE_TIME        = "particles_life_time";
        };

        /**
         * Define plugin sdf elements
         */
        struct Elements
        {
            static constexpr const char* MODEL  = "model";
            static constexpr const char* POSE   = "pose";
        };

        /**
         * Define plugin sdf attributes
         */
        struct Attrs
        {
            static constexpr const char* NAME  = "name";

        };

        const char* PARTICLE_SPHERE_MODEL_SDF_PATH = "model://particle_sphere/model.sdf";

    };
    GZ_REGISTER_WORLD_PLUGIN(ParticleShooterPlugin)
}


#endif //TEST_PARTICLE_SHOOTER_PLUGIN_H

