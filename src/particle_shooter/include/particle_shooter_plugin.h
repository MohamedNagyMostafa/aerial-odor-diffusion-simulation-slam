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
#include <thread>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>


typedef std::vector<std::thread> th_vector;
namespace gazebo
{
    typedef std::vector<physics::ModelPtr> m_vector;
    typedef m_vector::iterator modelIter;

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
         * Generate a mode with @param modelName name, and insert it to the world.
         * @param modelName model's name to be inserted.
         */
        void generateModelByName_Add2World(std::string modelName);

        /**
         * threading for gazebo simulation to update the environment status.
         */
        void OnUpdate_environmentUpdate();

        /**
         * threading for gazebo simulation to generate particles in the world.
         */
        void OnUpdate_particleGenerator();


        /**
         * Compute concentration of @param particle using Eq. [1/(4.pi.D.t)]^(3/2) exp(-r^2/3Dt)
         * @param particle a particle from the world.
         */
        void computeParticleConcentration(physics::ModelPtr& particle, float_t& t, sdf::ElementPtr& concentrationElement);

        /**
         * This function will be called during the simulation time to update particle
         * location at each time step by computing the displacement.
         * @param particle a particle in the world.
         * @param dt interval time between the last update and current time.
         */
        void updateParticlePosition(physics::ModelPtr& particle, float_t& dt);

        /**
         * Generate random numbers in range @param min and @param max
         * @param min minimum boundary
         * @param max max boundary
         * @return
         */
        double_t randomInRange(const float_t& min, const float_t& max);

        /**
         * Update particles status in the world.
         * @param models vector of particles
         */
        void updateParticlesInEnv(modelIter begin, modelIter end);

    private:
        physics::WorldPtr       _world;
        int32_t                 _maxModelCapacity;
        ignition::math::Pose3d  _sourcePose;
        double_t                _sourcePoseOffsetRadius;
        long                    _particleIdx;
        float_t                 _lastEmitsTime;

        // Diffusion parameters
        float_t _sourceStrength;        // mole/s
        float_t _diffusionCoefficient;  // m/s

        event::ConnectionPtr _particleGeneratorEvent;
        event::ConnectionPtr _environmentUpdateEvent;

        /**
         * Define plugin arguments.
         */
        struct Args
        {
            static constexpr const char* Max_Model_Capacity         = "max_capacity";
            static constexpr const char* SOURCE_POSE                = "source_position";
            static constexpr const char* SOURCE_POSE_OFFSET_RADIUS  = "source_position_offsets_radius";
            static constexpr const char* SOURCE_STRENGTH            = "source_strength";
            static constexpr const char* DIFFUSION_COEFFICIENT      = "diffusion_coefficient";
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

        /**
         * Define plugin sdf attributes
         */
        struct Attrs
        {
            static constexpr const char* NAME  = "name";

        };

        const char* PARTICLE_SPHERE_MODEL_SDF_PATH  = "model://particle_sphere/model.sdf";
        const std::string PARTICLE_MODEL_NAME       = "particle_";

        // TODO: This number should adjusted based on the number of other
        // TODO: models in the world. '3' refers to drone & sun & ground plan models.
        const int8_t NUM_IRRELEVANT_MODELS_WORLD    = 3;


        const int8_t NUM_UPDATE_THREADS                 = 20;
    };
    GZ_REGISTER_WORLD_PLUGIN(ParticleShooterPlugin)
}


#endif //TEST_PARTICLE_SHOOTER_PLUGIN_H

