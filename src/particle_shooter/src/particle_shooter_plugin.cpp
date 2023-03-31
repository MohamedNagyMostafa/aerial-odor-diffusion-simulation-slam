
#include "particle_shooter_plugin.h"

gazebo::ParticleShooterPlugin::ParticleShooterPlugin() {}
gazebo::ParticleShooterPlugin::~ParticleShooterPlugin() noexcept {}

void gazebo::ParticleShooterPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
    // Check ros is ready.
    if(!ros::isInitialized())
    {
        ROS_FATAL_STREAM("ROS still not been initialized. Failed to initialize particle_shooter_plugin");
        return;
    }

    this->_world = world;

    // Define args.
    argsInitialization(sdf);

    this->_lastEmitsTime    = 0.f;
    this->_particleIdx      = 0;

    // Particle generator thread.
    this->particleGeneratorEvent = event::Events::ConnectWorldUpdateBegin(std::bind(&ParticleShooterPlugin::OnUpdate_particleGenerator, this));

    // Environment update thread.
    this->environmentUpdateEvent = event::Events::ConnectWorldUpdateBegin(std::bind(&ParticleShooterPlugin::OnUpdate_environmentUpdate, this));
}

void gazebo::ParticleShooterPlugin::generateModelByName_Add2World(std::string modelName)
{
    sdf::SDFPtr particleSDF = sdf::readFile(PARTICLE_SPHERE_MODEL_SDF_PATH);

    ignition::math::Pose3d particlePose(
            randomInRange(_sourcePose.X() - _sourcePoseOffsetRadius, _sourcePose.X() + _sourcePoseOffsetRadius),
            randomInRange(_sourcePose.Y() - _sourcePoseOffsetRadius, _sourcePose.Y() + _sourcePoseOffsetRadius),
            randomInRange(_sourcePose.Z() - _sourcePoseOffsetRadius, _sourcePose.Z() + _sourcePoseOffsetRadius),
            _sourcePose.Roll(),
            _sourcePose.Pitch(),
            _sourcePose.Yaw()
            );

    particleSDF->Root()
                ->GetElement(Elements::MODEL)
                ->GetAttribute(Attrs::NAME)
                ->Set(modelName);

    particleSDF->Root()
                ->GetElement(Elements::MODEL)
                ->FindElement(Elements::POSE)
                ->Set(particlePose);

    this->_world->InsertModelSDF(*particleSDF);
}

void gazebo::ParticleShooterPlugin::argsInitialization(sdf::ElementPtr &sdf)
{
    if(sdf->HasElement(Args::Max_Model_Capacity))
    {
        this->_maxModelCapacity = sdf->Get<int>(Args::Max_Model_Capacity);
    }

    if(sdf->HasElement(Args::SOURCE_POSE))
    {
        this->_sourcePose   = sdf->Get<ignition::math::Pose3d>(Args::SOURCE_POSE);
    }

    if(sdf->HasElement(Args::SOURCE_POSE_OFFSET_RADIUS))
    {
        this->_sourcePoseOffsetRadius   = sdf->Get<double>(Args::SOURCE_POSE_OFFSET_RADIUS);
    }

    if(sdf->HasElement(Args::SOURCE_STRENGTH))
    {
        this->_sourceStrength   = sdf->Get<double>(Args::SOURCE_STRENGTH);
    }

    if(sdf->HasElement(Args::DIFFUSION_COEFFICIENT))
    {
        this->_diffusionCoefficient   = sdf->Get<double>(Args::DIFFUSION_COEFFICIENT);
    }

}

double_t gazebo::ParticleShooterPlugin::randomInRange(const float_t& min, const float_t& max){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(min, max);

    return dis(gen);
}

void gazebo::ParticleShooterPlugin::OnUpdate_environmentUpdate()
{
    ///////////////////////////////////////////////
    ////// Update Particle Status in the Env.//////
    ///////////////////////////////////////////////

    float_t currentTime         = this->_world->SimTime().Float();
    // TODO: *0.0001 just to make it slow.
    float_t intervalDuration    = abs(currentTime - _lastEmitsTime) *0.0001;

    // Update particle concentration, remove particles with zero or negative concentration.
    for(auto model : this->_world->Models())
    {
        if(model->GetName().find(PARTICLE_MODEL_NAME) != std::string::npos)
        {
            sdf::ElementPtr concentration;

            // Compute concentration
            computeParticleConcentration(model, currentTime, concentration);

            // Remove particle with zero or negative concentration.
            if(concentration->Get<double>() <= 0.0000000001f)
            {
                this->_world->RemoveModel(model);
                continue;
            }
            // TODO: Uncomment below line to show the concentration of particle 1 decreasing by time.
//            if(model->GetName() == PARTICLE_MODEL_NAME+"1")
//                ROS_FATAL_STREAM("Concentration is " << concentration->Get<double>());

            // Update particle position and transparency.
            updateParticlePosition(model, intervalDuration);

        }
    }
}

void gazebo::ParticleShooterPlugin::OnUpdate_particleGenerator()
{
    //////////////////////////////////////
    ////// Particle Generation Step /////
    /////////////////////////////////////

    // Compute number of particles needed to be pushed
    float_t currentTime         = this->_world->SimTime().Float();
    float_t intervalDuration    = abs(currentTime - _lastEmitsTime);
    int32_t  numParticles       = _sourceStrength * abs(intervalDuration);

    int32_t totalNumModels      = this->_world->Models().size();

    // Check system capacity.
    if(numParticles > 0 && totalNumModels - NUM_IRRELEVANT_MODELS_WORLD < _maxModelCapacity)
    {
        int32_t availableCapacity   = _maxModelCapacity - (totalNumModels - NUM_IRRELEVANT_MODELS_WORLD);
        numParticles = (availableCapacity > numParticles)? numParticles: availableCapacity;

        _lastEmitsTime = currentTime;

        // Multi-threading block
        {
            th_vector generatorThreads;
            for(int32_t i = 0; i < numParticles; i++)
            {
                std::string particleModelName = PARTICLE_MODEL_NAME + std::to_string(_particleIdx + i);

                generatorThreads.push_back(
                        std::thread( [this, particleModelName] {
                            generateModelByName_Add2World(particleModelName);
                        })
                );
            }


            for (auto& generatorThread : generatorThreads)
                generatorThread.join();

        }


        _particleIdx += numParticles;
    }

}

void gazebo::ParticleShooterPlugin::updateParticlePosition(physics::ModelPtr& particle, float_t& dt)
{
    ignition::math::Pose3 currentParticlePose = particle->WorldPose();

    double_t dx = sqrt(2 * _diffusionCoefficient * dt);
    double_t dy = sqrt(2 * _diffusionCoefficient * dt);
    double_t dz = sqrt(2 * _diffusionCoefficient * dt);

    // TODO: uncomment this line if you would like to trace particle 1 location
//    if(particle->GetName() == PARTICLE_MODEL_NAME+"1")
//        ROS_FATAL_STREAM("dx "<< dx << " dy " << dy << " dz " << dz);

    currentParticlePose.SetX(currentParticlePose.X() + dx);
    currentParticlePose.SetY(currentParticlePose.Y() + dy);
    // TODO: not adding dz to avoid diagonal fly .. we need 2D.
    currentParticlePose.SetZ(currentParticlePose.Z());

    particle->SetWorldPose(currentParticlePose);
}

void gazebo::ParticleShooterPlugin::computeParticleConcentration(physics::ModelPtr& particle, float_t& t, sdf::ElementPtr& concentrationElement)
{
    // Euclidean distance between the particle and the emitter.
    ignition::math::Pose3 currParticlePose  = particle->WorldPose();

    double_t r  = sqrt(
            pow(_sourcePose.X() - currParticlePose.X(), 2) +
            pow(_sourcePose.Y() - currParticlePose.Y(), 2) +
            pow(_sourcePose.Z() - currParticlePose.Z(), 2));

    // Compute the particle concentration.
    double_t concentration = pow(_sourceStrength/(4 * M_PI * _diffusionCoefficient * t), 3./2.) * exp(-pow(r,2)/(4 * _diffusionCoefficient * t));

    // Update the concentration in the sdf file.
    concentrationElement      = particle->GetSDF()->FindElement(Elements::CONCENTRATION);

    concentrationElement->Set(concentration);

}
