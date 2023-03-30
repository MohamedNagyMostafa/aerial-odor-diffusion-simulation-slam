
#include "particle_shooter_plugin.h"

gazebo::ParticleShooterPlugin::ParticleShooterPlugin() {}
gazebo::ParticleShooterPlugin::~ParticleShooterPlugin() noexcept
{
    ROS_FATAL_STREAM("Free memory stored for particles");

    for(std::shared_ptr<Particle> particle: _worldParticles)
    {
        particle.reset();
    }
    _worldParticles.clear();
    ROS_FATAL_STREAM("Memory free completed");

}

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

    // Initialize all particles
    particlesInitialization();
}

void gazebo::ParticleShooterPlugin::generateModelByName_Add2World(std::string modelName)
{
    sdf::SDFPtr particleSDF = sdf::readFile(PARTICLE_SPHERE_MODEL_SDF_PATH);

    ignition::math::Pose3d particlePose(
            randomInRange(_sourcePose.X() - _sourcePoseOffsetRadius, _sourcePose.X() + _sourcePoseOffsetRadius),
            randomInRange(_sourcePose.Y() - _sourcePoseOffsetRadius, _sourcePose.Y() + _sourcePoseOffsetRadius),
            _sourcePose.Z(),
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

void gazebo::ParticleShooterPlugin::particlesInitialization()
{

    for(int32_t particleIdx = 0; particleIdx < _numParticles; particleIdx++)
    {

        float_t time = this->_world->SimTime().Float(); // time in seconds.
        std::string particleName = "particle_" + std::to_string(particleIdx);

        generateModelByName_Add2World(particleName);

        _worldParticles.push_back(std::shared_ptr<Particle>(new Particle(particleName, time, _particleLifeTime)));

    }
}

void gazebo::ParticleShooterPlugin::argsInitialization(sdf::ElementPtr &sdf)
{
    if(sdf->HasElement(Args::NUM_PARTICLES))
    {
        this->_numParticles = sdf->Get<int>(Args::NUM_PARTICLES);
    }

    if(sdf->HasElement(Args::SOURCE_POSE))
    {
        this->_sourcePose   = sdf->Get<ignition::math::Pose3d>(Args::SOURCE_POSE);
    }

    if(sdf->HasElement(Args::SOURCE_POSE_OFFSET_RADIUS))
    {
        this->_sourcePoseOffsetRadius   = sdf->Get<double>(Args::SOURCE_POSE_OFFSET_RADIUS);
    }

    if(sdf->HasElement(Args::PARTICLE_VELOCITY))
    {
        this->_particleVelocity         = sdf->Get<double>(Args::PARTICLE_VELOCITY);
    }

    if(sdf->HasElement(Args::PARTICLE_LIFE_TIME))
    {
        this->_particleLifeTime         = sdf->Get<double>(Args::PARTICLE_LIFE_TIME);
    }
}

double_t gazebo::ParticleShooterPlugin::randomInRange(const float_t& min, const float_t& max){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(min, max);

    return dis(gen);
}


//
//
//{
//
//        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
//
//            if (!ros::isInitialized()) {
//                ROS_FATAL_STREAM("A ROS node in gazebo is not initialized. Unable to load" << PLUGIN_NAME);
//                return;
//            }
//
//
//            //      physics::ModelPtr gasStationModel = _world->ModelByName("gas_station_73");
//            //      ignition::math::Pose3d gasStationPos  = gasStationModel->WorldPose();
//
//            ROS_ERROR_STREAM("Load particle plugin begins!");
//            // Read arguments offsets, source pose
//            this->_world = _world;
//
//            generateParticle("particle_1");
//
//            ROS_ERROR_STREAM("Number of models After" <<  physics::get_world()->Models().size());
////
////            physics::ModelPtr particleModel = _world-> ModelByName("particle_1");
////
////            if(particleSDF == nullptr)
////            {
////                ROS_ERROR_STREAM("Could not find particle model");
////                return;
////            }
////
//            this->_updateConnection = event::Events::ConnectWorldUpdateBegin(
//                    std::bind(&ParticleShooterPlugin::OnUpdate, this));
//
////            ROS_ERROR_STREAM(_world->Models().size());//->AddForce(ignition::math::Vector3d(1.,0.,0.));
//
//            //
//            //    // Make sure the ROS node for Gazebo has already been initialized
//            //    if (!ros::isInitialized())
//            //    {
//            //      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
//            //        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
//            //      return;
//            //    }
//            //      auto node_ = transport::NodePtr (new transport::Node());
//            //      node_-> Init("odor_node");
//            //
//            //      this->world = _world;
//            //    GZ_ASSERT(this->world != NULL, "Got NULL world pointer!");
//            //    this->sdf = _sdf;
//            //    GZ_ASSERT(this->sdf != NULL, "Got NULL SDF element pointer!");
//            //
//            //    // Check if Config Elements exist, otherwise they will have default value
//            //    if (_sdf->HasElement("reset_frequency"))
//            //      this->reset_frequency = _sdf->Get<double>("reset_frequency");
//            //
//            //    if (_sdf->HasElement("x_axis_force"))
//            //      this->x_axis_force = _sdf->Get<double>("x_axis_force");
//            //    if (_sdf->HasElement("y_axis_force"))
//            //      this->y_axis_force = _sdf->Get<double>("y_axis_force");
//            //    if (_sdf->HasElement("z_axis_force"))
//            //      this->z_axis_force = _sdf->Get<double>("z_axis_force");
//            //
//            //    if (_sdf->HasElement("x_origin"))
//            //      this->x_origin = _sdf->Get<double>("x_origin");
//            //    if (_sdf->HasElement("y_origin"))
//            //      this->y_origin = _sdf->Get<double>("y_origin");
//            //    if (_sdf->HasElement("z_origin"))
//            //      this->z_origin = _sdf->Get<double>("z_origin");
//            //
//            //    if (_sdf->HasElement("random_range"))
//            //      this->random_range = _sdf->Get<double>("random_range");
//            //
//            //
//            //
//            //    // We wait for all system to be ready an amount of seconds
//            //    float seconds_to_wait = 5.0;
//            //    this->WaitForseconds(seconds_to_wait);
//            //
//            //    // Update Time Init
//            //    this->old_secs =this->world->SimTime().Float();
//            //    // Listen to the update event. This event is broadcast every
//            //    // simulation iteration.
//
//
//            //    GetParticleList();
//            //    OutputParticleList();
//            //
//            ROS_DEBUG("Particle Generator Ready....");
//        }
//
//        /**
//         * This function is to generate a particle in the world at certain position.
//         * @param name
//         */
//        void generateParticle(const std::string &name) {
////
////            sdf::ElementPtr modelElem(new sdf::Element);
////            sdf::ElementPtr uriElem(new sdf::Element);
////            sdf::ElementPtr includeElem(new sdf::Element);
////            sdf::ElementPtr sdf(new sdf::Element);
////
////            modelElem->SetName("model");
////            modelElem->AddAttribute("name", "string", "model_particle_1", true);
////
////            includeElem->SetName("include");
////
////            uriElem->SetName("uri");
////            uriElem->Set("model://particle_sphere");
////
////            sdf->SetName("sdf");
////
////            includeElem->AddElementDescription(uriElem);
////            modelElem->AddElementDescription(includeElem);
////            sdf->AddElementDescription(modelElem);
////
////            sdf::SDF file;
////            file.Root()->SetParent(modelElem);
////            _world->InsertModelString("<sdf version='1.6'>"
////                                      "<model name=\"model_particle\">\n"
////                                      "      <include>\n"
////                                      "        <uri>model://particle_sphere</uri>\n"
////                                      "      </include>\n"
////                                      "    </model></sdf>");
//
//
//            sdf::SDFPtr particleSDF = sdf::readFile(PARTICLE_SPHERE_MODEL_SDF_PATH);
//            particleSDF->Root()->GetElement(MODEL_ELT)->GetAttribute(NAME_ATTR)->Set(name);
////
//            particleSDF->Root()->GetElement(MODEL_ELT)->FindElement("pose")->Set(
//                    ignition::math::Pose3(1.0, 0.0, 0.03, 0.0, 0.0, 0.0));
////
////            sdf::ElementPtr include(new sdf::ElementPtr());
////            include->
////            particleSDF->Root()->GetElement(MODEL_ELT)->SetParent()
////            particleSDF->Root(particleSDF->Root()->GetElement(MODEL_ELT));
////
//            this->_world->InsertModelSDF(*particleSDF);
//
////            particleSDF->Root()->GetElement(MODEL_ELT)->FindElement("pose")->Set(
////                    ignition::math::Pose3(1.0, 1.0, 0.03, 0.0, 0.0, 0.0));
////
//////            ROS_ERROR_STREAM("Number of models" << _world->Models().size());
////
////            physics::ModelPtr sunModel(new physics::Model());
////            physics::ModelPtr sunModel = physics::ModelPtr(*this->_world);
////            physics::ModelPtr model = this->_world->ModelByName("model_particle");
////            model->SetWorldPose(ignition::math::Pose3(1.0, 0.0, 0.03, 0.0, 0.0, 0.0));
////            model->SetName("particle 1");
////            model->SetLinearVel(ignition::math::Vector3d(0., 0.1, 0.0));
////            physics::ModelPtr modelk;
////            *modelk = model-;
////            modelk->SetName("particle 2");
//
//        }
//
//            void Reset() {
//
//                this->reseting_plugin = true;
//                ROS_ERROR("Reseted the simulation world, we Restart the time clock");
//                // Update Time Init
//                this->old_secs = 0.0;
//                double new_secs = 0.0;
//                double delta = -1.0;
//
//                while (delta < 0.0) {
//                    // We change Direction
//                    ROS_ERROR("Waiting until Clock is reseted and delta is not negative > Update delta=%f, new_secs=%f",
//                              delta, new_secs);
//                    new_secs = this->world->SimTime().Float();
//                    delta = new_secs - this->old_secs;
//                    ROS_ERROR("Updated until Clock is reseted > Update delta=%f, new_secs=%f", delta, new_secs);
//
//                }
//
//                this->reseting_plugin = false;
//
//            }
//
//
//            // Called by the world update start event
//            public: void OnUpdate() {
//
////            auto particle = this->_world->ModelByName("particle_1");
////
////            if(particle)
////            {
////                particle->GetLink("link")->SetForce(ignition::math::Vector3d(0.01, 0.0, 0.0));
////            }
//            ROS_ERROR_STREAM("Number of models After" <<  physics::get_world()->Models().size());
//
////                if (this->reseting_plugin) {
////                    ROS_ERROR("Reseting in Process, please wait...");
////                } else {
////                    // TODO: Check what is necessary now here
////                    double new_secs = this->world->SimTime().Float();
////                    double delta = new_secs - this->old_secs;
////
////                    double max_delta = 0.0;
////
////                    if (this->reset_frequency != 0.0) {
////                        max_delta = 1.0 / this->reset_frequency;
////                    }
////
////                    if (delta > max_delta && delta != 0.0) {
////                        // We update the Old Time variable.
////                        this->old_secs = new_secs;
////
////
////                        if (this->model_to_update_index_now >= this->modelIDToName_size) {
////                            this->model_to_update_index_now = 0;
////                        }
////
////                        // Update the Particles
////                        UpdateParticles(this->model_to_update_index_now);
////                        this->model_to_update_index_now++;
////
////
////                    }
////                }
////
//
//            }
//
//            void WaitForseconds(float seconds_to_wait) {
//                unsigned int microseconds;
//                microseconds = seconds_to_wait * 1e6;
//                ROS_WARN("Waiting for %f seconds", seconds_to_wait);
//                usleep(microseconds);
//                ROS_WARN("Done waiting...");
//
//            }
//
//            void UpdateParticles(int model_to_update_index) {
//                for (auto model: this->world->Models()) {
//                    std::string model_name = model->GetName();
//                    if (this->modelIDToName[model_to_update_index] == model_name) {
//                        this->MoveParticle(model);
//                        this->SetForceParticle(model);
//                    }
//
//                }
//            }
//
//
//            void GetParticleList() {
//                this->modelIDToName.clear();
//                // Initialize color map.
//                this->modelIDToName_size = 0;
//                int i = 0;
//                for (auto model: this->world->Models()) {
//                    std::string model_name = model->GetName();
//                    if (model_name.find(this->particle_base_name) != std::string::npos) {
//                        this->modelIDToName[i] = model->GetName();
//                        i++;
//                    }
//
//                }
//
//                this->modelIDToName_size = modelIDToName.size();
//            }
//
//            void OutputParticleList() {
//                ROS_WARN("Start OutputParticleList...");
//
//                for (auto const &x: this->modelIDToName) {
//                    ROS_WARN("ModelID=%i, Name=%s", x.first, x.second.c_str());
//                }
//
//                ROS_WARN("END OutputParticleList...");
//
//            }
//
//
//            void MoveParticle(boost::shared_ptr<gazebo::physics::Model> model) {
//
//                std::string model_name = model->GetName();
//
//                float x_pos_rand = 0.0;
//                float y_pos_rand = 0.0;
//                float z_pos_rand = 0.0;
//                float roll_rand = 0.0;
//                float pitch_rand = 0.0;
//                float yaw_rand = 0.0;
//
//                // If the model name contains the substring particle, we consider it a particle
//                if (model_name.find(this->particle_base_name) != std::string::npos) {
//                    ROS_WARN("Moving model=%s", model_name.c_str());
//
//                    float x_pos_rand = RandomFloat(this->x_origin - this->random_range,
//                                                   this->x_origin + this->random_range);
//                    float y_pos_rand = RandomFloat(this->y_origin - this->random_range,
//                                                   this->y_origin + this->random_range);
//                    float z_pos_rand = RandomFloat(this->z_origin - this->random_range,
//                                                   this->z_origin + this->random_range);
//
//                    ROS_DEBUG("POSE-RANDOM[X,Y,Z,Roll,Pitch,Yaw=[%f,%f,%f,%f,%f,%f], model=%s", x_pos_rand, y_pos_rand,
//                              z_pos_rand, roll_rand, pitch_rand, yaw_rand, model_name.c_str());
//                    //ignition::math::Pose3 initPose(ignition::math::Vector3<float>(x_pos_rand, y_pos_rand, z_pos_rand), ignition::math::Quaternion<float>(roll_rand, pitch_rand, yaw_rand));
//
//                    model->SetWorldPose(
//                            ignition::math::Pose3d(
//                                    ignition::math::Vector3d(x_pos_rand, y_pos_rand, z_pos_rand),
//                                    ignition::math::Quaterniond(roll_rand, pitch_rand, yaw_rand)
//                            )
//                    );
//
//                    ROS_DEBUG("Moving model=%s....END", model_name.c_str());
//
//                }
//
//
//            }
//
//
//            void SetForceParticle(boost::shared_ptr<gazebo::physics::Model> model) {
//
//                std::string model_name = model->GetName();
//
//                // If the model name contains the substring particle, we consider it a particle
//                if (model_name.find(this->particle_base_name) != std::string::npos) {
//                    ROS_WARN("FORCE APPLIED[X,Y,Z]=[%f,%f,%f]", this->x_axis_force, this->y_axis_force,
//                             this->z_axis_force);
//                    model->GetLink("link")->SetForce(
//                            ignition::math::Vector3d(this->x_axis_force, this->y_axis_force, this->z_axis_force));
//                }
//
//
//            }
//
//
//            float RandomFloat(float a, float b) {
//                float random = ((float) rand()) / (float) RAND_MAX;
//                float diff = b - a;
//                float r = random * diff;
//                return a + r;
//            }
//
//
//            // Pointer to the update event connection
//            private:
//            event::ConnectionPtr _updateConnection;
//            physics::WorldPtr _world;
//
//            std::string PARTICLE_SPHERE_MODEL_SDF_PATH = "model://particle_sphere/model.sdf";
//
//            const std::string NAME_ATTR = "name";
//            const std::string MODEL_ELT = "model";
//            const std::string POSE_ELT = "model";
//
//            /// \brief World pointer.
//            protected: gazebo::physics::WorldPtr world;
//            /// \brief SDF pointer.
//            protected: sdf::ElementPtr sdf;
//            /// \brief Maps model IDs to ModelNames
//            private: std::map<int, std::string> modelIDToName;
//
//            const std::string PLUGIN_NAME = "particle_shooter_plugin";
//            // Update Loop frequency, rate at which we restart the positions and apply force to particles
//            double reset_frequency = 2.0;
//            // Time Memory
//            double old_secs;
//            // Force Direction
//            double x_axis_force = 0.0;
//            double y_axis_force = 0.0;
//            double z_axis_force = 0.0;
//            double x_origin = 0.0;
//            double y_origin = 0.0;
//            double z_origin = 1.0;
//
//            double random_range = 0.1;
//
//            // Reseting Flag
//            bool reseting_plugin = false;
//
//            int modelIDToName_size = 0;
//            int model_to_update_index_now = 0;
//
//            std::string particle_base_name = "particle";
//
//        };
//
//
//}


