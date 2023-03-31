//
// Created by nagy on 31/03/23.
//
#include <drone_concentration_plugin.h>

gazebo::DroneConcentrationPlugin::DroneConcentrationPlugin() {}

gazebo::DroneConcentrationPlugin::~DroneConcentrationPlugin() noexcept {}

void gazebo::DroneConcentrationPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
    // Node initialization
    int argc    = 0;
    char **argv = NULL;

    ros::init(argc, argv, PLUGIN_NODE_NAME, ros::init_options::NoSigintHandler);

    this->_dronePoseNodeHandler.reset(new ros::NodeHandle(""));

    this->_world    = world;
    this->_squareConcentrationRange = 1.; // 1 m^2


    // Subscribe to the topic
    this->_dronePoseSubscriber  = this->_dronePoseNodeHandler->subscribe(
            Topic::DRONE_POSE,
            1,
            &DroneConcentrationPlugin::OnDroneLocationListener,
            this);

    ROS_FATAL_STREAM("Concentration Plugin Loaded Successfully");
}

void gazebo::DroneConcentrationPlugin::OnDroneLocationListener(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // retrieve drone pose.
    ignition::math::Pose3 dronePose(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z
            );

    // Measuer concentration
    int32_t     numSensedParticles          = 0;
    double_t    concentrationSensedRegion  = 0.; // C/m^2

    auto worldModels = this->_world->Models();

    for(auto& model: worldModels)
    {
        std::string modelName   = model->GetName();

        if(modelName.find(PARTICLE_MODEL_NAME) != std::string::npos)
        {
            // Found a particle!

            ignition::math::Pose3 particleLocation = model->WorldPose();

            bool isSensed   = isSensedBySensor(
                    particleLocation,
                    dronePose
                    );

            if(isSensed)
            {
                // Read particle concentration
                sdf::ElementPtr particleConcentration = model->GetSDF()->FindElement(Elements::CONCENTRATION);

                concentrationSensedRegion   += particleConcentration->Get<double>();
                numSensedParticles          += 1;
            }

        }
    }

    double_t concentrationPerMeterSquare = concentrationSensedRegion/ pow(_squareConcentrationRange, 2);

    ROS_FATAL_STREAM("At the drone location (m^2) number of particles sensed: "
                    <<numSensedParticles
                    << " with concentration "
                    << concentrationPerMeterSquare
                    << "C/m^2");

}

bool gazebo::DroneConcentrationPlugin::isSensedBySensor(ignition::math::Pose3<double> &particlePose,
                                                        ignition::math::Pose3<double> &dronePose){
    return  (particlePose.X() >= dronePose.X() - _squareConcentrationRange && particlePose.X() <= dronePose.X() + _squareConcentrationRange) &&
            (particlePose.Y() >= dronePose.Y() - _squareConcentrationRange && particlePose.Y() <= dronePose.Y() + _squareConcentrationRange);
}