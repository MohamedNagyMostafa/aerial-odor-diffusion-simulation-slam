//
// Created by nagy on 30/03/23.
//

#ifndef PARTICLE_SHOOTER_PARTICLE_H
#define PARTICLE_SHOOTER_PARTICLE_H

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



class Particle
{
public:
    Particle(std::string& name, float_t & initTime, double_t & lifeTime, bool isActive = true)
    {
        _name   = name;
        _initTime   = initTime;
        _lifeTime   = lifeTime;
        _isActive   = isActive;
    }

    ~Particle();

    bool isActive(){ return _isActive; }

    bool update(float& currTime)
    {
        if(currTime - _initTime >= _lifeTime)
            _isActive = false;

        return isActive();
    }

    void reactivate(float& initTime) { _isActive = true; _initTime = initTime; }

private:
    std::string _name;
    float_t      _initTime;
    float_t      _lifeTime;
    bool        _isActive;

};
#endif //PARTICLE_SHOOTER_PARTICLE_H
