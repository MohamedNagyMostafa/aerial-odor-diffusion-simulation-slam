//
// Created by nagy on 13/04/23.
//

#ifndef FIND_SOURCE_CONCENTRATIONZONE_H
#define FIND_SOURCE_CONCENTRATIONZONE_H
#include <iostream>
#include <geometry_msgs/PoseStamped.h>

class ConcentrationZone
{
public:
    ConcentrationZone(){
        probability  = 0;
        picked       = 0;
    };

    ConcentrationZone(std::string regionId, float concentration, geometry_msgs::PoseStamped pose, float probability  = 0): region(regionId), concentration(concentration), pose(pose)
    {
        picked      = 0;
    }

    ~ConcentrationZone(){};

    void pickIncrement(){ picked++; totalPicked++;}

    void computeProbability(){ probability = (totalPicked > 0)?picked/ totalPicked : 0;}

    float getProbability() const{ return probability; }

    std::string getZone() const{ return region; }

    geometry_msgs::PoseStamped getPose() const{ return pose;}

    float getConcentration() const { return concentration;}

    void setZone(std::string zone) { region = zone; }

    void setConcentration(float concentration){ this->concentration = concentration;}

    void setPose(geometry_msgs::PoseStamped& pose)  { this->pose = pose;}

    ConcentrationZone& operator=(const ConcentrationZone& obj)
    {
        this->picked        = obj.picked;
        this->concentration = obj.concentration;
        this->region        = obj.region;
        this->probability   = obj.probability;
        this->pose          = obj.pose;

        return *this;
    }



private:

    std::string     region;
    geometry_msgs::PoseStamped pose;

    float           concentration;
    int             picked;
    float           probability;
    static float    totalPicked;

};

float ConcentrationZone::totalPicked = 0;

#endif //FIND_SOURCE_CONCENTRATIONZONE_H
