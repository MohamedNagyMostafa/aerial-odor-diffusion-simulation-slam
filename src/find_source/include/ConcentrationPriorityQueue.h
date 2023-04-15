//
// Created by nagy on 13/04/23.
//

#ifndef FIND_SOURCE_CONCENTRATIONPRIORITYQUEUE_H
#define FIND_SOURCE_CONCENTRATIONPRIORITYQUEUE_H
#include <queue>
#include <unordered_map>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include "ConcentrationZone.h"
#include "Utils.h"

__attribute__((unused)) std::mutex point_cloud_queue_mutex;

template <typename T>

class ConcentrationComp
{
public:
    bool operator()(std::shared_ptr<T> obj1, std::shared_ptr<T> obj2)
    {
        return (obj1->getConcentration() < obj2->getConcentration());
    }
};


template <typename T>
class DynamicPriorityQueue : public std::priority_queue<std::shared_ptr<T>, std::vector<std::shared_ptr<T>>, ConcentrationComp<T>>
{
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr gaussianGraph;

public:
    explicit DynamicPriorityQueue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& gaussianGraph): gaussianGraph(gaussianGraph){}

    std::shared_ptr<T> remove(const std::string & value)
    {
        std::shared_ptr<T> obj;
        for(auto iter = this->c.begin(); iter != this->c.end(); iter++)
        {

            if(iter->get()->getZone() == value)
            {
                obj = (*iter);

                this->c.erase(iter);
                break;
            }
        }
        // Re-heap.
        std::make_heap(this->c.begin(), this->c.end(), this->comp);

        return obj;
    }

    void updateProbability()
    {


        for(auto iter = this->c.begin(); iter != this->c.end(); iter++)
        {
            iter->get()->computeProbability();

            for(int i =-1; i < 2; i++)
            {
                for(int j =-1; j < 2; j++)
                {
                    std::lock_guard<std::mutex> lock(point_cloud_queue_mutex);

                    pcl::PointXYZRGB& point = gaussianGraph->at(int(iter->get()->getPose().pose.position.x + WORLD_BOUNDARY_MAX_X) + i,
                                                                 int(iter->get()->getPose().pose.position.y + WORLD_BOUNDARY_MAX_Y) + j);
                    point.z = iter->get()->getProbability() * 20;

                    Utils::setProbabilityColor(iter->get()->getProbability() , point);

                }
            }
        }
    }

};

class OdorPriorityQueue
{
public:
    explicit OdorPriorityQueue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& gaussianGraph): gaussianGraph(gaussianGraph), queue(DynamicPriorityQueue<ConcentrationZone>(gaussianGraph)){}

    void add(geometry_msgs::PoseStamped& pose, float& concentration)
    {
        std::string zone;
        generateZoneToken(pose, zone);


        if(map.count(zone))
        {
            std::shared_ptr<ConcentrationZone> obj;

            obj   = queue.remove(zone);

            obj->setConcentration(concentration);
            queue.push(obj);
            //TODO: Check here.
            return;
        }
        std::shared_ptr<ConcentrationZone> obj(new ConcentrationZone);

        obj->setConcentration(concentration);
        obj->setZone(zone);
        obj->setPose(pose);

        queue.push(obj);
        map[zone]   = *obj;
    }

    std::shared_ptr<ConcentrationZone> top() const
    {
        return queue.top();
    }

    void pop()
    {
        std::shared_ptr<ConcentrationZone> obj   = queue.top();
        queue.pop();
        map.erase(obj->getZone());
    }

    bool empty() const
    {
        return queue.empty();
    }

    void clear()
    {

        while(!queue.empty())
        {
            auto obj = queue.top();

            pcl::PointXYZRGB& point = gaussianGraph->at(int(obj->getPose().pose.position.x + WORLD_BOUNDARY_MAX_X),
                                                        int(obj->getPose().pose.position.y + WORLD_BOUNDARY_MAX_Y));
            point.z = 0.f;

            Utils::setProbabilityColor(point.z, point);

            queue.pop();
        }

    }

    void updateProbabilityGraph() { queue.updateProbability();}

private:
    DynamicPriorityQueue<ConcentrationZone> queue;
    std::unordered_map<std::string , ConcentrationZone> map;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr gaussianGraph;

    static void generateZoneToken(geometry_msgs::PoseStamped& pose, std::string& token)
    {
        token = "x:" + std::to_string(pose.pose.position.x) + "y:" + std::to_string(pose.pose.position.y);
    }
};
#endif //FIND_SOURCE_CONCENTRATIONPRIORITYQUEUE_H
