//
// Created by nagy on 13/04/23.
//

#ifndef FIND_SOURCE_CONCENTRATIONPRIORITYQUEUE_H
#define FIND_SOURCE_CONCENTRATIONPRIORITYQUEUE_H
#include <queue>
#include <unordered_map>
#include <std_msgs/Float32MultiArray.h>

#include "ConcentrationZone.h"

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
    std_msgs::Float32MultiArray gaussianGraph;

public:
    explicit DynamicPriorityQueue(std_msgs::Float32MultiArray& gaussianGraph): gaussianGraph(gaussianGraph){}

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
            gaussianGraph.data[int(iter->get()->getPose().pose.position.x),int(iter->get()->getPose().pose.position.y)] = iter->get()->getProbability();
        }
    }

};

class OdorPriorityQueue
{
public:
    explicit OdorPriorityQueue(std_msgs::Float32MultiArray& gaussianGraph): gaussianGraph(gaussianGraph), queue(DynamicPriorityQueue<ConcentrationZone>(gaussianGraph)){}

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
            queue.pop();
    }

    void updateProbabilityGraph() { queue.updateProbability();}

private:
    DynamicPriorityQueue<ConcentrationZone> queue;
    std::unordered_map<std::string , ConcentrationZone> map;
    std_msgs::Float32MultiArray gaussianGraph;

    static void generateZoneToken(geometry_msgs::PoseStamped& pose, std::string& token)
    {
        token = "x:" + std::to_string(pose.pose.position.x) + "y:" + std::to_string(pose.pose.position.y);
    }
};
#endif //FIND_SOURCE_CONCENTRATIONPRIORITYQUEUE_H
