//
// Created by nagy on 13/04/23.
//

#ifndef FIND_SOURCE_CONCENTRATIONPRIORITYQUEUE_H
#define FIND_SOURCE_CONCENTRATIONPRIORITYQUEUE_H
#include <queue>
#include <unordered_map>

#include "ConcentrationZone.h"


template <typename T>
class DynamicPriorityQueue : public std::priority_queue<T, std::vector<T>>
{
public:
    T remove(const std::string & value)
    {
        T obj;
        for(auto iter = this->c.begin(); iter != this->c.end(); iter++)
        {

            if(iter->getZone() == value)
            {
                obj = *iter;
                this->c.erase(iter);
                break;
            }
        }
        // Re-heap.
        std::make_heap(this->c.begin(), this->c.end(), this->comp);

        return obj;
    }
};

class OdorPriorityQueue
{
public:
    void add(geometry_msgs::PoseStamped& pose, float& concentration)
    {
        std::string zone;
        generateZoneToken(pose, zone);

        ConcentrationZone obj;
        if(map.count(zone))
        {
            obj   = queue.remove(zone);

            obj.setConcentration(concentration);
            queue.push(obj);
        }

        obj.setConcentration(concentration);
        obj.setZone(zone);
        obj.setPose(pose);

        queue.push(obj);
        map[zone]   = obj;
    }

    ConcentrationZone top() const
    {
        return queue.top();
    }

    void pop()
    {
        ConcentrationZone obj   = queue.top();
        queue.pop();
        map.erase(obj.getZone());
    }

    bool empty() const
    {
        return queue.empty();
    }


private:
    DynamicPriorityQueue<ConcentrationZone> queue;
    std::unordered_map<std::string , ConcentrationZone> map;

    static void generateZoneToken(geometry_msgs::PoseStamped& pose, std::string& token)
    {
        token = "x:" + std::to_string(pose.pose.position.x) + "y:" + std::to_string(pose.pose.position.y);
    }
};
#endif //FIND_SOURCE_CONCENTRATIONPRIORITYQUEUE_H
