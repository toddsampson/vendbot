#ifndef _ROS_rtabmap_ros_MapData_h
#define _ROS_rtabmap_ros_MapData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "rtabmap_ros/MapGraph.h"
#include "rtabmap_ros/NodeData.h"

namespace rtabmap_ros
{

  class MapData : public ros::Msg
  {
    public:
      std_msgs::Header header;
      rtabmap_ros::MapGraph graph;
      uint8_t nodes_length;
      rtabmap_ros::NodeData st_nodes;
      rtabmap_ros::NodeData * nodes;

    MapData():
      header(),
      graph(),
      nodes_length(0), nodes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->graph.serialize(outbuffer + offset);
      *(outbuffer + offset++) = nodes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < nodes_length; i++){
      offset += this->nodes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->graph.deserialize(inbuffer + offset);
      uint8_t nodes_lengthT = *(inbuffer + offset++);
      if(nodes_lengthT > nodes_length)
        this->nodes = (rtabmap_ros::NodeData*)realloc(this->nodes, nodes_lengthT * sizeof(rtabmap_ros::NodeData));
      offset += 3;
      nodes_length = nodes_lengthT;
      for( uint8_t i = 0; i < nodes_length; i++){
      offset += this->st_nodes.deserialize(inbuffer + offset);
        memcpy( &(this->nodes[i]), &(this->st_nodes), sizeof(rtabmap_ros::NodeData));
      }
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/MapData"; };
    const char * getMD5(){ return "6b367704e520148d04af9f6848659a47"; };

  };

}
#endif