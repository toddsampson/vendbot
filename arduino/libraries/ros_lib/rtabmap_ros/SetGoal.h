#ifndef _ROS_SERVICE_SetGoal_h
#define _ROS_SERVICE_SetGoal_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace rtabmap_ros
{

static const char SETGOAL[] = "rtabmap_ros/SetGoal";

  class SetGoalRequest : public ros::Msg
  {
    public:
      int32_t node_id;
      const char* node_label;

    SetGoalRequest():
      node_id(0),
      node_label("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_node_id;
      u_node_id.real = this->node_id;
      *(outbuffer + offset + 0) = (u_node_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_node_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_node_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_node_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->node_id);
      uint32_t length_node_label = strlen(this->node_label);
      memcpy(outbuffer + offset, &length_node_label, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->node_label, length_node_label);
      offset += length_node_label;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_node_id;
      u_node_id.base = 0;
      u_node_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_node_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_node_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_node_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->node_id = u_node_id.real;
      offset += sizeof(this->node_id);
      uint32_t length_node_label;
      memcpy(&length_node_label, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_node_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_node_label-1]=0;
      this->node_label = (char *)(inbuffer + offset-1);
      offset += length_node_label;
     return offset;
    }

    const char * getType(){ return SETGOAL; };
    const char * getMD5(){ return "baadfb04a43ec26085eb7bebc9a80862"; };

  };

  class SetGoalResponse : public ros::Msg
  {
    public:
      uint8_t path_ids_length;
      int32_t st_path_ids;
      int32_t * path_ids;
      uint8_t path_poses_length;
      geometry_msgs::Pose st_path_poses;
      geometry_msgs::Pose * path_poses;

    SetGoalResponse():
      path_ids_length(0), path_ids(NULL),
      path_poses_length(0), path_poses(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = path_ids_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < path_ids_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_path_idsi;
      u_path_idsi.real = this->path_ids[i];
      *(outbuffer + offset + 0) = (u_path_idsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_path_idsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_path_idsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_path_idsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->path_ids[i]);
      }
      *(outbuffer + offset++) = path_poses_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < path_poses_length; i++){
      offset += this->path_poses[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t path_ids_lengthT = *(inbuffer + offset++);
      if(path_ids_lengthT > path_ids_length)
        this->path_ids = (int32_t*)realloc(this->path_ids, path_ids_lengthT * sizeof(int32_t));
      offset += 3;
      path_ids_length = path_ids_lengthT;
      for( uint8_t i = 0; i < path_ids_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_path_ids;
      u_st_path_ids.base = 0;
      u_st_path_ids.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_path_ids.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_path_ids.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_path_ids.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_path_ids = u_st_path_ids.real;
      offset += sizeof(this->st_path_ids);
        memcpy( &(this->path_ids[i]), &(this->st_path_ids), sizeof(int32_t));
      }
      uint8_t path_poses_lengthT = *(inbuffer + offset++);
      if(path_poses_lengthT > path_poses_length)
        this->path_poses = (geometry_msgs::Pose*)realloc(this->path_poses, path_poses_lengthT * sizeof(geometry_msgs::Pose));
      offset += 3;
      path_poses_length = path_poses_lengthT;
      for( uint8_t i = 0; i < path_poses_length; i++){
      offset += this->st_path_poses.deserialize(inbuffer + offset);
        memcpy( &(this->path_poses[i]), &(this->st_path_poses), sizeof(geometry_msgs::Pose));
      }
     return offset;
    }

    const char * getType(){ return SETGOAL; };
    const char * getMD5(){ return "c48b04c7cc1287d5c4a5b9c0637dbc3d"; };

  };

  class SetGoal {
    public:
    typedef SetGoalRequest Request;
    typedef SetGoalResponse Response;
  };

}
#endif
