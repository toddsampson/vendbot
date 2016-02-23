#ifndef _ROS_SERVICE_ListLabels_h
#define _ROS_SERVICE_ListLabels_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rtabmap_ros
{

static const char LISTLABELS[] = "rtabmap_ros/ListLabels";

  class ListLabelsRequest : public ros::Msg
  {
    public:

    ListLabelsRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return LISTLABELS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ListLabelsResponse : public ros::Msg
  {
    public:
      uint8_t labels_length;
      char* st_labels;
      char* * labels;

    ListLabelsResponse():
      labels_length(0), labels(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = labels_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < labels_length; i++){
      uint32_t length_labelsi = strlen(this->labels[i]);
      memcpy(outbuffer + offset, &length_labelsi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->labels[i], length_labelsi);
      offset += length_labelsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t labels_lengthT = *(inbuffer + offset++);
      if(labels_lengthT > labels_length)
        this->labels = (char**)realloc(this->labels, labels_lengthT * sizeof(char*));
      offset += 3;
      labels_length = labels_lengthT;
      for( uint8_t i = 0; i < labels_length; i++){
      uint32_t length_st_labels;
      memcpy(&length_st_labels, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_labels; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_labels-1]=0;
      this->st_labels = (char *)(inbuffer + offset-1);
      offset += length_st_labels;
        memcpy( &(this->labels[i]), &(this->st_labels), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return LISTLABELS; };
    const char * getMD5(){ return "17b562487ca772bdfa2c078ef00d870f"; };

  };

  class ListLabels {
    public:
    typedef ListLabelsRequest Request;
    typedef ListLabelsResponse Response;
  };

}
#endif
