#ifndef _ROS_rosbag_fancy_msgs_PlayTopicStatus_h
#define _ROS_rosbag_fancy_msgs_PlayTopicStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosbag_fancy_msgs
{

  class PlayTopicStatus : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef float _rate_type;
      _rate_type rate;
      typedef float _bandwidth_type;
      _bandwidth_type bandwidth;

    PlayTopicStatus():
      name(""),
      rate(0),
      bandwidth(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      union {
        float real;
        uint32_t base;
      } u_rate;
      u_rate.real = this->rate;
      *(outbuffer + offset + 0) = (u_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rate);
      union {
        float real;
        uint32_t base;
      } u_bandwidth;
      u_bandwidth.real = this->bandwidth;
      *(outbuffer + offset + 0) = (u_bandwidth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bandwidth.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bandwidth.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bandwidth.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bandwidth);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      union {
        float real;
        uint32_t base;
      } u_rate;
      u_rate.base = 0;
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rate = u_rate.real;
      offset += sizeof(this->rate);
      union {
        float real;
        uint32_t base;
      } u_bandwidth;
      u_bandwidth.base = 0;
      u_bandwidth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bandwidth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bandwidth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bandwidth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bandwidth = u_bandwidth.real;
      offset += sizeof(this->bandwidth);
     return offset;
    }

    virtual const char * getType() override { return "rosbag_fancy_msgs/PlayTopicStatus"; };
    virtual const char * getMD5() override { return "f9fdaffdd7fc115927db7f4b9e8cca43"; };

  };

}
#endif
