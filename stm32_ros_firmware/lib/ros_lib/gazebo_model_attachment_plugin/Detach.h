#ifndef _ROS_SERVICE_Detach_h
#define _ROS_SERVICE_Detach_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gazebo_model_attachment_plugin
{

static const char DETACH[] = "gazebo_model_attachment_plugin/Detach";

  class DetachRequest : public ros::Msg
  {
    public:
      typedef const char* _joint_name_type;
      _joint_name_type joint_name;
      typedef const char* _model_name_1_type;
      _model_name_1_type model_name_1;
      typedef const char* _model_name_2_type;
      _model_name_2_type model_name_2;

    DetachRequest():
      joint_name(""),
      model_name_1(""),
      model_name_2("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_joint_name = strlen(this->joint_name);
      varToArr(outbuffer + offset, length_joint_name);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_name, length_joint_name);
      offset += length_joint_name;
      uint32_t length_model_name_1 = strlen(this->model_name_1);
      varToArr(outbuffer + offset, length_model_name_1);
      offset += 4;
      memcpy(outbuffer + offset, this->model_name_1, length_model_name_1);
      offset += length_model_name_1;
      uint32_t length_model_name_2 = strlen(this->model_name_2);
      varToArr(outbuffer + offset, length_model_name_2);
      offset += 4;
      memcpy(outbuffer + offset, this->model_name_2, length_model_name_2);
      offset += length_model_name_2;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_joint_name;
      arrToVar(length_joint_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joint_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joint_name-1]=0;
      this->joint_name = (char *)(inbuffer + offset-1);
      offset += length_joint_name;
      uint32_t length_model_name_1;
      arrToVar(length_model_name_1, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_model_name_1; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_model_name_1-1]=0;
      this->model_name_1 = (char *)(inbuffer + offset-1);
      offset += length_model_name_1;
      uint32_t length_model_name_2;
      arrToVar(length_model_name_2, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_model_name_2; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_model_name_2-1]=0;
      this->model_name_2 = (char *)(inbuffer + offset-1);
      offset += length_model_name_2;
     return offset;
    }

    virtual const char * getType() override { return DETACH; };
    virtual const char * getMD5() override { return "11ada2739d4d4a28b54f2b6269ce953b"; };

  };

  class DetachResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;

    DetachResponse():
      success(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
     return offset;
    }

    virtual const char * getType() override { return DETACH; };
    virtual const char * getMD5() override { return "937c9679a518e3a18d831e57125ea522"; };

  };

  class Detach {
    public:
    typedef DetachRequest Request;
    typedef DetachResponse Response;
  };

}
#endif
