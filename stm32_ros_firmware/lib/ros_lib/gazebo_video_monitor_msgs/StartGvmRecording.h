#ifndef _ROS_SERVICE_StartGvmRecording_h
#define _ROS_SERVICE_StartGvmRecording_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gazebo_video_monitor_msgs
{

static const char STARTGVMRECORDING[] = "gazebo_video_monitor_msgs/StartGvmRecording";

  class StartGvmRecordingRequest : public ros::Msg
  {
    public:
      typedef bool _disable_window_type;
      _disable_window_type disable_window;
      typedef bool _world_as_main_view_type;
      _world_as_main_view_type world_as_main_view;

    StartGvmRecordingRequest():
      disable_window(0),
      world_as_main_view(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_disable_window;
      u_disable_window.real = this->disable_window;
      *(outbuffer + offset + 0) = (u_disable_window.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->disable_window);
      union {
        bool real;
        uint8_t base;
      } u_world_as_main_view;
      u_world_as_main_view.real = this->world_as_main_view;
      *(outbuffer + offset + 0) = (u_world_as_main_view.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->world_as_main_view);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_disable_window;
      u_disable_window.base = 0;
      u_disable_window.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->disable_window = u_disable_window.real;
      offset += sizeof(this->disable_window);
      union {
        bool real;
        uint8_t base;
      } u_world_as_main_view;
      u_world_as_main_view.base = 0;
      u_world_as_main_view.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->world_as_main_view = u_world_as_main_view.real;
      offset += sizeof(this->world_as_main_view);
     return offset;
    }

    virtual const char * getType() override { return STARTGVMRECORDING; };
    virtual const char * getMD5() override { return "74a62fa060bc26d4ef1f8e73a79a72f6"; };

  };

  class StartGvmRecordingResponse : public ros::Msg
  {
    public:

    StartGvmRecordingResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return STARTGVMRECORDING; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class StartGvmRecording {
    public:
    typedef StartGvmRecordingRequest Request;
    typedef StartGvmRecordingResponse Response;
  };

}
#endif
