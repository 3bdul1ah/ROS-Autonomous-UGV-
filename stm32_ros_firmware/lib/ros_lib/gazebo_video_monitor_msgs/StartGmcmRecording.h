#ifndef _ROS_SERVICE_StartGmcmRecording_h
#define _ROS_SERVICE_StartGmcmRecording_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "gazebo_video_monitor_msgs/Strings.h"

namespace gazebo_video_monitor_msgs
{

static const char STARTGMCMRECORDING[] = "gazebo_video_monitor_msgs/StartGmcmRecording";

  class StartGmcmRecordingRequest : public ros::Msg
  {
    public:
      typedef gazebo_video_monitor_msgs::Strings _cameras_type;
      _cameras_type cameras;

    StartGmcmRecordingRequest():
      cameras()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->cameras.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->cameras.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return STARTGMCMRECORDING; };
    virtual const char * getMD5() override { return "176032962ec0f63db7c7bb19a86e7410"; };

  };

  class StartGmcmRecordingResponse : public ros::Msg
  {
    public:

    StartGmcmRecordingResponse()
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

    virtual const char * getType() override { return STARTGMCMRECORDING; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class StartGmcmRecording {
    public:
    typedef StartGmcmRecordingRequest Request;
    typedef StartGmcmRecordingResponse Response;
  };

}
#endif
