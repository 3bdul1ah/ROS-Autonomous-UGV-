#ifndef _ROS_SERVICE_StopRecording_h
#define _ROS_SERVICE_StopRecording_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gazebo_video_monitor_msgs
{

static const char STOPRECORDING[] = "gazebo_video_monitor_msgs/StopRecording";

  class StopRecordingRequest : public ros::Msg
  {
    public:
      typedef bool _discard_type;
      _discard_type discard;
      typedef const char* _filename_type;
      _filename_type filename;

    StopRecordingRequest():
      discard(0),
      filename("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_discard;
      u_discard.real = this->discard;
      *(outbuffer + offset + 0) = (u_discard.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->discard);
      uint32_t length_filename = strlen(this->filename);
      varToArr(outbuffer + offset, length_filename);
      offset += 4;
      memcpy(outbuffer + offset, this->filename, length_filename);
      offset += length_filename;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_discard;
      u_discard.base = 0;
      u_discard.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->discard = u_discard.real;
      offset += sizeof(this->discard);
      uint32_t length_filename;
      arrToVar(length_filename, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_filename; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_filename-1]=0;
      this->filename = (char *)(inbuffer + offset-1);
      offset += length_filename;
     return offset;
    }

    virtual const char * getType() override { return STOPRECORDING; };
    virtual const char * getMD5() override { return "2996a92a7f8b4bf91dc0b84a5fda46f1"; };

  };

  class StopRecordingResponse : public ros::Msg
  {
    public:
      typedef const char* _path_type;
      _path_type path;
      typedef bool _success_type;
      _success_type success;

    StopRecordingResponse():
      path(""),
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_path = strlen(this->path);
      varToArr(outbuffer + offset, length_path);
      offset += 4;
      memcpy(outbuffer + offset, this->path, length_path);
      offset += length_path;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_path;
      arrToVar(length_path, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_path; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_path-1]=0;
      this->path = (char *)(inbuffer + offset-1);
      offset += length_path;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return STOPRECORDING; };
    virtual const char * getMD5() override { return "33ae47e901cc2aa6ac7b9defcd67a418"; };

  };

  class StopRecording {
    public:
    typedef StopRecordingRequest Request;
    typedef StopRecordingResponse Response;
  };

}
#endif
