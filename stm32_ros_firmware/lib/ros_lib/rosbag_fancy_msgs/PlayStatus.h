#ifndef _ROS_rosbag_fancy_msgs_PlayStatus_h
#define _ROS_rosbag_fancy_msgs_PlayStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/duration.h"
#include "ros/time.h"
#include "rosbag_fancy_msgs/PlayTopicStatus.h"

namespace rosbag_fancy_msgs
{

  class PlayStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _status_type;
      _status_type status;
      typedef ros::Duration _duration_type;
      _duration_type duration;
      typedef ros::Duration _current_type;
      _current_type current;
      typedef ros::Time _currentTime_type;
      _currentTime_type currentTime;
      typedef ros::Time _startTime_type;
      _startTime_type startTime;
      typedef ros::Time _endTime_type;
      _endTime_type endTime;
      uint32_t bagfiles_length;
      typedef char* _bagfiles_type;
      _bagfiles_type st_bagfiles;
      _bagfiles_type * bagfiles;
      uint32_t topics_length;
      typedef rosbag_fancy_msgs::PlayTopicStatus _topics_type;
      _topics_type st_topics;
      _topics_type * topics;
      enum { STATUS_PAUSED =  0 };
      enum { STATUS_RUNNING =  1 };

    PlayStatus():
      header(),
      status(0),
      duration(),
      current(),
      currentTime(),
      startTime(),
      endTime(),
      bagfiles_length(0), st_bagfiles(), bagfiles(nullptr),
      topics_length(0), st_topics(), topics(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      *(outbuffer + offset + 0) = (this->duration.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->duration.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->duration.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->duration.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duration.sec);
      *(outbuffer + offset + 0) = (this->duration.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->duration.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->duration.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->duration.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duration.nsec);
      *(outbuffer + offset + 0) = (this->current.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->current.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->current.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current.sec);
      *(outbuffer + offset + 0) = (this->current.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->current.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->current.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current.nsec);
      *(outbuffer + offset + 0) = (this->currentTime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->currentTime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->currentTime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->currentTime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currentTime.sec);
      *(outbuffer + offset + 0) = (this->currentTime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->currentTime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->currentTime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->currentTime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currentTime.nsec);
      *(outbuffer + offset + 0) = (this->startTime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->startTime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->startTime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->startTime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->startTime.sec);
      *(outbuffer + offset + 0) = (this->startTime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->startTime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->startTime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->startTime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->startTime.nsec);
      *(outbuffer + offset + 0) = (this->endTime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->endTime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->endTime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->endTime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->endTime.sec);
      *(outbuffer + offset + 0) = (this->endTime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->endTime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->endTime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->endTime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->endTime.nsec);
      *(outbuffer + offset + 0) = (this->bagfiles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->bagfiles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->bagfiles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->bagfiles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bagfiles_length);
      for( uint32_t i = 0; i < bagfiles_length; i++){
      uint32_t length_bagfilesi = strlen(this->bagfiles[i]);
      varToArr(outbuffer + offset, length_bagfilesi);
      offset += 4;
      memcpy(outbuffer + offset, this->bagfiles[i], length_bagfilesi);
      offset += length_bagfilesi;
      }
      *(outbuffer + offset + 0) = (this->topics_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->topics_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->topics_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->topics_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->topics_length);
      for( uint32_t i = 0; i < topics_length; i++){
      offset += this->topics[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
      this->duration.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->duration.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->duration.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->duration.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->duration.sec);
      this->duration.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->duration.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->duration.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->duration.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->duration.nsec);
      this->current.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->current.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->current.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->current.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->current.sec);
      this->current.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->current.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->current.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->current.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->current.nsec);
      this->currentTime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->currentTime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->currentTime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->currentTime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->currentTime.sec);
      this->currentTime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->currentTime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->currentTime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->currentTime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->currentTime.nsec);
      this->startTime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->startTime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->startTime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->startTime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->startTime.sec);
      this->startTime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->startTime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->startTime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->startTime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->startTime.nsec);
      this->endTime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->endTime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->endTime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->endTime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->endTime.sec);
      this->endTime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->endTime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->endTime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->endTime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->endTime.nsec);
      uint32_t bagfiles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      bagfiles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      bagfiles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      bagfiles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->bagfiles_length);
      if(bagfiles_lengthT > bagfiles_length)
        this->bagfiles = (char**)realloc(this->bagfiles, bagfiles_lengthT * sizeof(char*));
      bagfiles_length = bagfiles_lengthT;
      for( uint32_t i = 0; i < bagfiles_length; i++){
      uint32_t length_st_bagfiles;
      arrToVar(length_st_bagfiles, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_bagfiles; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_bagfiles-1]=0;
      this->st_bagfiles = (char *)(inbuffer + offset-1);
      offset += length_st_bagfiles;
        memcpy( &(this->bagfiles[i]), &(this->st_bagfiles), sizeof(char*));
      }
      uint32_t topics_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->topics_length);
      if(topics_lengthT > topics_length)
        this->topics = (rosbag_fancy_msgs::PlayTopicStatus*)realloc(this->topics, topics_lengthT * sizeof(rosbag_fancy_msgs::PlayTopicStatus));
      topics_length = topics_lengthT;
      for( uint32_t i = 0; i < topics_length; i++){
      offset += this->st_topics.deserialize(inbuffer + offset);
        memcpy( &(this->topics[i]), &(this->st_topics), sizeof(rosbag_fancy_msgs::PlayTopicStatus));
      }
     return offset;
    }

    virtual const char * getType() override { return "rosbag_fancy_msgs/PlayStatus"; };
    virtual const char * getMD5() override { return "18d8189d68cad8bcb9b6fe0e4a2348ef"; };

  };

}
#endif
