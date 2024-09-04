#ifndef _ROS_hri_msgs_LiveSpeech_h
#define _ROS_hri_msgs_LiveSpeech_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hri_msgs
{

  class LiveSpeech : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _incremental_type;
      _incremental_type incremental;
      typedef const char* _final_type;
      _final_type final;
      typedef float _confidence_type;
      _confidence_type confidence;
      typedef const char* _language_type;
      _language_type language;

    LiveSpeech():
      header(),
      incremental(""),
      final(""),
      confidence(0),
      language("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_incremental = strlen(this->incremental);
      varToArr(outbuffer + offset, length_incremental);
      offset += 4;
      memcpy(outbuffer + offset, this->incremental, length_incremental);
      offset += length_incremental;
      uint32_t length_final = strlen(this->final);
      varToArr(outbuffer + offset, length_final);
      offset += 4;
      memcpy(outbuffer + offset, this->final, length_final);
      offset += length_final;
      offset += serializeAvrFloat64(outbuffer + offset, this->confidence);
      uint32_t length_language = strlen(this->language);
      varToArr(outbuffer + offset, length_language);
      offset += 4;
      memcpy(outbuffer + offset, this->language, length_language);
      offset += length_language;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_incremental;
      arrToVar(length_incremental, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_incremental; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_incremental-1]=0;
      this->incremental = (char *)(inbuffer + offset-1);
      offset += length_incremental;
      uint32_t length_final;
      arrToVar(length_final, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_final; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_final-1]=0;
      this->final = (char *)(inbuffer + offset-1);
      offset += length_final;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->confidence));
      uint32_t length_language;
      arrToVar(length_language, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_language; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_language-1]=0;
      this->language = (char *)(inbuffer + offset-1);
      offset += length_language;
     return offset;
    }

    virtual const char * getType() override { return "hri_msgs/LiveSpeech"; };
    virtual const char * getMD5() override { return "691fad051ce92a736be494854c99c2d5"; };

  };

}
#endif
