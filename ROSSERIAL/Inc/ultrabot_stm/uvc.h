#ifndef _ROS_SERVICE_uvc_h
#define _ROS_SERVICE_uvc_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ultrabot_stm
{

static const char UVC[] = "ultrabot_stm/uvc";

  class uvcRequest : public ros::Msg
  {
    public:
      typedef bool _left1_type;
      _left1_type left1;
      typedef bool _left2_type;
      _left2_type left2;
      typedef bool _right1_type;
      _right1_type right1;
      typedef bool _right2_type;
      _right2_type right2;
      typedef bool _fans_type;
      _fans_type fans;

    uvcRequest():
      left1(0),
      left2(0),
      right1(0),
      right2(0),
      fans(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_left1;
      u_left1.real = this->left1;
      *(outbuffer + offset + 0) = (u_left1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left1);
      union {
        bool real;
        uint8_t base;
      } u_left2;
      u_left2.real = this->left2;
      *(outbuffer + offset + 0) = (u_left2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left2);
      union {
        bool real;
        uint8_t base;
      } u_right1;
      u_right1.real = this->right1;
      *(outbuffer + offset + 0) = (u_right1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right1);
      union {
        bool real;
        uint8_t base;
      } u_right2;
      u_right2.real = this->right2;
      *(outbuffer + offset + 0) = (u_right2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right2);
      union {
        bool real;
        uint8_t base;
      } u_fans;
      u_fans.real = this->fans;
      *(outbuffer + offset + 0) = (u_fans.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fans);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_left1;
      u_left1.base = 0;
      u_left1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->left1 = u_left1.real;
      offset += sizeof(this->left1);
      union {
        bool real;
        uint8_t base;
      } u_left2;
      u_left2.base = 0;
      u_left2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->left2 = u_left2.real;
      offset += sizeof(this->left2);
      union {
        bool real;
        uint8_t base;
      } u_right1;
      u_right1.base = 0;
      u_right1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->right1 = u_right1.real;
      offset += sizeof(this->right1);
      union {
        bool real;
        uint8_t base;
      } u_right2;
      u_right2.base = 0;
      u_right2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->right2 = u_right2.real;
      offset += sizeof(this->right2);
      union {
        bool real;
        uint8_t base;
      } u_fans;
      u_fans.base = 0;
      u_fans.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fans = u_fans.real;
      offset += sizeof(this->fans);
     return offset;
    }

    virtual const char * getType() override { return UVC; };
    virtual const char * getMD5() override { return "4c3f9bf79d01e6b005342e7a86b066c8"; };

  };

  class uvcResponse : public ros::Msg
  {
    public:
      uint8_t timeout[5];
      bool status[5];

    uvcResponse():
      timeout(),
      status()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 5; i++){
      *(outbuffer + offset + 0) = (this->timeout[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->timeout[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        bool real;
        uint8_t base;
      } u_statusi;
      u_statusi.real = this->status[i];
      *(outbuffer + offset + 0) = (u_statusi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 5; i++){
      this->timeout[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->timeout[i]);
      }
      for( uint32_t i = 0; i < 5; i++){
      union {
        bool real;
        uint8_t base;
      } u_statusi;
      u_statusi.base = 0;
      u_statusi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status[i] = u_statusi.real;
      offset += sizeof(this->status[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return UVC; };
    virtual const char * getMD5() override { return "75d711d503d84d7db5f91c05511e4107"; };

  };

  class uvc {
    public:
    typedef uvcRequest Request;
    typedef uvcResponse Response;
  };

}
#endif
