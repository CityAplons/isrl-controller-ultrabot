#ifndef _ROS_SERVICE_relay_h
#define _ROS_SERVICE_relay_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace runbot_stm
{

static const char RELAY[] = "runbot_stm/relay";

  class relayRequest : public ros::Msg
  {
    public:
      typedef bool _relay1_type;
      _relay1_type relay1;
      typedef bool _relay2_type;
      _relay2_type relay2;
      typedef bool _relay3_type;
      _relay3_type relay3;
      typedef bool _relay4_type;
      _relay4_type relay4;
      typedef bool _relay5_type;
      _relay5_type relay5;
      typedef bool _relay6_type;
      _relay6_type relay6;
      typedef bool _relay7_type;
      _relay7_type relay7;

    relayRequest():
      relay1(0),
      relay2(0),
      relay3(0),
      relay4(0),
      relay5(0),
      relay6(0),
      relay7(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_relay1;
      u_relay1.real = this->relay1;
      *(outbuffer + offset + 0) = (u_relay1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relay1);
      union {
        bool real;
        uint8_t base;
      } u_relay2;
      u_relay2.real = this->relay2;
      *(outbuffer + offset + 0) = (u_relay2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relay2);
      union {
        bool real;
        uint8_t base;
      } u_relay3;
      u_relay3.real = this->relay3;
      *(outbuffer + offset + 0) = (u_relay3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relay3);
      union {
        bool real;
        uint8_t base;
      } u_relay4;
      u_relay4.real = this->relay4;
      *(outbuffer + offset + 0) = (u_relay4.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relay4);
      union {
        bool real;
        uint8_t base;
      } u_relay5;
      u_relay5.real = this->relay5;
      *(outbuffer + offset + 0) = (u_relay5.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relay5);
      union {
        bool real;
        uint8_t base;
      } u_relay6;
      u_relay6.real = this->relay6;
      *(outbuffer + offset + 0) = (u_relay6.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relay6);
      union {
        bool real;
        uint8_t base;
      } u_relay7;
      u_relay7.real = this->relay7;
      *(outbuffer + offset + 0) = (u_relay7.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relay7);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_relay1;
      u_relay1.base = 0;
      u_relay1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relay1 = u_relay1.real;
      offset += sizeof(this->relay1);
      union {
        bool real;
        uint8_t base;
      } u_relay2;
      u_relay2.base = 0;
      u_relay2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relay2 = u_relay2.real;
      offset += sizeof(this->relay2);
      union {
        bool real;
        uint8_t base;
      } u_relay3;
      u_relay3.base = 0;
      u_relay3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relay3 = u_relay3.real;
      offset += sizeof(this->relay3);
      union {
        bool real;
        uint8_t base;
      } u_relay4;
      u_relay4.base = 0;
      u_relay4.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relay4 = u_relay4.real;
      offset += sizeof(this->relay4);
      union {
        bool real;
        uint8_t base;
      } u_relay5;
      u_relay5.base = 0;
      u_relay5.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relay5 = u_relay5.real;
      offset += sizeof(this->relay5);
      union {
        bool real;
        uint8_t base;
      } u_relay6;
      u_relay6.base = 0;
      u_relay6.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relay6 = u_relay6.real;
      offset += sizeof(this->relay6);
      union {
        bool real;
        uint8_t base;
      } u_relay7;
      u_relay7.base = 0;
      u_relay7.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relay7 = u_relay7.real;
      offset += sizeof(this->relay7);
     return offset;
    }

    virtual const char * getType() override { return RELAY; };
    virtual const char * getMD5() override { return "c362b6eec2a47a5d7dc788338e9ef5b3"; };

  };

  class relayResponse : public ros::Msg
  {
    public:
      uint8_t timeout[7];
      bool status[7];

    relayResponse():
      timeout(),
      status()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 7; i++){
      *(outbuffer + offset + 0) = (this->timeout[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->timeout[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
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
      for( uint32_t i = 0; i < 7; i++){
      this->timeout[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->timeout[i]);
      }
      for( uint32_t i = 0; i < 7; i++){
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

    virtual const char * getType() override { return RELAY; };
    virtual const char * getMD5() override { return "f43ec10cb8a78b28246677b9fcd7be49"; };

  };

  class relay {
    public:
    typedef relayRequest Request;
    typedef relayResponse Response;
  };

}
#endif
