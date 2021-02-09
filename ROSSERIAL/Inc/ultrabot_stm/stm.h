#ifndef _ROS_ultrabot_stm_stm_h
#define _ROS_ultrabot_stm_stm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ultrabot_stm/power.h"

namespace ultrabot_stm
{

  class stm : public ros::Msg
  {
    public:
      uint32_t power_state_length;
      typedef ultrabot_stm::power _power_state_type;
      _power_state_type st_power_state;
      _power_state_type * power_state;
      typedef bool _emergency_type;
      _emergency_type emergency;
      uint16_t rangefinders[10];

    stm():
      power_state_length(0), st_power_state(), power_state(nullptr),
      emergency(0),
      rangefinders()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->power_state_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->power_state_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->power_state_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->power_state_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->power_state_length);
      for( uint32_t i = 0; i < power_state_length; i++){
      offset += this->power_state[i].serialize(outbuffer + offset);
      }
      union {
        bool real;
        uint8_t base;
      } u_emergency;
      u_emergency.real = this->emergency;
      *(outbuffer + offset + 0) = (u_emergency.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emergency);
      for( uint32_t i = 0; i < 10; i++){
      *(outbuffer + offset + 0) = (this->rangefinders[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rangefinders[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rangefinders[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t power_state_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      power_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      power_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      power_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->power_state_length);
      if(power_state_lengthT > power_state_length)
        this->power_state = (ultrabot_stm::power*)realloc(this->power_state, power_state_lengthT * sizeof(ultrabot_stm::power));
      power_state_length = power_state_lengthT;
      for( uint32_t i = 0; i < power_state_length; i++){
      offset += this->st_power_state.deserialize(inbuffer + offset);
        memcpy( &(this->power_state[i]), &(this->st_power_state), sizeof(ultrabot_stm::power));
      }
      union {
        bool real;
        uint8_t base;
      } u_emergency;
      u_emergency.base = 0;
      u_emergency.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->emergency = u_emergency.real;
      offset += sizeof(this->emergency);
      for( uint32_t i = 0; i < 10; i++){
      this->rangefinders[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->rangefinders[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->rangefinders[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ultrabot_stm/stm"; };
    virtual const char * getMD5() override { return "e48d2e416274cd327d2b3718f804f70d"; };

  };

}
#endif
