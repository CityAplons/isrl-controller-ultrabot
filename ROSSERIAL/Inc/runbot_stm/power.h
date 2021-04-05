#ifndef _ROS_runbot_stm_power_h
#define _ROS_runbot_stm_power_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace runbot_stm
{

  class power : public ros::Msg
  {
    public:
      typedef uint8_t _battery_state_type;
      _battery_state_type battery_state;
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef int16_t _current_type;
      _current_type current;
      typedef float _power_consumption_type;
      _power_consumption_type power_consumption;

    power():
      battery_state(0),
      voltage(0),
      current(0),
      power_consumption(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->battery_state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->battery_state);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_current;
      u_current.real = this->current;
      *(outbuffer + offset + 0) = (u_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_power_consumption;
      u_power_consumption.real = this->power_consumption;
      *(outbuffer + offset + 0) = (u_power_consumption.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_power_consumption.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_power_consumption.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_power_consumption.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->power_consumption);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->battery_state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->battery_state);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
      union {
        int16_t real;
        uint16_t base;
      } u_current;
      u_current.base = 0;
      u_current.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->current = u_current.real;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_power_consumption;
      u_power_consumption.base = 0;
      u_power_consumption.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_power_consumption.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_power_consumption.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_power_consumption.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->power_consumption = u_power_consumption.real;
      offset += sizeof(this->power_consumption);
     return offset;
    }

    virtual const char * getType() override { return "runbot_stm/power"; };
    virtual const char * getMD5() override { return "3e17f18495d90b28ad85608062e408f2"; };

  };

}
#endif
