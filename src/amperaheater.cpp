/*
 * This file is part of the stm32-... project.
 *
 * Copyright (C) 2021 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
//This class handles cabin heaters such as the Ampera heater (swcan on CAN3) or VW heater (LIN).

/*
Ampera heater info from : https://leafdriveblog.wordpress.com/2018/12/05/5kw-electric-heater/
http://s80ev.blogspot.com/2016/12/the-heat-is-on.html
https://github.com/neuweiler/GEVCUExtension/blob/develop/EberspaecherHeater.cpp

 * The heater communicates using J1939 protocol. It has to be "woken up" one time with a 0x100 message and then
 * must see a "keep alive" to stay active, which is the 0x621 message. The message repetition rate is between
 * 25 and 100ms intervals.

LV connector has SW CAN, ENABLE and GND pins.

Temperature sensor is 3.2Kohm at 21degC. It is NTC type.

The Eberspacher CAN version of their PTC liquid heater used in the Chevrolet Volt will work when used with a 33.33Kb SWCAN.
The data below is what we have found to be the minimum required to turn on and operate this heater.
This capture will operate the heater at approximately 33% of full power.
To command higher power, increase the value of message 0x10720099 byte 1 (it begins with byte 0) which is 3E below.
0C is the lowest value and results in about 600W
We saw full power heat when 85 was used as the value for byte 1 and that value will vary based upon inlet temperature.

The heater  returns 2 messages, one is empty, the other seems to contain the actual power/48 in byte 1
1047809D 3   00 08 57


ID,      LEN,D0,D1,D2,D3,D4,D5,D6,D7
621      8   00 52 00 00 00 00 00 00
102740CB 3   41 00 00
10720099 5   02 3E 00 00 00
102CC040 8   01 01 CF 18 00 51 06 6D
13FFE060 0
10242040 1   02

*/

#include "amperaheater.h"
#include "digio.h"
#include "anain.h"
#include "picontroller.h"

static uint8_t ampera_msg_cnt=0;

AmperaHeater::AmperaHeater(CanHardware* c)
{
   can = c;
   can->AddCallback(this);
   HandleClear();
   tempController.SetMinMaxY(0, 6000);
   tempController.SetGains(50, 5);
   tempController.SetCallingFrequency(100);
}

void AmperaHeater::SetTargetTemperature(float temp)
{
   tempController.SetRef(temp);
   float power = tempController.Run(GetTemperature());
}

void AmperaHeater::SetPower(uint16_t power)
{
   //if we are disabled do nothing but set isAwake to false for next wakeup ...
   if(power==0)
      isAwake = false;
   else//otherwise do everything
   {
      uint8_t data[8] = {0};

      if (!isAwake)
      {
         SendWakeup();
         isAwake = true;
         ampera_msg_cnt=0;
      }

      switch(ampera_msg_cnt)
      {
      case 0:
         DigIo::sw_mode0.Set(); // set normal mode
         //0x621,False,1,8,0,52,0,0,0,0,0,0
         //keep alive msg
         data[1] = 0x52;
         can->Send(0x621, data, 8);
         ampera_msg_cnt++;
         break;
      case 1:
         //0x102740CB, True,  3, 41,00,00,00,00,00,00,00 - cmd1
         data[0] = 0x41;
         can->Send(0x102740CB, data, 3);
         ampera_msg_cnt++;
         break;
      case 2:
         //0x10720099, True,  5, 02,3E,00,00,00,00,00,00 - control
         data[0] = 0x2;
         data[1] = power / 48;
         can->Send(0x10720099, data, 5);
         ampera_msg_cnt++;
         break;
      case 3:
         //0x102CC040, True,  8, 01,01,CF,18,00,51,06,6D - cmd2
         data[0] = 0x01;
         data[1] = 0x01;
         data[2] = 0xcf;
         data[3] = 0x18;
         data[4] = 0x00;
         data[5] = 0x51;
         data[6] = 0x06;
         data[7] = 0x6d;
         can->Send(0x102CC040, data, 8);
         ampera_msg_cnt++;
         break;
      case 4:
         //0x13FFE060, True,  0, 00,00,00,00,00,00,00,00 - cmd3
         can->Send(0x13FFE060, data, 0);
         ampera_msg_cnt++;
         break;
      case 5:
         // 0x10242040, True,  1, 02,00,00,00,00,00,00,00 - cmd4
         data[0] = 0x02;
         can->Send(0x10242040, data, 1);
         ampera_msg_cnt=0;
         break;
      }
   }
}

float AmperaHeater::GetTemperature()
{
   const uint16_t lut[] = { 2950, 2600, 2330, 2070, 1850, 1650, 1470, 1280, 1120, 960, 830, 710, 615, 520, 440, 370, 300 };
   const uint16_t tabSize = sizeof(lut) / sizeof(lut[0]);
   const uint16_t minTemp = 0;
   const uint16_t maxTemp = 80;
   const uint16_t step = 5;
   const uint16_t digit = AnaIn::temp.Get();
   uint16_t last;

   for (uint32_t i = 0; i < tabSize; i++)
   {
      uint16_t cur = lut[i];

      if (cur <= digit)
      {
         //if we are outside the lookup table range, return minimum
         if (0 == i) return minTemp;
         float a = cur - digit;
         float b = cur - last;
         float c = step * a / b;
         float d = (step * i) + minTemp;
         return d - c;
      }
      last = cur;
   }
   return maxTemp;
}

void AmperaHeater::HandleClear()
{
   can->RegisterUserMessage(0x1047809D);
}

void AmperaHeater::HandleRx(uint32_t canId, uint32_t data[2], uint8_t )
{
   uint8_t* bytes = (uint8_t*)data;
   if (canId == 0x1047809D)
   {
      reportedPower = bytes[1] * 48;
   }
}

#define FLASH_DELAY 9000
static void delay(void)
{
   int i;
   for (i = 0; i < FLASH_DELAY; i++)       /* Wait a bit. */
      __asm__("nop");
}

/*
 * Wake up all SW-CAN devices by switching the transceiver to HV mode and
 * sending the command 0x100 and switching the HV mode off again.
 */
void AmperaHeater::SendWakeup()
{
   uint8_t data[8] = { 0 };
   DigIo::sw_mode0.Clear();
   delay();
   // 0x100, False, 0, 00,00,00,00,00,00,00,00
   can->Send(0x100, data, 8);
   //may need delay here
   delay();
   DigIo::sw_mode0.Set();
}
