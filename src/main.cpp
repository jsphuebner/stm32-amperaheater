/*
 * This file is part of the stm32-template project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
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
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include "stm32_can.h"
#include "canmap.h"
#include "cansdo.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "amperaheater.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "terminalcommands.h"
#include "sdocommands.h"

#define PRINT_JSON 0

static Stm32Scheduler* scheduler;
static CanHardware* can;
static CanMap* canMap;
static Heater* heater;

//sample 100ms task
static void Ms100Task(void)
{
   int power = Param::GetInt(Param::maxpower);

   DigIo::led_out.Toggle();
   //The boot loader enables the watchdog, we have to reset it
   //at least every 2s or otherwise the controller is hard reset.
   iwdg_reset();
   //Calculate CPU load. Don't be surprised if it is zero.
   float cpuLoad = scheduler->GetCpuLoad();
   //This sets a fixed point value WITHOUT calling the parm_Change() function
   Param::SetFloat(Param::cpuload, cpuLoad / 10);

   //heater->SetPower(power);
   heater->SetTargetTemperature(Param::GetFloat(Param::targetemp));

   if (power > 500)
      DigIo::pump.Set();
   else
      DigIo::pump.Clear();

   Param::SetInt(Param::tempraw, AnaIn::temp.Get());
   Param::SetFloat(Param::temperature, heater->GetTemperature());
   Param::SetFloat(Param::power, heater->GetPower());

   canMap->SendAll();
}


/** This function is called when the user changes a parameter */
void Param::Change(Param::PARAM_NUM paramNum)
{
   switch (paramNum)
   {
   default:
      //Handle general parameter changes here. Add paramNum labels for handling specific parameters
      break;
   }
}

//Whichever timer(s) you use for the scheduler, you have to
//implement their ISRs here and call into the respective scheduler
extern "C" void tim2_isr(void)
{
   scheduler->Run();
}

extern "C" int main(void)
{
   extern const TERM_CMD termCmds[];

   clock_setup(); //Must always come first
   rtc_setup();
   ANA_IN_CONFIGURE(ANA_IN_LIST);
   DIG_IO_CONFIGURE(DIG_IO_LIST);
   AnaIn::Start(); //Starts background ADC conversion via DMA
   write_bootloader_pininit(); //Instructs boot loader to initialize certain pins

   nvic_setup(); //Set up some interrupts
   parm_load(); //Load stored parameters

   Stm32Scheduler s(TIM2); //We never exit main so it's ok to put it on stack
   scheduler = &s;
   //Initialize CAN1, including interrupts. Clock must be enabled in clock_setup()
   Stm32Can c(CAN1, (CanHardware::baudrates)Param::GetInt(Param::canspeed));
   Stm32Can c2(CAN2, CanHardware::Baud33);
   CanMap cm(&c);
   CanSdo sdo(&c, &cm);
   AmperaHeater ht(&c2);
   sdo.SetNodeId(5); //Set node ID for SDO access e.g. by wifi module
   //store a pointer for easier access
   can = &c;
   canMap = &cm;
   heater = &ht;

   //This is all we need to do to set up a terminal on USART3
   Terminal t(USART3, termCmds);
   TerminalCommands::SetCanMap(canMap);
   //Tell SDO class which CAN map to operate on
   SdoCommands::SetCanMap(&cm);

   s.AddTask(Ms100Task, 100);

   //backward compatibility, version 4 was the first to support the "stream" command
   Param::SetInt(Param::version, 4);
   Param::Change(Param::PARAM_LAST); //Call callback one for general parameter propagation

   //Now all our main() does is running the terminal
   //All other processing takes place in the scheduler or other interrupt service routines
   //The terminal has lowest priority, so even loading it down heavily will not disturb
   //our more important processing routines.
   while(1)
   {
      char c = 0;
      CanSdo::SdoFrame* sdoFrame = sdo.GetPendingUserspaceSdo();

      t.Run();

      if (sdo.GetPrintRequest() == PRINT_JSON)
      {
         TerminalCommands::PrintParamsJson(&sdo, &c);
      }

      if (0 != sdoFrame)
      {
         SdoCommands::ProcessStandardCommands(sdoFrame);
         sdo.SendSdoReply(sdoFrame);
      }
   }


   return 0;
}

