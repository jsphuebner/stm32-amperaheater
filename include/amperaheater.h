/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
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

#ifndef AMPERAHEATER_H
#define AMPERAHEATER_H

#include <heater.h>
#include "canhardware.h"
#include "picontroller.h"

class AmperaHeater : public Heater, CanCallback
{
   public:
      /** Default constructor */
      AmperaHeater(CanHardware *can);
      void SetTargetTemperature(float temp);
      void SetPower(uint16_t power);
      float GetTemperature() override;
      float GetPower() { return reportedPower; }
      void HandleClear();
      void HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc);

   private:
      PiControllerFloat tempController;
      uint16_t reportedPower;
      bool isAwake=false;
      void SendWakeup();
};

#endif // AMPERAHEATER_H
