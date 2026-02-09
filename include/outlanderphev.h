/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2024 ZombieVerter contributors
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
 *
 * Outlander PHEV dual-motor inverter controller.
 * Composes front (OutlanderInverter) and rear (RearOutlanderInverter) motors
 * to provide EV-AWD and hybrid operating modes.
 */

#ifndef OUTLANDERPHEV_H
#define OUTLANDERPHEV_H

#include "inverter.h"
#include "outlanderinverter.h"
#include "rearoutlanderinverter.h"
#include "ice_state.h"

class OutlanderPHEV : public Inverter
{
public:
   OutlanderPHEV();
   void SetTorque(float torquePercent);
   float GetMotorTemperature();
   float GetInverterTemperature();
   float GetInverterVoltage();
   float GetMotorSpeed();
   int GetInverterState();
   void Task10Ms();
   void Task100Ms();
   void DecodeCAN(int id, uint32_t* data);
   void SetCanInterface(CanHardware* c);
   void DeInit();

   void SetSecondCanInterface(CanHardware* c);
   ICEState* GetICEState() { return &iceState; }

private:
   OutlanderInverter front;
   RearOutlanderInverter rear;
   ICEState iceState;
   bool hasSecondCan;
   float frontTorqueCmd;
   float rearTorqueCmd;
};

#endif // OUTLANDERPHEV_H
