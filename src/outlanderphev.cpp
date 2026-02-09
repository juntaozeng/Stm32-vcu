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
 * Manages both front and rear Outlander inverters simultaneously.
 *
 * Supported modes:
 *   EV_Only     - Front motor only (same as original Outlander inverter)
 *   EV_AWD      - Both front and rear motors, configurable torque split
 *   Hybrid_Series   - ICE drives front generator, rear motor traction
 *   Hybrid_Parallel - ICE + front motor + rear motor all driving
 *   Charge_Hold     - ICE charges battery via front motor, rear for traction
 */

#include "outlanderphev.h"
#include "my_math.h"
#include "params.h"
#include "param_prj.h"
#include "OutlanderHeartBeat.h"

OutlanderPHEV::OutlanderPHEV()
   : hasSecondCan(false), frontTorqueCmd(0), rearTorqueCmd(0)
{
}

void OutlanderPHEV::SetCanInterface(CanHardware* c)
{
   // First CAN interface goes to front motor
   front.SetCanInterface(c);
   OutlanderHeartBeat::SetCanInterface(c); // register for dual-CAN heartbeat
   can = c;
}

void OutlanderPHEV::SetSecondCanInterface(CanHardware* c)
{
   // Second CAN interface goes to rear motor
   rear.SetCanInterface(c);
   hasSecondCan = true;
}

void OutlanderPHEV::DeInit()
{
   hasSecondCan = false;
   frontTorqueCmd = 0;
   rearTorqueCmd = 0;
   iceState.RequestStop();
}

void OutlanderPHEV::SetTorque(float torquePercent)
{
   int driveMode = Param::GetInt(Param::DriveMode);
   float frontSplit = Param::GetFloat(Param::FrontRearSplit) / 100.0f;
   float rearSplit = 1.0f - frontSplit;
   float hybridDerate = Param::GetFloat(Param::HybridDerate) / 100.0f;

   switch (driveMode)
   {
   case DRV_EV_ONLY:
      // Front motor only — classic single-motor Outlander behavior
      frontTorqueCmd = torquePercent;
      rearTorqueCmd = 0;
      break;

   case DRV_EV_AWD:
      // Both motors, configurable torque split
      frontTorqueCmd = torquePercent * frontSplit;
      rearTorqueCmd = torquePercent * rearSplit;
      break;

   case DRV_HYBRID_SERIES:
      if (iceState.IsRunning())
      {
         // ICE drives front motor as generator (negative torque = generating)
         // Rear motor provides all traction
         frontTorqueCmd = -hybridDerate * 100.0f; // generating at configured level
         rearTorqueCmd = torquePercent;
      }
      else if (iceState.IsCranking())
      {
         // During cranking, front motor cranks ICE at low torque
         frontTorqueCmd = Param::GetFloat(Param::ICE_CrkTrq);
         rearTorqueCmd = 0; // hold position during crank
      }
      else
      {
         // ICE not running, fall back to EV-AWD
         frontTorqueCmd = torquePercent * frontSplit;
         rearTorqueCmd = torquePercent * rearSplit;
      }
      break;

   case DRV_HYBRID_PARALLEL:
      if (iceState.IsRunning())
      {
         // ICE adds to front axle mechanically.
         // Reduce front electric torque to prevent over-acceleration.
         // Total front = ICE_contribution (unknown) + electric front
         // Strategy: reduce front electric by hybridDerate amount
         frontTorqueCmd = torquePercent * frontSplit * (1.0f - hybridDerate);
         rearTorqueCmd = torquePercent * rearSplit;
      }
      else if (iceState.IsCranking())
      {
         frontTorqueCmd = Param::GetFloat(Param::ICE_CrkTrq);
         rearTorqueCmd = 0;
      }
      else
      {
         // ICE not running, fall back to EV-AWD
         frontTorqueCmd = torquePercent * frontSplit;
         rearTorqueCmd = torquePercent * rearSplit;
      }
      break;

   case DRV_CHARGE_HOLD:
      if (iceState.IsRunning())
      {
         // Front motor generates at max configured rate
         frontTorqueCmd = -hybridDerate * 100.0f;
         // Rear motor provides minimal traction
         rearTorqueCmd = torquePercent * 0.5f;
      }
      else if (iceState.IsCranking())
      {
         frontTorqueCmd = Param::GetFloat(Param::ICE_CrkTrq);
         rearTorqueCmd = 0;
      }
      else
      {
         frontTorqueCmd = torquePercent * frontSplit;
         rearTorqueCmd = torquePercent * rearSplit;
      }
      break;

   default:
      frontTorqueCmd = torquePercent;
      rearTorqueCmd = 0;
      break;
   }

   // Clamp torque values
   if (frontTorqueCmd > 100.0f) frontTorqueCmd = 100.0f;
   if (frontTorqueCmd < -100.0f) frontTorqueCmd = -100.0f;
   if (rearTorqueCmd > 100.0f) rearTorqueCmd = 100.0f;
   if (rearTorqueCmd < -100.0f) rearTorqueCmd = -100.0f;

   // Send to motors
   front.SetTorque(frontTorqueCmd);
   if (hasSecondCan)
   {
      rear.SetTorque(rearTorqueCmd);
   }

   // Publish torque split to display
   Param::SetFloat(Param::FrontTorq, frontTorqueCmd);
   Param::SetFloat(Param::RearTorq, rearTorqueCmd);
}

float OutlanderPHEV::GetMotorTemperature()
{
   float ft = front.GetMotorTemperature();
   if (hasSecondCan)
   {
      float rt = rear.GetMotorTemperature();
      return MAX(ft, rt);
   }
   return ft;
}

float OutlanderPHEV::GetInverterTemperature()
{
   float ft = front.GetInverterTemperature();
   if (hasSecondCan)
   {
      float rt = rear.GetInverterTemperature();
      return MAX(ft, rt);
   }
   return ft;
}

float OutlanderPHEV::GetInverterVoltage()
{
   return front.GetInverterVoltage();
}

float OutlanderPHEV::GetMotorSpeed()
{
   return front.GetMotorSpeed();
}

int OutlanderPHEV::GetInverterState()
{
   return front.GetInverterState();
}

void OutlanderPHEV::Task10Ms()
{
   front.Task10Ms();
   if (hasSecondCan)
   {
      rear.Task10Ms();
   }

   // Update ICE state machine
   iceState.Task10Ms();

   // Manage ICE start/stop based on drive mode
   int driveMode = Param::GetInt(Param::DriveMode);
   int opmode = Param::GetInt(Param::opmode);

   if (opmode == MOD_RUN)
   {
      bool wantsICE = (driveMode == DRV_HYBRID_SERIES ||
                       driveMode == DRV_HYBRID_PARALLEL ||
                       driveMode == DRV_CHARGE_HOLD);

      if (wantsICE && iceState.GetState() == ICEState::ICE_OFF)
      {
         iceState.RequestStart();
      }
      else if (!wantsICE && iceState.IsRunning())
      {
         iceState.RequestStop();
      }
   }
   else
   {
      // Not in RUN mode, ensure ICE is stopped
      if (iceState.GetState() != ICEState::ICE_OFF)
      {
         iceState.RequestStop();
      }
   }

   // Publish drive mode feedback
   if (iceState.IsRunning())
   {
      Param::SetInt(Param::DriveModeFB, driveMode);
   }
   else
   {
      // If ICE not running, actual mode is EV variant
      if (driveMode == DRV_EV_ONLY)
         Param::SetInt(Param::DriveModeFB, DRV_EV_ONLY);
      else
         Param::SetInt(Param::DriveModeFB, DRV_EV_AWD); // fallback is AWD
   }
}

void OutlanderPHEV::Task100Ms()
{
   front.Task100Ms();
   if (hasSecondCan)
   {
      rear.Task100Ms();
   }
}

void OutlanderPHEV::DecodeCAN(int id, uint32_t* data)
{
   // LIMITATION: The CAN callback doesn't indicate which bus a message
   // came from. Since front and rear use identical CAN IDs (0x289, 0x299,
   // 0x733), we can only reliably decode one set of feedback.
   // Front motor is primary — its speed/voltage/temp is what we report.
   // Rear motor feedback is not available in this architecture.
   // Torque commands (outbound) are unaffected — each motor gets correct
   // commands on its respective CAN bus.
   front.DecodeCAN(id, data);
}
