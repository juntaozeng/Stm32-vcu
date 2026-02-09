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
 */

#include "ice_state.h"
#include "my_math.h"
#include "param_prj.h"

ICEState::ICEState()
   : state(ICE_OFF), crankTimer(0), stallTimer(0),
     startRequested(false), iceDetectedByMG1(false),
     mg1Speed(0), mg2Speed(0), ratio(2.6f)
{
}

void ICEState::RequestStart()
{
   if (state == ICE_OFF || state == ICE_ERROR)
   {
      startRequested = true;
      state = ICE_CRANKING;
      crankTimer = Param::GetInt(Param::ICE_CrkTmo) * 100; // convert seconds to 10ms ticks
      SetStartOutput(true);
   }
}

void ICEState::RequestStop()
{
   startRequested = false;
   SetStartOutput(false);

   if (state == ICE_RUNNING || state == ICE_CRANKING)
   {
      state = ICE_OFF;
   }
}

void ICEState::SetMG1SpeedFeedback(int16_t mg1, int16_t mg2, float planetaryRatio)
{
   mg1Speed = mg1;
   mg2Speed = mg2;
   ratio = planetaryRatio;
}

bool ICEState::DetectICERunning()
{
   int detectMode = Param::GetInt(Param::ICE_Detect);

   switch (detectMode)
   {
   case ICE_DET_DIGITAL:
   {
      DigIo* pin = IOMatrix::GetPin(IOMatrix::ICE_RUNNING_IN);
      return pin->Get();
   }
   case ICE_DET_MG1_SPEED:
   {
      // In EV-Free mode with ICE not running, MG1 speed should be:
      //   expected_mg1 = -(ratio) * mg2Speed
      // If ICE is running, MG1 speed deviates significantly from this
      int16_t expectedMG1 = (int16_t)(-(ratio) * mg2Speed);
      int16_t deviation = ABS(mg1Speed - expectedMG1);
      // If deviation > 500 RPM and MG2 is spinning, ICE is likely running
      return (deviation > 500 && ABS(mg2Speed) > 200);
   }
   case ICE_DET_CAN:
      // Reserved for future CAN-based detection
      return false;
   default:
      return false;
   }
}

void ICEState::SetStartOutput(bool on)
{
   DigIo* pin = IOMatrix::GetPin(IOMatrix::ICE_START_OUT);
   if (on)
      pin->Set();
   else
      pin->Clear();
}

void ICEState::Task10Ms()
{
   bool iceRunning = DetectICERunning();

   // Publish ICE detection to display
   Param::SetInt(Param::ICE_Status, state);

   switch (state)
   {
   case ICE_OFF:
      // Nothing to do, waiting for RequestStart()
      stallTimer = 0;
      break;

   case ICE_CRANKING:
      if (iceRunning)
      {
         state = ICE_RUNNING;
         crankTimer = 0;
      }
      else if (crankTimer > 0)
      {
         crankTimer--;
      }
      else
      {
         // Crank timeout expired without ICE starting
         SetStartOutput(false);
         state = ICE_ERROR;
      }
      break;

   case ICE_RUNNING:
      if (!startRequested)
      {
         // Stop was requested while running
         SetStartOutput(false);
         state = ICE_OFF;
      }
      else if (!iceRunning)
      {
         // ICE was running but detection lost — possible stall
         stallTimer++;
         if (stallTimer > 50) // 500ms debounce
         {
            state = ICE_STALLING;
         }
      }
      else
      {
         stallTimer = 0;
      }
      break;

   case ICE_STALLING:
      // ICE stalled unexpectedly — go to error
      SetStartOutput(false);
      startRequested = false;
      state = ICE_ERROR;
      break;

   case ICE_ERROR:
      // Stay in error until RequestStart() or RequestStop() resets
      SetStartOutput(false);
      break;
   }
}
