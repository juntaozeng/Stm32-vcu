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

#ifndef ICE_STATE_H
#define ICE_STATE_H

#include <stdint.h>
#include "iomatrix.h"
#include "params.h"

class ICEState
{
public:
   enum State { ICE_OFF = 0, ICE_CRANKING = 1, ICE_RUNNING = 2, ICE_STALLING = 3, ICE_ERROR = 4 };

   ICEState();
   void RequestStart();
   void RequestStop();
   void Task10Ms();
   bool IsRunning() { return state == ICE_RUNNING; }
   bool IsCranking() { return state == ICE_CRANKING; }
   State GetState() { return state; }
   void SetMG1SpeedFeedback(int16_t mg1, int16_t mg2, float planetaryRatio);

private:
   State state;
   uint16_t crankTimer;
   uint16_t stallTimer;
   bool startRequested;
   bool iceDetectedByMG1;
   int16_t mg1Speed;
   int16_t mg2Speed;
   float ratio;

   bool DetectICERunning();
   void SetStartOutput(bool on);
};

#endif // ICE_STATE_H
