/*
      Copyright 2021      Hans Beerman <hans.beerman@xs4all.nl>
                          Stichting Makerspace Leiden, the Netherlands.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#pragma once

#include <Wire.h>
#include "Adafruit_MCP23X17.h"

extern bool resetButtonPressed;

extern bool panicButtonPressed;

void setup_MCP23017();

void relay1On();

void relay1Off();

void FET1On();

void FET1Off();

void FET1BlinkOn(unsigned long aBlinkTime);

void FET1BlinkOff();

void FET1BlinkLoop();
