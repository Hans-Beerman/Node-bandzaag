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

#include <Wire.h>
#include "MCP23017IO.h"
#include <Arduino.h>
//#include "RFID.h"

Adafruit_MCP23X17 mcp;

#define INPUT_CHANGE_WINDOW     500 // in ms = time input changes are not taking into account
#define FET1_OUTPUT             11
#define FET2_OUTPUT             12
#define RELAY1_OUTPUT           8
#define RELAY2_OUTPUT           9
#define RELAY3_OUTPUT           10
#define SWITCH1_INPUT           0
#define SWITCH2_INPUT           1
#define SWITCH3_INPUT           2

bool relay1IsOn = false;
bool FET1IsOn = false;

unsigned long FET1BlinkTime;
unsigned long FET1BlinkStartTime;
bool FET1BlinkIsOn = false;
bool FET1BlinkFETIsOn = false;
bool setupReady = false;

void setup_MCP23017() {
    Wire.begin(RFID_SDA_PIN, RFID_SCL_PIN, RFID_I2C_FREQ);
    mcp.begin_I2C(MCP23XXX_ADDR, &Wire);
    mcp.pinMode(RELAY1_OUTPUT, OUTPUT);
    mcp.digitalWrite(RELAY1_OUTPUT, 0);
    mcp.pinMode(FET1_OUTPUT, OUTPUT);
    mcp.digitalWrite(FET1_OUTPUT, 0);
    setupReady = true;
}

void relay1On() {
    if (!setupReady) {
        setup_MCP23017();
    }
    if (!relay1IsOn) {
        mcp.digitalWrite(RELAY1_OUTPUT, 1);
        relay1IsOn = true;
    }
}

void relay1Off() {
    if (!setupReady) {
        setup_MCP23017();
    }
    if (relay1IsOn) {
        mcp.digitalWrite(RELAY1_OUTPUT, 0);
        relay1IsOn = false;
    }
}

void FET1On() {
    if (!setupReady) {
        setup_MCP23017();
    }
    if (!FET1IsOn) {
        mcp.digitalWrite(FET1_OUTPUT, 1);
        FET1IsOn = true;
    }
}

void FET1Off() {
    if (!setupReady) {
        setup_MCP23017();
    }
    if (FET1IsOn) {
        mcp.digitalWrite(FET1_OUTPUT, 0);
        FET1IsOn = false;
    }
}


void FET1BlinkOn(unsigned long aBlinkTime) {
    FET1On();
    FET1BlinkFETIsOn = true;
    FET1BlinkTime = aBlinkTime;
    FET1BlinkIsOn = true;
    FET1BlinkStartTime = millis();
}

void FET1BlinkOff() {
    FET1BlinkIsOn = false;
    FET1Off();
}

void FET1BlinkLoop() {
    if (FET1BlinkIsOn) {
        unsigned long now = millis();
        if ((now - FET1BlinkStartTime) >= FET1BlinkTime) {
            if (FET1BlinkFETIsOn) {
                FET1Off();
                FET1BlinkFETIsOn = false;
            } else {
                FET1On();
                FET1BlinkFETIsOn = true;
            }
            FET1BlinkStartTime = now;
        }
    }
}