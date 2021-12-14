/*
   Copyright 2015-2018 Dirk-Willem van Gulik <dirkx@webweaving.org>
   Copyright 2020-2021 Hans Beerman <hans.beerman@xs4all.nl>
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

#ifndef ESP32
#error "The Node lintzaag uses an ESP32 based board (Olimex ESP32-PoE)!"
#endif

// An Olimex ESP32-PoE is used (default is POESP board (Aart board))
#define ESP32_PoE


// for debugging
// see DEBUGIT flag in platformio.ini

// i2C params
// Defined in platformio.ini:
// RFID_SDA_PIN=13
// RFID_SCL_PIN=16

#include <Arduino.h>
#include <PowerNodeV11.h>
#include <ACNode.h>
#include <RFID.h>   // NFC version
#include <SIG2.h>
#include <Cache.h>
#include <OptoDebounce.h>
#include <ButtonDebounce.h>
#include <EEPROM.h>
#include <WiFiUdp.h>
#include <NTP.h> // install NTP by Stefan Staub
#include <CurrentTransformerWithCallbacks.h>

//
// information about NTP.h, see: https://platformio.org/lib/show/5438/NTP
//

#include "MCP23017IO.h"
#include ".passwd.h"

/* If using WiFi instead of a fixed ethernet connection, and/or OTA:

Change .passwd.h in the /include directoy containing the following information

#pragma once

#define WIFI_NETWORK "YOUR_SSID"
#define WIFI_PASSWD "YOUR_PASSWD"

#define OTA_PASSWD "YOUR_OTA_PASSWD"
// This OTA_PASSWD must be the same as the one in platformio.ini
*/

// software version
#define SOFTWARE_VERSION "  V1.0.0.0 "

#define MACHINE "lintzaag"

// See https://mailman.makerspaceleiden.nl/mailman/private/deelnemers/2019-February/019837.html

// Introduced by alex - 2020-01-8

// Clear EEProm + Cache button
// Press BUT1 on Olimex ESP32 PoE module before (re)boot of node
// keep BUT1 pressed for at least 5 s
// After the release of BUT1 node will restart with empty EEProm and empty cache
#define CLEAR_EEPROM_AND_CACHE_BUTTON           (34)
#define CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED   (LOW)
#define MAX_WAIT_TIME_BUTTON_PRESSED            (4000)  // in ms

#define MAX_WAIT_FOR_CARD_TIME                  (20) // in s = 20 seconds

// for normal operation
#define MAX_ACTIVE_TIME                         (120) // in minutes = 2 hour
// for test
// #define MAX_ACTIVE_TIME                         (1) // in minutes = 1 minute
// for normal operation
#define MAX_IDLE_TIME                           (35 * 60 * 1000) // auto power off after 35 minutes of no use.
// for test
// #define MAX_IDLE_TIME                           (1 * 60 * 1000) // auto power off after 1 minute of no use.

#define CHECK_NFC_READER_AVAILABLE_TIME_WINDOW  (10000) // in ms  
#define GPIOPORT_I2C_RECOVER_SWITCH             (15)       

#define OPTO_COUPLER_INPUT1                     (32)
#define OPTO_COUPLER_INPUT2                     (33)
#define OPTO_COUPLER_INPUT3                     (36)

#define CURRENT_SAMPLES_PER_SEC                 (500) // one sample each 2 ms
#define CURRENT_THRESHOLD                       (0.15)  // Irms ~ 4A, with current transformer 1:1000 and resistor of 36 ohm

#define BLINK_NOT_CONNECTED                     (1000)	// blink on/of every 1000 ms
#define BLINK_ERROR                             (300) // blink on/of every 300 ms
#define BLINK_CHECKING_RFIDCARD                 (600) // blink on/of every 600 ms

CurrentTransformerWithCallbacks currentSensor = CurrentTransformerWithCallbacks(CURRENT_GPIO, CURRENT_SAMPLES_PER_SEC);

// 230VAC optocoupler
OptoDebounce opto1(OPTO_COUPLER_INPUT1); // wired to the 230VAC of the welding equipment, to detect if it is switched on or not

bool opto1IsOn = false;
bool previousOpto1IsOn = false;
bool opto1Error = false;
bool powerWhileSwitchedOffError = false;

#ifdef WIFI_NETWORK
ACNode node = ACNode(MACHINE, WIFI_NETWORK, WIFI_PASSWD);
#else
ACNode node = ACNode(MACHINE);
#endif

#define USE_CACHE_FOR_TAGS true
#define USE_NFC_RFID_CARD true

// NTP update window
#define NTP_UPDATE_WINDOW (60000) // in ms

RFID reader = RFID(USE_CACHE_FOR_TAGS, USE_NFC_RFID_CARD); // use tags are stored in cache, to allow access in case the MQTT server is down; also use NFC RFID card

MqttLogStream mqttlogStream = MqttLogStream();
TelnetSerialStream telnetSerialStream = TelnetSerialStream();

#ifdef OTA_PASSWD
OTA ota = OTA(OTA_PASSWD);
#endif

WiFiUDP wifiUDP;
NTP ntp(wifiUDP);

char ntpBootDateTimeStr[64];
unsigned long NTPUpdatedTime = 0;
int previousHour = -1;

char ntpCurrentDateTimeStr[64];
int currentHour;
int currentMinutes;
int currentSecs;
int currentDay;
int currentMonth;
int currentYear;

// LED aartLed = LED(SYSTEM_LED);    // defaults to the aartLed - otherwise specify a GPIO.

typedef enum {
  BOOTING,                  // starting up
  OUTOFORDER,               // device not functional.
  REBOOT,                   // forcefull reboot
  TRANSIENTERROR,           // hopefully goes away level error
  NOCONN,                   // sort of fairly hopless (though we can cache RFIDs!)
  NOTACTIVE,                // waiting for card.
  CHECKINGCARD,
  CLEARSTATUS,
  APPROVED,
  REJECTCARD,
  REJECTED,
  POWERED,
  ACTIVE,                   // running
} machinestates_t;


#define NEVER (0)

struct {
  const char * label;                   // name of this state
  LED::led_state_t ledState;            // flashing pattern for the aartLED. Zie ook https://wiki.makerspaceleiden.nl/mediawiki/index.php/Powernode_1.1.
  time_t maxTimeInMilliSeconds;         // how long we can stay in this state before we timeout.
  machinestates_t failStateOnTimeout;   // what state we transition to on timeout.
  unsigned long timeInState;
  unsigned long timeoutTransitions;
  unsigned long autoReportCycle;
} state[ACTIVE + 1] =
{
  { "Booting",                           LED::LED_ERROR,                      120 * 1000, REBOOT,                       0 },
  { "Module out of order",               LED::LED_ERROR,                      120 * 1000, REBOOT,                       5 * 60 * 1000 },
  { "Rebooting",                         LED::LED_ERROR,                      120 * 1000, REBOOT,                       0 },
  { "Transient Error",                   LED::LED_ERROR,                        5 * 1000, NOTACTIVE,                    5 * 60 * 1000 },
  { "No network",                        LED::LED_FLASH,                           NEVER, NOCONN,                       0 },
  { "Not active",                        LED::LED_IDLE,                            NEVER, NOTACTIVE,                    0 },
  { "Checking card",                     LED::LED_PENDING,                      7 * 1000, REJECTCARD,                   0 },
  { "Clear status",                      LED::LED_PENDING,                         NEVER, NOTACTIVE,                    0 },
  { "Approved card",                     LED::LED_PENDING, MAX_WAIT_FOR_CARD_TIME * 1000, CLEARSTATUS,                  0 },
  { "Reject card",                       LED::LED_ERROR,                        5 * 1000, REJECTED,                     0 },
  { "Rejected",                          LED::LED_ERROR,                        7 * 1000, CLEARSTATUS,                  0 },
  { "Powered - but idle",                LED::LED_ON,                              NEVER, NOTACTIVE,                    0 },
  { "Active (running)",                  LED::LED_ON,        MAX_ACTIVE_TIME * 60 * 1000, CLEARSTATUS,                  0 },
};

unsigned long laststatechange = 0, lastReport = 0;
static machinestates_t laststate = OUTOFORDER;
machinestates_t machinestate = BOOTING;

// to handle onconnect only once (only after reboot)
static bool firstOnConnectTime = true;

char reportStr[128];

unsigned long approvedCards = 0;
unsigned long rejectedCards = 0;

// For storing the local IP address of the node
IPAddress theLocalIPAddress;

unsigned long lastCheckNFCReaderTime = 0;

bool nodeWasConnected = false;

void checkClearEEPromAndCacheButtonPressed(void) {
  unsigned long ButtonPressedTime;
  unsigned long currentSecs;
  unsigned long prevSecs;
  bool firstTime = true;

  // check CLEAR_EEPROM_AND_CACHE_BUTTON pressed
  pinMode(CLEAR_EEPROM_AND_CACHE_BUTTON, INPUT);
  // check if button is pressed for at least 3 s
  Log.println("Checking if the button is pressed for clearing EEProm and cache");
  ButtonPressedTime = millis();  
  prevSecs = MAX_WAIT_TIME_BUTTON_PRESSED / 1000;
  Log.print(prevSecs);
  Log.print(" s");
  while (digitalRead(CLEAR_EEPROM_AND_CACHE_BUTTON) == CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED) {
    if ((millis() - ButtonPressedTime) >= MAX_WAIT_TIME_BUTTON_PRESSED) {
      if (firstTime == true) {
        Log.print("\rPlease release button");
        firstTime = false;
      }
    } else {
      currentSecs = (MAX_WAIT_TIME_BUTTON_PRESSED - millis()) / 1000;
      if ((currentSecs != prevSecs) && (currentSecs >= 0)) {
        Log.print("\r");
        Log.print(currentSecs);
        Log.print(" s");
        prevSecs = currentSecs;
      }
    }
  }
  if (millis() - ButtonPressedTime >= MAX_WAIT_TIME_BUTTON_PRESSED) {
    Log.print("\rButton for clearing EEProm and cache was pressed for more than ");
    Log.print(MAX_WAIT_TIME_BUTTON_PRESSED / 1000);
    Log.println(" s, EEProm and Cache will be cleared!");
    // Clear EEPROM
    EEPROM.begin(1024);
    wipe_eeprom();
    Log.println("EEProm cleared!");
    // Clear cache
    prepareCache(true);
    Log.println("Cache cleared!");
    // wait until button is released, than reboot
    while (digitalRead(CLEAR_EEPROM_AND_CACHE_BUTTON) == CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED) {
    // do nothing here
    }
    Log.println("Node will be restarted");
    // restart node
    ESP.restart();
  } else {
    Log.println("\rButton was not (or not long enough) pressed to clear EEProm and cache");
  }
}

void setup_GPIO() {
  // for recovery switch I2C
  pinMode(GPIOPORT_I2C_RECOVER_SWITCH, OUTPUT);
  digitalWrite(GPIOPORT_I2C_RECOVER_SWITCH, 0);
}

#define CONNECTION_ERROR_INTERVAL_TIME        (1000)
#define CONNECTION_ERROR_TIME                 (100)

bool connectionErrorIsActive = false;
unsigned long connectionErrorTime = 0;

void resetNFCReader() {
  if (USE_NFC_RFID_CARD) {
    pinMode(RFID_SCL_PIN, OUTPUT);
    digitalWrite(RFID_SCL_PIN, 0);
    pinMode(RFID_SDA_PIN, OUTPUT);
    digitalWrite(RFID_SDA_PIN, 0);
    digitalWrite(GPIOPORT_I2C_RECOVER_SWITCH, 1);
    delay(500);
    digitalWrite(GPIOPORT_I2C_RECOVER_SWITCH, 0);
    reader.begin();
  }
}

void checkNFCReaderAvailable(bool onlyShowError) {
  if (USE_NFC_RFID_CARD) {
    if (!reader.CheckPN53xBoardAvailable()) {
      // Error in communuication with RFID reader, try resetting communication
      Serial.println("Error in communication with RFID reader. Resetting communication\r");
      pinMode(RFID_SCL_PIN, OUTPUT);
      digitalWrite(RFID_SCL_PIN, 0);
      pinMode(RFID_SDA_PIN, OUTPUT);
      digitalWrite(RFID_SDA_PIN, 0);
      digitalWrite(GPIOPORT_I2C_RECOVER_SWITCH, 1);
      delay(500);
      digitalWrite(GPIOPORT_I2C_RECOVER_SWITCH, 0);
      reader.begin();
    } else {
      // No error
      if (!onlyShowError) {
        Serial.println("Reader is available!\r");
      }
    }
  }
}

void getCurrentNTPDateTime() {
  currentHour = ntp.hours();
  currentMinutes = ntp.minutes();
  currentSecs = ntp.seconds();
  currentDay = ntp.day();
  currentMonth = ntp.month();
  currentYear = ntp.year();
  snprintf(ntpCurrentDateTimeStr, sizeof(ntpCurrentDateTimeStr), "%4d-%02d-%02d %02d:%02d:%02d", currentYear, currentMonth, currentDay, currentHour, currentMinutes, currentSecs);
}

void initNTP(void) {
  ntp.ruleDST("CEST", Last, Sun, Mar, 2, 120); // last sunday in march 2:00, timezone +120min (+1 GMT + 1h summertime offset)
  ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60); // last sunday in october 3:00, timezone +60min (+1 GMT)
  ntp.begin();
  ntp.update();

  getCurrentNTPDateTime();

  snprintf(ntpBootDateTimeStr, sizeof(ntpBootDateTimeStr), "Last boot: %s", ntpCurrentDateTimeStr);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n\n");
  Serial.println("Booted: " __FILE__ " " __DATE__ " " __TIME__ );
  
  setup_GPIO();

  setup_MCP23017();

  relay1Off();

  checkClearEEPromAndCacheButtonPressed();

  node.set_mqtt_prefix("ac");
  node.set_master("master");

  node.onConnect([]() {
    Log.println("Connected");
    if (firstOnConnectTime == true) {
      firstOnConnectTime = false;
      machinestate = NOTACTIVE;
      if ((!opto1Error) && (!powerWhileSwitchedOffError)) {
        FET1BlinkOff();
        FET1Off();
      }
    }
  });
  node.onDisconnect([]() {
    Log.println("Disconnected");
    FET1BlinkOn(BLINK_NOT_CONNECTED);
    machinestate = NOCONN;
  });
  node.onError([](acnode_error_t err) {
    Log.printf("Error %d\n", err);
    machinestate = NOTACTIVE;
  });

 
  currentSensor.setOnLimit(CURRENT_THRESHOLD);
  currentSensor.onCurrentOn([]() {
    Serial.println("CurrentSensor = on (current detected > 4A (Irms)!)\r");
    if (machinestate < POWERED) {
      powerWhileSwitchedOffError = true;
      FET1BlinkOn(BLINK_ERROR);
      Log.println("Very strange - current observed while we are switched 'off'. Should not happen.");
    } else {
      if (machinestate != ACTIVE) {
        machinestate = ACTIVE;
        Log.println("Motor started");
      };
    };
  });

  currentSensor.onCurrentOff([]() {
    Serial.println("CurrentSensor = off (current detected < 4A (Irms)!\r");
    // We let the auto-power off on timeout do its work.
    if (machinestate > POWERED) {
      machinestate = POWERED;
      Log.println("Motor stopped");
    } else {
        if (machinestate < POWERED) {
          if (powerWhileSwitchedOffError) {
            powerWhileSwitchedOffError = false;
            FET1BlinkOff();            
          };
        }
    }
  });

  node.onApproval([](const char * machine) {
  if (!opto1Error && !powerWhileSwitchedOffError) {
      Debug.print("Got approve for using the bandzaag: \n\r");

      getCurrentNTPDateTime();

      Log.printf("%s => Approve received to use bandzaag\n\r", ntpCurrentDateTimeStr);    
      
      Debug.println(machine);
      if ((machinestate == NOTACTIVE) || (machinestate == CHECKINGCARD)) {
        approvedCards++;
        machinestate = APPROVED;
        Serial.print("User is approved!\n\r");
      }
    }
  });

  node.onDenied([](const char * machine) {
    if (!opto1Error && !powerWhileSwitchedOffError) {
      FET1BlinkOn(BLINK_ERROR);
      relay1Off();
      
      Debug.println("Got denied");
      Serial.print("User is denied!\n\r");

      getCurrentNTPDateTime();

      Log.printf("%s => Deny received for using the bandzaag\n\r", ntpCurrentDateTimeStr);    

      machinestate = REJECTED;
    }
  });

  // For test:  
  // node.set_report_period(10 * 1000);
  node.set_report_period(REPORT_PERIOD);
  node.onReport([](JsonObject  & report) {
    report["state"] = state[machinestate].label;

#ifdef OTA_PASSWD
    report["ota"] = true;
#else
    report["ota"] = false;
#endif
    snprintf(reportStr, sizeof(reportStr), "%lu", approvedCards);
    report["approved cards"] = reportStr;
    snprintf(reportStr, sizeof(reportStr), "%lu", rejectedCards);
    report["rejected cards"] = reportStr;

    theLocalIPAddress = node.localIP();
    report["IP_address"] = theLocalIPAddress.toString();
    report["Last_Reboot"] = ntpBootDateTimeStr;
  });

  reader.onSwipe([](const char * tag) -> ACBase::cmd_result_t {
    // avoid swithing messing with the swipe process
    if (opto1Error || powerWhileSwitchedOffError) {
      if (opto1Error) {
        Debug.printf("Ignoring a normal swipe - error: bandzaag is already powered! Must be manualy switched off first!");
      } else {
        if (powerWhileSwitchedOffError) {
          Debug.printf("Ignoring a normal swipe - error: power detected while relais is not switched on yet");
        }
      }
      checkNFCReaderAvailable(true);
      return ACBase::CMD_CLAIMED;
    } else {
      if ((machinestate == ACTIVE) || (machinestate == POWERED) || (machinestate == APPROVED)) {
        Debug.printf("Ignoring a normal swipe - bandzaag is already active, or powered, or RFID card is already approved.");
        checkNFCReaderAvailable(true);
        return ACBase::CMD_CLAIMED;
      }
    }

    // We'r declining so that the core library handle sending
    // an approval request, keep state, and so on.
    //
    Debug.printf("Detected a normal swipe.\n");
    checkNFCReaderAvailable(true);
    FET1BlinkOn(BLINK_CHECKING_RFIDCARD);
    machinestate = CHECKINGCARD;
  //  buzz = CHECK;
    return ACBase::CMD_DECLINE;
  });


  // This reports things such as FW version of the card; which can 'wedge' it. So we
  // disable it unless we absolutely positively need that information.
  //
  reader.set_debug(false);
  node.addHandler(&reader);
 
#ifdef OTA_PASSWD
  node.addHandler(&ota);
#endif

  Log.addPrintStream(std::make_shared<MqttLogStream>(mqttlogStream));

  auto t = std::make_shared<TelnetSerialStream>(telnetSerialStream);
  Log.addPrintStream(t);
  Debug.addPrintStream(t);

  // node.set_debug(true);
  // node.set_debugAlive(true);

// if Olimex ESP32-PoE board is used
#ifdef ESP32_PoE  
  node.begin(BOARD_OLIMEX);
#endif

  initNTP();

// if default, board (POESP, board Aart) is used
#ifndef ESP32_PoE
  node.begin();
#endif
  Log.println("Booted: " __FILE__ " " __DATE__ " " __TIME__ );
  Log.println(ntpBootDateTimeStr);

  resetNFCReader();
}

unsigned long now;
unsigned long lastCheckSwitch1 = 0;
bool currentResetButtonPressed = false;

void optocoupler_loop() {

  opto1.loop();

  if (opto1.state() == OptoDebounce::ON) { 
    opto1IsOn = true;
    if (!previousOpto1IsOn) {
      Serial.print("OptoCoupler = on\n\r");
      previousOpto1IsOn = true;
    }
    if (!opto1Error && (machinestate != ACTIVE) && (machinestate != POWERED)) {
      FET1BlinkOn(BLINK_ERROR);
      opto1Error = true;
    }
  } else {
    opto1IsOn = false;
    if (opto1Error) {
      if (!powerWhileSwitchedOffError) {
        FET1BlinkOff();
      }
      opto1Error = false;
    }
    if (previousOpto1IsOn) {
      Serial.print("OptoCoupler = off\n\r");
      previousOpto1IsOn = false;
    }
  }
}

void loop() {
  node.loop();

  FET1BlinkLoop();

  now = millis();

  optocoupler_loop();

  currentSensor.loop();

  if ((now - NTPUpdatedTime) > NTP_UPDATE_WINDOW) {
    ntp.update();
    NTPUpdatedTime = now;
  }

  if (USE_NFC_RFID_CARD) {
    if ((now - lastCheckNFCReaderTime) > CHECK_NFC_READER_AVAILABLE_TIME_WINDOW) {
      lastCheckNFCReaderTime = now;
      Serial.print("Check Reader Available\n\r");
      checkNFCReaderAvailable(false);
    }
  }

  if (laststate != machinestate) {
    Debug.printf("Changed from state <%s> to state <%s>\n",
                 state[laststate].label, state[machinestate].label);

    state[laststate].timeInState += (millis() - laststatechange) / 1000;
    laststate = machinestate;
    laststatechange = millis();
    return;
  }

  if (state[machinestate].maxTimeInMilliSeconds != NEVER &&
      ((millis() - laststatechange) > state[machinestate].maxTimeInMilliSeconds))
  {
    state[machinestate].timeoutTransitions++;

    laststate = machinestate;
    machinestate = state[machinestate].failStateOnTimeout;

    Log.printf("Time-out; transition from <%s> to <%s>\n",
               state[laststate].label, state[machinestate].label);
    return;
  };

  if (state[machinestate].autoReportCycle && \
      (millis() - laststatechange) > state[machinestate].autoReportCycle && \
      (millis() - lastReport) > state[machinestate].autoReportCycle)
  {
    Log.printf("State: %s now for %lu seconds", state[laststate].label, (millis() - laststatechange) / 1000);
    lastReport = millis();
  };

  switch (machinestate) {
    case REBOOT:
      node.delayedReboot();
      break;

    case NOTACTIVE:
    case CHECKINGCARD:
      break;
    case CLEARSTATUS:
      if (!opto1Error && !powerWhileSwitchedOffError) {
        relay1Off();
        FET1BlinkOff();
        machinestate = NOTACTIVE;
      }
      break;
    case APPROVED:
      if (!opto1Error && !powerWhileSwitchedOffError) {
        relay1On();
        FET1BlinkOff();
        FET1On();
        powerWhileSwitchedOffError = false;
        machinestate = POWERED;
      } else {
        machinestate = NOTACTIVE;
      }
      break;
    case REJECTCARD:
      if (!opto1Error && !powerWhileSwitchedOffError) {
        FET1BlinkOn(BLINK_ERROR);
        relay1Off();
      
        Debug.println("User not approved in time");
        Serial.print("User not approved in time, user is rejected!\n\r");

        getCurrentNTPDateTime();
        Log.printf("%s => Approve not received in time for using the bandzaag\n\r", ntpCurrentDateTimeStr);    

        machinestate = REJECTED;
      } else {
        machinestate = NOTACTIVE;
      }
      break;
    case REJECTED:
      break;
    case POWERED:
      if ((millis() - laststatechange) > MAX_IDLE_TIME) {
        Log.printf("Machine idle for too long - switching off.\n");
        machinestate = CLEARSTATUS;
      }
      break;
    case ACTIVE:
      break;
    case BOOTING:
    case OUTOFORDER:
    case TRANSIENTERROR:
    case NOCONN:
      break;
  };
}
