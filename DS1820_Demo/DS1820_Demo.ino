
// Pete Wentworth, using up the lockdown time, March 2020.
// A demo to show my library in action, and optionally
// compare against some features of "the standard approach".

#include "AsyncTemperatures.h"

// Comment out this define to use my background reader "free solo"
#define CompareAgainstDallasLib

int numDevices = 2;
typedef byte DeviceAddress[8];

// Here I have just hard-wired the addresses of some of my sensors.
// If you don't have multiple sensors, perhaps just use the same address twice.
// If you don't know your device IDs, use DallasLib demo to find it!
DeviceAddress device[] =
{ {0x10, 0x31, 0x41, 0x26, 0x00, 0x08, 0x00, 0x0A},
  {0x28, 0xFF, 0x6F, 0x45, 0x80, 0x14, 0x02, 0x5E}
};

typedef byte ScratchPad[9];
ScratchPad sPad;

#ifdef CompareAgainstDallasLib

#include <OneWire.h>
#include <DallasTemperature.h>

// Data bus for OneWire is plugged into GPIO pin 12 on my Arduino Uno.  Or Pin 10 on a Mega
const int OneWireBusPin = 10;
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(OneWireBusPin);
DallasTemperature sensors(&oneWire);
#endif


void setup(void) {
  char buf[60];
  Serial.begin(115200);
  Serial.print("DS1820_Demo version 1.0 at "); Serial.print(__DATE__); Serial.print(" "); Serial.println(__TIME__);

#ifdef CompareAgainstDallasLib
  sensors.begin();
#endif

  myTemperatureSensors.begin();
}

const int None = 0;
const int Async = 1;
const int Dallas = 2;

/*
void loop(void) {

#ifdef CompareAgainstDallasLib
  senseAndCountFreeTime(Dallas);    // Use the Dallas lib, and see what we get.
#endif

  senseAndCountFreeTime(None);      // Don't use any sensing, we'll get an idea of raw loop speed
  senseAndCountFreeTime(Async);     // Use the Async lib, and see how many times we get around the loop

  delay(20000);
}
*/ 
//You could use this main loop for looking at timings.
void loop(void) {    
   myTemperatureSensors.doTestTimings(1000);
   delay(20000);
}



// The experiment senses each of the two sensors per second.
// Outside of that time, it loops around counting to get an idea of
// processing power available to this "main thread" while also having to
// measure each sensor once.

long freeTimeCount = 0;

void senseAndCountFreeTime(int sensingToUse) {

  long experimentStartedAt = millis();
  long lastExperimentStartedAt = experimentStartedAt;
  long realWorkCount = 0;
  double realWorkAnswer;
  bool dallasCompleted = false;

  int asyncState = 0;  // not started

  while (true)
  {
    long timeNow = millis();
    if (timeNow - experimentStartedAt >= 1000) break;  // Once your second is up, report results
    switch (sensingToUse) {
      case None: break;  // no sensing
      case Async: { // use async lib

          switch (asyncState) {
            case 0:  {  // start temp conversions
                myTemperatureSensors.convertAllTemperaturesAsync(); // start the ball rolling
                asyncState = 1;
              }
              break;
            case 1:  {  // wait for temp conversions, start scratchpad read on first sensor
                if (myTemperatureSensors.getStatus() == 0) { // If temp convesion is complete
                  myTemperatureSensors.readScratchpadAsync(device[0], sPad); // start reading first device into sPad
                  asyncState = 2;
                }
              }
              break;

            case 2: { // wait for first scratchpad read to complete, print it, and start getting scratchpad for next sensor
                if (myTemperatureSensors.getStatus() == 0) { // We got the scratchpad
                  printStuff(" AsyncReader ", device[0], sPad);
                  // start the next sensor scratchpad reading
                  myTemperatureSensors.readScratchpadAsync(device[1], sPad);
                  asyncState = 3;
                }
                break;

              case 3:  {  // wait for second scratchpad read to complete, print it, and we're done.
                  if (myTemperatureSensors.getStatus() == 0) { // We got this one done

                    printStuff(" AsyncReader ", device[1], sPad);
                    asyncState = 4;  // we're done completely
                  }
                }
                break;

              case 4: // nothing else to do
                break;
              }

          }
          break;
        }
        break;

#ifdef CompareAgainstDallasLib
      case Dallas: {
          // Use dallas lib
          if (!dallasCompleted) {
            // To make a fairer comparison, use the Async feature here
            sensors.setWaitForConversion(false);
            sensors.requestTemperatures();
            while (! sensors.isConversionComplete()) {  // While its not ready
              realWorkAnswer = doSomeRealWork(millis());
              realWorkCount++;
            }
            sensors.readScratchPad(device[0], sPad);
            printStuff(" DallasReader ", device[0], sPad);
            sensors.readScratchPad(device[1], sPad);
            printStuff(" DallasReader ", device[1], sPad);
            dallasCompleted = true;
          }
        }
        break;
#endif
    }
    realWorkAnswer = doSomeRealWork(millis());
    realWorkCount++;
  }

  // Once a full second is up, report what happened
  switch (sensingToUse) {

    case None: {
        Serial.print("With NO sensing:  realWorkCount execution count = "); Serial.println(realWorkCount);
      }
      break;
    case Async: {
        Serial.print("Using Async lib:  realWorkCount execution count = "); Serial.println(realWorkCount);
      }
      break;
    case Dallas: {
        Serial.print("Using Dallas lib: realWorkCount execution count = "); Serial.println(realWorkCount);
      }
      break;
  }
  Serial.println();
}

double doSomeRealWork(long seed)
{ // complicated enough so that GCC cannot optimize it away.
  return sqrt(abs(sin(seed)));
}

void printStuff( const char* label, DeviceAddress device, uint8_t *sPad)
{
  printAddress(device);
  printScratchPad(label, sPad);
  float tempC = myTemperatureSensors.getTempC(device, sPad);
  Serial.print("  TempC = "); Serial.println(tempC);
}

// print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// print labelled scratchpad contents
void printScratchPad(const char* label, uint8_t *sPad)
{
  char buf[16];
  Serial.print(label);
  for (int i = 0; i < 9; i++) {
    sprintf(buf, " %02X", sPad[i]);
    Serial.print(buf);
  }
}
