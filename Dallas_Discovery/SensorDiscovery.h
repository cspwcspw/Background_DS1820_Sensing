
// Pete Wentworth 2 April 2020.
// Enumerate Dallas devices / DS1820-type devices on a 1-Wire bus.
// When I finally understood the fiendishly clever device enumeration algorithm,
// I had to try it for myself.  Using the OneWire and DallasTemperature libraries
// are probably your better bet, but this is a nice companion to my AsyncTemperature
// library. Most of the explanation is on GitHub.
// The OneWire pin is hardwired.  I don't support parasitic mode yet.

#pragma once

#include <util/delay.h>   // For _delay_us() which is more accurate than delayMicroseconds

#define SEARCHROM       0xF0  // Initiates the next cycle of device discovery.

class SensorDiscovery
{
    // Wiring:
    // On a UNO, PORTB, bit 4 maps to pin 12.  Connect your 1-wire bus there.
    // On a Mega2560, PORTB bit 4 maps to pin 10.
    const byte busPinMask = 0b00010000;

// -------- Ignoring parasitic mode, there are only three valid "electrical" moves on a 1-wire bus:
#define pullBusLow()    DDRB |= busPinMask; PORTB &= ~busPinMask  
#define releaseBus()    DDRB &= ~busPinMask              // Set direction for INPUT, i.e. high impedance
#define sampleBus()    (((byte) PINB & busPinMask) != 0)
// --------

  private:
    byte *inputBuf;

#define setBitInID(i)       { inputBuf[i/8] |= (1 << (i%8)); }
#define unsetBitInID(i)     { inputBuf[i/8] &= (~(1 << (i%8))); }
#define isBitInID(i)        ((inputBuf[i/8] & (1 << (i%8))) != 0)

    byte fork[8];  // 64 bits indicate where "forks in the search" still require backtracking.

#define setForkPoint(i)     { fork[i/8] |= (1 << (i%8)); }
#define unsetForkPoint(i)   { fork[i/8] &= (~(1 << (i%8))); }
#define isForkPoint(i)      ((fork[i/8] & (1 << (i%8))) != 0)

    bool firstTime;

    int findLastForkPoint()
    {
      for (int i = 63; i >= 0; i--)
      {
        if (isForkPoint(i)) return i;
      }
      return -1;
    }

    byte readBit()
    {
      noInterrupts();  // Where timing windows are critical for the protocol, don't allow interrupts.
      pullBusLow();
      _delay_us(6);
      releaseBus();
      _delay_us(9);
      byte b = sampleBus();
      interrupts();
      _delay_us(60);
      return b;
    }

    // 0 means success, 1 means no devices on the bus.
    byte Reset() {
      // Specs, pg 2 of  http://ww1.microchip.com/downloads/en/appnotes/01199a.pdf
      // Drive bus low, delay 480 μs.
      // Release bus, delay 70 μs.
      // Sample bus: 0 = device(s) present,
      //             1 = no device present
      // Delay 410 μs.

      pullBusLow();
      _delay_us(480);
      noInterrupts();
      releaseBus();
      _delay_us(70);
      byte b = sampleBus();
      interrupts();
      _delay_us(410);
      return b;
    }

    void sendBit(byte b)
    {
      // Serial.print("Send bit "); Serial.println(b);
      if (b == 1) {
        // Specs, pg 2 of  http://ww1.microchip.com/downloads/en/appnotes/01199a.pdf
        // Drive bus low, delay 6 μs.
        // Release bus, delay 64 μs
        noInterrupts();
        pullBusLow();
        _delay_us(6);
        releaseBus();
        interrupts();
        _delay_us(64);
      }
      else {
        // Specs, pg 2 of  http://ww1.microchip.com/downloads/en/appnotes/01199a.pdf
        // Drive bus low, delay 60 μs.
        // Release bus, delay 10 μs.
        pullBusLow();
        _delay_us(60);
        releaseBus();
        _delay_us(10);
      }
    }

    void sendByte(byte b)
    {
      for (int i = 0; i < 8; i++)
      {
        sendBit(b & 0x01);
        b >>= 1;
      }
    }


  public:

    // Call this to start a new search.  Supply a buffer at least 8 bytes long.
    void begin(byte *deviceID)
    {
      inputBuf = deviceID;
      memset(inputBuf, 0, 8);
      memset(fork, 0, 8);
      firstTime = true;
    }

    // returns 0 for success, 1 means no more devices to find. Your buffer contains the deviceID
    byte findNextDevice()
    {
      int frozenTreeDepth;     // Up to this depth, we just follow the ID, then only we have freedom to explore

      if (firstTime)
      { firstTime = false;
        frozenTreeDepth = -1;
      }
      else {
        // The initialization is a bit different when we have to pick up on the old tree-search.
        frozenTreeDepth = findLastForkPoint();
        if (frozenTreeDepth < 0) return 1;  // No more devices to find.

        for (int i = frozenTreeDepth + 1; i < 64; i++) {
          unsetBitInID(i);               // Clear all the bits to the right of the frozenTreeDepth in the ID
        }
        unsetForkPoint(frozenTreeDepth);
        setBitInID(frozenTreeDepth);     // force the search to go to the right at this point.
        frozenTreeDepth++;
      }

      byte chooseRight = 42;
      byte resp = Reset();
      if (resp != 0) return resp;
      sendByte(SEARCHROM);
      for (int searchDepth = 0; searchDepth < 64; searchDepth++)
      {
        byte b1 = readBit();
        byte b0 = readBit();
        byte response = (b1 << 1) | b0;

        switch (response) {   // sort out what to do for each of the four possible response scenarios


          case 2:  {   // 10  Everyone still in contention sent 1 first, then sent complement 0.
              chooseRight = 1;
            }
            break;

          case 1:  {  // 01  Everyone still in contention likes the 0.
              chooseRight = 0;
            }
            break;

          case 0:  // Readings 00. "Sensors to the left of me, sensors to the right, I'm stuck in the middle with you."
            {
              if (searchDepth <= frozenTreeDepth)  // then whatever the ID bit says is the direction we take down the tree.
              {
                chooseRight = isBitInID(searchDepth);
              }
              else {
                chooseRight = 0;
                setForkPoint(searchDepth);  // But set up for a backtrack to later go the other way.
              }
            }
            break;

          case 3:  // Readings 11. No sensors out there. Did they fall off the bus?
            {
              return 2;  // return some "ghost" code, as the other library described it.
            }
            break;
        }

        // Every time the master sends a bit, non-matching sensors drop out of the running.  So on each
        // tree-traversal from the top we're trying to eliminate all contenders so that we finish up with
        // the ROM ID of just one sensor.

        if (chooseRight == 1)  {
          setBitInID(searchDepth);
          sendBit(1);
        }
        else {
          unsetBitInID(searchDepth);
          sendBit(0);
        }

      } // end of For loop, when tree depth has reached 64, we've succeeded.
      return 0;
    }

};
