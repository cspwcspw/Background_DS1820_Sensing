
#pragma once

// An implementation of the 1-wire and Dallas 1820 temperature sensor, all mangled together.
// By Pete Wentworth, filling up his Social Distancing and LockDown Time, March 2020.

// Of course I have borrowed code freely from the OneWire library at
// http://www.pjrc.com/teensy/td_libs_OneWire.html
// and the DallasTemperature library.  Many thanks to all the contributors.

// The 1-wire protocol needs long blocking periods, and the "obvious" implementation requires
// much use ofdelayMicroseconds.  For example, resetting the bus before each read requires
// three successive wait times of 480us, 70us, and 410us.  But even more hectic, getting
// bus devices to perform a temperature conversion can require a delay of 750ms, (yes, millis)
// according to the DallasTemperature library. Devices that are busy hold the shared bus line
// low, so after initiating conversion one has to wait the for slowest device to finish.
//
// The key operations I want - initiate temperature conversions
// and then reading the scratchpad(s) (9 bytes, 72 bits)
// means the protocol cannot reset the bus, send a device address, and get the scratchpad
// value in less than about 10 millisecs, almost all spent blocking, and busy-waiting.
// We can't make the wire go faster.
//
// But a fun thing to do is to drive the protocol through an interpreter 
// that does not use long busy waits so your MCU can be doing its usual
// stuff in the meantime.

#include <util/delay.h>   // For _delay_us() which is more accurate than delayMicroseconds

const int stackSize = 20;  

// Various debugging and diagnostic stuff ---------------

byte stackSnapshot[stackSize];
const int LED_ALERT = 8;   // Can turn on a LED to show some condition occurred.


void showStackSnapshot()
{
  Serial.println("Stack   snapshot ");

  for (int i = 0; i < stackSize; i++) {
    if (i % 16 == 0) Serial.println();
    Serial.print(stackSnapshot[i]); Serial.print(" ");
  }
  Serial.println();
}

// I hang a scope on this pin and use it as a trigger
// to meaure things, especially useful for watching interval timing on the bus.
const int debugPin = 13;
void toggleDebugLine()
{
  static bool debugOutput = false;
  digitalWrite(debugPin, debugOutput);
  debugOutput = !debugOutput;
}

// ---------------------

// How things work:
// AsyncTemperatureReader is a "background process" that interprets pseudo instructions.
// Each doTimeslice() call does some non-blocking steps of the task
// and then yields with a "holdoff" value indicating that it can't do anything
// more until after the holdoff time has lapsed. Holdoff values are
// expressed as a number of tics on TIMER2.
//
// doTimeslice() is "pumped", i.e. called by, the TIMER2 ISR.
// (For debugging, we can suppress the timer interrupts and manually single-step calls to doTimeslice.)
// Before exiting, the ISR sets up new values for the TIMER2 registers to achieve the requested
// holdoff delay.  When the holdoff time has elapsed the ISR will run again, and give the
// interpreter its next timeslice.
//
// Pending code to be executed is stored in a bytecode stack.
// But some opcodes are macros. When popped off the stack and executed, they can expand
// into multiple more primitive instructions in the stack. These newly expanded instructions
// can also be macros. Each instruction, primitive or macro, may be followed by some
// operand bytes on the stack.

// As an example, the main operation I want is ReadScratchPad.  It expands into macros
// to Reset the device, Select the device by sending the device's addressID,
// and then Read back the device's scratchpad into a 9-byte buffer (supplied by the caller).
// This needs 72 bit-reads from the sensor. Each bit-read must drive the line low,
// pause, release the line, pause, then sample the line and store the bit, and then pause again.
// And so on. Short pauses are done inline with _delay_us(), but longer pauses
// end the current timeslice, with a return value that indicates the required holdoff delay.
// The instruction stack must be left in such a state that the next doTimeslice()
// call can pick up the flow of logic where we last left off.

// The onewire protocol is quite forgiving about timing.
// The master (that's us) always controls timing on the bus by driving the line low.
// then releasing the line to allow the pull-up to pull it high. Then any one of the
// attached slave devices can send info back to the master by pulling the shared
// bus-line low.  (Slaves must first be addressed and "given the bus", otherwise they
// do not talk). Loose protocol timings are acceptable.

// This code is very specific for my little sensors - some from the cheap Chinese
// 37-piece sensor kit.  So I have not yet catered for the more exotic features like
// CRC checks, parasitic power mode, different device resolutions, etc.
// That is left as a homework exercise for someone else :-)

// Wiring:
// On a UNO, PORTB, bit 4 maps to pin 12.  Connect your 1-wire bus there.
// On a Mega2560, PORTB bit 4 maps to pin 10.
// Connect your 1-wire bus there. And if you're going to also use the
// Dallas lib in the main program on a Mega, change the pin number.


// There are only three "electrical" things the master can do on a 1-wire bus:

const byte busPinMask = 0b00010000;
inline void pullBusLow()
{
  DDRB |= busPinMask;     // Set direction for OUTPUT
  PORTB &= ~busPinMask;   // Turn off the port bit to write a LOW
}

inline void releaseBus()
{
  DDRB &= ~busPinMask;   // Set direction for INPUT, i.e. high impedance
}

inline byte sampleBus()
{
  return (PINB & busPinMask) != 0;    // Read value of 1-wire pin.
}


// OneWire commands, only some are used here
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy EEPROM
#define READSCRATCH     0xBE  // Read EEPROM
#define WRITESCRATCH    0x4E  // Write to EEPROM
#define RECALLSCRATCH   0xB8  // Reload from last known
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition
#define SELECTDEVICE    0x55  // Specific device address to follow
#define SKIPROMWILDCARD 0xCC  // Skip device address, all devices must execute command
// Used in this lib ahead of STARTCONVO command.


// Status codes. 
const byte Success = 0x00;           // Zero means "ok, all done successfully"
const byte StillBusy = 0x01;
const byte NoDeviceOnBus = 0x02;     // bit set indicates no device on bus
const byte DevicesAreBusy = 0x04;    // bit set means we're still waiting for sensors to complete their conversions

// The opcodes for our interpreter...

const byte BusLow = 1;                // Drive the bus low
const byte ReadRemainingBits = 2;     // One-byte opand is bit index position [0..71] where next incoming bit is stored into scratchpad
const byte SendRemainingBits = 3;     // Two one-byte opands, the bitcount still to be sent, and the byte part still to be sent
const byte SendRemainingIDBytes = 4;  // One byte opand is index of next ID byte to send. It initiates SendRemainingBits for the next ID byte.
const byte WaitForBusRelease = 5;     // A test-then-yield operation repeatedly executed while waiting for all sensors to complete temperature conversions.
const byte BusRelease = 6;            // Allow the bus to float up to its pull-up value
const byte ClearBusyStatus = 7;       // Typically scheduled as the last instruction after the final delay before the interpreter becomes idle.
const byte BusSample = 8;             // Part of the Reset sequence requires sampling the bus to confirm that there are some devices present.
const byte TestTimings = 9;           // Two-byte opand is number of times to still repeat our test timing sequence
const byte ReadScratchPad = 10;       // Initiates reading of whole scratchpad.  No opand
const byte StartIDSend = 11;          // After Reset we have to address a specific device by ID in order to read its scratchpad
const byte Reset = 12;                // Initiates the 1-wire bus Reset.  It needs long delays, achieved here by ending the timeslice.
const byte Yield = 13;                // Ends the current timeslice.  The one byte opand is the number of TIMER2 tics we need to be inactive for.


// http://ww1.microchip.com/downloads/en/appnotes/01199a.pdf
// The protocol mandates certain delays (desired). I map those into
// TIMER2 counter values (it depends on my prescaler resolution).
// There are some inaccuracies and overheads because the ISR and the
// code here takes time to execute...  So measured these on the
// oscilloscope as best I could.
// We use /64 clock prescaler.  If you change that you will
// need to adjust these numbers.    Values must be <= 255.

//  DelayDesired     = OCR2A tics   //  Observed delay when measured
const byte Micros55  = 8;    //        todo
const byte Micros60  = 10;   //
const byte Micros64  = 10;   //
const byte Micros70  = 11;   //
const byte Micros410 = 96;   //
const byte Micros480 = 110;  //


class AsyncTemperatureReader
{

  public:
    int stackHighTide;  // Diagnostic

  private:
    byte theCode[stackSize];
    byte topOfStack;

    uint8_t status;

    byte receiveRegister;     // As bits arrive on the wire we store them in here.

    const uint8_t *deviceAddr;     // The device address of the sensor
    byte idByteIndex;        // Counts up from 0 to 8 as we send address bytes

    uint8_t *sPad;           // A 9-byte scratchpad for reading info from the sensor

    void flushStack() {
      topOfStack = 0;
    }

    void showStack(char * header) // diagnostic
    {
      Serial.print(header); Serial.print(" topOfStack");  Serial.println(topOfStack);
    }

    void push(byte opCode)
    { // Pre: interrupts already disabled;

      if (topOfStack >= stackSize) {
        digitalWrite(LED_ALERT, HIGH);
        Serial.println("Stack overflow");
        // snapshot the stack
        for (int i = 0; i < stackSize; i++) {
          stackSnapshot[i] = 0xFF;
        }
        for (int i = 0; i < topOfStack; i++) {
          stackSnapshot[i] = theCode[i];
        }
        // flushStack();
        return;
      }

      theCode[topOfStack] = opCode;
      topOfStack++;

      if (topOfStack >= stackHighTide) {
        stackHighTide = topOfStack;

        // snapshot the stack
        for (int i = 0; i < stackSize; i++) {
          stackSnapshot[i] = 0xFF;
        }
        for (int i = 0; i < topOfStack; i++) {
          stackSnapshot[i] = theCode[i];
        }
      }
    }

    inline byte pop()
    {
      return theCode[--topOfStack];
    }

    inline void pushSendOneByte(byte b)
    { push(b);
      push(8);
      push(SendRemainingBits);
    }

    void YieldFor(byte numberOfTics)
    {
      push(numberOfTics);
      push(Yield);
    }


  public:

    uint8_t doTimeslice()
    {

      // Pre: interrupts are disabled.
      do {

        if (topOfStack == 0) {   // If nothing to do, just keep slowly idling by ticking the counter over
          return 255;   // Maximum holdoff
        }

        byte opCode = theCode[--topOfStack];

        switch (opCode) {     // Now execute the primitive opCode

          case BusLow:
            {
              pullBusLow();
            }
            break;

          case  BusRelease:
            {
              releaseBus();
              _delay_us(10);
            }
            break;

          case SendRemainingBits: {
              byte bitsToGo = pop();
              byte toSend = pop();
              byte theBitToSend = toSend & 0x01;
              if (--bitsToGo  > 0) {
                toSend >>= 1;
                push(toSend);
                push(bitsToGo);
                push(SendRemainingBits);
              }

              if (theBitToSend == 1) {
                // Specs, pg 2 of  http://ww1.microchip.com/downloads/en/appnotes/01199a.pdf
                // Drive bus low, delay 6 μs.
                // Release bus, delay 64 μs
                pullBusLow();
                _delay_us(6);
                releaseBus();
                YieldFor(Micros64);
              }
              else {
                // Specs, pg 2 of  http://ww1.microchip.com/downloads/en/appnotes/01199a.pdf
                // Drive bus low, delay 60 μs.
                // Release bus, delay 10 μs.
                pullBusLow();
                push(BusRelease);
                YieldFor(Micros60);
              }
            }
            break;

          case  ClearBusyStatus: {
              status  &= (~StillBusy);
            }
            break;

          case ReadScratchPad: {
              // Push what we need to do onto the stack, back to front...
              push(Reset);

              receiveRegister = 0;
              push(0);              // index of next bit to store [0..72]
              push(ReadRemainingBits);
              pushSendOneByte(READSCRATCH);
              push(StartIDSend);
              push(Reset);
            }
            break;

          case ReadRemainingBits:
            {
              //   Specs, pg 2 of  http://ww1.microchip.com/downloads/en/appnotes/01199a.pdf
              //   Drive bus low, delay 6 μs.
              //   Release bus, delay 9 μs.
              //   Sample bus to read bit from slave.
              //   Delay 55 μs.


              digitalWrite(debugPin, LOW);
              pullBusLow();
              _delay_us(6);
              releaseBus();
              _delay_us(9);
              int thisBit = sampleBus();
              digitalWrite(debugPin, HIGH);
              // Shift the new bit into the receive register
              byte bitPos = pop();          // a value 0..71
              if (thisBit)  { // we need to store the 1 bit
                byte mask = 0x01 << (bitPos % 8); // create a mask
                receiveRegister |= mask;
              }
              bitPos++;   // advance for next time
              if (bitPos % 8 == 0) // but wait, have we just completed a whole byte?
              {
                byte sPadIndx = (bitPos / 8) - 1;
                sPad[sPadIndx] = receiveRegister;
                receiveRegister = 0;
                // But wait again. Have we just completed the whole scratchpad read?
                if (bitPos < 72) { // no, rinse and repeat
                  push(bitPos);
                  push(ReadRemainingBits);
                }
              }
              else {
                push(bitPos);
                push(ReadRemainingBits);
              }
              YieldFor(Micros55);
            }
            break;

          case StartIDSend: {
              idByteIndex = 0;
              push(SendRemainingIDBytes);
              pushSendOneByte(SELECTDEVICE);
            }
            break;

          case SendRemainingIDBytes: {
              if (idByteIndex < 8) {
                push(SendRemainingIDBytes);              // There will still be more to send after this one.
                push(deviceAddr[idByteIndex++]);
                push(8); // send 8 bits
                push(SendRemainingBits);
              }
            }
            break;


          case Reset: {
              // Specs, pg 2 of  http://ww1.microchip.com/downloads/en/appnotes/01199a.pdf
              // Drive bus low, delay 480 μs.
              // Release bus, delay 70 μs.
              // Sample bus: 0 = device(s) present,
              //             1 = no device present
              // Delay 410 μs.
              // Put operations on back to front ...
              YieldFor(Micros410);
              push(BusSample);
              YieldFor(Micros70);
              push(BusRelease);
              YieldFor(Micros480);
              push(BusLow);
            }
            break;

          case Yield: {
              byte tics = pop();
              return (tics);
            }
            break;

          case BusSample: {
              releaseBus();
              int thisBit = sampleBus();
              if (thisBit == 1) {
                // No device present on bus.  Set the status accordingly, and abandon all pending computation.
                //  flushStack();
                status |= NoDeviceOnBus;
                digitalWrite(LED_ALERT, HIGH);
              }
              // If there is a device present, we can just carry on
            }
            break;

          case WaitForBusRelease:
            { // Here is where we can repeat this test-and-wait again cycle a really long time if
              // we have a slow device converting temperatures
              releaseBus();
              int thisBit = sampleBus();
              if (thisBit == 0) // No, some device is still holding the bus LOW
              {
                push(WaitForBusRelease);  // Loop around to try again after about
                YieldFor(255);
                //   toggleDebugLine();        // Rattle the debug line so we can watch it on the scope
              }
              else {   // yay, all devices are ready, clear the waiting status and get on with other things
                status &= ~DevicesAreBusy;
              }
            }
            break;

          case TestTimings: {

              releaseBus();
              _delay_us(10);

              unsigned int loByte = pop();
              unsigned int hiByte = pop();
              unsigned int toGo = (hiByte << 8) | loByte;

              if (toGo > 1) {
                // Put another TestTimings on the stack for next time.
                toGo--;
                push(toGo >> 8);   // hiByte
                push(toGo & 0xFF); // loByte
                push(TestTimings);
              }

              pullBusLow();

              switch (toGo % 5) {  // This will create a sequence of 4 different timing pulses to look at on an oscilloscope

                case 0:
                  YieldFor(Micros480);
                  break;
                case 1:
                  YieldFor(Micros70);
                  break;
                case 2:
                  YieldFor(Micros64);
                  break;
                case 3:
                  toggleDebugLine();
                  YieldFor(Micros55);
                  break;
                case 4:
                  YieldFor(Micros55);
                  break;
              }
            }

            break;


        } // switch(opCode)

      } while (true);  // Loop until one of the Yield operations ends the loop

    }


    void readScratchpadAsync(const uint8_t* deviceAddress, uint8_t *scratchPad)
    {
      cli();
      deviceAddr = deviceAddress;
      sPad = scratchPad;
      flushStack();
      status = StillBusy;
      push(ClearBusyStatus); // Operations back to front on the stack: do this when ReadScratchPad terminates
      push(ReadScratchPad);
      sei();
    }

    void resetAsync()
    {
      cli();
      flushStack();
      status = StillBusy;
      push(ClearBusyStatus);  // Do this when Reset terminates
      push(Reset);
      sei();
    }

    void convertAllTemperaturesAsync() {
      cli();
      flushStack();
      status =  DevicesAreBusy;
      push(WaitForBusRelease);
      pushSendOneByte(STARTCONVO);
      pushSendOneByte(SKIPROMWILDCARD);
      push(Reset);
      sei();
    }

    int getRaw(byte *deviceAddress, byte *scratchPad) 
    {
      byte lsb, msb, b6, b7;
      cli();
      lsb = scratchPad[0];
      msb = scratchPad[1];
      b6 = scratchPad[6];
      b7 = scratchPad[7];
      sei();

      // Now interpret the raw values on the basis of the type of sensor
      switch (deviceAddress[0]) {

        case 0x28: {
            int16_t raw = (((int16_t) msb) << 11) | (((int16_t) lsb) << 3);
            return raw;  // 128'ths of a degree, with 0 count meaning 0 degrees
          }

        case 0x10: {

            // My 0x10 family are fakes, or very early DS1820s. (Marked DS1820 on the package)
            // They measure 8 signficant bits and a sign bit.
            // They count half degrees in the MSB:LSB.  The "offset"  MSB:LSB is wildly wrong.
            // The did not do well in my deep freeze, and can't read negative temperatures. 
            // Additional sixteenths of a degree are extracted by the leftover count
            // in sPad[6].  sPad[6] counts down from 0x10 to 0,  Halfway down its count and
            // again when it gets to 0 it increments MSB:LSB. 
            // At zero it resets to 0X10.  Count rate increases with increasing temperature.
            // A problem is the "initial value" of MSB:LSB is nowhere near
            // where the spec sheet says, (like about 66 degrees off) so I have had
            // to hack an adjustment based on my experiments.

            int16_t raw = (((int16_t) msb) << 8) | (lsb & 0XFE); // Half degrees, with any last half degree discarded

            raw = (raw << 3)    // make space for 16th's
                  + (b7 - b6);  // and add the four bits (sixteenths) from the leftovers

            // These are now in 16'ths of a degree. Change them to 128ths. Wait for better hardware with more sub-degree resoluation.
            int16_t x  = raw << 3;
            
            // Linear remapping calculated from (x0,y0) and (x1, y1) measurements at about 22 degrees and 60 degrees.  f(x) = a + bx
            // I don't have a good reference themometer, so these numbers might be off by some margin.  Let me know if you can measure accurately.
            float a = 8900;
            float b = 1.29;
            int16_t y  = (int16_t) (a  +  b * x);

            return (y);
          }
      }

      return 0xFFFF;  // To mean "we have no idea" 
    }

    float getTempC(byte * deviceAddress, byte * scratchPad)
    {
      float raw = getRaw(deviceAddress, scratchPad);
      return (raw / 128.0);
    }

    void doTestTimings(uint16_t repeats)
    {
      cli();
      flushStack();
      status = StillBusy;
      push(ClearBusyStatus);
      push(BusRelease);
      push(repeats >> 8);  // HiByte
      push(repeats & 0xFF); // LoByte

      push(TestTimings);
      sei();
    }

    byte getStatus()
    {
      byte result;
      cli();
      result = status;
      sei();
      return result;
    }

    void begin()
    {
      // Initial setup of the timer, etc. 
      
      cli();   //stop interrupts
      pinMode(debugPin, OUTPUT);
      flushStack();

      // https://www.instructables.com/id/Arduino-Timer-Interrupts/
      // Page references refer to  https://www.sparkfun.com/datasheets/Components/SMD/ATMega328.pdf
      TCCR2A = 0;                   // set entire TCCR2A register to 0  pg158
      TCCR2B = 0;                   // same for TCCR2B
      TCCR2A |= (1 << WGM21);       // turn on CTC mode
      TIMSK2 |= (1 << OCIE2A);      // enable timer compare interrupt

      OCR2A = 255;  // set our first interrupt event to occur as far into the future possile
      TCNT2  = 0;   //initialize counter value to 0
      TCCR2B |= (1 << CS22);  // pg 162./ Pg188  Attach timer to prescaler source. This starts the timer

      sei();   //allow interrupts
    }

    byte busyWaitForZeroStatus(const char *msg, int millisTimeout)
    { byte response;
      int count = 0;
      while (true) {
        response = getStatus();
        if (response == 0) return response;
        if (++count >= millisTimeout) {
          Serial.print(msg);
          Serial.print(" tired of waiting for response. BIN resp = ");
          Serial.print(response, BIN); Serial.print("  after millis ="); Serial.println(millisTimeout);
          return response;
        }
        delay(1);
      }
    }
};

AsyncTemperatureReader myTemperatureSensors;   // A single instance of the class manages one wire.

// Diagnostic, keeps track of the longest interval in the ISR.
// Can be read and zerod in main program within a critical section.
long ISR_max_busytime = 0;        


ISR(TIMER2_COMPA_vect) {
//  long t0 =  micros();                          // diagnostic

  TCCR2B = 0;                                   // pg 162. Stop the timer
  OCR2A = myTemperatureSensors.doTimeslice();  // Do a timeslice, and collect the next required delay
  TCNT2 = 0;                                    // re-start the counter again from zero
  TCCR2B |= (1 << CS22);                        // pg 162.  Mega=pg188 Set prescaler, Start the timer    

//  long et = micros() - t0;   // diagnostic
 // if (et > ISR_max_busytime) ISR_max_busytime = et;
}
