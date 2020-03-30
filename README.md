# Background Sensing of 1-Wire DS1820 Temperature Sensors





##  Temperature sensing with Dallas DS1820-type sensors

These little temperature sensors are extermely cheap (the ones I got were 
probably fake Chinese ones, or old surplus stock).

![sensors](Images/sensors.png "Temperature Sensors")

Devices using the 1-wire protocol share the single wire bus: 
any device can drive the bus LOW, but when all are idle the bus they have to 
provide high impedance and the bus will be pulled-up via a resistor to HIGH.  
Coordination depends on the Master controlling the slave devices, 
and on timing. 

The protocol needs long blocking periods, and the "obvious" implementation requires
much use of delayMicroseconds(). For example, resetting the bus requires
three successive wait times of 480us, 70us, and 410us.  

But the devices on the bus can be even more hectic. Getting bus devices to all
simultaneously perform a temperature conversion can require a delay of 
750ms, (yes, millis) before the slowest device might release the bus to indicate 
completion. 
https://www.maximintegrated.com/en/design/technical-documents/app-notes/4/4377.html 

The key operations I need to repeat - initiate temperature conversions,
and then read the device scratchpad(s) (each 9 bytes, 72 bits)
means the protocol cannot reset the bus, send a device address, and get the scratchpad
value in less than about 10 millisecs, almost all spent blocking, and busy-waiting.
We can't make the wire or the devices go faster.  And for some applications 
being blocked for long waits is unacceptable. 

So I describe here how to drive the protocol through an interpreter 
that runs in the "background", and does not use long busy waits, 
so your MCU can get on with doing its other work in the meantime.
 
## How the bankground sensing works

AsyncTemperatureReader is a "time-sliced background process" 
on the Arduino (only tested
on a UNO and a MEGA2560 to date). It interprets pseudo instructions. 
Each time the interpreter gets a "timeslice", it does some non-blocking 
steps of the task and then yields with a "holdoff" value indicating 
that it can't do anything 
more until after the holdoff interval has lapsed. Holdoff values are
expressed as a number of tics on TIMER2.  (For anyone who has been around long 
enough to remember the first time-slicing operating systems, the idea is called
"non-preemptive multitasking".  Whatever is running cannot be premptively stopped -
the running process has to voluntarily yield control.)

The interpreter gets its chance to run when its doTimeslice() method is called by  
the TIMER2 interrupt service routine.  When the interpreter yields control back
to the ISR caller, it returns a holdoff time that it wants to wait before its 
next timeslice.  The ISR sets up new values for the TIMER2 registers to 
achieve the requested holdoff delay.  When the holdoff time has elapsed 
the timer will fire the ISR to run again, and the ISR will give the
interpreter its next timeslice.

Pending code or steps to be executed are stored in a bytecode stack.
But some opcodes are macros. When popped off the stack and executed, they can expand
into multiple more primitive instructions in the stack. These newly expanded instructions
can also be macros. Each instruction, primitive or macro, may be followed by zero or 
more operands on the stack.

As an example, suppose we request a ReadScratchPad operation.  
It expands into instructions or macros in the stack that will eventually 
sequence the operations and delays needed to 
Reset the device, Select the device by sending the device's addressID,
and then Read back the device's scratchpad into a 9-byte buffer (supplied by the caller).

Even just reading back the scratchpad requires 72 bit-reads from the sensor. 
Each bit-read is a sequence of actions that must drive the line low,
pause, release the line, pause, then sample the line and store the bit, 
and then pause again. And so on. 

Short pauses are done inline with _delay_us(), but longer pauses
end the current timeslice.  Of course the instruction stack must be 
left in such a state that the next doTimeslice() 
call can pick up the flow of logic where we last left off.

The onewire protocol is quite forgiving about timing.
The master (that's us) always controls timing on the bus by driving the line low.
then releasing the line to allow the pull-up to pull it high. Then any one of the
attached slave devices can send info back to the master by pulling the shared
bus-line low.  (Slaves must first be addressed and "given the right to use the bus", otherwise they do not talk). Loose protocol timings are acceptable.

This code is very specific for my little sensors. 
So I have not yet catered for the more exotic features like Address Search for devices 
on the bus, CRC checks, parasitic power mode, different device resolutions, etc.
That is left as a homework exercise for someone else :-)

This is therefore closer to a minimalistic 
"proof of concept" rather than a fully-featured system.

## A Peek Under The Hood

The interpreter updates status codes as the timeslices run.  
```
const byte Success = 0x00;           // Zero means "ok, all done successfully"
const byte StillBusy = 0x01;
const byte NoDeviceOnBus = 0x02;     // bit set indicates no device on bus
const byte DevicesAreBusy = 0x04;    // bit set means we're still waiting for sensors to complete their conversions
```


The user can call a public method to retrieve the status, and learn if the requested 
operation has terminated or failed. 
Because the status code and the scratchpad, etc. are accessed by both the main program and the interpreter doTimeSlice which is called from the TIMER2 ISR, we need to create 
some critical sections when accessing the shared variables.
 

```
    byte getStatus()
    {
      byte result;
      cli();
      result = status;
      sei();
      return result;
    }

``` 

The interpreter has 13 different opcodes:

```
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

 
 Here is a fragment of code to tell all devices on the bus to do a temperature conversion.  The code is a bit tricky to read because
the last opcode that is pushed onto the code stack will be executed first.  So the sequence we need here is to first reset the bus,
then send a byte to indicate that all devices should execute the next command, then ask them all to start their conversions. And
WaitForBusRelease will test the bus, and either set the status to indicate that all devices have finished, or it will terminate the 
timeslice and wait for another turn to check the bus again. 
 ```
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
```

The interpreter just repeats a cycle of popping an opCode off the stack and dispatching with 
a switch statement that has one case for each opcode: 

```
    // Pre: interrupts are disabled.
      do {

        if (topOfStack == 0) {   // If nothing to do, just keep slowly idling by ticking the counter over
          return 255;            // Maximum holdoff
        }

        byte opCode = theCode[--topOfStack];

        switch (opCode) {     // Now execute the primitive opCode

          case BusLow:
            {
              pullBusLow();
            }
            break;

          case BusRelease:
            {
              releaseBus();
              _delay_us(10);
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
              }
              else {   // yay, all devices are ready, clear the waiting status and get on with other things
                status &= ~DevicesAreBusy;
              }
            }
            break;
```

This last opCode demonstrates a technique I use for "looping". The bus is sampled to see if 
all devices have finished temperature conversion.  If so, the status is cleared and the interpreter
can continue with any other opCodes on the stack.  But if the bus is not clear yet, this code 
pushes another "WaitForBusRelease" back onto the code stack, and yields.  So the timer will wake
it again after about a millsecond, and it can check the status again.

Sometimes opCodes also have operands also on the stack.  For example, when we're transmitting a byte of
data, the stack contains the opCode and two parameters:  the count of bits still to be transmitted,
and the remaining bits of the byte that is being sent.  So it looks like this:

```
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


```

## Results

This repository contains a test program that assumes two sensors are on the bus.  It 
intiates conversions on all sensors, and then reads each one and prints its scratchpad
content, and converts the scratchpad into degrees Celcius.  But while the background
process is busy, we count the progress on same fake work in the foreground. 

## Limitations





