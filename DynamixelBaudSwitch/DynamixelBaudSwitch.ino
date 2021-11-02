/*
DynamixelBaudSwitch.ino
  written by cbteeple

  *****************************************************************************
  Decription:
  Switch the baud rate of a dynamixel servo using hardware serial

  Dependencies:
   - AX12A library (https://github.com/jumejume1/AX-12A-servo-library)

  Hardware:
   - This example uses the AX12A library using hardware serial, so connect
     your servo to the TX pin on your arduino.
   - Check that the board is properly grounded with dynamixel power supply.
   - This example changes a the servo's baud rate from the default (1000000bps)
     to 57600bps for compatibility with my "DynamixelGrip" firmware.
*/

#include "Arduino.h"
#include "AX12A.h"

#define DIRECTION_PIN 	(10u)
#define SERVO_ID				(1u)

// Define baud rates
#define CURRENT_BAUD_RATE  (1000000ul) // Current baude rate the servo is set to
#define NEW_BAUD_RATE (57600ul)        // New baud rate to set.


void setup()
{
  // NOTE: if you are running an arduino micro, comment out all the code inside
  // the setup() function.
  
  // Connect to the servo at its current baud rate, and set the new baud rate
	ax12a.begin(CURRENT_BAUD_RATE, DIRECTION_PIN, &Serial);
  ax12a.setBD(SERVO_ID, NEW_BAUD_RATE);

  // Reset the servo (reboot)
  ax12a.reset(SERVO_ID);

  // Connect to the servo at the new baud rate and set the new baud rate again
  ax12a.begin(NEW_BAUD_RATE, DIRECTION_PIN, &Serial);
  ax12a.setBD(SERVO_ID, NEW_BAUD_RATE);
}

bool firstcall=true;

void loop()
{
  // Do nothing here. Everything is taken care of in the setup.
  
  // NOTE: if you are running an arduino micro, Serial communication is not enabled
  // in setup(). Uncomment this code and comment out the code in "Setup".
  /*
  if (firstcall){
    // Connect to the servo at its current baud rate, and set the new baud rate
    ax12a.begin(CURRENT_BAUD_RATE, DIRECTION_PIN, &Serial);
    ax12a.setBD(SERVO_ID, NEW_BAUD_RATE);
  
    // Reset the servo (reboot)
    ax12a.reset(SERVO_ID);
  
    // Connect to the servo at the new baud rate and set the new baud rate again
    ax12a.begin(NEW_BAUD_RATE, DIRECTION_PIN, &Serial);
    ax12a.setBD(SERVO_ID, NEW_BAUD_RATE);

    firstcall=false;
  }
  */


  
}
