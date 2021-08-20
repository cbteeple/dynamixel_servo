# Dynamixel Servo

A program for controlling several Dynamixel servos with a standard communication protocol.

## Dependencies:
- [HalfDuplexSerial-for-Arduino](https://github.com/akira215/HalfDuplexSerial-for-Arduino) library (_required_) - For communicating with the servos via half-duplex serial
- [DynamixelArduino](https://github.com/akira215/DynamixelArduino) library (_required_) - For controlling dynamixel servos as nice objects.

## Setup
1. Define the comm pin to use (since dynamixels can be daisy-chained). This pin might need to have interrupt capabillities to work correctly.
2. Define the number of servos
3. Define a list of servo ID's. These are the internal ID's of the servos) (what the servos think they are). By default the ID is always "1" on new servos.

```cpp
int servo_comm_pin = 2;
#define NUM_SERVOS 6
int servo_id[] = {1,2,3,4,5,6};
```
4. Upload the program to an Arduino (I used an Arduino Nano).
5. Set up your servo with 12V power and 

## Usage
Send serial commands using a `[COMMAND];[ARG];[ARG]...` structure. This struture is described in detail in the [documentation for Ctrl-P](https://ctrl-p.cbteeple.com/latest/firmware/firmware_commands), but this servo program uses a different command set.

## Commands

### Main Commands:
- **SET** - Set the position setpoints in whatever units you are using (see below).
  - **SET;_[#1];[#2]_** - Set the position of all servos to the same value. #1 is a dummy number for compatibllity, #2 (`float`) is the servo position.
  - **SET;_[#1];[#2-#N+1]_** - Set the positions of all servos. #1 is a dummy number for compatibllity, #2-#N+1 (`float`) are the positions of each servo.
- **SPEED** - Set the speed of the servos (in direct units)
  - **SPEED;_[#1]_** - Set the speed of the servos to one value. #1 (`int`) is the desired speed.
  - **SPEED;_[#1-#N]_** - Set the speed of the servos. #1-#N (`int`) are the desired speeds. 
- **TORQUE** - Set the torque of the servos (in direct units)
  - **TORQUE;_[#1]_** - Set the torque of the servos to one value. #1 (`int`) is the desired torque.
  - **TORQUE;_[#1-#N]_** - Set the torque of the servos. #1-#N (`int`) are the desired torques. 
- **CONT** - Set the continuous operation mode (i.e. torque-determined stop vs. continuous torque)
  - **CONT;_[#1]_** - Set the continuous operation mode of the servos to one value. #1 (`bool`) is the desired mode.
  - **CONT;_[#1-#N]_** - Set the continuous operation mode of the servos. #1-#N (`bool`) are the desired torques.


### Safety Commands:
- **MAX** - Set the maximum position of the servos (in direct units)
  - **MAX;_[#1]_** - Set the maximum position of the servos to one value. #1 (`int`) is the desired maximum position.
  - **MAX;_[#1-#N]_** - Set the maximum position of the servos. #1-#N (`int`) are the desired maximum positions. 
- **MIN** - Set the maximum position of the servos (in direct units)
  - **MIN;_[#1]_** - Set the minimum position of the servos to one value. #1 (`int`) is the desired minimum position.
  - **MIN;_[#1-#N]_** - Set the minimum position of the servos. #1-#N (`int`) are the desired minimum positions. 

### Communication Commands
- **ON** - Turn on live data output
  - **ON** - (No Args)
- **OFF** - Turn off live data output
  - **OFF** - (No Args)
- **UNITS** - Set the output units 
  - **UNITS;_[#1]_** - #1 (`int`) is the unit type: (0 = direct servo units, 1 = angle (degrees), 2 = angle (radians), 3 = gripper width (mm)). Gripper width units are calculated for the [Trossen Phantom Parallel Gripper](https://www.trossenrobotics.com/p/phantomx-parallel-ax12-gripper.aspx)
- **ECHO** - Set the state of command echos
  - **ECHO;_[#1]_** - #1 is the echo state (0=off, 1=on)

### Servo Setup Commands
- **ID** - Set the Servo IDs that you want to use
  - **ID;_[#1-#N]_** - Set the servo IDs. #1-#N (`int`) are the desired IDs. 
- **SWAPID** - Swap a servo's onboard ID
  - **SWAPID;_[#1];[#2]_** - #1 (`int`) is the current ID, #2 (`int`) is the desired ID to swap to.
- **REBOOT** - Reboot a servo
  - **REBOOT;_[#1]_** - #1 (`int`) is the ID of the servo to reboot.
 
### Servo Diagnostics
- **PING** - Ping a servo
  - **PING;_[#1]_** - #1 (`int`) is the ID of the servo
- **VOLT** - Get the voltage (in V) of all servos
  - **VOLT** - (No Args)
- **TEMP** - Get the temperature (in C) of all servos
  - **TEMP** - (No Args)
- **FRIMWARE** - Get the firmware number of all servos
  - **FRIMWARE** - (No Args)
- **MODEL** - Get the model number of all servos
  - **MODEL** - (No Args)
  
### Other Commands:
- **LED** - Set the LED state of a servo
  - **LED;_[#1]_** - #1 (`int`) is the ID of the servo, #2 (`bool`) is the LED state (0=off, 1=on).


## Notes:
1. The communication protocol is backward-compatible with my [Ctrl-P pressure control system](https://github.com/cbteeple/pressure_controller).
