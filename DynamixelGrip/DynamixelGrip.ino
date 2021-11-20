/*
DynamixelGrip.ino
  written by cbteeple

  *****************************************************************************
  Decription:
  This library enables simple control of a dynamixel servo via commands.

  Dependencies:
   - SoftHalfDuplexSerial library (https://github.com/akira215/HalfDuplexSerial-for-Arduino)
   - DynamixelAx library (https://github.com/akira215/DynamixelArduino)

  Hardware:
   - This example uses the SoftHalfDuplexSerial library, so check that the connected data pin supports change interrupts
   - Check that the board is properly grounded with dynamixel power supply.
   - The servo's baudrate must be changed from its default. Use the "DynamixelBaudSwitch.ino" sketch to switch it from 1000000bps to 57600bps.
*/

#include <SoftHalfDuplexSerial.h>
#include <DynamixelAx.h>

#define NUM_SERVOS 5

int servo_id[] = {1,2,3,4,5,6};
int servo_comm_pin = 2;

softHalfDuplexSerial port(servo_comm_pin); // data pin 8
dxlAx dxlCom(&port);

String _readString;         // Input string from serial monitor
bool _strComplete = false;
int _id = 1;                // Default Dynamixel servo ID

unsigned int setpoint[NUM_SERVOS];
unsigned int curr_pos[NUM_SERVOS];
unsigned int max_pos[NUM_SERVOS];
unsigned int min_pos[NUM_SERVOS];
unsigned int speed[NUM_SERVOS];
unsigned int torque[NUM_SERVOS];
bool         cont_hold[NUM_SERVOS];

bool new_setpoint = false;
bool echo_global = true;
unsigned int units = 1;
bool isMoving[NUM_SERVOS];
unsigned long curr_time = 0;
unsigned long last_time = 0;
unsigned int data_time = 100;
bool data_on = false;

void printServoId(String msg);
void printDxlResult();
void printDxlError(unsigned short dxlError);
void reset_moving();
bool any_moving();
bool all_moving();
int  get_result();


unsigned int pos_init = 512;
unsigned int max_pos_init = 1023;
unsigned int min_pos_init = 0;

void setup() {
  // Open serial communications and wait for port to open (PC communication)
  Serial.setTimeout(30);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("_StartingCOM!");

  // Start the communication with the dynamixels
  dxlCom.begin(57600);

  // Initialize variable vectors
  for(int i=0; i<NUM_SERVOS; i++){
    setpoint[i] = pos_init;
    dxlCom.readPresentPosition(servo_id[i]);
    curr_pos[i] = get_result();
    max_pos[i]  = max_pos_init;
    min_pos[i]  = min_pos_init;
    speed[i]    = 1023;         //Start with max speed
    torque[i]   = 1023;         //Start with max torque
    cont_hold[i]   = false;        //Start with no continuous hold

    dxlCom.setTorqueLimit(servo_id[i],torque[i]);
    clear_result();
    dxlCom.setMovingSpeed(servo_id[i],speed[i]);
    clear_result();
    dxlCom.setGoalPosition(servo_id[i],setpoint[i]);
    clear_result();
  }
  
  reset_moving();
}

/////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  // Run the move if there's a new setpoint
  if (new_setpoint){
    new_setpoint=false;
    reset_moving();
    // Set the new goal positions
    for(int i=0; i<NUM_SERVOS; i++){
      dxlCom.setGoalPosition(servo_id[i],setpoint[i]);
      //printDxlResult();
      get_result();
    }

    delay(10);

    // Wait until no more motion occurs
    int final_pos = 0; 
    while (any_moving()){
      unsigned short error = DXL_ERR_SUCCESS;
      int result = -1;
      for(int i=0; i<NUM_SERVOS; i++){
        if(isMoving[i]){
          while(dxlCom.isBusy()); // waiting the status return delay time
          dxlCom.readPresentPosition(servo_id[i]);
          //Serial.print("Pos : ");
          while(!dxlCom.dxlDataReady());        // waiting the answer of servo
          result = dxlCom.readDxlResult();
          error = dxlCom.readDxlError();
          if(error!=DXL_ERR_SUCCESS) // readDxlResult should always be called before readDxlData
            printDxlError(error);
          curr_pos[i] = result;
          //Serial.print(curr_pos[i]);
          //Serial.print('\t');
      
          while(dxlCom.isBusy()); // waiting the status return delay time
          dxlCom.readPresentLoad(servo_id[i]);
          //Serial.print("Load : ");
          while(!dxlCom.dxlDataReady());        // waiting the answer of servo
          result = dxlCom.readDxlResult();
          error = dxlCom.readDxlError();
          if(error!=DXL_ERR_SUCCESS) // readDxlResult should always be called before readDxlData
            printDxlError(error);
          //Serial.println(dxlCom.readDxlResult());
          
          while(dxlCom.isBusy()); // waiting the status return delay time (for testing if it is moving)
          dxlCom.isMoving(servo_id[i]);
          while(!dxlCom.dxlDataReady());        // waiting the answer of servo
          result = dxlCom.readDxlResult();
          error = dxlCom.readDxlError();
          if(error!=DXL_ERR_SUCCESS) // readDxlResult should always be called before readDxlData
            printDxlError(error);
          isMoving[i] = result;
        }

        if (!isMoving[i] & !cont_hold[i]){
          dxlCom.setGoalPosition(servo_id[i],curr_pos[i]);
          while(!dxlCom.dxlDataReady());        // waiting the answer of servo
          result = dxlCom.readDxlResult();
          error = dxlCom.readDxlError();
        }
      }
      curr_time = millis();
      send_data(0);
      send_data(1);
      bool new_data = recvWithEndMarker();
      if (new_data){
        break;
      }
    }
  }
  else{
    for(int i=0; i<NUM_SERVOS; i++){
      dxlCom.readPresentPosition(servo_id[i]);
      curr_pos[i] = get_result();
    }
  }

  // check for new serial messages
  //check_serial();
  recvWithEndMarker();

  
  curr_time = millis();
  
  //if (data_on & (curr_time>=last_time+data_time)){
  if (data_on){
    send_data(0);
    send_data(1);
  }
  /*
    send_data(0);
    send_data(1);
    //last_time = curr_time;
    delay(data_time);
  }
  */

}



float pi = 3.14159;

/*
 * Convert from motor position to angle
 */
int angle_to_pos(float angle, bool use_degrees){
  if (use_degrees){
    return 512-int(angle*1024/300.0);
  }
  else{
    return 512-int(angle*1024/5.23599);
  }
}

int angle_to_pos(float angle){
  return angle_to_pos(angle, false);
}


/*
 * Convert from motor position to angle
 */
float pos_to_angle(int pos, bool use_degrees){
  if (use_degrees){
    return (512-pos)*300.0/1024.0;
  }
  else{
    return (512-pos)*5.23599/1024.0;
  }
}

float pos_to_angle(int pos){
  return pos_to_angle(pos, false);
}



float gripper_max_width = 30.0;
/*
 * Convert from gripper distance to motor position
 */
int dist_to_pos(float distance){
  if (distance < pos_to_dist(0)){
    distance = pos_to_dist(0);
  }
  
  float angle = 0;
  float disp = gripper_max_width - distance;
  float vert = gripper_max_width-16;
  if (distance>=0 & distance<=vert){
    angle = pi - acos((disp-16.0)/16.0);
  }
  else{
    angle = acos(1-disp/16.0);
  }
  int pos = angle_to_pos(angle);
  if (pos<0){
    pos=0;
  }
  return pos;
}

/*
 * Convert from motor position to gripper distance
 */
float pos_to_dist(int pos){
  float angle = pos_to_angle(pos);
  float disp = 0;
  if (angle>=pi/2.0 & angle<=pi){
    disp = 16.0 + 16*(cos(pi - angle));
  }
  else{
    disp = 16.0*(1-cos(angle));
  }
  return gripper_max_width-disp;
}


/*
 * Convert the units from display units to internal units
 */
float convert_units_in(float input){
  switch(units){
    case 0: // If using integer units
      return int(input);
    break;
    case 1: // If using degrees
      return angle_to_pos(input, true);
    break;
    case 2: // If using radians
      return angle_to_pos(input, false);
    break;
    case 3: // If using gripper distance
      return dist_to_pos(input);
    break;
    default:
      return input;
  }
}


/*
 * Convert the units from internal units to display units
 */
float convert_units_out(float input){
  switch(units){
    case 0: // If using integer units
      return input;
    break;
    case 1: // If using degrees
      return pos_to_angle(input, true);
    break;
    case 2: // If using radians
      return pos_to_angle(input, false);
    break;
    case 3: // If using gripper distance
      return pos_to_dist(input);
    break;
    default:
      return input;
  }
}


/////////////////////////////////////////////////////////////////////////////////////
/*
 * Print Data
 */
void send_data(int type){
  String out_str =String(curr_time);

  switch (type){
    case 0:
      out_str += '\t';
      out_str += "0";
      for(int i=0; i<NUM_SERVOS; i++){
        out_str += '\t'+String(convert_units_out(setpoint[i]));
      }
    break;
    case 1:
      out_str += '\t';
      out_str += "1";
      for(int i=0; i<NUM_SERVOS; i++){
        out_str += '\t'+String(convert_units_out(curr_pos[i]));
      }
    break;
  }
  Serial.println(out_str);
}

/////////////////////////////////////////////////////////////////////////////////////
/*
Handle Imcomming Serial Commands 
*/
// TODO: Move commands from main loop to the "parse_command" function

String command="";
// Check for new serial data
/*
String check_serial() {
  // Get new command
  while (Serial.available() >0) {
    // get the new byte:
    //command = Serial.readStringUntil('\n');
    char inChar = (char)Serial.read();
    // Add new byte to the inputString:
    command += inChar;
    // If the incoming character is a newline, set a flag so we can process it
    if (inChar == '\n') {
      command.toUpperCase();
      parse_command(command);
      command=""; 
    }
  }

  return command;
}

*/

bool check_serial() {
  //unsigned long start_time = micros();
  delay(2);
  while (Serial.available()) {
    //command = Serial.readStringUntil('\n');
    //command.toUpperCase();
    //parse_command(command);
    // get the new byte:
    byte byte_str = Serial.read();
    char inChar = (char)byte_str;
    // Add new byte to the inputString:
    command += inChar;
    // If the incoming character is a newline, set a flag so we can process it
    if (inChar == '\n') {
      command.toUpperCase();
      parse_command(command);
      command=""; 
      Serial.flush();
      return true;
    }
  }
  return false;
}


bool recvWithEndMarker() {
  char endMarker = '\n';
  char rc;
  
  while (Serial.available() > 0) {
    rc = Serial.read();

    if (rc != endMarker) {
      command += rc;
    }
    else {
      command.toUpperCase();
      parse_command(command);
      command="";
      return true;
    }
  }
  return false;
}

// Send a string via serial
void send_string(String bc_string){
  Serial.println(bc_string);
}


// Parse Commands
void parse_command(String command){
  if (command.length()){
    String out_str="_";
    bool echo_one_time=false;
       
    if(command.startsWith("SET")){
      if (get_string_value(command,';', NUM_SERVOS+1).length()){
        for(int i=0; i<NUM_SERVOS+1; i++){
          float val = get_string_value(command,';', i+2).toFloat();
          val = convert_units_in(val);

          if (val<=max_pos[i] & val>=min_pos[i] ){
            setpoint[i] = val;
          }

        }
        new_setpoint = true;
        out_str+="New ";
      }
      else if (get_string_value(command,';', 2).length()){
        float allset=get_string_value(command,';', 2).toFloat();
        allset = convert_units_in(allset);

        for(int i=0; i<NUM_SERVOS; i++){
          if (allset<=max_pos[i] & allset>=min_pos[i] ){
            setpoint[i] = allset;
          }
          
        }
        new_setpoint = true;
        out_str+="New ";
      }
      out_str+="SET: ";
      for(int i=0; i<NUM_SERVOS; i++){
        out_str += '\t'+String(convert_units_out(setpoint[i]));
      }
      
    }
    else if(command.startsWith("SPEED")){
      if (get_string_value(command,';', NUM_SERVOS).length()){
        for(int i=0; i<NUM_SERVOS; i++){
          float val = get_string_value(command,';', i+1).toInt();
          speed[i] = val;
        }
        out_str+="New ";
      }
      else if (get_string_value(command,';', 1).length()){
        float allset=get_string_value(command,';', 1).toInt();

        for(int i=0; i<NUM_SERVOS; i++){
          speed[i] = allset;
        }
        out_str+="New ";
      }
      out_str+="SPEED: ";
      for(int i=0; i<NUM_SERVOS; i++){
        dxlCom.setMovingSpeed(servo_id[i],speed[i]);
        //printDxlResult();
        out_str += '\t'+String(speed[i]);
      }
      
    }

    else if(command.startsWith("TORQUE")){
      if (get_string_value(command,';', NUM_SERVOS).length()){
        for(int i=0; i<NUM_SERVOS; i++){
          float val = get_string_value(command,';', i+1).toInt();
          torque[i] = val;
        }
        out_str+="New ";
      }
      else if (get_string_value(command,';', 1).length()){
        float allset=get_string_value(command,';', 1).toInt();

        for(int i=0; i<NUM_SERVOS; i++){
          torque[i] = allset;
        }
        out_str+="New ";
      }
      out_str+="TORQUE: ";
      for(int i=0; i<NUM_SERVOS; i++){
        dxlCom.setTorqueLimit(servo_id[i],torque[i]);
        //printDxlResult();
        out_str += '\t'+String(torque[i]);
      }
      
    }
    else if(command.startsWith("CONT")){
      if (get_string_value(command,';', NUM_SERVOS).length()){
        for(int i=0; i<NUM_SERVOS; i++){
          cont_hold[i] = bool(get_string_value(command,';', i+1).toInt());

        }
        out_str+="New ";
      }
      else if (get_string_value(command,';', 1).length()){
        bool allset=bool(get_string_value(command,';', 1).toInt());
        
        for(int i=0; i<NUM_SERVOS; i++){
            cont_hold[i] = allset;
          
        }
        out_str+="New ";
      }
      out_str+="CONT: ";
      for(int i=0; i<NUM_SERVOS; i++){
        out_str += '\t'+String(cont_hold[i]);
      }
      
    }



    else if(command.startsWith("ID")){
      if (get_string_value(command,';', NUM_SERVOS).length()){
        for(int i=0; i<NUM_SERVOS; i++){
          float val = get_string_value(command,';', i+1).toInt();

          if (val<=max_pos[i] & val>=min_pos[i] ){
            servo_id[i] = val;
          }

        }
        new_setpoint = true;
        out_str+="New ";
      }
      out_str+="ID: ";
      for(int i=0; i<NUM_SERVOS; i++){
        out_str += '\t'+String(servo_id[i]);
      }
      
    }
    
    else if(command.startsWith("SWAPID")){
      if (get_string_value(command,';', 2).length()){
        int old_id = get_string_value(command,';', 1).toInt();
        int new_id = get_string_value(command,';', 2).toInt();
        dxlCom.setId(old_id, new_id);
        //printDxlResult();
      }  
    }
    else if(command.startsWith("LED")){
      if (get_string_value(command,';', 2).length()){
        int id = get_string_value(command,';', 1).toInt();
        bool val = bool(get_string_value(command,';', 2).toInt());
        dxlCom.setLedEnable(id, val);
        //printDxlResult();
      }  
    }

    else if(command.startsWith("REBOOT")){
      if (get_string_value(command,';', 1).length()){
        int id = get_string_value(command,';', 1).toInt();
        dxlCom.reboot(id);
        //printDxlResult();
        out_str += "REBOOT: ";
        out_str += '\t'+String(id);
      }
    }
    else if(command.startsWith("PING")){
      if (get_string_value(command,';', 1).length()){
        int id = get_string_value(command,';', 1).toInt();
        dxlCom.ping(id);
        out_str += "PING: ";
        out_str += '\t'+String(id);
        out_str += '\t'+String(get_result());
      }
    }

    else if(command.startsWith("VOLT")){
      out_str+="VOLT: ";
      for(int i=0; i<NUM_SERVOS; i++){
        dxlCom.readVoltage(servo_id[i]);
        out_str += '\t'+String(get_result()/10.0,1);
      }    
    }
    else if(command.startsWith("TEMP")){
      out_str+="TEMP: ";
      for(int i=0; i<NUM_SERVOS; i++){
        dxlCom.readTemperature(servo_id[i]);
        out_str += '\t'+String(get_result());
      }    
    }
    else if(command.startsWith("FIRMWARE")){
      out_str+="FIRMWARE: ";
      for(int i=0; i<NUM_SERVOS; i++){
        dxlCom.readFirmware(servo_id[i]);
        out_str += '\t'+String(get_result());
      }    
    }
    else if(command.startsWith("MODEL")){
      out_str+="MODEL: ";
      for(int i=0; i<NUM_SERVOS; i++){
        dxlCom.readModelNumber(servo_id[i]);
        out_str += '\t'+String(get_result());
      }    
    }
    else if(command.startsWith("ON")){
      data_on = true;
      out_str+="ON: "; 
    }
    else if(command.startsWith("OFF")){
      data_on = false;
      out_str+="OFF: "; 
    }
    else if(command.startsWith("CAL")){
      for (float i=0; i<=30; i=i+0.25){
        dxlCom.setGoalPosition(servo_id[0],dist_to_pos(30.0-i));
        clear_result();
        delay(50);
      }
      for (float i=0; i<=30; i=i+0.25){
        dxlCom.setGoalPosition(servo_id[0],dist_to_pos(i));
        clear_result();
        delay(50);
      }
      for(int i=0; i<NUM_SERVOS; i++){
        dxlCom.setGoalPosition(servo_id[i],setpoint[i]);
        clear_result();
      }
    
    }
    else if(command.startsWith("MAX")){
      if (get_string_value(command,';', NUM_SERVOS).length()){
        for(int i=0; i<NUM_SERVOS; i++){
          max_pos[i] = get_string_value(command,';', i+1).toInt();
          
        }
        out_str+="New ";
      }
      else if (get_string_value(command,';', 1).length()){
        float allset=get_string_value(command,';', 1).toInt();

        for(int i=0; i<NUM_SERVOS; i++){
          max_pos[i] = allset;
          
        }
        out_str+="New ";
      }
      out_str+="MAX: ";
      for(int i=0; i<NUM_SERVOS; i++){
        out_str += '\t'+String(max_pos[i]);
      }
      
    }


    else if(command.startsWith("MIN")){
      if (get_string_value(command,';', NUM_SERVOS).length()){
        for(int i=0; i<NUM_SERVOS; i++){
          min_pos[i] = get_string_value(command,';', i+1).toInt();
          
        }
        out_str+="New ";
      }
      else if (get_string_value(command,';', 1).length()){
        float allset=get_string_value(command,';', 1).toInt();

        for(int i=0; i<NUM_SERVOS; i++){
          min_pos[i] = allset;
          
        }
        out_str+="New ";
      }
      out_str+="MIN: ";
      for(int i=0; i<NUM_SERVOS; i++){
        out_str += '\t'+String(min_pos[i]);
      }
      
    }

    else if(command.startsWith("ECHO")){
      if (get_string_value(command,';', 1).length()){
        echo_global = bool(get_string_value(command,';', 1).toInt());
        echo_one_time = true;
        out_str+="New ";
      }
      out_str+="ECHO: " + String(echo_global);
    }


    else if(command.startsWith("UNITS")){
      if (get_string_value(command,';', 1).length()){
        units = constrain(get_string_value(command,';', 1).toInt(),0,4);
        out_str+="New ";
      }
      out_str+="UNITS: " + String(units); 
    }


    
    else{
      out_str = "Unrecognized Command: ";  
      out_str += command;
    }

    if (echo_global or echo_one_time){
      send_string(out_str);
      echo_one_time = false;
    }
  }
}

// Get one element of a string command with delimeter
String get_string_value(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}




// Float mapping (not native to arduino for some reason)
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


bool any_moving(){
  for(int i=0; i<NUM_SERVOS; i++){
    if (isMoving[i]){
      return true;
    }
  }
  return false;
}

bool all_moving(){
  for(int i=0; i<NUM_SERVOS; i++){
    if (!isMoving[i]){
      return false;
    }
  }
  return true;
}

void reset_moving(){
  for(int i=0; i<NUM_SERVOS; i++){
    isMoving[i] = true;
  }
}


/////////////////////////////////////////////////////////////////////////////////////
int get_result(){
while(!dxlCom.dxlDataReady());
dxlCom.readDxlError();
return  dxlCom.readDxlResult();
}

void clear_result(){
  dxlCom.readDxlError();
  dxlCom.readDxlResult();
}



void printDxlResult()
{
   while(!dxlCom.dxlDataReady());        // waiting the answer of servo
   printDxlError(dxlCom.readDxlError());
   Serial.println(dxlCom.readDxlResult());
}

void printServoId(String msg)
{
  Serial.print(msg);
  Serial.print(" servo ID ");
  Serial.print(_id);
  Serial.print(" - ");
}

void printDxlError(unsigned short dxlError)
{
  // after any operation error can be retrieve using dx::readDxlResult() (i.e. after read or write operation)
  if(dxlError == DXL_ERR_SUCCESS)
    Serial.println("OK");
  else
  {
    if (dxlError & DXL_ERR_VOLTAGE)
      Serial.print("voltage out of range-");
    if (dxlError & DXL_ERR_ANGLE)
      Serial.print("angle out of range-");
    if (dxlError & DXL_ERR_OVERHEATING)
      Serial.print("overheating-");
    if (dxlError & DXL_ERR_RANGE)
      Serial.print("cmd out of range-");
    if (dxlError & DXL_ERR_TX_CHECKSUM)
      Serial.print("Tx CRC invalid-");
    if (dxlError & DXL_ERR_OVERLOAD )
      Serial.print("overload-");
    if (dxlError & DXL_ERR_INSTRUCTION )
      Serial.print("undefined instruction-");
    if (dxlError & DXL_ERR_TX_FAIL )
      Serial.print("Tx No header-");
    if (dxlError & DXL_ERR_RX_FAIL )
      Serial.print("Rx No header-");
    if (dxlError & DXL_ERR_TX_ERROR  )
      Serial.print("Tx error-");
    if (dxlError & DXL_ERR_RX_LENGTH   )
      Serial.print("Rx length invalid-");  // Not implemented yet
    if (dxlError & DXL_ERR_RX_TIMEOUT)
      Serial.print("timeout-");
    if (dxlError & DXL_ERR_RX_CORRUPT)
      Serial.print("Rx CRC invalid-");
    if (dxlError & DXL_ERR_ID )
      Serial.print("Wrong ID answered-"); // ?? Hardware issue
    Serial.println();
  }
}
