//BMS controller software for Arduino Nano
// Version: Arduino IDE 1.8.5  + Cloud
//
// Originally code written by Stuart Pittaway, and heavyly modified by Tim Milgart
//
// Start Date: 13.01.2018
//
// Update: 05-02-2018
// (c) 2018 Tim Milgart 
// This is the cleanest code for the controller called Master - it talks to the cell modules over isolated i2c bus.  The code will fit into a Arduoni Nano, It can be compiled for a Uno
// as this it will give us some more storage space, as the bootsection is loaded with Optiboot.
// There is a second code with lots of features, OLED display aso. But this is the clean one. If the modules have the corect addresses, it will same some time.
// Update 03.04.2018
// Sketch uses 9866 bytes (32%) of program storage space. Maximum is 30720 bytes. Global variables use 1400 bytes (68%) of dynamic memory, leaving 648 bytes for local variables. Maximum is 2048 bytes.
// Need to optimize a bit and remove lots of unused functions and calls  
// Done with BusInit, Should give a new number to a new module
// Sketch uses 9742 bytes (33%) of program storage space. Maximum is 28672 bytes.
// Global variables use 1193 bytes (46%) of dynamic memory, leaving 1367 bytes for local variables. Maximum is 2560 bytes.
// Update 03.06.2018
// New name "BMS-Controller_12"
// Done with SortByVolt, so it will turn-on the resistorn if one module is 0,05v over the lowest module
// Sketch uses 7906 bytes (27%) of program storage space. Maximum is 28672 bytes.
// Global variables use 850 bytes (33%) of dynamic memory, leaving 1710 bytes for local variables. Maximum is 2560 bytes. 
// Update 03.07.2018
// optimized a bit still much shit to be deleted  
// Sketch uses 8302 bytes (27%) of program storage space. Maximum is 30720 bytes.
// Global variables use 1427 bytes (69%) of dynamic memory, leaving 621 bytes for local variables. Maximum is 2048 bytes.
// Update 09.07.2018
// Added a watchdog, as the bus tend to hang 
// Update 10.07.2018
// Removed unused functions and calls, added a sort algorithm...still need remove unused code, and make a proper init. 
// Update 12.07.2018
// Made a counter for the on-time of the module with the active resistor. So there will be no fire or hot resistor.
// Update 13.07.2018
// Cleaned up in debug information. All serial communication removed
// Sketch uses 6398 bytes (19%) of program storage space. Maximum is 32256 bytes.
// Global variables use 640 bytes (31%) of dynamic memory, leaving 1408 bytes for local variables. Maximum is 2048 bytes.



// the I2C communication
#include <Wire.h>

// watchdog
#include <avr/wdt.h>     

//Inter-packet/request delay on i2c bus
#define delay_ms                              20

#define COMMAND_BIT                           6

#define COMMAND_factory_default               3
#define COMMAND_set_slave_address             4
#define COMMAND_set_temperature_calibration   7
#define COMMAND_set_bypass_voltage            8

#define read_voltage                          10
#define read_temperature                      11
#define read_raw_voltage                      14
#define read_error_counter                    15
#define read_bypass_enabled_state             16

//Default i2c SLAVE address/Unconfigurated 
#define DEFAULT_SLAVE_ADDR                    21
#define NUMBER_MODULES_IN_THE_SYSTEM           7
#define BYPASS_TIME                           15 //30.sec
#define MODULE_VOLT_DIFFERENCE                50 //milli volt
 
//Configured cell modules use i2c addresses 24 to 48 (24S)
#define DEFAULT_SLAVE_ADDR_START_RANGE        24
#define DEFAULT_SLAVE_ADDR_END_RANGE DEFAULT_SLAVE_ADDR_START_RANGE + 24 

union {uint16_t val; uint8_t buffer[2];} uint16_t_to_bytes;  
const byte MODULE_ID = NUMBER_MODULES_IN_THE_SYSTEM;
const byte COUNT = NUMBER_MODULES_IN_THE_SYSTEM;
volatile uint8_t Counter_for_Bypass  = BYPASS_TIME;
typedef struct cell_module {int volt; byte id;} records; records celledata[COUNT]; 
uint8_t i=0;

//----------------------------------------------SEND COMMANDO-----------------------------------------------------------------------
//Send only 2 values.   1=cell_id 2=cmd 
uint8_t  send_command(uint8_t cell_id, uint8_t cmd) 
{
  Wire.beginTransmission(cell_id); // transmit to device
  Wire.write(cmd);  //Command configure device address
  uint8_t ret = Wire.endTransmission();  // stop transmitting
  delay(delay_ms);
  return ret;
}

//Send 3 values, the last with int8. 1=cell_id 2=cmd 3=value
uint8_t  send_command(uint8_t cell_id, uint8_t cmd, uint8_t byteValue) 
{
  Wire.beginTransmission(cell_id); // transmit to device
  Wire.write(cmd);  //Command configure device address
  Wire.write(byteValue);  //Value
  uint8_t ret = Wire.endTransmission();  // stop transmitting
  delay(delay_ms);
  return ret;
}
//Send 3 værdier, den sidste med int16.  1=cell_id 2=cmd 3=værdien
uint8_t send_command(uint8_t cell_id, uint8_t cmd, uint16_t Value) 
{
  uint16_t_to_bytes.val = Value;
  Wire.beginTransmission(cell_id); // transmit to device
  Wire.write(cmd);  //Command configure device address
  Wire.write(uint16_t_to_bytes.buffer[0]);
  Wire.write(uint16_t_to_bytes.buffer[1]);
  uint8_t ret = Wire.endTransmission();  // stop transmitting
  delay(delay_ms);
  return ret;
}
//--------------------------------------------------COMMAND_BIT-------------------------------------------------------------------
uint8_t cmdByte(uint8_t cmd) 
{
  bitSet(cmd, COMMAND_BIT);
  return cmd;
}
//--------------------------------------------------READ VALUES-------------------------------------------------------------------
//receive 3 values, first with int16
uint16_t read_uint16_from_cell(uint8_t cell_id, uint8_t cmd) 
{
  send_command(cell_id, cmd);
  uint8_t status = Wire.requestFrom((uint8_t)cell_id, (uint8_t)2);
  uint8_t buffer[4];
  buffer[0] = (uint8_t)Wire.read();
  buffer[1] = (uint8_t)Wire.read();
  delay(delay_ms);
  return word(buffer[0], buffer[1]);
}
//modtag 3 værdier første med int8
uint8_t read_uint8_t_from_cell(uint8_t cell_id, uint8_t cmd) {
  send_command(cell_id, cmd);
  uint8_t status = Wire.requestFrom((uint8_t)cell_id, (uint8_t)1);
  uint8_t buffer = (uint8_t)Wire.read();
  delay(delay_ms);
  return buffer;
}
//-----------------------------------------------COMMAND CALLS----------------------------------------------------------------------
uint8_t command_factory_reset(uint8_t cell_id) 
{
  return send_command(cell_id, cmdByte( COMMAND_factory_default )); 
}

//1=cell_id 2=cmd 3=ny adresse
uint8_t command_set_slave_address(uint8_t cell_id, uint8_t newAddress) 
{
  return send_command(cell_id, cmdByte( COMMAND_set_slave_address ), newAddress);
}

uint8_t command_set_bypass_voltage(uint8_t cell_id, uint16_t  value) 
{
  return send_command(cell_id, cmdByte(COMMAND_set_bypass_voltage), value);
}
//-----------------------------------------------READ CALLS----------------------------------------------------------------------

uint16_t cell_read_voltage(uint8_t cell_id) 
{
  return read_uint16_from_cell(cell_id, read_voltage); //10
}

uint16_t cell_read_bypass_enabled_state(uint8_t cell_id) 
{
  return read_uint8_t_from_cell(cell_id, read_bypass_enabled_state); //16
}

uint16_t cell_read_raw_voltage(uint8_t cell_id) 
{
  return read_uint16_from_cell(cell_id, read_raw_voltage); //14
}

uint16_t cell_read_error_counter(uint8_t cell_id) 
{
  return read_uint16_from_cell(cell_id, read_error_counter); //15
}

uint16_t cell_read_board_temp(uint8_t cell_id) 
{
  return read_uint16_from_cell(cell_id, read_temperature);  //11
}
//------------------------------------------------ SETUP -----------------------------------------------------------------------
void setup() 
{
  wdt_disable();                  // not yet
  Wire.begin(); //SDA/SCL         // now for Nano
  Wire.setTimeout(1000);          //1000ms timeout
  Wire.setClock(100000);          //100khz on the bus
  
  sei();    
  BusInit();                      // Init the modules
  wdt_enable(WDTO_4S);            // start watchdog
}
//----------------------------------------------------------------------------------------------------------------------
uint8_t cell_id = DEFAULT_SLAVE_ADDR;    

//------------------------------------------------BUBBLESORT------------------------------------------------------------
// callback function for doing comparisons
template<typename T>
int myCompareFunction (const void * arg1, const void * arg2)
  { 
  T * a = (T *) arg1;  // cast to pointers to type T
  T * b = (T *) arg2;
  // a less than b? 
  if (*a < *b)
    return -1;
  // a greater than b?
  if (*a > *b)
    return 1;
  // must be equal
  return 0;
  }  // end of myCompareFunction

//-------------------------------------------------Init Modules------------------------------------------------------------------
void BusInit() 
{
  uint8_t address,i=0;

 // Serial.println();      //4 bytes    
  
  for (uint8_t j=0; j < MODULE_ID; j++)
  {
   celledata[j].id   = 0; //der skal initialiseres
   celledata[j].volt = 0; //der skal initialiseres
  }

  Serial.println("i2c SCAN for Modules: "); 
  for ( address = DEFAULT_SLAVE_ADDR; address <= DEFAULT_SLAVE_ADDR_END_RANGE; address++ ) 
   {
     Serial.print("Looking for addresse "); Serial.println(address);  
     Wire.beginTransmission(address);
     byte error = Wire.endTransmission();
     if (error == 0) //found one
      {
        Serial.print("Found i2c address ");
        Serial.println(address);
        cell_id = address;
        celledata[i].id=address;  //add the address to the array
         Serial.print("New Module: "); Serial.print(i);  Serial.print(" on address: "); Serial.println(address); 
        i++;
        command_set_slave_address(cell_id, (uint8_t)address);
        if (address==DEFAULT_SLAVE_ADDR)
        for (uint8_t address = DEFAULT_SLAVE_ADDR_START_RANGE; address <= DEFAULT_SLAVE_ADDR_END_RANGE; address++ )  
         {
            Serial.println();          
            Serial.print("Provisioning address ");
            command_set_slave_address(DEFAULT_SLAVE_ADDR, (uint8_t)address);
            Serial.print("new address=");
            Serial.println(address);
            cell_id = address;
            celledata[i].id=address;  //add the address to the array
         }
      }
   }
  uint8_t cell_sort [COUNT] = {celledata[0].id, celledata[1].id, celledata[2].id};
  qsort (cell_sort, MODULE_ID, sizeof (byte), myCompareFunction<byte>); //Husk at bruge byte her, da cell_id = byte
  i=0;  
}
//------------------------------------------------- SORTERING ------------------------------------------------------------------
void SortByVolt() 
{
  uint16_t SortedOnVolt [MODULE_ID][2] ={ {celledata[0].volt, celledata[0].id}, {celledata[1].volt, celledata[1].id },{celledata[2].volt, celledata[2].id }};
 
  //Data in cells not read correct. No dynamic memory saved if removed
  if (celledata[0].volt<3000) return;
  if (celledata[1].volt<3000) return;
  if (celledata[2].volt<3000) return;

  qsort (SortedOnVolt, MODULE_ID, 2*sizeof (int), myCompareFunction<int>); //Kun qsort bliver stående, resten kan remmes

  if (SortedOnVolt[0][0]<(SortedOnVolt[2][0]-MODULE_VOLT_DIFFERENCE))   //is the higest more than MODULE_VOLT_DIFFERENCE higer than the lowest, then turn on the resistor on the higest
    { 
      cell_id=SortedOnVolt[2][1]; //The cell with the higest value

      //Make a cooldown periode for 30 sec -> BYPASS_TIME
      if (Counter_for_Bypass==BYPASS_TIME)
        {
          for (byte k=0; k<3; k++)
            {
              command_set_bypass_voltage(celledata[k].id, 1000);  //reset the rest of the modules, so only one is active at the time
            } 
          command_set_bypass_voltage(cell_id, 4100);             // 4000 er bare en tilfældig værdi. den skal bare være mellem 3000 og 4200, så tænder modstanden.   
        }
     
      if ( Counter_for_Bypass<=BYPASS_TIME)
        Counter_for_Bypass--;

      if ( Counter_for_Bypass==0)                             //we are ready again
        Counter_for_Bypass=BYPASS_TIME;   
    } 
}
//------------------------------------------------- LOOP ------------------------------------------------------------------
void loop() 
{
 
 uint16_t data16;
 uint8_t ModuleCount=0;
 
 Serial.print("i2c SCAN for Modules: "); //  Show the connected devices on the BUS 
 for (uint8_t address = DEFAULT_SLAVE_ADDR; address <= DEFAULT_SLAVE_ADDR_END_RANGE; address++ ) // for(address =21; address<= 48; address++)
   {
     Wire.beginTransmission(address);
     byte error = Wire.endTransmission();
     if (error == 0)
       {
         Serial.print(address);
         Serial.print(' ');
         cell_id = address;
         celledata[ModuleCount].id=address;  //add the address to the array
         ModuleCount++;
       }
   }
 Serial.println(".");
 Serial.print("Module "); Serial.print(i); Serial.print(" = "); Serial.print(celledata[i].id);
 Serial.print(" Address "); Serial.print(" = "); Serial.println(celledata[i].id);
 celledata[i].volt=cell_read_raw_voltage(celledata[i].id); //14
 if (celledata[i].volt<1000){ celledata[i].volt=1; }
 if (celledata[i].volt>5000){ celledata[i].volt=1; }
 if (celledata[i].volt==255) {celledata[i].volt=1; }
 Serial.print("Voltage = "); Serial.print(celledata[i].volt);
 data16 = cell_read_board_temp(celledata[i].id);  //11
 if (data16>500) data16=0;
 if (data16==255) data16=0;
 Serial.print("  Temperature = "); Serial.println(data16);
 Serial.println("------------------------------------------");
 delay(100);
 digitalWrite(13, LOW);  // green diode
 delay(100);
 if (i>=2) 
   {
     i=0;               // only sort when we are done reading all modules  
     SortByVolt();      // Sort the values from volt (last_raw_adc)
   }
 else     
   i++; 

 wdt_reset(); // reset WDT 
 delay(500);
 digitalWrite(13, HIGH);  // green diode
}//end of loop
//------------------------------------------------------------------------------------------------------------
