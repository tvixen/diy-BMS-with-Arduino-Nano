//BMS module software for Arduino Nano
// Version: Arduino IDE 1.8.5  + Cloud
//
// Originally code written by Stuart Pittaway, and heavyly modified by Tim Milgart
//
// Start Date: 13.01.2018
/*
 * This code is supposed to work with a OLED and a 16bit ADC chip called ADS1115
   

   REMEMBER TO CHANGE THE ADDRESS ON THE ADC AND OLED BEFORE UPLOAD !!!   NEW PCB VERSION TO COME!

   This is the code for the Slave - it talks to the Master over isolated i2c bus.  The code will fit into a Arduoni Nano, but the code is compiled for a Uno
   because it will give us some more space, as the bootsection is loaded with Optiboot.
   Update 28/6-2018
   Removed several updates.... 
   Update 02/07-2018
   Removed all GreenLED activities, so it's only my own LED that works. incl inline code for the ATTiny85 chip
   Update 05/07-2018
   Creates brand new design. So only the relevant call is retained.
   3 values are now on the i2c bus.
   Update 05/07-2018 
   Volt and temp is working, but still I2C problems
   Update 06.07.2018
   Added a watchdog...the easy way to get the I2C bus behave
   Sketch uses 16010 bytes (55%) of program storage space. Maximum is 28672 bytes.
   Global variables use 888 bytes (34%) of dynamic memory, leaving 1672 bytes for local variables. Maximum is 2560 bytes. 
   Update 08.07.2018
   Removed unused functions and calls ..still need to remove unused code.
   Update 09.07.2018
   Discovered that the chip is turned around 180 degrees, so the bus side with the high current is pointing towards the cold side of ALUM1250.
   Also the resistors and caps is incorrect. This will be solved in a new version of my PCB. Along with a new SPI display
   Update 12.07.2018
   Added Check_if_OverCharged(). This procedure can turn on the resistor if the voltage is over 4.230 volts 
   Added PackNo, so the module number is showen in the dispaly along with the voltage.
   Update 13.07.2018
   Did a fair cleanup of the code. Deleted some debug information.
   Sketch uses 13510 bytes (41%) of program storage space. Maximum is 32256 bytes.
   Global variables use 865 bytes (42%) of dynamic memory, leaving 1183 bytes for local variables. Maximum is 2048 bytes.
*/


#include <EEPROM.h>
#include <avr/sleep.h>   // til sleep og interrupt
#include <avr/wdt.h>     // watchdog
#include <Wire.h>        // kan tale med Alle Arduino print

#include <Adafruit_ADS1015.h>    // include ADC 1115 
#define I2C_ADC_ADDRESS 0x4A
Adafruit_ADS1115 ads1115(I2C_ADC_ADDRESS);  // construct an ads1115 at address xxx

#include "SSD1306Ascii.h"        // Font for Oled
#include "SSD1306AsciiWire.h"    // Font of Wire for Oled
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C         // Oled address

// Define proper RST_PIN if required.
#define RST_PIN -1              // Reset pin for Oled
SSD1306AsciiWire oled;          // Construct of AsciiWire 


//Where in EEPROM do we store the configuration
#define EEPROM_CHECKSUM_ADDRESS 0
#define EEPROM_CONFIG_ADDRESS 20

#define MIN_BYPASS_VOLTAGE 3000U
#define MAX_BYPASS_VOLTAGE 4200U

//Number of cycles between voltage reading checks (20 = approx 20 seconds)
#define BYPASS_COUNTER_MAX 20

//Default i2c SLAVE address (used for auto provision of address)
#define DEFAULT_SLAVE_ADDR 21

//Configured cell modules use i2c addresses 24 to 48 (24S)
#define DEFAULT_SLAVE_ADDR_START_RANGE 24

//If we receive a cmdByte with BIT 6 set its a command byte so there is another byte waiting for us to process
#define COMMAND_BIT 6

#define COMMAND_factory_default 3
#define COMMAND_set_slave_address 4
#define COMMAND_set_bypass_voltage 8

#define read_voltage 10
#define read_temperature 11
#define read_voltage_calibration 12
#define read_raw_voltage 14
#define read_error_counter 15
#define read_bypass_enabled_state 16

// Port define 
#define DischargePin 2    // Red led
#define GreenLedPin  13   // Green led

volatile float ADCvalue =4.201; //value from ADC
volatile uint16_t results = 0;  //mellem resultat fra ADC
volatile uint16_t temperature_probe = 0;
volatile uint8_t reading_count = 0;
volatile uint8_t cmdByte = 0;
volatile uint8_t last_i2c_request = 255;
volatile uint16_t last_raw_adc = 0;
volatile uint8_t ByPassCounter = 0;
volatile boolean ByPassEnabled = false;
volatile uint16_t TemperaturValue = 0;
volatile uint8_t PacketNo = 0;
volatile bool EventFlag   = false;
uint16_t error_counter = 0;


//Default values
struct cell_module_config 
{
  uint8_t SLAVE_ADDR = DEFAULT_SLAVE_ADDR;
  uint16_t ADC_VCC_Value = 4201;
  uint16_t TemperatureProbe = 10;
};

static cell_module_config myConfig;

void setup() 
{
  Wire.setTimeout(1000);
  Wire.setClock(100000);  //100khz
  Wire.begin();    
  Serial.begin(115200);
  
  pinMode(GreenLedPin, OUTPUT);  //min kode for greenled
  digitalWrite(GreenLedPin, LOW); 
    
  pinMode(DischargePin, OUTPUT); //PB4 = PIN 2  //Mit print siger D2
  digitalWrite(DischargePin, LOW);
  
  wdt_disable(); // not yet

  //3 x blink 
  for (int led=0; led <=6; led++)
   {
     BlinkGreenLed();  //tænder grønled
     delay(200);
   }
  
  //Load our EEPROM configuration, if bad write something
  if (!LoadConfigFromEEPROM()) 
    {
     Serial.println("No Config in EEprom. Module is not initialized = 21");  
     for (int led=0; led <=24; led++)
      {
        BlinkGreenLed();  //tænder grønled
        delay(50);
      }
    }

   //Load our EEPROM configuration, and put it over in our global variables
   PacketNo=myConfig.SLAVE_ADDR;
   last_raw_adc=myConfig.ADC_VCC_Value;
   TemperaturValue=myConfig.TemperatureProbe;

  sei();                  // Enable Global Interrupts
  initOled();             // Oled display startup
  initADC();              // call ads1115.begin();
  ReadTemp();             // read temp
  ReadADC();              // read voltage
  PackNumber();           // Battery pack get its number in the display
  drawVoltage();          // Write to  OLED
  Check_if_OverCharged(); //Check the voltage...is it overcharged

  Serial.println(" Setup done for version Cell Module 16");

  init_i2c();             //init the communication
  wdt_enable(WDTO_4S);    //Watchdog set to 4 sec
}
//----------------------------------------------------------------------------------------------------------------------

boolean inPanicMode = false;
void panic() 
{
  inPanicMode = true;
}
//----------------------------------------------------------------------------------------------------------------------
void loop() 
{
  wdt_reset();           // Watchdog reset
  CheckBypass();         // Check voltage
  ReadTemp();            // Read Temp 
  ReadADC();             // Read ADC
  drawVoltage();         // Read ADCvalue 
  Check_if_OverCharged();// Check battery voltage
  

  if (last_i2c_request > 0) 
   {
     //Count down loop for requests to see if i2c bus hangs or controller stops talking
     last_i2c_request--;
   }

  //If we are on the default SLAVE address then use different LED pattern to indicate this
  if (myConfig.SLAVE_ADDR == DEFAULT_SLAVE_ADDR) // DEFAULT_SLAVE_ADDR=21
   {
     Serial.println("Packet number = 21 = Unconfigured!");
   } 
  else 
   {
     Serial.print("Packet number =");Serial.println(PacketNo);
     if (last_i2c_request == 0 && inPanicMode == false) 
      {
        //Panic after a few seconds of not receiving i2c requests...
        Serial.println("Panic, not receiving i2c requests! = init_i2c");
        panic();

        //Try resetting the i2c bus
        Wire.end();
        init_i2c();
        error_counter++;
      }

     if (last_i2c_request > 0 && inPanicMode == true) 
      {
        inPanicMode = false;
      }
   } //else
}
//----------------------------------------------EEPROM READ/WRITE--------------------------------------------------------------
uint32_t calculateCRC32(const uint8_t *data, size_t length) // used in write & load configtoEEprom
{
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

void WriteConfigToEEPROM() {
  EEPROM.put(EEPROM_CONFIG_ADDRESS, myConfig);
  EEPROM.put(EEPROM_CHECKSUM_ADDRESS, calculateCRC32((uint8_t*)&myConfig, sizeof(cell_module_config)));
}

bool LoadConfigFromEEPROM() {
  cell_module_config restoredConfig;
  uint32_t existingChecksum;

  EEPROM.get(EEPROM_CONFIG_ADDRESS, restoredConfig);
  EEPROM.get(EEPROM_CHECKSUM_ADDRESS, existingChecksum);

  // Calculate the checksum of an entire buffer at once.
  uint32_t checksum = calculateCRC32((uint8_t*)&restoredConfig, sizeof(cell_module_config));

  if (checksum == existingChecksum) {
    //Clone the config into our global variable and return all OK
    memcpy(&myConfig, &restoredConfig, sizeof(cell_module_config));
    return true;
  }

  //Config is not configured or gone bad, return FALSE
  return false;
}

void factory_default() {
  EEPROM.put(EEPROM_CHECKSUM_ADDRESS, 0);
}
//------------------------------------------------------------------------------------------------------------

void init_i2c() {
  Wire.begin(myConfig.SLAVE_ADDR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
}
//-----------------------------------------REBOOT-------------------------------------------------------------------

void Reboot() 
{
  while (1)     //Infinity loop
  { 
    ledOff(); 
  }
}
//-----------------------------------------BLINK-------------------------------------------------------------------

bool Ledon;
inline void BlinkGreenLed() //LED on or OFF
{ Serial.println("on/off"); 
  if(Ledon)
    digitalWrite(GreenLedPin, HIGH); 
  else
    digitalWrite(GreenLedPin, LOW); 
  Ledon=!Ledon;
}
//-----------------------------------------LED OFF-----------------------------------------------------------------
inline void ledOff() 
{
  digitalWrite(GreenLedPin, LOW); // 2 byte kan spares ved at skrive asm kode i forholdt til digitalwrite
}
//-------------------------------------------SEND-----------------------------------------------------------------
void sendUnsignedInt(uint16_t number) 
{
  Wire.write((byte)((number >> 8) & 0xFF));
  Wire.write((byte)(number & 0xFF));
}

void sendByte(uint8_t number) 
{
  Wire.write(number);
}
//--------------------------------------UNION----------------------------------------------------------------------
union 
{
  uint16_t val;
  uint8_t b[2];
} uint16_t_to_bytes;

uint16_t readUINT16() 
{
  uint16_t_to_bytes.b[0] = Wire.read();
  uint16_t_to_bytes.b[1] = Wire.read();
  return uint16_t_to_bytes.val;
}
//---------------------------------------receiveEvent---------------------------------------------------------------------
// function that executes whenever COMMAND data is received from master
void receiveEvent(int howMany)
{
  if (howMany <= 0) return;
  if (cmdByte != 0) error_counter++;
  cmdByte = Wire.read();
  howMany--;

  if (bitRead(cmdByte, COMMAND_BIT)) 
    {
      bitClear(cmdByte, COMMAND_BIT);
      switch (cmdByte) 
        {
          case COMMAND_factory_default: //3 hvis addresser skal fornyes
           {
             factory_default();
             Reboot();
           }
          break;
          
         case COMMAND_set_slave_address: //4 .. Set i2c slave address and write to EEPROM, then reboot
         if (howMany == 1 ) 
           {
              uint8_t newAddress = Wire.read();
              PacketNo=newAddress;
              if (newAddress != myConfig.SLAVE_ADDR && newAddress >= DEFAULT_SLAVE_ADDR_START_RANGE && newAddress <= 48) 
                {
                  myConfig.SLAVE_ADDR = newAddress;
                  PacketNo=newAddress;
                  WriteConfigToEEPROM();
                  Reboot();
                }
           }
         break;

         case COMMAND_set_bypass_voltage:  // 8  Værdien hvor modstanden skal tænde ... skal være fast, så når det er lavet slettes denne her.
         if (howMany == sizeof(uint16_t)) 
           {
              uint16_t newValue = readUINT16();
              if (newValue >= MIN_BYPASS_VOLTAGE && newValue <= MAX_BYPASS_VOLTAGE ) 
               {
                 ByPassCounter = BYPASS_COUNTER_MAX; //BYPASS_COUNTER_MAX =240
                 ByPassEnabled = true;
               } 
             else 
               {
                 ByPassCounter = 0;
                 ByPassEnabled = false;
               }
           }
         break;
        } //switch (cmdByte)

    cmdByte = 0;
    } 
  else 
    {
      //Its a READ request

      switch (cmdByte) 
       {
         case read_voltage: //10
           {  
             //Serial.print("read_voltage. CmdByte = ");
             //Serial.println(cmdByte);
           }
         break;
       }
    }
  EventFlag=true;
  // clear rx buffer
  while (Wire.available()) Wire.read();
}
//---------------------------------------requestEvent---------------------------------------------------------------------
// function that executes whenever data is requested by master (this answers requestFrom command)
void requestEvent() 
{
  switch (cmdByte) 
   {
     case read_raw_voltage: 
       {
         sendUnsignedInt(last_raw_adc); //14
       } 
     break;

     case read_error_counter: 
       {
         sendUnsignedInt(error_counter); //15
       }
     break;

     case read_bypass_enabled_state: 
       {
         sendByte(ByPassEnabled); //16
       }  
     break;

     case read_temperature: 
       {
         myConfig.TemperatureProbe= TemperaturValue;
         sendUnsignedInt(TemperaturValue); // ved ikke om det her virker
       }
     break;

     case read_voltage_calibration:
       { 
         myConfig.ADC_VCC_Value = last_raw_adc; 
         sendUnsignedInt(myConfig.ADC_VCC_Value);
       }
      break;

 
      default://Dont do anything - timeout
      break;
   }//switch (cmdByte) 

  //Clear cmdByte
  cmdByte = 0;

  //Reset when we last processed a request, if this times out master has stopped communicating with module
  last_i2c_request = 150;
}
//---------------------------------------Temperatur på Batteri---------------------------------------------------------------------
void ReadTemp() //Læs temp fra PTC føler
{
  uint16_t value = 0;  
   
  for(int i=0;i<10;i++)         // taking 10 samples from sensors with a inerval of 2msec and then average the samples data collected
    {
      value   += analogRead(6); //read the voltage from Pin A6
      delay(2);
    }
    
   value=value/10;              //average of the 10 measurements
   TemperaturValue=value/22;    //over in global variable
}
//----------------------------------Balance del--------------------------------------------------------------------------
void CheckBypass()              //control of the voltage, is it too high then the resistance must take over.
{
  
  if (ByPassEnabled)
   { 
     if (ByPassCounter > 0)      // blir 240 når den blir sat
      {
        //We are in ACTIVE BYPASS mode - the RESISTOR will be ACTIVE + BURNING ENERGY
        ByPassCounter--;          //tæl 1 ned
        digitalWrite(DischargePin, HIGH);      //Turn on the power
        Serial.println("ACTIVE BYPASS MODE!");   // test only
      
        if (ByPassCounter == 0)
         {
           digitalWrite(DischargePin, LOW);     //Turn it off again
         }
       }  
    }
  else 
    digitalWrite(DischargePin, LOW);         //Safety check we ensure bypass is always off if not enabled. (Do to communication error)

   
  BlinkGreenLed();                              //turn on the Green LED shows that we have read the Temp
  reading_count++;
}
//------------------------------------------------------------------------------------------------------------
void initADC()
{
  ads1115.begin();                           // start of ADC ADS1115
}
//------------------------------------------------------------------------------------------------------------
void ReadADC()
{
  ADCvalue=0;
  results =0 ;
  
  float multiplier = 0.1876F;                // ADS1115  @ +/- 6.144V gain (16-bit results) 
  results = ads1115.readADC_Differential_0_1();  
 
  ADCvalue=results*multiplier;               // put the result in the float to get all the digits
  last_raw_adc = ADCvalue;                   // convert the flot to an integer
}
//------------------------------------------------------------------------------------------------------------
void initOled() 
{
  
#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  oled.setFont(Verdana12);
  oled.set1X();
  oled.clear();
  oled.setCursor(0,0);
  oled.setLetterSpacing(1);
  oled.print("Made By Tim Milgart");  
  delay(1000);
  oled.clear();  
}
//------------------------------------------------------------------------------------------------------------
void PackNumber()       // Show the number of the module on the screen
{ 
  oled.set1X();
  oled.setFont(TimesNewRoman16_bold);
  oled.clear();
  oled.setCursor(85,6);
  oled.print("No");
  oled.print(PacketNo);
  oled.setCursor(40,6);
  oled.print("Volts");
  oled.setCursor(27,6);
  oled.print("'");
  oled.setFont(lcdnums14x24);
  oled.set2X();
}
//------------------------------------------------------------------------------------------------------------
void drawVoltage(void)    //Show the voltage of the module on the screen
{
if (results>6100)         //Remember the results is without comma... so 5 cifre before comma
  {
    if (EventFlag)        //OLED is only cleared when there has been eventcall ... that is not enough .. still on the head sometimes
      {
        oled.begin(&Adafruit128x64, I2C_ADDRESS);
        oled.set1X();
        oled.setFont(TimesNewRoman16_bold);
        oled.clear();
        oled.setCursor(85,6);
        oled.print("No");
        oled.print(PacketNo);
        oled.setCursor(40,6);
        oled.print("Volts");
        EventFlag=false;
      }
    oled.setCursor(27,6);
    oled.print("'");
    oled.set2X();
    oled.setFont(lcdnums14x24);
    oled.setCursor(0,0);
    Serial.println("-----------------------------------------------------");
    Serial.print("ADC Value ="); Serial.println(ADCvalue); 
    Serial.print("TemperaturValue ="); Serial.println(TemperaturValue); Serial.println();
    Serial.println("-----------------------------------------------------");
    oled.print(ADCvalue,0); 
  } 
else 
  {
    oled.set2X();                              // sætter størrelse på bogstaver
    oled.clear(0 ,128, 0, 5);                  // sletter teksten
    oled.clear(0 ,30, 0, 6);                   // sletter teksten
    oled.setFont(TimesNewRoman16_bold);        // sæt ny font   
    oled.setCursor(0,1);                       // placer cursor
    oled.print("No input    ");                // skriv på skærm
    delay(500);
    oled.clear(0 ,128, 0, 5);                  // sletter teksten
    oled.set1X();                              // sætter størrelse på bogstaver
    oled.setCursor(25,1);                      // placer cursor
    oled.print("Made By ");                    // skriv på skærm
    oled.setCursor(15,3);                      // placer cursor
    oled.print("Tim Milgart ");                // skriv på skærm
  }
}
//------------------------------------------------------------------------------------------------------------
void Check_if_OverCharged() // This is done as an additional local security for overvoltage and has nothing to do with balance.
{
  if (results>22640) 
    {
      
      digitalWrite(DischargePin, HIGH);   //Start discharge
      Serial.println("DISCHARGING !!!");
      delay(1000);                        //lets give it some time to discharge 
    } 
  else 
    {  
     if (!ByPassEnabled)
     digitalWrite(DischargePin, LOW);     //Stop discharge
     delay(1000);                         //Cool down 
    } 
}
//------------------------------------------------------------------------------------------------------------
