
// MAX6675 library - Version: Latest
#include <max6675.h>

// Wire for avr - Version: Latest
#include <Wire.h>

// Adafruit ADS1X15 - Version: 1.1.1
#include <Adafruit_ADS1015.h>

// Adafruit MCP4728 - Version: Latest
#include <Adafruit_MCP4728.h>

// SPI for avr - Version: Latest
#include <SPI.h>

// Adafruit MAX31856 library - Version: Latest
#include <Adafruit_MAX31856.h>
#define DRDY_PIN 13

// Adafruit TCA9548A Multiplexer definition
#define TCAADDR 0x70

#include <Adafruit_MCP4725.h>

/*
  -> This sketch is a combination of the following sketches repared to read and
  write the analog signals using the analog in pins and PWM pins on the Arduino
  Mega board.The following list shares more light on the usefullness of this
  code.
  -> Pins A0 to A15 = Analog Input pins = Signals from 5 MFCs (10 pins), 2 MFMs
  (4 pins) and Back Pressure Regulator (2 pins)
  -> Pins 0 to 1 = Serial Communication = Tx, Rx communication with 1 MFC
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Pins are defined or assigned in groups on basis of its types then numerically
// in ascending order in this section to its appropriate variables Analog In
// Header Pins
const int aipm1 = A0; // ai = analog input, p = positive pin, m = mfc, n = number
const int ainm1 = A1; // ai = analog input, p = negative pin, m = mfc, n = number

const int aipm2 = A2;
const int ainm2 = A3;

const int aipm3 = A4;
const int ainm3 = A5;

const int aipm4 = A6;
const int ainm4 = A7;

const int aipm5 = A8;
const int ainm5 = A9;

const int aipmm1 = A10; // Pins are open
const int ainmm1 = A11; // Pins are open

const int aipmm2 = A12; // Pins are open
const int ainmm2 = A13; // Pins are open

const int aipbpr1 = A14; // bpr = Back Pressure Regulator, Single Ended Reference Pin
const int ainbpr1 = A15; // bpr = Back Pressure Regulator, Ground Pin

// Digital (PWM) Output Header Pins
const int Relay1 = 2; // Sunfounder 4-Channel Relay, Channel#1 Switch
const int Relay2 = 3; // Sunfounder 4-Channel Relay, Channel#2 Switch
const int Relay3 = 4; // Sunfounder 4-Channel Relay, Channel#3 Switch
const int Relay4 = 5; // Sunfounder 4-Channel Relay, Channel#4 Switch
/*
  const int ****** = 6;
  const int ****** = 7;
  const int ****** = 8;
  const int ****** = 9;
  const int ****** = 10;
  const int ****** = 11;
  const int ****** = 12;
  const int ****** = 13; //Already defined at the start of the code
*/

// (Serial) Communication Header Pins
/*
  const int ****** = 0;  // RX0
  const int ****** = 1;  // TX0
  const int ****** = 14; // RX3
  const int ****** = 15; // TX3
  const int ****** = 16; // RX2
  const int ****** = 17; // TX2
  const int ****** = 18; // RX1
  const int ****** = 19; // TX1
  const int ****** = 20; //Being used for SDA commuincation but doesn't have to defined in the code for Arduino
  const int ****** = 21; //Being used for SCL communication but doesn't have to defined in the code for Arduino
*/

// Digital Input/Output Header Pins
/*
  const int ****** = 22;
  const int ****** = 23;
  const int ****** = 24;
  const int ****** = 25;
  const int ****** = 26;
  const int ****** = 27;
*/
const int TC5DI = 28;   // Adafruit MAX38156 SDI or DI Pin (TC5 Thermocouple)
const int TC5CS = 29;   // Adafruit MAX38156 CS Pin (TC5 Thermocouple)
const int TC5CLK = 30;   // Adafruit MAX38156 SCK or CLK Pin (TC5 Thermocouple)
const int TC5DO = 31;   // Adafruit MAX38156 SDO or DO Pin (TC5 Thermocouple)
const int TC2DI = 32;   // Adafruit MAX38156 SDI or DI Pin (TC2 Thermocouple)
const int TC2CS = 33;   // Adafruit MAX38156 CS Pin (TC2 Thermocouple)
const int TC2CLK = 34;   // Adafruit MAX38156 SCK or CLK Pin (TC2 Thermocouple)
const int TC2DO = 35;   // Adafruit MAX38156 SDO or DO Pin (TC2 Thermocouple)
const int TC7DO = 36;  // Generic MAX6675 DO pin (TC7 Thermocouple)
const int TC7CS = 37;  // Generic MAX6675 CS pin (TC7 Thermocouple)
const int TC7CLK = 38; // Generic MAX6675 CLK pin (TC7 Thermocouple)
const int TC6DO = 39;  // Generic MAX6675 DO pin (TC6 Thermocouple)
const int TC6CS = 40;  // Generic MAX6675 CS pin (TC6 Thermocouple)
const int TC6CLK = 41; // Generic MAX6675 CLK pin (TC6 Thermocouple)
const int TC3DO = 42;  // Generic MAX6675 DO pin (TC5 Thermocouple)
const int TC3CS = 43;  // Generic MAX6675 CS pin (TC5 Thermocouple)
const int TC3CLK = 44; // Generic MAX6675 CLK pin (TC5 Thermocouple)
const int TC4DO = 45;  // Generic MAX6675 DO pin (TC4 Thermocouple)
const int TC4CS = 46;  // Generic MAX6675 CS pin (TC4 Thermocouple)
const int TC4CLK = 47; // Generic MAX6675 CLK pin (TC4 Thermocouple)
// const int ***** = 48;
// const int ***** = 49;
const int TC1DI = 50;   // Adafruit MAX38156 SDI or DI Pin (TC1 Thermocouple)
const int TC1DO = 51;   // Adafruit MAX38156 SDO or DO Pin (TC1 Thermocouple)
const int TC1CLK = 52;    // Adafruit MAX38156 SCK or CLK Pin (TC1 Thermocouple)
const int TC1CS = 53;  // Adafruit MAX38156 CS Pin (TC1 Thermocouple)

// Initialising all variables to store all the values of the Setpoints and Reads
// in volt Variables to store the signal read in 10-bit format (Range = 2^10)
// Range = 0-1023
// Resolution = 5/1024 = 0.0049 V/bit
int aip_m1; // ai = analog input, p = positive pin, m = mass flow controller, n = number of the instrument (in this case 1)
int ain_m1; // ai = analog input, n = negative pin, m = mfc, n = number of the instrument (in this case 1)
int aip_m2;
int ain_m2;
int aip_m3;
int ain_m3;
int aip_m4;
int ain_m4;
int aip_m5;
int ain_m5;
int aip_mm1; // ai = analog input, p = positive pin, mm = mass flow meter, n = number of the instrument (in this case 1)
int ain_mm1; // ai = analog input, n = negative pin, mm = mass flow meter, n = number of the instrument (in this case 1)
int aip_mm2;
int ain_mm2;
int aip_bpr1; // ai = analog input, p = positive pin, bpr = back pressure regulator, n = number of the instrument (in this case 1)
int ain_bpr1; // ai = analog input, n = negative pin, bpr = back pressure regulator, n = number of the instrument (in this case 1)
int aip_pt1;  // ai = analog input, n = positive pin, pt = pressure transducer, n = number of the instrument (in this case 1) [On Adafruit ADS1115]
int ain_pt1;  // ai = analog input, n = negative pin, pt = pressure transducer, n = number of the instrument (in this case 1) [On Adafruit ADS1115]
int aip_pt2; // [On Adafruit ADS1115]
int ain_pt2; // [On Adafruit ADS1115]
int aip_ndir; // ai = analog input, n = positive pin, NDIR = NDIR for ppm level ammonia detection [On Adafruit ADS1115]
int ain_ndir; // ai = analog input, n = negative pin, NDIR = NDIR for ppm level ammonia detection [On Adafruit ADS1115]

int fr_m1; // variable to store the processed values, fr = flow rate, m = mfc, n = number of instrument
int fr_m2;
int fr_m3;
int fr_m4;
int fr_m5;
int fr_mm1; // variable to store the processed values, fr = flow rate, mm = mfm, n = number of instrument
int fr_mm2; 
int p_bpr1; // variable to store the processed values, p = pressure, bpr = back pressure regulator, n = number of instrument [On Adafruit ADS1115]
int p_pt1;  // variable to store the processed values, p = pressure, pt = pressure transducer, n = number of instrument [On Adafruit ADS1115]
int p_pt2;  // [On Adafruit ADS1115]
int16_t nh3_conc_ndir; // Address 0x48 on pins A0-A1 [On Adafruit ADS1115]
int16_t nh3_conc_ppbLAD; // Address 0x48 on pins A2-A3 [On Adafruit ADS1115]

int sp_fr_m1; // variable to store the setpoint and send as argument to Arduino [Channels for all outputs on Adafruit DAC4728]
int sp_fr_m2; // sp = setpint, fr = flow rate, m = mfc
int sp_fr_m3;
int sp_fr_m4;
int sp_fr_m5;
int sp_p_bpr1; // sp = setpoint, p = pressure, bpr = back pressure regulator

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Defining the Objects for various Adafruit Max31856 thermocouple logging
// modules (Software handled SPI is employed hence all 4 pins need to be
// defined)
Adafruit_MAX31856 tc1 = Adafruit_MAX31856(TC1CS, TC1DI, TC1DO, TC1CLK);
Adafruit_MAX31856 tc2 = Adafruit_MAX31856(TC2CS, TC2DI, TC2DO, TC2CLK);
MAX6675 tc3 = MAX6675(TC3CLK, TC3CS, TC3DO);
// Defining the Objects for various generic Max6675 thermocouple logging modules
// (only 3 pins are present DO, CS and CLK)
MAX6675 tc4 = MAX6675(TC4CLK, TC4CS, TC4DO);
Adafruit_MAX31856 tc5 = Adafruit_MAX31856(TC5CS, TC5DI, TC5DO, TC5CLK);
MAX6675 tc6 = MAX6675(TC6CLK, TC6CS, TC6DO);
MAX6675 tc7 = MAX6675(TC7CLK, TC7CS, TC7DO);


/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sub-sketch to choose from the I2C devices or boards connected to the Adafruit TCA9238A multiplxer breakout board
void tcaselect(uint8_t i) {
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Defining the Objects for various Adafruit I2C devices alongwith their addresses and decriptions
Adafruit_ADS1115 ads1115_1(0x48); // Construct an object with the module Adafruit ADS1115
                                  // which is a 16-bit 4-channel analog output device
                                  // (Digital to Analog Convertor) with the address (0x48)
Adafruit_ADS1115 ads1115_2(0x49); // Construct an object with the module Adafruit ADS1115
                                  // which is a 16-bit 4-channel analog output device
                                  // (Digital to Analog Convertor) with the address (0x49)
Adafruit_ADS1115 ads1115_3(0x4A); // Construct an object with the module Adafruit ADS1115
                                  // which is a 16-bit 4-channel analog output device
                                  // (Digital to Analog Convertor) with the address (0x4A)

// Adafruit MCP4728 4-Channel 12-bit I2C Digital to Analog Convertor
Adafruit_MCP4728 mcp4728_1;
Adafruit_MCP4728 mcp4728_2;

// Adafruit MCP4725 1-Channel 12-bit I2C Digital to Analog Convertor
Adafruit_MCP4725 mcp4725_1;
Adafruit_MCP4725 mcp4725_2;
Adafruit_MCP4725 mcp4725_3;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Defining global variables that will be needed in void loop{} to store sub-strings obtained from the serial port via Serial.readString() command.
// Hereafter, the string will be divided into sub-strings and will go-through a nested if-else and switch functions to execute the approriate set of commands.
// The nested if-else and switch functions are used to reduce loop runtime signficantly. 
// If it is difficult to follow the program flow, once can define if-else ladder inside the void loop{} function which will reduce the complexity but increase the runtime of each loop.
int16_t ads1115_1a, ads1115_1b;
int16_t ads1115_2a, ads1115_2b;
int16_t ads1115_3a, ads1115_3b;
String inString = "";
String CMD = "";
String INST_Code = "";
String INST_SN = "";
String SSP = "";
int len;
float SetValue;
int INST_N;
char terminator = ';';
String RLY_POS = "";
//////////////////////////////////////////////////////////////////////////////////////////////////////////

float factor_10_bit = 1;       // For arduino analog pins 0.0049
float factor_12_bit = 1;     // For MCP4728 DAC boards 0.001221
float factor_16_bit = 1;   // For ADS1115 ADC boards (the range of this board is from -Vref to +Vref, hence the range of 5 V is doubled a factor of 2)0.00015259

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup and Initialisation part of the code for Arduino Mega to run once when
// it loading or rebooting everytime
void setup() {
  Serial.begin(9600);
  delay(5000);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  // Thermocouple data logger module setup and initilisaion
  // Two different types of thermocouple reading boards are uses.
  // The generic one is MAX6675 while the library used to program it is MAX6675 from Adafruit. (Note: This one can only read K type thermocouple)
  // The other one is Adafruit Max31856 while the library used to program it is ADAFRUIT_MAX31856. (This one can read a variety of 2-wire Thermcouples)
  pinMode(DRDY_PIN,INPUT); // DRDY Pin for Adafruit MAX31856 Thermocouple Reader Module
  // Thermocouple TC1
  tc1.setThermocoupleType(MAX31856_TCTYPE_K);
  // Serial.print("The temperature of Thermocouple TC-1 is ");
  // Serial.print(tc1.readThermocoupleTemperature());
  // Serial.print("°C.");
  // Serial.print("TC-1 MAX31856 Initialised with Thermocouple Type K");
  
  // Thermocouple TC2
  tc2.setThermocoupleType(MAX31856_TCTYPE_J);
  // Serial.print("The temperature of Thermocouple TC-2 is ");
  // Serial.print(tc2.readThermocoupleTemperature());
  // Serial.print("°C.");
  // Serial.print("TC-2 MAX31856 Initialised with Thermocouple Type J");

  // Thermocouple TC3
  tc3.readCelsius();
  // Serial.print("The temperature of Thermocouple TC-3 is ");
  // Serial.print(tc3.readCelsius());
  // Serial.print("°C.");
  // Serial.print("TC-3 MAX6675 Initialised with Thermocouple Type K");

  // Thermocouple TC4
  tc4.readCelsius();  
  // Serial.print("The temperature of Thermocouple TC-4 is ");
  // Serial.print(tc4.readCelsius());
  // Serial.print("°C.");
  // Serial.print("TC-4 MAX6675 Initialised with Thermocouple Type K");
  
  // Thermocouple TC5
  tc5.setThermocoupleType(MAX31856_TCTYPE_J);
  // Serial.print("The temperature of Thermocouple TC-5 is ");
  // Serial.print(tc5.readThermocoupleTemperature());
  // Serial.print("°C.");
  // Serial.print("TC-5 MAX31856 Initialised with Thermocouple Type J");
  
  // Thermocouple TC6
  tc6.readCelsius();
  // Serial.print("The temperature of Thermocouple TC-6 is ");
  // Serial.print(tc6.readCelsius());
  // Serial.print("°C.");
  // Serial.print("TC-6 MAX6675 Initialised with Thermocouple Type K");

  // Thermocouple TC7
  tc7.readCelsius();
  // Serial.print("The temperature of Thermocouple TC-7 is ");
  // Serial.print(tc7.readCelsius());
  // Serial.print("°C.");
  // Serial.print("TC-7 MAX6675 Initialised with Thermocouple Type K");
  // Thermocouple data logger module setup and initisation sub-sketch ends
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  // Analog input sub-sketch
  // All MFCs, MFMs, PTs, Detectors are Initialised in this Section
  // Define pin behaviour here
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
 // pinMode(A10, INPUT);
 // pinMode(A11, INPUT);
 // pinMode(A12, INPUT);
 // pinMode(A13, INPUT);
  pinMode(A14, INPUT);
  pinMode(A15, INPUT);

  // Varible definition and initialisation output
  aip_m1 = analogRead(aipm1);
  
  ain_m1 = analogRead(ainm1);

  aip_m2 = analogRead(aipm2);
  
  ain_m2 = analogRead(ainm2);

  aip_m3 = analogRead(aipm3);
  
  ain_m3 = analogRead(ainm3);
  
  aip_m4 = analogRead(aipm4);

  ain_m4 = analogRead(ainm4);
 
  aip_m5 = analogRead(aipm5);

  ain_m5 = analogRead(ainm5);

  aip_mm1 = analogRead(aipmm1);

  ain_mm1 = analogRead(ainmm1);

  aip_mm2 = analogRead(aipmm2);

  ain_mm2 = analogRead(ainmm2);

  aip_bpr1 = analogRead(aipbpr1);

  ain_bpr1 = analogRead(ainbpr1);
  
  // Analog input configuration sub-sketch ends
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  // I2C module Objects Setup and Initialisation Sub-Sketch
  // All I2C devices have their own addreses in hexdecimal format
  // Some have the flexibility to choose one address from multiple addresses (usually 2 choices are available, no more than 4 choices are availables on any I2C device)
  // If there are 2 or more I2C devices with same address connected on the same trasmission lines (SCA and SCL pins, #20 and #21 pins on Arduino Mega), there will be issues with data communication
  // In order to avoid these issues, a multiplexer is used, multiplexer has isolated channels and lets the Arduino communicate to only one of its channel at any given point of time
  // In our case we have 2 MCP4728 with the same address 0x60 and it just comes with a singular address
  // Hence, a TCA9538-A multiplexer is used for resolving this issue
  // Each MCP4728 is connected to Channel#2 and Channel#3 of the multiplexer
  // Also, there are 3 ADS1115 boards connected to Channel#1 of the multiplexer. Eventhough, all three are identical, by shorting certain pins on these boards, one of the 4 addresses can be chosen
  // The 4 addresses for ADS1115 are from 0x48 to 0x4B and hence, all 3 can be connected to the same transmission without any issues
  
  // Choosing the Channel#2 (Channel#0 to Channel#7 (Total: 8 Channels))
  // Initlialising the channels on MCP4728 breakout board present on Channel 2 of multiplexer
//  tcaselect(2);
//  mcp4728_1.begin();
//  if (!mcp4728_1.begin())  
//    mcp4728_1.setChannelValue(MCP4728_CHANNEL_A,0); // MCP4728-1 Channel-A controls setpoint for MFC-1
//    mcp4728_1.setChannelValue(MCP4728_CHANNEL_B,0); // MCP4728-1 Channel-A controls setpoint for MFC-2
//    mcp4728_1.setChannelValue(MCP4728_CHANNEL_C,0); // MCP4728-1 Channel-A controls setpoint for MFC-3
//    mcp4728_1.setChannelValue(MCP4728_CHANNEL_D,0); // MCP4728-1 Channel-A controls setpoint for MFC-4 
//    mcp4728_1.saveToEEPROM();
//
//  // Choosing the Channel#3 (Channel#0 to Channel#7 (Total: 8 Channels))
//  // Initlialising the channels on MCP4728 breakout board present on Channel 3  
//  tcaselect(3);
//  mcp4728_2.begin();
//    mcp4728_2.setChannelValue(MCP4728_CHANNEL_A,0); // MCP4728-1 Channel-A controls setpoint for MFC-5
//    mcp4728_2.setChannelValue(MCP4728_CHANNEL_B,0); // MCP4728-1 Channel-A controls setpoint for BPR-1
//    mcp4728_2.setChannelValue(MCP4728_CHANNEL_C,0);
//    mcp4728_2.setChannelValue(MCP4728_CHANNEL_D,0);
//    mcp4728_2.saveToEEPROM();

  tcaselect(4);
  mcp4725_1.begin();
      mcp4725_1.setVoltage(0,false);

    tcaselect(5);
  mcp4725_2.begin();
      mcp4725_2.setVoltage(0,false);

    tcaselect(6);
  mcp4725_3.begin();
      mcp4725_3.setVoltage(0,false);
  
  // Choosing the Channel#1 (Channel#0 to Channel#7 (Total: 8 Channels))
  // Initlialising the channels on MCP4728 breakout board present on channels of multiplexer
  tcaselect(1);
  ads1115_1.begin();
  ads1115_2.begin();
  ads1115_3.begin();
  // I2C module Objects Setup and Initialisation Sub-Sketch
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  // 4-channel relay configuration sub-sketch
  // Define control pins here
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  pinMode(Relay3, OUTPUT);
  pinMode(Relay4, OUTPUT);

  // Define default configuration here
  digitalWrite(Relay1, HIGH);
  digitalWrite(Relay2, HIGH);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay4, HIGH);
//  Serial.print("All relays have been initialised in off position.");
  // 4-channel relay configuration sub-sketch ends
  ////////////////////////////////////////////////////////////////////////////////////////////////////////
}
/* The format for the serial commands is as follows:
        => Starting Character = : (Colon)
        => Command Mode = PV (Present Value) or SV (Setpoint Value)
        => Instrument Code = AAA (3 Alphabets)
                      - MFC = Mass Flow Controller
                      - MFM = Mass Flow Meter
                      - ADC = Analog to Digital Convertor
                      - DAC = Digital to Analog Convertor
                      - BPR = Back Pressure Regulator
                      - _TC = Thermocouple
                      - RYL = Relay
        => Instrument Number = NN (00 to 99)
        => Setpoint Value = nnn.nnn (Only for commands with SV CMD, there is no limit on the number of digits on before after the decimal point)
        => Use ON/OFF instead of nnn.nnn for Relays       
        => Terminaltion Character = ;
    --> To get the paraneter value the format is :PVAAANN; for example :PVMFM02; will return the value for mass flow rate of MFM-2
    --> To set the parameter value the format is :SVAAANNnn.nnnn; for example :SVDAC121145.235; will set the value of ABC parameter for XYZ instrument connected to 2nd channel of DAC-1
                                                                                                       to 1145.235, in this case the it will set the mass flow rate of MFC-2 to 1145.235 
    --> For relays, the format will be :SVRLYNNON (example :SVRYL01ON) to turn ON and :SVRYLNNOFF; (:SVRYL03OFF;) to turn OFF */

void loop() {

  inString = "";
      CMD = "";
      INST_Code = "";
      INST_SN = "";
      RLY_POS = "";
      SSP = "";
      
      if(Serial.available()){                             // Arduino will only run the loop if it reads that there is some incoming data from the serial port
      inString = Serial.readStringUntil(terminator);          // To read the string (and not converted value of the serial commands) until the terminator character for generating the sub-string
      //Serial.print(inString);                               
      CMD = inString.substring(1,3);                          // Storing the 2nd and 3rd character in variable CMD: PV (Present Value) or SV (Setpoint Value)
      //Serial.print(CMD);                                    
      INST_Code = inString.substring(3,6);                    // Storing the instrument type or code: MFC (Mass Flow Convertor), MFM (Mass Flow Meter), ADC (Analog to Digital Convertor - Analog Input)
      //Serial.print(INST_Code);                                                        //BPR (Back Pressure Regulator), DAC (Digital to Analog Convertor - Analog Out), _TC (Thermocouple), RYL (Relay)
      INST_SN = inString.substring(6,8);                      // Storing the serial number of the instrument as String
      //Serial.print(INST_SN);
      INST_N = INST_SN.toInt();                               // Converting and storing the serial number as integer
      //Serial.print(INST_N);
      len = inString.length();                                // Length of the input string recieved by the serial port without the termination character
      if(len > 8){SSP = inString.substring(9);}               // If more than 9 characters present in string, then store the setpoint which starts from 9th character in variable SSP as String
      
      // Nested if-else and switch case sub-sketches start here
      // The code is boradly divided in 3-parts based on topmost if-else statement.
      // The program is as per the following flow chart:
           /*             Serial Command String In ------> Breaking String in Sub-Strings 
                                                                          |         
                                                                          V        
          Compare Sub-String CMD to dertermine correct If statement with PV or ---------------------------------------------- SV--------------------------------> If not PV or SV, then Error Message
                                                                          |                                                     |
                                                                          V                                                     V
Compare INST_Code Sub-String to determine correct If statement with     MFC,MFM,ADC (NDIR,PT),BPR,_TC                          DAC (MFC,BPR),Relay
Use INST_N to determine the correct Switch case so the appropriate commands can be executed and the correct parameter value can be returned. */

// The present value if statement starts here
      if (CMD == "PV"){ 
            if (INST_Code == "MFC"){    // MFC if block and switch case structure starts here    
              switch(INST_N){           
                case 1:
                //////////////////////////////////////////////////////////////////////////////////////////////////////
                //Analog Input Voltage Reading Script Starts for onboard pins numbered from A0 to A15
                //10-bit Resolution = 0.0049 V/bit@ Vref = 5 V
                //Mass Flow Controllers and Mass Flow Meters
                fr_m1 = aip_m1 - ain_m1; // Value still in 10-bit resolution, needs to be converted to decimal format (multiply by resolution and send value to LabVIEW)
                Serial.print(fr_m1 * factor_10_bit);
                Serial.println(";");
                break;

                case 2:
                fr_m2 = aip_m2 - ain_m2; // Value still in 10-bit resolution, needs to be converted to decimal format (multiply by resolution and send value to LabVIEW)
                Serial.print(fr_m2 * factor_10_bit);
                Serial.println(";");
                break;
        
                case 3:        
                fr_m3 = aip_m3 - ain_m3; // Value still in 10-bit resolution, needs to be converted to decimal format (multiply by resolution and send value to LabVIEW)
                Serial.print(fr_m3 * factor_10_bit);
                Serial.println(";");
                break;        

                case 4:
                fr_m4 = aip_m4 - ain_m4; // Value still in 10-bit resolution, needs to be converted to decimal format (multiply by resolution and send value to LabVIEW)
                Serial.print(fr_m4 * factor_10_bit);
                Serial.println(";");
                break;

                case 5:
                fr_m5 = aip_m5 - ain_m5; // Value still in 10-bit resolution, needs to be converted to decimal format (multiply by resolution and send value to LabVIEW)
                Serial.print(fr_m5 * factor_10_bit);
                Serial.println(";");                
                break;                
                }
              }

 /*            if (INST_Code == "MFM"){      // MFM if block and switch case structure starts here
              switch(INST_N){      
                case 1:
                fr_mm1 = aip_mm1 - ain_mm1; // Value still in 10-bit resolution, needs to be converted to decimal format (multiply by resolution and send value to LabVIEW)
                Serial.print(fr_mm1);
                break;

                case 2:
                fr_mm2 = aip_mm2 - ain_mm2; // Value still in 10-bit resolution, needs to be converted to decimal format (multiply by resolution and send value to LabVIEW)
                Serial.print(fr_mm2);
                break;  
                }      
              }             */

            //Back Pressure Regulators and Pressure Transducers
            if (INST_Code == "BPR"){        // BPR if block starts here
                p_bpr1 = aip_bpr1 - ain_bpr1; // Value still in 10-bit resolution, needs to be converted to decimal format (multiply by resolution and send value to LabVIEW) 
                Serial.print(p_bpr1 * factor_10_bit);
                Serial.println(";");
              }      
        // Analog Input Voltage Reading Sub-Sketch for onboard pins numbered from A0 to A15 ends
        //////////////////////////////////////////////////////////////////////////////////////////////////////      

        // Analog Input Voltage Reading Sub-Sketch for I2C module channels start
          // There are 3 Analog to Digital Convertor Adafruit ADS1115 connected in series
          /* The three boards are with the address 0x48 (NDIR on pins A0-A1, Pins A2-A3 are open), 
          0x49 (all 4 pins are open) and 0x4A (PT-1 on pins A0-A1 and PT-2 on pins A2-A3)*/
          
            if (INST_Code == "ADC"){                  // ADC if block and switch case starts here
              tcaselect(1);
              switch(INST_N){
                  case 11:                          // MFM-1
                  ads1115_1a = ads1115_1.readADC_Differential_0_1();
                  Serial.print(ads1115_1a*factor_16_bit);
                  Serial.println(";");
                  break;

                  case 12:                          // MFM-2
                  ads1115_1b = ads1115_1.readADC_Differential_2_3();
                  Serial.print(ads1115_1b*factor_16_bit);
                  Serial.println(";");
                  break;
            
                  case 21:                          // PT-1
                  ads1115_2a = ads1115_2.readADC_Differential_0_1();
                  Serial.print(ads1115_2a*factor_16_bit);      
                  Serial.println(";");                  
                  break;

                  case 22:                          // PT-2
                  ads1115_2b = ads1115_2.readADC_Differential_2_3();
                  Serial.print(ads1115_2b*factor_16_bit);
                  Serial.println(";");
                  break;               

                  case 31:                          // NDIR
                  ads1115_3a = ads1115_3.readADC_Differential_0_1();
                  Serial.print(ads1115_3a*factor_16_bit);
                  Serial.println(";");
                  break;
                  
                  case 32:
                  ads1115_3b = ads1115_3.readADC_Differential_2_3();
                  Serial.print(ads1115_3b*factor_16_bit);
                  Serial.println(";");
                  break;
              }
            }
          // Analog Input Voltage Reading Sub-Sketch for I2C module channels end
          //////////////////////////////////////////////////////////////////////////////////////////////////////   
            
          //////////////////////////////////////////////////////////////////////////////////////////////////////
          // Thermocouple data logger module sub-sketch for continuous operation
            if (INST_Code == "_TC"){                  // Thermocouple if block and switch case starts here
            switch(INST_N){
            // Thermocouple TC-1
              case 1:
                Serial.print(tc1.readThermocoupleTemperature());
                Serial.println(";");
                break;

            // Thermocouple TC-2
              case 2:
                Serial.print(tc2.readThermocoupleTemperature());
                Serial.println(";");
                break;          

            // Thermocouple TC-3
              case 3:  
                Serial.print(tc3.readCelsius());
                Serial.println(";");
                break;

            // Thermocouple TC-4
              case 4:
                Serial.print(tc4.readCelsius());
                Serial.println(";");
                break;

            // Thermocouple TC-5
              case 5:
                Serial.print(tc5.readThermocoupleTemperature());
                Serial.println(";");
                break;        
            
            // Thermocouple TC-6
              case 6:
                Serial.print(tc6.readCelsius());
                Serial.println(";");
                break;

            // Thermocouple TC-7
              case 7:
                Serial.print(tc7.readCelsius());
                Serial.println(";");
                break;
              }
            }
      }                
            // Thermocouple data logger module sub-sketch for continuous operation finished
          //////////////////////////////////////////////////////////////////////////////////////////////////////  
          /////////////////////////////////////////////////////////////////////////////////////////////

          // All read operations end here. Hereon, in the code, only write operations are created.

  /////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  
    else if (CMD == "SV"){                // The SV if block starts here
        SSP = inString.substring(8);
        //Serial.print(SSP);
        SetValue = SSP.toFloat();
        float SP_12 = SetValue/factor_12_bit;            // Convert decimnal to 12-bit base (Resolution = 0.001122 V/bit)
        //Serial.print("Command Passed the variable");
       // Serial.print(INST_N);
          if (INST_Code == "DAC"){                  // DAC if block and switch case structure starts here
            switch(INST_N){
            case 11:
            tcaselect(2);
            mcp4728_1.setChannelValue(MCP4728_CHANNEL_A,sp_fr_m1); // MCP4728-1 Channel-A controls setpoint for MFC-1
            break;

            case 12:
            tcaselect(2);
            sp_fr_m2 = round(SP_12);
            mcp4728_1.setChannelValue(MCP4728_CHANNEL_B,sp_fr_m2); // MCP4728-1 Channel-A controls setpoint for MFC-2
            break;

            case 13:
            tcaselect(2);
            sp_fr_m3 = round(SP_12);
            mcp4728_1.setChannelValue(MCP4728_CHANNEL_C,sp_fr_m3); // MCP4728-1 Channel-A controls setpoint for MFC-3
            break;

            case 14:
            tcaselect(2);
            sp_fr_m4 = round(SP_12);    
            mcp4728_1.setChannelValue(MCP4728_CHANNEL_D,sp_fr_m4); // MCP4728-1 Channel-A controls setpoint for MFC-4
            break;
            
            case 21:
//            tcaselect(3);
//            sp_fr_m5 = round(SP_12);
//            mcp4728_2.setChannelValue(MCP4728_CHANNEL_A,sp_fr_m5); // MCP4728-1 Channel-A controls setpoint for MFC-5
//            break;                    

            tcaselect(4);
            sp_fr_m5 = round(SP_12);
            mcp4725_1.setVoltage(sp_fr_m5,false); // MCP4728-1 Channel-A controls setpoint for MFC-5
            Serial.print(sp_fr_m5);
            Serial.print(";");
            break;
            
            case 22:
//            tcaselect(3);
//            sp_p_bpr1 = round(SP_12);
//            mcp4728_1.setChannelValue(MCP4728_CHANNEL_B,sp_p_bpr1); // MCP4728-1 Channel-A controls setpoint for BPR-1
//            break;


            tcaselect(5);
            sp_p_bpr1 = round(SP_12);
            mcp4725_2.setVoltage(0,false); // MCP4728-1 Channel-A controls setpoint for MFC-5
            break;  
            }
           // mcp4728_2.setChannelValue(MCP4728_CHANNEL_C, 0);
           // mcp4728_2.setChannelValue(MCP4728_CHANNEL_D, 0);
          }
  //////////////////////////////////////////////////////////////////////////////////////////////////////
        // The PID code will be implemented in LabVIEW NXG
        // However, commands have been added for shutdown of Electromechanical Relays by Arduino itself if temperature crosses 70 C 
        // TC-4 (direct in contact with HP ammonia before the MFC) and TC-5 (MFC surface) has been used for this application
        if (INST_Code == "RLY"){                        // The if Relay block and switch case starts here
        RLY_POS = inString.substring(6);
        // Serial.print(RLY_POS);
          switch(INST_N){          
            case 1:
            if(RLY_POS == "01ON"){digitalWrite(Relay1,LOW);}
            else {digitalWrite(Relay1,HIGH);}
            if(tc4.readCelsius() > 70) {digitalWrite(Relay1,HIGH);}
            break;

            case 2:           
            if(RLY_POS == "02ON"){digitalWrite(Relay2,LOW);}
            else {digitalWrite(Relay2,HIGH);}
            if(tc4.readCelsius() > 70) {digitalWrite(Relay2,HIGH);}
            break;            

            case 3:        
            if(RLY_POS == "03ON"){digitalWrite(Relay3,LOW);}
            else {digitalWrite(Relay3,HIGH);}            
            if(tc5.readThermocoupleTemperature() > 70) {digitalWrite(Relay1,HIGH);}
            break;

            case 4:        
            if(RLY_POS == "04ON"){digitalWrite(Relay4,LOW);}
            else {digitalWrite(Relay4,HIGH);
            if(tc5.readThermocoupleTemperature() > 70) {digitalWrite(Relay4,HIGH);}
            break;
            }
          }
        }
      }
  // 4-channel realy sub-sketch for continuous operation
  //////////////////////////////////////////////////////////////////////////////////////////////////////
    }

}

/* List of Valid Serial Commands */
//-------------------------------------------------------------------------------------------------------------------------
// Command         = Instrument Code = Parameter                 = Instrument Name -> Comments, if any
//-------------------------------------------------------------------------------------------------------------------------
//All the read commands are listed in the table bellow---------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------
// :PVMFC01;       = MFC-1           = Mass Flow Rate            = Mass Flow Controller
// :PVMFC02;       = MFC-2           = Mass Flow Rate            = Mass Flow Controller
// :PVMFC03;       = MFC-3           = Mass Flow Rate            = Mass Flow Controller
// :PVMFC04;       = MFC-4           = Mass Flow Rate            = Mass Flow Controller
// :PVMFC05;       = MFC-5           = Mass Flow Rate            = Mass Flow Controller
// :PVBPR01;       = BPR-1           = Pressure                  = Back Pressure Regulator
// :PVADC11;       = MFM-1           = Mass Flow Rate            = Mass Flow Meter -> @Analog Input to ADS1115
// :PVADC12;       = MFM-2           = Mass Flow Rate            = Mass Flow Meter -> @Analog Input to ADS1115
// :PVADC21;       = PT-1            = Pressure                  = Pressure Transducer -> @Analog Input to ADS1115
// :PVADC22;       = PT-2            = Pressure                  = Pressure Transducer -> @Analog Input to ADS1115
// :PVADC31;       = NDIR            = Ammonia Concentration     = NDIR Ammonia Detector -> @Analog Input to ADS1115
// :PVADC32;       = ------------------------------------Channel Open------------------------------------
// :PV_TC01;       = TC-1            = Temperature               = Thermocouple, K-type Probe
// :PV_TC02;       = TC-2            = Temperature               = Thermocouple, K-type Probe
// :PV_TC03;       = TC-3            = Temperature               = Thermocouple, K-type Probe
// :PV_TC04;       = TC-4            = Temperature               = Thermocouple, K-type Probe
// :PV_TC05;       = TC-5            = Temperature               = Thermocouple, J-type Surface Adhesive
// :PV_TC06;       = TC-6            = Temperature               = Thermocouple, K-type Probe
// :PV_TC07;       = TC-7            = Temperature               = Thermocouple, K-type Probe
//-------------------------------------------------------------------------------------------------------------------------
//All the write commands are listed in the table bellow--------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------
// :SVDAC11nn.nn;  = MFC-1           = Mass Flow Rate            = Mass Flow Controller -> @Analog Output from MCP4728
// :SVDAC12nn.nn;  = MFC-2           = Mass Flow Rate            = Mass Flow Controller -> @Analog Output from MCP4728
// :SVDAC13nn.nn;  = MFC-3           = Mass Flow Rate            = Mass Flow Controller -> @Analog Output from MCP4728
// :SVDAC14nn.nn;  = MFC-4           = Mass Flow Rate            = Mass Flow Controller -> @Analog Output from MCP4728
// :SVDAC21nn.nn;  = MFC-5           = Mass Flow Rate            = Mass Flow Controller -> @Analog Output from MCP4728
// :SVDAC22nn.nn;  = BPR-1           = Pressure                  = Back Pressure Regulator -> @Analog Output from MCP4728
// :SVDAC23nn.nn;  = ------------------------------------Analog Output Channel Open----------------------
// :SVDAC24nn.nn;  = ------------------------------------Analog Output Channel Open----------------------
// :SVRLY01ON;     = Relay-1         = On/Off State              = Electromechanical Relay -> For Off state, :SVRYLY01OFF;
// :SVRLY02ON;     = Relay-2         = On/Off State              = Electromechanical Relay -> For Off state, :SVRYLY02OFF;
// :SVRLY03ON;     = Relay-3         = On/Off State              = Electromechanical Relay -> For Off state, :SVRYLY03OFF;
// :SVRLY04ON;     = Relay-4         = On/Off State              = Electromechanical Relay -> For Off state, :SVRYLY04OFF;
//-------------------------------------------------------------------------------------------------------------------------
