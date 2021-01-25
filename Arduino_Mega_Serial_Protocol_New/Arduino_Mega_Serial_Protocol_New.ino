#include <max6675.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_MCP4728.h>
#include <SPI.h>
#include <Adafruit_MAX31856.h>
#define DRDY_PIN 13
#include <Adafruit_MCP4725.h>

const int led = 13;    //Pin for resetting or restarting the arduino board
const int TC5DI = 28;  // Adafruit MAX38156 SDI or DI Pin (TC5 Thermocouple)
const int TC5CS = 29;  // Adafruit MAX38156 CS Pin (TC5 Thermocouple)
const int TC5CLK = 30; // Adafruit MAX38156 SCK or CLK Pin (TC5 Thermocouple)
const int TC5DO = 31;  // Adafruit MAX38156 SDO or DO Pin (TC5 Thermocouple)
const int TC2DI = 32;  // Adafruit MAX38156 SDI or DI Pin (TC2 Thermocouple)
const int TC2CS = 33;  // Adafruit MAX38156 CS Pin (TC2 Thermocouple)
const int TC2CLK = 34; // Adafruit MAX38156 SCK or CLK Pin (TC2 Thermocouple)
const int TC2DO = 35;  // Adafruit MAX38156 SDO or DO Pin (TC2 Thermocouple)
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
const int TC1CLK = 52;  // Adafruit MAX38156 SCK or CLK Pin (TC1 Thermocouple)
const int TC1CS = 53;   // Adafruit MAX38156 CS Pin (TC1 Thermocouple)

const int Relay1 = 2;   // Sunfounder 4-Channel Relay Shield Ch#1
const int Relay2 = 3;   // Sunfounder 4-Channel Relay Shield Ch#2
const int Relay3 = 4;   // Sunfounder 4-Channel Relay Shield Ch#3
const int Relay4 = 5;   // Sunfounder 4-Channel Relay Shield Ch#4

String serial_input_string;
String CMD;
String INST_Code;
String INST_SN;
int INST_N;
int len;
String SetPoint;
int flow_signal_mfc1;
int input_mfc_1;
int flow_signal_mfc2;
int input_mfc_2;
int flow_signal_mfc3;
int input_mfc_3;
int flow_signal_mfc4;
int input_mfc_4;
int flow_signal_mfc5;
int input_mfc_5;
int pressure_signal_bpr1;
int input_bpr_1;
int flow_signal_mfm1;
int input_mfm_1;
int flow_signal_mfm2;
int input_mfm_2;
int pressure_signal_pt1;
int input_pt_1;
int pressure_signal_pt2;
int input_pt_2;
int concentration_signal_ndir;
int input_ndir;
String SSP;
String RLY_POS;
int t1, t2, t3, t4, t5, t6, t7;

Adafruit_MAX31856 tc1 = Adafruit_MAX31856(TC1CS, TC1DI, TC1DO, TC1CLK);
Adafruit_MAX31856 tc2 = Adafruit_MAX31856(TC2CS, TC2DI, TC2DO, TC2CLK);
MAX6675 tc3 = MAX6675(TC3CLK, TC3CS, TC3DO);
MAX6675 tc4 = MAX6675(TC4CLK, TC4CS, TC4DO);
Adafruit_MAX31856 tc5 = Adafruit_MAX31856(TC5CS, TC5DI, TC5DO, TC5CLK);
MAX6675 tc6 = MAX6675(TC6CLK, TC6CS, TC6DO);
MAX6675 tc7 = MAX6675(TC7CLK, TC7CS, TC7DO);

Adafruit_ADS1115 ads1115_1(0x48);
Adafruit_ADS1115 ads1115_2(0x49);
//Adafruit_ADS1115 ads1115_3(0x4A);

int16_t ads1115_1a, ads1115_1b;
int16_t ads1115_2a, ads1115_2b;
int16_t ads1115_3a, ads1115_3b;

void setup(void) {
  Serial.begin(9600);
  tc1.begin();
  tc1.setThermocoupleType(MAX31856_TCTYPE_K);
  tc2.begin();
  tc2.setThermocoupleType(MAX31856_TCTYPE_J);
  tc5.begin();
  tc5.setThermocoupleType(MAX31856_TCTYPE_J);
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
  pinMode(A14, INPUT);
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  pinMode(Relay3, OUTPUT);
  pinMode(Relay4, OUTPUT);
  digitalWrite(Relay1, HIGH);
  digitalWrite(Relay2, HIGH);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay4, HIGH);
  ads1115_1.begin();
  ads1115_2.begin();
  //ads1115_3.begin();
}

void loop(void) 
{ 
  if (Serial.available() > 0){  
    serial_input_string = (Serial.readString());
    CMD = (String)(serial_input_string.substring(1,3));
    INST_Code = (String)(serial_input_string.substring(3,6));
    INST_SN = (String)(serial_input_string.substring(6,8));
    INST_N = (int)((INST_SN.toInt()));
    len = (int)(String(INST_N).length());
    
    if (len == 8){
      SetPoint = (String)("serial_input_string.substring(9)");
    }
    
    if (CMD == "PV"){
      if (INST_Code == "MFC"){

        switch(INST_N){
          case 1:
          flow_signal_mfc1 = (int)(analogRead(A0) - analogRead(A1));
          input_mfc_1 = (map(flow_signal_mfc1,0, 1023, 0 , 5000));
          Serial.println(input_mfc_1);
          break;

          case 2:
          flow_signal_mfc2 = (int)(analogRead(A2) - analogRead(A3));
          input_mfc_2 = (map(flow_signal_mfc2,0, 1023, 0 , 5000));
          Serial.println(input_mfc_2);
          break;

          case 3:
          flow_signal_mfc3 = (int)(analogRead(A4) - analogRead(A5));
          input_mfc_3 = (map(flow_signal_mfc3,0, 1023, 0 , 5000));
          Serial.println(input_mfc_3);
          break;

          case 4:
          flow_signal_mfc4 = (int)(analogRead(A6) - analogRead(A7));
          input_mfc_4 = (map(flow_signal_mfc4,0, 1023, 0 , 5000));
          Serial.println(input_mfc_4);
          break;

          case 5:
          flow_signal_mfc5 = (int)(analogRead(A8) - analogRead(A9));
          input_mfc_5 = (map(flow_signal_mfc5,0, 1023, 0 , 5000));
          Serial.println(input_mfc_5);
          break;
        }
      }
      
      else if (INST_Code == "BPR"){

        switch(INST_N){
          case 1:
          pressure_signal_bpr1 = (int)(analogRead(A14));
          input_bpr_1 = (map(pressure_signal_bpr1,0, 1023, 0 , 5000));
          Serial.println(input_bpr_1);
          break;
        }
      } 
      
      else if (INST_Code == "ADC") {
        
        switch(INST_N){
          case 11:
          flow_signal_mfm1 = (int)("ads1115_1.readADC_Differential_0_1()");
          input_mfm_1 = (map(flow_signal_mfm1,0, 65535, 0 , 5000));
          Serial.println(input_mfm_1);
          break;

          case 12:
          flow_signal_mfm2 = (int)("ads1115_1.readADC_Differential_2_3()");
          input_mfm_2 = (map(concentration_signal_ndir,0, 65535, 1000 , 5000));
          Serial.println(input_mfm_2);
          break;

          case 21:
          pressure_signal_pt1 = (int)("ads1115_2.readADC_Differential_0_1()");
          input_pt_1 = (map(pressure_signal_pt1,0, 65535, 0 , 5000));
          Serial.println(input_pt_1);
          break;

          case 22:
          pressure_signal_pt2 = (int)("ads1115_2.readADC_SingleEnded(2)");
          input_pt_2 = (map(pressure_signal_pt2,0, 65535, 0 , 5000));
          Serial.println(input_pt_2);
          break;

          case 31:
          concentration_signal_ndir = (int)("ads1115_3.readADC_Differential_0_1()");
          input_ndir = (map(concentration_signal_ndir,0, 65535, 0 , 5000));
          Serial.println(input_ndir);
          break;
        }
      }
       
      else if (INST_Code == "PTC") {
          switch(INST_N){
            case 1:
              t1 = tc1.readThermocoupleTemperature();
              Serial.println(t1*1000);
              break;
              
            case 2:
              t2 = tc2.readThermocoupleTemperature();
              Serial.println(t2*1000);
              break;          

            case 3:  
              t3 = tc3.readCelsius();
              Serial.println(t3*1000);
              break;

            case 4:
              t4 = tc4.readCelsius();
              Serial.println(t4*1000);
              break;

            case 5:
              t5 = tc5.readThermocoupleTemperature();
              Serial.println(t5*1000);
              break;        
            
            case 6:
              t6 = tc6.readCelsius();
              Serial.println(t6*1000);
              break;

            case 7:
              t7 = tc7.readCelsius();
              Serial.println(t7*1000);
              break;
           }
         }
      }
  
      else if (CMD == "SV"){
        SSP = serial_input_string.substring(8);
        SetPoint = SSP.toInt();
        if (INST_Code == "RLY"){
        RLY_POS = serial_input_string.substring(6);
        
        switch(INST_N){                     
          case 1:
          if(RLY_POS == "01ON"){digitalWrite(Relay1,LOW);}
          if(RLY_POS == "01OFF"){digitalWrite(Relay1,HIGH);}
          if(tc4.readCelsius() > 70) {digitalWrite(Relay1,HIGH);}
          break;

          case 2:           
          if(RLY_POS == "02ON"){digitalWrite(Relay2,LOW);}
          if(RLY_POS == "02OFF"){digitalWrite(Relay2,HIGH);}
          if(tc4.readCelsius() > 70) {digitalWrite(Relay2,HIGH);}
          break;            

          case 3:        
          if(RLY_POS == "03ON"){digitalWrite(Relay3,LOW);}
          if(RLY_POS == "03OFF"){digitalWrite(Relay3,HIGH);}            
          if(tc5.readThermocoupleTemperature() > 70) {digitalWrite(Relay1,HIGH);}
          break;

          case 4:        
          if(RLY_POS == "04ON"){digitalWrite(Relay4,LOW);}
          if(RLY_POS == "04OFF"){digitalWrite(Relay3,HIGH);}
          if(tc5.readThermocoupleTemperature() > 70) {digitalWrite(Relay4,HIGH);}
          break;
          }
        }
      }

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
// :PVADC32;       = ------------------------------------Channel Open------------------------------------------------------
// :PV_TC01;       = TC-1            = Temperature               = Thermocouple, K-type Probe
// :PV_TC02;       = TC-2            = Temperature               = Thermocouple, K-type Probe
// :PV_TC03;       = TC-3            = Temperature               = Thermocouple, K-type Probe
// :PV_TC04;       = TC-4            = Temperature               = Thermocouple, K-type Probe
// :PV_TC05;       = TC-5            = Temperature               = Thermocouple, J-type Surface Adhesive
// :PV_TC06;       = TC-6            = Temperature               = Thermocouple, K-type Probe
// :PV_TC07;       = TC-7            = Temperature               = Thermocouple, K-type Probe
//-------------------------------------------------------------------------------------------------------------------------
//All the print commands are listed in the table bellow--------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------
// :SVDAC11nn.nn;  = MFC-1           = Mass Flow Rate            = Mass Flow Controller -> @Analog Output from MCP4728
// :SVDAC12nn.nn;  = MFC-2           = Mass Flow Rate            = Mass Flow Controller -> @Analog Output from MCP4728
// :SVDAC13nn.nn;  = MFC-3           = Mass Flow Rate            = Mass Flow Controller -> @Analog Output from MCP4728
// :SVDAC14nn.nn;  = MFC-4           = Mass Flow Rate            = Mass Flow Controller -> @Analog Output from MCP4728
// :SVDAC21nn.nn;  = MFC-5           = Mass Flow Rate            = Mass Flow Controller -> @Analog Output from MCP4725
// :SVDAC22nn.nn;  = BPR-1           = Pressure                  = Back Pressure Regulator -> @Analog Output from MCP4725
// :SVDAC23nn.nn;  = ------------------------------------Analog Output Channel Open----------------------
// :SVDAC24nn.nn;  = ------------------------------------Analog Output Channel Open----------------------
// :SVRLY01ON;     = Relay-1         = On/Off State              = Electromechanical Relay -> For Off state, :SVRYLY01OFF;
// :SVRLY02ON;     = Relay-2         = On/Off State              = Electromechanical Relay -> For Off state, :SVRYLY02OFF;
// :SVRLY03ON;     = Relay-3         = On/Off State              = Electromechanical Relay -> For Off state, :SVRYLY03OFF;
// :SVRLY04ON;     = Relay-4         = On/Off State              = Electromechanical Relay -> For Off state, :SVRYLY04OFF;
// :SVRLY00OFF;    = Relat-1,2,3,4   = Off State                 = Commands to set all relays off 
//-------------------------------------------------------------------------------------------------------------------------
//Arduino board soft "Reset" function--------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------
// :RST;
