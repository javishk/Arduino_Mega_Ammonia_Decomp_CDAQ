#include <Adafruit_MCP4725.h>
#include <Adafruit_MCP4728.h>

// Adafruit MCP4728 4-Channel 12-bit I2C Digital to Analog Convertor
Adafruit_MCP4728 mcp4728_1;

// Adafruit MCP4725 1-Channel 12-bit I2C Digital to Analog Convertor
Adafruit_MCP4725 mcp4725_1;
Adafruit_MCP4725 mcp4725_2;

String inString = "";
String CMD = "";
String INST_Code = "";
String INST_SN = "";
String SSP = "";
int len;
int SetValue;
int SetPoint;
int INST_N;
 
void setup() {
  Serial.begin(19200);
  // put your setup code here, to run once:
 mcp4728_1.begin();    //The analog output from this Digital to Analog Convertor with 12-bit resolution sends the setpoint to MFC-1,2,3,4
    mcp4728_1.setChannelValue(MCP4728_CHANNEL_A,0); // MCP4728-1 Channel-A controls setpoint for MFC-1
    mcp4728_1.setChannelValue(MCP4728_CHANNEL_B,0); // MCP4728-1 Channel-A controls setpoint for MFC-2
    mcp4728_1.setChannelValue(MCP4728_CHANNEL_C,0); // MCP4728-1 Channel-A controls setpoint for MFC-3
    mcp4728_1.setChannelValue(MCP4728_CHANNEL_D,0); // MCP4728-1 Channel-A controls setpoint for MFC-4 
 
  mcp4725_1.begin(0x62);
      mcp4725_1.setVoltage(0,false); // MCP4728-1 Channel-A controls setpoint for MFC-5 

  mcp4725_2.begin(0x63);
      mcp4725_2.setVoltage(0,false); // MCP4728-2 Channel-A controls setpoint for BPR-1
}

void loop() {
  // put your main code here, to run repeatedly:
   inString = "";
      CMD = "";
      INST_Code = "";
      INST_SN = "";
      SSP = "";
      
      if(Serial.available()){                             // Arduino will only run the loop if it reads that there is some incoming data from the serial port
      inString = Serial.readString();          // To read the string (and not converted value of the serial commands) until the terminator character for generating the sub-string
   //   Serial.println(inString);                               
      CMD = inString.substring(1,3);                          // Storing the 2nd and 3rd character in variable CMD: PV (Present Value) or SV (Setpoint Value)
   //   Serial.println(CMD);                                    
      INST_Code = inString.substring(3,6);                    // Storing the instrument type or code: MFC (Mass Flow Convertor), MFM (Mass Flow Meter), ADC (Analog to Digital Convertor - Analog Input)
   //   Serial.println(INST_Code);                                                        //BPR (Back Pressure Regulator), DAC (Digital to Analog Convertor - Analog Out), _TC (Thermocouple), RYL (Relay)
      INST_SN = inString.substring(6,8);                      // Storing the serial number of the instrument as String
    //  Serial.println(INST_SN);
      INST_N = INST_SN.toInt();                               // Converting and storing the serial number as integer
     // Serial.println(INST_N);
      len = inString.length();                                // Length of the input string recieved by the serial port without the termination character
      if(len > 8){SSP = inString.substring(9);}               // If more than 9 characters present in string, then store the setpoint which starts from 9th character in variable SSP as String

      if (CMD == "SV"){                // The SV if block starts here
        SSP = inString.substring(8);
        SetValue = (SSP.toInt())*100;
    //    Serial.print(SSP);
        SetPoint = map(SetValue, 0, 5000, 0, 4096);
        
      // Serial.println("Command Passed the variable");
   //    Serial.println(INST_N);
   //    Serial.println(SetValue);
          if (INST_Code == "DAC"){                  // DAC if block and switch case structure starts here
            switch(INST_N){
            case 11:
            mcp4728_1.setChannelValue(MCP4728_CHANNEL_A,SetValue, MCP4728_VREF_VDD); // MCP4728-1 Channel-A controls setpoint for MFC-1
            break;

            case 12:
            mcp4728_1.setChannelValue(MCP4728_CHANNEL_B,SetValue, MCP4728_VREF_VDD); // MCP4728-1 Channel-A controls setpoint for MFC-2
            break;

            case 13:
            mcp4728_1.setChannelValue(MCP4728_CHANNEL_C,SetValue, MCP4728_VREF_VDD); // MCP4728-1 Channel-A controls setpoint for MFC-3
            break;

            case 14:
            mcp4728_1.setChannelValue(MCP4728_CHANNEL_D,SetValue, MCP4728_VREF_VDD); // MCP4728-1 Channel-A controls setpoint for MFC-4
            break;
            
            case 21:
            mcp4725_1.setVoltage(SetValue,false); // MCP4725-1 Channel-A controls setpoint for MFC-5
            break;
            
            case 22:
            mcp4725_2.setVoltage(SetValue,false); // MCP4725-1 Channel-A controls setpoint for BPR-1
            break;  
            }
          }
      }
   }
}
