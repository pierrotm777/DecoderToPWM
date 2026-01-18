/* Ce code ne fonctionne qu'avec un Arduino Uno,Pro Mini ou Pro Micro */

/* L'option JetiEx ne fonctionne qu'avec un Pro Micro */
/* L'option Nv.Failsafe ne fonctionne qu'avec un Pro Mini */

/* Décodeur pour commander 1 servos sur un canal choisi à partir d'un signal type:
 - PPM basé sur les librairies RC Navy https://github.com/RC-Navy/DigisparkArduinoIntegration/tree/master/libraries/DigisparkTinyCppmReader
 - SBUS basé sur les librairies RC Navy https://github.com/RC-Navy/DigisparkArduinoIntegration/tree/master/libraries/RcBusRx
    Un inverseur du signal est nécessaire (http://www.ernstc.dk/arduino/sbus.html)
 - IBUS basé sur la librairie Rc Navy  https://github.com/RC-Navy/DigisparkArduinoIntegration/tree/master/libraries/RcBusRx
 - SRXL basé sur les librairies RC Navy https://github.com/RC-Navy/DigisparkArduinoIntegration/tree/master/libraries/RcBusRx
 - SUMD basé sur l'exemple Rc Navy https://github.com/RC-Navy/DigisparkArduinoIntegration/tree/master/libraries/RcBusRx
 - JETIEX basé sur la librairie Rc Navy https://github.com/RC-Navy/DigisparkArduinoIntegration/tree/master/libraries/RcBusRx
 - CRSF basé sur la librairie Rc Navy https://github.com/RC-Navy/DigisparkArduinoIntegration/tree/master/libraries/RcBusRx
*/
 

#include <Rcul.h>
#include <TinyPinChange.h>
#include <SoftRcPulseOut.h>
#include <TinyCppmReader.h>// configurer le le pour atmega328
#include <EEPROM.h>
#include <RcBusRx.h>
#include <Streaming.h>
#include <SerialCommand.h>

float VERSION_DECODER = 1.0;


uint32_t LedStartMs=millis();
boolean  LedState=LOW;
unsigned long startedWaiting = millis();
unsigned long started1s = millis();
bool InputSignalExist = false;

static bool consoleMode = false;
static bool dbgRun = true;

#define LED_SIGNAL_FOUND      250
#define LED_SIGNAL_NOTFOUND   1000
#define LED    				        17 // declare LED in pin 17
#define FAILSAFE_BUTTON       A3 // button on A3

#define PRINT_BUF_SIZE      100
static char PrintBuf[PRINT_BUF_SIZE + 1];
#define PRINTF(fmt, ...)    do{if(Serial){snprintf_P(PrintBuf, PRINT_BUF_SIZE, PSTR(fmt) ,##__VA_ARGS__);Serial.print(PrintBuf);}}while(0)
//#define PRINT_P(FlashStr)   do{if(Serial){Serial.print(FlashStr);}}while(0)

enum {INPUT_MODE_PWM = 0, INPUT_MODE_CPPM, INPUT_MODE_SBUS, INPUT_MODE_IBUS, INPUT_MODE_SUMD, INPUT_MODE_JETI, INPUT_MODE_SRXL, INPUT_MODE_SRXL2, INPUT_MODE_CRSF, INPUT_MODE_NB};
const char * INPUT_MODE_STR[]   = {"PWM", "CPPM", "SBUS", "IBUS", "SUMD", "JETI", "SRXL", "SRXL2", "CRSF"};

typedef struct{
  uint16_t  Id;
  uint8_t   InputType;
  uint8_t   Channel;
} NvParamSt_t;

NvParamSt_t Nv;

SerialCommand sCmd;

/*
 Hardware Wiring:
 ==============
                            
 .--------.                                  
 |        |
 |        |                          
 |    GND |------- GND         
 |        |                  
       RX |------- Rc Input
 |        |                        
 |     BP |------- Failsafe button (nécessite trop de mémoire).
 '--------'                                 
                                   
  Pro Micro                                
                    
*/

TinyCppmReader TinyCppmReader; 
SoftRcPulseOut PwmOutput;

void setup()
{
  
  pinMode(LED, OUTPUT);             // set LED as an output
  pinMode(FAILSAFE_BUTTON, INPUT_PULLUP);
  
  Serial.begin(115200);

  // Timeout USB Serial (ne bloque pas sans USB)
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 1500)) {
    delay(1);
  }

  if (Serial) {
    Serial.print("\r\nDecoder to PWM v");Serial.println(VERSION_DECODER);
  }

  EEPROM.get(0,Nv);
  delay(250);
  if (Nv.Id != 0x1234)
  {
    Nv.Id = 0x1234;
    Nv.InputType = INPUT_MODE_SBUS;
    Nv.Channel = 1;
    EEPROM.put(0,Nv);
  }

  switch (Nv.InputType)
  {
    case INPUT_MODE_CPPM:
      blinkNTime(1,125,250);
      Serial << F("PPM in use") << endl;
      TinyCppmReader.attach(0); // Attach TinyPpmReader to Rx pin 
      break;
    case INPUT_MODE_SBUS:
      blinkNTime(2,125,250);
      Serial << F("SBUS in use") << endl;
      Serial.flush();delay(500);
      Serial.begin(SBUS_RX_SERIAL_CFG);
      RcBusRx.serialAttach(&Serial);        
      RcBusRx.setProto(RC_BUS_RX_SBUS);
      break;
    case INPUT_MODE_IBUS:
      blinkNTime(3,125,250);
      Serial << F("IBUS in use") << endl;
      Serial.flush();
      Serial.begin(IBUS_RX_SERIAL_CFG);
      RcBusRx.serialAttach(&Serial);        
      RcBusRx.setProto(RC_BUS_RX_IBUS);
      break;
    case INPUT_MODE_SUMD:
      blinkNTime(4,125,250);
      Serial << F("SUMD Nv.InputType in use") << endl;
      Serial.flush();
      Serial.begin(SUMD_RX_SERIAL_CFG);
      RcBusRx.serialAttach(&Serial);        
      RcBusRx.setProto(RC_BUS_RX_SUMD);
      break;
    case INPUT_MODE_JETI:
      blinkNTime(5,125,250);
      Serial << F("JETIEx in use") << endl;
      Serial.flush();
      Serial.begin(JETI_RX_SERIAL_CFG);
      RcBusRx.serialAttach(&Serial);        
      RcBusRx.setProto(RC_BUS_RX_JETI);
      break;
    case INPUT_MODE_SRXL:
      blinkNTime(6,125,250);
      Serial << F("SRLX in use") << endl;
      Serial.flush(); // wait for last transmitted data to be sent
      Serial.begin(SRXL_RX_SERIAL_CFG);
      RcBusRx.serialAttach(&Serial);        
      RcBusRx.setProto(RC_BUS_RX_SRXL);
      break;
    case INPUT_MODE_SRXL2:
      Serial << F("SRXL2 in use, not avaliable") << endl;
      break;
    case INPUT_MODE_CRSF:
      blinkNTime(8,125,250);
      Serial << F("CRSF in use") << endl;
      Serial.flush(); // wait for last transmitted data to be sent
      Serial.begin(CRSF_RX_SERIAL_CFG);
      RcBusRx.serialAttach(&Serial);        
      RcBusRx.setProto(RC_BUS_RX_CRSF);
      break;      
  }


  PwmOutput.attach(2);

}//setup

void loop()
{
    // console
  serialService();
  

  if (Nv.InputType == 1)//PPM
  {
    if (TinyCppmReader.isSynchro())
    {
      InputSignalExist = true;
      //Idx=TinyPpmReader.width_us(1);Serial.print(F("Ch1"));Serial.print(F("="));Serial.print(Idx);Serial.println(F(" us"));
      PwmOutput.write_us(TinyCppmReader.width_us(Nv.Channel));
      SoftRcPulseOut::refresh(1);
        
    }
    else
    {
      InputSignalExist = false;
    }  
  }//PPM
  
  if (Nv.InputType > 1)
  { 

    RcBusRx.process(); /* Don't forget to call the SBusRx.process()! */
    if(RcBusRx.isSynchro()) /* One SBUS frame just arrived */
    {
      InputSignalExist = true;
      PwmOutput.write_us(RcBusRx.width_us(Nv.Channel));
      SoftRcPulseOut::refresh(1);      
    }
    else
    {
      InputSignalExist = false;
    }

  }


  if(InputSignalExist == true)
  {
    // Blink each 250ms if IBUS found on Rx pin
    if(millis()-LedStartMs>=LED_SIGNAL_FOUND)
    {
      LedState = !LedState;
      digitalWrite(LED, LedState);
      LedStartMs=millis(); // Restart the Chrono for the LED 
    }              
  }
  else
  {
    if (consoleMode == false)
    {
      // Blink each 1s if Rc signal is not found on Rx pin
      if(millis()-LedStartMs>=LED_SIGNAL_NOTFOUND)
      {
        LedState = !LedState;
        digitalWrite(LED, LedState);
        LedStartMs=millis(); // Restart the Chrono for the LED 
      }
    }
    else {// console mode
      digitalWrite(LED, HIGH);//set led OFF
    }
  }

  
}//loop


void waitMs(unsigned long timetowait)
{
  static unsigned long beginTime = 0;
  static unsigned long lastTime = 0;
  beginTime = millis();
  lastTime = beginTime + timetowait;
  do
  {
  }
  while (lastTime > millis());
}


// used for flashing a pin
void blinkNTime(int count, int onInterval, int offInterval)
{
  byte i;
  for (i = 0; i < count; i++) 
  {
    waitMs(offInterval);
    digitalWrite(LED, HIGH);      //     turn on LED//digitalWrite(LED_PIN,HIGH);
    waitMs(onInterval);
    digitalWrite(LED, LOW);      //     turn on LED//digitalWrite(LED_PIN,LOW);  
  }
}

