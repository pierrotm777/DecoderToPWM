/* Ce code ne fonctionne qu'avec un Arduino Uno,Pro Mini ou Pro Micro */

/* L'option JetiEx ne fonctionne qu'avec un Pro Micro */
/* L'option Failsafe ne fonctionne qu'avec un Pro Mini */

/* Décodeur pour commander 1 servos sur un canal choisi à partir d'un signal type:
 - PPM basé sur les librairies RC Navy https://github.com/RC-Navy/DigisparkArduinoIntegration/tree/master/libraries/DigisparkTinyCppmReader
 - SBUS basé sur les librairies RC Navy https://github.com/RC-Navy/DigisparkArduinoIntegration/tree/master/libraries/RcBusRx
    Un inverseur du signal est nécessaire (http://www.ernstc.dk/arduino/sbus.html)
 - IBUS basé sur la librairie Rc Navy  https://github.com/RC-Navy/DigisparkArduinoIntegration/tree/master/libraries/RcBusRx
 - SRXL basé sur les librairies RC Navy https://github.com/RC-Navy/DigisparkArduinoIntegration/tree/master/libraries/RcBusRx
 - SUMD basé sur l'exemple Rc Navy https://github.com/RC-Navy/DigisparkArduinoIntegration/tree/master/libraries/RcBusRx
 - DSMX basé sur la librairie https://github.com/Quarduino/SpektrumSatellite
 - JETIEX basé sur la librairie Rc Navy https://github.com/RC-Navy/DigisparkArduinoIntegration/tree/master/libraries/RcBusRx
 - MULTIWII basé sur la librairie https://github.com/fdivitto/MSP
*/
 

#include <Rcul.h>
#include <TinyPinChange.h>
#include <SoftRcPulseOut.h>
#include <TinyCppmReader.h>// configurer le le pour atmega328
#include <EEPROM.h>
#include <RcBusRx.h>
#include <Streaming.h>

float VERSION_DECODER = 0.6;

//#include <Vcc.h>
//const float VccMin   = 0.0;           // Minimum expected Vcc level, in Volts.
//const float VccMax   = 5.0;           // Maximum expected Vcc level, in Volts.
//const float VccCorrection = 1.0/1.0;  // Measured Vcc by multimeter divided by reported Vcc
//Vcc vcc(VccCorrection);
//bool LowPower = false;

/* Macro function to declare an output pin */
#define out(x)      _out(x)
#define _out(bit,port)  DDR##port |= (1 << bit)
/* Macro function to declare an input pin */
#define in(x)     _in(x)
#define _in(bit,port) DDR##port &= ~(1 << bit)
/* Macro function to set an output pin high */
#define on(x)     _on(x)
#define _on(bit,port) PORT##port |= (1 << bit)
/* Macro function to set an output pin low */
#define off(x)      _off(x)
#define _off(bit,port)  PORT##port &= ~(1 << bit)
/* Macro function to set internal pullup resistor of input pin (same as "on" macro)*/
#define pullup(x)   _on(x)
/* Macro function to get state of input pin */
#define get(x)      _get(x)
#define _get(bit,port)  (PIN##port & (1 << bit))
/* Macro function to toggle an output pin */
#define flip(x)     _flip(x)
#define _flip(bit,port) PORT##port ^= (1 << bit)

uint32_t LedStartMs=millis();
//boolean  LedState=LOW;
unsigned long startedWaiting = millis();
unsigned long started1s = millis();
bool InFailsafeMode = true;
bool InputSignalExist = false;

#define LED_SIGNAL_FOUND      250
#define LED_SIGNAL_NOTFOUND   1000
#define LED    				        5,B // declare LED in PCB5 (D13)
#define FAILSAFE_BUTTON       3,C // button on A3
/*
 Hardware Wiring:
 ==============
                            
 .--------.                                  
 |        |
 |        |                          
 |    GND |------- GND         
 |        |                  
       RX |------- SIGNAL_INPUT_PIN
 |        |                        
 |     BP |------- Failsafe button (nécessite trop de mémoire).
 '--------'                                 
                                   
  Pro Micro                                
                    
*/

//0 PD2 Pro Micro (use also for Serial Configuration)
uint8_t SIGNAL_INPUT_PIN = 0; //PPM,SBUS,IBUS,DSMX,RXL,SUMD and JETIEx input

//uint8_t CHANNEL_NB = 8;     //8 ou 16

TinyCppmReader TinyCppmReader; 

SoftRcPulseOut myservo1;

#include <FlySkyIBus.h>

//#include <SpektrumSattelite.h>
//SpektrumSattelite Dsmx;
#include <DSMRX.h>
DSM2048 Dsmx;

//MultiWII
#include<MSP.h>
MSP msp;


boolean RunConfig = false;
uint8_t canalNb, mode, failsafe;//, nboutput, reverse, pulsetype;


#define LONGUEUR_MSG_MAX   5              /* ex: S1,1500,1500,1000,1000,2,2000,1250,1000,2000,1,0,5 ou S2,1,0,99,2,0,0,0,1000,20000,0,0,4.0 */
#define RETOUR_CHARRIOT    0x0D           /* CR (code ASCII) */
#define PASSAGE_LIGNE      0x0A           /* LF (code ASCII) */
#define BACK_SPACE         0x08
char Message[LONGUEUR_MSG_MAX + 1];

void setup()
{
//  float v = vcc.Read_Volts();
//  if (v <=4) LowPower = true;
  
  out(LED);             // set LED as an output
  in(FAILSAFE_BUTTON);
  pullup(FAILSAFE_BUTTON);
  
  Serial.begin(115200); 
  while (!Serial);// wait for serial port to connect.
  Serial << F("Version:")<< EEPROMReadFloat(500) << endl;
  if (EEPROMReadFloat(500) != VERSION_DECODER)
  {
    //waitMs(500);
    Serial <<F("Def ault mode set: PPM,8 outputs,9-16,failsafe off")<< endl;
    EEPROM.write(0,0);// PWM canal number (1 to 16)
    EEPROM.write(1,1);// PPM
    //EEPROM.write(2,0);// 8 servos
    //EEPROM.write(3,0);// define channel 9 to 16 on D2-D9
    //EEPROM.write(5,0);// failsafe Off
    EEPROMWriteFloat(500,VERSION_DECODER);
  }
  
  if (RunConfig == false)
  {
    String sdata="";
    Serial << F("Wait Return");
    byte ch;
    while(millis() - startedWaiting <= 3000) //waiting 3s return key
    { 
      /* Check 1s */
      if(millis()-started1s>=1000)
      {
        Serial << F(".");started1s=millis();
      }
      if(Serial.available() > 0)
      {
        ch = Serial.read();
        sdata += (char)ch;
        if (ch=='\r')
        {
          sdata.trim(); // Process command in sdata.
          sdata = ""; // Clear the string ready for the next command.
          RunConfig = true;
          break;
        }        
      }   
    }
  }

//*********************
//  RunConfig = true;
//*********************

  (RunConfig == true?Serial << endl << endl << F("Configuration mode is actived") << endl:Serial << endl << endl << F("Starting without configuration") << endl);
  
  canalNb = EEPROM.read(0);// PWM canal number (1 to 16)
  mode = EEPROM.read(1);// 1 PPM, 2 SBUS, 3 IBUS ...
  //nboutput = EEPROM.read(2);// 8 or 16 servos
  //reverse = EEPROM.read(3);// define channel 9 to 16 on D2-D9 or D10-A4
  //pulsetype = EEPROM.read(4);//define pulse neg or pos for output ppm
  failsafe = EEPROM.read(5);

  Serial << F("PWM canal ") << canalNb << (" in use") << endl;  
  
  switch (mode)
  {
    case 1:
      blinkNTime(1,125,250);
      Serial << F("PPM mode in use") << endl;
      if (RunConfig == false)
      {
//        if (type==0)
//        {
//          Serial.end();
//          TinyCppmReader.attach(SIGNAL_INPUT_PIN); // Attach TinyPpmReader to SIGNAL_INPUT_PIN pin 
//        }
      }
      break;
    case 2:
      blinkNTime(2,125,250);
      Serial << F("SBUS mode in use") << endl;
      if (RunConfig == false)
      {
        Serial.flush();delay(500);
        Serial.begin(SBUS_RX_SERIAL_CFG);
        RcBusRx.serialAttach(&Serial);        
        RcBusRx.setProto(RC_BUS_RX_SBUS);
      }
      break;
    case 3:
      blinkNTime(3,125,250);
      Serial << F("IBUS mode in use") << endl;
      if (RunConfig == false)
      {
        Serial.flush();
        Serial.begin(IBUS_RX_SERIAL_CFG);
        RcBusRx.serialAttach(&Serial);        
        RcBusRx.setProto(RC_BUS_RX_IBUS);
      }
      break;
    case 4:
      blinkNTime(4,125,250);
      Serial << F("DSMX mode in use") << endl;
      if (RunConfig == false)
      {
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(115200);
      }
      break;
    case 5:
      blinkNTime(5,125,250);
      Serial << F("SRLX mode in use") << endl;
      if (RunConfig == false)
      {
        Serial.flush(); // wait for last transmitted data to be sent
        Serial.begin(SRXL_RX_SERIAL_CFG);
        RcBusRx.serialAttach(&Serial);        
        RcBusRx.setProto(RC_BUS_RX_SRXL);
      }
      break;
    case 6:
      blinkNTime(6,125,250);
      Serial << F("SUMD mode in use") << endl;
      if (RunConfig == false)
      {
        Serial.flush();
        Serial.begin(SUMD_RX_SERIAL_CFG);
        RcBusRx.serialAttach(&Serial);        
        RcBusRx.setProto(RC_BUS_RX_SUMD);
      }
      break;
    case 7:
      blinkNTime(7,125,250);
      Serial << F("JETIEx mode in use") << endl;
      if (RunConfig == false)
      {
        Serial.flush();
        Serial.begin(JETI_RX_SERIAL_CFG);
        RcBusRx.serialAttach(&Serial);        
        RcBusRx.setProto(RC_BUS_RX_JETI);
      }
      break;
    case 8:
      blinkNTime(8,125,250);
      Serial << F("MultiWii mode in use") << endl;
      if (RunConfig == false)
      {
        Serial.flush();
        Serial.begin(115200);
        msp.begin(Serial);
      }
      break;      
  }


  myservo1.attach(2);//PD2

  
  //waitMs(1000);
#if !defined(__AVR_ATmega32U4__)
  //writeFailsafeTest();
#endif
}//setup

void loop()
{
//  if (LowPower == true)// if Vcc < 4v
//  {
//    // Blink each 250ms if PPM found on pin 2
//    blinkNTime(5,LED_SIGNAL_FOUND,LED_SIGNAL_FOUND);
//    waitMs(1000);
//    blinkNTime(1,LED_SIGNAL_FOUND,LED_SIGNAL_FOUND);
//    waitMs(1000);
//  }
  
  if (RunConfig == true)
  {
      handleSerialDecoder();
      
      //h Help
      //q quit
      //1 to 16 set Pwm canal number
      //p set PPM mode
      //s set SBUS mode
      //i set IBUS mode
      //d set DSMX mode
      //m set SRLX mode
      //u set SUMD mode
      //j set JETIEx mode
      //k set MULTIWII mode
      //f set Failsafe values
      
      //e reset EEPROM (command hidden)   
  }

  if ((RunConfig == false) /*&& (LowPower == false)*/)
  { 
      if (mode == 1)//PPM
      {
        if (TinyCppmReader.isSynchro())
        {
          InputSignalExist = true;
          //Idx=TinyPpmReader.width_us(1);Serial.print(F("Ch1"));Serial.print(F("="));Serial.print(Idx);Serial.println(F(" us"));
          myservo1.write_us(TinyCppmReader.width_us(canalNb));
          SoftRcPulseOut::refresh(1);
            
        }
        else
        {
          InputSignalExist = false;
//          if (failsafe == 1)
//            readFailsafeValues();
        }  
      }//PPM
     
      if (mode == 2/*SBUS*/ || mode == 3/*IBUS*/ || mode == 5/*SRXL*/ || mode == 6/*SUMD*/ || mode == 7/*JETI*/)
      { 

        RcBusRx.process(); /* Don't forget to call the SBusRx.process()! */
        if(RcBusRx.isSynchro()) /* One SBUS frame just arrived */
        {
          InputSignalExist = true;
          myservo1.write_us(RcBusRx.width_us(canalNb));
          SoftRcPulseOut::refresh(1);      
        }
        else
        {
          InputSignalExist = false;
        }

      }//mode == 2/*SBUS*/ || mode == 3/*IBUS*/ || mode == 5/*SRXL*/ || mode == 6/*SUMD*/

      if (mode == 4)//DSMX
      {
        if (Dsmx.gotNewFrame()) 
        {
          InputSignalExist = true;
          uint16_t ch[16];
          Dsmx.getChannelValues(ch, 16);
          myservo1.write_us(ch[canalNb]);
          SoftRcPulseOut::refresh(1);
          //Serial.print("Fade count = ");
          //Serial.println(rx.getFadeCount());
        }
        else  if (Dsmx.timedOut(micros())) 
        {
          InputSignalExist = false;
        }
      }//DSMX

      if (mode == 7)//MULTIWII
      {
        msp_rc_t rc;
        if (msp.request(MSP_RC, &rc, sizeof(rc))) {
          
//          uint16_t roll     = rc.channelValue[0];
//          uint16_t pitch    = rc.channelValue[1];
//          uint16_t yaw      = rc.channelValue[2];
//          uint16_t throttle = rc.channelValue[3];
          myservo1.write_us(rc.channelValue[canalNb]);
          SoftRcPulseOut::refresh(1);
        }              
      }
       
    }//type 0


  
  if(InputSignalExist == true)
  {
    // Blink each 250ms if IBUS found on Rx pin
    if(millis()-LedStartMs>=LED_SIGNAL_FOUND)
    {
      flip(LED);
      LedStartMs=millis(); // Restart the Chrono for the LED 
    }              
  }
  else
  {
    // Blink each 1s if IBUS not found on Rx pin
    if(millis()-LedStartMs>=LED_SIGNAL_NOTFOUND)
    {
      flip(LED);
      LedStartMs=millis(); // Restart the Chrono for the LED 
    }            
  }

  
}//loop


void handleSerialDecoder() {
  // we only care about two characters to change the pwm
  if (MsgDisponible() >= 0) 
  {
    String mystr = (String)Message;
    uint8_t myInt = mystr.toInt();
    
    if (mystr == "h")
    {
      Serial << F("h Help") << endl;
      Serial << F("q quit") << endl;
      Serial << F("set canal number (1 to 16)") << endl;
      Serial << F("p set PPM mode") << endl;
      Serial << F("s set SBUS mode") << endl;
      Serial << F("i set IBUS mode") << endl;
      Serial << F("d set DSMX mode") << endl;
      Serial << F("m set SRLX mode") << endl;
      Serial << F("u set SUMD mode") << endl;
      Serial << F("j set JETIEX mode") << endl;
      Serial << F("k set MULTIWII mode") << endl;
      Serial << F("f set Failsafe values") << endl;      
    }
    else if(mystr == "q")
    {
      Serial << F("Exit  configuration mode") << endl;
      delay(200);
      RunConfig = false;
    }
    else if (myInt > 0 && myInt < 17)
    {
      EEPROM.write(0,myInt);
      Serial << F("PWM canal ") << myInt << (" in use") << endl;  
    }
    else if (mystr == "p")
    {
      Serial << F("Set in PPM mode") << endl;
      EEPROM.write(1,1);  
    }
    else if (mystr == "s")
    {
      Serial << F("Set in SBUS mode") << endl;
      EEPROM.write(1,2);  
    }
    else if (mystr == "i")
    {
      Serial << F("Set in IBUS mode") << endl;
      EEPROM.write(1,3);  
    }
    else if (mystr == "d")
    {
      Serial << F("Set in DSMX mode") << endl;
      EEPROM.write(1,4);  
    }
    else if (mystr == "m")
    {
      Serial << F("Set in SRLX mode") << endl;
      EEPROM.write(1,5);  
    }
    else if (mystr == "u")
    {
      Serial << F("Set in SUMD mode") << endl;
      EEPROM.write(1,6);  
    }
    else if (mystr == "j")
    {
      Serial << F("Set in JETIEX mode") << endl;
      EEPROM.write(1,7);  
    }
    else if (mystr == "k")
    {
      Serial << F("Set in MULTIWII mode") << endl;
      EEPROM.write(1,8);  
    }     
    else if (mystr == "f")
    {
      if (EEPROM.read(5) == 0)
      {
        EEPROM.write(5,1);Serial << F("Failsafe mode is On") << endl;
        uint8_t g = 1;
        for (uint8_t f = 0; f <= 30; f +=2)
        {
          Serial << F("Servo") << g << F("=") << EEPROMReadInt(100+f) << endl;
          g +=1;//Serial.print(F("(address"));Serial.print(100+f);Serial.println(F(")"));
        }
        Serial << endl;
      }
      else
      {
        EEPROM.write(5,0);Serial << F("Failsafe mode is Off") << endl;
      }
    }
    else if (mystr == "e")
    {
      Serial << endl;
      for (int i = 0 ; i < EEPROM.length() ; i++) {
        EEPROM.write(i, 0);
      }
      Serial << F("Reset EEPROM Done") << endl << endl;
    }

  }
}

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
    on(LED);      //     turn on LED//digitalWrite(LED_PIN,HIGH);
    waitMs(onInterval);
    off(LED);      //     turn on LED//digitalWrite(LED_PIN,LOW);  
  }
}

void EEPROMWriteInt(int address, int value)
{
  byte two = (value & 0xFF);
  byte one = ((value >> 8) & 0xFF);
  
  EEPROM.update(address, two);
  EEPROM.update(address + 1, one);
}
 
int EEPROMReadInt(int address)
{
  long two = EEPROM.read(address);
  long one = EEPROM.read(address + 1);
 
  return ((two << 0) & 0xFFFFFF) + ((one << 8) & 0xFFFFFFFF);
}

float EEPROMReadFloat(unsigned int addr)
{
  union
  {
    byte b[4];
    float f;
  } data;
  for(int i = 0; i < 4; i++)
  {
    data.b[i] = EEPROM.read(addr+i);
  }
  return data.f;
}
void EEPROMWriteFloat(unsigned int addr, float x)
{
  union
  {
    byte b[4];
    float f;
  } data;
  data.f = x;
  for(int i = 0; i < 4; i++)
  {
    EEPROM.write(addr+i, data.b[i]);
  }
}


#if !defined(__AVR_ATmega32U4__)
void writeFailsafeTest()
{
  //write a value from 0 to 180
  for (uint8_t f = 100; f <= 130; f +=2)
  {
    EEPROMWriteInt(f,10);
  }
}

void readFailsafeValues()
{
    //write a value from 0 to 180
    myservo1.write(EEPROMReadInt(100));//PD2
}
#endif

int8_t MsgDisponible(void)/* merci a LOUSSOUARN Philippe pour ce code */
{
  int8_t Ret = -1;
  char CaractereRecu;
  static uint8_t Idx = 0;

  
  if(Serial.available() > 0)
  {
    CaractereRecu = Serial.read();
    switch(CaractereRecu)
    {
      case RETOUR_CHARRIOT:
      case PASSAGE_LIGNE:
        Message[Idx] = 0;
        Ret = Idx;
        Idx = 0;
        break; 
      case BACK_SPACE: // Gestion touche d'effacement du dernier caractere sur un terminal fonctionnant caractere par caractere (ex: HyperTerminal, TeraTerm, etc...)
        if(Idx) Idx--;
        break;
      default:
        if(Idx < LONGUEUR_MSG_MAX)
        {
            Message[Idx] = CaractereRecu;
            Idx++;
        }
        else Idx = 0; /* Reset index for the next message */
        break;
    }
  }
  return(Ret); 
}
