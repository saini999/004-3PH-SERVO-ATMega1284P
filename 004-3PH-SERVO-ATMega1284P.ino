/*
Project: NAS-004-3PH-SERVO
Description: Automatic Voltage Stablizer Control for Three Phase AC Supply
Project URL: https://github.com/saini999/004-3PH-SERVO-ATMega1284P
Author: Harnoor Saini
Github: https://github.com/saini999
Discord: N00R#2080
Email: noor@noorautomation.in

////////////////////////////////////////////////////
//// THIS WILL NOT WORK WITH ARDUINO UNO/Mega2560 BOARD /////
/////// Due to less I/O Pins on UNO Board //////////
////////////////////////////////////////////////////

MCU: ATMega1284P (44Pin)
MCU Arduino Core: MightyCore by MCUDude //https://github.com/MCUdude/MightyCore
MCU Clock: 8Mhz (Internal Oscillator)

MCU Fuse Bits: 
 FuseLow : 0xE2
 FuseHigh: 0x99
 For BOD 2.7v - FuseExtended: 0xFD
 For BOD 4.3v - FuseExtended: 0xFC //The one i use.

Fuses Can be Set by reading the microcontroller datasheet or from the Website: https://www.engbedded.com/fusecalc/   //Working As of 24Dec2022


WARN: This will only work with Internal Oscillator as Occillator Pins are being used as I/O.

Project Start Date: 24-Dec-2022
Last Update: 25-Dec-2022

Input Voltage: Phase R: PIN PA0
               Phase Y: PIN PA1
               Phase B: PIN PA2

Output Voltage: Phase R: PIN PA3
                Phase Y: PIN PA4
                Phase B: PIN PA5
Current CT:   ALL Phase: PIN PA6

Input AC supply for Frequency: PIN PA3 //Using Same pin as Phase R Output Voltage Pin
                                                                 
Servo Motor: Phase R: Forward: PIN PB0
                      Reverse: PIN PB1
             Phase Y: Forward: PIN PB2
                      Reverse: PIN PB3
             Phase B: Forward: PIN PB4
                      Reverse: PIN PB5

Power Relay / Contactor Relay: PIN PB6                        
                                                             
ok/Menu button: Pin PC4 -----------------------+                                                             
plus/Up button: Pin PC5                        | To Open Setup Press All Buttons 
minus/Down button: Pin PC6                     |       At the Same Time!         
setup/Settings button: NONE -------------------+ 

Display I/O are Defined in setupDisplay() function at the end of the code.

Parameters: IHu/IHv = Input High Voltage
            ILu/ILv = Input Low Voltage
            OHu/OHv = Output High Voltage
            OLu/OLv = Output Low Voltage
            SETu/SETv = Set Voltage
            OuL/OvL = Overload Current
            TOn = Relay/Contactor ON Delay
            TOff = Relay/Contactor CUTOFF Delay
            DIFF = Voltage Difference from Set Voltage

Run Mode Display: NOOR: WaterMark
                  R IP: R Phase Input Voltage
                  R OP: R Phase Output Voltage
                  Y IP: Y Phase Input Voltage
                  Y OP: Y Phase Output Voltage
                  B IP: B Phase Input Voltage
                  B OP: B Phase Output Voltage
                  LoAd: Current in AMPs
                  Freq: AC Frequency of Phase R        

ALARMS: A AI: All Input Voltage Error
        A AO: All Output Voltage Error
        A rI: R Phase Input Voltage Error
        A YI: Y Phase Input Voltage Error
        A bI: B Phase Input Voltage Error
        A rO: R Phase Output Voltage Error
        A YO: Y Phase Output Voltage Error
        A bO: B Phase Output Voltage Error

*/

#include <EEPROM.h>
#include <BlockNot.h> //https://github.com/EasyG0ing1/BlockNot  //Can be Installed in Arduino IDE's Library Manager
#include "SevSeg.h" //https://github.com/sparkfun/SevSeg  //Can't be installed by library Manager, You'll have to manually downlaod the files.


//Define Microcontroller Pins Instead of using Variables to optimize the code

#define ok PIN_PC4
#define plus PIN_PC5
#define minus PIN_PC6

#define RINPIN PIN_PA0
#define YINPIN PIN_PA1
#define BINPIN PIN_PA2
#define ROTPIN PIN_PA3
#define YOTPIN PIN_PA4
#define BOTPIN PIN_PA5
#define current PIN_PA6
#define motorRFW PIN_PB0
#define motorRBW PIN_PB1
#define motorYFW PIN_PB2
#define motorYBW PIN_PB3  
#define motorBFW PIN_PB4
#define motorBBW PIN_PB5
#define contactor PIN_PB6
#define hz PIN_PA7

/*
#define hz PIN_PC2
#define inVolt PIN_PC3
#define outVolt PIN_PC4
#define current PIN_PC5
#define motor0Fwd PIN_PB4
#define motor0Rev PIN_PB3
#define power PIN_PB2
*/


//Intialize the SevenSegament Display
SevSeg display1;
//Timer that updates Main screen every Second.
BlockNot refresh(1, SECONDS);
BlockNot btn0(1, SECONDS);
BlockNot btn1(1, SECONDS);
BlockNot inch(100);
//Timer that updates all Input Variables
BlockNot checkin(250);

//All Global Variables
bool setupm = false;
int IHV;
int ILV;
int OHV;
int OLV;
int SETV;
int OVL;
int TON;
int TOFF;
int DIFF;
int VCALRI, VCALRO, VCALYI, VCALYO, VCALBI, VCALBO;
int ACAL;
int DIRR;
int DIRY;
int DIRB;
int padm[ 4 ] = {0, 0, 0, 0};
///////////////////////////////////////////
////////////////Admin Passcode/////////////
//////////////////// 1313 /////////////////
///////////////////////////////////////////
///////////////////////////////////////////
/*
float VCALBRI = VCALRI/1000;
float VCALBRO = VCALRO/1000;
float VCALBYI = VCALYI/1000;
float VCALBYO = VCALYO/1000;
float VCALBBI = VCALBI/1000;
float VCALBBO = VCALBO/1000;
*/
//float acalb = ACAL/1000;


//String padnumber;
int enc;
/////////////////////////////////////////////////////////////////////////////////////
/*const int ok = PIN_PC3;
const int plus = PIN_PC4;
const int minus = PIN_PC5;
///////////////////////////////////////////////////////////////
///const int setupPin = PIN_PB4; //No Longer using Setup Pin///
///////////////////////////////////////////////////////////////
const int inVolt = PIN_PC0;
const int outVolt = PIN_PC1;
const int current = PIN_PC2;                                 No Longer Being Used....
const int motor0Fwd = PIN_PB6;
const int motor0Rev = PIN_PB7;
const int power = PIN_PB5;
int encMenu = 0;*/
///////////////////////////////////////////////////////////////////////////////////////
int encMenu;
int menu;
long ontime,offtime;
float freq;   /*,hzdiff*/
int rinvoltage,routvoltage,yinvoltage,youtvoltage,binvoltage,boutvoltage,currentload;
int digstat;
bool auth = false;
//char load[5];
//char frequency[5];
//bool admin = false;

bool okold = false;
bool plusold = false;
bool minusold = false;
bool resetrefresh = false;
bool alarmOnce = false;
bool onTimer = false;
bool offTimer = false;
bool hzold = false;
//uncomment these variables if running without setup pin
/**/
bool mode = false;
bool switched = false;
bool tmrstp;
/**/

BlockNot on(TON, SECONDS);
BlockNot off(TOFF, SECONDS);

void setup() {
//Setup Display Data
setupDisplay();
//Setup Inputs 
//setIN(); is a custom Function Defined at the bottom of the Program!
setIN(ok);
setIN(plus);
setIN(minus);
setIN(RINPIN);
setIN(YINPIN);
setIN(BINPIN);
setIN(ROTPIN);
setIN(YOTPIN);
setIN(BOTPIN);
setIN(current);
setIN(hz);
///////////////////////////////////////////////////////
/////////comment setup pin if not needed///////////////
//setIN(setupPin); //change setup mode from RUN/SETUP//
///////////////////////////////////////////////////////

//Set Outputs
//setOUT(); is a custom Function Defined at the bottom of the Program!
setOUT(motorRFW);
setOUT(motorRBW);
setOUT(motorYFW);
setOUT(motorYBW);
setOUT(motorBFW);
setOUT(motorBBW);
setOUT(contactor);
//Setup Parameter Variables

//This for Variables to read from EEPROM Memory of the uC on startup
//comment these variables while testing in proteus
//uncomment when programming Arduino/MCU
/**/

//using the EEPROM.h Librabry Included by the Arduino IDE & Arduino Core

IHV = 2 * EEPROM.read(0);
ILV = 2 * EEPROM.read(1);
OHV = 2 * EEPROM.read(2);
OLV = 2 * EEPROM.read(3);
SETV = 2 * EEPROM.read(4);
OVL = EEPROM.read(5);
TON = EEPROM.read(6);
TOFF = EEPROM.read(7);
DIFF = EEPROM.read(8);
VCALRI = 4 * EEPROM.read(9);
VCALRO = 4 * EEPROM.read(10);
VCALYI = 4 * EEPROM.read(11);
VCALYO = 4 * EEPROM.read(12);
VCALBI = 4 * EEPROM.read(13);
VCALBO = 4 * EEPROM.read(14);
ACAL = 4 * EEPROM.read(15);
DIRR = EEPROM.read(16);
DIRY = EEPROM.read(17);
DIRB = EEPROM.read(18);

/*
VCALBRI = VCALRI/1000;
VCALBRO = VCALRO/1000;
VCALBYI = VCALYI/1000;
VCALBYO = VCALYO/1000;
VCALBBI = VCALBI/1000;
VCALBBO = VCALBO/1000;
*/
/**/
///////////////////////////////////////////////////////////////////////
//uncomment these variables while testing in proteus
//comment when programming Arduino/MCU
/*
IHV = 560;
ILV = 50;
OHV = 580;
OLV = 50;
SETV = 220;                           ONLY FOR TESTING PURPOSES!
OVL = 800;
TON = 3;
TOFF = 0;
DIFF = 5;
*/

//Setup Timers (BlockNot Lib)
/////////////////////////////////////////////////////////////////////////

//Setup Variable Timers after the variable is read from the Memory!
on.setDuration(TON, SECONDS);
off.setDuration(TOFF, SECONDS);
//Reset the Timers after time update.
on.reset();
off.reset();

}

void loop() {
  //Check OK/Menu Button
  checkok();
  //Check Plus/Up Button
  checkplus();
  //Check Minus/Down Button
  checkminus();
////////////////////////////////////////////////////////
  //comment this if running without setup pin
  /*if(read(setupPin)){
    runSetup();
  } else {
    runNormal();
  }
*/
///////////////////////////////////////////////////////
  //Switch to Parameter Edit/Run Mode
  /* Uncomment this for not using setup Pin*/
  if(mode){
    runSetup();
  } else {
    runNormal();
  }
  if(read(ok) && read(plus) && read(minus) && switched == false){
    mode = !mode;
    switched = true;
    encMenu = 0;
  }
  if(!read(ok) && !read(plus) && !read(minus) && switched == true){
    switched = false;
  }
  /**/
}

//Check Input Frequency

//Checks the wavelenght of the Sine wave to calculate the frequency
//ontime is the time that sinewave stays Positive or above the 0v Threshold
//offtime is the time that sinewave stays negative or below the 0v Threshold
void checkhz() {
  //calculates the pulse width in milliseconds //timesout after 60ms to avoid blocking other code if no input is detected!
  ontime = pulseIn(hz, HIGH);
  offtime = pulseIn(hz, LOW);
  //Calculates the Frequency by dividing the ontime and offtime with 1 sec time (or 1000000 ms)
  freq = 1000000.0 / (ontime + offtime);//(ontime + offtime);
  //dtostrf(freq, 5, 2, frequency);
}


//Check If input voltage is within Low & High voltage Set by Parameters
bool inputVok() {
  if(rinvoltage > ILV && rinvoltage < IHV && yinvoltage > ILV && yinvoltage < IHV && binvoltage > ILV && binvoltage < IHV){
    return true;
  } else {
    return false;
  }
}
//Check If output voltage is within Low & High voltage Set by Parameters
bool outputVok() {
  if(routvoltage > OLV && routvoltage < OHV && youtvoltage > OLV && youtvoltage < OHV && boutvoltage > OLV && boutvoltage < OHV){
    return true;
  } else {
    return false;
  }
}

//Check If Current Load is lower than max current Set by Parameters
bool currentok() {
  if(currentload < OVL){
    return true;
  } else {
    return false;
  }
}


//Check Voltage Difference from Set Voltage
bool rdiffcheck() { //(returns true if difference is more than set difference and runs motor)
  int dif = SETV - routvoltage;
  if(dif < 0){
    dif = dif * -1;
  }
  if(dif > DIFF){
    return true;
  } else {
    return false;
  }
}

bool ydiffcheck() { //(returns true if difference is more than set difference and runs motor)
  int dif = SETV - youtvoltage;
  if(dif < 0){
    dif = dif * -1;
  }
  if(dif > DIFF){
    return true;
  } else {
    return false;
  }
}

bool bdiffcheck() { //(returns true if difference is more than set difference and runs motor)
  int dif = SETV - boutvoltage;
  if(dif < 0){
    dif = dif * -1;
  }
  if(dif > DIFF){
    return true;
  } else {
    return false;
  }
}


//Run Mode

void runNormal() {
  if(checkin.triggered()){
    //////////////////////////////
    //OVo();                    //
    //IVo();                    //
    //ampo();                   //
    ////////////////////////////// No Longer Used...!
    checkinputs();
  }
  //digitalWrite(motor0Rev, HIGH);
  if(DIRR == 0){
  if(routvoltage < SETV && rdiffcheck() && inputVok() && currentok()){
    digitalWrite(motorRFW, HIGH);
  } else {
    digitalWrite(motorRFW, LOW);
  }
  if(routvoltage > SETV && rdiffcheck() && inputVok() && currentok()){
    digitalWrite(motorRBW, HIGH);
  } else {
    digitalWrite(motorRBW, LOW);
  }
  }else{
  if(routvoltage > SETV && rdiffcheck() && inputVok() && currentok()){
    digitalWrite(motorRFW, HIGH);
  } else {
    digitalWrite(motorRFW, LOW);
  }
  if(routvoltage < SETV && rdiffcheck() && inputVok() && currentok()){
    digitalWrite(motorRBW, HIGH);
  } else {
    digitalWrite(motorRBW, LOW);
  }
  }


  if(DIRY == 0){
  if(youtvoltage < SETV && ydiffcheck() && inputVok() && currentok()){
    digitalWrite(motorYFW, HIGH);
  } else {
    digitalWrite(motorYFW, LOW);
  }
  if(youtvoltage > SETV && ydiffcheck() && inputVok() && currentok()){
    digitalWrite(motorYBW, HIGH);
  } else {
    digitalWrite(motorYBW, LOW);
  }
  }else{
  if(youtvoltage > SETV && ydiffcheck() && inputVok() && currentok()){
    digitalWrite(motorYFW, HIGH);
  } else {
    digitalWrite(motorYFW, LOW);
  }
  if(youtvoltage < SETV && ydiffcheck() && inputVok() && currentok()){
    digitalWrite(motorYBW, HIGH);
  } else {
    digitalWrite(motorYBW, LOW);
  }
  }




  if(DIRB == 0){
  if(boutvoltage < SETV && bdiffcheck() && inputVok() && currentok()){
    digitalWrite(motorBFW, HIGH);
  } else {
    digitalWrite(motorBFW, LOW);
  }
  if(boutvoltage > SETV && bdiffcheck() && inputVok() && currentok()){
    digitalWrite(motorBBW, HIGH);
  } else {
    digitalWrite(motorBBW, LOW);
  }
  }else{
  if(boutvoltage > SETV && bdiffcheck() && inputVok() && currentok()){
    digitalWrite(motorBFW, HIGH);
  } else {
    digitalWrite(motorBFW, LOW);
  }
  if(boutvoltage < SETV && bdiffcheck() && inputVok() && currentok()){
    digitalWrite(motorBBW, HIGH);
  } else {
    digitalWrite(motorBBW, LOW);
  }
  }



  if(checksystem()){
    updateScreenData(true);
  } else {
    updateScreenData(false);
  }

  updatePower();

}

//Check if Input,Output Voltage and current is within the set range

bool checksystem() {
  if(inputVok()) {
    if(outputVok()) {
      if(currentok()){
            return true;
      } else { return false; }
    } else { return false; }
  } else { return false; }
}

//Control Output Supply Relay

void updatePower() {
  //digitalWrite(contactor, HIGH);
  if(checksystem()){
    if(on.triggered(false)){  
      digitalWrite(contactor, HIGH);
      off.reset();
    }
  }
  else {
    if(off.triggered(false)) {
      digitalWrite(contactor, LOW);
      on.reset();
    }
  }
}

//Update Run Mode Screen

bool RINOK() {
  if(rinvoltage > ILV && rinvoltage < IHV){
    return true;
  } else {
    return false;
  }
}

bool YINOK() {
  if(yinvoltage > ILV && yinvoltage < IHV){
    return true;
  } else {
    return false;
  }
}

bool BINOK() {
  if(binvoltage > ILV && binvoltage < IHV){
    return true;
  } else {
    return false;
  }
}

bool ROTOK() {
  if(routvoltage > OLV && routvoltage < OHV){
    return true;
  } else {
    return false;
  }
}

bool YOTOK() {
  if(youtvoltage > OLV && youtvoltage < OHV){
    return true;
  } else {
    return false;
  }
}

bool BOTOK() {
  if(boutvoltage > OLV && boutvoltage < OHV){
    return true;
  } else {
    return false;
  }
}


void updateScreenData(bool status) {
  //uncomment !mode and comment !read(setupPin) if setupPin is not being used
  if(!mode/*!read(setupPin)*/){
    if(!resetrefresh){
      refresh.reset();
      resetrefresh = true;
    }
    if(!status && !alarmOnce){
      alarmOnce = true;
      menu == -1;
    }
    if(status && alarmOnce){
      alarmOnce = false;
      menu == 0;
    }
    if(refresh.triggered()){

      OVo();
      IVo();
      ampo();


      switch (menu)
      {
        case 28:
          checkhz();
          break;
        case 0:
          IVo();
          break;
        case 2:
          OVo();
          break;
        case 4:
          ampo();
          break;
        default:
          break;
      }
      if(!tmrstp){
        menu++;
      }
    }

    //Show Error if Available
    
    if(!status && menu == -1){
      if(!inputVok() && !outputVok() && !currentok()){
        display("A AL", 0);
      } else {
        if(!inputVok()){
          if(!RINOK() && !YINOK() && !BINOK()) { display("A AI", 0);}
          else{
            if(!RINOK()){ display("A rI", 0); }
            if(!YINOK()){ display("A YI", 0); }
            if(!BINOK()){ display("A bI", 0); }
          }
        }
        if(!outputVok()){
          if(!ROTOK() && !YOTOK() && !BOTOK()) { display("A AO", 0);}
          else{
            if(!ROTOK()){ display("A rO", 0); }
            if(!YOTOK()){ display("A YO", 0); }
            if(!BOTOK()){ display("A bO", 0); }
          }
        }
        if(!currentok()){
          display("ALOL", 0);
        }
      }
    }

    switch (menu)
    {
    case 0:
      display("noor", 0);
      break;
    case 1:
      display("r IP", 0);
      break;
    case 2:
      displayVar(rinvoltage, 0);
      break;
    case 3:
      display("r OP", 0);
      break;
    case 4:
      displayVar(routvoltage, 0);
      break;
    case 5:
      display("Y IP", 0);
      break;
    case 6:
      displayVar(yinvoltage, 0);
      break;
    case 7:
      display("Y OP", 0);
      break;
    case 8:
      displayVar(youtvoltage, 0);
      break;
    case 9:
      display("b IP", 0);
      break;
    case 10:
      displayVar(binvoltage, 0);
      break;
    case 11:
      display("b OP", 0);
      break;
    case 12:
      displayVar(boutvoltage, 0);
      break;
    case 13:
      display("rYIP", 0);
      break;
    case 14:
      displayVar((rinvoltage + yinvoltage) * 0.866, 0);
      break;
    case 15:
      display("rYOP", 0);
      break;
    case 16:
      displayVar((routvoltage + youtvoltage) * 0.866, 0);
      break;
    case 17:
      display("YbIP", 0);
      break;
    case 18:
      displayVar((yinvoltage + binvoltage) * 0.866, 0);
      break;
    case 19:
      display("YbOP", 0);
      break;
    case 20:
      displayVar((youtvoltage + boutvoltage) * 0.866, 0);
      break;
    case 21:
      display("brIP", 0);
      break;
    case 22:
      displayVar((binvoltage + rinvoltage) * 0.866, 0);
      break;
    case 23:
      display("brOP", 0);
      break;
    case 24:
      displayVar((boutvoltage + routvoltage) * 0.866, 0);
      break;
    case 25:
      display("LoAd", 0);
      break;
    case 26:
      displayVar((int)currentload, 0);
      break;
    case 27:
      display("FrEq", 0);
      break;
    case 28:
      displayVar((int)freq, 0);
      break;
    case 29:
      if(status){
        menu = 0;
      } else {
        menu = -1;
      }
      break;
    case -2:
      menu = 28;
      break;
    case -1:
      if(status){ menu = 28; }
      break;
    }

    /*if(menu == 0){
      display("InPu", 0);
    }
    if(menu == 1){
      displayVar(involtage, 0);
    }
    if(menu == 2){
      display("Outu", 0);
    }
    if(menu == 3){
      displayVar(outvoltage, 0);
    }
    if(menu == 4){
      display("LoAd", 0);
    }
    if(menu == 5){
      displayVar(currentload, 0);
    }
    if(menu == 6){
      display("FrEq", 0);
    }
    if(menu == 7){
      displayVar((int)freq, 0);
    }
    if(menu == 8){
      if(status){
        menu = 0;
      } else {
        menu = -1;
      }
    }*/
  }
}
//update inputs once based on highest voltage...
////////////////////////////////////////////////////////////////
/*
void IVo() {
  involtage = 0.64 * analogRead(inVolt);
}

void OVo() {
  outvoltage = 0.64 * analogRead(outVolt);
}

void ampo() {
  currentload = 0.64 * analogRead(current); /// No Longer Used...
}
*/
/////////////////////////////////////////////////////////////////


//update inputs once based on highest voltage...
void checkinputs() {
  IVo();
  OVo();
  ampo();
}

void IVo() {

float rin1, yin1, bin1;
for(int i=0; i<1; i++) {
  if(((VCALRI/1000.0) * analogRead(RINPIN)) > rin1){
    rin1 = (VCALRI/1000.0) * analogRead(RINPIN);
    }
  rinvoltage = (int)rin1;
  if(((VCALYI/1000.0) * analogRead(YINPIN)) > yin1){
    yin1 = (VCALYI/1000.0) * analogRead(YINPIN);
    }
  yinvoltage = (int)yin1;
  if(((VCALBI/1000.0) * analogRead(BINPIN)) > bin1){
    bin1 = (VCALBI/1000.0) * analogRead(BINPIN);
    }
  binvoltage = (int)bin1;
  }
}



void OVo() {
float rot1, yot1, bot1;
for(int i=0; i<1; i++) {
  if(((VCALRO/1000.0) * analogRead(ROTPIN)) > rot1){
    rot1 = (VCALRO/1000.0) * analogRead(ROTPIN);
    }
  routvoltage = (int)rot1;
  if(((VCALYO/1000.0) * analogRead(YOTPIN)) > yot1){
    yot1 = (VCALYO/1000.0) * analogRead(YOTPIN);
    }
  youtvoltage = (int)yot1;
  if(((VCALBO/1000.0) * analogRead(BOTPIN)) > bot1){
    bot1 = (VCALBO/1000.0) * analogRead(BOTPIN);
    }
  boutvoltage = (int)bot1;
  }
}

void ampo() {
float ampov;
for(int i=0; i<1; i++) {
  if(((ACAL/1000.0) * analogRead(current)) > ampov){
    ampov = (ACAL/1000.0) * analogRead(current);
    }
  }
currentload = ampov;
//dtostrf(ampov, 5, 1, load);

}

////////////////////////////////////////////////////////////
//Setp display on Setup Mode

void home() {
  display("SETP", 0);
}

//Setup Input High Voltage

void menuIHV() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("IHu", 0);
  }
}

//Setup Input low Voltage

void menuILV() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("ILu", 0);
  }
}

//Setup Output High Voltage

void menuOHV() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("OHu", 0);
  }
}

//Setup Output Low Voltage

void menuOLV() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("OLu", 0);
  }
}

//Setup Set Voltage

void menuSETV() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("SETu", 0);
  }
}

//Setup Overload

void menuOVL() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("OuL", 0);
  }
}

//Setup Waiting time for Output Relay to turn on

void menuTON() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("tOn", 0);
  }
}

//Setup Waiting time for Output Relay to turn off

void menuTOFF() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("tOFF", 0);
  }
}



//setup voltage difference from set voltage to move motor

void menuDIFF() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("dIFF", 0);
  }
}

void menuPADM() {
    String padnumber;
    padnumber = padnumber + (String)padm[ 0 ];
    padnumber = padnumber + (String)padm[ 1 ];
    padnumber = padnumber + (String)padm[ 2 ];
    padnumber = padnumber + (String)padm[ 3 ];
    if(refresh.triggered(false)){
    display(padnumber, 0);
  } else {
    display("P Ad", 0);
  }
}


void adminHome(){
  display("nAS", 0);
}

void voltCalibRI() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("rICL", 0);
  }
}


void voltCalibRO() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("roCL", 0);
  }
}


void voltCalibYI() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("YICL", 0);
  }
}


void voltCalibYO() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("YoCL", 0);
  }
}


void voltCalibBI() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("bICL", 0);
  }
}


void voltCalibBO() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("boCL", 0);
  }
}

void ampCalib() {
  if(refresh.triggered(false)){
    displayVar(enc, 0);
  } else {
    display("ACAL", 0);
  }
}

void motorDirR() {
  if(refresh.triggered(false)){
    if(enc == 0){
      display("R Fd", 0);
    } else if(enc == 1){
      display("R bd", 0);
    }
  } else {
    display("dIrr", 0);
  }
}

void motorDirY() {
  if(refresh.triggered(false)){
    if(enc == 0){
      display("Y Fd", 0);
    } else if(enc == 1){
      display("Y bd", 0);
    }
  } else {
    display("dIrY", 0);
  }
}

void motorDirB() {
  if(refresh.triggered(false)){
    if(enc == 0){
      display("b Fd", 0);
    } else if(enc == 1){
      display("b bd", 0);
    }
  } else {
    display("dIrb", 0);
  }
}


void menuEND() {
  display("End", 0);
}

//Change Screens/Menus on pessing OK/Menu

void runSetup() {
  if(digstat > 3){ digstat = 0; }
  if(padm[ 0 ] > 9) { padm[ 0 ] = 0; }
  if(padm[ 1 ] > 9) { padm[ 1 ] = 0; }
  if(padm[ 2 ] > 9) { padm[ 2 ] = 0; }
  if(padm[ 3 ] > 9) { padm[ 3 ] = 0; }
  int OTs[7] = { motorRFW, motorRBW, motorYFW, motorYBW, motorBFW, motorBBW, contactor };
  //digitalWrite(contactor, HIGH);
  for(int i = 0; i < 7; i++){
    digitalWrite(OTs[i], LOW);
  }
  if(auth == false){
    switch (encMenu)
    {
    case 0:
      home();
      break;
    case 1:
      menuIHV();
      break;
    case 2:
      menuILV();
      break;
    case 3:
      menuOHV();
      break;
    case 4:
      menuOLV();
      break;
    case 5:
      menuSETV();
      break;
    case 6:
      menuOVL();
      break;
    case 7:
      menuTON();
      break;
    case 8:
      menuTOFF();
      break;
    case 9:
      menuDIFF();
      break;
    case 10:
      menuPADM();
      break;
    case 11:
      menuEND();
      break;
    default:
      encMenu = 0;
      break;
    }
  }
  if(auth == true)
  {
    switch (encMenu)
    {
      case 0:
        adminHome();
        break;
      case 1:
        voltCalibRI();
        if(enc > 999){
          enc = 0;
        } else if(enc < 0){
          enc = 999;
        }
        break;
      case 2:
        voltCalibRO();
        if(enc > 999){
          enc = 0;
        } else if(enc < 0){
          enc = 999;
        }
        break;
      case 3:
        voltCalibYI();
        if(enc > 999){
          enc = 0;
        } else if(enc < 0){
          enc = 999;
        }
        break;
      case 4:
        voltCalibYO();
        if(enc > 999){
          enc = 0;
        } else if(enc < 0){
          enc = 999;
        }
        break;
      case 5:
        voltCalibBI();
        if(enc > 999){
          enc = 0;
        } else if(enc < 0){
          enc = 999;
        }
        break;
      case 6:
        voltCalibBO();
        if(enc > 999){
          enc = 0;
        } else if(enc < 0){
          enc = 999;
        }
        break;
      case 7:
        ampCalib();
        if(enc > 999){
          enc = 0;
        } else if(enc < 0){
          enc = 999;
        }
        break;
      case 8:
        motorDirR();
        if(enc > 1){
          enc = 0;
        } else if (enc < 0){
          enc = 1;
        }
        break;
      case 9:
        motorDirY();
        if(enc > 1){
          enc = 0;
        } else if (enc < 0){
          enc = 1;
        }
        break;
      case 10:
        encMenu = 11;
        break;
      case 11:
        motorDirB();
        if(enc > 1){
          enc = 0;
        } else if (enc < 0){
          enc = 1;
        }
        break;
      case 12:
        menuEND();
        break;
      default:
        encMenu = 0;
        break;
    }
  }
}

//Check OK Button Pressed



void checkok() {
  if(read(ok) && okold == !read(ok)){
  okold = read(ok);
  if(encMenu != 10){
    encMenu++;
  } else if (encMenu == 10){
    // here 1313 is the secret admin code for opening motor direction and calibration settings
    if(padm[ 0 ] == 1 && padm[ 1 ] == 3 && padm[ 2 ] == 1 && padm[ 3 ] == 3){
      //if(padm == pass){
      auth = true;
      encMenu = 0;
    }
    else{
      encMenu++;
    }
  }
  if(!mode && !tmrstp){
    tmrstp = true;
  }
  else if(!mode) {
    tmrstp = false;
  }
  refresh.reset();
  encUpdate();
  eepromUpdate();
  }
  if(read(ok) == false){
  okold = read(ok);
  }
}

//Save Parameters to MCU EEPROM Memory (only if changed)

void eepromUpdate() {
  EEPROM.update(0, IHV/2);
  EEPROM.update(1, ILV/2);
  EEPROM.update(2, OHV/2);
  EEPROM.update(3, OLV/2);
  EEPROM.update(4, SETV/2);
  EEPROM.update(5, OVL);
  EEPROM.update(6, TON);
  EEPROM.update(7, TOFF);
  EEPROM.update(8, DIFF);
  EEPROM.update(9, VCALRI/4);
  EEPROM.update(10, VCALRO/4);
  EEPROM.update(11, VCALYI/4);
  EEPROM.update(12, VCALYO/4);
  EEPROM.update(13, VCALBI/4);
  EEPROM.update(14, VCALBO/4);
  EEPROM.update(15, ACAL/4);
  EEPROM.update(16, DIRR);
  EEPROM.update(17, DIRY);
  EEPROM.update(18, DIRB);
}

//Update Parameters on Menu Change

void encUpdate() {
if(auth == false){  
  switch (encMenu)
  {
  case 0:
    break;
  case 1:
    enc = IHV;
    done();
    break;
  case 2:
    IHV = enc;
    enc = ILV;
    done();
    break;
  case 3:
    ILV = enc;
    enc = OHV;
    done();
    break;
  case 4:
    OHV = enc;
    enc = OLV;
    done();
    break;
  case 5:
    OLV = enc;
    enc = SETV;
    done();
    break;
  case 6:
    SETV = enc;
    enc = OVL;
    done();
    break;
  case 7:
    OVL = enc;
    enc = TON;
    done();
    break;
  case 8:
    TON = enc;
    enc = TOFF;
    on.setDuration(TON, SECONDS);
    on.reset();
    done();
    break;
  case 9:
    TOFF = enc;
    enc = DIFF;
    off.setDuration(TOFF, SECONDS);
    off.reset();
    done();
    break;
  case 10:
    DIFF = enc;
    done();
    padm[ 0 ] = 0;
    padm[ 1 ] = 0;
    padm[ 2 ] = 0;
    padm[ 3 ] = 0;
    break;
  default:
    break;
  }
}
if(auth == true){
  switch (encMenu)
  {
  case 0:
    break;
  case 1:
    enc = VCALRI;
    done();
    break;
  case 2:
    VCALRI = enc;
    //VCALBRI = VCALRI/1000;
    enc = VCALRO;
    done();
    break;
  case 3:
    VCALRO = enc;
    //VCALBRO = VCALRO/1000;
    enc = VCALYI;
    done();
    break;
  case 4:
    VCALYI = enc;
    //VCALBYI = VCALYI/1000;
    enc = VCALYO;
    done();
    break;
  case 5:
    VCALYO = enc;
    //VCALBYO = VCALYO/1000;
    enc = VCALBI;
    done();
    break;
  case 6:
    VCALBI = enc;
    //VCALBBI = VCALBI/1000;
    enc = VCALBO;
    done();
    break;
  case 7:
    VCALBO = enc;
    //VCALBBO = VCALBO/1000;
    enc = ACAL;
    done();
    break;
  case 8:
    ACAL = enc;
    //acalb = ACAL/1000;
    enc = DIRR;
    done();
    break;
  case 9:
    DIRR = enc;
    enc = DIRY;
    done();
    break;
  case 11:
    DIRY = enc;
    enc = DIRB;
    done();
    break;
  case 12:
    DIRB = enc;
    done();
    break;
  }
}
}

void done() {display("donE", 0);}

//Display INT Variable

void displayVar(int var, int deci) {
  char buffer[5];
  sprintf(buffer, "%4d", var);
  display1.DisplayString(buffer, deci);
}

//Check Plus Button Pressed



void checkplus() {
  if(encMenu != 10){
    if(read(plus)){
      if(btn0.triggered(false)){
        if(inch.triggered(true)){
          enc++;
        }
      }
    }
  }
  if(read(plus) && plusold == !read(plus)){
  plusold = read(plus);
  if(encMenu != 10){
    enc++;
  } else {
    switch (digstat)
    {
      case 0:
        padm[ 0 ]++;
        break;
      case 1:
        padm[ 1 ]++;
        break;
      case 2:
        padm[ 2 ]++;
        break;
      case 3:
        padm[ 3 ]++;
        break;
      default:
        break;
    }
  }
  if(!mode){
    menu++;
  }
  }
  if(read(plus) == false){
  plusold = read(plus);
  btn0.reset();
  }
}

//Check Minus Button Pressed



void checkminus() {
  if(encMenu != 10){
    if(read(minus)){
      if(btn1.triggered(false)){
        if(inch.triggered(true)){
          enc--;
        }
      }
    }
  }
  if(read(minus) && minusold == !read(minus)){
    minusold = read(minus);
    if(encMenu != 10){
      enc--;
    } if(encMenu == 10) {
      digstat = digstat + 1;
    }
    if(!mode){
      menu--;
    }
  }
  if(read(minus) == false){
  minusold = read(minus);
  btn1.reset();
  }
}

// Setup Inputs

void setIN(int PIN) {
  pinMode(PIN, INPUT);
}

//Setup Outputs

void setOUT(int PIN) {
  pinMode(PIN, OUTPUT);
}

//Read Input

bool read(int PIN) {
  if(digitalRead(PIN)) {
    return true;
  } else {
    return false;
  }
}

//Display String Variable

void display(String str, int deci) {
  int strl = str.length();
  if(strl < 4) {
    //char16 = no display on screen
    str = char(16) + str;
  }

  int str_len = str.length() + 1;
  char data[str_len];
  str.toCharArray(data, str_len);
  display1.DisplayString(data, deci);
}

//Setup Sevenseg Display

void setupDisplay() {
    int displayType = COMMON_ANODE;

   int digit1 = PIN_PC3; 
   int digit2 = PIN_PC2; 
   int digit3 = PIN_PC1; 
   int digit4 = PIN_PC0; 
   
   
   int segA = PIN_PD0; 
   int segB = PIN_PD1;
   int segC = PIN_PD2; 
   int segD = PIN_PD3; 
   int segE = PIN_PD4; 
   int segF = PIN_PD5; 
   int segG = PIN_PD6; 
   int segDP = PIN_PD7; 
   
  int numberOfDigits = 4; 

  display1.Begin(displayType, numberOfDigits, digit1, digit2, digit3, digit4, segA, segB, segC, segD, segE, segF, segG, segDP);
  
  display1.SetBrightness(100); 
}
