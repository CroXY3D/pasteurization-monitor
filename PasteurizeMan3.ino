

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <LcdBarGraphAda.h>

// ADAFRUIT LCD backpack color defines
#define RGBLCD_RED 0x1
#define RGBLCD_GREEN 0x2
#define RGBLCD_BLUE 0x4
#define RGBLCD_YELLOW (RGBLCD_RED | RGBLCD_GREEN)
#define RGBLCD_MAGENTA (RGBLCD_RED | RGBLCD_BLUE)
#define RGBLCD_TEAL (RGBLCD_GREEN | RGB_LCD_BLUE)
#define RGBLCD_WHITE (RGBLCD_RED | RGBLCD_GREEN | RGBLCD_BLUE)


// PIN ASSIGNMENTS
#define STROBE_PIN 1
#define PIEZO_PIN 8
#define ONE_WIRE_SENSOR_BOTTLE_PIN 12
//#define ONE_WIRE_SENSOR_BATH_PIN 13


// DEFAULT PASTEURIZATION SETTINGS
#define PAST_REFTEMP 60.0  // Should be 60C
#define PAST_RESETTEMP 30.0  // should be around 25 or 30 
#define PAST_ZVALUE 7.0
#define PAST_THRESHOLD_PULL  25.0 // # of PU when should be pulled from bath
#define PAST_THRESHOLD_COMPLETE 50.0 // # of PUs when completed

// These are also used as EEPROM addresses for each of these settings (Except PROG_RUN), and must match SETTINGS structure
#define PROG_RUN 0
#define PROG_SETTINGPULL 1
#define PROG_SETTINGCOMPLETE 2
#define PROG_SETTINGREFTEMP 3
#define PROG_SETTINGZVALUE 4    // uggg, need more precision.   store x10?
#define PROG_SETTINGRESETTEMP 5
#define PROG_MIN 0
#define PROG_MAX 6

  
#define SETTINGS_VER 3    // bump when settings struct changes
// Settings
//    Stores user modifiable settings in EEPROM
//
//  WARNING!  Must be simple class.   Bump SETTINGS_VER if class is modified
class Settings
  {
public:    
  Settings()
    {
    Init();
 
    };
  
  void Init();  
  void WriteEEPROM();
  int FReadEEPROM();  
  uint8_t GetSetting(int md);
  void SetSetting(int md, uint8_t b);
 
    
  uint8_t bVer;
  uint8_t wPUPull;
  uint8_t wPUComplete;
  uint8_t wRefTemp;
  uint8_t wZValue;
  uint8_t wResetTemp;
  };
  


// Setting flags
#define SF_BYTE 0x01
#define SF_TEMPERATURE 0x80

typedef struct _settingentry
  {
  char *szName;
  uint8_t flags;
  uint8_t bMin;
  uint8_t bMax;
  } SETTING_ENTRY;

// WARNING:  Must mirror Settings class layout  
SETTING_ENTRY rgse[] =
  {
    {"Pull at PU", SF_BYTE, 1, 100},
    {"Total PU", SF_BYTE ,1, 100},
    {"Ref T", SF_BYTE|SF_TEMPERATURE,20,80},
    {"Z Value", SF_BYTE,1, 20},
    {"Zero PU T", SF_BYTE|SF_TEMPERATURE,0, 100}
  };  




class PMan
  {
public: 
  PMan() : oneWireBottle(ONE_WIRE_SENSOR_BOTTLE_PIN), sensorsBottle(&oneWireBottle)
    {
    fAlertsEnabled = 1;
    fStrobeOn = 0;
    //bg.setup(&lcd, 16, 0, 1);
    };


  void setup();
  void loop();
   
private:  
  void SetPastMode(int md);
  int FReadTemperatures(void);
  void HandlePasteurization(void);
  void DisplayPasteurizationProgress(void);
  void MakePasteurizationSounds(void);
  void UpdateStrobe();
  void HandleButtons(void);
  void DelayButtonUp(int ms);
  
  
  void HandleSettings(uint8_t buttons);
  void DisplaySetting(void);

void print2Digits(int w);
void printFloat2(float num, int digRight);
void printTemp(char * strLabel, float numTemp, int prec);
void printMMSS(unsigned long ms);
 

  void SetSettingFromMdProg(uint8_t b) { settings.SetSetting(mdProg, b); }
  uint8_t GetSettingFromMdProg() {return settings.GetSetting(mdProg);}
  
  Adafruit_RGBLCDShield lcd;  // = Adafruit_RGBLCDShield();


  // temp
  void InitBargraph(void);  
  void DrawBargraph(float value, float maxValue);

  LcdBarGraphAda bg; // = LcdBarGraphAda(&lcd, 16);
  
  Settings settings;

  int mdProg;
  int mdPast;

  // process variables
  float numTempHeater;
  float numTempBath;
  float numTempBottle;
  double  numPU;
  unsigned long msTime;
  unsigned long msTimePasteurization;
 
  unsigned long msLastButton;
  unsigned long msSoundInterval;
  int fAlertsEnabled;
  int fStrobeOn;
  
  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  OneWire oneWireBottle; //(ONE_WIRE_SENSOR_BOTTLE_PIN);
  //OneWire oneWireBath(ONE_WIRE_SENSOR_BATH);


// Pass our oneWire reference to Dallas Temperature. 
  DallasTemperature sensorsBottle; //(&oneWireBottle);
  //DallasTemperature sensorsBath(&oneWireBath);

  
  };



PMan pman;


// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.



#define DEG_CHAR 223
#define PU_CHAR 7
#define DHC_CHAR 6
uint8_t rgbPUChar[] = {0x1c,0x14,0x1c,0x10,0x15,0x5,0x7,0x0};
uint8_t rgbDHCChar[] = {0x6,0xf,0x1f,0x1f,0xb,0x7,0xf,0xf};
#define DX_LCD 16
#define DY_LCD 2

// globals used by HandlePasteurization and DisplayPasteurizationProgress



#define PAST_MODE_COLD 0
#define PAST_MODE_ACCUMULATING 1
#define PAST_MODE_PULL 2
#define PAST_MODE_COMPLETE 3





/* Hardware design:

   
Buttons:

Up
Down
Menu   // cycles thru settings.   Times out after nn seconds
Reset  // resets system and PUs.  
Reset+Menu returns settings to default

Hardware Connnections:  (this is old!!!!)
ANA4,5: I2C to Adafruit RGB Display Shield
DIG2: Heater OneWire Dallas Temp Sensor
DIG3: Bath OneWire Dallas Temp Sensor
DIG4: Bottle OneWire Dallas TempSensor

TBD Features:
Buzzer upon Bath pull, completion?
DS1820 is +/- 0.5deg C.   Calibrate?  Assume worst, menu option?


*/


/*-----------------
   UTIL FUNCTIONS
-------------------*/
void MyTone(int freq, int msDuration)
  {
  if (freq != 0)
    tone(PIEZO_PIN, freq);
  delay(msDuration);
  noTone(PIEZO_PIN);
  delay(50);
  }





void Settings::Init()
  {
  bVer = SETTINGS_VER;
  wPUPull = PAST_THRESHOLD_PULL;
  wPUComplete = PAST_THRESHOLD_COMPLETE;
  wRefTemp = PAST_REFTEMP;
  wZValue = PAST_ZVALUE;
  wResetTemp = PAST_RESETTEMP;  
  }

void Settings::WriteEEPROM()
  {
  uint8_t *pb;
  int ib;
  
  pb = (uint8_t *)this; //&settings;
  for (ib = 0; ib < sizeof(Settings); ib++)
    {
    EEPROM.write(ib, *pb);
    pb++;
    }
  }
 

  
int Settings::FReadEEPROM()
  {
  uint8_t *pb;
  int ib;
  uint8_t b;
  
  b = EEPROM.read(0);
  if (b != SETTINGS_VER)
    return 0;
  
  pb = (uint8_t *)this; //&settings;
  for (ib = 0; ib < sizeof(Settings); ib++)
    {
    *pb = EEPROM.read(ib);
    pb++;
    }
  return 1;
  }
  
  
uint8_t Settings::GetSetting(int md)
  {
  uint8_t *pb;
  pb = (uint8_t *)this; //&settings;
  return *(pb+md);
  }
  
void Settings::SetSetting(int md, uint8_t b)
  {
  uint8_t *pb;
  pb = (uint8_t *)this; // &settings;
  *(pb+md) = b;
  WriteEEPROM();  // ISSUE:  Writes all settings.
  }
  


void PMan::setup(void)
{
    uint8_t buttons;

 // start serial port
#ifdef LOG_SERIAL
  Serial.begin(9600);
  Serial.println("Pasteurization Log");
#endif
  
  // Start up the library
  sensorsBottle.begin();
  //sensorsBath.begin();

  fStrobeOn = 0;
  fAlertsEnabled = 1;
  pinMode(STROBE_PIN, OUTPUT);
  pinMode(PIEZO_PIN, OUTPUT);
  
  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);
  lcd.setBacklight(RGBLCD_BLUE);
  lcd.clear();

  lcd.createChar(PU_CHAR, rgbPUChar);
  lcd.createChar(DHC_CHAR, rgbDHCChar);
  InitBargraph();
 // InitSettings();
 
  buttons = lcd.readButtons();
  if (!settings.FReadEEPROM())
    buttons |= BUTTON_SELECT;
  if (buttons & BUTTON_SELECT)
    {
    settings.Init();
    settings.WriteEEPROM();    
    lcd.println("Default settings");
    lcd.setCursor(0,1);
    lcd.print("loaded");
    delay(2000);
    lcd.clear();
    }
 

  numPU = 0;
  msTime = millis();
  msLastButton = msTime;  // init last button so we don't revert to run on first button press
  msSoundInterval = msTime;
  msTimePasteurization = 0;
  mdPast = PAST_MODE_COLD;
  mdProg = PROG_RUN;
  
  sensorsBottle.setWaitForConversion(false);
  sensorsBottle.requestTemperatures(); // Send the command to get temperatures
//  sensorsBath.setWaitForConversion(false);
//  sensorsBath.requestTemperatures(); // Send the command to get temperatures
}

void PMan::print2Digits(int w)
  {
  w = w % 100;
  lcd.print(w/10);
  lcd.print(w%10);
  }
  
void PMan::printFloat2(float num, int digRight)
  {
  int numT;
  
  numT = num;
  if (numT >= 10)
    lcd.print((int) numT / 10);
  else 
    lcd.print(' ');
  lcd.print((int) numT % 10);
  if (digRight > 0)
    {
    lcd.print('.');
    lcd.print((int) (num * 10)%10);
    }
  }
  
  
void PMan::printTemp(char * strLabel, float numTemp, int prec)
  {
  if (prec == 0) // round.   Really should round if prec == 1 too but i just don't care.
    numTemp += 0.5;
  lcd.print(strLabel);
  printFloat2(numTemp, prec);
  lcd.print((char)DEG_CHAR);
  }
  

void PMan::printMMSS(unsigned long ms)
  {
  unsigned long sec;
  
  sec = ms / 1000;
  print2Digits(sec/60);
  lcd.print(":");
  print2Digits(sec % 60);
  }
  
  
void PMan::DisplaySetting(void)
  {
  SETTING_ENTRY *pse;
  
  pse = &rgse[mdProg-1];

  lcd.setCursor(0,0);
  lcd.print(pse->szName);
  lcd.print(": ");
  print2Digits(GetSettingFromMdProg());
  if (pse->flags & SF_TEMPERATURE)
    lcd.print((char)DEG_CHAR);
  lcd.print("      ");  // finish out the line
  }
  
void PMan::HandleSettings(uint8_t buttons)
  {
  uint8_t b;
  SETTING_ENTRY *pse;
  if (buttons & (BUTTON_UP|BUTTON_DOWN))
    {
    pse = &rgse[mdProg-1]; 
    b = GetSettingFromMdProg();
    if (buttons & BUTTON_UP)
      {
      if (b < pse->bMax)
        SetSettingFromMdProg(b+1);
      }
    else
      {
      if (b > pse->bMin)
        SetSettingFromMdProg(b-1);  
      } 
    }
  DisplaySetting();
  }
 


  
void PMan::SetPastMode(int md)
  {
  int i;
  if (mdPast != md)
    {
    mdPast = md;
    switch (mdPast)
      {
    case PAST_MODE_COLD:
      fAlertsEnabled = 1;
      fStrobeOn = 0;
      break;
    case PAST_MODE_ACCUMULATING:
      fStrobeOn = 0;
      break;
    case PAST_MODE_PULL:
      fStrobeOn = 11;
      break;
    case PAST_MODE_COMPLETE:
      fStrobeOn = 1;
      break;
      }
    }

  }

int PMan::FReadTemperatures(void)
  {
   
  float numTempBottleSensor;
  
  numTempBottleSensor = sensorsBottle.getTempCByIndex(0);    
  sensorsBottle.requestTemperatures();
  
  if (numTempBottleSensor == DEVICE_DISCONNECTED)
    {
    numTempBottle = 0; // ACK!   Is this correct, or should I just pause if it's a hiccup?
    delay(1000);  // wait 1 second to get sample again.
    return 0;  
    }
   
  // FIR filter -- probably want something to filter out spurious signals
  numTempBottle = (numTempBottle + numTempBottleSensor)/2;
  return 1;
  }
  
void PMan::HandlePasteurization(void)
{
  unsigned long msTimeNew;
  unsigned long msDelta;
  int mdPastNew;
  
  // update 1x per second    
  msTimeNew = millis();
  msDelta = msTimeNew-msTime;
  if (msDelta < 1000)
    return;

  if (!FReadTemperatures())
    return;

  mdPastNew = mdPast;
  if (numTempBottle <= settings.wResetTemp)
    {
    numPU = 0;
    msTimePasteurization = 0;
    mdPastNew = PAST_MODE_COLD;
    }
// OLD!   Per Andrew Lea, don't count PUs < wRefTemp (60)  if (numTempBottle > ((int) settings.wRefTemp-(settings.wZValue*3/2)))
  if (numTempBottle >= settings.wRefTemp)
    {
    if (msDelta > 0)
      {
      msTimePasteurization += msDelta;
      numPU += ((float)msDelta/60000.0)*pow(10,(numTempBottle-settings.wRefTemp)/settings.wZValue);
      }  
    SetPastMode(PAST_MODE_ACCUMULATING);
    }  
  if (numPU >= settings.wPUPull)
    mdPastNew = PAST_MODE_PULL;
  if (numPU >= settings.wPUComplete)
    mdPastNew = PAST_MODE_COMPLETE;
  SetPastMode(mdPastNew);
  msTime = msTimeNew;
  
}


void PMan::InitBargraph(void)
  {
  bg.setup(&lcd, DX_LCD, 0, 1);
  }


void PMan::DrawBargraph(float value, float maxValue)
  {
  bg.drawValue(value, maxValue);
  }


void PMan::DisplayPasteurizationProgress(void)
  {
  switch (mdPast)
    {
  default:
    case PAST_MODE_COLD:
     // lcd.noBlink();
      lcd.setBacklight(RGBLCD_WHITE);
      break;
    case PAST_MODE_ACCUMULATING:
      //lcd.noBlink();
      lcd.setBacklight(RGBLCD_GREEN);
      break;
    case PAST_MODE_PULL:
      //lcd.blink();
      lcd.setBacklight(RGBLCD_YELLOW);
      break;
    case PAST_MODE_COMPLETE:
      //lcd.blink();
      lcd.setBacklight(RGBLCD_RED);
      break;
    }
     
  lcd.home();
  if (numPU >= 10.0)
    {
  //  if (numPU < 100)
    //  lcd.print(" ");
    lcd.print(numPU, 0);  // 2 chars if < 100
    }
  else
    lcd.print(numPU, 1);  // 3 chars
  lcd.write(PU_CHAR);     
  
  if (numPU >= 10.0 && numPU < 100)  
    lcd.print(" ");
  
  lcd.print(" ");
  printTemp("", numTempBottle, 1);
  lcd.print(" ");

  lcd.setCursor(11,0);
  printMMSS(msTimePasteurization);

  DrawBargraph(numPU, settings.wPUPull);
  //lcd.setCursor(15,1);
  //lcd.write(DHC_CHAR);

    
#ifdef LOG_SERIAL
  Serial.print(msTimePasteurization);
  Serial.print(',');
  Serial.print(numTempHeater);
  Serial.print(',');
  Serial.print(numTempBath);
  Serial.print(',');
  Serial.print(numTempBottle);
  Serial.print(',');
  Serial.println(numPU);
#endif
  }
  

  
void PMan::MakePasteurizationSounds(void)
  {
  unsigned long ms;
  
  if (!fAlertsEnabled)
    return;  
  ms = millis();
  if (ms > msSoundInterval+10000)
    {
    msSoundInterval = ms;
    
    switch (mdPast)
      {
    case PAST_MODE_COLD:
      break;
    case PAST_MODE_ACCUMULATING:
      MyTone(100, 100);
      break;
    case PAST_MODE_PULL:
      MyTone(1000, 450);
      MyTone(400, 300);
      MyTone(600, 200);
      MyTone(400, 300);
  
      break;
    case PAST_MODE_COMPLETE:
      MyTone(1000, 400);
      MyTone(2000, 400);
      MyTone(1000, 400);
      MyTone(2000, 400);
      MyTone(1000, 400);
      MyTone(2000, 400);
      break;
      }
    }

  }
  
void PMan::UpdateStrobe(void)
  {
  digitalWrite(STROBE_PIN, fAlertsEnabled && fStrobeOn);
  }
      

void PMan::DelayButtonUp(int ms)
  {
  uint8_t buttons;
  unsigned long msStop;
  
  msStop = millis()+ms;
  
  while (millis() < msStop)
    {
    buttons = lcd.readButtons();
    if (buttons == 0)
      return;
    }
  }
  
  
 
void PMan::HandleButtons(void)
  {
   uint8_t buttons;
   
   buttons = lcd.readButtons();
   if (buttons)
     msLastButton = millis();
   else
     {
     if ((millis() > (msLastButton+8000)) && (mdProg != PROG_RUN))  // Revert to RUN display after 8 seconds of no button presses
       mdProg = PROG_RUN;
     }
   
   if (!buttons)
     return;
   if (fAlertsEnabled && fStrobeOn)
     {
     fAlertsEnabled = 0;  // kinda goofy. but annoying piezo sounds comes on with strobe, and this turns both off
     goto Return;
     }
  
   
   if (buttons & BUTTON_SELECT)
      {
      lcd.clear();
      mdProg ++;
      if (mdProg == PROG_MAX)
        mdProg = PROG_MIN;
      } 
  if (mdProg != PROG_RUN)
    HandleSettings(buttons);
 Return:
  if (buttons)
    DelayButtonUp(400); 
  }


void PMan::loop(void)
{ 

  HandlePasteurization();  
  HandleButtons();

  if (mdProg == PROG_RUN)
    {
    DisplayPasteurizationProgress();
    MakePasteurizationSounds();
    UpdateStrobe();
    }
}
 

// arduino sketch entrypoints vectors

void setup(void)
  {
  pman.setup();  
  }

void loop(void)
  {
  pman.loop();
  }
  
  
