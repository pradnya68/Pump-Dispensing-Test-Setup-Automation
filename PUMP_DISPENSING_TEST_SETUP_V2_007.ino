// v2.005 + power-cut recovery (periodic autosave of running pumps + resume state)
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <SD.h>

// ------------------- Hardware pins -------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);

const uint8_t BRINE_PUMP       = 8;
const uint8_t HYDRA_PUMP       = 5;
const uint8_t SUBMERSIBLE_PUMP = 2;
const uint8_t BDD_PUMP         = 6;
const uint8_t RELAY_PIN        = 7;
const uint8_t SELONOID_VALVE1  = 32;
const uint8_t SELONOID_VALVE2  = 13;

const uint8_t BRINE_FEED       = 50;
const uint8_t HYDRA_FEED       = 49;
const uint8_t SUBMERSIBLE_FEED = 14;
const uint8_t BDD_FEED         = 34;
const uint8_t RELAY_FEED       = 33;

const uint8_t BTN_LEFT   = 24;
const uint8_t BTN_RIGHT  = 25;
const uint8_t BTN_SELECT = 26;
const uint8_t BTN_BACK   = 27;

const uint8_t SD_CS = 10;

// ------------------- EEPROM address map -------------------
// calibration bytes: 0,1,2,3 (already used)
// totalRunSec (unsigned long) stored at: 4*idx  => addresses 4,8,12,16 (we will use same as earlier but careful)
// relayCount stored at address 20 (unsigned long using EEPROM.put/get)
// pumpRunning flags: 40..43 (bytes)
// relayBlinking flag: 44 (byte)
const int EEPROM_CAL_BASE = 0;         // bytes 0..3
const int EEPROM_TOTAL_BASE = 4;       // 4,8,12,16
const int EEPROM_RELAY_COUNT_ADDR = 20; // unsigned long at 20
const int EEPROM_RUNNING_BASE = 40;    // bytes 40..43 for pumpRunning flags
const int EEPROM_RELAY_FLAG_ADDR = 44; // byte for relayBlinking flag

// ------------------- Application state -------------------
enum MainMenu { MENU_ALL=0, MENU_MANUAL=1, MENU_CAL=2, MENU_LOG=3 };
MainMenu menu = MENU_ALL;
bool inSubMenu = false;
int subIndex = 0;
bool inCalibration = false;
int calIndex = 0;
bool inLogView = false;
int logIndex = 0;

uint8_t calPercent[4] = {100,100,100,100};
uint8_t calPWM[4]     = {255,255,255,255};

unsigned long pumpStartMillis[4] = {0,0,0,0};
bool pumpRunning[4] = {false,false,false,false};
unsigned long totalRunSec[4] = {0,0,0,0};

unsigned long relayCount = 0;

// SD logging
File logFile;

// Buffering logs
#define MAX_BUFFER 20
String logBuffer[MAX_BUFFER];
int logBufferCount = 0;
unsigned long lastFlush = 0;

// Relay blinking
bool relayBlinking = false;
unsigned long lastRelayMillis = 0;
bool relayState = false;

// ------------------- Autosave control -------------------
// Autosave interval for running pumps (ms). Adjust if you want less frequent writes.
const unsigned long AUTOSAVE_INTERVAL_MS = 10000UL; // 10s
unsigned long lastAutosaveMillis = 0;
// Per-pump last autosave to reduce writes if multiple pumps run (keeps granularity per pump)
unsigned long lastPumpAutosave[4] = {0,0,0,0};

// ------------------- Button debounce -------------------
struct Button {
  uint8_t pin; bool lastStable; bool lastRead; unsigned long lastBounceMillis; bool pressedEvent;
};
const unsigned long DEBOUNCE_MS = 40;
Button btnLeft   = {BTN_LEFT, HIGH, HIGH, 0, false};
Button btnRight  = {BTN_RIGHT, HIGH, HIGH, 0, false};
Button btnSelect = {BTN_SELECT, HIGH, HIGH, 0, false};
Button btnBack   = {BTN_BACK, HIGH, HIGH, 0, false};

void updateButton(Button &b){
  bool raw=digitalRead(b.pin); unsigned long now=millis();
  if(raw!=b.lastRead){ b.lastBounceMillis=now; b.lastRead=raw; }
  if((now-b.lastBounceMillis)>=DEBOUNCE_MS){
    if(raw!=b.lastStable){ b.lastStable=raw; if(raw==LOW) b.pressedEvent=true; }
  }
}
bool wasPressed(Button &b){ if(b.pressedEvent){ b.pressedEvent=false; return true;} return false; }

// ------------------- Utilities -------------------
int pumpPin(int idx){ switch(idx){ case 0:return BRINE_PUMP; case 1:return HYDRA_PUMP; case 2:return SUBMERSIBLE_PUMP; case 3:return BDD_PUMP;} return BRINE_PUMP;}
const char* pumpName(int idx){ switch(idx){ case 0:return "Brine"; case 1:return "Hydra"; case 2:return "Sub"; case 3:return "BDD";} return "Pump"; }
void updatePWMFromPercent(int idx){ if(idx<0||idx>3) return; calPWM[idx]=map(calPercent[idx],0,100,0,255); if(pumpRunning[idx]) analogWrite(pumpPin(idx),calPWM[idx]);}
void saveCalToEEPROM(int idx){ if(idx<0||idx>3) return; EEPROM.update(EEPROM_CAL_BASE + idx,calPercent[idx]); logBoth(String("Saved cal ")+pumpName(idx)+" = "+String(calPercent[idx])); }
void loadCalFromEEPROM(){ for(int i=0;i<4;i++){ uint8_t v=EEPROM.read(EEPROM_CAL_BASE + i); if(v==0xFF || v>100) v=100; calPercent[i]=v; updatePWMFromPercent(i);} }

// ------------------- EEPROM helpers -------------------
void writeLongToEEPROM(int addr, unsigned long value){ EEPROM.put(addr, value); }
unsigned long readLongFromEEPROM(int addr){ unsigned long value=0; EEPROM.get(addr,value); return value; }

// ------------------- Logging -------------------
String formatTime(unsigned long seconds){ unsigned long h=seconds/3600; unsigned long m=(seconds%3600)/60; char buf[10]; sprintf(buf,"%02lu:%02lu",h,m); return String(buf);} 

void bufferLog(String line){
  Serial.println(line);
  if(logBufferCount<MAX_BUFFER){ logBuffer[logBufferCount++]=line; }
}

void flushLogsToSD(){
  if(logBufferCount==0) return;
  logFile=SD.open("log.csv", FILE_WRITE);
  if(logFile){ for(int i=0;i<logBufferCount;i++){ logFile.println(logBuffer[i]); } logFile.close(); logBufferCount=0; }
}

void logBoth(String line){ bufferLog(line); }

// ------------------- Relay blink -------------------
void startRelayBlink(){ relayBlinking=true; relayState=false; lastRelayMillis=millis(); digitalWrite(RELAY_PIN, LOW); digitalWrite(SELONOID_VALVE1, LOW); digitalWrite(SELONOID_VALVE2, LOW); }
void stopRelayBlink(){ relayBlinking=false; digitalWrite(RELAY_PIN, LOW); digitalWrite(SELONOID_VALVE1, LOW); digitalWrite(SELONOID_VALVE2, LOW); }
void handleRelayBlink(){ if(!relayBlinking)return; unsigned long now=millis(); if(now-lastRelayMillis>=500){ relayState=!relayState; digitalWrite(RELAY_PIN, relayState?HIGH:LOW); digitalWrite(SELONOID_VALVE1, relayState?HIGH:LOW); digitalWrite(SELONOID_VALVE2, relayState?HIGH:LOW);  lastRelayMillis=now; }}

// ------------------- UI -------------------
void drawMainMenu(){ 
  lcd.clear(); 
  lcd.setCursor(0,0); 
  lcd.print("Select: v2.007"); 
  lcd.setCursor(0,1); 
  switch(menu){ 
    case MENU_ALL:lcd.print("[All] Man Cal Log"); 
    break; 
    case MENU_MANUAL:lcd.print("All [Man] Cal Log"); 
    break; 
    case MENU_CAL:lcd.print("All Man [Cal] Log"); 
    break; 
    case MENU_LOG:lcd.print("All Man Cal [Log]"); 
    break; 
    }
}

void drawAllSubmenu(bool showingOn){ lcd.clear(); lcd.setCursor(0,0); lcd.print("All Pumps"); lcd.setCursor(0,1); lcd.print(showingOn?"[ON]  OFF":" ON  [OFF]");}
void drawManualMenu(){ const char* names[5] = {"Brine","Hydra","Sub","BDD","Relay"}; lcd.clear(); lcd.setCursor(0,0); lcd.print(names[subIndex]); lcd.setCursor(0,1); if(subIndex<4) lcd.print(pumpRunning[subIndex]?"ON":"OFF"); else lcd.print(relayBlinking?"RELAY BLINK":"RELAY OFF");}
void drawCalMenu(){
  if(calIndex<4){ lcd.clear(); lcd.setCursor(0,0); lcd.print("Cal "); lcd.print(pumpName(calIndex)); lcd.setCursor(0,1); lcd.print(calPercent[calIndex]); lcd.print("% "); lcd.print(calPWM[calIndex]); }
  else{ lcd.clear(); lcd.setCursor(0,0); lcd.print("Clear All?"); lcd.setCursor(0,1); lcd.print("Sel=Yes Back=No"); }
}
void drawCentered(String l1,String l2){ lcd.clear(); lcd.setCursor(0,0); lcd.print(l1); lcd.setCursor(0,1); lcd.print(l2);} 
void drawLogView(){
  switch(logIndex){
    case 0: drawCentered("Brine total:",formatTime(totalRunSec[0])); break;
    case 1: drawCentered("Hydra total:",formatTime(totalRunSec[1])); break;
    case 2: drawCentered("BDD total:",formatTime(totalRunSec[3])); break;
    case 3: drawCentered("Sub total:",formatTime(totalRunSec[2])); break;
    case 4: drawCentered("Relay count:",String(relayCount)); break;
  }
}

// ------------------- Pump State EEPROM helpers -------------------
void savePumpStateToEEPROM(){
  for(int i=0;i<4;i++){
    EEPROM.update(EEPROM_RUNNING_BASE + i, pumpRunning[i] ? 1 : 0);
  }
  EEPROM.update(EEPROM_RELAY_FLAG_ADDR, relayBlinking ? 1 : 0);
}

void loadPumpStateFromEEPROMAndResume(){
  // If a pump was running before power loss, resume it.
  for(int i=0;i<4;i++){
    uint8_t s = EEPROM.read(EEPROM_RUNNING_BASE + i);
    if(s == 1){
      // Resume pump: set running flag, PWM and start time
      if(!pumpRunning[i]){
        // set pumpRunning and pumpStartMillis to now so new run continues from saved total
        pumpRunning[i] = true;
        pumpStartMillis[i] = millis();
        analogWrite(pumpPin(i), calPWM[i]);
        logBoth(String("Resume ") + pumpName(i) + ",RESUMED");
      }
    }
  }
  uint8_t rb = EEPROM.read(EEPROM_RELAY_FLAG_ADDR);
  if(rb == 1){
    // resume relay blink
    relayBlinking = false; // ensure startRelayBlink sets timers correctly
    startRelayBlink();
    logBoth("Resume Relay Blink");
  }
}

// ------------------- Pump Control -------------------
void startPump(int idx){
  if(idx < 0 || idx > 3) return;
  if(calPWM[idx] == 0) return;  // Do not start if PWM = 0%
  analogWrite(pumpPin(idx), calPWM[idx]);
  pumpStartMillis[idx] = millis();
  pumpRunning[idx] = true;
  logBoth(String(pumpName(idx)) + ",ON");
  savePumpStateToEEPROM();
  unsigned long snapshot = totalRunSec[idx];
  writeLongToEEPROM(EEPROM_TOTAL_BASE + idx*4, snapshot);
  lastPumpAutosave[idx] = millis();
}


void stopPump(int idx){
  if(idx < 0 || idx > 3) return;
  analogWrite(pumpPin(idx), 0);
  if(pumpRunning[idx] && calPWM[idx] > 0){
    unsigned long dur = (millis() - pumpStartMillis[idx]) / 1000;
    totalRunSec[idx] += dur;
    writeLongToEEPROM(EEPROM_TOTAL_BASE + idx*4, totalRunSec[idx]);
    logBoth(String(pumpName(idx)) + ",OFF,run=" + String(dur) + "s,total=" + formatTime(totalRunSec[idx]));
  }
  pumpRunning[idx] = false;
  savePumpStateToEEPROM();
  lastPumpAutosave[idx] = millis();
}


void toggleRelay(){
  if(relayBlinking){ stopRelayBlink(); logBoth("Relay,OFF"); }
  else{ startRelayBlink(); logBoth("Relay,ON"); }
  relayCount++;
  writeLongToEEPROM(EEPROM_RELAY_COUNT_ADDR, relayCount);
  logBoth(String("Relay,TOGGLE,count=") + String(relayCount));
  // Save relay flag and pump flags to EEPROM
  savePumpStateToEEPROM();
}

// Toggle all pumps
void toggleAllPumps(){
  bool anyOff = false;
  // Check if any pump that can actually run (PWM>0) is off
  for(int i=0;i<4;i++){
    if(calPWM[i] > 0 && !pumpRunning[i]) anyOff = true;
  }

  if(anyOff){
    // Start only pumps that have PWM>0
    for(int i=0;i<4;i++){
      if(calPWM[i] > 0) startPump(i);
    }
    toggleRelay();
  } else {
    // Stop only pumps that have PWM>0
    for(int i=0;i<4;i++){
      if(calPWM[i] > 0) stopPump(i);
    }
    toggleRelay();
  }

  drawAllSubmenu(anyOff);
}



// ------------------- Setup -------------------
void setup(){
  Serial.begin(115200);
  pinMode(BRINE_PUMP,OUTPUT); pinMode(HYDRA_PUMP,OUTPUT); pinMode(SUBMERSIBLE_PUMP,OUTPUT); pinMode(BDD_PUMP,OUTPUT); pinMode(RELAY_PIN,OUTPUT);pinMode(SELONOID_VALVE1,OUTPUT);pinMode(SELONOID_VALVE2,OUTPUT);
  pinMode(BRINE_FEED,INPUT_PULLUP); pinMode(HYDRA_FEED,INPUT_PULLUP); pinMode(SUBMERSIBLE_FEED,INPUT_PULLUP); pinMode(BDD_FEED,INPUT_PULLUP); pinMode(RELAY_FEED,INPUT_PULLUP);
  pinMode(BTN_LEFT,INPUT_PULLUP); pinMode(BTN_RIGHT,INPUT_PULLUP); pinMode(BTN_SELECT,INPUT_PULLUP); pinMode(BTN_BACK,INPUT_PULLUP);

  lcd.init(); lcd.backlight();
  if(!SD.begin(SD_CS)) Serial.println("SD init failed");
  else{ logFile=SD.open("log.csv",FILE_WRITE); if(logFile){ logFile.println("Pump,Event,Duration,RelayCount"); logFile.close(); } }

  // load calibration & PWM
  loadCalFromEEPROM();

  // load totals
  for(int i=0;i<4;i++) totalRunSec[i] = readLongFromEEPROM(EEPROM_TOTAL_BASE + i*4);

  // load relay count
  relayCount = readLongFromEEPROM(EEPROM_RELAY_COUNT_ADDR);

  // initialize last autosave timestamps
  unsigned long now = millis();
  for(int i=0;i<4;i++) lastPumpAutosave[i] = now;

  // Resume pumps/relay that were running before power loss (if any)
  loadPumpStateFromEEPROMAndResume();

  bufferLog(String("Init relay count: ") + String(relayCount));
  for(int i=0;i<4;i++) bufferLog(String("Init ") + pumpName(i) + " total: " + String(totalRunSec[i]));
  drawMainMenu();
}

// ------------------- Loop -------------------
void loop(){
  updateButton(btnLeft); updateButton(btnRight); updateButton(btnSelect); updateButton(btnBack);

  if(wasPressed(btnLeft)){
    if(inCalibration){ if(calIndex<4){ calPercent[calIndex]=(calPercent[calIndex]>=10?calPercent[calIndex]-10:0); updatePWMFromPercent(calIndex); drawCalMenu(); }}
    else if(inSubMenu){ subIndex=(subIndex-1+5)%5; drawManualMenu(); }
    else if(inLogView){ logIndex=(logIndex-1+5)%5; drawLogView(); }
    else{ menu=MainMenu((menu-1+4)%4); drawMainMenu(); }
  }
  if(wasPressed(btnRight)){
    if(inCalibration){ if(calIndex<4){ calPercent[calIndex]=(calPercent[calIndex]<=90?calPercent[calIndex]+10:100); updatePWMFromPercent(calIndex); drawCalMenu(); }}
    else if(inSubMenu){ subIndex=(subIndex+1)%5; drawManualMenu(); }
    else if(inLogView){ logIndex=(logIndex+1)%5; drawLogView(); }
    else{ menu=MainMenu((menu+1)%4); drawMainMenu(); }
  }
  if(wasPressed(btnSelect)){
    if(inCalibration){
      if(calIndex<4){ saveCalToEEPROM(calIndex); calIndex++; if(calIndex>4){ inCalibration=false; drawMainMenu(); } else drawCalMenu(); }
      else{ // Clear All selected
        for(int i=0;i<4;i++){ totalRunSec[i]=0; writeLongToEEPROM(EEPROM_TOTAL_BASE + i*4,0);} relayCount=0; writeLongToEEPROM(EEPROM_RELAY_COUNT_ADDR,0);
        bufferLog("All totals cleared");
        lcd.clear(); lcd.setCursor(0,0); lcd.print("All Cleared!"); delay(1000);
        inCalibration=false; drawMainMenu();
      }
    }
    else if(inSubMenu){ if(subIndex<4){ if(pumpRunning[subIndex]) stopPump(subIndex); else startPump(subIndex);} else toggleRelay(); drawManualMenu(); }
    else if(inLogView){ }
    else{
      switch(menu){
        case MENU_ALL: toggleAllPumps(); break;
        case MENU_MANUAL: inSubMenu=true; subIndex=0; drawManualMenu(); break;
        case MENU_CAL: inCalibration=true; calIndex=0; drawCalMenu(); break;
        case MENU_LOG: inLogView=true; logIndex=0; drawLogView(); break;
      }
    }
  }
  if(wasPressed(btnBack)){
    if(inCalibration){ inCalibration=false; drawMainMenu(); }
    else if(inSubMenu){ inSubMenu=false; drawMainMenu(); }
    else if(inLogView){ inLogView=false; drawMainMenu(); }
    else drawMainMenu();
  }

  // Relay feedback counting
  static int lastRelayFeed=HIGH; int curRelayFeed=digitalRead(RELAY_FEED);
  if(lastRelayFeed==HIGH && curRelayFeed==LOW){ relayCount++; writeLongToEEPROM(EEPROM_RELAY_COUNT_ADDR,relayCount); bufferLog(String("Relay feedback: ")+String(relayCount)); }
  lastRelayFeed = curRelayFeed;

  // Periodic partial runtime autosave for running pumps
  unsigned long now = millis();
  if(now - lastAutosaveMillis >= AUTOSAVE_INTERVAL_MS){
    lastAutosaveMillis = now;
    for(int i=0;i<4;i++){
      if(pumpRunning[i] && calPWM[i] > 0){   // Only count if PWM > 0
        unsigned long runningSec = (now - pumpStartMillis[i]) / 1000;
        unsigned long snapshot = totalRunSec[i] + runningSec;
        unsigned long stored = readLongFromEEPROM(EEPROM_TOTAL_BASE + i*4);
        if(snapshot != stored){
          writeLongToEEPROM(EEPROM_TOTAL_BASE + i*4, snapshot);
          bufferLog(String("Autosave ") + pumpName(i) + ",snap=" + String(snapshot) + "s");
        }
        lastPumpAutosave[i] = now;
      }
    }
    savePumpStateToEEPROM();
}


  handleRelayBlink();

  // flush logs periodically
  if(millis() - lastFlush >= 10000){ flushLogsToSD(); lastFlush = millis(); }

  delay(5);
}
