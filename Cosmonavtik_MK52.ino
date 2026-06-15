#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <RTClib.h>      
#include <EEPROM.h>
#include <Servo.h>
#include <math.h>        

LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(7, DHT11);
RTC_DS3231 rtc;
Servo valveServo;

const int BTN1=2, BTN2=3, BTN3=8, RGB_R=4, RGB_G=5, RGB_B=6, LED_SIGNAL=9, SERVO_PIN=10; 
enum SystemModes { HOME, FM, METEO, CALC, PONG, CALENDAR };
SystemModes mode = HOME;

int slot=0, step=0, ballX=7, ballY=1, ballDX=1, ballDY=-1, padL=0, padR=0, historyIdx=0, cmdIdx=0, lastOp=0, realRSSI=0;
float regX=0.0, regY=0.0, freq=101.2;
bool isEnterNum = true, isProg = false; 
unsigned long lastAct = 0;
float tempHistory[] = {22, 22, 23, 22, 23, 24, 23, 24};

const char* cmds[] = {
  "0","1","2","3","4","5","6","7","8","9",".","[CX]",
  "[+]","[-]","[*]","[/]","[=]","[SH]","[CH]","[TSI]",
  "[PRG]","[RUN]","[BAL]","[DER]","[INT]"
};
#define TOTAL_CMDS 25
const char* days[] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};

byte b1[]={0,0,0,0,0,0,0,31}, b2[]={0,0,0,0,0,31,31,31}, b3[]={0,0,0,31,31,31,31,31}, b4[]={31,31,31,31,31,31,31,31};

void setFreq(float f) {
  unsigned int fb = (4 * (f * 1000000 + 225000)) / 32768;
  Wire.beginTransmission(0x60); Wire.write(fb >> 8); Wire.write(fb & 0xFF); Wire.write(0xB0); Wire.write(0x10); Wire.write(0x00); Wire.endTransmission();
  delay(20); Wire.requestFrom(0x60, 5);
  if (Wire.available() >= 4) { Wire.read(); Wire.read(); Wire.read(); realRSSI = map(((Wire.read() >> 4) & 0x0F), 0, 15, 0, 100); }
}

void setRGB(int r, int g, int b) { analogWrite(RGB_R, r); analogWrite(RGB_G, g); analogWrite(RGB_B, b); }
void pD(int v) { if(v<10) lcd.print("0"); lcd.print(v); }

void renderUI() {
  lcd.setCursor(0, 0); DateTime now = rtc.now();
  if (mode == HOME) {
    lcd.print("TIME: "); pD(now.hour()); lcd.print(":"); pD(now.minute()); lcd.print("   FM ");
    lcd.setCursor(0, 1); lcd.print(freq, 1); lcd.print("MHz   HUM: "); lcd.print(dht.readHumidity(), 0); lcd.print("%");
  } else if (mode == FM) {
    lcd.print("FM RECEIVER     "); lcd.setCursor(0, 1); lcd.print(freq, 1); lcd.print("MHz RSSI:"); lcd.print(realRSSI); lcd.print("%   ");
  } else if (mode == METEO) {
    lcd.print("WEATHER LAB MON "); lcd.setCursor(0, 1); lcd.print("T:"); lcd.print(dht.readTemperature(),0); lcd.print("C H:"); lcd.print(dht.readHumidity(),0); lcd.print("% ");
    lcd.setCursor(11, 1); 
    for (int i = 0; i < 5; i++) { int idx = (historyIdx + i) % 8; lcd.write(tempHistory[idx] < 22.2 ? 1 : (tempHistory[idx] < 22.8 ? 2 : (tempHistory[idx] < 23.5 ? 3 : 4))); }
  } else if (mode == CALC) {
    lcd.print(isProg ? "MK52 PRG  ST:" : "MK52 CALC ST:"); pD(step); lcd.print("  "); 
    lcd.setCursor(0, 1); lcd.print(cmds[cmdIdx]); lcd.print("        "); lcd.setCursor(7, 1); lcd.print("X:"); 
    if (regX >= 10.0 || regX <= -10.0) { lcd.print((long)regX); lcd.print("    "); } else { lcd.print(regX, 1); lcd.print("    "); }
  } else if (mode == CALENDAR) {
    lcd.print("DATE: "); pD(now.day()); lcd.print("/"); pD(now.month()); lcd.print("/"); lcd.print(now.year());
    lcd.setCursor(0, 1); lcd.print("DAY: "); lcd.print(days[now.dayOfTheWeek()]); lcd.print("        ");
  }
}

void setSystemMode(SystemModes m) {
  mode = m; lcd.clear(); digitalWrite(LED_SIGNAL, LOW); 
  if (mode == HOME) setRGB(0, 255, 0);
  else if (mode == FM) setRGB(255, 255, 255);
  else if (mode == METEO) setRGB(0, 255, 255);
  else if (mode == CALC) setRGB(128, 0, 128);
  else if (mode == PONG) setRGB(255, 0, 0);
  else if (mode == CALENDAR) setRGB(255, 165, 0);
  renderUI();
}

int getBtn(int p) {
  if (digitalRead(p) == LOW) { unsigned long s = millis(); while (digitalRead(p) == LOW); return (millis() - s > 600) ? 2 : 1; }
  return 0;
}

void calcOp(byte opCode) {
  if (opCode >= 12 && opCode <= 15) { regY = regX; regX = 0; lastOp = opCode; }
  else if (opCode == 16) { if (lastOp == 12) regX = regY + regX; else if (lastOp == 13) regX = regY - regX; else if (lastOp == 14) regX = regY * regX; else if (lastOp == 15 && regX != 0) regX = regY / regX; }
  else if (opCode == 17) regX = sinh(regX); else if (opCode == 18) regX = cosh(regX);
  else if (opCode == 19) { if (lastOp == 14 && regY > 0) { regX = regX * log(regY); valveServo.write(constrain(map((int)regX, 0, 9000, 0, 180), 0, 180)); } } 
  else if (opCode == 22) { if(regY > 0) { float rad = regX * M_PI / 180.0; float v = regY; regX = (v * v * sin(2.0 * rad)) / 9.81; regY = (v * v * sin(rad) * sin(rad)) / (2.0 * 9.81); valveServo.write(constrain((int)regX, 0, 180)); } }
  else if (opCode == 23) regX = 2.0 * regX; else if (opCode == 24) regX = (regX * regX * regX) / 3.0;
}

void runStoredProgram() {
  isProg = false; isEnterNum = false; regX = 0; regY = 0;
  for (int i = 0; i < 30; i++) {
    byte c = EEPROM.read((slot * 30) + i); if (c == 255 || c == 20) break; 
    if (c >= 0 && c <= 10) { if (c <= 9) { regX = isEnterNum ? regX * 10 + c : c; isEnterNum = true; } else if (c == 10) isEnterNum = true; } 
    else { isEnterNum = false; calcOp(c); }
    delay(50); 
  }
}

void executeMK52Logic() {
  if (isProg && cmdIdx == 11) { if (step > 0) { step--; EEPROM.update((slot * 30) + step, 255); regX = 0; } return; }
  if (isProg && cmdIdx != 20) { EEPROM.update((slot * 30) + step, (uint8_t)cmdIdx); step++; if (step >= 30) { step = 0; isProg = false; } return; }
  if (cmdIdx >= 0 && cmdIdx <= 9) { regX = isEnterNum ? regX * 10 + cmdIdx : cmdIdx; isEnterNum = true; return; }
  if (cmdIdx == 10) { isEnterNum = true; return; } isEnterNum = false; 
  if (cmdIdx == 11) { regX = 0; }
  else if (cmdIdx == 20) { isProg = !isProg; if (isProg) { for(int i=0; i<30; i++) EEPROM.update((slot * 30) + i, 255); step = 0; } }
  else if (cmdIdx == 21) runStoredProgram();
  else calcOp(cmdIdx);
}

void handleButtonsLogic() {
  int b1 = getBtn(BTN1), b2 = getBtn(BTN2), b3 = getBtn(BTN3); if (b1 > 0 || b2 > 0 || b3 > 0) lastAct = millis(); 
  if (mode == PONG) { if (b3 == 1) { setSystemMode(HOME); return; } if (b1 == 1) padL = !padL; if (b2 == 1) padR = !padR; return; }
  if (mode == HOME) { if (b1 == 1) setSystemMode(FM); if (b1 == 2) setSystemMode(CALENDAR); if (b2 == 1) setSystemMode(METEO); if (b2 == 2) setSystemMode(CALC); } 
  else if (mode == FM) {
    if (b1 == 1) { freq += 0.1; setFreq(freq); EEPROM.put(500, freq); renderUI(); } 
    if (b2 == 1) { freq -= 0.1; setFreq(freq); EEPROM.put(500, freq); renderUI(); } 
    if (b2 == 2) { lcd.clear(); lcd.print("SCANNING..."); freq = 102.7; setFreq(freq); delay(1200); setSystemMode(FM); } if (b3 == 1) setSystemMode(HOME); 
  } else if (mode == METEO || mode == CALENDAR) { if (b3 == 1) setSystemMode(HOME); } 
  else if (mode == CALC) {
    if (b1 == 1) { cmdIdx = (cmdIdx + 1) % TOTAL_CMDS; renderUI(); } 
    if (b2 == 1) { cmdIdx--; if (cmdIdx < 0) cmdIdx = TOTAL_CMDS - 1; renderUI(); } 
    if (b2 == 2) { executeMK52Logic(); renderUI(); } 
    if (b3 == 1) { if (regX >= 0) regX = sqrt(regX); renderUI(); } if (b3 == 2) { isProg = false; setSystemMode(HOME); } 
  }
}

void handleBackgroundTick() {
  static unsigned long lastTick = 0; if (millis() - lastTick < 1000) return; lastTick = millis();
  if (mode == FM) { analogWrite(LED_SIGNAL, map(realRSSI, 0, 100, 0, 255)); setFreq(freq); } 
  DateTime now = rtc.now(); if (now.hour() == 7 && now.minute() == 0 && now.second() == 0) setSystemMode(FM);
  static int gTimer = 0; if (++gTimer >= 3600) { gTimer = 0; tempHistory[historyIdx] = dht.readTemperature(); if (isnan(tempHistory[historyIdx])) tempHistory[historyIdx] = 23.0; historyIdx = (historyIdx + 1) % 8; }
  
  if (mode == CALC && !isProg) {
    Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission(false); Wire.requestFrom(0x68, 6, true);
    if(Wire.available() >= 6) {
      int16_t acX = Wire.read() << 8 | Wire.read(); Wire.read(); Wire.read(); Wire.read(); Wire.read();
      valveServo.write(constrain(90 - map(acX, -16384, 16384, -90, 90), 0, 180));
    }
  }

  if (millis() - lastAct > 20000 && mode == HOME) setSystemMode(PONG);
  if (mode != PONG) renderUI();
  else {
    ballX += ballDX; ballY += ballDY; if (ballY <= 0 || ballY >= 1) ballDY = -ballDY;
    if (ballX <= 1 && ballY == padL) { ballDX = 1; ballX = 1; } if (ballX >= 14 && ballY == padR) { ballDX = -1; ballX = 14; }
    if (ballX < 0 || ballX > 15) { digitalWrite(LED_SIGNAL, HIGH); delay(150); digitalWrite(LED_SIGNAL, LOW); ballX = 7; ballY = 1; } 
    lcd.clear(); lcd.setCursor(0, padL); lcd.print("|"); lcd.setCursor(15, padR); lcd.print("#"); lcd.setCursor(ballX, ballY); lcd.print("o");
  }
}

void setup() {
  Wire.begin(); lcd.init(); lcd.backlight(); dht.begin(); rtc.begin(); valveServo.attach(SERVO_PIN); valveServo.write(90);
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
  pinMode(BTN1, INPUT_PULLUP); pinMode(BTN2, INPUT_PULLUP); pinMode(BTN3, INPUT_PULLUP);
  pinMode(RGB_R, OUTPUT); pinMode(RGB_G, OUTPUT); pinMode(RGB_B, OUTPUT); pinMode(LED_SIGNAL, OUTPUT);
  lcd.createChar(1, b1); lcd.createChar(2, b2); lcd.createChar(3, b3); lcd.createChar(4, b4);
  setRGB(128, 0, 128); lcd.setCursor(2, 0); lcd.print("HELLO WORLD!"); lcd.setCursor(3, 1); lcd.print("DIY SYSTEM"); delay(2000); lcd.clear();
  if (rtc.lostPower()) { rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); }
  float sf; EEPROM.get(500, sf); if (sf >= 87.5 && sf <= 108.0) freq = sf; setFreq(freq); lastAct = millis(); setSystemMode(HOME);
}

void loop() { handleButtonsLogic(); handleBackgroundTick(); }

