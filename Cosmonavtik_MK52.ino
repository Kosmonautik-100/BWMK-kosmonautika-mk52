#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h> 
#include <DHT.h> 
#include <math.h> 

DHT dht(7, DHT11); LiquidCrystal_I2C lcd(0x27, 16, 2); 
float f, t = 0, rx = 0, ry = 0, ip = 0, dc = 0.0, rg = 0.0; 
const float F_MIN = 87.5, F_MAX = 108.0;
int h = 9, m = 15, s = 0, u = 0, md = 0, ci = 0, gm = 0, id = 0, last_op = 0;
unsigned long tc = 0, td = 0, tg = 0, ta = 0, bu = 0, bd = 0, b3 = 0; 
int th[] = {0, 0, 0, 0}, bx = 7, by = 0, dx = 1, dy = 1, pl = 0, pr = 1;
bool pm = false, clr_x = false; byte st[] = {0, 0, 0, 0, 0};
const char* ops[] = {".", "+", "-", "*", "/", "=", "B^", "PRG", "AVT", "G:LUN", "X>P", "LN", "EXP", "SIN", "COS", "TAN", "SH", "CH", "TH"};
byte bc = { {0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,31}, {0,0,0,0,0,0,31,31}, {0,0,0,0,0,31,31,31}, {0,0,0,31,31,31,31,31}, {0,0,31,31,31,31,31,31}, {0,31,31,31,31,31,31,31}, {31,31,31,31,31,31,31,31} };

void setRTC(byte hr, byte min) {
  Wire.beginTransmission(0x68); Wire.write(0); Wire.write(0);
  Wire.write(((min/10)<<4)|(min%10)); Wire.write(((hr/10)<<4)|(hr%10)); Wire.endTransmission();
}
void readRTC() {
  Wire.beginTransmission(0x68); Wire.write(0); Wire.endTransmission(); Wire.requestFrom(0x68, 3);
  if (Wire.available()) { s = Wire.read(); s = (s>>4)*10+(s&0x0F); m = Wire.read(); m = (m>>4)*10+(m&0x0F); h = Wire.read(); h = (h>>4)*10+(h&0x0F); }
}
void rgb(int r, int g, int b) { analogWrite(4, 255-r); analogWrite(5, 255-g); analogWrite(6, 255-b); }
void snd(byte b3_v, byte b4_v) {
  unsigned int fb = 4 * (f * 1000000 + 225000) / 32768;
  Wire.beginTransmission(0x60); Wire.write(fb>>8); Wire.write(fb&0xFF); Wire.write(b3_v); Wire.write(b4_v); Wire.write(0x00); Wire.endTransmission();
}
int getS() { 
  Wire.requestFrom(0x60, 5); for (int i = 0; i < 3; i++) if (Wire.available()) Wire.read();
  return Wire.available() ? (Wire.read()>>4)&0x0F : 0; 
}
void chk() { int sig = getS(); rgb(sig<=4?255:0, (sig>4&&sig<10)?180:(sig>=10?255:0), 0); }
void autoScan() {
  lcd.clear(); lcd.print("SCANNING..."); float bf = f; int ms = 0;
  for (float cf = F_MIN; cf <= F_MAX; cf += 0.2) { f = cf; snd(0xB0,0x10); delay(60); int sl = getS(); if (sl > ms) { ms = sl; bf = cf; } }
  f = bf; snd(0xB0,0x10); chk(); EEPROM.put(10, f);
}
void addH(int v, int *arr) { for (int i = 0; i < 3; i++) arr[i] = arr[i+1]; arr = v; }
int getB(int v, int mn, int mx) { int b = map(v, mn, mx, 1, 7); return b<1?1:(b>7?7:b); }
void runPrg() { for (int i = 0; i < 5; i++) { if(st[i] != 0) { calc(st[i]); delay(15); } } }
void doEqual() {
  if(last_op == 11) rx = ry + rx; else if(last_op == 12) rx = ry - rx; else if(last_op == 13) rx = ry * rx; else if(last_op == 14 && rx != 0) rx = ry / rx;
  ip = rx; clr_x = true; last_op = 0;
}
void rwEep(bool wr) { for (int i = 0; i < 5; i++) { if(wr) EEPROM.write(50+i, st[i]); else st[i] = EEPROM.read(50+i); } }
void calc(int op) {
  if (pm && op != 20) { if (id < 5) st[id++] = op; return; }
  if (op <= 9) { 
    if (clr_x) { ip = 0; dc = 0.0; clr_x = false; }
    if (dc > 0.0) { ip = ip + op * dc; dc *= 0.1; } else { ip = ip * 10 + op; } rx = ip; 
  }
  else if (op == 10) dc = 0.1; 
  else if (op == 16) { ry = rx; ip = 0; dc = 0.0; clr_x = true; } 
  else if (op == 11 || op == 12 || op == 13 || op == 14) { ry = rx; last_op = op; ip = 0; dc = 0.0; clr_x = true; }
  else if (op == 17) { pm = true; id = 0; for (int i = 0; i < 5; i++) st[i] = 0; } 
  else if (op == 18) { rwEep(false); runPrg(); } 
  else if (op == 19) { gm = 1; pm = false; ip = 0; dc = 0.0; rg = 1000.0; rx = 50.0; ry = 0; }
  else if (op == 20) { rwEep(true); pm = false; } 
  else if (op == 21) { if(rx > 0) rx = log(rx); ip = rx; } else if (op == 22) { rx = exp(rx); ip = rx; }
  else if (op == 23) rx = sin(rx * 3.1415 / 180.0); else if (op == 24) rx = cos(rx * 3.1415 / 180.0); 
  else if (op == 25) { if(cos(rx) != 0) rx = tan(rx * 3.1415 / 180.0); }
  else if (op == 26) rx = sinh(rx); else if (op == 27) rx = cosh(rx); else if (op == 28) rx = tanh(rx);
  else if (op == 15) doEqual();
}
void ui() {
  lcd.clear();
  if (md == 0) {
    if(h < 10) lcd.print("0"); lcd.print(h); lcd.print(s % 2 == 0 ? ":" : " "); if(m < 10) lcd.print("0"); lcd.print(m);
    lcd.setCursor(8, 0); lcd.print("HUM: "); lcd.print(u); lcd.print("%"); lcd.setCursor(0, 1); lcd.print("Radio: "); lcd.print(f, 1); lcd.print("MHz");
  } else if (md == 1) { 
    lcd.print("MODE: FM RADIO"); lcd.setCursor(0, 1); lcd.print("F: "); lcd.print(f, 1); lcd.print("MHz S:"); lcd.print(getS()); 
  } else if (md == 2) { 
    lcd.print("T:"); lcd.print((int)t); lcd.print("C ["); for(int i = 0; i < 4; i++) lcd.write(getB(th[i], 10, 40)); 
    lcd.print("]"); lcd.setCursor(0, 1); lcd.print("HUMIDITY: "); lcd.print(u); lcd.print("%");
  } else if (md == 3) {
    if(gm == 0) { if(pm) { lcd.print("MK52 [PRG] Sh:"); lcd.print(id); } else { lcd.print("MK-52 CALC"); } } else lcd.print("G:LUNALET");
    lcd.setCursor(0, 1); lcd.print("[");
    if (ci <= 9) lcd.print(ci); 
    else if (ci >= 10 && ci <= 28) lcd.print(ops[ci - 10]); lcd.print("]");
    if(gm == 1) { lcd.setCursor(6, 1); lcd.print("V:"); lcd.print((int)rx); lcd.setCursor(11, 1); lcd.print("H:"); lcd.print((int)rg); } else { lcd.setCursor(8, 1); lcd.print("X:"); lcd.print(rx, 2); }
  }
}
void setup() {
  Wire.begin(); dht.begin(); randomSeed(analogRead(0));
  pinMode(2, INPUT_PULLUP); pinMode(3, INPUT_PULLUP); pinMode(8, INPUT_PULLUP); for(int i = 4; i <= 6; i++) pinMode(i, OUTPUT);
  lcd.init(); lcd.backlight(); for(int i = 0; i < 8; i++) lcd.createChar(i, bc[i]);
  lcd.clear(); lcd.setCursor(2, 0); lcd.print("HELLO WORLD!"); lcd.setCursor(3, 1); lcd.print("DIY SYSTEM");
  rgb(0, 0, 255); delay(10000); rgb(255, 255, 255); snd(0x00, 0x80);
  EEPROM.get(10, f); if(f < F_MIN || f > F_MAX || isnan(f)) { f = 101.2; EEPROM.put(10, f); } readRTC(); tc = millis(); ta = millis(); ui();
}
void loop() {
  if (millis() - tc >= 1000) {
    tc += 1000; readRTC();
    if(s == 0) { if(m % 15 == 0) addH((int)t, th); if(h == 7 && m == 0) { md = 1; snd(0xB0, 0x10); chk(); } if(gm == 1) { rg -= rx; rx += 1; } }
    if(md == 0 && (millis() - ta >= 120000)) { md = 4; bx = 7; by = 0; dx = 1; dy = 1; rgb(0, 0, 255); } if(md != 3 && md != 4) ui();
  }
  if (md != 4 && millis() - td >= 5000) { td = millis(); t = dht.readTemperature(); u = dht.readHumidity(); if(!isnan(t) && th == 0) for(int i = 0; i < 4; i++) th[i] = (int)t; if(md != 3) ui(); }
  if (digitalRead(2) == LOW || digitalRead(3) == LOW || digitalRead(8) == LOW) if(md == 0) ta = millis();
  if (md == 0) {
    if(digitalRead(2) == LOW) { md = 1; snd(0xB0, 0x10); chk(); ui(); delay(300); }
    if(digitalRead(3) == LOW) { md = 3; gm = 0; rx = ry = ip = ci = 0; dc = 0.0; clr_x = false; last_op = 0; rgb(255, 0, 255); ui(); delay(300); }
    if(digitalRead(8) == LOW) { md = 2; snd(0x00, 0x80); rgb(0, 255, 255); ui(); delay(300); }
  }
  else if (md == 1) {
    if(digitalRead(2) == LOW && digitalRead(3) == LOW) { autoScan(); ui(); delay(500); return; }
    if(digitalRead(8) == LOW) { if(b3 == 0) b3 = millis(); if(millis() - b3 > 1500) { md = 0; snd(0x00, 0x80); rgb(255, 255, 255); ui(); b3 = 0; delay(500); } }
    if(digitalRead(2) == LOW) { f += 0.1; if(f > F_MAX) f = F_MIN; snd(0xB0, 0x10); chk(); ui(); delay(200); }
    if(digitalRead(3) == LOW) { f -= 0.1; if(f < F_MIN) f = F_MAX; snd(0xB0, 0x10); chk(); ui(); delay(200); }
  }
  else if (md == 2) { if(digitalRead(8) == LOW) { if(b3 == 0) b3 = millis(); if(millis() - b3 > 1500) { md = 0; rgb(255, 255, 255); ui(); b3 = 0; delay(500); } } else b3 = 0; }
  else if (md == 3) {
    if(digitalRead(2) == LOW) { if(bu == 0) bu = millis(); if(millis() - bu > 1500) { pm = false; id = 0; rgb(255, 0, 255); ui(); bu = 0; delay(500); return; } } 
    else { if(bu > 0) { if(millis() - bu < 1500) { ci = (ci + 1) % 29; ui(); } bu = 0; delay(150); } }
    if(digitalRead(3) == LOW) { if(bd == 0) bd = millis(); if(millis() - bd >= 1500 && millis() - bd < 4000) { ci = (ci == 0) ? 28 : ci - 1; ui(); delay(250); bd = millis(); } }
    else { if(bd > 0) { if(millis() - bd < 1500) { calc(ci); ui(); } bd = 0; delay(150); } }
    if(digitalRead(8) == LOW) { if(b3 == 0) b3 = millis(); if(millis() - b3 > 1500) { md = 0; rgb(255, 255, 255); ui(); b3 = 0; delay(500); return; } } 
    else { if(b3 > 0) { if(millis() - b3 < 1500) { rx = (rx >= 0) ? sqrt(rx) : rx; ip = rx; ui(); } else { rx = pow(ry, rx); ip = rx; ui(); } b3 = 0; delay(150); } }
  }
  else if (md == 4) {
    if (millis() - tg >= 250) { 
      tg = millis(); bx += dx; by += dy; if(by < 0){by = 0; dy = -dy;} if(by > 1){by = 1; dy = -dy;} if(bx == 1 && by == pl){dx = -dx; rgb(0, 255, 0);} if(bx == 14 && by == pr){dx = -dx; rgb(0, 255, 0);}
      if(bx < 0 || bx > 15){bx = 7; by = 0; dx = -dx; rgb(255, 0, 0); delay(300);} lcd.clear(); lcd.setCursor(1, pl); lcd.print("|"); lcd.setCursor(14, pr); lcd.print("|"); lcd.setCursor(bx, by); lcd.print("o"); 
    }
    if(digitalRead(8) == LOW) { if(b3 == 0) b3 = millis(); if(millis() - b3 > 1500) { md = 0; ta = millis(); rgb(255, 255, 255); ui(); b3 = 0; delay(500); } }
    if(digitalRead(2) == LOW) { pl = (pl + 1) % 2; delay(150); } if(digitalRead(3) == LOW) { pr = (pr + 1) % 2; delay(150); }
  }
}
