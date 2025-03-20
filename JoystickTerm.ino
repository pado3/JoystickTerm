/*
  JoystickTerm.ino
  AV controller via Wi-Fi and BLE with Arduino and ESP32-WROOM-32E
  copyright (c) by @pado3@mstdn.jp
  r3.0 2025/03/20 use SPIFFS to store CAL value, and some minor update
  r2.0 2025/03/16 implement light sleep, add F5(reload) on opt+space
  r1.0 2025/03/13 initial release

  Note:
  1. Depends on the version of ESP32 board library on Arduino IDE.
      I recommend v2.0.17.
  2. The BLE Keyboard & Mouse Combo library is distributed here:
      https://github.com/peter-pakanun/ESP32-BLE-Combo
      (care: typo in line 15 of the example at readme.md)
  3. The following network parameters are described in private.h:
      WIFI_SSID, WIFI_PASS, HOST_IP, APP_PATH
  4. Memory shortage occurs with default partition scheme (1.2MB APP).
      You need at least 2MB APP.
*/

#include "private.h"
#include <SPIFFS.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <BleCombo.h>

BleCombo bleCombo;

// global and notable parameters
String THIS_PROGRAM = "\n===== start JoystickTerm r3.0 by @pado3@mstdn.jp 2025/03/20 =====";
uint32_t WDT_TIMEOUT_NORMAL = 15000;  // watchdog for normal condition, 15sec in msec
uint32_t WDT_TIMEOUT_CONNECT = 1000;  // watchdog for connection, 1sec in msec
uint32_t WDT_TIMEOUT_FORMAT = 60000;  // watchdog for format SPIFFS, 60sec in msec
uint32_t WDT_TIMEOUT = WDT_TIMEOUT_NORMAL;
hw_timer_t *WD_timer = NULL;
uint32_t SLEEP_TIMEOUT = 120000;      // 2min in msec
uint32_t CONN_TIMEOUT = 15000;        // 15sec in msec
uint32_t SLEEP_timer = 0;   // msec
uint16_t LOOP_COUNT = 0;    // loop() counter: max. SLEEP_TIMEOUT*REFRESH_RATE/1000
uint8_t REFRESH_RATE = 20;  // Hz, too fast(~30), cannot move cursor
uint8_t WAIT_KEY = 500;     // prevent key chattering, msec
uint8_t TOUCH_MARGIN = 2;   // Touch sensor CAL value to threshold margin
String TV_VOL_UP = "1A";    // IR remocon code of TV, use with web app
String TV_VOL_DOWN = "1E";

/* pin configuration */
// activity indicator (green LED)
uint8_t act = 12;     // IO12 of ESP32
// WiFi control
uint8_t rp_pwr = 15;  // IO15 of ESP32, code 1
uint8_t tv_pwr = 16;  // IO16 of ESP32, code 2
uint8_t tv_src = 17;  // IO17 of ESP32, code 3
uint8_t sp_src = 5;   // IO5  of ESP32, code 4
// BLE macro
uint8_t firefox = 4;  // IO4  of ESP32, code 11
// BLE control
uint8_t click = 14;   // IO14 of ESP32, code 12
uint8_t reverse = 27; // IO27 of ESP32, code 13
uint8_t forward = 26; // IO26 of ESP32, code 14
uint8_t space = 25;   // IO25 of ESP32, code 15
uint8_t escape = 0;   // IO0 of ESP32, BOOT for programing, code 16
uint8_t vol_up = 22;  // IO32 of ESP32, code 17
uint8_t vol_dn = 32;  // IO22 of ESP32, code 18
// option key
uint8_t option = 23;  // IO23 of ESP32, no key code (scroll, option)
// touch sensor
uint8_t touch = T2;   // IO2 of ESP32 for touch detection
// ADC input
uint8_t js_x = 39;    // IO39 of ESP32, SVN
uint8_t js_y = 36;    // IO36 of ESP32, SVP
uint8_t tp_v = 34;    // IO34 of ESP32, TP_VBATT = VBATT/2
float x_cal = 0.0;    // average central value of X
float y_cal = 0.0;    // average central value of Y
float v_batt = 0.0;   // battery voltage (V)
float batt_max = 4.2; // maximum battery voltage
float batt_min = 3.3; // minimum battery voltage, ESP3.0min + reg0.3max

void setup() {
  bleCombo.setName("JoystickTerm");
  // setup Watch Dog Timer
  WD_timer = timerBegin(0, 80, true); // timer0, 80MHz/80=1MHz, div:uint16_t
  timerAttachInterrupt(WD_timer, &resetModule, true); // attach callback
  timerAlarmWrite(WD_timer, WDT_TIMEOUT*1000, false); // set time in usec
  timerAlarmEnable(WD_timer);                         // enable interrupt
  // setup I/O pins
  pinMode(act, OUTPUT);           // activity LED
  pinMode(rp_pwr, INPUT_PULLUP);  // RPi5 power SW
  pinMode(tv_pwr, INPUT_PULLUP);  // TV power SW
  pinMode(tv_src, INPUT_PULLUP);  // TV source selecter
  pinMode(sp_src, INPUT_PULLUP);  // Soundbar source selector
  pinMode(firefox, INPUT_PULLUP); // launch Firefox (macro)
  pinMode(click, INPUT_PULLUP);   // mouse click
  pinMode(vol_up, INPUT_PULLUP);  // volume up
  pinMode(vol_dn, INPUT_PULLUP);  // volume down
  pinMode(escape, INPUT_PULLUP);  // ESC (exit fullscreen)
  pinMode(space, INPUT_PULLUP);   // space (pause)
  pinMode(reverse, INPUT_PULLUP); // left arrow
  pinMode(forward, INPUT_PULLUP); // right arrow
  pinMode(option, INPUT_PULLUP);  // option key (ex. click to right)
  // setup ADC for all channels
  analogReadResolution(10);       // 10bit 1024step
  analogSetAttenuation(ADC_11db); // set input range 150~3100 mV
  // peripheral startup routine
  Serial.begin(115200);
  Serial.println(THIS_PROGRAM);
  // Serial.println("START " __FILE__ " from " __DATE__ " " __TIME__);
  joystick_cal();   // 1sec calibration
  scan_wifi();      // scan first for fast connection
  SLEEP_timer = millis(); // initialize timer 
}

void loop() {
  timerWrite(WD_timer, 0);  // reset WDT (feed watchdog)
  move_cursor();
  uint8_t cmd = get_command();
  if (1 <= cmd && cmd <= 4) { // command via Wi-Fi
    String CMD = String(cmd, HEX);  // compatible TV remote command
    send_wifi_command(CMD);
    delay(WAIT_KEY);
    SLEEP_timer = millis(); // reset SLEEP timer
    LOOP_COUNT = 0;
  } else if (11 <= cmd && cmd <= 18) {  // command via BLE
    send_ble_command(cmd);
    delay(WAIT_KEY);
    SLEEP_timer = millis(); // reset SLEEP timer
    LOOP_COUNT = 0;
  }
  if ((millis() - SLEEP_timer) > SLEEP_TIMEOUT) {
    Serial.println(F("SLEEP Timeout occurs. "));
    disconnect_wifi();
    light_sleep();
    LOOP_COUNT = 0;
  }
  delay(int(1000/REFRESH_RATE));
  if (LOOP_COUNT % 100 == 0) {
    if (LOOP_COUNT % 1000 == 0) {
      Serial.println();
    }
    Serial.print(LOOP_COUNT);
  } else if (LOOP_COUNT % REFRESH_RATE == 0) {
    Serial.print(F("."));
  }
  LOOP_COUNT++;
}

// WDT handler
void ARDUINO_ISR_ATTR resetModule() {
  ESP.restart();
}

void light_sleep() {
  // calibrate touch sensor
  Serial.print(F("calibrate touch sensor before sleep... "));
  uint8_t i = 0;
  uint8_t count = 10;
  uint16_t t_cal = 0;
  for (i = 0; i < count; i++) {
    t_cal += touchRead(touch);
    delay(10);
  }
  t_cal = int(t_cal / count) - TOUCH_MARGIN;
  Serial.print(F("your threshold is: "));
  Serial.print(t_cal);
  Serial.println(F(", Wait for touch Zzz."));
  Serial.flush();
  for (i = 0; i < 3; i++) {
    digitalWrite(act, HIGH);
    delay(50);
    digitalWrite(act, LOW);
    delay(50);
  }
  touchSleepWakeUpEnable(touch, t_cal);
  esp_light_sleep_start();
  Serial.println(F("Wake up!"));
  SLEEP_timer = millis(); // reset SLEEP timer
  for (i = 0; i < 3; i++) {
    digitalWrite(act, HIGH);
    delay(50);
    digitalWrite(act, LOW);
    delay(50);
  }
}

void joystick_cal() {
  uint8_t count = 100;
  digitalWrite(act, HIGH);  // work as code starting indicator
  // use SPIFFS to store CAL data
  WDT_TIMEOUT = WDT_TIMEOUT_FORMAT;
  timerWrite(WD_timer, 0);  // reset WDT
  timerAlarmWrite(WD_timer, WDT_TIMEOUT*1000, false); //set time in usec
  SPIFFS.begin(true); // Format filesystem at the first time, need 40sec
  WDT_TIMEOUT = WDT_TIMEOUT_NORMAL;
  timerWrite(WD_timer, 0);  // reset WDT
  timerAlarmWrite(WD_timer, WDT_TIMEOUT*1000, false); //set time in usec
  File fr = SPIFFS.open("/js.dat"); // read first
  if (fr && !option_value()) {  // file exist and option is not pressed
    Serial.print(F("read CAL data from file. (x_cal*"));
    Serial.print(count);
    Serial.print(F(": "));
    char c;
    String buf = "";
    while(fr.available()) {
      c = fr.read();
      if (c != '\n') {
        buf += String(c);
      } else {
        x_cal = buf.toFloat();
        Serial.print(int(x_cal));
        buf = "";
      }
    }
    y_cal = buf.toFloat();
    Serial.print(F(", y_cal*"));
    Serial.print(count);
    Serial.print(F(": "));
    Serial.print(int(y_cal));
    Serial.println(F(")"));
    fr.close();
  } else {  // file not exist or option is pressed
    File fw = SPIFFS.open("/js.dat", FILE_WRITE); // clear the file
    Serial.println(F("CAL file is not exist or option key pressed."));
    // proceed calibration
    Serial.print(F("calibrating joystick"));
    for (uint8_t i = 0; i < count; i++) {
      x_cal += float(analogRead(js_x)); // analogRead() 0-1023
      delay(1000/(2*count));
      y_cal += float(analogRead(js_y));
      delay(1000/(2*count));
      if (i % 5 == 0) {
        Serial.print(F("."));
      }
    }
    // store CAL data
    Serial.print(F("\nwrite CAL data to file. (x_cal*"));
    Serial.print(count);
    Serial.print(F(": "));
    Serial.print(int(x_cal));
    Serial.print(F(", y_cal*"));
    Serial.print(count);
    Serial.print(F(": "));
    Serial.print(int(y_cal));
    Serial.println(F(")"));
    fw.println(String(int(x_cal)));
    fw.print(String(int(y_cal)));
    fw.close();
  }
  x_cal /= count;
  y_cal /= count;
  // tp_v = VBATT/2, direct read the calibrated value
  v_batt = float(analogReadMilliVolts(tp_v))*2/1000;  // global
  digitalWrite(act, LOW);
  Serial.print(F("read or proceed CAL completed. x_cal: "));
  Serial.print(x_cal);
  Serial.print(F(" y_cal: "));
  Serial.print(y_cal);
  Serial.print(F(", battery voltage: "));
  Serial.print(v_batt);
  Serial.println(F("V"));
}

void reboot() {
  Serial.print(F("\nreboot now"));
  for(int8_t i = 0; i < 3; i++) {
    Serial.print(F("."));
  digitalWrite(act, HIGH);
    delay(50);
    digitalWrite(act, LOW);
    delay(50);
  }
  ESP.restart();
}

void scan_wifi() {
  digitalWrite(act, HIGH);
  Serial.print(F("Scanning Wi-Fi... "));
  //  int16_t scanNetworks(
  //    bool async = false, 
  //    bool show_hidden = false, 
  //    bool passive = false, 
  //    uint32_t max_ms_per_chan = 300, // shorten for home-use & fast scan
  //    uint8_t channel = 0,
  //    const char *ssid = nullptr,
  //    const uint8_t *bssid = nullptr
  //  );
  int16_t nSSID = WiFi.scanNetworks(false, false, false, 50);
  WiFi.disconnect(true);   // turn the Wi-Fi radio off.
  bool scan_flag = false;
  digitalWrite(act, LOW);
  if (nSSID == 0) {
    Serial.println(F("no AP found, but continue"));
  } else {
    Serial.print(nSSID);
    Serial.print(F(" APs found"));
    for (int n = 0; n < nSSID; n++) {
        // Print SSID and RSSI for each network found
        Serial.println();
        Serial.print(n + 1);
        Serial.print(F(": "));
        Serial.print(WiFi.SSID(n));
        Serial.print(F(" ("));
        Serial.print(WiFi.RSSI(n));
        Serial.print(F("dBm)"));
        Serial.print((WiFi.encryptionType(n) == WIFI_AUTH_OPEN)?" ":"*");
        if (WiFi.SSID(n) == WIFI_SSID) {
          Serial.print(F("<----- your AP"));
          scan_flag = true;
        }
    }
    if (scan_flag != true) {
      Serial.println(F("cannot found your AP, but continue"));
    }
  }
}

bool connect_wifi() {
  if (WiFi.status() == WL_CONNECTED) { return true; }
  Serial.print(F("\nstart & connecting Wi-Fi"));
  // sometimes freeze in connection, so WDT make short
  WDT_TIMEOUT = WDT_TIMEOUT_CONNECT;
  timerWrite(WD_timer, 0);  // reset WDT
  timerAlarmWrite(WD_timer, WDT_TIMEOUT*1000, false); //set time in usec
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(F("."));
    digitalWrite(act, HIGH);
    delay(50);
    digitalWrite(act, LOW);
    delay(50);
    i++;
    // over CONN_TIMEOUT(e.g. host not respond), reboot & scan
    if (i > CONN_TIMEOUT/100) { reboot(); }
    timerWrite(WD_timer, 0);  // reset WDT
  }
  // WDT recover to normal condition
  WDT_TIMEOUT = WDT_TIMEOUT_NORMAL;
  timerWrite(WD_timer, 0);  // reset WDT
  timerAlarmWrite(WD_timer, WDT_TIMEOUT*1000, false); //set time in usec
  Serial.print(F("CONNECTED! IP address: "));
  Serial.println(WiFi.localIP());
  return true;
}

void disconnect_wifi() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("disconnecting Wi-Fi"));
    WiFi.disconnect(true);   // turn the Wi-Fi radio off. Error occures if BLE is on
  }
}

void send_wifi_command(String CMD) {
  if(connect_wifi()) {
    String url = String("http://") + HOST_IP + APP_PATH + CMD;
    Serial.print(F("\nAccess to "));
    Serial.println(url);
    HTTPClient http;
    Serial.print(F("request... "));
    http.begin(url);
    Serial.print(F("get "));
    // start connection and send HTTP header
    int httpCode = http.GET();
    // httpCode will be negative on error
    if(httpCode > 0) {
      String line = http.getString();
      Serial.println(line);
    } else {
      Serial.print(F("failed with this error: "));
      Serial.printf("%s\n", http.errorToString(httpCode).c_str());
      reboot();
    }
    http.end();
  }
}

bool connect_ble() {
  uint32_t i = 0;
  if (bleCombo.isConnected() == true) { return true; }
  Serial.print(F("\nstart & connecting BLE"));
  // sometimes freeze in connection, so WDT make short
  WDT_TIMEOUT = WDT_TIMEOUT_CONNECT;
  timerWrite(WD_timer, 0);  // reset WDT
  timerAlarmWrite(WD_timer, WDT_TIMEOUT*1000, false); //set time in usec
  bleCombo.begin();
  while (bleCombo.isConnected() == false) {
    Serial.print(F("."));
    digitalWrite(act, HIGH);
    delay(50);
    digitalWrite(act, LOW);
    delay(50);
    i++;
    // over CONN_TIMEOUT(e.g. host not respond), reboot
    if (i > CONN_TIMEOUT/100) { reboot(); }
    timerWrite(WD_timer, 0);  // reset WDT
  }
  // WDT recover to normal condition
  WDT_TIMEOUT = WDT_TIMEOUT_NORMAL;
  timerWrite(WD_timer, 0);  // reset WDT
  timerAlarmWrite(WD_timer, WDT_TIMEOUT*1000, false); //set time in usec
  Serial.print(F("\nBLE CONNECTED."));
  return true;
}

/* do not use
void disconnect_ble() {
  Serial.println(F("\ndisconnecting BLE"));
  bleCombo.end();  // this cannot power off the BLE, so reboot is needed
  reboot();
}
*/

uint8_t get_command() {
  if (!digitalRead(rp_pwr)){
    Serial.print(F("\nrp_pwr key pressed "));  return 1;
  } else if (!digitalRead(tv_pwr)){
    Serial.print(F("\ntv_pwr key pressed "));  return 2;
  } else if (!digitalRead(tv_src)){
    Serial.print(F("\ntv_src key pressed "));  return 3;
  } else if (!digitalRead(sp_src)){
    Serial.print(F("\nsp_src key pressed "));  return 4;
  } else if (!digitalRead(firefox)){
    Serial.print(F("\nfirefox key pressed ")); return 11;
  } else if (!digitalRead(click)){
    Serial.print(F("\nclick key pressed "));   return 12;
  } else if (!digitalRead(vol_up)){
    Serial.print(F("\nvol_up key pressed "));  return 13;
  } else if (!digitalRead(vol_dn)){
    Serial.print(F("\nvol_dn key pressed "));  return 14;
  } else if (!digitalRead(escape)){
    Serial.print(F("\nescape key pressed "));  return 15;
  } else if (!digitalRead(space)){
    Serial.print(F("\nspace key pressed "));   return 16;
  } else if (!digitalRead(reverse)){
    Serial.print(F("\nreverse key pressed ")); return 17;
  } else if (!digitalRead(forward)){
    Serial.print(F("\nforward key pressed ")); return 18;
  } else {
                                                return 0;
  }
}

bool option_value() {
  // pressed: LOW : return true
  if (!digitalRead(option)) {
    Serial.print(F("+Option "));
    return true;
  }
  return false;
}

void move_cursor() {
  // read all ADC related values
  // x: approx. +/-300, ADC linear region
  float x_value = x_cal - float(analogRead(js_x));
  // y: display coordinate is top to bottom, then reverse direction
  float y_value = float(analogRead(js_y)) - y_cal;
  // tp_v = VBATT/2, direct read the calibrated value
  v_batt = float(analogReadMilliVolts(tp_v))*2/1000;  // global
  // move (value^3) to 50pix max, 300^3=27e6, then /54e4
  signed char x_move = int(pow(x_value, 3.0) /54e4);
  signed char y_move = int(pow(y_value, 3.0) /54e4);
  if (abs(x_move) > 0 || abs(y_move)>0) {
    if (connect_ble()) {
      bool wheel = option_value();
      SLEEP_timer = millis(); // reset SLEEP timer
      Serial.print(F("\n x_move:"));
      Serial.print(x_move);
      Serial.print(F(" y_move:"));
      Serial.print(y_move);
      Serial.print(F(" OPTION:"));
      Serial.print(wheel);
      Serial.print(F(" VBATT:"));
      Serial.print(v_batt);
      Serial.println(F("V"));
      // if option is pressed, wheel is true = active
      if (wheel) {
        // move: x, y, v-wheel, h-wheel. wheel direction is my preference
        bleCombo.move(0, 0, -y_move/25, -x_move/25);  // wheel 4 step
      } else {
        bleCombo.move(x_move, y_move);
      }
    } else {
      Serial.print(F("NC "));
    }
  }
}

void send_ble_command(uint8_t cmd) {
  SLEEP_timer = millis(); // reset SLEEP timer
  if(connect_ble()) {
    bool opt = option_value();
    switch (cmd) {
      case 11:  // Firefox macro
        bleCombo.write(KEY_LEFT_GUI);
        delay(300);
        bleCombo.write(KEY_DOWN_ARROW);
        delay(500);
        bleCombo.write(KEY_DOWN_ARROW);
        delay(500);
        bleCombo.write(KEY_RIGHT_ARROW);
        delay(500);
        bleCombo.write(KEY_DOWN_ARROW);
        delay(500);
        bleCombo.write(KEY_RETURN);
        break;
      case 12:
        if(opt) {
          bleCombo.click(MOUSE_RIGHT);
        } else {
          bleCombo.click(MOUSE_LEFT);
        }
        break;
      case 13:  // vol_up
        if(opt) {
          send_wifi_command(TV_VOL_UP);
        } else {
          bleCombo.write(KEY_MEDIA_VOLUME_UP);
        }
        break;
      case 14:  // vol_dn
        if(opt) {
          send_wifi_command(TV_VOL_DOWN);
        } else {
          bleCombo.write(KEY_MEDIA_VOLUME_DOWN);
        }
        break;
      case 15:  // escape(command+w)
        if(opt) {
          // bleCombo.print("f");  // full screen with RugbyPass TV
          bleCombo.press(KEY_LEFT_CTRL); // Close TAB
          bleCombo.print("w");
          delay(100);
          bleCombo.releaseAll();
        } else {
          bleCombo.write(KEY_ESC);
        }
        break;
      case 16:  // space(reload)
        if(opt) {
          bleCombo.write(KEY_F5);
        } else {
          bleCombo.print(" ");
        }
        break;
      case 17:  // reverse(J)
        if(opt) {
          bleCombo.write(KEY_MEDIA_WWW_BACK); // browser BACK(previous page)
        } else {
          bleCombo.write(KEY_LEFT_ARROW);
        }
        break;
      case 18:  // forward(L)
        if(opt) {
          bleCombo.print("l");  // +10sec with RugbyPass TV
        } else {
          bleCombo.write(KEY_RIGHT_ARROW);
        }
        break;
    }
    uint8_t batt_percent = int(100*(v_batt - batt_min)/(batt_max - batt_min));
    // note: in DEBUG, v_batt~5.2V. 
    //        then 100*(5.2-3.4)/(4.2-3.4)=225 < 255. 255=5.44V, this is unrealistic.
    if (batt_percent < 0) {batt_percent = 0;}
    if (100 < batt_percent) {batt_percent = 100;}
    Serial.print(F(", battery level: "));
    Serial.print(batt_percent);
    Serial.println(F("%"));
    bleCombo.setBatteryLevel(batt_percent);
    if (batt_percent < 20) {
      Serial.println(F("main battery low"));
      uint8_t i;
      for (i=0; i<10; i++) {
        digitalWrite(act, HIGH);
        delay(200);
        digitalWrite(act, LOW);
        delay(200);
      }
    }
  }
}
