#include <Arduino.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

/* LCD */
#define CUR_SCREEN 0
#define EXPECTED_SCREEN 1
//#define REFRESH_RATE_IN_MS 100
//#define FULL_STOP_Delay_MS 1000

/* KEYPAD */
#define RESET_KEYA 'A'
#define RESET_KEYB 'B'
#define STOP_KEY 'D'
#define START_KEY 'C'
#define DP_KEY '*'

/* ARDUINO PIN */
// #define RELAY1_PIN A2
// #define RELAY2_PIN A1
#define RELAY1_PIN 11
#define RELAY2_PIN 12
#define LED_START 10
#define LED_STOP 11
#define BUZZ 12
#define rxPin 13
#define txPin A0
#define TRIGGER_LEVEL LOW

/* STATE */
#define STOP_STATE 0
#define RUN_STATE 1

byte key = NO_KEY;
const byte ROWS = 4; // four rows
const byte COLS = 4; // three columns
char keys[ROWS][COLS] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
        };
byte rowPins[ROWS] = {9, 8, 7, 6}; // connect to the row pinouts of the keypad
byte colPins[COLS] = {5, 4, 3, 2}; // connect to the column pinouts of the keypad

Keypad my_keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

enum Mode
{
  MODE_A = 0,
  MODE_B
};
struct device_params
{
  float cur_weight = 0;
  float expected_weight = 0;
  int state_run_stop = STOP_STATE;
  Mode mode = MODE_A;
} DeviceParams;

int num_dp = 2;
bool input_dp = false;
float input_a = 10;
float input_b = 1;
float weight[3] = {1, 2, 3};
uint8_t count_get_weight = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial mySerial (rxPin, txPin);

long last_show_cur_weight = 0;
/********** Buffer using to read data from scale **********/
#define MAX_LEN 100
char msg[MAX_LEN];
int msg_len = 0;

/********** Display running mode **********/
void show_mode() {
  // lcd.setCursor(10,EXPECTED_SCREEN);
  
  switch (EEPROM.read(5)) {
  case 'A':
  {
    lcd.setCursor(0, 0);
    lcd.print("A-BOM");
    lcd.setCursor(0, 1);
    lcd.print("     ");
    break;
  }
  case 'B':
  {
    lcd.setCursor(0, 0);
    lcd.print("     ");
    lcd.setCursor(0, 1);
    lcd.print("B-HUT ");
    break;
  }
  default:
  {
    lcd.setCursor(0, 0);
    lcd.print("CHUA CHON");
    break;
  }
  }
}
/********** Reset status device when press key A **********/
void reset_state_in() {
  DeviceParams.expected_weight = 0;
  input_dp = false;
  input_a = 10;
  input_b = 1;
  DeviceParams.mode = MODE_A;
}

/********** Reset status device when press key B **********/
void reset_state_out() {
  DeviceParams.expected_weight = 0;
  input_dp = false;
  input_a = 10;
  input_b = 1;
  DeviceParams.mode = MODE_B;
}

/********** Turn on Relay **********/
void onRelay() {
    digitalWrite(RELAY1_PIN, TRIGGER_LEVEL);
    digitalWrite(RELAY2_PIN, TRIGGER_LEVEL);
    Serial.println("ON RELAY");
}

/********** Turn off relay **********/
void offRelay() {
    digitalWrite(RELAY1_PIN, HIGH);
    delay(500);
    digitalWrite(RELAY2_PIN, HIGH);
    Serial.println("OFF RELAY");
}

/********** Convert float number to String and show **********/
void show_float(int addr, float x) {
  String s = String(x, num_dp);
  char cs[15];
  while (s.length() > 8) {
    s.remove(0, 1);
  }
  sprintf(cs, "%9s kg", s.c_str());
  lcd.setCursor(6, addr);
  lcd.print(cs);
  if (addr == CUR_SCREEN) {
    last_show_cur_weight = millis();
  }
}

/********** Get data and parse from Ampcells Scale Indicator **********/
bool update_cur_weight() {
  char c;
  bool new_msg = false;
  //Read string from scale indicator
  while (mySerial.available()) {
    c = mySerial.read();

    if (c == 10 || c == 13) {
      new_msg = true;
      msg[msg_len] = 0;
      break;
    }
    msg[msg_len] = c;
    msg_len += 1;
    if (msg_len >= MAX_LEN) {
      msg_len = 0;
    }
  }

  if (new_msg) {
    if (msg_len >= 17) {
      const int NEG_POS = 6;
      const int NUM_LEN = 7;
      bool neg = msg[NEG_POS] == '-';
      char *num_s = &msg[NEG_POS + 1];
      num_s[NEG_POS + NUM_LEN + 1] = 0;
      weight[count_get_weight] = atof(num_s);
      count_get_weight += 1;

      if (count_get_weight == 3) {
        if (weight[0] == weight[1] && weight[1] == weight[2]) {
          DeviceParams.cur_weight = weight[2];
        }
        if (neg) {
          DeviceParams.cur_weight = -DeviceParams.cur_weight;
        }
        count_get_weight = 0;
        msg_len = 0;
        return true;
      }
    }
  }
  // "ST,NT,    0.00 kg"
  // "US,NT,   15.10 kg"
  if ((msg_len == 3 || msg_len == 6) && (msg[msg_len - 1] != ',')) {
    // Serial.println("wrong msg detected");
    msg_len = 0;
  }
  if ((msg_len == 16) && (msg[msg_len - 1] != 'k')) {
    // Serial.println("wrong msg detected");
    msg_len = 0;
  }
  if ((msg_len == 17) && (msg[msg_len - 1] != 'g')) {
    // Serial.println("wrong msg detected");
    msg_len = 0;
  }
  return false;
}

/********** Stop device **********/
void force_stop() {
  offRelay();
  if (DeviceParams.state_run_stop == RUN_STATE) { // get_time();
    DeviceParams.state_run_stop = STOP_STATE;
  
    Serial.print("KL send in time stop task loop:");
    Serial.println(DeviceParams.cur_weight);
  }
}

/********** Check if scale get the expected weight **********/
bool check_cond_stop() {
  if (DeviceParams.mode == MODE_A) { // bom vao
    if (DeviceParams.cur_weight >= DeviceParams.expected_weight) {
      return true;
    }
  } else { // hut ra
    if (DeviceParams.cur_weight <= DeviceParams.expected_weight + 0.4) {
      return true;
    }
  }
  return false;
}

/********** Run device **********/
void start_run() {
  if (check_cond_stop()) {
    return;
  }
  if (DeviceParams.state_run_stop == STOP_STATE) {
    DeviceParams.state_run_stop = RUN_STATE;
    // Update weight to EEPROM
    if (DeviceParams.expected_weight != EEPROM.read(0)) {
      EEPROM.write(0, DeviceParams.expected_weight);
    }
  }
  onRelay();
}

/********** Get button **********/
bool update_key(char key) {
  //Press 'D' on keypad and state is running
  if ((key == STOP_KEY) && (DeviceParams.state_run_stop == RUN_STATE)) {
    force_stop();
    return true;
  }
  //Press 'A' to choose mode A: "BOM"
  if (key == RESET_KEYA) {
    force_stop();
    reset_state_in();
    EEPROM.write(5, RESET_KEYA);
    show_mode();
    return true;
  }
  //Press 'B' to choose mode B: "HUT"
  if (key == RESET_KEYB) {
    force_stop();
    reset_state_out();
    EEPROM.write(5, RESET_KEYB);
    show_mode();
    return true;
  }
  //When running, device ignore the entering number. Stop it first
  if (DeviceParams.state_run_stop == RUN_STATE) {
    return false;
  }
  //Press 'C' to start
  if (key == START_KEY) {
    start_run();
    return true;
  }
  //Press '*' to enter '.'
  if (key == DP_KEY) {
    input_dp = true;
    input_a = 1;
    input_b = 0.1;
    return true;
  }
  //Press '#': reserve
  
  //Entering number
  if (isDigit(key)) {
    //Allow input decimal value
    DeviceParams.expected_weight = DeviceParams.expected_weight * input_a + (key - '0') * input_b;
    if (input_dp) {
      input_b = input_b / 10;
    }
    if (DeviceParams.expected_weight > 99999.99) {
      DeviceParams.expected_weight = (int)DeviceParams.expected_weight % 100000;
    }
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  lcd.init();
  lcd.backlight();
  show_mode();
}

void loop() {
  
  // Update weight and show on LCD
  if (update_cur_weight()) {
    if (millis() - last_show_cur_weight > 150) {
      show_float(CUR_SCREEN, DeviceParams.cur_weight);
    }
  }
  
  // Read value from keypad
  key = my_keypad.getKey();

  if (key != NO_KEY) {
    if (update_key(key)) {
      show_float(EXPECTED_SCREEN, DeviceParams.expected_weight);
    }
  }

  // Check if the scale get the expected weight
  if (DeviceParams.state_run_stop == RUN_STATE) {
    if (check_cond_stop()) {
      force_stop();
    }
  }
}