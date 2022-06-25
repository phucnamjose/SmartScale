#include <Arduino.h>

#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define CUR_SCREEN 0
#define EXPECTED_SCREEN 1
#define REFRESH_RATE_IN_MS 100
#define FULL_STOP_Delay_MS 1000
#define SEND_START 1
#define SEND_STOP 2
#define SEND_IDLE 3

#define RESET_KEYA 'A'
#define RESET_KEYB 'B'
#define STOP_KEY 'D'
#define START_KEY 'C'
#define DP_KEY '*'
#define RELAY1_PIN 18
#define RELAY2_PIN 19

#define START_PIN 0
#define STOP_PIN 4
#define TRIGGER_LEVEL LOW

#define STOP_STATE 0
#define RUN_STATE 1
#define STOPPING_STATE 2
#define MAX_COUNT 100
#define CONFIRMED_COUNT 10
#define SD_CS 15

byte key = NO_KEY;
const byte ROWS = 4; // four rows
const byte COLS = 4; // three columns
char keys[ROWS][COLS] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
        };

byte rowPins[ROWS] = {36, 39, 34, 35}; // connect to the row pinouts of the keypad
byte colPins[COLS] = {32, 33, 25, 23}; // connect to the column pinouts of the keypad

Keypad my_keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

struct device_params
{
  float cur_weight = 12345.67;
  float expected_weight = 112233.44;
  String FullTime, monthStamp, dayStamp, timeStamp, event;
  int state_run_stop = STOP_STATE;
  int state_in_out = 1;
  uint8_t flag_send = 3;
} DeviceParams;
int num_dp = 2;
bool input_dp = false;
float input_a = 10;
float input_b = 1;
uint8_t mode_run = '\0';
float weight[3] = {1, 2, 3};
uint8_t count_get_weight = 0;
LiquidCrystal_I2C lcd(0x38, 16, 2);

enum BtnState
{
  BTN_STATE_00 = 0,
  BTN_STATE_01,
  BTN_STATE_10,
  BTN_STATE_11
};

BtnState btn_state = BTN_STATE_00;
int btn_count = 0;
// state_in_out = 1;
// int state_run_stop = STOP_STATE;
long last_force_stop = 0;

long last_show_cur_weight = 0;
long last_show_state = 0;
/********** Buffer using to read data from scale **********/
#define MAX_LEN 100
char msg[MAX_LEN];
int msg_len = 0;

/********** Display running mode **********/
void show_mode() {
  // lcd.setCursor(10,EXPECTED_SCREEN);
  lcd.setCursor(8, 2);
  switch (EEPROM.read(5)) {
  case 'A':
  {
    lcd.print("A-BOM VAO");
    break;
  }
  case 'B':
  {
    lcd.print("B-HUT RA ");
    break;
  }
  default:
  {
    lcd.print("CHUA CHON");
    break;
  }
  }
}

void reset_state_in() {
  DeviceParams.expected_weight = 0;
  input_dp = false;
  input_a = 10;
  input_b = 1;
  DeviceParams.state_in_out = 1;
}
/********** Reset status device when press key B **********/
void reset_state_out() {
  DeviceParams.expected_weight = 0;
  input_dp = false;
  input_a = 10;
  input_b = 1;
  DeviceParams.state_in_out = -1;
}
/********** Dispaly device status **********/

void show_state() {
  if (millis() - last_show_state < 2 * REFRESH_RATE_IN_MS) {
    return;
  }
  last_show_state = millis();

  lcd.setCursor(8, 3);
  if (DeviceParams.state_run_stop == STOP_STATE) {
    lcd.print("STOP");
  }
  if (DeviceParams.state_run_stop == RUN_STATE) {
    lcd.print("RUN ");
  }
}
/********** On Relay **********/
void onRelay(int x) {
  if (x == 1) {
    digitalWrite(RELAY1_PIN, TRIGGER_LEVEL);
  } else if (x == 2) {
    digitalWrite(RELAY2_PIN, HIGH - TRIGGER_LEVEL);
  }
}

/********** Convert float number to String and show **********/
void show_float(int addr, float x) {
  String s = String(x, num_dp);
  char cs[15];
  while (s.length() > 8) {
    s.remove(0, 1);
  }
  sprintf(cs, "%9s kg", s.c_str());
  lcd.setCursor(8, addr);
  lcd.print(cs);
  if (addr == CUR_SCREEN) {
    last_show_cur_weight = millis();
  }
}

void setup() {
  Serial.begin(9600);
}
/********** Get data and parse from Ampcells Scale Indicator **********/
bool update_cur_weight() {
  char c;
  bool new_msg = false;
  while (Serial.available()) {
    c = Serial.read();

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
/********** Check have press Start/Stop Button **********/
void update_btn_state() {
  // int v = analogRead(btn_pin);
  uint8_t v = digitalRead(START_PIN);
  v = v << 1;
  v += digitalRead(STOP_PIN);
  // StartPin=1 & StopPin=1
  if (v == 3) {
    if (btn_state != BTN_STATE_00) {
      btn_state = BTN_STATE_00;
      btn_count = 0;
    } else if (btn_count < MAX_COUNT) {
      btn_count += 1;
    }
  } else if (v == 2) {
    if (btn_state != BTN_STATE_01) {
      btn_state = BTN_STATE_01;
      btn_count = 0;
    } else if (btn_count < MAX_COUNT) {
      btn_count += 1;
    }
  } else if (v == 1) {
    if (btn_state != BTN_STATE_10) {
      btn_state = BTN_STATE_10;
      btn_count = 0;
    } else if (btn_count < MAX_COUNT) {
      btn_count += 1;
    }
  } else {
    if (btn_state != BTN_STATE_11) {
      btn_state = BTN_STATE_11;
      btn_count = 0;
    } else if (btn_count < MAX_COUNT) {
      btn_count += 1;
    }
  }
  // PrintBtnState();
}
/********** Turn off relay **********/
void offRelay(int x) {
  if (x == 1) {
    digitalWrite(RELAY1_PIN, HIGH - TRIGGER_LEVEL);
  } else if (x == 2) {
    digitalWrite(RELAY2_PIN, TRIGGER_LEVEL);
  }
}
/********** Stop device **********/
void force_stop() {
  offRelay(2);
  if (DeviceParams.state_run_stop == RUN_STATE) { // get_time();
    DeviceParams.flag_send = SEND_STOP;
    DeviceParams.state_run_stop = STOP_STATE;
    DeviceParams.cur_weight -= 0.4;
    if (1) {
      Serial.print("KL send in time stop task loop:");
      Serial.println(DeviceParams.cur_weight);
      // DeviceParams.flag_send = SEND_IDLE;
    } else {
      Serial.print("KL send in time stop task loop:");
      Serial.println(DeviceParams.cur_weight);
      Serial.println(DeviceParams.flag_send);
    }
  }
}
/********** Get START and STOP button **********/
char get_extra_keys() {
  if ((btn_state == BTN_STATE_10) && (btn_count > CONFIRMED_COUNT) && (btn_count <= MAX_COUNT + 1)) {
    btn_count = 2 * MAX_COUNT;
    return START_KEY;
  } else if ((btn_state == BTN_STATE_01) && (btn_count > CONFIRMED_COUNT) && (btn_count <= MAX_COUNT + 1)) {
    btn_count = 2 * MAX_COUNT;
    return STOP_KEY;
  } else
    return NO_KEY;
}
bool check_cond_stop() {
  if (DeviceParams.state_in_out > 0) { // bom vao
    if (DeviceParams.cur_weight >= DeviceParams.expected_weight) {
      return true;
    }
  } else { // hut ra
    if (DeviceParams.cur_weight <= DeviceParams.expected_weight + 0.4) {
      return true;
    }
  }

  // if ((btn_state == BTN_STATE_01) && (btn_count > CONFIRMED_COUNT))
  // {
  //   return true;
  // }
  return false;
}

/********** Run device **********/
void start_run() {
  if (check_cond_stop()) {
    return;
  }
  if (DeviceParams.state_run_stop == STOP_STATE) { // get_time();
    DeviceParams.flag_send = SEND_START;
    DeviceParams.state_run_stop = RUN_STATE;
    if (1) {
      Serial.print("KL send in time stop task loop:");
      Serial.println(DeviceParams.cur_weight);
    } else {
      Serial.print("KL send in time stop task loop:");
      Serial.println(DeviceParams.cur_weight);
      Serial.println(DeviceParams.flag_send);
    }
    if (DeviceParams.expected_weight != EEPROM.read(0)) {
      EEPROM.write(0, DeviceParams.expected_weight);
    }
  }

  show_state();
  onRelay(2);
}
/********** Get button **********/
bool update_key(char key) {
  if ((key == STOP_KEY) && (DeviceParams.state_run_stop == RUN_STATE)) {
    force_stop();
    return true;
  }
  if (key == RESET_KEYA) {
    force_stop();
    reset_state_in();
    EEPROM.write(5, RESET_KEYA);
    show_mode();
    return true;
  }
  if (key == RESET_KEYB) {
    force_stop();
    reset_state_out();
    EEPROM.write(5, RESET_KEYB);
    show_mode();
    return true;
  }

  if (DeviceParams.state_run_stop == RUN_STATE) {
    return false;
  }

  if (key == START_KEY) {
    start_run();
    return true;
  }

  if (key == DP_KEY) {
    input_dp = true;
    input_a = 1;
    input_b = 0.1;
    return true;
  }

  if (isDigit(key)) {
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
void loop() {

  if (update_cur_weight()) {
    if (millis() - last_show_cur_weight > 150) {
      show_float(CUR_SCREEN, DeviceParams.cur_weight);
    }
  }

  key = my_keypad.getKey();
  update_btn_state();

  if (key == NO_KEY) {
    key = get_extra_keys();
  }

  if (key != NO_KEY) {
    if (update_key(key)) {
      show_float(EXPECTED_SCREEN, DeviceParams.expected_weight);
    }
  }

  if (DeviceParams.state_run_stop == RUN_STATE) {
    if (check_cond_stop()) {
      force_stop();
    }
  }
  show_state();
}